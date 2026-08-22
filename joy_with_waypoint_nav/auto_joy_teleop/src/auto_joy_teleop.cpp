#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include "rclcpp/time_source.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std;

enum class GoalStatus
{
    HOME,
    X,
    Y,
    NONE
};

class AutoJoyTeleop : public rclcpp::Node
{
public:
    AutoJoyTeleop() : Node("auto_joy_teleop")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&AutoJoyTeleop::joy_callback, this, placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        rumble_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 10);
        pid_pub_ = this->create_publisher<std_msgs::msg::Int32>("pid/control", 10);
        nav_status_pub_  = this->create_publisher<std_msgs::msg::Int32>("robot/nav_status", 10);
        goal_status_pub_ = this->create_publisher<std_msgs::msg::String>("goal_status", 10);
        hill_hold_pub_ = this->create_publisher<std_msgs::msg::Bool>("hill_hold_control", 10);
        cancel_goal_client_ = this->create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");
        clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("/local_costmap/clear_entirely_local_costmap");
        rumble_timer_ = this->create_wall_timer(100ms, std::bind(&AutoJoyTeleop::rumble_callback, this)); // Lower period than 100ms won't lead to any significant effect

        // Subscribe to Nav2 navigate_to_pose action status to drive robot/nav_status
        nav_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status", 10,
            std::bind(&AutoJoyTeleop::nav_status_callback, this, placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<int>("axis_linear", 1);
        this->declare_parameter<int>("axis_angular", 0);
        this->declare_parameter<double>("scale_linear", 0.5);
        this->declare_parameter<double>("scale_angular", 1.0);
        this->declare_parameter<int>("deadman_axis", 2);
        this->declare_parameter<int>("enable_button", -1);
        this->declare_parameter<int>("enable_turbo_button", -1);
        this->declare_parameter<double>("scale_linear_turbo", 1.0);

        axis_linear_ = this->get_parameter("axis_linear").as_int();
        axis_angular_ = this->get_parameter("axis_angular").as_int();
        l_scale_ = this->get_parameter("scale_linear").as_double();
        a_scale_ = this->get_parameter("scale_angular").as_double();
        deadman_axis_ = this->get_parameter("deadman_axis").as_int();
        enable_button_ = this->get_parameter("enable_button").as_int();
        enable_turbo_button_ = this->get_parameter("enable_turbo_button").as_int();
        scale_linear_turbo_ = this->get_parameter("scale_linear_turbo").as_double();

        increment_ = 0.01;

        goal_status_ = GoalStatus::NONE;

        rumble_clear_costmap_ = 0;
        rumble_cancel_goal_ = 0;

        log_interval_ = 2000;

        home_.header.frame_id = "map";
        home_.header.stamp = this->now();
        home_.pose.position.x = 0.0;
        home_.pose.position.y = 0.0;
        home_.pose.position.z = 0.0;
        home_.pose.orientation.x = 0.0;
        home_.pose.orientation.y = 0.0;
        home_.pose.orientation.z = 0.0;
        home_.pose.orientation.w = 1.0;

        x_goal_set_ = false;
        y_goal_set_ = false;

        trigger_ = false;
        nav2_active_ = false;
        joy_active_last_ = false;

        rumble_.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
        rumble_.id = 0;
        last_cmd_vel_time_ = this->now();

        hill_hold_control_ = false;
        hill_hold_button_pressed_ = false;

        RCLCPP_INFO(this->get_logger(), "[NODE INITIATED]");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy &joy_msg)
    {
        if (joy_msg.axes.empty())
        {
            return;
        }

        // Adjust speed scale if D-pad inputs are present
        if (joy_msg.axes.size() > 7 && std::abs(joy_msg.axes[7]) > 0.5)
        {
            l_scale_ = std::max(0.05, std::min(2.0, l_scale_ + joy_msg.axes[7] * increment_));
        }
        if (joy_msg.axes.size() > 6 && std::abs(joy_msg.axes[6]) > 0.5)
        {
            a_scale_ = std::max(0.05, std::min(3.0, a_scale_ - joy_msg.axes[6] * increment_));
        }

        // Check if deadman key / trigger is active
        bool deadman_active = false;
        bool use_deadman = (deadman_axis_ >= 0 || enable_button_ >= 0);

        if (deadman_axis_ >= 0 && static_cast<size_t>(deadman_axis_) < joy_msg.axes.size())
        {
            if (joy_msg.axes[deadman_axis_] < 0.0)
            {
                deadman_active = true;
            }
        }
        if (enable_button_ >= 0 && static_cast<size_t>(enable_button_) < joy_msg.buttons.size())
        {
            if (joy_msg.buttons[enable_button_] != 0)
            {
                deadman_active = true;
            }
        }

        double cur_linear_scale = l_scale_;
        if (enable_turbo_button_ >= 0 && static_cast<size_t>(enable_turbo_button_) < joy_msg.buttons.size())
        {
            if (joy_msg.buttons[enable_turbo_button_])
            {
                cur_linear_scale = scale_linear_turbo_;
            }
        }

        auto robot_vel = geometry_msgs::msg::Twist();

        // When nav2 is actively navigating, do NOT publish any cmd_vel from joystick
        // This prevents any interference with the nav2 controller
        // User can still cancel the goal (button 1) to regain manual control
        if (!nav2_active_)
        {
            if (use_deadman)
            {
                if (deadman_active)
                {
                    // Trigger held: publish joystick velocity (same as old publishTwist)
                    if (static_cast<size_t>(axis_linear_) < joy_msg.axes.size())
                    {
                        robot_vel.linear.x = cur_linear_scale * joy_msg.axes[axis_linear_];
                    }
                    if (static_cast<size_t>(axis_angular_) < joy_msg.axes.size())
                    {
                        robot_vel.angular.z = a_scale_ * joy_msg.axes[axis_angular_];
                    }
                    cmd_vel_pub_->publish(robot_vel);
                }
                else
                {
                    // Trigger released: publish stop (same as old stopTwist)
                    cmd_vel_pub_->publish(robot_vel); // robot_vel is already all zeros
                }
            }
            else
            {
                if (static_cast<size_t>(axis_linear_) < joy_msg.axes.size())
                {
                    robot_vel.linear.x = cur_linear_scale * joy_msg.axes[axis_linear_];
                }
                if (static_cast<size_t>(axis_angular_) < joy_msg.axes.size())
                {
                    robot_vel.angular.z = a_scale_ * joy_msg.axes[axis_angular_];
                }
                cmd_vel_pub_->publish(robot_vel);
            }
        }

        if (joy_msg.buttons.size() > 0 && joy_msg.buttons[0] && goal_status_ != GoalStatus::HOME)
        {
            goal_pub_->publish(home_);
            goal_status_ = GoalStatus::HOME;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: HOME");
        }
        else if (joy_msg.buttons.size() > 1 && joy_msg.buttons[1])
        {
            auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();

            if (!cancel_goal_client_->service_is_ready())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Cancel Goal Service not available");
            }
            else
            {
                cancelled_goal_ = true;
                auto result = cancel_goal_client_->async_send_request(request);
                goal_status_ = GoalStatus::NONE;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Cancelling Current Goal");
            }
        }
        else if (joy_msg.buttons.size() > 2 && joy_msg.buttons[2] && goal_status_ != GoalStatus::X)
        {
            if (x_goal_set_)
            {
                goal_pub_->publish(x_goal_);
                goal_status_ = GoalStatus::X;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: X");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Store X goal before send goal");
            }
        }
        else if (joy_msg.buttons.size() > 3 && joy_msg.buttons[3] && goal_status_ != GoalStatus::Y)
        {
            if (y_goal_set_)
            {
                goal_pub_->publish(y_goal_);
                goal_status_ = GoalStatus::Y;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Setting Robot Goal: Y");
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Store Y goal before send goal");
            }
        }

        if (joy_msg.buttons.size() > 4 && joy_msg.buttons[4])
        {
            geometry_msgs::msg::TransformStamped t;
            try
            {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                x_goal_.header.frame_id = "map";
                x_goal_.header.stamp = this->now();
                x_goal_.pose.position.x = t.transform.translation.x;
                x_goal_.pose.position.y = t.transform.translation.y;
                x_goal_.pose.position.z = 0.0;
                x_goal_.pose.orientation.x = 0.0;
                x_goal_.pose.orientation.y = 0.0;
                x_goal_.pose.orientation.z = t.transform.rotation.z;
                x_goal_.pose.orientation.w = t.transform.rotation.w;

                x_goal_set_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Storing X goal");
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Could not transform: %s", ex.what());
            }
        }
        else if (joy_msg.buttons.size() > 5 && joy_msg.buttons[5])
        {
            geometry_msgs::msg::TransformStamped t;
            try
            {
                t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

                y_goal_.header.frame_id = "map";
                y_goal_.header.stamp = this->now();
                y_goal_.pose.position.x = t.transform.translation.x;
                y_goal_.pose.position.y = t.transform.translation.y;
                y_goal_.pose.position.z = 0.0;
                y_goal_.pose.orientation.x = 0.0;
                y_goal_.pose.orientation.y = 0.0;
                y_goal_.pose.orientation.z = t.transform.rotation.z;
                y_goal_.pose.orientation.w = t.transform.rotation.w;

                y_goal_set_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Storing Y goal");
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Could not transform: %s", ex.what());
            }
        }
        if (joy_msg.buttons.size() > 8 && joy_msg.buttons[8])
        {
            auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
            if (!clear_costmap_client_->service_is_ready())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Clear Costmap Service not available");
            }
            else
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Clearing Costmap");
                auto result = clear_costmap_client_->async_send_request(request);
                rumble_clear_costmap_ = 5; // For 0.5 seconds rumble
            }
        }
        if (joy_msg.buttons.size() > 9 && joy_msg.buttons[9])
        {
            if (!hill_hold_button_pressed_)
            {
                hill_hold_control_ = !hill_hold_control_;
                auto hill_hold_msg = std_msgs::msg::Bool();
                hill_hold_msg.data = hill_hold_control_;
                hill_hold_pub_->publish(hill_hold_msg);
                hill_hold_button_pressed_ = true;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), log_interval_, "Hill Hold Control: %s", hill_hold_control_ ? "ON" : "OFF");
            }
        }
        else
        {
            hill_hold_button_pressed_ = false;
        }
    }

    /**
     * Translates Nav2 action GoalStatus codes into a simple integer published
     * on /robot/nav_status:
     *   0 = IDLE        (no active goal)
     *   1 = NAVIGATING  (ACCEPTED or EXECUTING)
     *   2 = SUCCEEDED
     *   3 = CANCELED
     *   4 = ABORTED / FAILED
     *
     * Nav2 status codes (from action_msgs/GoalStatus):
     *   STATUS_UNKNOWN   = 0
     *   STATUS_ACCEPTED  = 1
     *   STATUS_EXECUTING = 2
     *   STATUS_CANCELING = 3
     *   STATUS_SUCCEEDED = 4
     *   STATUS_CANCELED  = 5
     *   STATUS_ABORTED   = 6
     */
    void nav_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
    {
        // Default: idle
        int nav_status = 0;

        // Walk all goals; most recent (last) one wins
        for (const auto & goal_info : msg->status_list)
        {
            switch (goal_info.status)
            {
                case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
                case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                case action_msgs::msg::GoalStatus::STATUS_CANCELING:
                    nav_status = 1;  // actively navigating
                    break;
                case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                    nav_status = 2;
                    break;
                case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                    nav_status = 3;
                    break;
                case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                    nav_status = 4;
                    break;
                default:
                    nav_status = 0;
                    break;
            }
        }

        // Update nav2_active_ flag so joy_callback knows whether to publish cmd_vel
        nav2_active_ = (nav_status == 1);

        auto status_msg = std_msgs::msg::Int32();
        status_msg.data = nav_status;
        nav_status_pub_->publish(status_msg);

        // Labels shared by both publishers
        static const char* labels[] = {"IDLE", "NAVIGATING", "SUCCEEDED", "CANCELED", "ABORTED"};
        const char* label = (nav_status >= 0 && nav_status <= 4) ? labels[nav_status] : "UNKNOWN";

        // Edge-trigger: only publish /goal_status and log on actual state change
        static int last_status = -1;
        if (nav_status != last_status)
        {
            RCLCPP_INFO(this->get_logger(), "Nav status: %s (%d)", label, nav_status);

            // Publish human-readable status string on /goal_status
            auto gs_msg = std_msgs::msg::String();
            gs_msg.data = label;
            goal_status_pub_->publish(gs_msg);

            last_status = nav_status;
        }
    }

    void rumble_callback()
    {
        if (rumble_clear_costmap_ > 0)
        {
            rumble_clear_costmap_--;
            rumble_.intensity = 1.0;
        }
        else if (rumble_cancel_goal_ > 0)
        {
            rumble_cancel_goal_--;
            rumble_.intensity = 1.0;
        }
        else
        {
            rumble_.intensity = 0.0;
        }

        if (rumble_.intensity != 0.0)
        {
            rumble_pub_->publish(rumble_);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr nav_status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr rumble_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pid_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr nav_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hill_hold_pub_;

    rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_client_;
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client_;

    rclcpp::TimerBase::SharedPtr rumble_timer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    geometry_msgs::msg::PoseStamped home_;
    geometry_msgs::msg::PoseStamped x_goal_;
    geometry_msgs::msg::PoseStamped y_goal_;

    sensor_msgs::msg::JoyFeedback rumble_;

    double x_vel_;
    double y_vel_;
    double z_vel_;
    double a_scale_;
    double l_scale_;
    double increment_;

    bool x_goal_set_;
    bool y_goal_set_;
    bool trigger_;
    bool cancelled_goal_;
    bool nav2_active_;  // true when nav2 is actively executing a goal
    bool joy_active_last_{false}; // tracks if teleop was actively publishing
    bool hill_hold_control_;
    bool hill_hold_button_pressed_;

    GoalStatus goal_status_;

    int log_interval_;
    int rumble_clear_costmap_;
    int rumble_cancel_goal_;

    int axis_linear_{1};
    int axis_angular_{0};
    int deadman_axis_{2};
    int enable_button_{-1};
    int enable_turbo_button_{-1};
    double scale_linear_turbo_{1.0};

    rclcpp::Time last_cmd_vel_time_{0, 0, RCL_ROS_TIME};
    const double THROTTLE_RATE = 0.05;

    // Joystick deadzone to filter out stick drift noise
    static constexpr double DEADZONE = 0.05;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoJoyTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
