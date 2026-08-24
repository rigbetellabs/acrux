# Acrux ROS 2 Jazzy Release

![acrux Logo](img/logo.png)

## Table of Contents
- [Acrux ROS 2 Jazzy Release](#acrux-ros-2-jazzy-release)
  - [Table of Contents](#table-of-contents)
  - [1. Installation](#1-installation)
  - [2. Connection](#2-connection)
    - [Initial Wifi Setup](#initial-wifi-setup)
      - [1. Create a mobile hotspot](#1-create-a-mobile-hotspot)
      - [2. Start the robot](#2-start-the-robot)
      - [3. SSH into the robot](#3-ssh-into-the-robot)
      - [4. Connect to Wifi](#4-connect-to-wifi)
      - [5. SSH using your Wifi](#5-ssh-using-your-wifi)
  - [3. NUC Instructions](#3-nuc-instructions)
    - [Instructions to remove Intel NUC from robot](#instructions-to-remove-intel-nuc-from-robot)
    - [USB ports Configuration](#usb-ports-configuration)
  - [4. Package Description](#4-package-description)
    - [4.1 acrux\_bringup](#41-acrux_bringup)
    - [4.2 acrux\_description](#42-acrux_description)
    - [4.3 acrux\_firmware](#43-acrux_firmware)
    - [4.4 acrux\_gazebo](#44-acrux_gazebo)
    - [4.5 acrux\_navigation](#45-acrux_navigation)
    - [4.6 acrux\_slam](#46-acrux_slam)
  - [5. Launch Sequence](#5-launch-sequence)
    - [5.1 Simulation (Gazebo Harmonic)](#51-simulation-gazebo-harmonic)
    - [5.2 Real Robot Operation](#52-real-robot-operation)
    - [5.3 Launch Arguments Reference](#53-launch-arguments-reference)
    - [5.4 SLAM & Mapping Modes](#54-slam--mapping-modes)
    - [5.5 Saving Maps](#55-saving-maps)
    - [5.6 Localization & Navigation Modes](#56-localization--navigation-modes)
  - [6. Low-Level ROS Topics](#6-low-level-ros-topics)
    - [`/battery/percentage`](#batterypercentage)
    - [`/battery/voltage`](#batteryvoltage)
    - [`/cmd_vel`](#cmd_vel)
    - [`/pid/control`](#pidcontrol)
    - [`/diagnostics/test`](#diagnosticstest)
    - [`/wheel/ticks`](#wheelticks)
    - [`/wheel/vel`](#wheelvel)
  - [7. Acrux Robot Parameters](#7-acrux-robot-parameters)
  - [8. Joystick Control Instructions](#8-joystick-control-instructions)
  - [9. LED Indicators Instructions](#9-led-indicators-instructions)

<div style="page-break-after: always;"></div>

## 1. Installation

### Prerequisites
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2**: ROS 2 Jazzy Jalisco
- **Simulator**: Gazebo Harmonic (`gz-sim8`)

### Clone and Setup
```bash
mkdir -p ~/acrux_ws/src
cd ~/acrux_ws/src
git clone -b ros2-jazzy https://github.com/rigbetellabs/acrux.git
```

### Install Dependencies
Run the automated installation script:
```bash
cd ~/acrux_ws/src/acrux
chmod +x install.sh
./install.sh
```

Or install dependencies manually via `apt`:
```bash
cd ~/acrux_ws/src/acrux
cat requirements.txt | xargs sudo apt-get install -y
```

Install Gazebo Harmonic:
```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```
> [!IMPORTANT]
> For the YDLidar package to work on hardware, install the YDLidar-SDK:
> ```bash
> git clone https://github.com/YDLIDAR/YDLidar-SDK.git
> sudo apt install -y cmake pkg-config python3-pip swig
> mkdir -p YDLidar-SDK/build && cd YDLidar-SDK/build
> cmake .. && make -j$(nproc)
> sudo make install
> sudo ldconfig
> cd .. && pip install .
> ```

### Build the Workspace
```bash
cd ~/acrux_ws
colcon build --symlink-install
source install/setup.bash
```

<div style="page-break-after: always;"></div>

## 2. Connection

### Initial Wifi Setup

> [!NOTE]
> By default, the robot starts up automatically upon bootup, running ROS 2 locally without needing an external Wi-Fi network.

Follow these steps to connect the robot to your Wi-Fi:

#### 1. Create a mobile hotspot
Initiate a hotspot from your smartphone or laptop with these credentials:
- **Hotspot Name**: `admin`
- **Hotspot Password**: `adminadmin`

<p align="center">
<img src="img/mobilehotspot.jpeg" width="250"/>
</p>

#### 2. Start the robot
Power on the robot and wait until it connects to your hotspot network:

| On powering on: | When connected to hotspot: | 
|---|---|
| ![Step1](img/booting.gif) | ![Step2](img/admin.gif) |

#### 3. SSH into the robot
- Connect your laptop to the same hotspot network.
- Open a terminal and SSH into the robot:
```bash
ssh "your-robot-name"@"your-robot-ip"
```
> [!TIP]
> The robot username and password are provided upon deployment and labeled on the internal computer. The robot IP address is shown on the robot's onboard display once connected.

#### 4. Connect to Wifi
- List available Wi-Fi networks:
```bash
sudo nmcli dev wifi list --rescan yes
```
- Connect to your Wi-Fi network:
```bash
sudo nmcli device wifi connect "your-wifi-name" password "your-wifi-password"
```

> [!IMPORTANT]
> This command will switch network interfaces and close the current SSH session. Wait ~30 seconds for the robot to connect to your Wi-Fi; the new IP will appear on the robot display.

#### 5. SSH using your Wifi
Connect your PC to the same Wi-Fi and SSH into the robot using its newly assigned IP.

<div style="page-break-after: always;"></div>

## 3. NUC Instructions
### Instructions to remove Intel NUC from robot

| Step 1 | Step 2 | Step 3 |
|---|---|---|
| ![Step1](img/step1.gif) | ![Step2](img/step2.gif) | ![Step3](img/step3.gif) |

*(Video walkthrough available [here](https://youtu.be/-I9eqPhfBqA?si=ZTHeQBfnzq4X63mW))*

### USB Ports Configuration
> [!IMPORTANT]
> Connect the onboard hardware USB devices as illustrated below:

![USB Port Connections](img/port_connections.png)

<div style="page-break-after: always;"></div>

## 4. Package Description

### 4.1 acrux_bringup
| Launch File | Description |
|---|---|
| `autobringup.launch.py` | Primary master launch file for the entire autonomous stack. Supports simulation and real hardware, Cartographer SLAM, SLAM Toolbox, AMCL localization, and Nav2 navigation. |
| `bringup.launch.py` | Hardware bringup file that starts MicroROS agent, LiDAR drivers, and joystick control without navigation. |

### 4.2 acrux_description
| Launch File | Description |
|---|---|
| `display.launch.py` | Loads URDF/Xacro, launches Gazebo Harmonic and RViz2 visualization. |
| `rviz.launch.py` | Launches RViz2 with the configured Acrux visualization profiles. |
| `state_publisher.launch.py` | Starts `robot_state_publisher` for TF broadcasts from URDF. |

### 4.3 acrux_firmware
| Launch File / Script | Description |
|---|---|
| `auto_joy_teleop.launch.py` | Joystick driver and waypoint teleoperation node. |
| `network_status_publisher_node` | Broadcasts network connectivity telemetry to onboard displays. |
| `x2_params.yaml` | Hardware configuration parameters for YDLidar sensors. |

### 4.4 acrux_gazebo
| Launch File / Config | Description |
|---|---|
| `gazebo.launch.py` | Launches Gazebo Harmonic simulation environment and spawns the Acrux robot model with ROS-Gazebo bridge. |
| `ros_gz_bridge.yaml` | ROS 2 Jazzy $\leftrightarrow$ Gazebo Harmonic topic bridge configuration (`/clock`, `/scan`, `/cmd_vel`, `/joint_states`). |
| `nav2_test_world.sdf` | Default simulated evaluation environment. |

### 4.5 acrux_navigation
| Launch File / Config | Description |
|---|---|
| `navigation.launch.py` | Launches Nav2 navigation stack (SmacPlanner, DWB controller, behavior server, costmaps). |
| `nav2_params.yaml` | Nav2 configuration for real hardware (default, subscribes to `/scan_filtered`, `use_sim_time: False`). |
| `nav2_params_sim.yaml` | Nav2 configuration for simulation (subscribes to `/scan`, `use_sim_time: True`). |
| `map_saver.launch.py` | Utility to save generated occupancy grid maps to disk. |

### 4.6 acrux_slam
| Launch File / Config | Description |
|---|---|
| `cartographer.launch.py` | Launches Google Cartographer for 2D SLAM and pure odometry estimation (dynamically uses `/scan_filtered` when `use_sim_time:=False`). |
| `slam_toolbox.launch.py` | Launches SLAM Toolbox for online async SLAM and graph-based lifelong localization. |
| `slam_toolbox_params.yaml` | SLAM Toolbox configuration for real hardware (default, `/scan_filtered`). |
| `slam_toolbox_params_sim.yaml` | SLAM Toolbox configuration for simulation (`/scan`). |

<div style="page-break-after: always;"></div>

## 5. Launch Sequence

### 5.1 Simulation (Gazebo Harmonic)

#### Full Autonomous Stack (Gazebo + RViz2 + Nav2 + Cartographer SLAM):
```bash
ros2 launch acrux_bringup autobringup.launch.py use_sim_time:=True exploration:=True toolbox:=False
```

### 5.2 Real Robot Operation

#### Full Autonomous Stack on Physical Robot:
```bash
ros2 launch acrux_bringup autobringup.launch.py exploration:=True
```

#### Hardware & Sensors Only (No Navigation):
```bash
ros2 launch acrux_bringup bringup.launch.py joy:=True
```

### 5.3 Launch Arguments Reference

| Argument | Description | Default |
|---|---|---|
| `use_sim_time` | Set `True` for Gazebo simulation clock, `False` for real robot hardware. | `False` |
| `exploration` | Set `True` for SLAM mapping mode, `False` for map-based localization. | `True` |
| `slam` | Set `True` for **pure Cartographer SLAM only** (no Nav2 navigation nodes). | `False` |
| `toolbox` | Set `True` to use **SLAM Toolbox** for mapping/localization. `False` uses Cartographer / AMCL. | `False` |
| `map_file` | Path to a `.yaml` map file for localization mode (`exploration:=False`). | `nav2_test_map.yaml` |
| `joy` | Enable joystick teleoperation and waypoint navigation. | `True` |

---

### 5.4 SLAM & Mapping Modes

#### 1. Full Cartographer SLAM + Nav2 Navigation:
```bash
# Simulation:
ros2 launch acrux_bringup autobringup.launch.py use_sim_time:=True exploration:=True toolbox:=False

# Real Robot:
ros2 launch acrux_bringup autobringup.launch.py exploration:=True toolbox:=False
```

#### 2. Pure Cartographer SLAM (No Nav2 / Manual Mapping):
```bash
ros2 launch acrux_bringup autobringup.launch.py slam:=True
```

#### 3. SLAM Toolbox Online Mapping + Nav2:
```bash
# Simulation:
ros2 launch acrux_bringup autobringup.launch.py use_sim_time:=True exploration:=True toolbox:=True

# Real Robot:
ros2 launch acrux_bringup autobringup.launch.py exploration:=True toolbox:=True
```

---

### 5.5 Saving Maps

To save an active SLAM map:
```bash
# Standard 2D Occupancy Grid (.yaml + .pgm):
ros2 launch acrux_navigation map_saver.launch.py map_file_path:=~/acrux_ws/src/acrux_navigation/maps/my_map

# Or using nav2_map_server CLI:
ros2 run nav2_map_server map_saver_cli -f ~/acrux_ws/src/acrux_navigation/maps/my_map
```

---

### 5.6 Localization & Navigation Modes

#### 1. AMCL Localization on Saved 2D Map (`.yaml` / `.pgm`):
```bash
# Simulation:
ros2 launch acrux_bringup autobringup.launch.py \
  use_sim_time:=True \
  exploration:=False \
  toolbox:=False \
  map_file:=$(ros2 pkg prefix --share acrux_navigation)/maps/nav2_test_map.yaml

# Real Robot:
ros2 launch acrux_bringup autobringup.launch.py \
  exploration:=False \
  toolbox:=False \
  map_file:=/path/to/your/map.yaml
```

#### 2. SLAM Toolbox Graph Localization (`.posegraph` + `.data`):
```bash
ros2 launch acrux_bringup autobringup.launch.py \
  use_sim_time:=True \
  exploration:=False \
  toolbox:=True \
  map_file:=$(ros2 pkg prefix --share acrux_navigation)/maps/my_room.yaml
```

<div style="page-break-after: always;"></div>

## 6. Low-Level ROS Topics

#### `/battery/percentage`
Reports remaining battery percentage ($0\% - 100\%$).
- $> 20\%$: Normal operation.
- $15\% - 20\%$: Periodic alert beep (every 2 min).
- $< 10\%$: Critical alert beep.
> [!CAUTION]
> Do not discharge the battery below `10%` to prevent permanent cell damage.

#### `/battery/voltage`
Battery pack voltage (ranging from $25.2\text{ V}$ full to $19.8\text{ V}$ empty).

#### `/cmd_vel`
Target velocity input (`geometry_msgs/msg/Twist`) received by the motor controller.

#### `/pid/control`
Integer topic (`std_msgs/msg/Int32`) controlling low-level PID modes:
- `0`: Stop PID
- `1`: Fast response PID
- `2`: Smooth PID
- `3`: Super-smooth PID

#### `/wheel/ticks`
Raw encoder ticks array `[lf, lb, rf, rb]` for all four wheels.

#### `/wheel/vel`
Computed wheel velocities `[lf, lb, rf, rb]` from encoders.

<div style="page-break-after: always;"></div>

## 7. Acrux Robot Parameters

| Parameter | Specification |
|---|---|
| **Drive Type** | Differential Drive |
| **Wheel Diameter** | $0.1\text{ m}$ |
| **Track Width (Separation)** | $0.5\text{ m}$ |
| **Motors** | Planetary DC Geared Motors |
| **Rated RPM** | $100\text{ RPM}$ |
| **Encoder Type** | Magnetic Encoder ($498\text{ PPR}$) |
| **Microcontroller** | DOIT ESP32 DevKit V1 |
| **Onboard Computer** | Intel NUC |
| **Payload Capacity** | $100\text{ kg}$ |
| **Battery Life** | $\approx 3\text{ hours}$ |
| **Battery Type** | 6S Li-ion ($22.2\text{ V}$) |

<div style="page-break-after: always;"></div>

## 8. Joystick Control Instructions
![autojoy](img/autojoyteleop.png)

<div style="page-break-after: always;"></div>

## 9. LED Indicators Instructions

### Nomenclature
![LED Nomenclature](img/led-instruction.png)

---

### Status Patterns

| Indication Type | Meaning |
|---|---|
| **Orange Fading Effect** | ROS 2 not connected / initializing |
| **Blue Sidelights, White Headlights, Red Brakelights** | ROS 2 Connected & Ready |
| **Yellow Status Lights + Beep** | Navigating towards goal |
| **Green Status Lights Flashing $3\times$ + Buzzer** | Goal reached successfully |
| **Purple Status Lights + Beep** | Waypoint / Goal location stored |
| **Orange Status Lights** | Clearing Costmaps |
| **Orange Blinking Indicator Lights** | Turn signal (direction of travel) |
| **Red Status Lights** | Goal canceled / Mission aborted |
| **All Red Flashing** | Emergency Stop activated |
