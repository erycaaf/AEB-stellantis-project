# AEB Gazebo Simulation

ROS 2 Humble + Gazebo Classic 11 simulation of an **Autonomous Emergency
Braking (AEB)** system, implementing the Euro NCAP Car-to-Car
(CCRs, CCRm, CCRb) test scenarios.

Designed to run on **Windows 11 + WSL2 (Ubuntu 22.04)**, but the same
package builds and runs on native Linux.

> **Note** — This README documents the standalone (non-Docker)
> developer flow. The supported entry point for the SIL stack is
> `cd sil && bash ./start_sil.sh` (or `docker compose up`); see
> [`../README.md`](../README.md) for the containerised quickstart.
> Use the steps below only when iterating on the ROS 2 nodes
> outside Docker.

---

## Architecture

```mermaid
graph LR
    subgraph Gazebo 11
        GZ[gzserver]
        EGO[ego_vehicle]
        TGT[target_vehicle]
    end

    GZ -->|odom| PERC[perception_node<br/><i>radar + lidar fusion</i>]
    PERC -->|distance, TTC,<br/>speed| AEB[aeb_node<br/><i>FSM + braking</i>]
    AEB -->|brake_cmd| SC[scenario_controller<br/><i>vehicle control</i>]
    SC -->|cmd_vel| GZ
    PERC -->|data| DASH[dashboard_node<br/><i>matplotlib (legacy)</i>]
    AEB -->|FSM state,<br/>alerts| DASH

    style GZ fill:#2196F3,color:#fff
    style PERC fill:#FF9800,color:#fff
    style AEB fill:#F44336,color:#fff
    style SC fill:#4CAF50,color:#fff
    style DASH fill:#9C27B0,color:#fff
```

### ROS 2 nodes

| Node | Description |
|------|-------------|
| `scenario_controller.py` | Drives both vehicles, reads brake commands from CAN, detects scenario end conditions. |
| `perception_node.py`     | Simulated radar (77 GHz, σ = 0.25 m) + lidar (σ = 0.05 m) with sensor fusion, plausibility check (FR-PER-006), and fault detection (FR-PER-007). |
| `aeb_node.py`            | Reference Python AEB controller (FSM-based: STANDBY → WARNING → BRAKE_L1/L2/L3 → POST_BRAKE) with TTC thresholds per UNECE R152. **Not used in the SIL stack** — there the FSM runs on the Zephyr ECU. Kept here for standalone testing. |
| `dashboard_node.py`      | **Deprecated.** Matplotlib panel: speedometer, TTC bar, FSM state, brake indicator, alerts. Superseded by the web dashboard at `http://localhost:3000` in the SIL stack. |

### Scenarios

| Scenario | Ego speed | Target speed | Initial gap | Description |
|----------|-----------|--------------|-------------|-------------|
| `ccrs_20`     | 20 km/h | 0 km/h  | 100 m | Car-to-Car Rear Stationary |
| `ccrs_30`     | 30 km/h | 0 km/h  | 100 m | |
| `ccrs_40`     | 40 km/h | 0 km/h  | 100 m | |
| `ccrs_50`     | 50 km/h | 0 km/h  | 100 m | |
| `ccrm`        | 50 km/h | 20 km/h |  50 m | Car-to-Car Rear Moving |
| `ccrb_d2_g12` | 50 km/h | 50 km/h |  12 m | Car-to-Car Rear Braking (decel = 2 m/s²) |
| `ccrb_d6_g12` | 50 km/h | 50 km/h |  12 m | Car-to-Car Rear Braking (decel = 6 m/s²) |
| `ccrb_d2_g40` | 50 km/h | 50 km/h |  40 m | Car-to-Car Rear Braking (decel = 2 m/s²) |
| `ccrb_d6_g40` | 50 km/h | 50 km/h |  40 m | Car-to-Car Rear Braking (decel = 6 m/s²) |

---

## Prerequisites

- **Windows 11** with WSL2 enabled (or any Linux host)
- **Ubuntu 22.04**
- **ROS 2 Humble** (Desktop install)
- **Gazebo Classic 11** (bundled with ROS 2 Humble Desktop)

---

## Installation

### 1. Install WSL2 and Ubuntu 22.04

In an **Administrator PowerShell**:

```powershell
wsl --install -d Ubuntu-22.04
```

Reboot if requested. Open **Ubuntu 22.04** from the Start menu and create
your user.

### 2. Install ROS 2 Humble

In the Ubuntu shell:

```bash
# Configure locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add the ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes Gazebo 11)
sudo apt update
sudo apt install -y ros-humble-desktop

# Extra dependencies
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-gazebo-ros-pkgs \
  python3-pip \
  python3-pyqt5

pip3 install matplotlib

# Source ROS 2 from your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Fetch the 3D vehicle models

```bash
mkdir -p ~/.gazebo/models
git clone --depth 1 https://github.com/osrf/gazebo_models.git /tmp/gazebo_models
cp -r /tmp/gazebo_models/hatchback_blue ~/.gazebo/models/
cp -r /tmp/gazebo_models/hatchback      ~/.gazebo/models/
cp -r /tmp/gazebo_models/suv            ~/.gazebo/models/
rm -rf /tmp/gazebo_models
```

### 4. Clone and build the package

```bash
mkdir -p ~/aeb_ws/src
cd ~/aeb_ws/src
git clone https://github.com/erycaaf/AEB-stellantis-project.git
ln -s AEB-stellantis-project/sil/aeb_gazebo aeb_gazebo

cd ~/aeb_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select aeb_gazebo
source install/setup.bash

# Make the Python nodes executable
chmod +x ~/aeb_ws/src/aeb_gazebo/src/*.py
```

---

## Running the simulation

### Quick launch (recommended)

```bash
cd ~/aeb_ws/src/aeb_gazebo
./run.sh ccrs_40
```

### Manual launch

```bash
source /opt/ros/humble/setup.bash
source ~/aeb_ws/install/setup.bash
ros2 launch aeb_gazebo aeb_with_dashboard.launch.py scenario:=ccrs_40
```

### Switch scenario

```bash
./run.sh ccrm           # Car-to-Car Rear Moving
./run.sh ccrb_d6_g40    # Car-to-Car Rear Braking
```

### What you should see

1. **Gazebo window** — 3D view of the road with the ego vehicle (blue
   hatchback) and the target vehicle (SUV).
2. **Dashboard window** — real-time matplotlib panel with speedometer,
   TTC, FSM state, brake indicator, and visual / audible alerts;
   includes a **RESTART** button. Note: this dashboard is the legacy
   path, deprecated in favour of the web dashboard (`:3000`) when the
   full SIL stack is running.

---

## ROS 2 topics

| Topic | Type | Description |
|-------|------|-------------|
| `/aeb/distance`        | `Float64` | Fused distance to target (m) |
| `/aeb/ego_speed`       | `Float64` | Ego vehicle speed (km/h) |
| `/aeb/target_speed`    | `Float64` | Target vehicle speed (km/h) |
| `/aeb/ttc`             | `Float64` | Time-to-collision (s) |
| `/aeb/brake_cmd`       | `Float64` | Brake command (0–100 %) |
| `/aeb/fsm_state`       | `String`  | FSM state (`STANDBY`, `WARNING`, `BRAKE_L1/L2/L3`, `POST_BRAKE`) |
| `/aeb/alert_visual`    | `Float64` | Visual alert active (0/1) |
| `/aeb/alert_audible`   | `Float64` | Audible alert active (0/1) |
| `/aeb/sensor_fault`    | `Float64` | Sensor fault flag (0/1) |
| `/aeb/scenario_status` | `String`  | Scenario result (`STOPPED` / `COLLISION`) |

---

## Troubleshooting

### Gazebo window does not appear

WSLg (the WSL GUI) is required. Check:

```bash
echo $DISPLAY
# Should print something like :0
```

If empty, update WSL2 from an Administrator PowerShell:

```powershell
wsl --update
```

### Gazebo is very slow

1. Close other heavy applications.
2. Restart WSL2 between runs:
   ```powershell
   wsl --shutdown
   ```
3. Make sure no `gzserver` processes are stuck:
   ```bash
   killall -9 gzserver gzclient 2>/dev/null
   ```

### Dashboard crashes (SIGSEGV)

Install the Qt5 backend for matplotlib:

```bash
sudo apt install -y python3-pyqt5
pip3 install PyQt5
```

### `python3\r: No such file or directory`

The Python files have Windows line endings. Fix them:

```bash
cd ~/aeb_ws/src/aeb_gazebo
find . -name "*.py" | xargs sed -i 's/\r$//'
```

### `colcon: command not found`

```bash
sudo apt install -y python3-colcon-common-extensions
```

---

## AEB FSM thresholds

| Transition | TTC threshold | Action |
|------------|---------------|--------|
| STANDBY → WARNING       | TTC < 4.0 s              | Visual alert ON                       |
| WARNING → BRAKE_L1      | TTC < 3.0 s (after 800 ms) | 20 % brake (2 m/s²)                |
| BRAKE_L1 → BRAKE_L2     | TTC < 2.2 s              | 40 % brake (4 m/s²)                   |
| BRAKE_L2 → BRAKE_L3     | TTC < 1.8 s              | 60 % brake (6 m/s²)                   |
| Any → POST_BRAKE        | v_ego < V_STOP_THRESHOLD | Hold 50 % for 2 s                     |
| POST_BRAKE → STANDBY    | After 2 s                 | Release brake                          |

Operating speed range: 10–60 km/h. Below 10 km/h while already braking,
the FSM stays in the active brake band and lets the distance-floor
logic carry the vehicle to a full stop (FR-DEC-005, FR-BRK-001).

---

## References

- UNECE Regulation No. 152 — Autonomous Emergency Braking systems
- Euro NCAP AEB Car-to-Car Test Protocol v4.3
- ISO 22839 — Forward collision mitigation systems
- ISO 26262 — Functional safety for road vehicles

---

## License

Part of an academic project for the post-graduate programme in
Automotive Electronic Engineering.
