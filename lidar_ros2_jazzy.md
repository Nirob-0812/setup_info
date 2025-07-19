
# 🧠 ROS 2 Jazzy + RP LIDAR A1M8 on Raspberry Pi 5 (Ubuntu 24.04 LTS)

This guide shows how to install **ROS 2 Jazzy**, configure the **RP LIDAR A1M8**, and visualize live laser scans in **RViz2**.

> ✅ Tested on Raspberry Pi 5 running **Ubuntu 24.04 LTS (64-bit)**  
> ✅ Works with **ROS 2 Jazzy Jalisco + sllidar_ros2**  
> ✅ Visualized using **RViz2**

---

## 📦 Step 1: Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt upgrade -y

# Install required tools
sudo apt install locales curl gnupg lsb-release git -y

# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key and source list
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy full desktop version
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions -y

# Source ROS 2 automatically
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🛠 Step 2: Create and Build Your ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🔁 Step 3: Clone and Build RP LIDAR Driver (`sllidar_ros2`)

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## ⚡ Step 4: Fix Serial Port Access

Ensure your user is in the correct group:
```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

Also allow permission to access the port:
```bash
sudo chmod 666 /dev/ttyUSB0
```

> You can replace `/dev/ttyUSB0` with `/dev/rplidar` if you use a udev rule.

---

## ▶️ Step 5: Launch the LIDAR with RViz2

```bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py serial_port:=/dev/ttyUSB0 frame_id:=laser use_sim_time:=false
```

> Replace `/dev/ttyUSB0` with your actual port if needed.

---

## 🧪 Step 6: View in RViz2 (if it doesn't auto-launch)

```bash
rviz2
```

In RViz2:
- Set **Fixed Frame**: `laser`
- Add **LaserScan** display
- Set **Topic** to `/scan`

---

## ✅ Check LIDAR Data

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic hz /scan
```

✅ You should see laser scan data printed at around `10 Hz`.

---

## 🧯 Troubleshooting

| Problem                         | Fix                                                                 |
|----------------------------------|----------------------------------------------------------------------|
| `ros2: command not found`       | Run `source /opt/ros/jazzy/setup.bash`                              |
| LIDAR shows once then freezes   | Add `use_sim_time:=false` in launch                                 |
| Error code `80008004`           | Wrong serial port or permission — check `/dev/ttyUSB*`              |
| Port changes (USB0 to USB1)     | Use a **powered USB hub** or udev rule to fix `/dev/rplidar`        |
| RViz shows no points            | Set correct **Fixed Frame**, add **LaserScan**, set topic to `/scan`|

---

## ✅ Done!

You now have your **RP LIDAR A1M8** running in **ROS 2 Jazzy** on **Ubuntu 24.04** with real-time data visible in **RViz2**.

> Fork this repo or use this script for your own robotics setup!

---

## 🧠 Credits

- [Slamtec RPLIDAR A1M8](https://www.slamtec.com/en/Lidar)
- [sllidar_ros2 GitHub](https://github.com/Slamtec/sllidar_ros2)
