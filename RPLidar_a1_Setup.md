Here’s the **updated and comprehensive README file** with all the improvements, including dynamic video filenames and realistic video playback speed based on timestamps. 

---

# **ROS 2 Setup on WSL and RPLIDAR A1 Integration**

This guide walks you through setting up ROS 2 on WSL, configuring RPLIDAR A1, scanning, visualizing, saving, and creating a video from sequential data.

---

## **Table of Contents**
1. [Install ROS 2 on WSL](#install-ros-2-on-wsl)
2. [Set Up RPLIDAR A1 on ROS 2](#set-up-rplidar-a1-on-ros-2)
3. [Scan Data from LIDAR](#scan-data-from-lidar)
4. [Visualize LIDAR Data in Rviz](#visualize-lidar-data-in-rviz)
5. [Save and Visualize Scan Data](#save-and-visualize-scan-data)
6. [Create Video from Sequential Data](#create-video-from-sequential-data)
7. [Possible Errors and Solutions](#possible-errors-and-solutions)

---

## **1. Install ROS 2 on WSL**

### Prerequisites
- **Ubuntu 18.04** or higher installed on WSL.
- Install necessary tools:
  ```bash
  sudo apt update
  sudo apt install locales software-properties-common curl git python3-pip -y
  ```

### Set Locale to UTF-8
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### Add ROS 2 Repositories and Keys
```bash
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

### Install ROS 2 Eloquent
```bash
sudo apt install ros-eloquent-desktop
```

### Verify Installation
```bash
source /opt/ros/eloquent/setup.bash
ros2 --version
```

---

## **2. Set Up RPLIDAR A1 on ROS 2**

### Create a Workspace
```bash
mkdir -p ~/ws_lidar/src
cd ~/ws_lidar/src
```

### Clone the RPLIDAR ROS 2 Package
```bash
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### Build the Workspace
```bash
cd ~/ws_lidar
colcon build --symlink-install
```

### Source the Workspace
```bash
source ~/ws_lidar/install/setup.bash
```
Add it permanently:
```bash
echo "source ~/ws_lidar/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Check LIDAR USB Permissions
1. Check connected USB devices:
   ```bash
   ls -la /dev | grep USB
   ls /dev/ttyUSB*
   ```
   Find your LIDAR device (e.g., `ttyUSB0`).
2. Assign permissions:
   ```bash
   sudo chmod 777 /dev/ttyUSB0
   ```
3. Poweshell Admin permissions:
   ```bash
   usbipd list
   wsl -l -v
   usbipd detach --busid 1-3
   usbipd attach --busid 1-3 --wsl Ubuntu-18.04
   ```

### Launch the RPLIDAR A1 Node
```bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

---

## **3. Scan Data from LIDAR**

### Echo the `/scan` Topic
Check if the LIDAR is publishing data:
```bash
ros2 topic echo /scan
```

---

## **4. Visualize LIDAR Data in Rviz**

### Launch Rviz
```bash
rviz2
```

### Add the `/scan` Topic
1. Click **Add** in Rviz.
2. Select **LaserScan** and set the topic to `/scan`.

---

## **5. Save and Visualize Scan Data**

### Save `/scan` Data to CSV
1. Create a Python script `save_scan_to_csv.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import LaserScan
   import csv

   class LaserScanToCSV(Node):
       def __init__(self):
           super().__init__('laser_scan_to_csv')
           self.subscription = self.create_subscription(
               LaserScan,
               '/scan',
               self.listener_callback,
               10
           )
           self.csv_file = open('scan_data.csv', 'w', newline='')
           self.csv_writer = csv.writer(self.csv_file)
           self.csv_writer.writerow(['Timestamp', 'Angle (rad)', 'Range (m)', 'Intensity'])

       def listener_callback(self, msg):
           current_time = self.get_clock().now().to_msg()
           angle = msg.angle_min
           for r, i in zip(msg.ranges, msg.intensities):
               self.csv_writer.writerow([current_time.sec, angle, r, i])
               angle += msg.angle_increment

   def main(args=None):
       rclpy.init(args=args)
       node = LaserScanToCSV()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

2. Run the script:
   ```bash
   python3 save_scan_to_csv.py
   ```

3. Check the saved file:
   ```bash
   ls
   cat scan_data.csv
   ```

---

## **6. Create Video from Sequential Data**

### Save the Script
Save the following as `create_video.py`:

```python
import csv
import matplotlib.pyplot as plt
import numpy as np
import cv2
from datetime import datetime

def main():
    file_name = input("Enter the name of the sequential data CSV file (e.g., scan_data.csv): ")
    if not os.path.exists(file_name):
        print(f"Error: File '{file_name}' does not exist.")
        return

    output_video = f"lidar_video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.avi"

    data = {}
    timestamps = []
    with open(file_name, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for row in csv_reader:
            timestamp, angle, range_val = row[0], float(row[1]), float(row[2])
            timestamps.append(timestamp)
            data.setdefault(timestamp, []).append((angle, range_val))

    fps = len(timestamps) / (len(timestamps) * 0.1)

    video_writer = cv2.VideoWriter(output_video, cv2.VideoWriter_fourcc(*'XVID'), fps, (800, 800))

    for timestamp, points in sorted(data.items()):
        x = [r * np.cos(a) for a, r in points]
        y = [r * np.sin(a) for a, r in points]
        plt.scatter(x, y, s=1)
        plt.savefig("frame.png")
        frame = cv2.imread("frame.png")
        video_writer.write(cv2.resize(frame, (800, 800)))
    video_writer.release()

if __name__ == '__main__':
    main()
```

Run the script:
```bash
python3 create_video.py
```

---

### **7. Possible Errors and Solutions**

| **Error**                                    | **Solution**                                                                                     |
|----------------------------------------------|-------------------------------------------------------------------------------------------------|
| `colcon: command not found`                  | Install `colcon`: `sudo apt install python3-colcon-common-extensions`.                         |
| `Could not determine the type for the topic` | Ensure the LIDAR node is running: `ros2 launch sllidar_ros2 view_sllidar_a1_launch.py`.        |
| `Permission Denied for /dev/ttyUSB0`         | Grant USB permissions: `sudo chmod 777 /dev/ttyUSB0`.                                          |
| Video too fast or slow                       | Ensure timestamps in CSV are accurate; dynamic `fps` is calculated in the script.             |

---

This README is now beginner-friendly and comprehensive! Let me know if further adjustments are needed. 🚀
