# Software

This folder stores the Domabot ROS2 packages.

## Requirements

Base requirements:
- OS Ubuntu + ROS2

Tested on Ubuntu 22.04 with ROS2 Humble

## Install

### OS & system packages:
1. Open `Software Updater` app, click `Settings` button, on tab `Update` choose in dropdown list `Subscribed to` option `Security and recommended update`.
2. Click `Close` button, than `Update`.
3. Open terminal and input next command:
```bash
sudo apt update && sudo apt upgrade && sudo apt dist-upgrade
```

### ROS2

Chose ROS2 distro on [docs page](https://docs.ros.org/).
And perform install steps.

For development & tests was used [ROS2 humble](https://docs.ros.org/en/humble/).

Also install next ROS2 packages:
```bash
ros-<ROS_DISTRO>-usb-cam
```

Prepare your ros2 workspace directory by ros documentation.

### Dev packages

#### Modbus library

```bash
sudo apt install libmodbus5 libmodbus-dev
```

#### Intel Realsens packages

1. Install librealsense by [manual](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages).

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

If `librealsense2-dkms` produces install error, perform:
```bash
sudo apt-get install gcc-12
```

2. Instal ROS wrapper for Intel Realsense by [manual](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu).
```bash
sudo apt install ros-<ROS_DISTRO>-librealsense2*
```

### Domabot packages

1. Link Domabot packages with your ros workspace:

```bash
cd </path/to/your/ros2_ws>
cd src
ln -s <path_to>/domabot/software/. .
cd ..
```

2. Run default build command:

```bash
colcon build --symlink-install
```
