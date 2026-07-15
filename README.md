# Density-Based Safe Motion Planning for Social Navigation

Density-field obstacle avoidance on a Unitree Go1 using a ZED2i camera, Jetson AGX Xavier, ROS Noetic, and high-level Unitree velocity commands.

<p align="center">
  <video src="docs/assets/videos/11-48_baseline.mp4" controls width="720"></video>
</p>

## Overview

This repository contains the robot-side experiment code, density-based navigation utilities, data-sync scripts, and plotting tools used for social-navigation experiments with moving human obstacles. The current experiment stack subscribes to 3D detections from the ZED/Jetson perception pipeline, transforms detections into world-frame obstacles, computes a density-field control direction, and publishes Unitree `HighCmd` messages to `/high_cmd` through the `unitree_ros_to_real` interface.

## Hardware

- Robot: Unitree Go1
- Camera: ZED2i
- Onboard computer: Jetson AGX Xavier running JetPack 5.1.2 / L4T R35.4.1
- Host PC: ROS Noetic workstation
- Power: [MAXOAK K2 185Wh / 50000mAh external battery pack](https://maxoak.net/products/maxoak-k2-185wh-50000mah-power-bank), using the 20 V output to power the Jetson AGX Xavier
- Mounting: custom 3D-printed camera/compute hardware; saved Bambu print files are under `hardware/safe_gcode/`

## Software Stack

- ROS Noetic
- Python 3
- JetPack 5.1.2-b104 / Jetson Linux L4T R35.4.1
- ZED SDK 5.1.2
- `autoware_msgs/DetectedObjectArray` detections on `/detections`
- `geometry_msgs/PoseStamped` camera/robot pose on `/camera_pose`
- `unitree_legged_msgs/HighCmd` velocity commands on `/high_cmd`

## Repository Layout

```text
ros_nodes/        ROS experiment and helper nodes
scripts/          bag sync, conversion, and analysis tools
density_utils/    density-field controller and dynamics utilities
docs/             GitHub Pages website and setup documentation
docker/           Jetson container run scripts and environment notes
hardware/         saved G-code print files and hardware notes
analysis/         notes for plots and metrics
trial_data/       local data staging; raw bags are ignored by Git
```


## Reproducing the Jetson Docker Environment

The Jetson camera/perception container used in the lab is named:

```text
object_detection:latest
```

It runs on the Jetson AGX Xavier with JetPack 5.1.2 / L4T R35.4.1 and ZED SDK 5.1.2. The maintained rebuild recipe is in `docker/Dockerfile`, but the most faithful recovery path is the saved Docker image tar.

### Option 1: Copy And Load The Saved Image

Keep the private image tar out of Git. On the host PC, place it at:

```text
docker/image_backup/object_detection_jetson_agx_xavier_jp512_zed512.tar
```

Copy it to the Jetson:

```bash
rsync -avh --progress \
  docker/image_backup/object_detection_jetson_agx_xavier_jp512_zed512.tar \
  lab@192.168.12.202:/home/lab/object_Detection_shared/
```

SSH into the Jetson and load it:

```bash
ssh lab@192.168.12.202
sudo docker load -i /home/lab/object_Detection_shared/object_detection_jetson_agx_xavier_jp512_zed512.tar
sudo docker images | grep object_detection
```

### Option 2: Rebuild On The Jetson

Clone this repo on the Jetson and build from the checked-in Dockerfile:

```bash
git clone git@github.com:sriram-2502/go1_density_social_nav.git
cd go1_density_social_nav
make docker-build
```

The rebuild starts from:

```dockerfile
FROM stereolabs/zed:5.1.2-tools-devel-jetson-jp5.1.2
```

and adds the ROS Noetic Python/runtime dependencies, `autoware_ai_messages`, and the copied Jetson camera nodes in `jetson_nodes/`. This should reproduce the functional camera/perception environment, but it is not expected to be byte-for-byte identical to the saved lab image until validated on the Jetson.

The main camera node uses the ZED SDK Python API for object detection (`pyzed.sl.ObjectDetectionParameters`, `enable_object_detection`, and `retrieve_objects`), publishes detections as `autoware_msgs/DetectedObjectArray` on `/detections`, publishes camera pose on `/camera_pose`, publishes image frames on `/video`, and records timestamped video/pose bags under `/home/bags` inside the container. Because `/home` is mounted to `/home/lab/object_Detection_shared` by the Docker run command, those bags appear on the Jetson host in the shared folder.

## Install And Build On Host PC

These instructions follow the original `zed2_object_tracking` host/Jetson workflow, adapted for this density-controller repo.

### 1. Install ROS Noetic And Tools

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  python3-rosdep \
  python3-catkin-tools \
  git \
  ros-noetic-vision-msgs

sudo rosdep init || true
rosdep update

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
```

The density controller also needs `autoware_msgs` and `unitree_legged_msgs` available in your ROS environment. In the lab setup these come from the Autoware message workspace and the Unitree real-robot workspace used by `unitree_ros_to_real` / `go1_wifi_ws`.

### ROS Detection Message Types

Use the Autoware message path for the current hardware pipeline:

- Current Jetson camera node: `jetson_nodes/camera_node.py`
- Current detection topic: `/detections`
- Current message type: `autoware_msgs/DetectedObjectArray`
- Current controller: `ros_nodes/motion_plan_exp.py`
- Current RViz helper: `ros_nodes/markerFrom3d.py`

Some older helper scripts still use `vision_msgs/Detection3DArray` (`jetson_nodes/object_detection.py`, `ros_nodes/object_subscriber.py`, and `ros_nodes/motion_plan.py`). Those are legacy/baseline utilities from the earlier ZED object-tracking workflow. They are useful for reference, but they are not the main Go1 density social-navigation pipeline.

### 2. Clone This Repo

```bash
git clone git@github.com:sriram-2502/go1_density_social_nav.git
cd go1_density_social_nav
```

If your Unitree bridge workspace is separate, build/source it as usual:

```bash
cd ~/go1_wifi_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

Optional, if this is the workspace you always use for the Go1:

```bash
echo "source ~/go1_wifi_ws/devel/setup.bash" >> ~/.bashrc
```

## Run The Experiment

### 0. Put The Robot In Standing Mode

The Go1 should be in stand mode before testing commands. As a sanity check, confirm the robot can move with the joystick before sending ROS high-level commands.

### 1. Connect The Host PC To The Go1 Wi-Fi

On Ubuntu, select the Unitree/Go1 Wi-Fi hotspot.

```text
Password: 00000000
```

Verify that the host received a `192.168.12.x` address:

```bash
ip a | grep -A2 -E "wlx|wlan|wlp"
```

Ping the hotspot gateway:

```bash
ping -c 2 192.168.12.1
```

If the ping succeeds, the Wi-Fi link is good.

### 2. Add The Route To The Robot Internal Network

Find the Wi-Fi interface name:

```bash
ip link | grep -E "wlx|wlan|wlp"
```

Add the route, replacing `<WIFI_IFACE>` with your interface:

```bash
sudo ip route add 192.168.123.0/24 via 192.168.12.1 dev <WIFI_IFACE>
```

Example:

```bash
sudo ip route add 192.168.123.0/24 via 192.168.12.1 dev wlp2s0
```

Verify the route:

```bash
ip route | grep 192.168.123
```

Ping the robot control PC:

```bash
ping -c 2 192.168.123.161
```

If adding the route returns `File exists`, the route is already set.

### 3. Set ROS Environment Variables On The Host PC

In every host terminal used for ROS:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<YOUR_HOST_WIFI_IP>
```

Find the host Wi-Fi IP:

```bash
ip -4 addr show <WIFI_IFACE> | grep inet
```

Example for the OptiPlex host:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.12.162
```

Example for the laptop:

```bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=192.168.12.135
```

You can add the correct exports to `~/.bashrc` for convenience.

### 4. Start ROS Master

In a host terminal:

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<YOUR_HOST_WIFI_IP>
roscore
```

### 5. Start The Unitree High-Level Bridge

In a new host terminal, with the ROS environment set and the Unitree workspace sourced:

```bash
source /opt/ros/noetic/setup.bash
source ~/go1_wifi_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<YOUR_HOST_WIFI_IP>
roslaunch unitree_legged_real real.launch ctrl_level:=highlevel
```

Confirm the command/state topics exist:

```bash
rostopic list | egrep "high_cmd|high_state"
```

Expected:

```text
/high_cmd
/high_state
```

Optional sanity check:

```bash
rostopic echo -n 1 /high_state
```

### 6. SSH To The Jetson AGX Xavier

Make sure the Jetson is powered on and connected to the Go1 Wi-Fi network. In the lab setup, it connects automatically if the Unitree hotspot is available at boot. If the Jetson was powered before the Go1 hotspot came up, restart the Jetson.

From the host PC:

```bash
ssh lab@192.168.12.202
```

### 7. Run The ZED Camera Docker Container On The Jetson

On the Jetson:

```bash
sudo docker run -it --rm \
  --runtime nvidia \
  --network host \
  --privileged \
  --add-host host-pc:192.168.12.162 \
  -e DISPLAY=$DISPLAY \
  -v /home/lab/object_Detection_shared:/home \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/local/zed/resources:/usr/local/zed/resources \
  -v /dev/bus/usb:/dev/bus/usb \
  object_detection
```

If the host PC IP is different, update the `--add-host` IP. The important network behavior is `--network host`, because the container must talk to the host PC ROS master over the Go1 Wi-Fi network.

### 8. Set ROS Variables Inside The Jetson Container

Inside the container, point ROS to the host PC master and advertise the Jetson IP:

```bash
export ROS_MASTER_URI=http://192.168.12.162:11311
export ROS_IP=192.168.12.202
```

For the laptop host example:

```bash
export ROS_MASTER_URI=http://192.168.12.135:11311
export ROS_IP=192.168.12.202
```

### 9. Run The Camera Node Inside Docker

Inside the container:

```bash
cd /home
source /opt/ros/noetic/setup.bash
source /home/autoware/devel/setup.bash
python3 camera_node.py
```

This publishes:

- `/detections` as `autoware_msgs/DetectedObjectArray`
- `/camera_pose` as `geometry_msgs/PoseStamped`
- `/video` as `sensor_msgs/Image`

It also writes camera/pose bags under `/home/bags` in the container, which maps to:

```text
/home/lab/object_Detection_shared/bags
```

on the Jetson host.

### 10. Optional RViz Visualization

In a new host terminal from this repo:

```bash
source /opt/ros/noetic/setup.bash
source ~/go1_wifi_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<YOUR_HOST_WIFI_IP>
python3 ros_nodes/markerFrom3d.py
```

In another host terminal:

```bash
rosrun rviz rviz
```

In RViz:

1. Set the fixed frame to `camera` or `world`.
2. Add the marker topic for detected objects.
3. Add the image topic to view the ZED camera stream.

### 11. Start The Density Controller

In a host terminal from this repo, start the robot/camera frame helper:

```bash
source /opt/ros/noetic/setup.bash
source ~/go1_wifi_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<YOUR_HOST_WIFI_IP>
python3 ros_nodes/create_frames.py
```

In another host terminal from this repo, run the density social-navigation controller:

```bash
source /opt/ros/noetic/setup.bash
source ~/go1_wifi_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=<YOUR_HOST_WIFI_IP>
python3 ros_nodes/motion_plan_exp.py
```

For the simpler baseline planner, use:

```bash
python3 ros_nodes/motion_plan.py
```

## Copy And Analyze ROS Bags

Copy bags from the Jetson shared folder to the host PC:

```bash
scripts/sync_jetson_bags.sh --list
scripts/sync_jetson_bags.sh --dry-run
scripts/sync_jetson_bags.sh
```

Generate plots from paired marker/velocity bags:

```bash
/usr/bin/python3 scripts/analyze_bag_runs.py --bag-dir trial_data/raw --out-dir trial_data/processed
```

Convert a camera image bag to MP4:

```bash
/usr/bin/python3 scripts/bag2mp4.py video.bag video.mp4 --topic video --fps 10
```

## Status

This repository is being cleaned from lab experiment code into a reproducible project repository. Raw ROS bags and large generated outputs are intentionally excluded from Git.
