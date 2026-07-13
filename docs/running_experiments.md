# Running Experiments

## Host PC

```bash
roscore
```

## Jetson / Camera Container

Run the ZED/perception container with the shared folder mounted at `/home`.

## Robot Control

```bash
python3 ros_nodes/motion_plan_exp.py
```

The experiment node:

- subscribes to `/detections` as `autoware_msgs/DetectedObjectArray`
- subscribes to `/camera_pose` as `geometry_msgs/PoseStamped`
- publishes `/detections_markers` for RViz
- publishes Unitree `HighCmd` commands to `/high_cmd`
- writes timestamped `vel_*.bag` and `markers_*.bag` files
