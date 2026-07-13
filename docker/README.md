# Docker Environment

Do not commit Docker image binaries to Git. The lab image is large, Jetson/aarch64-specific, and should be kept as a private artifact only.

## Verified Platform

- Jetson: AGX Xavier
- JetPack: 5.1.2-b104
- Jetson Linux / L4T: R35.4.1
- ZED SDK: 5.1.2
- ROS in the container: Noetic
- Python in the container: 3.8.10

## Fastest Reproduction Path

If you have the saved private image tar, place it at:

```text
docker/image_backup/object_detection_jetson_agx_xavier_jp512_zed512.tar
```

Then run on the Jetson:

```bash
make docker-load
make docker-run
```

This restores the exact saved `object_detection:latest` image. `docker/image_backup/` is ignored by Git.

## Optional Private Source Tar

If you also have a private Docker source/build-context tar, keep it outside Git under:

```text
docker/source_backup/docker_source.tar
```

Unpack it for local reference with:

```bash
make docker-unpack-source
```

That backup is useful for audit and recovery, but the GitHub-facing reproducible path should stay in `docker/Dockerfile` and `Makefile`. `docker/source_backup/` is ignored by Git.

## Rebuild From Source

The clean rebuild path is:

```bash
make docker-build
make docker-run
```

`make docker-build` uses `docker/Dockerfile`, which starts from:

```text
stereolabs/zed:5.1.2-tools-devel-jetson-jp5.1.2
```

It installs the ROS Noetic Python/runtime packages used by the camera scripts, builds `autoware_ai_messages` for `autoware_msgs/DetectedObjectArray`, and copies the custom Jetson-side nodes from `jetson_nodes/` into `/home`.

This rebuild is the maintainable path for GitHub. It is not guaranteed to be byte-for-byte identical to the original lab image because the original `object_detection:latest` image had several manual layers and no dedicated source Dockerfile was recovered.

## Run Container

Use:

```bash
make docker-run
```

or directly:

```bash
./docker/run_camera_container.sh
```

Equivalent expanded command:

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

The shared folder mount means files saved in the container under `/home` appear on the Jetson host under:

```text
/home/lab/object_Detection_shared
```

`camera_node.py` writes camera and pose bags under `/home/bags` in the container, so on the Jetson they appear under:

```text
/home/lab/object_Detection_shared/bags
```

## What Was Recovered

The lab image used for perception was:

```text
object_detection:latest
```

`docker history object_detection:latest` showed it was based on:

```text
stereolabs/zed:5.1.2-tools-devel-jetson-jp5.1.2
```

with four manual `/bin/bash` layers on top:

```text
20 MB
12.9 MB
2.52 GB
1.71 GB
```

A search on the Jetson did not find a dedicated Dockerfile or Makefile for `object_detection`. The relevant generic base-image source appeared to be under:

```text
/home/lab/jetson-containers/packages/hw/zed/Dockerfile
```

The useful recovered metadata is stored in `docker/metadata/`:

```text
docker_apt_manual.txt
docker_pip_freeze.txt
docker_ros_env.txt
object_detection_history_full.txt
object_detection_inspect.json
autoware_src_listing.txt
docker_jetson_node_summary.txt
```

## Private Image Backup Commands

Save on the Jetson:

```bash
sudo docker save object_detection:latest -o /home/lab/object_Detection_shared/object_detection_jetson_agx_xavier_jp512_zed512.tar
```

Restore later on a Jetson:

```bash
sudo docker load -i docker/image_backup/object_detection_jetson_agx_xavier_jp512_zed512.tar
```
