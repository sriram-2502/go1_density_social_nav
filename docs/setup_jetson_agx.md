# Jetson AGX Xavier Setup

This project uses a Jetson AGX Xavier with ROS Noetic, JetPack 5.1.2-b104 / Jetson Linux L4T R35.4.1, and ZED SDK 5.1.2 support for a ZED2i camera.

## Verified JetPack

Observed on the Jetson AGX Xavier:

```text
Jetson Linux / L4T: R35.4.1
JetPack: 5.1.2-b104
Architecture: aarch64
```

Commands used:

```bash
cat /etc/nv_tegra_release
apt-cache show nvidia-jetpack | grep Version
```

## Verified ZED SDK

Observed through the Python ZED API:

```text
ZED SDK: 5.1.2
```

Command used:

```bash
python3 - <<'PY'
import pyzed.sl as sl
print(sl.Camera.get_sdk_version())
PY
```

Additional checks:

```bash
ls /usr/local/zed
QT_QPA_PLATFORM=offscreen /usr/local/zed/tools/ZED_Diagnostic
```

## Verify Camera

```bash
lsusb | grep -i stereolabs
```

## Docker Runtime

The lab workflow mounts the host shared directory into the container as `/home` so bags saved under `/home` in Docker are visible on the Jetson host.
