# Hardware

## Platform

- Unitree Go1 quadruped
- ZED2i stereo camera
- Jetson AGX Xavier, older 2022-era unit
- MAXOAK K2 185Wh / 50000mAh external battery pack

## Camera Mount

Place STL files for the custom ZED2i mount in:

```text
hardware/camera_mount_stl/
```

Add photos of the mounted camera and Jetson wiring under:

```text
docs/assets/images/
```

## Details To Verify

On the Jetson:

```bash
cat /etc/nv_tegra_release
jetson_release
/usr/local/zed/tools/ZED_Diagnostic
```

If `jetson_release` is missing, install/use `jtop` only if appropriate for the lab machine.
