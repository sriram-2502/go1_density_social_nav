# Hardware

## Platform

- Unitree Go1 quadruped
- ZED2i stereo camera
- Jetson AGX Xavier, older 2022-era unit
- [MAXOAK K2 185Wh / 50000mAh external battery pack](https://maxoak.net/products/maxoak-k2-185wh-50000mah-power-bank), using the 20 V output to power the Jetson AGX Xavier

## Saved G-code

Saved Bambu Studio print files for the custom camera/compute mounting hardware are stored in:

```text
hardware/safe_gcode/
```

These `.gcode.3mf` files are print-ready archives, not clean STL/CAD sources. Keep the original `.stl`, `.step`, CAD, or unsliced `.3mf` project files here if they become available.

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
