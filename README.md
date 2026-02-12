# KeDei 6.2 SPI TFT display driver

Linux DRM kernel module for the KeDei 6.2 TFT display (480×320, R61581
controller).  Targets Raspberry Pi 3 Model B, kernel 6.12+.

## Hardware

The KeDei 6.2 board connects to the RPi 40-pin header over SPI0.  A
74HC595 shift register on the board converts SPI into a parallel bus for
the R61581 LCD controller.  Every SPI transaction is a 3-byte packet:

| Prefix | Meaning |
|--------|---------|
| `0x11` | Command |
| `0x15` | Data    |
| `0x00` | Reset control |

Two GPIOs are used:

- **GPIO 8 (CE0)** — 74HC595 latch (RCLK), active-high.  Directly
  toggled by the driver around each SPI packet.
- **GPIO 7 (CE1)** — SPI chip-select for `spi0.1`, active-low.  Managed
  by the SPI framework.

The backlight is hardwired to 5 V and cannot be turned off in software.
On shutdown the driver fills the screen with black pixels so the display
appears dark.

## Building (on the Raspberry Pi)

```bash
sudo apt install raspberrypi-kernel-headers build-essential device-tree-compiler
make            # builds kedei62.ko
make dtbo       # compiles kedei62.dtbo
```

## Installing

```bash
sudo cp kedei62.ko /lib/modules/$(uname -r)/kernel/misc/
sudo depmod -a

sudo cp kedei62.dtbo /boot/firmware/overlays/
```

Add to `/boot/firmware/config.txt`:

```
dtparam=spi=on
dtoverlay=kedei62
```

Reboot.  The display appears as `/dev/dri/card*` and fbdev emulation
provides `/dev/fb*`.

## DT overlay parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `speed`   | 39000000 | SPI clock frequency (Hz) |
| `rotate`  | 270     | Display rotation in degrees (0, 90, 180, 270) |

Example: `dtoverlay=kedei62,rotate=0,speed=16000000`

## Buildroot

Copy or symlink the package directory into `package/kedei62/`:

```
package/kedei62/
├── Config.in
└── kedei62.mk
```

Add to `package/Config.in`:

```
source "package/kedei62/Config.in"
```

Enable `BR2_PACKAGE_KEDEI62` in `menuconfig`.  The `.mk` file uses
`$(eval $(kernel-module))` to build the `.ko` and a post-build hook to
compile the DT overlay.

Ensure the board's `config.txt` contains the `dtoverlay=kedei62`
line and run `make rpi-firmware-rebuild` if `config.txt` is embedded via
`genimage`.

## Console on the display

```bash
sudo con2fbmap 1 1
```

## Files

| File | Purpose |
|------|---------|
| `kedei62.c`   | Driver source (DRM simple-pipe, async SPI pixel push) |
| `kedei62.dts` | Device Tree overlay for RPi SPI0 |
| `Makefile`    | Out-of-tree kernel module build |
| `kedei62.mk`  | Buildroot package definition |
| `Config.in`   | Buildroot Kconfig fragment |

## License

GPL-2.0-only
