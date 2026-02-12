################################################################################
#
# kedei62 — DRM driver for KeDei 6.2 SPI TFT display
#
################################################################################

KEDEI62_VERSION = 1.0
KEDEI62_SITE = https://github.com/naens/kedei62
KEDEI62_SITE_METHOD = git

KEDEI62_LICENSE = GPL-2.0
KEDEI62_LICENSE_FILES = kedei62.c
KEDEI62_INSTALL_IMAGES = YES

# Use the kernel-module infra to build the .ko
$(eval $(kernel-module))

# Compile the DT overlay after the kernel modules (so dtc is available).
define KEDEI62_BUILD_DT_OVERLAY
	$(LINUX_DIR)/scripts/dtc/dtc -@ -I dts -O dtb \
		-o $(@D)/kedei62.dtbo $(@D)/kedei62.dts
endef
KEDEI62_POST_BUILD_HOOKS += KEDEI62_BUILD_DT_OVERLAY

# Install the overlay where genimage picks it up for the boot FAT partition.
define KEDEI62_INSTALL_IMAGES_CMDS
	$(INSTALL) -D -m 0644 $(@D)/kedei62.dtbo \
		$(BINARIES_DIR)/rpi-firmware/overlays/kedei62.dtbo
endef

$(eval $(generic-package))
