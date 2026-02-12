// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for the KeDei 6.2 SPI TFT display (480×320, R61581).
 *
 * The display is wired through a 74HC595 shift register that acts as a
 * parallel-bus adapter on the SPI lines.  Every SPI transaction is a
 * 3-byte packet: {prefix, high, low}.  Prefix 0x11 = command,
 * 0x15 = data, 0x00 = reset control.  GPIO 8 (active-high) latches
 * the shift register; GPIO 7 / CE1 is the SPI chip-select.
 *
 * Uses drm_simple_display_pipe with an async workqueue for pixel
 * updates — pipe_update() records dirty rectangles and schedules a
 * worker that pushes pixels to the panel over SPI.
 *
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#define DRIVER_NAME	"kedei62"
#define DRIVER_DESC	"KeDei 6.2 display driver"
#define DRIVER_MAJOR	1
#define DRIVER_MINOR	0

/* Native panel resolution (before rotation) */
#define WIDTH		480
#define HEIGHT		320

/* R61581 Memory Access Control (0x36) values per rotation */
static const u8 lcd_rotations[4] = {
	0x0A,	/*   0°  portrait  — MV=0, MY=0, MX=0, BGR */
	0x6A,	/*  90°  landscape — MV=1, MY=0, MX=1, BGR */
	0xCA,	/* 180°  portrait  — MV=0, MY=1, MX=1, BGR */
	0xAA,	/* 270°  landscape — MV=1, MY=1, MX=0, BGR */
};

struct kedei62 {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;
	const struct drm_display_mode *mode;
	struct spi_device *spi;
	struct gpio_desc *cs;
	unsigned int width;
	unsigned int height;
	unsigned int rotation;
	u8 *tx_buf;

	/* Async display update (worker + dirty-rect coalescing) */
	struct work_struct update_work;
	spinlock_t dirty_lock;     /* protects dirty_rect, dirty, fb_* */
	struct drm_rect dirty_rect;
	void *fb_vaddr;            /* GEM buffer virtual address */
	u32 fb_pitch;              /* framebuffer pitch in bytes */
	bool dirty;                /* true when dirty_rect is valid */
	bool enabled;              /* gate: true after pipe_enable completes */
};

#define drm_to_kedei(x) container_of(x, struct kedei62, drm)

/* ------------------------------------------------------------------ */
/*  SPI primitives                                                    */
/* ------------------------------------------------------------------ */

/*
 * Send a 3-byte SPI packet through the 74HC595 shift register.
 * The latch GPIO (RCLK) is raised before the transfer and lowered
 * afterwards — the rising edge latches the previous shift-register
 * contents onto the parallel bus.
 */
static int kedei_write(struct kedei62 *kd, u8 header, u8 b1, u8 b2)
{
	struct spi_transfer tr = {};
	int ret;

	kd->tx_buf[0] = header;
	kd->tx_buf[1] = b1;
	kd->tx_buf[2] = b2;

	tr.tx_buf = kd->tx_buf;
	tr.len = 3;

	if (kd->cs)
		gpiod_set_value(kd->cs, 1);

	ret = spi_sync_transfer(kd->spi, &tr, 1);

	if (kd->cs)
		gpiod_set_value(kd->cs, 0);

	return ret;
}

/* Send a command byte (prefix 0x11) to the R61581. */
static void lcd_cmd(struct kedei62 *kd, u8 cmd)
{
	kedei_write(kd, 0x11, 0x00, cmd);
}

/* Send a data byte (prefix 0x15) to the R61581. */
static void lcd_data(struct kedei62 *kd, u8 dat)
{
	kedei_write(kd, 0x15, 0x00, dat);
}

/* Program the Memory Access Control register (0x36) for rotation. */
static void lcd_setrotation(struct kedei62 *kd, u8 m)
{
	lcd_cmd(kd, 0x36);
	lcd_data(kd, lcd_rotations[m & 3]);
}

/* Send a command followed by a variable-length data sequence. */
static void kedei62_command_buf(struct kedei62 *kd, u8 cmd,
				const u8 *data, size_t len)
{
	size_t i;

	lcd_cmd(kd, cmd);
	for (i = 0; i < len; i++)
		lcd_data(kd, data[i]);
}

#define kedei62_command(kd, cmd, seq...) \
({ \
	const u8 d[] = { seq }; \
	kedei62_command_buf(kd, cmd, d, ARRAY_SIZE(d)); \
})

/* ------------------------------------------------------------------ */
/*  Hardware reset / power control                                    */
/* ------------------------------------------------------------------ */

/*
 * Hardware reset via the 74HC595 bus.  Prefix 0x00 addresses the
 * CPLD reset line; bit 0 of the second byte controls the reset pin:
 *   {0x00, 0x01, 0x00} = assert reset  (active-high)
 *   {0x00, 0x00, 0x00} = release reset
 */
static void reset(struct kedei62 *kd)
{
	kedei_write(kd, 0x00, 0x01, 0x00);	/* assert reset */
	mdelay(50);
	kedei_write(kd, 0x00, 0x00, 0x00);	/* release reset */
	mdelay(100);
}

/*
 * Fill the entire display with black pixels.
 *
 * The KeDei 6.2 backlight is hardwired to 5 V — it cannot be turned
 * off in software.  Sending Display OFF (0x28) or Sleep In (0x10)
 * causes the R61581 to stop driving the LCD, and the always-on
 * backlight shines through the transparent panel → white screen.
 *
 * Instead, we fill the GRAM with black and leave the controller in
 * normal display mode.  Black pixels block the backlight, so the
 * screen appears dark.  When the system actually cuts power, the
 * backlight and controller die together.
 *
 * Takes ~1.3 s at 39 MHz SPI (153 600 pixels × 3 bytes each).
 */
static void fill_black(struct kedei62 *kd)
{
	unsigned int i, total = kd->width * kd->height;

	if (!total)
		return;

	lcd_cmd(kd, 0x2A);
	lcd_data(kd, 0x00);
	lcd_data(kd, 0x00);
	lcd_data(kd, (kd->width - 1) >> 8);
	lcd_data(kd, (kd->width - 1) & 0xFF);

	lcd_cmd(kd, 0x2B);
	lcd_data(kd, 0x00);
	lcd_data(kd, 0x00);
	lcd_data(kd, (kd->height - 1) >> 8);
	lcd_data(kd, (kd->height - 1) & 0xFF);

	lcd_cmd(kd, 0x2C);

	for (i = 0; i < total; i++)
		kedei_write(kd, 0x15, 0x00, 0x00);
}

/* ------------------------------------------------------------------ */
/*  Pixel push & async worker                                        */
/* ------------------------------------------------------------------ */

/*
 * Push pixels from the GEM buffer to the display over SPI.
 *
 * Sets the R61581 address window to the dirty rectangle, then writes
 * each RGB565 pixel as a 3-byte {0x15, hi, lo} SPI packet.
 * Called from the update worker — runs in process context,
 * never blocks the atomic commit path.
 */
static void kedei62_push_pixels(struct kedei62 *kd, struct drm_rect *rect)
{
	u16 x, y;
	int idx;

	if (!drm_dev_enter(&kd->drm, &idx))
		return;

	/* Column address set (0x2A) */
	lcd_cmd(kd, 0x2A);
	lcd_data(kd, rect->x1 >> 8);
	lcd_data(kd, rect->x1 & 0xFF);
	lcd_data(kd, (rect->x2 - 1) >> 8);
	lcd_data(kd, (rect->x2 - 1) & 0xFF);

	/* Page address set (0x2B) */
	lcd_cmd(kd, 0x2B);
	lcd_data(kd, rect->y1 >> 8);
	lcd_data(kd, rect->y1 & 0xFF);
	lcd_data(kd, (rect->y2 - 1) >> 8);
	lcd_data(kd, (rect->y2 - 1) & 0xFF);

	/* Memory write (0x2C) */
	lcd_cmd(kd, 0x2C);

	for (y = rect->y1; y < rect->y2; y++) {
		u16 *line = (u16 *)((u8 *)kd->fb_vaddr +
				    y * kd->fb_pitch + rect->x1 * 2);
		for (x = rect->x1; x < rect->x2; x++) {
			u16 pixel = *line++;

			kedei_write(kd, 0x15, pixel >> 8, pixel & 0xFF);
		}
	}

	drm_dev_exit(idx);
}

/*
 * Workqueue callback — picks up the accumulated dirty rectangle
 * and pushes those pixels to the display.  Loops until no more
 * dirty work remains, so rapid pipe_update() calls are coalesced
 * into a single SPI transfer of the union rectangle.
 */
static void kedei62_update_worker(struct work_struct *work)
{
	struct kedei62 *kd = container_of(work, struct kedei62, update_work);
	struct drm_rect rect;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&kd->dirty_lock, flags);
		if (!kd->dirty || !kd->enabled) {
			spin_unlock_irqrestore(&kd->dirty_lock, flags);
			return;
		}
		rect = kd->dirty_rect;
		kd->dirty = false;
		spin_unlock_irqrestore(&kd->dirty_lock, flags);

		kedei62_push_pixels(kd, &rect);
	}
}

/* ------------------------------------------------------------------ */
/*  DRM simple-pipe operations                                       */
/* ------------------------------------------------------------------ */

static enum drm_mode_status
kedei62_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
			const struct drm_display_mode *mode)
{
	struct drm_crtc *crtc = &pipe->crtc;
	struct kedei62 *kd = drm_to_kedei(crtc->dev);

	return drm_crtc_helper_mode_valid_fixed(crtc, mode, kd->mode);
}

static void kedei62_pipe_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct kedei62 *kd = drm_to_kedei(pipe->crtc.dev);
	struct drm_gem_dma_object *dma_obj;
	unsigned long flags;
	int idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	dev_info(kd->drm.dev, "Enabling display\n");

	reset(kd);

	/* NOP + synchronization pulses to flush the 74HC595 pipeline */
	lcd_cmd(kd, 0x00);
	mdelay(10);
	lcd_cmd(kd, 0xFF);
	lcd_cmd(kd, 0xFF);
	mdelay(10);
	lcd_cmd(kd, 0xFF);
	lcd_cmd(kd, 0xFF);
	lcd_cmd(kd, 0xFF);
	lcd_cmd(kd, 0xFF);
	mdelay(15);

	/* 0x11: Sleep Out — wake up the controller */
	lcd_cmd(kd, 0x11);
	mdelay(150);

	/* 0xB0: Manufacturer Command Access Protect — unlock */
	kedei62_command(kd, 0xB0, 0x00);

	/* 0xB3: Frame Memory Access & Interface Setting */
	kedei62_command(kd, 0xB3, 0x02, 0x00, 0x00, 0x00);

	/* 0xB9: Internal oscillator (vendor-specific) */
	kedei62_command(kd, 0xB9, 0x01, 0x00, 0x0F, 0x0F);

	/* 0xC0: Panel Driving Setting */
	kedei62_command(kd, 0xC0, 0x13, 0x3B, 0x00, 0x02,
			0x00, 0x01, 0x00, 0x43);

	/* 0xC1: Display Timing for Normal Mode */
	kedei62_command(kd, 0xC1, 0x08, 0x0F, 0x08, 0x08);

	/* 0xC4: Source/Gate Timing (vendor-specific) */
	kedei62_command(kd, 0xC4, 0x11, 0x07, 0x03, 0x04);

	/* 0xC6: Interface Control (vendor-specific) */
	kedei62_command(kd, 0xC6, 0x00);

	/* 0xC8: Gamma Setting (20 parameters for pos/neg curves) */
	kedei62_command(kd, 0xC8, 0x03, 0x03, 0x13, 0x5C,
			0x03, 0x07, 0x14, 0x08,
			0x00, 0x21, 0x08, 0x14,
			0x07, 0x53, 0x0C, 0x13,
			0x03, 0x03, 0x21, 0x00);

	/* 0x35: Tearing Effect Line ON */
	kedei62_command(kd, 0x35, 0x00);

	/* 0x36: Memory Access Control — landscape, BGR */
	kedei62_command(kd, 0x36, 0x60);

	/* 0x3A: Pixel Format — 16 bits/pixel (RGB565) */
	kedei62_command(kd, 0x3A, 0x55);

	/* 0x44: Set Tear Scanline */
	kedei62_command(kd, 0x44, 0x00, 0x01);

	/* 0xD0: Power Setting */
	kedei62_command(kd, 0xD0, 0x07, 0x07, 0x1D, 0x03);

	/* 0xD1: VCOM Control */
	kedei62_command(kd, 0xD1, 0x03, 0x30, 0x10);

	/* 0xD2: Power Setting for Normal Mode */
	kedei62_command(kd, 0xD2, 0x03, 0x14, 0x04);

	/* 0x29: Display ON */
	lcd_cmd(kd, 0x29);
	mdelay(30);

	/* 0x2A: Column Address Set — 0..319 (0x013F) */
	kedei62_command(kd, 0x2A, 0x00, 0x00, 0x01, 0x3F);

	/* 0x2B: Page Address Set — 0..479 (0x01E0) */
	kedei62_command(kd, 0x2B, 0x00, 0x00, 0x01, 0xE0);

	/* 0xB4: Display Mode / Frame Inversion — normal */
	kedei62_command(kd, 0xB4, 0x00);

	/* 0x2C: Memory Write — ready for pixel data */
	lcd_cmd(kd, 0x2C);
	mdelay(10);

	/*
	 * Apply rotation from the DT "rotate" property (degrees).
	 * Program the R61581's Memory Access Control register (0x36)
	 * and swap width/height for portrait orientations.
	 * Default: 270° (landscape, connector on the right).
	 */
	switch (kd->rotation) {
	case 0:
		kd->width = HEIGHT;
		kd->height = WIDTH;
		lcd_setrotation(kd, 0);
		break;
	case 90:
		kd->width = WIDTH;
		kd->height = HEIGHT;
		lcd_setrotation(kd, 1);
		break;
	case 180:
		kd->width = HEIGHT;
		kd->height = WIDTH;
		lcd_setrotation(kd, 2);
		break;
	case 270:
	default:
		kd->width = WIDTH;
		kd->height = HEIGHT;
		lcd_setrotation(kd, 3);
		break;
	}

	/*
	 * Mark the display as ready.  pipe_update() calls that arrived
	 * before this point (DRM commits planes before enables) were
	 * dropped, preventing SPI bus corruption during init.
	 *
	 * Trigger the initial full-screen push from the plane state
	 * that was provided with this enable call.
	 */
	if (plane_state->fb) {
		dma_obj = drm_fb_dma_get_gem_obj(plane_state->fb, 0);

		spin_lock_irqsave(&kd->dirty_lock, flags);
		kd->fb_vaddr = dma_obj->vaddr;
		kd->fb_pitch = plane_state->fb->pitches[0];
		kd->dirty_rect.x1 = 0;
		kd->dirty_rect.y1 = 0;
		kd->dirty_rect.x2 = kd->width;
		kd->dirty_rect.y2 = kd->height;
		kd->dirty = true;
		kd->enabled = true;
		spin_unlock_irqrestore(&kd->dirty_lock, flags);

		schedule_work(&kd->update_work);
	} else {
		kd->enabled = true;
	}

	drm_dev_exit(idx);
}

static void kedei62_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct kedei62 *kd = drm_to_kedei(pipe->crtc.dev);

	kd->enabled = false;
	cancel_work_sync(&kd->update_work);
	fill_black(kd);
}

static void kedei62_pipe_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_state)
{
	struct kedei62 *kd = drm_to_kedei(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_gem_dma_object *dma_obj;
	struct drm_rect rect;
	unsigned long flags;

	if (!pipe->crtc.state->active || !kd->enabled)
		return;

	if (!drm_atomic_helper_damage_merged(old_state, state, &rect))
		return;

	dma_obj = drm_fb_dma_get_gem_obj(state->fb, 0);

	spin_lock_irqsave(&kd->dirty_lock, flags);
	kd->fb_vaddr = dma_obj->vaddr;
	kd->fb_pitch = state->fb->pitches[0];
	if (kd->dirty) {
		/* Merge with pending dirty area */
		kd->dirty_rect.x1 = min(kd->dirty_rect.x1, rect.x1);
		kd->dirty_rect.y1 = min(kd->dirty_rect.y1, rect.y1);
		kd->dirty_rect.x2 = max(kd->dirty_rect.x2, rect.x2);
		kd->dirty_rect.y2 = max(kd->dirty_rect.y2, rect.y2);
	} else {
		kd->dirty_rect = rect;
		kd->dirty = true;
	}
	spin_unlock_irqrestore(&kd->dirty_lock, flags);

	schedule_work(&kd->update_work);
}

static const struct drm_simple_display_pipe_funcs kedei62_pipe_funcs = {
	.mode_valid = kedei62_pipe_mode_valid,
	.enable     = kedei62_pipe_enable,
	.disable    = kedei62_pipe_disable,
	.update     = kedei62_pipe_update,
};

/* ------------------------------------------------------------------ */
/*  Mode, format & connector                                         */
/* ------------------------------------------------------------------ */

static const struct drm_display_mode kedei62_mode = {
	DRM_SIMPLE_MODE(480, 320, 73, 49),
};

static const u32 kedei62_formats[] = {
	DRM_FORMAT_RGB565,
};

static int kedei62_connector_get_modes(struct drm_connector *connector)
{
	struct kedei62 *kd = drm_to_kedei(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, kd->mode);
}

static const struct drm_connector_helper_funcs kedei62_connector_hfuncs = {
	.get_modes = kedei62_connector_get_modes,
};

static const struct drm_connector_funcs kedei62_connector_funcs = {
	.reset                  = drm_atomic_helper_connector_reset,
	.fill_modes             = drm_helper_probe_single_connector_modes,
	.destroy                = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
};

/* ------------------------------------------------------------------ */
/*  DRM driver                                                       */
/* ------------------------------------------------------------------ */

static const struct drm_mode_config_funcs kedei62_mode_config_funcs = {
	.fb_create     = drm_gem_fb_create_with_dirty,
	.atomic_check  = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(kedei62_fops);

static const struct drm_driver kedei62_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops            = &kedei62_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.name            = DRIVER_NAME,
	.desc            = DRIVER_DESC,
	.major           = DRIVER_MAJOR,
	.minor           = DRIVER_MINOR,
};

/* ------------------------------------------------------------------ */
/*  SPI driver (probe / remove / shutdown)                            */
/* ------------------------------------------------------------------ */

static const struct of_device_id kedei62_of_match[] = {
	{ .compatible = "kedei,kedei62" },
	{},
};
MODULE_DEVICE_TABLE(of, kedei62_of_match);

static const struct spi_device_id kedei62_id[] = {
	{ "kedei62", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, kedei62_id);

static int kedei62_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct kedei62 *kd;
	struct drm_device *drm;
	struct gpio_desc *cs;
	u32 rotation = 0;
	int ret;

	/*
	 * SPI devices don't have a DMA mask by default.
	 * drm_gem_dma needs one to allocate framebuffer memory.
	 */
	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set DMA mask\n");

	kd = devm_drm_dev_alloc(dev, &kedei62_driver, struct kedei62, drm);
	if (IS_ERR(kd))
		return PTR_ERR(kd);

	drm = &kd->drm;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width  = WIDTH;
	drm->mode_config.max_width  = WIDTH;
	drm->mode_config.min_height = HEIGHT;
	drm->mode_config.max_height = HEIGHT;
	drm->mode_config.funcs = &kedei62_mode_config_funcs;

	cs = devm_gpiod_get(dev, "cs", GPIOD_OUT_LOW);
	if (IS_ERR(cs))
		return dev_err_probe(dev, PTR_ERR(cs), "Failed to get GPIO 'cs'\n");

	device_property_read_u32(dev, "rotate", &rotation);
	kd->mode = &kedei62_mode;
	kd->spi = spi;
	kd->cs = cs;
	kd->rotation = rotation;

	kd->tx_buf = devm_kmalloc(dev, 16, GFP_KERNEL | GFP_DMA);
	if (!kd->tx_buf)
		return -ENOMEM;

	spin_lock_init(&kd->dirty_lock);
	INIT_WORK(&kd->update_work, kedei62_update_worker);

	drm_connector_helper_add(&kd->connector, &kedei62_connector_hfuncs);
	ret = drm_connector_init(drm, &kd->connector, &kedei62_connector_funcs,
				 DRM_MODE_CONNECTOR_SPI);
	if (ret)
		return ret;

	ret = drm_simple_display_pipe_init(drm, &kd->pipe, &kedei62_pipe_funcs,
					   kedei62_formats,
					   ARRAY_SIZE(kedei62_formats),
					   NULL, &kd->connector);
	if (ret)
		return ret;

	drm_plane_enable_fb_damage_clips(&kd->pipe.plane);

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_dma_setup(drm, 16);

	return 0;
}

static void kedei62_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void kedei62_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver kedei62_spi_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = kedei62_of_match,
	},
	.id_table = kedei62_id,
	.probe    = kedei62_probe,
	.remove   = kedei62_remove,
	.shutdown = kedei62_shutdown,
};
module_spi_driver(kedei62_spi_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("naens <naens@gmx.com>");
MODULE_LICENSE("GPL");
