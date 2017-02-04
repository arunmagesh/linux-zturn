/*
 * Copyright 2015 Freescale Semiconductor, Inc.
 *
 * Freescale DCU drm device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/fsl_devices.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/backlight.h>
#include <video/videomode.h>
#include <video/of_display_timing.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder_slave.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
//#include <drm/drm_modes.h>


#define SII902X_INPUT_BUS_FMT	0x08
#define SII902X_TPI_AVI_INPUT_FMT	0x09
#define SII902X_TPI_AVI_OUTPUT_FMT	0x0A
#define SII902X_SYS_CONTROL	0x1A
#define SII902X_SYS_CTR_DDC_REQ	BIT(2)
#define SII902X_SYS_CTR_DDC_BUS_AVAI	(BIT(2) | BIT(1))
#define SII902X_TPI_FAMILY_DEV_ID	0x1B
#define SII902X_TPI_DEV_REV_ID	0x1C
#define SII902X_TPI_REV_LEVEL_ID	0x1D
#define SII902X_POWER_STATE	0x1E
#define SII902X_TPI_AUDIO_CFG0	0x24
#define SII902X_TPI_AUDIO_CFG1	0x25
#define SII902X_TPI_AUDIO_CFG2	0x26
#define SII902X_TPI_AUDIO_CFG3	0x27
#define SII902X_TPI_HDCP_REV	0x30
#define SII902X_TPI_INT_ENABLE	0x3C
#define SII902X_TPI_INT_STATUS	0x3D
#define SII902X_TPI_INT_PLUG_IN	BIT(2)
#define SII902X_GENERAL_PURPOSE_IO0	0xBC
#define SII902X_GENERAL_PURPOSE_IO1	0xBD
#define SII902X_GENERAL_PURPOSE_IO2	0xBE
#define SII902X_TRANS_MODE_DIFF	0xC7

#define DET_RETRY_CNT	2

bool g_enable_hdmi = 1;
#define SII_EDID_LEN	512
struct sii902x_data {
	struct i2c_client *client;
	struct delayed_work det_work;
	uint8_t edid_buf[SII_EDID_LEN];
	struct fb_info *fbi;
	int retries;
} *sii902x;

#define to_sii902x_data(x) \
	((struct sii902x_data *)to_encoder_slave(x)->slave_priv)

static struct i2c_client *sii902x_to_i2c(struct sii902x_data *sii902x)
{
	return sii902x->client;
}

/* HW access functions */
static s32 sii902x_write(const struct i2c_client *client,
			 u8 command, u8 value)
{
	return i2c_smbus_write_byte_data(client, command, value);
}

static s32 sii902x_read(const struct i2c_client *client, u8 command)
{
	int val;

	val = i2c_smbus_read_word_data(client, command);
	msleep(1);
	return val & 0xff;
}

static void sii902x_poweron(void)
{
	/* Turn on DVI or HDMI */
	pr_info(KERN_INFO "%s\n", __func__);
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, 0x01); // 0x00 if dvi
}

static void sii902x_poweroff(void)
{
	/* disable tmds before changing resolution */
	pr_info(KERN_INFO "%s\n", __func__);
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, 0x11);	// 0x10 if dvi
}


static void sii902x_chip_id(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	/* read device ID */
	val = sii902x_read(client, SII902X_TPI_FAMILY_DEV_ID);
	pr_info("Sii902x: read id = 0x%02X", val);
	val = sii902x_read(client, SII902X_TPI_DEV_REV_ID);
	pr_info("-0x%02X", val);
	val = sii902x_read(client, SII902X_TPI_REV_LEVEL_ID);
	pr_info("-0x%02X", val);
	val = sii902x_read(client, SII902X_TPI_HDCP_REV);
	pr_info("-0x%02X\n", val);
}

static int sii902x_cable_connected(void)
{
	/* Power on sii902x */
	sii902x_poweron();
	
	return 0;
}

static int sii902x_initialize(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int ret, cnt;

	for (cnt = 0; cnt < 5; cnt++) {
		/* Set 902x in hardware TPI mode on and jump out of D3 state */
		ret = sii902x_write(client, SII902X_TRANS_MODE_DIFF, 0x00);
		if (ret < 0)
			break;
	}
	if (ret != 0)
		dev_err(&client->dev, "cound not find device\n");

	return ret;
}

static void sii902x_enable_source(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	sii902x_write(client, SII902X_GENERAL_PURPOSE_IO0, 0x01);
	sii902x_write(client, SII902X_GENERAL_PURPOSE_IO1, 0x82);
	val = sii902x_read(client, SII902X_GENERAL_PURPOSE_IO2);
	val |= 0x1;
	sii902x_write(client, SII902X_GENERAL_PURPOSE_IO2, val);
}

static void sii902x_power_up_tx(struct sii902x_data *sii902x)
{
	struct i2c_client *client = sii902x_to_i2c(sii902x);
	int val;

	val = sii902x_read(client, SII902X_POWER_STATE);
	val &= ~0x3;
	sii902x_write(client, SII902X_POWER_STATE, val);
}

static int sii902x_get_edid_preconfig(void)
{
	int old, dat, ret = 0, cnt = 100;

	old = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);

	do {
		cnt--;
		sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_REQ);
		msleep(200);
		dat = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	} while ((!(dat & 0x2)) && cnt);

	if (!cnt) {
		ret = -1;
		goto done;
	}

	sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_BUS_AVAI);

done:
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, old);
	return ret;
}

static int __init enable_hdmi_setup(char *str)
{
	g_enable_hdmi = true;

	return 1;
}

__setup("hdmi", enable_hdmi_setup);

static void sii902x_hpd(struct sii902x_data *work)
{
	int dat;
	//char event_string[16];
	//char *envp[] = { event_string, NULL };

	pr_info("%s\n", __func__);

	dat = i2c_smbus_read_byte_data(sii902x->client, SII902X_TPI_INT_STATUS);

	pr_info(KERN_ERR"status: %#X, sii902x->retries: %d\n", dat, sii902x->retries);
//	if ((dat & 0x1) || sii902x->retries > 0) {
	if (dat >= 0) {
		/* cable connection changes */
		if (dat & SII902X_TPI_INT_PLUG_IN) {
			//sii902x.cable_plugin = 1;
			g_enable_hdmi = 1;
			//dev_dbg(&sii902x.client->dev, "EVENT=plugin\n");
			//sprintf(event_string, "EVENT=plugin");
			if (sii902x_cable_connected() < 0 && sii902x->retries > 0) {
				sii902x->retries --;
				//schedule_delayed_work(&(sii902x->det_work), msecs_to_jiffies(500));
			} else {
				sii902x->retries = 0;
			}
		} else {
			sii902x->retries = 0;
			//sii902x.cable_plugin = 0;
			g_enable_hdmi = 1;
//			dev_dbg(&sii902x.client->dev, "EVENT=plugout\n");
	//		sprintf(event_string, "EVENT=plugout");
			/* Power off sii902x */
			sii902x_poweroff();
		}
		//kobject_uevent_env(&sii902x->client->dev.kobj, KOBJ_CHANGE, envp);
	} else {
		//dev_err(&sii902x.client->dev, "i2c bus error!!!\n");
		sii902x->retries = 0;
	}
	i2c_smbus_write_byte_data(sii902x->client, SII902X_TPI_INT_STATUS, dat);

	//dev_dbg(&sii902x.client->dev, "exit %s\n", __func__);
}

static irqreturn_t sii902x_detect_handler(int irq, void *data)
{	
	pr_info("sii9022a in irq handle\n");
	
		//if (sii902x->retries == 0) {/* no need to schedule workqueue if retries > 0 */
			sii902x->retries = DET_RETRY_CNT;
			//schedule_delayed_work(&(sii902x->det_work), msecs_to_jiffies(100/*20*/));
		//}
	sii902x_hpd(sii902x);
	//if (g_enable_hdmi)
	//	g_enable_hdmi = false;

	return IRQ_HANDLED;
}

static int sii902x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct i2c_adapter *adap = to_i2c_adapter(client->dev.parent);
	int ret, err;

	if (!g_enable_hdmi)
		return -EPERM;

	if (!i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -ENODEV;
	}

	sii902x = devm_kzalloc(&client->dev, sizeof(*sii902x), GFP_KERNEL);
	if (!sii902x)
		return -ENOMEM;

	sii902x->client = client;
	i2c_set_clientdata(client, sii902x);

	err = sii902x_initialize(sii902x);
	if (err)
		return err;

	sii902x_chip_id(sii902x);
	sii902x_power_up_tx(sii902x);
	sii902x_enable_source(sii902x);

	if (sii902x_get_edid_preconfig() < 0)
		dev_warn(&client->dev, "Read edid preconfig failed\n");

	if (client->irq) {
		// ret = devm_request_irq(&client->dev, client->irq,
				       // sii902x_detect_handler, IRQF_ONESHOT,
				       // "SII902x_det", sii902x);
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
						sii902x_detect_handler,
						IRQF_ONESHOT, dev_name(&client->dev),
						sii902x);
		if (ret) {
			dev_warn(&client->dev,
				 "cound not request det irq %d\n",
				 client->irq);
			return ret;
		} else {
			/*enable cable hot plug irq*/
			pr_info("enable sii9022a interrupt\n");
			sii902x_write(client, SII902X_TPI_INT_ENABLE, 0x01);
			//INIT_DELAYED_WORK(&(sii902x->det_work), det_worker);
		}
	}
	
	sii902x->retries = 0;
	
	return 0;
}

static int sii902x_remove(struct i2c_client *client)
{	
	pr_info("%s\n",__func__);
	sii902x_poweroff();
	return 0;
}

/* DRM encoder functions */

static void
sii902x_encoder_dpms(struct drm_encoder *encoder, int mode)
{	
	pr_info("%s\n",__func__);
	if (mode == DRM_MODE_DPMS_ON)
		sii902x_poweron();
	else
		sii902x_poweroff();
}

static enum drm_connector_status
sii902x_encoder_detect(struct drm_encoder *encoder,
		       struct drm_connector *connector)
{
	//struct sii902x_data *adv7511 = to_sii902x_data(encoder);
	enum drm_connector_status status;
	unsigned int val;
	//bool hpd;
	//int ret;
	
	pr_info("%s\n",__func__);
	
	val = sii902x_read(sii902x->client, SII902X_TPI_INT_STATUS);
	
	if (val & SII902X_TPI_INT_PLUG_IN)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	// hpd = adv7511_hpd(adv7511);

	// /* The chip resets itself when the cable is disconnected, so in case
	 // * there is a pending HPD interrupt and the cable is connected there was
	 // * at least one transition from disconnected to connected and the chip
	 // * has to be reinitialized. */
	// if (status == connector_status_connected && hpd && adv7511->powered) {
		// regcache_mark_dirty(adv7511->regmap);
		// adv7511_power_on(adv7511);
		// adv7511_get_modes(encoder, connector);
		// if (adv7511->status == connector_status_connected)
			// status = connector_status_disconnected;
	// } else {
		// /* Renable HPD sensing */
		// regmap_update_bits(adv7511->regmap, ADV7511_REG_POWER2,
				   // ADV7511_REG_POWER2_HPD_SRC_MASK,
				   // ADV7511_REG_POWER2_HPD_SRC_BOTH);
	// }

	// adv7511->status = status;
	return status;
}

static int
sii902x_encoder_mode_valid(struct drm_encoder *encoder,
			  struct drm_display_mode *mode)
{
	pr_info("%s\n",__func__);
	return MODE_OK;
}

void drm_display_mode_to_videomode(const struct drm_display_mode *dmode,
                                   struct videomode *vm)
{
	vm->hactive = dmode->hdisplay;
	vm->hfront_porch = dmode->hsync_start - dmode->hdisplay;
	vm->hsync_len = dmode->hsync_end - dmode->hsync_start;
	vm->hback_porch = dmode->htotal - dmode->hsync_end;

	vm->vactive = dmode->vdisplay;
	vm->vfront_porch = dmode->vsync_start - dmode->vdisplay;
	vm->vsync_len = dmode->vsync_end - dmode->vsync_start;
	vm->vback_porch = dmode->vtotal - dmode->vsync_end;

	vm->pixelclock = dmode->clock * 1000;

	vm->flags = 0;
	if (dmode->flags & DRM_MODE_FLAG_PHSYNC)
			vm->flags |= DISPLAY_FLAGS_HSYNC_HIGH;
	else if (dmode->flags & DRM_MODE_FLAG_NHSYNC)
			vm->flags |= DISPLAY_FLAGS_HSYNC_LOW;
	if (dmode->flags & DRM_MODE_FLAG_PVSYNC)
			vm->flags |= DISPLAY_FLAGS_VSYNC_HIGH;
	else if (dmode->flags & DRM_MODE_FLAG_NVSYNC)
			vm->flags |= DISPLAY_FLAGS_VSYNC_LOW;
	if (dmode->flags & DRM_MODE_FLAG_INTERLACE)
			vm->flags |= DISPLAY_FLAGS_INTERLACED;
	if (dmode->flags & DRM_MODE_FLAG_DBLSCAN)
			vm->flags |= DISPLAY_FLAGS_DOUBLESCAN;
	if (dmode->flags & DRM_MODE_FLAG_DBLCLK)
			vm->flags |= DISPLAY_FLAGS_DOUBLECLK;
}

static void
sii902x_encoder_mode_set(struct drm_encoder *encoder,
			struct drm_display_mode *mode,
			struct drm_display_mode *adjusted_mode)
{
	struct videomode vm;
	u16 data[4];
	u32 refresh;
	u8 *tmp;
	int i;
	
	pr_info("%s\n",__func__);
	
	/* Power up */
	sii902x_power_up_tx(sii902x);

	drm_display_mode_to_videomode(mode, &vm);
	data[0] = PICOS2KHZ(vm.pixelclock) / 10;
	data[2] = vm.hsync_len + vm.hback_porch +
		  vm.hactive + vm.hfront_porch;
	data[3] = vm.vsync_len + vm.vfront_porch +
		  vm.vactive + vm.vback_porch;
	refresh = data[2] * data[3];
	refresh = (PICOS2KHZ(vm.pixelclock) * 1000) / refresh;
	data[1] = refresh * 100;

	tmp = (u8 *)data;
	for (i = 0; i < 8; i++)
		sii902x_write(sii902x->client, i, tmp[i]);

	/* input bus/pixel: full pixel wide (24bit), rising edge */
	sii902x_write(sii902x->client, SII902X_INPUT_BUS_FMT, 0x70);
	/* Set input format to RGB */
	sii902x_write(sii902x->client, SII902X_TPI_AVI_INPUT_FMT, 0x00);
	/* set output format to RGB */
	sii902x_write(sii902x->client, SII902X_TPI_AVI_OUTPUT_FMT, 0x00);
	/* audio setup */
	sii902x_write(sii902x->client, SII902X_TPI_AUDIO_CFG1, 0x00);
	sii902x_write(sii902x->client, SII902X_TPI_AUDIO_CFG2, 0x40);
	sii902x_write(sii902x->client, SII902X_TPI_AUDIO_CFG3, 0x00);
}

/* -----------------------------------------------------------------------------
 * EDID retrieval
 */
 /*--------------------------------EDID-begin-----------------------------*/
//#define EDID_LENGTH				0x80 defined in include/drm/drm_edid.h == 128
#define EDID_ROM_ADDR       	0x50

static int myir_edid_readblk(struct i2c_adapter *adp,
		unsigned short addr, unsigned char *edid)
{
	int ret = 0, extblknum = 0;
	unsigned char regaddr = 0x0;
	struct i2c_msg msg[2] = {
		{
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &regaddr,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= EDID_LENGTH,
		.buf	= edid,
		},
	};

	ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		pr_info("unable to read EDID block, ret=%d(expected:%d)\n", ret, ARRAY_SIZE(msg));
		return -EIO;
	}

	if (edid[1] == 0x00)
		return -ENOENT;

	extblknum = edid[0x7E];
	
	if (extblknum) {
		regaddr = 128;
		msg[1].buf = edid + EDID_LENGTH;

		ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
		if (ret != ARRAY_SIZE(msg)) {
			pr_info("unable to read EDID ext block, ret=%d(expected:%d)\n", ret, ARRAY_SIZE(msg));
			return -EIO;
		}
	}

	return extblknum;
}

static int myir_edid_readsegblk(struct i2c_adapter *adp, unsigned short addr,
			unsigned char *edid, int seg_num)
{
	int ret = 0;
	unsigned char segment = 0x1, regaddr = 0;
	struct i2c_msg msg[3] = {
		{
		.addr	= 0x30,
		.flags	= 0,
		.len	= 1,
		.buf	= &segment,
		}, {
		.addr	= addr,
		.flags	= 0,
		.len	= 1,
		.buf	= &regaddr,
		}, {
		.addr	= addr,
		.flags	= I2C_M_RD,
		.len	= EDID_LENGTH,
		.buf	= edid,
		},
	};

	ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		pr_info("unable to read EDID block\n");
		return -EIO;
	}

	if (seg_num == 2) {
		regaddr = 128;
		msg[2].buf = edid + EDID_LENGTH;

		ret = i2c_transfer(adp, msg, ARRAY_SIZE(msg));
		if (ret != ARRAY_SIZE(msg)) {
			pr_info("unable to read EDID block\n");
			return -EIO;
		}
	}

	return ret;
}

/* make sure edid has 512 bytes*/
int myir_edid_read(struct i2c_adapter *adp, unsigned short addr,
	unsigned char *edid)
{
	int ret = 0, extblknum;

	if (!adp || !edid)
		return -EINVAL;

	memset(edid, 0, EDID_LENGTH*4);

	extblknum = myir_edid_readblk(adp, addr, edid);

	//printk(KERN_ERR"extblknum1: %d\n", extblknum);
	//printk(KERN_ERR"extblknum2: %d\n", extblknum);
	//printk(KERN_ERR"extblknum3: %d\n", extblknum);
	
	if (extblknum < 0)
		return extblknum;

	// /* edid first block parsing */
	// memset(&fbi->monspecs, 0, sizeof(fbi->monspecs));
	// fb_edid_to_monspecs(edid, &fbi->monspecs);

	if (extblknum) {
		//int i;

		/* need read segment block? */
		if (extblknum > 1) {
			ret = myir_edid_readsegblk(adp, addr,
				edid + EDID_LENGTH*2, extblknum - 1);
			if (ret < 0) {
				return ret;
			}
		}

		// for (i = 1; i <= extblknum; i++)
			// /* edid ext block parsing */
			// myir_edid_parse_ext_blk(edid + i*EDID_LENGTH,
					// cfg, &fbi->monspecs);
	}

	return 0;
}

static int sii902x_get_edid_block(void *data, u8 *buf, unsigned int block,
				  size_t len)
{
	struct sii902x_data *sii902x_ptr = data;
	int ret;
	
	pr_info("%s\n",__func__);

	if (len > 128)
		return -EINVAL;
	
	ret = myir_edid_read(sii902x_ptr->client->adapter,EDID_ROM_ADDR,sii902x_ptr->edid_buf);

	if (block % 2 == 0)
		memcpy(buf, sii902x_ptr->edid_buf, len);
	else
		memcpy(buf, sii902x_ptr->edid_buf + 128, len);

	return ret;
}
/*--------------------------------EDID-end-----------------------------*/

struct edid *sii902x_drm_get_edid(struct drm_connector *connector,
				  //struct i2c_adapter *adapter)
				  struct sii902x_data * sii902x_ptr)
{
	int old, dat, cnt = 100;
	struct edid *edid;
	
	pr_info("%s\n",__func__);
	
	old = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	
	//Loop till 0x1A[1] reads "1"
	do {
		cnt--;
		sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_REQ);
		msleep(200);
		dat = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	} while ((!(dat & 0x2)) && cnt);
	
	// Failure... restore original value.
	if (!cnt) {
		pr_info("Access DDC failed!\n");
		edid = NULL;
		goto done;
	}
	
	// lock host DDC bus access (0x1A[2:1] = 11)
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
		      old | SII902X_SYS_CTR_DDC_BUS_AVAI);

	/* edid reading */
	//edid = drm_get_edid(connector, adapter);
	edid = drm_do_get_edid(connector, sii902x_get_edid_block, sii902x_ptr);

	cnt = 100;
	do {
		cnt--;
		sii902x_write(sii902x->client, SII902X_SYS_CONTROL,
			      old & ~SII902X_SYS_CTR_DDC_BUS_AVAI);
		msleep(20);
		dat = sii902x_read(sii902x->client, SII902X_SYS_CONTROL);
	} while ((dat & SII902X_SYS_CTR_DDC_BUS_AVAI) && cnt); // When 0x1A[2:1] read "0"

	if (!cnt) {
		edid = NULL;
		pr_info("Release DDC failed!\n");
	}

done:
	sii902x_write(sii902x->client, SII902X_SYS_CONTROL, old);
	return edid;
}

static int
sii902x_encoder_get_modes(struct drm_encoder *encoder,
			 struct drm_connector *connector)
{
	//struct drm_device *dev = connector->dev;
	//struct i2c_adapter *adap = to_i2c_adapter(sii902x->client->dev.parent);
	struct edid *edid;
	struct drm_display_mode *mode;
	int ret;
	ret = 0;
	
	pr_info("%s\n",__func__);
	
	//edid = sii902x_drm_get_edid(connector, adap);
	edid = sii902x_drm_get_edid(connector, to_sii902x_data(encoder));
	if (edid) {
		drm_mode_connector_update_edid_property(connector, edid);
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	} else {
		pr_info("failed to get edid\n");
	}

	list_for_each_entry(mode, &connector->probed_modes, head) {
		if (mode->hdisplay == 1024 || mode->vdisplay == 768)
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		else
			mode->type &= ~DRM_MODE_TYPE_PREFERRED;
	}

	return ret;
}

static struct drm_encoder_slave_funcs sii902x_encoder_funcs = {
	.dpms = sii902x_encoder_dpms,
	.mode_valid = sii902x_encoder_mode_valid,
	.mode_set = sii902x_encoder_mode_set,
	.detect = sii902x_encoder_detect,
	.get_modes = sii902x_encoder_get_modes,
};
static int
sii902x_encoder_init(struct i2c_client *client,
		    struct drm_device *dev,
		    struct drm_encoder_slave *encoder)
{
	struct sii902x_data * sii902x_ptr = i2c_get_clientdata(client);
	encoder->slave_priv = sii902x_ptr;
	
	encoder->slave_funcs = &sii902x_encoder_funcs;

	return 0;
}
static const struct i2c_device_id sii902x_id[] = {
	{ "sii902x", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, sii902x_id);

static const struct of_device_id sii902x_dt_ids[] = {
	{ .compatible = "sil,sii9022a", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sii902x_dt_ids);

static struct drm_i2c_encoder_driver sii902x_driver = {
	.i2c_driver = {
		.probe = sii902x_probe,
		.remove = sii902x_remove,
		.driver = {
			.name = "sii902x",
			.owner = THIS_MODULE,
			.of_match_table = sii902x_dt_ids,
		},
		.id_table = sii902x_id,
	},
	.encoder_init = sii902x_encoder_init,
};

/* Module initialization */

static int __init sii902x_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &sii902x_driver);
}

static void __exit sii902x_exit(void)
{
	drm_i2c_encoder_unregister(&sii902x_driver);
}


MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SII902x DVI/HDMI driver");
MODULE_LICENSE("GPL");


module_init(sii902x_init);
module_exit(sii902x_exit);
