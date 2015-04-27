/*
 * Driver for OV5645 CMOS Image Sensor from Omnivision
 *
 * Copyright (C) 2015 Linaro Ltd, Vaibhav Hiremath <vaibhav.hiremath@linaro.org>
 *
 *
 * Based on Sony OV5642 Camera Driver
 * Copyright (C) 2011, Bastian Hecht <hechtb@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/* OV5645 registers */
#define REG_CHIP_ID_HIGH		0x300a
#define REG_CHIP_ID_LOW			0x300b

/* Others registers should follow here */

/* active pixel array size */
#define OV5645_SENSOR_SIZE_X		2592
#define OV5645_SENSOR_SIZE_Y		1944

/*
 * About OV5645 resolutions:
 */

#define OV5645_DEFAULT_FORMAT		V4L2_MBUS_FMT_YUYV8_2X8
#define OV5645_DEFAULT_WIDTH		1280
#define OV5645_DEFAULT_HEIGHT		960

struct ov5645 {
	struct i2c_client *client;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_mbus_framefmt format;
	struct v4l2_rect crop;

	struct mutex power_lock; /* lock to protect power_count */
	int power_count;
};

/*
 * TODO: Until we get initialization sequence, comment this portion, as
 * someone else sets sensor registers for us
 */
#if 0
struct regval_list {
	u16 reg_num;
	u8 value;
};

static struct regval_list ov5645_default_regs_init[] = {
	{ 0xffff, 0xff },
};

#endif

static struct ov5645 *to_ov5645(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5645, subdev);
}

#if 0
static int ov5645_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	/* We have 16-bit i2c addresses - care for endianess */
	unsigned char data[2] = { reg >> 8, reg & 0xff };
	int ret;

	ret = i2c_master_send(client, data, 2);
	if (ret < 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 1) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

/*
 * convenience function to write 16 bit register values that are split up
 * into two consecutive high and low parts
 */
static int ov5645_reg_read16(struct i2c_client *client, u16 reg, u16 *val16)
{
	u8 val8;
	int ret;

	ret = ov5645_reg_read(client, reg, &val8);
	if (ret)
		return ret;

	*val16 = val8 << 8;

	ret = ov5645_reg_read(client, reg + 1, &val8);
	if (ret)
		return ret;

	*val16 |= val8;

	return 0;
}

static int ov5645_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };
	int ret;

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

/*
 * convenience function to write 16 bit register values that are split up
 * into two consecutive high and low parts
 */
static int ov5645_reg_write16(struct i2c_client *client, u16 reg, u16 val16)
{
	int ret;

	ret = ov5645_reg_write(client, reg, val16 >> 8);
	if (ret)
		return ret;
	return ov5645_reg_write(client, reg + 1, val16 & 0x00ff);
}

static int ov5645_write_array(struct i2c_client *client,
				struct regval_list *vals)
{
	while (vals->reg_num != 0xffff) {
		int ret = ov5645_reg_write(client, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	dev_dbg(&client->dev, "Register list loaded\n");
	return 0;
}

#endif

static int ov5645_power_on(struct ov5645 *ov5645)
{
	dev_dbg(&ov5645->client->dev, "power on\n");

	return 0;
}

static void ov5645_power_off(struct ov5645 *ov5645)
{
	dev_dbg(&ov5645->client->dev, "power off\n");
}

static int ov5645_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov5645 *ov5645 = to_ov5645(sd);
	int ret = 0;

	mutex_lock(&ov5645->power_lock);

	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (ov5645->power_count == !on) {
		if (on) {
			ret = ov5645_power_on(ov5645);
			if (ret < 0)
				goto out;
		} else {
			ov5645_power_off(ov5645);
		}
	}

	/* Update the power count. */
	ov5645->power_count += on ? 1 : -1;
	WARN_ON(ov5645->power_count < 0);

out:
	mutex_unlock(&ov5645->power_lock);
	return ret;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5645_get_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ov5645_reg_read(client, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ov5645_set_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ov5645_reg_write(client, reg->reg, reg->val);
}
#endif

static int ov5645_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* TODO: configure sensor to start streaming here */

	dev_dbg(&client->dev, "Streaming %s\n",
		enable ? "enabled" : "disabled");

	return 0;
}

static void ov5645_init_format(struct v4l2_mbus_framefmt *format,
			       struct v4l2_rect *crop)
{
	format->width = OV5645_DEFAULT_WIDTH;
	format->height = OV5645_DEFAULT_HEIGHT;
	format->code = OV5645_DEFAULT_FORMAT;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_JPEG;

	crop->left = 0;
	crop->top = 0;
	crop->width = OV5645_DEFAULT_WIDTH;
	crop->height = OV5645_DEFAULT_HEIGHT;
}

static void ov5645_init_cfg(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	format = v4l2_subdev_get_try_format(sd, cfg, 0);
	crop = v4l2_subdev_get_try_crop(sd, cfg, 0);

	ov5645_init_format(format, crop);
}

static int ov5645_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = OV5645_DEFAULT_FORMAT;
	return 0;
}

static int ov5645_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > 0 || fse->code != OV5645_DEFAULT_FORMAT)
		return -EINVAL;

	fse->min_width = OV5645_DEFAULT_WIDTH;
	fse->max_width = OV5645_DEFAULT_WIDTH;
	fse->min_height = OV5645_DEFAULT_HEIGHT;
	fse->max_height = OV5645_DEFAULT_HEIGHT;

	return 0;
}

static struct v4l2_mbus_framefmt *
ov5645_get_pad_format(struct ov5645 *ov5645,
		      struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
	default:
		return v4l2_subdev_get_try_format(&ov5645->subdev, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &ov5645->format;
	}
}

static int ov5645_get_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *format)
{
	struct ov5645 *ov5645 = to_ov5645(sd);

	format->format = *ov5645_get_pad_format(ov5645, cfg, format->pad,
						format->which);

	return 0;
}

static int ov5645_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_format *format)
{
	/* The format is fixed, set_fmt just becomes get_fmt. */
	return ov5645_get_fmt(sd, cfg, format);
}

static struct v4l2_subdev_core_ops ov5645_subdev_core_ops = {
	.s_power	= ov5645_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov5645_get_register,
	.s_register	= ov5645_set_register,
#endif
};

static struct v4l2_subdev_video_ops ov5645_subdev_video_ops = {
	.s_stream = ov5645_s_stream,
};

static struct v4l2_subdev_pad_ops ov5645_subdev_pad_ops = {
	.init_cfg	= ov5645_init_cfg,
	.enum_mbus_code	= ov5645_enum_mbus_code,
	.enum_frame_size = ov5645_enum_frame_size,
	.get_fmt	= ov5645_get_fmt,
	.set_fmt	= ov5645_set_fmt,
};

static struct v4l2_subdev_ops ov5645_subdev_ops = {
	.core	= &ov5645_subdev_core_ops,
	.video	= &ov5645_subdev_video_ops,
	.pad	= &ov5645_subdev_pad_ops,
};

static int ov5645_detect_sensor(struct ov5645 *ov5645)
{
	int ret;
	u16 id = 0;

	ret = ov5645_power_on(ov5645);
	if (ret < 0)
		return ret;

	/* Read sensor Model ID */
#if 0
	ret = ov5645_reg_read16(ov5645->client, REG_CHIP_ID_LOW, &id);
	if (ret < 0)
		goto done;
#else
	/*
	 * We have no I2C access to the device for the Google I/O demo as
	 * everything is configured by the firmware. Just fake the ID for now.
	 */
	id = 0x5645;
#endif

	dev_info(&ov5645->client->dev, "Chip ID 0x%04x detected\n", id);

	if (id != 0x5645) {
		ret = -ENODEV;
		goto done;
	}

done:
	ov5645_power_off(ov5645);
	return ret;
}

static int ov5645_probe(struct i2c_client *client,
			const struct i2c_device_id *dev_id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ov5645 *ov5645;
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov5645 = devm_kzalloc(&client->dev, sizeof(*ov5645), GFP_KERNEL);
	if (!ov5645)
		return -ENOMEM;

	mutex_init(&ov5645->power_lock);
	ov5645->client = client;

	ret = ov5645_detect_sensor(ov5645);
	if (ret) {
		dev_err(&client->dev, "sensor detection failed\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&ov5645->subdev, client, &ov5645_subdev_ops);

	ov5645->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&ov5645->subdev.entity, 1, &ov5645->pad, 0);
	if (ret < 0)
		return ret;

	ov5645_init_format(&ov5645->format, &ov5645->crop);

	ret = v4l2_async_register_subdev(&ov5645->subdev);
	if (ret < 0) {
		media_entity_cleanup(&ov5645->subdev.entity);
		return ret;
	}

	return 0;
}

static int ov5645_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct i2c_device_id ov5645_id[] = {
	{ "ov5645", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5645_id);

static struct i2c_driver ov5645_i2c_driver = {
	.driver = {
		.name = "ov5645",
	},
	.probe		= ov5645_probe,
	.remove		= ov5645_remove,
	.id_table	= ov5645_id,
};

module_i2c_driver(ov5645_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV5645 Camera driver");
MODULE_AUTHOR("Vaibhav Hiremath <vaibhav.hiremath@linaro.orgm>");
MODULE_LICENSE("GPL v2");
