/*
 * Goodix Gesture Module
 *
 * Copyright (C) 2019 - 2020 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/input/mt.h>
#include "goodix_ts_core.h"


#define GOODIX_GESTURE_DOUBLE_TAP		0xCC
#define GOODIX_GESTURE_SINGLE_TAP		0x4C
#define GOODIX_GESTURE_FOD_DOWN			0x46
#define GOODIX_GESTURE_FOD_UP			0x55

static ssize_t gsx_double_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
			(type & GESTURE_DOUBLE_TAP) ? "enable" : "disable");
}

static ssize_t gsx_double_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("enable double tap");
		cd->gesture_type |= GESTURE_DOUBLE_TAP;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info("disable double tap");
		cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t gsx_single_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
			(type & GESTURE_SINGLE_TAP) ? "enable" : "disable");
}

static ssize_t gsx_single_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("enable single tap");
		cd->gesture_type |= GESTURE_SINGLE_TAP;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info("disable single tap");
		cd->gesture_type &= ~GESTURE_SINGLE_TAP;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t gsx_fod_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
			(type & GESTURE_FOD_PRESS) ? "enable" : "disable");
}

static ssize_t gsx_fod_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("enable fod");
		cd->gesture_type |= GESTURE_FOD_PRESS;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info("disable fod");
		cd->gesture_type &= ~GESTURE_FOD_PRESS;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

int goodix_ts_report_gesture(struct goodix_ts_core *cd, struct goodix_ts_event *event)
{
	int fodx, fody, overlay_area;

	switch (event->gesture_type) {
	case GOODIX_GESTURE_SINGLE_TAP:
		if (cd->gesture_type & GESTURE_SINGLE_TAP) {
			ts_info("get SINGLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			// input_report_key(cd->input_dev, KEY_GOTO, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			// input_report_key(cd->input_dev, KEY_GOTO, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable SINGLE-TAP");
		}
		break;
	case GOODIX_GESTURE_DOUBLE_TAP:
		if (cd->gesture_type & GESTURE_DOUBLE_TAP) {
			ts_info("get DOUBLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable DOUBLE-TAP");
		}
		break;
	case GOODIX_GESTURE_FOD_DOWN:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info("get FOD-DOWN gesture");
			fodx = le16_to_cpup((__le16 *)event->gesture_data);
			fody = le16_to_cpup((__le16 *)(event->gesture_data + 2));
			overlay_area = event->gesture_data[4];
			ts_debug("fodx:%d fody:%d overlay_area:%d", fodx, fody, overlay_area);
			input_report_key(cd->input_dev, BTN_TOUCH, 1);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_X, fodx);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_Y, fody);
			input_report_abs(cd->input_dev, ABS_MT_WIDTH_MAJOR, overlay_area);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable FOD-DOWN");
		}
		break;
	case GOODIX_GESTURE_FOD_UP:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info("get FOD-UP gesture");
			// fodx = le16_to_cpup((__le16 *)gs_event.gesture_data);
			// fody = le16_to_cpup((__le16 *)(gs_event.gesture_data + 2));
			// overlay_area = gs_event.gesture_data[4];
			input_report_key(cd->input_dev, BTN_TOUCH, 0);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev,
					MT_TOOL_FINGER, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable FOD-UP");
		}
		break;
	default:
		ts_err("not support gesture type[%02X]", event->gesture_type);
		break;
	}

	return 0;
}

static DEVICE_ATTR(double_type, 0664, gsx_double_type_show, gsx_double_type_store);
static DEVICE_ATTR(single_type, 0664, gsx_single_type_show, gsx_single_type_store);
static DEVICE_ATTR(fod_type, 0664, gsx_fod_type_show, gsx_fod_type_store);

static struct attribute *gesture_attrs[] = {
	&dev_attr_double_type.attr,
	&dev_attr_single_type.attr,
	&dev_attr_fod_type.attr,
	NULL,
};

const static struct attribute_group gesture_sysfs_group = {
	.attrs = gesture_attrs,
};

int gesture_module_init(struct goodix_ts_core *core_data)
{
	int ret = 0;
	struct kobject *parent = &core_data->pdev->dev.kobj;

	/* gesture sysfs init */
	core_data->gesture_kobj = kobject_create_and_add("gesture", parent);
	if (!core_data->gesture_kobj) {
		ts_err("failed create gesture sysfs node!");
		goto err_out;
	}

	ret = sysfs_create_group(core_data->gesture_kobj, &gesture_sysfs_group);
	if (ret) {
		ts_err("failed create gesture sysfs files");
		kobject_put(core_data->gesture_kobj);
		goto err_out;
	}

	ts_info("gesture module init success");
	return 0;

err_out:
	ts_err("gesture module init failed!");
	return ret;
}

void gesture_module_exit(struct goodix_ts_core *core_data)
{
	ts_info("gesture module exit");

	sysfs_remove_group(core_data->gesture_kobj, &gesture_sysfs_group);
	kobject_put(core_data->gesture_kobj);
}
