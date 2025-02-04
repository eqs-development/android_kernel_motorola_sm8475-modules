 /*
  * Goodix Touchscreen Driver
  * Copyright (C) 2020 - 2021 Goodix, Inc.
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
  *
  */
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include "goodix_ts_core.h"

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
#include <linux/input/mt.h>
#define INPUT_TYPE_B_PROTOCOL
#endif
#include "goodix_ts_mmi.h"

/* goodix fb test */
// #include "../../../video/fbdev/core/fb_firefly.h"

#define GOODIX_DEFAULT_CFG_NAME		"goodix_cfg_group.cfg"
struct goodix_device_manager goodix_devices;
#define PINCTRL_STATE_ACTIVE    "cli_pmx_ts_active"
#define PINCTRL_STATE_SUSPEND   "cli_pmx_ts_suspend"

static void goodix_device_manager_init(void)
{
	if (goodix_devices.initilized)
		return;
	goodix_devices.initilized = true;
	INIT_LIST_HEAD(&goodix_devices.list);
}

static void goodix_device_manager_exit(void)
{
	struct goodix_device_resource *res, *next;

	if (!list_empty(&goodix_devices.list)) {
		list_for_each_entry_safe(res, next, &goodix_devices.list, list) {
			platform_device_unregister(&res->pdev);
			kfree(res);
		}
	}
	goodix_devices.initilized = false;
}

int goodix_device_register(struct goodix_device_resource *device)
{
	if (!device)
		return -ENXIO;

	list_add(&device->list, &goodix_devices.list);
	device->id = goodix_devices.nums++;
	sprintf(device->name, "%s.%d", GOODIX_CORE_DRIVER_NAME, device->id);
	ts_info("goodix ts register device %s", device->name);
	return 0;
}

static int goodix_send_ic_config(struct goodix_ts_core *cd, int type);

/* show driver infomation */
static ssize_t driver_info_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DriverVersion:%s\n",
			GOODIX_DRIVER_VERSION);
}

/* show chip infoamtion */
static ssize_t chip_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	struct goodix_fw_version chip_ver;
	struct goodix_ic_info ic_info;
	u8 temp_pid[8] = {0};
	int ret;
	int cnt = -EINVAL;

	if (hw_ops->read_version) {
		ret = hw_ops->read_version(cd, &chip_ver);
		if (!ret) {
			memcpy(temp_pid, chip_ver.rom_pid,
					sizeof(chip_ver.rom_pid));
			cnt = snprintf(&buf[0], PAGE_SIZE,
				"rom_pid:%s\nrom_vid:%02x%02x%02x\n",
				temp_pid, chip_ver.rom_vid[0],
				chip_ver.rom_vid[1], chip_ver.rom_vid[2]);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"patch_pid:%s\npatch_vid:%02x%02x%02x%02x\n",
				chip_ver.patch_pid, chip_ver.patch_vid[0],
				chip_ver.patch_vid[1], chip_ver.patch_vid[2],
				chip_ver.patch_vid[3]);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
				"sensorid:%d\n", chip_ver.sensor_id);
		}
	}

	if (hw_ops->get_ic_info) {
		ret = hw_ops->get_ic_info(cd, &ic_info);
		if (!ret) {
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
					"config_id:%x\n",
					ic_info.version.config_id);
			cnt += snprintf(&buf[cnt], PAGE_SIZE,
					"config_version:%x\n",
					ic_info.version.config_version);
		}
	}

	return cnt;
}

/* reset chip */
static ssize_t goodix_ts_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (!buf || count <= 0)
		return -EINVAL;
	if (buf[0] != '0')
		hw_ops->reset(core_data, GOODIX_NORMAL_RESET_DELAY_MS);
	return count;
}

/* read config */
static ssize_t read_cfg_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	int ret;
	int i;
	int offset;
	char *cfg_buf = NULL;

	cfg_buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!cfg_buf)
		return -ENOMEM;

	if (hw_ops->read_config)
		ret = hw_ops->read_config(core_data, cfg_buf, PAGE_SIZE);
	else
		ret = -EINVAL;

	if (ret > 0) {
		offset = 0;
		for (i = 0; i < 200; i++) { // only print 200 bytes
			offset += snprintf(&buf[offset], PAGE_SIZE - offset,
					"%02x,", cfg_buf[i]);
			if ((i + 1) % 20 == 0)
				buf[offset++] = '\n';
		}
	}

	kfree(cfg_buf);
	if (ret <= 0)
		return ret;

	return offset;
}

static u8 ascii2hex(u8 a)
{
	s8 value = 0;

	if (a >= '0' && a <= '9')
		value = a - '0';
	else if (a >= 'A' && a <= 'F')
		value = a - 'A' + 0x0A;
	else if (a >= 'a' && a <= 'f')
		value = a - 'a' + 0x0A;
	else
		value = 0xff;

	return value;
}

static int goodix_ts_convert_0x_data(const u8 *buf, int buf_size,
				     u8 *out_buf, int *out_buf_len)
{
	int i, m_size = 0;
	int temp_index = 0;
	u8 high, low;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] == 'x' || buf[i] == 'X')
			m_size++;
	}

	if (m_size <= 1) {
		ts_err("cfg file ERROR, valid data count:%d", m_size);
		return -EINVAL;
	}
	*out_buf_len = m_size;

	for (i = 0; i < buf_size; i++) {
		if (buf[i] != 'x' && buf[i] != 'X')
			continue;

		if (temp_index >= m_size) {
			ts_err("exchange cfg data error, overflow, temp_index:%d,m_size:%d",
					temp_index, m_size);
			return -EINVAL;
		}
		high = ascii2hex(buf[i + 1]);
		low = ascii2hex(buf[i + 2]);
		if (high == 0xff || low == 0xff) {
			ts_err("failed convert: 0x%x, 0x%x",
				buf[i + 1], buf[i + 2]);
			return -EINVAL;
		}
		out_buf[temp_index++] = (high << 4) + low;
	}
	return 0;
}

/* send config */
static ssize_t goodix_ts_send_cfg_store(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ic_config *config = NULL;
	const struct firmware *cfg_img = NULL;
	int ret;

	if (buf[0] != '1')
		return -EINVAL;

	hw_ops->irq_enable(core_data, false);

	ret = request_firmware(&cfg_img, GOODIX_DEFAULT_CFG_NAME, dev);
	if (ret < 0) {
		ts_err("cfg file [%s] not available,errno:%d",
			GOODIX_DEFAULT_CFG_NAME, ret);
		goto exit;
	} else {
		ts_info("cfg file [%s] is ready", GOODIX_DEFAULT_CFG_NAME);
	}

	config = kzalloc(sizeof(*config), GFP_KERNEL);
	if (!config)
		goto exit;

	if (goodix_ts_convert_0x_data(cfg_img->data, cfg_img->size,
			config->data, &config->len)) {
		ts_err("convert config data FAILED");
		goto exit;
	}

	if (hw_ops->send_config) {
		ret = hw_ops->send_config(core_data, config->data, config->len);
		if (ret < 0)
			ts_err("send config failed");
	}

exit:
	hw_ops->irq_enable(core_data, true);
	kfree(config);
	if (cfg_img)
		release_firmware(cfg_img);

	return count;
}

/* reg read/write */
static u32 rw_addr;
static u32 rw_len;
static u8 rw_flag;
static u8 store_buf[32];
static u8 show_buf[PAGE_SIZE];
static ssize_t goodix_ts_reg_rw_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	int ret;

	if (!rw_addr || !rw_len) {
		ts_err("address(0x%x) and length(%d) can't be null",
			rw_addr, rw_len);
		return -EINVAL;
	}

	if (rw_flag != 1) {
		ts_err("invalid rw flag %d, only support [1/2]", rw_flag);
		return -EINVAL;
	}

	ret = hw_ops->read(core_data, rw_addr, show_buf, rw_len);
	if (ret < 0) {
		ts_err("failed read addr(%x) length(%d)", rw_addr, rw_len);
		return snprintf(buf, PAGE_SIZE,
			"failed read addr(%x), len(%d)\n",
			rw_addr, rw_len);
	}

	return snprintf(buf, PAGE_SIZE, "0x%x,%d {%*ph}\n",
		rw_addr, rw_len, rw_len, show_buf);
}

static ssize_t goodix_ts_reg_rw_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	char *pos = NULL;
	char *token = NULL;
	long result = 0;
	int ret;
	int i;

	if (!buf || !count) {
		ts_err("invalid parame");
		goto err_out;
	}

	if (buf[0] == 'r') {
		rw_flag = 1;
	} else if (buf[0] == 'w') {
		rw_flag = 2;
	} else {
		ts_err("string must start with 'r/w'");
		goto err_out;
	}

	/* get addr */
	pos = (char *)buf;
	pos += 2;
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid address info");
		goto err_out;
	} else {
		if (kstrtol(token, 16, &result)) {
			ts_err("failed get addr info");
			goto err_out;
		}
		rw_addr = (u32)result;
		ts_info("rw addr is 0x%x", rw_addr);
	}

	/* get length */
	token = strsep(&pos, ":");
	if (!token) {
		ts_err("invalid length info");
		goto err_out;
	} else {
		if (kstrtol(token, 0, &result)) {
			ts_err("failed get length info");
			goto err_out;
		}
		rw_len = (u32)result;
		ts_info("rw length info is %d", rw_len);
		if (rw_len > sizeof(store_buf)) {
			ts_err("data len > %lu", sizeof(store_buf));
			goto err_out;
		}
	}

	if (rw_flag == 1)
		return count;

	for (i = 0; i < rw_len; i++) {
		token = strsep(&pos, ":");
		if (!token) {
			ts_err("invalid data info");
			goto err_out;
		} else {
			if (kstrtol(token, 16, &result)) {
				ts_err("failed get data[%d] info", i);
				goto err_out;
			}
			store_buf[i] = (u8)result;
			ts_info("get data[%d]=0x%x", i, store_buf[i]);
		}
	}
	ret = hw_ops->write(core_data, rw_addr, store_buf, rw_len);
	if (ret < 0) {
		ts_err("failed write addr(%x) data %*ph", rw_addr,
			rw_len, store_buf);
		goto err_out;
	}

	ts_info("%s write to addr (%x) with data %*ph",
		"success", rw_addr, rw_len, store_buf);

	return count;
err_out:
	snprintf(show_buf, PAGE_SIZE, "%s\n",
		"invalid params, format{r/w:4100:length:[41:21:31]}");
	return -EINVAL;

}

/* show irq infomation */
static ssize_t goodix_ts_irq_info_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct irq_desc *desc;
	size_t offset = 0;
	int r;

	r = snprintf(&buf[offset], PAGE_SIZE, "irq:%u\n", core_data->irq);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "state:%s\n",
		     atomic_read(&core_data->irq_enabled) ?
		     "enabled" : "disabled");
	if (r < 0)
		return -EINVAL;

	desc = irq_to_desc(core_data->irq);
	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "disable-depth:%d\n",
		     desc->depth);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset, "trigger-count:%zu\n",
		core_data->irq_trig_cnt);
	if (r < 0)
		return -EINVAL;

	offset += r;
	r = snprintf(&buf[offset], PAGE_SIZE - offset,
		     "echo 0/1 > irq_info to disable/enable irq\n");
	if (r < 0)
		return -EINVAL;

	offset += r;
	return offset;
}

/* enable/disable irq */
static ssize_t goodix_ts_irq_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		hw_ops->irq_enable(core_data, true);
	else
		hw_ops->irq_enable(core_data, false);
	return count;
}

/* show esd status */
static ssize_t goodix_ts_esd_info_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	struct goodix_ts_core *core_data = dev_get_drvdata(dev);
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "state:%s\n",
		     atomic_read(&ts_esd->esd_on) ?
		     "enabled" : "disabled");

	return r;
}

/* enable/disable esd */
static ssize_t goodix_ts_esd_info_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);

	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		goodix_ts_esd_on(cd);
	else
		goodix_ts_esd_off(cd);
	return count;
}

/* debug level show */
static ssize_t goodix_ts_debug_log_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	int r = 0;

	r = snprintf(buf, PAGE_SIZE, "state:%s\n",
		    debug_log_flag ?
		    "enabled" : "disabled");

	return r;
}

/* debug level store */
static ssize_t goodix_ts_debug_log_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	if (!buf || count <= 0)
		return -EINVAL;

	if (buf[0] != '0')
		debug_log_flag = true;
	else
		debug_log_flag = false;
	return count;
}

/* show die package site and mcu fabs */
#define DIE_INFO_START_FLASH_ADDR 0x1F300
static ssize_t die_info_show(struct device  *dev,
		struct device_attribute *attr, char *buf)
{
	struct goodix_ts_core *cd = dev_get_drvdata(dev);
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	u8 temp_buf[21];
	u8 pkg_site;
	u8 mcu_fab;
	int ret;

	ret = hw_ops->read_flash(cd, DIE_INFO_START_FLASH_ADDR, temp_buf, sizeof(temp_buf));
	if (ret < 0) {
		ts_err("read flash failed");
		return 0;
	}

	ts_info("die info:%*ph", (int)sizeof(temp_buf), temp_buf);

	pkg_site = temp_buf[1];
	mcu_fab = temp_buf[20];
	ret = snprintf(buf, PAGE_SIZE, "package_id:0x%02X mcu_fab:0x%02X\n", pkg_site, mcu_fab);

	return ret;
}

static DEVICE_ATTR(driver_info, 0440,
		driver_info_show, NULL);
static DEVICE_ATTR(chip_info, 0440,
		chip_info_show, NULL);
static DEVICE_ATTR(reset, 0220,
		NULL, goodix_ts_reset_store);
static DEVICE_ATTR(send_cfg, 0220,
		NULL, goodix_ts_send_cfg_store);
static DEVICE_ATTR(read_cfg, 0440,
		read_cfg_show, NULL);
static DEVICE_ATTR(reg_rw, 0664,
		goodix_ts_reg_rw_show, goodix_ts_reg_rw_store);
static DEVICE_ATTR(irq_info, 0664,
		goodix_ts_irq_info_show, goodix_ts_irq_info_store);
static DEVICE_ATTR(esd_info, 0664,
		goodix_ts_esd_info_show, goodix_ts_esd_info_store);
static DEVICE_ATTR(debug_log, 0664,
		goodix_ts_debug_log_show, goodix_ts_debug_log_store);
static DEVICE_ATTR(die_info, 0440,
		die_info_show, NULL);

static struct attribute *sysfs_attrs[] = {
	&dev_attr_driver_info.attr,
	&dev_attr_chip_info.attr,
	&dev_attr_reset.attr,
	&dev_attr_send_cfg.attr,
	&dev_attr_read_cfg.attr,
	&dev_attr_reg_rw.attr,
	&dev_attr_irq_info.attr,
	&dev_attr_esd_info.attr,
	&dev_attr_debug_log.attr,
	&dev_attr_die_info.attr,
	NULL,
};

static const struct attribute_group sysfs_group = {
	.attrs = sysfs_attrs,
};

static int goodix_ts_sysfs_init(struct goodix_ts_core *core_data)
{
	int ret;

	ret = sysfs_create_group(&core_data->pdev->dev.kobj, &sysfs_group);
	if (ret) {
		ts_err("failed create core sysfs group");
		return ret;
	}

	return ret;
}

static void goodix_ts_sysfs_exit(struct goodix_ts_core *core_data)
{
	sysfs_remove_group(&core_data->pdev->dev.kobj, &sysfs_group);
}

/* prosfs create */
static int rawdata_proc_show(struct seq_file *m, void *v)
{
	struct ts_rawdata_info *info;
	struct goodix_ts_core *cd;
	int tx;
	int rx;
	int ret;
	int i;
	int index;

	if (!m || !v)
		return -EIO;

	cd = m->private;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = cd->hw_ops->get_capacitance_data(cd, info);
	if (ret < 0) {
		ts_err("failed to get_capacitance_data, exit!");
		goto exit;
	}

	rx = info->buff[0];
	tx = info->buff[1];
	seq_printf(m, "TX:%d  RX:%d\n", tx, rx);
	seq_puts(m, "mutual_rawdata:\n");
	index = 2;
	for (i = 0; i < tx * rx; i++) {
		seq_printf(m, "%5d,", info->buff[index + i]);
		if ((i + 1) % tx == 0)
			seq_puts(m, "\n");
	}
	seq_puts(m, "mutual_diffdata:\n");
	index += tx * rx;
	for (i = 0; i < tx * rx; i++) {
		seq_printf(m, "%3d,", info->buff[index + i]);
		if ((i + 1) % tx == 0)
			seq_puts(m, "\n");
	}

exit:
	kfree(info);
	return ret;
}

static int rawdata_proc_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, rawdata_proc_show,
			PDE_DATA(inode), PAGE_SIZE * 10);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops rawdata_proc_fops = {
	.proc_open = rawdata_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations rawdata_proc_fops = {
	.open = rawdata_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static void goodix_ts_procfs_init(struct goodix_ts_core *core_data)
{
	int dev_id = core_data->pdev->id;
	char proc_node[32] = {0};

	sprintf(proc_node, "goodix_ts.%d", dev_id);
	core_data->proc_dir_entry = proc_mkdir(proc_node, NULL);
	if (!core_data->proc_dir_entry)
		return;

	if (proc_create_data("tp_capacitance_data",
			0664, core_data->proc_dir_entry,
			&rawdata_proc_fops, core_data) == NULL) {
		ts_err("failed to create proc entry");
	}
}

static void goodix_ts_procfs_exit(struct goodix_ts_core *core_data)
{
	int dev_id = core_data->pdev->id;
	char proc_node[32] = {0};

	sprintf(proc_node, "goodix_ts.%d", dev_id);

	remove_proc_entry("tp_capacitance_data", NULL);
	remove_proc_entry(proc_node, NULL);
}

#if IS_ENABLED(CONFIG_OF)
/**
 * goodix_parse_dt_resolution - parse resolution from dt
 * @node: devicetree node
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_parse_dt_resolution(struct device_node *node,
		struct goodix_ts_board_data *board_data)
{
	int ret;

	ret = of_property_read_u32(node, "goodix,panel-max-x",
				 &board_data->panel_max_x);
	if (ret) {
		ts_err("failed get panel-max-x");
		return ret;
	}

	ret = of_property_read_u32(node, "goodix,panel-max-y",
				 &board_data->panel_max_y);
	if (ret) {
		ts_err("failed get panel-max-y");
		return ret;
	}

	ret = of_property_read_u32(node, "goodix,panel-max-w",
				 &board_data->panel_max_w);
	if (ret) {
		ts_err("failed get panel-max-w");
		return ret;
	}

	ret = of_property_read_u32(node, "goodix,panel-max-p",
				 &board_data->panel_max_p);
	if (ret) {
		ts_err("failed get panel-max-p, use default");
		board_data->panel_max_p = GOODIX_PEN_MAX_PRESSURE;
	}

	return 0;
}

/**
 * goodix_parse_dt - parse board data from dt
 * @dev: pointer to device
 * @board_data: pointer to board data structure
 * return: 0 - no error, <0 error
 */
static int goodix_parse_dt(struct device_node *node,
	struct goodix_ts_board_data *board_data)
{
	const char *name_tmp;
	int r;

	if (!board_data) {
		ts_err("invalid board data");
		return -EINVAL;
	}

	r = of_get_named_gpio(node, "goodix,avdd-gpio", 0);
	if (r < 0) {
		ts_info("can't find avdd-gpio, use other power supply");
		board_data->avdd_gpio = 0;
	} else {
		ts_info("get avdd-gpio[%d] from dt", r);
		board_data->avdd_gpio = r;
	}

	r = of_get_named_gpio(node, "goodix,iovdd-gpio", 0);
	if (r < 0) {
		ts_info("can't find iovdd-gpio, use other power supply");
		board_data->iovdd_gpio = 0;
	} else {
		ts_info("get iovdd-gpio[%d] from dt", r);
		board_data->iovdd_gpio = r;
	}

	r = of_get_named_gpio(node, "goodix,reset-gpio", 0);
	if (r < 0) {
		ts_err("invalid reset-gpio in dt: %d", r);
		return -EINVAL;
	}
	ts_info("get reset-gpio[%d] from dt", r);
	board_data->reset_gpio = r;

	r = of_get_named_gpio(node, "goodix,irq-gpio", 0);
	if (r < 0) {
		ts_err("invalid irq-gpio in dt: %d", r);
		return -EINVAL;
	}
	ts_info("get irq-gpio[%d] from dt", r);
	board_data->irq_gpio = r;

	r = of_property_read_u32(node, "goodix,irq-flags",
			&board_data->irq_flags);
	if (r) {
		ts_err("invalid irq-flags");
		return -EINVAL;
	}

	memset(board_data->avdd_name, 0, sizeof(board_data->avdd_name));
	r = of_property_read_string(node, "goodix,avdd-name", &name_tmp);
	if (!r) {
		ts_info("avdd name from dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(board_data->avdd_name))
			strncpy(board_data->avdd_name,
				name_tmp, sizeof(board_data->avdd_name));
		else
			ts_info("invalied avdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(board_data->avdd_name));
	}

	memset(board_data->iovdd_name, 0, sizeof(board_data->iovdd_name));
	r = of_property_read_string(node, "goodix,iovdd-name", &name_tmp);
	if (!r) {
		ts_info("iovdd name from dt: %s", name_tmp);
		if (strlen(name_tmp) < sizeof(board_data->iovdd_name))
			strncpy(board_data->iovdd_name,
				name_tmp, sizeof(board_data->iovdd_name));
		else
			ts_info("invalied iovdd name length: %ld > %ld",
				strlen(name_tmp),
				sizeof(board_data->iovdd_name));
	}

	/* get firmware file name */
	r = of_property_read_string(node, "goodix,firmware-name", &name_tmp);
	if (!r) {
		ts_info("firmware name from dt: %s", name_tmp);
		strncpy(board_data->fw_name,
				name_tmp, sizeof(board_data->fw_name));
	} else {
		ts_info("can't find firmware name, use default: %s",
				TS_DEFAULT_FIRMWARE);
		strncpy(board_data->fw_name,
				TS_DEFAULT_FIRMWARE,
				sizeof(board_data->fw_name));
	}

	/* get config file name */
	r = of_property_read_string(node, "goodix,config-name", &name_tmp);
	if (!r) {
		ts_info("config name from dt: %s", name_tmp);
		strncpy(board_data->cfg_bin_name, name_tmp,
				sizeof(board_data->cfg_bin_name));
	} else {
		ts_info("can't find config name, use default: %s",
				TS_DEFAULT_CFG_BIN);
		strncpy(board_data->cfg_bin_name,
				TS_DEFAULT_CFG_BIN,
				sizeof(board_data->cfg_bin_name));
	}

	/* get xyz resolutions */
	r = goodix_parse_dt_resolution(node, board_data);
	if (r) {
		ts_err("Failed to parse resolutions:%d", r);
		return r;
	}

	/* get sleep mode flag */
	board_data->sleep_enable = of_property_read_bool(node,
			"goodix,sleep-enable");

	/*get pen-enable switch and pen keys, must after "key map"*/
	board_data->pen_enable = of_property_read_bool(node,
			"goodix,pen-enable");

	/* get ic compatible */
	r = of_property_read_string(node, "compatible", &name_tmp);
	if (r) {
		ts_err("get compatible failed");
		return r;
	} else {
	    ts_info("ic_name form dt: %s", name_tmp);
	    strncpy(board_data->ic_name, name_tmp, sizeof(board_data->ic_name));
	}

	ts_info("[DT]x:%d, y:%d, w:%d, p:%d sleep_enable:%d pen_enable:%d",
		board_data->panel_max_x, board_data->panel_max_y,
		board_data->panel_max_w, board_data->panel_max_p,
		board_data->sleep_enable, board_data->pen_enable);
	return 0;
}
#endif

static void goodix_ts_report_pen(struct input_dev *dev,
		struct goodix_pen_data *pen_data)
{
	int i;

	mutex_lock(&dev->mutex);

	if (pen_data->coords.status == TS_TOUCH) {
		input_report_key(dev, BTN_TOUCH, pen_data->is_hover ? 0 : 1);
		input_report_key(dev, BTN_TOOL_PEN, 1);
		input_report_abs(dev, ABS_X, pen_data->coords.x);
		input_report_abs(dev, ABS_Y, pen_data->coords.y);
		input_report_abs(dev, ABS_PRESSURE, pen_data->coords.p);
		if (pen_data->coords.p == 0)
			input_report_abs(dev, ABS_DISTANCE, 1);
		else
			input_report_abs(dev, ABS_DISTANCE, 0);
		input_report_abs(dev, ABS_TILT_X, pen_data->coords.tilt_x);
		input_report_abs(dev, ABS_TILT_Y, pen_data->coords.tilt_y);
		ts_debug("pen_data:x %d, y %d, p %d, tilt_x %d tilt_y %d key[%d %d]",
				pen_data->coords.x, pen_data->coords.y,
				pen_data->coords.p, pen_data->coords.tilt_x,
				pen_data->coords.tilt_y,
				pen_data->keys[0].status == TS_TOUCH ? 1 : 0,
				pen_data->keys[1].status == TS_TOUCH ? 1 : 0);
	} else {
		input_report_key(dev, BTN_TOUCH, 0);
		input_report_key(dev, BTN_TOOL_PEN, 0);
	}
	/* report pen button */
	for (i = 0; i < GOODIX_MAX_PEN_KEY; i++) {
		if (pen_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, pen_data->keys[i].code, 1);
		else
			input_report_key(dev, pen_data->keys[i].code, 0);
	}

	input_sync(dev);
	mutex_unlock(&dev->mutex);
}

static void goodix_ts_report_finger(struct input_dev *dev,
		struct goodix_touch_data *touch_data)
{
	unsigned int touch_num = touch_data->touch_num;
	int i;

	mutex_lock(&dev->mutex);

	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		if (touch_data->coords[i].status == TS_TOUCH) {
			ts_debug("report: id[%d], x %d, y %d, w %d", i,
				touch_data->coords[i].x,
				touch_data->coords[i].y,
				touch_data->coords[i].w);
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);
			input_report_abs(dev, ABS_MT_POSITION_X,
					touch_data->coords[i].x);
			input_report_abs(dev, ABS_MT_POSITION_Y,
					touch_data->coords[i].y);
			input_report_abs(dev, ABS_MT_TOUCH_MAJOR,
					touch_data->coords[i].w);
		} else {
			input_mt_slot(dev, i);
			input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
		}
	}

	for (i = 0; i < GOODIX_MAX_KEY; i++) {
		if (touch_data->keys[i].status == TS_TOUCH)
			input_report_key(dev, touch_data->keys[i].code, 1);
		else
			input_report_key(dev, touch_data->keys[i].code, 0);
	}

	input_report_key(dev, BTN_TOUCH, touch_num > 0 ? 1 : 0);
	input_sync(dev);

	mutex_unlock(&dev->mutex);
}

static int goodix_ts_request_handle(struct goodix_ts_core *cd,
	struct goodix_ts_event *ts_event)
{
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int ret = -1;

	if (ts_event->request_code == REQUEST_TYPE_CONFIG)
		ret = goodix_send_ic_config(cd, CONFIG_TYPE_NORMAL);
	else if (ts_event->request_code == REQUEST_TYPE_RESET)
		ret = hw_ops->reset(cd, GOODIX_NORMAL_RESET_DELAY_MS);
	else
		ts_info("can not handle request type 0x%x",
			  ts_event->request_code);
	if (ret)
		ts_err("failed handle request 0x%x",
			 ts_event->request_code);
	else
		ts_info("success handle ic request 0x%x",
			  ts_event->request_code);
	return ret;
}

/**
 * goodix_ts_threadirq_func - Bottom half of interrupt
 * This functions is excuted in thread context,
 * sleep in this function is permit.
 *
 * @data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static irqreturn_t goodix_ts_threadirq_func(int irq, void *data)
{
	struct goodix_ts_core *core_data = data;
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ts_event *ts_event = &core_data->ts_event;
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;
	int ret;

	disable_irq_nosync(core_data->irq);
	ts_esd->irq_status = true;
	core_data->irq_trig_cnt++;

	/* read touch data from touch device */
	ret = hw_ops->event_handler(core_data, ts_event);
	if (likely(!ret)) {
		if (ts_event->event_type & EVENT_TOUCH) {
			/* report touch */
			goodix_ts_report_finger(core_data->input_dev,
					&ts_event->touch_data);
		}
		if (core_data->board_data.pen_enable &&
				ts_event->event_type & EVENT_PEN) {
			goodix_ts_report_pen(core_data->pen_dev,
					&ts_event->pen_data);
		}
		if (ts_event->event_type & EVENT_REQUEST)
			goodix_ts_request_handle(core_data, ts_event);
		if (ts_event->event_type & EVENT_GESTURE)
			goodix_ts_report_gesture(core_data, ts_event);
	}

	enable_irq(core_data->irq);
	return IRQ_HANDLED;
}

/**
 * goodix_ts_init_irq - Requset interrput line from system
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_irq_setup(struct goodix_ts_core *core_data)
{
	const struct goodix_ts_board_data *ts_bdata = &core_data->board_data;
	int ret;

	/* if ts_bdata-> irq is invalid */
	core_data->irq = gpio_to_irq(ts_bdata->irq_gpio);
	if (core_data->irq < 0) {
		ts_err("failed get irq num %d", core_data->irq);
		return -EINVAL;
	}

	ts_info("IRQ:%u,flags:%d", core_data->irq, (int)ts_bdata->irq_flags);
	ret = devm_request_threaded_irq(&core_data->pdev->dev,
				      core_data->irq, NULL,
				      goodix_ts_threadirq_func,
				      ts_bdata->irq_flags | IRQF_ONESHOT,
				      GOODIX_CORE_DRIVER_NAME,
				      core_data);
	if (ret < 0)
		ts_err("Failed to requeset threaded irq:%d", ret);
	else
		atomic_set(&core_data->irq_enabled, 1);

	return ret;
}

/**
 * goodix_ts_power_init - Get regulator for touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_power_init(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = &core_data->board_data;
	struct device *dev = core_data->bus->dev;
	int ret = 0;

	ts_info("Power init");
	if (strlen(ts_bdata->avdd_name)) {
		core_data->avdd = devm_regulator_get(dev,
				 ts_bdata->avdd_name);
		if (IS_ERR_OR_NULL(core_data->avdd)) {
			ret = PTR_ERR(core_data->avdd);
			ts_err("Failed to get regulator avdd:%d", ret);
			core_data->avdd = NULL;
			return ret;
		}
	} else {
		ts_info("Avdd name is NULL");
	}

	if (strlen(ts_bdata->iovdd_name)) {
		core_data->iovdd = devm_regulator_get(dev,
				 ts_bdata->iovdd_name);
		if (IS_ERR_OR_NULL(core_data->iovdd)) {
			ret = PTR_ERR(core_data->iovdd);
			ts_err("Failed to get regulator iovdd:%d", ret);
			core_data->iovdd = NULL;
		}
	} else {
		ts_info("iovdd name is NULL");
	}

	return ret;
}

/**
 * goodix_ts_power_on - Turn on power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_on(struct goodix_ts_core *cd)
{
	int ret = 0;

	ts_info("Device power on");
	if (cd->power_on)
		return 0;

	ret = cd->hw_ops->power_on(cd, true);
	if (!ret)
		cd->power_on = 1;
	else
		ts_err("failed power on, %d", ret);
	return ret;
}

/**
 * goodix_ts_power_off - Turn off power to the touch device
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
int goodix_ts_power_off(struct goodix_ts_core *cd)
{
	int ret;

	if (!cd->power_on)
		return 0;

	ts_info("Device power off");
	ret = cd->hw_ops->power_on(cd, false);
	if (!ret)
		cd->power_on = 0;
	else
		ts_err("failed power off, %d", ret);

	return ret;
}

/**
 * goodix_ts_pinctrl_init - Get pinctrl handler and pinctrl_state
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_pinctrl_init(struct goodix_ts_core *core_data)
{
	int r = 0;

	/* get pinctrl handler from of node */
	core_data->pinctrl = devm_pinctrl_get(core_data->bus->dev);
	if (IS_ERR_OR_NULL(core_data->pinctrl)) {
		ts_info("Failed to get pinctrl handler[need confirm]");
		core_data->pinctrl = NULL;
		return -EINVAL;
	}
	ts_debug("success get pinctrl");
	/* active state */
	core_data->pin_sta_active = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(core_data->pin_sta_active)) {
		r = PTR_ERR(core_data->pin_sta_active);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_ACTIVE, r);
		core_data->pin_sta_active = NULL;
		goto exit_pinctrl_put;
	}
	ts_info("success get avtive pinctrl state");

	/* suspend state */
	core_data->pin_sta_suspend = pinctrl_lookup_state(core_data->pinctrl,
				PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(core_data->pin_sta_suspend)) {
		r = PTR_ERR(core_data->pin_sta_suspend);
		ts_err("Failed to get pinctrl state:%s, r:%d",
				PINCTRL_STATE_SUSPEND, r);
		core_data->pin_sta_suspend = NULL;
		goto exit_pinctrl_put;
	}
	ts_info("success get suspend pinctrl state");

	return 0;
exit_pinctrl_put:
	devm_pinctrl_put(core_data->pinctrl);
	core_data->pinctrl = NULL;
	return r;
}

/**
 * goodix_ts_gpio_setup - Request gpio resources from GPIO subsysten
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_gpio_setup(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = &core_data->board_data;
	int r = 0;

	ts_info("GPIO setup,reset-gpio:%d, irq-gpio:%d",
		ts_bdata->reset_gpio, ts_bdata->irq_gpio);
	/*
	 * after kenerl3.13, gpio_ api is deprecated, new
	 * driver should use gpiod_ api.
	 */
	r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->reset_gpio,
			GPIOF_OUT_INIT_LOW, "ts_reset_gpio");
	if (r < 0) {
		ts_err("Failed to request reset gpio, r:%d", r);
		return r;
	}

	r = devm_gpio_request_one(&core_data->pdev->dev,
			ts_bdata->irq_gpio,
			GPIOF_IN, "ts_irq_gpio");
	if (r < 0) {
		ts_err("Failed to request irq gpio, r:%d", r);
		return r;
	}

	if (ts_bdata->avdd_gpio > 0) {
		r = devm_gpio_request_one(&core_data->pdev->dev,
				ts_bdata->avdd_gpio,
				GPIOF_OUT_INIT_LOW, "ts_avdd_gpio");
		if (r < 0) {
			ts_err("Failed to request avdd-gpio, r:%d", r);
			return r;
		}
	}

	if (ts_bdata->iovdd_gpio > 0) {
		r = devm_gpio_request_one(&core_data->pdev->dev,
				ts_bdata->iovdd_gpio,
				GPIOF_OUT_INIT_LOW, "ts_iovdd_gpio");
		if (r < 0) {
			ts_err("Failed to request iovdd-gpio, r:%d", r);
			return r;
		}
	}

	return 0;
}

/**
 * goodix_ts_input_dev_config - Requset and config a input device
 *  then register it to input sybsystem.
 * @core_data: pointer to touch core data
 * return: 0 ok, <0 failed
 */
static int goodix_ts_input_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = &core_data->board_data;
	struct input_dev *input_dev = NULL;
	int dev_id = core_data->pdev->id;
	int r;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ts_err("Failed to allocated input device");
		return -ENOMEM;
	}

	core_data->input_dev = input_dev;
	input_set_drvdata(input_dev, core_data);

	sprintf(core_data->input_name, "%s_%d", GOODIX_CORE_DRIVER_NAME, dev_id);

	input_dev->name = core_data->input_name;
	input_dev->id.bustype = core_data->bus->bus_type;
	input_dev->id.product = 0x0100 + dev_id;
	input_dev->id.vendor = 0x27C6;
	input_dev->id.version = 0x0100;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	/* set input parameters */
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ts_bdata->panel_max_x - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ts_bdata->panel_max_y - 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, ts_bdata->panel_max_w - 1, 0, 0);
#ifdef INPUT_TYPE_B_PROTOCOL
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0)
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH,
			    INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GOODIX_MAX_TOUCH);
#endif
#endif

	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(input_dev, EV_KEY, KEY_GOTO);

	r = input_register_device(input_dev);
	if (r < 0) {
		ts_err("Unable to register input device");
		input_free_device(input_dev);
		return r;
	}

	return 0;
}

static int goodix_ts_pen_dev_config(struct goodix_ts_core *core_data)
{
	struct goodix_ts_board_data *ts_bdata = &core_data->board_data;
	struct input_dev *pen_dev = NULL;
	int dev_id = core_data->pdev->id;
	int r;

	pen_dev = input_allocate_device();
	if (!pen_dev) {
		ts_err("Failed to allocated pen device");
		return -ENOMEM;
	}

	core_data->pen_dev = pen_dev;
	input_set_drvdata(pen_dev, core_data);

	sprintf(core_data->input_pen_name, "%s.%d,%s", GOODIX_CORE_DRIVER_NAME, dev_id, "pen");
	pen_dev->name = core_data->input_pen_name;
	pen_dev->id.bustype = core_data->bus->bus_type;
	pen_dev->id.product = 0x0200 + dev_id;
	pen_dev->id.vendor = 0x27C6;
	pen_dev->id.version = 0x0100;

	pen_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	set_bit(ABS_X, pen_dev->absbit);
	set_bit(ABS_Y, pen_dev->absbit);
	set_bit(ABS_TILT_X, pen_dev->absbit);
	set_bit(ABS_TILT_Y, pen_dev->absbit);
	set_bit(BTN_STYLUS, pen_dev->keybit);
	set_bit(BTN_STYLUS2, pen_dev->keybit);
	set_bit(BTN_TOUCH, pen_dev->keybit);
	set_bit(BTN_TOOL_PEN, pen_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, pen_dev->propbit);
	input_set_abs_params(pen_dev, ABS_X, 0, ts_bdata->panel_max_x - 1, 0, 0);
	input_set_abs_params(pen_dev, ABS_Y, 0, ts_bdata->panel_max_y - 1, 0, 0);
	input_set_abs_params(pen_dev, ABS_PRESSURE, 0,
			     ts_bdata->panel_max_p - 1, 0, 0);
	input_set_abs_params(pen_dev, ABS_DISTANCE, 0, 255, 0, 0);
	input_set_abs_params(pen_dev, ABS_TILT_X,
			-GOODIX_PEN_MAX_TILT, GOODIX_PEN_MAX_TILT, 0, 0);
	input_set_abs_params(pen_dev, ABS_TILT_Y,
			-GOODIX_PEN_MAX_TILT, GOODIX_PEN_MAX_TILT, 0, 0);

	r = input_register_device(pen_dev);
	if (r < 0) {
		ts_err("Unable to register pen device");
		input_free_device(pen_dev);
		return r;
	}

	return 0;
}

void goodix_ts_input_dev_remove(struct goodix_ts_core *core_data)
{
	if (!core_data->input_dev)
		return;
	input_unregister_device(core_data->input_dev);
	input_free_device(core_data->input_dev);
	core_data->input_dev = NULL;
}

void goodix_ts_pen_dev_remove(struct goodix_ts_core *core_data)
{
	if (!core_data->pen_dev)
		return;
	input_unregister_device(core_data->pen_dev);
	input_free_device(core_data->pen_dev);
	core_data->pen_dev = NULL;
}

/**
 * goodix_ts_esd_work - check hardware status and recovery
 *  the hardware if needed.
 */
static void goodix_ts_esd_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct goodix_ts_esd *ts_esd = container_of(dwork,
			struct goodix_ts_esd, esd_work);
	struct goodix_ts_core *cd = container_of(ts_esd,
			struct goodix_ts_core, ts_esd);
	const struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;
	int ret = 0;

	if (ts_esd->irq_status)
		goto exit;

	if (!atomic_read(&ts_esd->esd_on))
		return;

	if (!hw_ops->esd_check)
		return;

	ret = hw_ops->esd_check(cd);
	if (ret) {
		ts_err("esd check failed");
		goodix_ts_power_off(cd);
		usleep_range(5000, 5100);
		goodix_ts_power_on(cd);
	}

exit:
	ts_esd->irq_status = false;
	if (atomic_read(&ts_esd->esd_on))
		schedule_delayed_work(&ts_esd->esd_work, 2 * HZ);
}

/**
 * goodix_ts_esd_on - turn on esd protection
 */
void goodix_ts_esd_on(struct goodix_ts_core *cd)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!misc->esd_addr)
		return;

	if (atomic_read(&ts_esd->esd_on))
		return;

	atomic_set(&ts_esd->esd_on, 1);
	if (!schedule_delayed_work(&ts_esd->esd_work, 2 * HZ))
		ts_info("esd work already in workqueue");

	ts_info("esd on");
}

/**
 * goodix_ts_esd_off - turn off esd protection
 */
void goodix_ts_esd_off(struct goodix_ts_core *cd)
{
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!atomic_read(&ts_esd->esd_on))
		return;

	atomic_set(&ts_esd->esd_on, 0);
	cancel_delayed_work_sync(&ts_esd->esd_work);
	ts_info("Esd off");
}

/**
 * goodix_ts_esd_init - initialize esd protection
 */
int goodix_ts_esd_init(struct goodix_ts_core *cd)
{
	struct goodix_ic_info_misc *misc = &cd->ic_info.misc;
	struct goodix_ts_esd *ts_esd = &cd->ts_esd;

	if (!cd->hw_ops->esd_check || !misc->esd_addr) {
		ts_info("missing key info for esd check");
		return 0;
	}

	INIT_DELAYED_WORK(&ts_esd->esd_work, goodix_ts_esd_work);
	ts_esd->ts_core = cd;
	atomic_set(&ts_esd->esd_on, 0);
	goodix_ts_esd_on(cd);

	return 0;
}

void goodix_ts_release_connects(struct goodix_ts_core *core_data)
{
	struct input_dev *input_dev = core_data->input_dev;
	struct input_dev *pen_dev = core_data->pen_dev;
	int i;

	mutex_lock(&input_dev->mutex);
	for (i = 0; i < GOODIX_MAX_TOUCH; i++) {
		input_mt_slot(input_dev, i);
		input_mt_report_slot_state(input_dev,
				MT_TOOL_FINGER,
				false);
	}
	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync_frame(input_dev);
	input_sync(input_dev);
	mutex_unlock(&input_dev->mutex);

	if (core_data->board_data.pen_enable) {
		mutex_lock(&pen_dev->mutex);
		input_report_key(pen_dev, BTN_TOUCH, 0);
		input_report_key(pen_dev, BTN_TOOL_PEN, 0);
		input_sync(pen_dev);
		mutex_unlock(&pen_dev->mutex);
	}

	if (core_data->gesture_type)
		core_data->hw_ops->after_event_handler(core_data);
}

#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
/**
 * goodix_ts_suspend - Touchscreen suspend function
 * Called by PM/FB/EARLYSUSPEN module to put the device to sleep
 */
static int goodix_ts_suspend(struct goodix_ts_core *core_data)
{
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (core_data->init_stage < CORE_INIT_STAGE2 ||
			atomic_read(&core_data->suspended))
		return 0;

	ts_info("Suspend start");
	atomic_set(&core_data->suspended, 1);
	/* disable irq */
	hw_ops->irq_enable(core_data, false);
	goodix_ts_esd_off(core_data);

	if (core_data->gesture_type) {
		/* enter gesture mode */
		ts_info("enter gesture mode[0x%x]", core_data->gesture_type);
		// TODO: send 0 means enable all gesture type
		hw_ops->gesture(core_data, 0);
		hw_ops->irq_enable(core_data, true);
		enable_irq_wake(core_data->irq);
	} else {
		/* enter sleep mode or power off */
		if (core_data->board_data.sleep_enable)
			hw_ops->suspend(core_data);
		else
			goodix_ts_power_off(core_data);
	}

	goodix_ts_release_connects(core_data);
	ts_info("Suspend end");
	return 0;
}

/**
 * goodix_ts_resume - Touchscreen resume function
 * Called by PM/FB/EARLYSUSPEN module to wakeup device
 */
static int goodix_ts_resume(struct goodix_ts_core *core_data)
{
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;

	if (core_data->init_stage < CORE_INIT_STAGE2 ||
			!atomic_read(&core_data->suspended))
		return 0;

	ts_info("Resume start");
	atomic_set(&core_data->suspended, 0);
	hw_ops->irq_enable(core_data, false);

	if (core_data->gesture_type) {
		disable_irq_wake(core_data->irq);
		hw_ops->resume(core_data);
	} else {
		/* reset device or power on*/
		if (core_data->board_data.sleep_enable)
			hw_ops->resume(core_data);
		else
			goodix_ts_power_on(core_data);
	}

	/* enable irq */
	hw_ops->irq_enable(core_data, true);
	/* open esd */
	goodix_ts_esd_on(core_data);
	ts_info("Resume end");
	return 0;
}

static void goodix_ts_resume_work(struct work_struct *work)
{
	struct goodix_ts_core *cd =
			container_of(work, struct goodix_ts_core, resume_work);
	goodix_ts_resume(cd);
}
#endif

#if IS_ENABLED(CONFIG_FB)
/**
 * goodix_ts_fb_notifier_callback - Framebuffer notifier callback
 * Called by kernel during framebuffer blanck/unblank phrase
 */
int goodix_ts_fb_notifier_callback(struct notifier_block *self,
	unsigned long event, void *data)
{
	struct goodix_ts_core *core_data =
		container_of(self, struct goodix_ts_core, fb_notifier);
	struct fb_event *fb_event = data;
	int *blank;

	blank = fb_event->data;
	if (fb_event && fb_event->data && core_data) {
		if (event == FB_EVENT_BLANK) {
			if (*blank == FB_BLANK_UNBLANK) {
				schedule_work(&core_data->resume_work);
			} else if (*blank == FB_BLANK_POWERDOWN) {
				cancel_work_sync(&core_data->resume_work);
				goodix_ts_suspend(core_data);
			}
		}
	}

	return 0;
}
#elif IS_ENABLED(CONFIG_PM) && !defined(CONFIG_INPUT_TOUCHSCREEN_MMI)
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_suspend(core_data);
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	struct goodix_ts_core *core_data =
		dev_get_drvdata(dev);

	return goodix_ts_resume(core_data);
}
static const struct dev_pm_ops dev_pm_ops = {
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
};
#else
/**
 * goodix_ts_pm_suspend - PM suspend function
 * Called by kernel during system suspend phrase
 */
static int goodix_ts_pm_suspend(struct device *dev)
{
	ts_info("system enters into pm_suspend");
	return 0;
}
/**
 * goodix_ts_pm_resume - PM resume function
 * Called by kernel during system wakeup
 */
static int goodix_ts_pm_resume(struct device *dev)
{
	ts_info("system resumes from pm_suspend");
	return 0;
}
static const struct dev_pm_ops dev_pm_ops = {
	.suspend = goodix_ts_pm_suspend,
	.resume = goodix_ts_pm_resume,
};
#endif

static void goodix_self_check(struct work_struct *work)
{
	struct goodix_ts_core *cd =
			container_of(work, struct goodix_ts_core, self_check_work);
	u32 fw_state_addr = cd->ic_info.misc.fw_state_addr;
	int update_flag = UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST | UPDATE_MODE_FORCE;
	u8 cur_cycle_cnt = 0;
	u8 pre_cycle_cnt = 0;
	int err_cnt = 0;
	int retry = 5;

	while (retry--) {
		cd->hw_ops->read(cd, fw_state_addr, &cur_cycle_cnt, 1);
		if (cur_cycle_cnt == pre_cycle_cnt)
			err_cnt++;
		pre_cycle_cnt = cur_cycle_cnt;
		msleep(20);
	}
	if (err_cnt > 1) {
		ts_err("Warning! The firmware maybe running abnormal, need upgrade.");
		goodix_do_fw_update(cd, update_flag);
	}
}

int goodix_ts_stage2_init(struct goodix_ts_core *cd)
{
	int ret;

	/* alloc/config/register input device */
	ret = goodix_ts_input_dev_config(cd);
	if (ret < 0) {
		ts_err("failed set input device");
		return ret;
	}

	if (cd->board_data.pen_enable) {
		ret = goodix_ts_pen_dev_config(cd);
		if (ret < 0) {
			ts_err("failed set pen device");
			goto err_finger;
		}
	}
	/* request irq line */
	ret = goodix_ts_irq_setup(cd);
	if (ret < 0) {
		ts_info("failed set irq");
		goto exit;
	}
	ts_info("success register irq");

#if IS_ENABLED(CONFIG_FB)
	cd->fb_notifier.notifier_call = goodix_ts_fb_notifier_callback;
	if (fb_register_client(&cd->fb_notifier))
		ts_err("Failed to register fb notifier client:%d", ret);
#endif
	/* create sysfs files */
	goodix_ts_sysfs_init(cd);

	/* create procfs files */
	goodix_ts_procfs_init(cd);

	/* esd protector */
	goodix_ts_esd_init(cd);

	/* gesture init */
	gesture_module_init(cd);

	/* inspect init */
	inspect_module_init(cd);

#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
	INIT_WORK(&cd->resume_work, goodix_ts_resume_work);
#endif

	/* Do self check on first boot */
	INIT_WORK(&cd->self_check_work, goodix_self_check);
	schedule_work(&cd->self_check_work);

	return 0;
exit:
	goodix_ts_pen_dev_remove(cd);
err_finger:
	goodix_ts_input_dev_remove(cd);
	return ret;
}

/* try send the config specified with type */
static int goodix_send_ic_config(struct goodix_ts_core *cd, int type)
{
	u32 config_id;
	struct goodix_ic_config *cfg;

	if (type >= GOODIX_MAX_CONFIG_GROUP) {
		ts_err("unsupproted config type %d", type);
		return -EINVAL;
	}

	cfg = &cd->ic_configs[type];
	if (cfg->len <= 0) {
		ts_info("no valid normal config found");
		return -EINVAL;
	}

	config_id = goodix_get_file_config_id(cfg->data);
	if (cd->ic_info.version.config_id == config_id) {
		ts_info("config id is equal 0x%x, skiped", config_id);
		return 0;
	}

	ts_info("try send config, id=0x%x", config_id);
	return cd->hw_ops->send_config(cd, cfg->data, cfg->len);
}

/**
 * goodix_later_init_thread - init IC fw and config
 * @data: point to goodix_ts_core
 *
 * This function respond for get fw version and try upgrade fw and config.
 * Note: when init encounter error, need release all resource allocated here.
 */
static int goodix_later_init_thread(void *data)
{
	int ret;
	int update_flag = UPDATE_MODE_BLOCK | UPDATE_MODE_SRC_REQUEST;
	struct goodix_ts_core *cd = data;
	struct goodix_ts_hw_ops *hw_ops = cd->hw_ops;

	/* step 1: read version */
	ret = hw_ops->read_version(cd, &cd->fw_version);
	if (ret < 0) {
		ts_err("failed to get version info, try to upgrade");
		update_flag |= UPDATE_MODE_FORCE;
	}

	/* step 2: read ic info */
	ret = hw_ops->get_ic_info(cd, &cd->ic_info);
	if (ret < 0) {
		ts_err("failed to get ic info, try to upgrade");
		update_flag |= UPDATE_MODE_FORCE;
	}

	/* step 3: do upgrade */
	ts_info("update flag: 0x%X", update_flag);
	goodix_do_fw_update(cd, update_flag);

	print_ic_info(&cd->ic_info);

	/* the recomend way to update ic config is throuth ISP,
	 * if not we will send config with interactive mode
	 */
	goodix_send_ic_config(cd, CONFIG_TYPE_NORMAL);

	/* init other resources */
	ret = goodix_ts_stage2_init(cd);
	if (ret) {
		ts_err("stage2 init failed");
		goto err_out;
	}

	cd->init_stage = CORE_INIT_STAGE2;
	return 0;
err_out:
	cd->init_stage = CORE_INIT_FAIL;
	return ret;
}

static int goodix_start_later_init(struct goodix_ts_core *ts_core)
{
	struct task_struct *init_thrd;
	/* create and run update thread */
	init_thrd = kthread_run(goodix_later_init_thread,
				ts_core, "goodix_init_thread");
	if (IS_ERR_OR_NULL(init_thrd)) {
		ts_err("Failed to create update thread:%ld",
		       PTR_ERR(init_thrd));
		return -EFAULT;
	}
	return 0;
}

/* goodix fb test */
// static void test_suspend(void *data)
// {
// 	struct goodix_ts_core *cd = data;
// 	cancel_work_sync(&cd->resume_work);
// 	goodix_ts_suspend(cd);
// }

// static void test_resume(void *data)
// {
// 	struct goodix_ts_core *cd = data;
// 	schedule_work(&cd->resume_work);
// }

/**
 * goodix_ts_probe - called by kernel when Goodix touch
 *  platform driver is added.
 */
static int goodix_ts_probe(struct platform_device *pdev)
{
	struct goodix_device_resource *dev_res =
			container_of(pdev, struct goodix_device_resource, pdev);
	struct goodix_ts_core *core_data;
	struct goodix_bus_interface *bus_interface;
	int ret;

	ts_info("IN");
	core_data = &dev_res->core_data;
	bus_interface = &dev_res->bus;

	if (IS_ENABLED(CONFIG_OF) && bus_interface->dev->of_node) {
		/* parse devicetree property */
		ret = goodix_parse_dt(bus_interface->dev->of_node,
					&core_data->board_data);
		if (ret) {
			ts_err("failed parse device info form dts, %d", ret);
			return -EINVAL;
		}
	} else {
		ts_err("no valid device tree node found");
		return -ENODEV;
	}

	core_data->hw_ops = goodix_get_hw_ops();
	if (!core_data->hw_ops) {
		ts_err("hw ops is NULL");
		return -EINVAL;
	}

	/* touch core layer is a platform driver */
	core_data->pdev = pdev;
	core_data->bus = bus_interface;
	platform_set_drvdata(pdev, core_data);

	/* get GPIO resource */
	ret = goodix_ts_gpio_setup(core_data);
	if (ret) {
		ts_err("failed init gpio");
		goto err_out;
	}

	ret = goodix_ts_power_init(core_data);
	if (ret) {
		ts_err("failed init power");
		goto err_out;
	}

	ret = goodix_ts_power_on(core_data);
	if (ret) {
		ts_err("failed power on");
		goto err_out;
	}

	/* update modue init */
	ret = goodix_fw_update_init(core_data);
	if (ret) {
		ts_err("firmware update module init failed");
		goto err_out;
	}

	/* Pinctrl handle is optional. */
	ret = goodix_ts_pinctrl_init(core_data);
	if (!ret && core_data->pinctrl) {
		ret = pinctrl_select_state(core_data->pinctrl,
					 core_data->pin_sta_active);
		if (ret < 0)
			ts_err("Failed to select active pinstate, r:%d", ret);
	}

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	ts_info("%s:goodix_ts_mmi_dev_register",__func__);
	ret = goodix_ts_mmi_dev_register(pdev);
	if (ret) {
		ts_info("Failed register touchscreen mmi.");
		goto err_out;
	}
#endif

	/* debug node init */
	ret = goodix_tools_init(core_data);
	if (ret) {
		ts_err("debug tools init failed");
		goto err_out;
	}

	/* goodix fb test */
	// fb_firefly_register(test_suspend, test_resume, core_data);

	core_data->init_stage = CORE_INIT_STAGE1;

	/* Try start a thread to get config-bin info */
	goodix_start_later_init(core_data);

	mutex_init(&core_data->cmd_lock);
	ts_info("goodix_ts_core probe success");
	return 0;

err_out:
	goodix_fw_update_uninit(core_data);
	goodix_ts_power_off(core_data);
	core_data->init_stage = CORE_INIT_FAIL;
	ts_err("goodix_ts_core failed, ret:%d", ret);
	return ret;
}

static int goodix_ts_remove(struct platform_device *pdev)
{
	struct goodix_ts_core *core_data = platform_get_drvdata(pdev);
	struct goodix_ts_hw_ops *hw_ops = core_data->hw_ops;
	struct goodix_ts_esd *ts_esd = &core_data->ts_esd;

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	ts_info("%s:goodix_ts_mmi_dev_unregister",__func__);
	goodix_ts_mmi_dev_unregister(pdev);
#endif

	goodix_tools_exit(core_data);
	goodix_fw_update_uninit(core_data);

	if (core_data->init_stage >= CORE_INIT_STAGE2) {
		gesture_module_exit(core_data);
		inspect_module_exit(core_data);
		hw_ops->irq_enable(core_data, false);
	#if IS_ENABLED(CONFIG_FB)
		fb_unregister_client(&core_data->fb_notifier);
	#endif
		if (atomic_read(&ts_esd->esd_on))
			goodix_ts_esd_off(core_data);

		goodix_ts_input_dev_remove(core_data);
		goodix_ts_pen_dev_remove(core_data);
		goodix_ts_sysfs_exit(core_data);
		goodix_ts_procfs_exit(core_data);
		goodix_ts_power_off(core_data);
	}

	return 0;
}

static const struct platform_device_id ts_core_ids[] = {
	{.name = GOODIX_CORE_DRIVER_NAME},
	{}
};
MODULE_DEVICE_TABLE(platform, ts_core_ids);

static struct platform_driver goodix_ts_driver = {
	.driver = {
		.name = GOODIX_CORE_DRIVER_NAME,
		.owner = THIS_MODULE,
#if IS_ENABLED(CONFIG_PM) && !IS_ENABLED(CONFIG_FB)
		.pm = &dev_pm_ops,
#endif
	},
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
	.id_table = ts_core_ids,
};

static int __init goodix_ts_core_init(void)
{
	int ret;

	ts_info("Core layer init:%s", GOODIX_DRIVER_VERSION);
	goodix_device_manager_init();
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI
	ret = goodix_spi_bus_init();
	if (ret < 0) {
		ts_err("failed to add spi bus driver");
		return ret;
	}
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_I2C
	ret = goodix_i2c_bus_init();
	if (ret < 0) {
		ts_err("failed to add i2c bus driver");
		return ret;
	}
#endif
	return platform_driver_register(&goodix_ts_driver);
}

static void __exit goodix_ts_core_exit(void)
{
	ts_info("Core layer exit");
	platform_driver_unregister(&goodix_ts_driver);
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_SPI
	goodix_spi_bus_exit();
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BRL_I2C
	goodix_i2c_bus_exit();
#endif
	goodix_device_manager_exit();
}

late_initcall(goodix_ts_core_init);
module_exit(goodix_ts_core_exit);

MODULE_DESCRIPTION("Goodix Touchscreen Driver Module");
MODULE_AUTHOR("Goodix, Inc.");
MODULE_LICENSE("GPL v2");
