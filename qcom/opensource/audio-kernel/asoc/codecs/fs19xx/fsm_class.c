/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2020-01-20 File created.
 */

#include "fsm_public.h"
#if defined(CONFIG_FSM_SYSFS)
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/version.h>


static int g_fsm_class_inited = 0;

#ifdef CONFIG_FSM_Q6AFE
static int fsm_check_dev_index(int ndev, int index)
{
	struct fsm_dev *fsm_dev;
	int dev_idx;
	int i;

	for (i = 0; i < ndev; i++) {
		fsm_dev = fsm_get_fsm_dev_by_id(i);
		if (fsm_dev == NULL)
			continue;
		dev_idx = fsm_get_index_by_position(fsm_dev->pos_mask);
		if (index == dev_idx)
			return 0;
	}

	return -EINVAL;
}

static ssize_t fsm_re25_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	fsm_config_t *cfg = fsm_get_config();
	struct fsadsp_cmd_re25 cmd_re25;
	int i, re25, size;
	int ret;

	cfg->force_calib = true;
	fsm_set_calib_mode();
	fsm_delay_ms(2500);

	memset(&cmd_re25, 0, sizeof(struct fsadsp_cmd_re25));
	ret = fsm_afe_save_re25(&cmd_re25);
	if (ret) {
		pr_err("save re25 failed:%d", ret);
		cfg->force_calib = false;
		return ret;
	}

	for (i = 0, size = 0; i < cmd_re25.ndev; i++) {
		re25 = cmd_re25.cal_data[i].re25;
		size += scnprintf(buf + size, PAGE_SIZE, "%d,", re25);
	}
	buf[size - 1] = '\n';
	cfg->force_calib = false;

	return size;
}

static ssize_t fsm_f0_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	int payload[FSM_CALIB_PAYLOAD_SIZE];
	struct preset_file *pfile;
	struct fsm_afe afe;
	int i, f0, size;
	int ret;

	// fsm_set_calib_mode();
	// fsm_delay_ms(5000);
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_CALIB;
	afe.op_set = false;
	ret = fsm_afe_send_apr(&afe, payload, sizeof(payload));
	if (ret) {
		pr_err("send apr failed:%d", ret);
		return ret;
	}
	pfile = fsm_get_presets();
	if (!pfile) {
		pr_debug("not found firmware");
		return -EINVAL;
	}

	for (i = 0, size = 0; i < pfile->hdr.ndev; i++) {
		f0 = (fsm_check_dev_index(pfile->hdr.ndev, i) == 0) ?
			payload[3+6*i] : -65535;
		size += scnprintf(buf + size, PAGE_SIZE, "%d,", f0);
	}
	buf[size - 1] = '\n';

	return size;
}
#endif

static ssize_t fsm_info_show(struct class *class,
				struct class_attribute *attr, char *buf)
{
	fsm_version_t version;
	struct preset_file *pfile;
	int dev_count;
	int len = 0;

	fsm_get_version(&version);
	len  = scnprintf(buf + len, PAGE_SIZE, "version: %s\n",
			version.code_version);
	len += scnprintf(buf + len, PAGE_SIZE, "branch : %s\n",
			version.git_branch);
	len += scnprintf(buf + len, PAGE_SIZE, "commit : %s\n",
			version.git_commit);
	len += scnprintf(buf + len, PAGE_SIZE, "date   : %s\n",
			version.code_date);
	pfile = fsm_get_presets();
	dev_count = (pfile ? pfile->hdr.ndev : 0);
	len += scnprintf(buf + len, PAGE_SIZE, "device : [%d, %d]\n",
			dev_count, fsm_dev_count());

	return len;
}

static ssize_t fsm_debug_store(struct class *class,
				struct class_attribute *attr, const char *buf, size_t len)
{
	fsm_config_t *cfg = fsm_get_config();
	int value = simple_strtoul(buf, NULL, 0);

	if (cfg) {
		cfg->i2c_debug = !!value;
	}
	pr_info("i2c debug: %s", (cfg->i2c_debug ? "ON" : "OFF"));

	return len;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
static struct class_attribute g_fsm_class_attrs[] = {
#ifdef CONFIG_FSM_Q6AFE
	__ATTR(fsm_re25, S_IRUGO, fsm_re25_show, NULL),
	__ATTR(fsm_f0, S_IRUGO, fsm_f0_show, NULL),
#endif
	__ATTR(fsm_info, S_IRUGO, fsm_info_show, NULL),
	__ATTR(fsm_debug, S_IWUSR, NULL, fsm_debug_store),
	__ATTR_NULL
};

static struct class g_fsm_class = {
	.name = FSM_DRV_NAME,
	.class_attrs = g_fsm_class_attrs,
};

#else
#ifdef CONFIG_FSM_Q6AFE
static CLASS_ATTR_RO(fsm_re25);
static CLASS_ATTR_RO(fsm_f0);
#endif
static CLASS_ATTR_RO(fsm_info);
static CLASS_ATTR_WO(fsm_debug);

static struct attribute *fsm_class_attrs[] = {
#ifdef CONFIG_FSM_Q6AFE
	&class_attr_fsm_re25.attr,
	&class_attr_fsm_f0.attr,
#endif
	&class_attr_fsm_info.attr,
	&class_attr_fsm_debug.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fsm_class);

/** Device model classes */
struct class g_fsm_class = {
	.name = FSM_DRV_NAME,
	.class_groups = fsm_class_groups,
};
#endif

int fsm_sysfs_init(struct device *dev)
{
	int ret;

	if (g_fsm_class_inited) {
		return MODULE_INITED;
	}
	// path: sys/class/$(FSM_DRV_NAME)
	ret = class_register(&g_fsm_class);
	if (!ret) {
		g_fsm_class_inited = 1;
	}

	return ret;
}

void fsm_sysfs_deinit(struct device *dev)
{
	class_unregister(&g_fsm_class);
	g_fsm_class_inited = 0;
}
#endif
