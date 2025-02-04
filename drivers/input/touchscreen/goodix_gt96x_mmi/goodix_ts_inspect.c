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

#include "goodix_ts_core.h"
#include <linux/rtc.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/stdarg.h>
#include <asm/uaccess.h>


/* test config */
#define TOTAL_FRAME_NUM 					16 /* rawdata test frames */
#define NOISEDATA_TEST_TIMES				1  /* noise test frames */

#define DISCARD_FRAMES						3

#define GOODIX_TEST_FILE_NAME				"gdx_test_limits"
#define MAX_PROC_OUTPUT_SIZE				2 * 1024 * 1024
#define MAX_DATA_BUFFER						30000
#define MAX_SHORT_NUM						15
#define MAX_LINE_LEN                		(1024 * 3 * 7)
#define MAX_DRV_NUM							52
#define MAX_SEN_NUM							75

#define STATISTICS_DATA_LEN					32
#define MAX_STR_LEN				 			32

#define CHN_VDD								0xFF
#define CHN_GND								0x7F
#define DRV_CHANNEL_FLAG		    		0x80

#define CSV_TP_SPECIAL_RAW_MIN				"special_raw_min"
#define CSV_TP_SPECIAL_RAW_MAX				"special_raw_max"
#define CSV_TP_SPECIAL_RAW_DELTA			"special_raw_delta"
#define CSV_TP_SHORT_THRESHOLD				"shortciurt_threshold"
#define CSV_TP_SPECIAL_SELFRAW_MAX			"special_selfraw_max"
#define CSV_TP_SPECIAL_SELFRAW_MIN			"special_selfraw_min"
#define CSV_TP_NOISE_LIMIT					"noise_data_limit"
#define CSV_TP_SELFNOISE_LIMIT				"noise_selfdata_limit"
#define CSV_TP_TEST_CONFIG					"test_config"

#define MAX_TEST_TIME_MS            		15000
#define DEFAULT_TEST_TIME_MS				7000

/* berlin A */
#define MAX_DRV_NUM_BRA				    	21
#define MAX_SEN_NUM_BRA				    	42
#define SHORT_TEST_TIME_REG_BRA				0x11FF2
#define DFT_ADC_DUMP_NUM_BRA				1396
#define DFT_SHORT_THRESHOLD_BRA  			16
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRA	16
#define SHORT_TEST_STATUS_REG_BRA			0x10400
#define SHORT_TEST_RESULT_REG_BRA			0x10410
#define DRV_DRV_SELFCODE_REG_BRA			0x1045E
#define SEN_SEN_SELFCODE_REG_BRA 			0x1084E
#define DRV_SEN_SELFCODE_REG_BRA			0x11712
#define DIFF_CODE_DATA_REG_BRA				0x11F72

/* berlin B */
#define MAX_DRV_NUM_BRB				    	52
#define MAX_SEN_NUM_BRB				    	75
#define SHORT_TEST_TIME_REG_BRB				0x26AE0
#define DFT_ADC_DUMP_NUM_BRB				762
#define DFT_SHORT_THRESHOLD_BRB				100
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRB	32
#define SHORT_TEST_STATUS_REG_BRB			0x20400
#define SHORT_TEST_RESULT_REG_BRB			0x20410
#define DRV_DRV_SELFCODE_REG_BRB			0x2049A
#define SEN_SEN_SELFCODE_REG_BRB 			0x21AF2
#define DRV_SEN_SELFCODE_REG_BRB			0x248A6
#define DIFF_CODE_DATA_REG_BRB				0x269E0

/* berlinD */
#define MAX_DRV_NUM_BRD				    	20
#define MAX_SEN_NUM_BRD				    	40
#define SHORT_TEST_TIME_REG_BRD				0x14D7A
#define DFT_ADC_DUMP_NUM_BRD				762
#define DFT_SHORT_THRESHOLD_BRD				100
#define DFT_DIFFCODE_SHORT_THRESHOLD_BRD	32
#define SHORT_TEST_STATUS_REG_BRD			0x13400
#define SHORT_TEST_RESULT_REG_BRD			0x13408
#define DRV_DRV_SELFCODE_REG_BRD			0x1344E
#define SEN_SEN_SELFCODE_REG_BRD 			0x137E6
#define DRV_SEN_SELFCODE_REG_BRD			0x14556
#define DIFF_CODE_DATA_REG_BRD				0x14D00

/* nottingham */
#define MAX_DRV_NUM_NOT				    	17
#define MAX_SEN_NUM_NOT				    	35
#define SHORT_TEST_TIME_REG_NOT				0x1479E
#define SHORT_TEST_STATUS_REG_NOT			0x13400
#define SHORT_TEST_RESULT_REG_NOT			0x13408
#define DRV_DRV_SELFCODE_REG_NOT			0x13446
#define SEN_SEN_SELFCODE_REG_NOT 			0x136EE
#define DRV_SEN_SELFCODE_REG_NOT			0x14152
#define DIFF_CODE_DATA_REG_NOT				0x14734

/* marseille */
#define MAX_DRV_NUM_MAR					20
#define MAX_SEN_NUM_MAR					34
#define SHORT_TEST_TIME_REG_MAR			0x138AC
#define SHORT_TEST_STATUS_REG_MAR			0x12400
#define SHORT_TEST_RESULT_REG_MAR			0x12408
#define DRV_DRV_SELFCODE_REG_MAR			0x12448
#define SEN_SEN_SELFCODE_REG_MAR 			0x127E0
#define DRV_SEN_SELFCODE_REG_MAR			0x137D0
#define DIFF_CODE_DATA_REG_MAR				0x1383E


#define ABS(val)			((val < 0)? -(val) : val)
#define MAX(a, b)			((a > b)? a : b)

enum GTP_TEST_ITEMS {
	GTP_CAP_TEST = 0,
	GTP_DELTA_TEST,
	GTP_NOISE_TEST,
	GTP_SHORT_TEST,
	GTP_SELFCAP_TEST,
	GTP_SELFNOISE_TEST,
	GTP_CUSTOM_INFO_TEST,
	GTP_CHIP_KEY_INFO_TEST,
	GTP_VERSION_TEST,
	MAX_TEST_ITEMS = 20
};

enum GTP_TEST_RESULTS {
	GTP_TEST_SOFT_ERR = 0,
	GTP_TEST_PANEL_ERR,
	GTP_TEST_OK
};

enum GTP_MUTUAL_DATA_TYPE {
	GTP_MUTUAL_RAW,
	GTP_MUTUAL_DIFF,
	GTP_SELF_RAW,
	GTP_SELF_DIFF
};

static int output_offset;
static char *output_buf;
static bool auto_test_result;

/* berlin A drv-sen map */
static u8 brl_a_drv_map[] = {
    42, 43, 44, 45, 46, 47, 48, 49,
    50, 51, 52, 53, 54, 55, 56, 57,
    58, 59, 60, 61, 62
};

static u8 brl_a_sen_map[] = {
    0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41
};

/* berlin B drv-sen map */
static u8 brl_b_drv_map[] = {
	75, 76, 77, 78, 79, 80, 81, 82,
	83, 84, 85, 86, 87, 88, 89, 90,
	91, 92, 93, 94, 95, 96, 97, 98,
	99, 100, 101, 102, 103, 104, 105,
	106, 107, 108, 109, 110, 111, 112,
	113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 125, 126
};

static u8 brl_b_sen_map[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
	11, 12, 13, 14, 15, 16, 17, 18,
	19, 20, 21, 22, 23, 24, 25, 26,
	27, 28, 29, 30, 31, 32, 33, 34,
	35, 36, 37, 38, 39, 40, 41, 42,
	43, 44, 45, 46, 47, 48, 49, 50,
	51, 52, 53, 54, 55, 56, 57, 58,
	59, 60, 61, 62, 63, 64, 65, 66,
	67, 68, 69, 70, 71, 72, 73, 74
};

/* berlin D drv-sen map */
static u8 brl_d_drv_map[] = {
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59,
};

static u8 brl_d_sen_map[] = {
    0, 1, 2, 3, 4, 5, 6, 7,
	8, 9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
};

/* nottingham drv-sen map */
static u8 not_drv_map[] = {
	35, 36, 37, 38, 39, 40, 41, 42, 43,
	44, 45, 46, 47, 48, 49, 50, 51
};

static u8 not_sen_map[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18,
	19, 20, 21, 22, 23, 24, 25, 26, 27,
	28, 29, 30, 31, 32, 33, 34
};

/* marseille drv-sen map */
static u8 mar_drv_map[] = {
	34, 35, 36, 37, 38, 39, 40, 41, 42, 43,
	44, 45, 46, 47, 48, 49, 50, 51, 52, 53,
};

static u8 mar_sen_map[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
	10, 11, 12, 13, 14, 15, 16, 17, 18,
	19, 20, 21, 22, 23, 24, 25, 26, 27,
	28, 29, 30, 31, 32, 33
};

typedef struct __attribute__((packed)) {
    u8 result;
	u8 drv_drv_num;
	u8 sen_sen_num;
	u8 drv_sen_num;
	u8 drv_gnd_avdd_num;
	u8 sen_gnd_avdd_num;
	u16 checksum;
} test_result_t;

#define CHIP_KEY_INFO_MAX_LEN 0x40
#define CHIP_UID_MAX_LEN 16
typedef struct {
    u8 hw_ver;
    u8 package_site;
    u8 chip_uid[CHIP_UID_MAX_LEN];
    u8 ft_test_result;
    u8 substrate_tech;
    u8 mcu_fab;
    u8 res[40];
    u8 vaild_len;
    u16 check_sum;
} chip_attr_info_t;

//test result
struct params_info_t {
	u32 max_drv_num;
	u32 max_sen_num;
	u8 *drv_map;
	u8 *sen_map;
	u32 short_test_time_reg;
	u32 short_test_status_reg;
	u32 short_test_result_reg;
	u32 drv_drv_selfcode_reg;
	u32 sen_sen_selfcode_reg;
	u32 drv_sen_selfcode_reg;
	u32 diffcode_data_reg;
	u16 short_test_dump_num;
	u16 dft_short_threshold;
	u16 short_diffcode_threshold;
};

struct params_info_t params_bra = {
	MAX_DRV_NUM_BRA,
	MAX_SEN_NUM_BRA,
	brl_a_drv_map,
	brl_a_sen_map,
	SHORT_TEST_TIME_REG_BRA,
	SHORT_TEST_STATUS_REG_BRA,
	SHORT_TEST_RESULT_REG_BRA,
	DRV_DRV_SELFCODE_REG_BRA,
	SEN_SEN_SELFCODE_REG_BRA,
	DRV_SEN_SELFCODE_REG_BRA,
	DIFF_CODE_DATA_REG_BRA,
	DFT_ADC_DUMP_NUM_BRA,
	DFT_SHORT_THRESHOLD_BRA,
	DFT_DIFFCODE_SHORT_THRESHOLD_BRA,
};

struct params_info_t params_brb = {
	MAX_DRV_NUM_BRB,
	MAX_SEN_NUM_BRB,
	brl_b_drv_map,
	brl_b_sen_map,
	SHORT_TEST_TIME_REG_BRB,
	SHORT_TEST_STATUS_REG_BRB,
	SHORT_TEST_RESULT_REG_BRB,
	DRV_DRV_SELFCODE_REG_BRB,
	SEN_SEN_SELFCODE_REG_BRB,
	DRV_SEN_SELFCODE_REG_BRB,
	DIFF_CODE_DATA_REG_BRB,
	DFT_ADC_DUMP_NUM_BRB,
	DFT_SHORT_THRESHOLD_BRB,
	DFT_DIFFCODE_SHORT_THRESHOLD_BRB,
};

struct params_info_t params_brd = {
	MAX_DRV_NUM_BRD,
	MAX_SEN_NUM_BRD,
	brl_d_drv_map,
	brl_d_sen_map,
	SHORT_TEST_TIME_REG_BRD,
	SHORT_TEST_STATUS_REG_BRD,
	SHORT_TEST_RESULT_REG_BRD,
	DRV_DRV_SELFCODE_REG_BRD,
	SEN_SEN_SELFCODE_REG_BRD,
	DRV_SEN_SELFCODE_REG_BRD,
	DIFF_CODE_DATA_REG_BRD,
	DFT_ADC_DUMP_NUM_BRD,
	DFT_SHORT_THRESHOLD_BRD,
	DFT_DIFFCODE_SHORT_THRESHOLD_BRD,
};

struct params_info_t params_not = {
	MAX_DRV_NUM_NOT,
	MAX_SEN_NUM_NOT,
	not_drv_map,
	not_sen_map,
	SHORT_TEST_TIME_REG_NOT,
	SHORT_TEST_STATUS_REG_NOT,
	SHORT_TEST_RESULT_REG_NOT,
	DRV_DRV_SELFCODE_REG_NOT,
	SEN_SEN_SELFCODE_REG_NOT,
	DRV_SEN_SELFCODE_REG_NOT,
	DIFF_CODE_DATA_REG_NOT,
	0,
	0,
	0,
};

struct params_info_t params_mar = {
	MAX_DRV_NUM_MAR,
	MAX_SEN_NUM_MAR,
	mar_drv_map,
	mar_sen_map,
	SHORT_TEST_TIME_REG_MAR,
	SHORT_TEST_STATUS_REG_MAR,
	SHORT_TEST_RESULT_REG_MAR,
	DRV_DRV_SELFCODE_REG_MAR,
	SEN_SEN_SELFCODE_REG_MAR,
	DRV_SEN_SELFCODE_REG_MAR,
	DIFF_CODE_DATA_REG_MAR,
	0,
	0,
	0,
};

struct ts_test_params {
	bool test_items[MAX_TEST_ITEMS];

	u32 rawdata_addr;
	u32 noisedata_addr;
	u32 self_rawdata_addr;
	u32 self_noisedata_addr;

	u32 drv_num;
	u32 sen_num;

	struct params_info_t *params_info;

	s32 cfg_buf[GOODIX_CFG_MAX_SIZE];
	s32 max_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s32 min_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s32 deviation_limits[MAX_DRV_NUM * MAX_SEN_NUM];
	s32 self_max_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s32 self_min_limits[MAX_DRV_NUM + MAX_SEN_NUM];
	s32 noise_threshold;
	s32 self_noise_threshold;

	u32 short_threshold;
	u32 r_drv_drv_threshold;
	u32 r_drv_sen_threshold;
	u32 r_sen_sen_threshold;
	u32 r_drv_gnd_threshold;
	u32 r_sen_gnd_threshold;
	u32 avdd_value;
};

struct ts_test_rawdata {
	s16 data[MAX_DRV_NUM * MAX_SEN_NUM];
	u32 size;
};

struct ts_test_self_rawdata {
	s16 data[MAX_DRV_NUM + MAX_SEN_NUM];
	u32 size;
};

struct ts_short_res {
	u8 short_num;
	s16 short_msg[4 * MAX_SHORT_NUM];
};

struct ts_open_res {
	u8 beyond_max_limit_cnt[MAX_DRV_NUM * MAX_SEN_NUM];
	u8 beyond_min_limit_cnt[MAX_DRV_NUM * MAX_SEN_NUM];
	u8 beyond_accord_limit_cnt[MAX_DRV_NUM * MAX_SEN_NUM];
};

struct goodix_ts_test {
	struct goodix_ts_core *ts;
	struct ts_test_params test_params;
	struct ts_test_rawdata rawdata[TOTAL_FRAME_NUM];
	struct ts_test_rawdata accord_arr[TOTAL_FRAME_NUM];
	struct ts_test_rawdata noisedata[NOISEDATA_TEST_TIMES];
	struct goodix_ic_config test_config;
	struct ts_test_self_rawdata self_rawdata;
	struct ts_test_self_rawdata self_noisedata;
	struct ts_short_res short_res;
	struct ts_open_res open_res;
	char cur_version[64];
	char chip_pid[8];
	chip_attr_info_t chip_info;
	char custom_info_flash[16];
	char custom_info_sram[16];

	char test_result[MAX_TEST_ITEMS];
};

static int cal_cha_to_cha_res(struct goodix_ts_test *ts_test, int v1, int v2)
{
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		return (v1 - v2) * 63 / v2;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		return (v1 - v2) * 74 / v2 + 20;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		return (v1 / v2 - 1) * 70 + 59;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE)
		return (v1 / v2 - 1) * 14;
	else
		return (v1 / v2 - 1) * 55 + 45; // nottingham
}

static int cal_cha_to_avdd_res(struct goodix_ts_test *ts_test, int v1, int v2)
{
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		return 64 * (2 * v2 - 25) * 40 / v1 - 40;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		return 64 * (2 * v2 - 25) * 99 / v1 - 60;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		return 64 * (2 * v2 - 25) * 93 / v1 - 20;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE)
		return 64 * (2 * v2 - 25) * 72 / v1 - 13;
	else
		return 64 * (2 * v2 - 25) * 76 / v1 - 15; // nottingham
}

static int cal_cha_to_gnd_res(struct goodix_ts_test *ts_test, int v)
{
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		return 64148 / v - 40;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		return 150500 / v - 60;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		return 145000 / v - 15;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE)
		return 28800 / v - 13;
	else
		return 120000 / v - 16; // nottingham
}

static int ts_test_reset(struct goodix_ts_test *ts_test,
    u32 delay_ms)
{
	return ts_test->ts->hw_ops->reset(ts_test->ts, delay_ms);
}

static int ts_test_read(struct goodix_ts_test *ts_test,
    u32 addr, u8 *data, u32 len)
{
	return ts_test->ts->hw_ops->read(ts_test->ts, addr, data, len);
}

static int ts_test_write(struct goodix_ts_test *ts_test,
    u32 addr, u8 *data, u32 len)
{
	return ts_test->ts->hw_ops->write(ts_test->ts, addr, data, len);
}

static int ts_test_send_cmd(struct goodix_ts_test *ts_test,
    struct goodix_ts_cmd *cmd)
{
	return ts_test->ts->hw_ops->send_cmd(ts_test->ts, cmd);
}

static int ts_test_irq_enable(struct goodix_ts_test *ts_test,
	bool flag)
{
	return ts_test->ts->hw_ops->irq_enable(ts_test->ts, flag);
}

static int ts_test_send_config(struct goodix_ts_test *ts_test,
    int type)
{
	struct goodix_ic_config *cfg;

	if (type >= GOODIX_MAX_CONFIG_GROUP) {
		ts_err("unsupproted config type %d", type);
		return -EINVAL;
	}
	cfg = &ts_test->ts->ic_configs[type];
	if (cfg->len <= 0) {
		ts_err("no valid normal config found");
		return -EINVAL;
	}

	return ts_test->ts->hw_ops->send_config(ts_test->ts, cfg->data, cfg->len);
}

static int ts_test_read_version(struct goodix_ts_test *ts_test,
    struct goodix_fw_version *version)
{
	return ts_test->ts->hw_ops->read_version(ts_test->ts, version);
}

static void proc_append_string(char *format_str, ...)
{
	va_list args;

	va_start(args, format_str);
	output_offset += vsprintf(output_buf + output_offset, format_str, args);
	va_end(args);
}

static void goto_next_line(char **ptr)
{
	do {
		*ptr = *ptr + 1;
	} while (**ptr != '\n' && **ptr != '\0');
	if (**ptr == '\0') {
		return;
	}
	*ptr = *ptr + 1;
}

static void copy_this_line(char *dest, char *src)
{
	char *copy_from;
	char *copy_to;

	copy_from = src;
	copy_to = dest;
	do {
		*copy_to = *copy_from;
		copy_from++;
		copy_to++;
	} while((*copy_from != '\n') && (*copy_from != '\r') && (*copy_from != '\0'));
	*copy_to = '\0';
}

static int getrid_space(s8* data, s32 len)
{
	u8* buf = NULL;
	s32 i;
	u32 count = 0;

	buf = (char*)vzalloc(len + 5);
	if (buf == NULL){
		ts_err("get space alloc error");
		return -ESRCH;
	}

	for (i = 0; i < len; i++)
	{
		if (data[i] == ' ' || data[i] == '\r' || data[i] == '\n')
		{
			continue;
		}
		buf[count++] = data[i];
	}

	buf[count++] = '\0';

	memcpy(data, buf, count);
	vfree(buf);

	return count;
}

static int parse_valid_data(char *buf_start, loff_t buf_size,
    char *ptr, s32 *data, s32 rows)
{
	int i = 0;
	int j = 0;
	char *token = NULL;
	char *tok_ptr = NULL;
	char *row_data = NULL;
	long temp_val;

	if (!ptr) {
		ts_err("ptr is NULL");
		return -EINVAL;
	}
	if (!data) {
		ts_err("data is NULL");
		return -EINVAL;
	}

	row_data = (char *)vzalloc(MAX_LINE_LEN);
	if (!row_data) {
		ts_err("alloc bytes %d failed.", MAX_LINE_LEN);
		return -ENOMEM;
	}

	for (i = 0; i < rows; i++) {
		memset(row_data, 0, MAX_LINE_LEN);
		copy_this_line(row_data, ptr);
		getrid_space(row_data, strlen(row_data));
		tok_ptr = row_data;
		while ((token = strsep(&tok_ptr,","))) {
			if (strlen(token) == 0)
				continue;
			if (kstrtol(token, 0, &temp_val)) {
				vfree(row_data);
				return -EINVAL;
			}
			data[j++] = (s32)temp_val;
		}
		if (i == rows - 1)
			break;
		goto_next_line(&ptr);				//next row
		if(!ptr || (0 == strlen(ptr)) || (ptr >= (buf_start + buf_size))) {
			ts_info("invalid ptr, return");
			vfree(row_data);
			row_data = NULL;
			return -EPERM;
		}
	}
	vfree(row_data);
	return j;
}

static int parse_csvfile(char *buf, size_t size, char *target_name,
        s32 *data, s32 rows, s32 col)
{
	int ret = 0;
	char *ptr = NULL;
	int read_ret;

	read_ret = size;
	if (read_ret > 0) {
		ptr = buf;
		ptr = strstr(ptr, target_name);
		if (!ptr) {
			ts_info("load %s failed 1, maybe not this item", target_name);
			return -EINTR;
		}

		goto_next_line(&ptr);
		if (!ptr || (0 == strlen(ptr))) {
			ts_err("load %s failed 2!", target_name);
			return -EIO;
		}

		if (data) {
			ret = parse_valid_data(buf, size, ptr, data, rows);
		} else {
			ts_err("load %s failed 3!", target_name);
			return -EINTR;
		}
	} else {
		ts_err("ret=%d, read_ret=%d", ret, read_ret);
		ret = -ENXIO;
	}

	return ret;
}


static void goodix_init_params(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_core *ts = ts_test->ts;
	struct ts_test_params *test_params = &ts_test->test_params;

	test_params->rawdata_addr = ts->ic_info.misc.mutual_rawdata_addr;
	test_params->noisedata_addr = ts->ic_info.misc.mutual_diffdata_addr;
	test_params->self_rawdata_addr = ts->ic_info.misc.self_rawdata_addr;
	test_params->self_noisedata_addr = ts->ic_info.misc.self_diffdata_addr;

	test_params->drv_num = ts->ic_info.parm.drv_num;
	test_params->sen_num = ts->ic_info.parm.sen_num;

	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_A)
		test_params->params_info = &params_bra;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_B)
		test_params->params_info = &params_brb;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D)
		test_params->params_info = &params_brd;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_NOTTINGHAM)
		test_params->params_info = &params_not;
	else if (ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE)
		test_params->params_info = &params_mar;
}

static int goodix_init_testlimits(struct goodix_ts_test *ts_test)
{
	int ret;
	int i;
	u32 data_buf[10] = {0};
	char *temp_buf = NULL;
	struct ts_test_params *test_params = &ts_test->test_params;
	struct goodix_ts_core *cd = ts_test->ts;
	const struct firmware *firmware = NULL;
	struct device *dev = &cd->pdev->dev;
	char limit_file[100] = {0};
	u32 tx = test_params->drv_num;
	u32 rx = test_params->sen_num;

	sprintf(limit_file, "%s_%d.csv", GOODIX_TEST_FILE_NAME,
			cd->fw_version.sensor_id);
	ts_info("limit_file_name:%s", limit_file);

	ret = request_firmware(&firmware, limit_file, dev);
	if (ret < 0) {
		ts_err("limits file [%s] not available", limit_file);
		return -EINVAL;
	}
	if (firmware->size <= 0) {
		ts_err("request_firmware, limits param length error,len:%zu",
			firmware->size);
		ret = -EINVAL;
		goto exit_free;
	}
	temp_buf = vzalloc(firmware->size + 1);
	if (!temp_buf) {
		ts_err("alloc bytes failed.");
		ret = -ENOMEM;
		goto exit_free;
	}
	memcpy(temp_buf, firmware->data, firmware->size);

	test_params->test_items[GTP_VERSION_TEST] = true;
	test_params->test_items[GTP_CHIP_KEY_INFO_TEST] = true;
	test_params->test_items[GTP_CUSTOM_INFO_TEST] = true;
	test_params->test_items[GTP_CAP_TEST] = true;
	test_params->test_items[GTP_DELTA_TEST] = true;

	/* obtain config data */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_TEST_CONFIG,
		test_params->cfg_buf, 1, GOODIX_CFG_MAX_SIZE);
	if (ret < 0) {
		ts_info("Can't find %s", CSV_TP_TEST_CONFIG);
		ts_test->test_config.len = cd->hw_ops->read_config(cd, ts_test->test_config.data, GOODIX_CFG_MAX_SIZE);
	} else {
		ts_info("parse_csvfile %s OK, cfg_len:%d", CSV_TP_TEST_CONFIG, ret);
		for (i = 0; i < ret; i++)
			ts_test->test_config.data[i] = (u8)test_params->cfg_buf[i];
		ts_test->test_config.len = ret;
	}

	/* obtain mutual_raw min */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_SPECIAL_RAW_MIN,
		test_params->min_limits, rx, tx);
	if (ret < 0) {
		ts_err("Failed get min_limits");
		goto exit_free;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_SPECIAL_RAW_MIN);
	}
	/* obtain mutual_raw max */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_SPECIAL_RAW_MAX,
		test_params->max_limits, rx, tx);
	if (ret < 0) {
		ts_err("Failed get max_limits");
		goto exit_free;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_SPECIAL_RAW_MAX);
	}
	/* obtain delta limit */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_SPECIAL_RAW_DELTA,
		test_params->deviation_limits, rx, tx);
	if (ret < 0) {
		ts_err("Failed get delta limit");
		goto exit_free;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_SPECIAL_RAW_DELTA);
	}

	/* obtain self_raw min */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_SPECIAL_SELFRAW_MIN,
		test_params->self_min_limits, 1, tx + rx);
	/* obtain self_raw max */
	ret |= parse_csvfile(temp_buf, firmware->size, CSV_TP_SPECIAL_SELFRAW_MAX,
		test_params->self_max_limits, 1, tx + rx);
	if (ret < 0) {
		ts_info("Can't find self_min_max_limits, ship this item");
		ret = 0;
		test_params->test_items[GTP_SELFCAP_TEST] = false;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_SPECIAL_SELFRAW_MIN);
		ts_info("parse_csvfile %s OK", CSV_TP_SPECIAL_SELFRAW_MAX);
		test_params->test_items[GTP_SELFCAP_TEST] = true;
	}

	/* obtain noise_threshold */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_NOISE_LIMIT,
		&test_params->noise_threshold, 1, 1);
	if (ret < 0) {
		ts_info("Can't find noise_threshold, skip this item");
		ret = 0;
		test_params->test_items[GTP_NOISE_TEST] = false;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_NOISE_LIMIT);
		test_params->test_items[GTP_NOISE_TEST] = true;
	}

	/* obtain self_noise_threshold */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_SELFNOISE_LIMIT,
		&test_params->self_noise_threshold, 1, 1);
	if (ret < 0) {
		ts_info("Can't find self_noise_threshold, skip this item");
		ret = 0;
		test_params->test_items[GTP_SELFNOISE_TEST] = false;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_SELFNOISE_LIMIT);
		test_params->test_items[GTP_SELFNOISE_TEST] = true;
	}

	 /* obtain short_params */
	ret = parse_csvfile(temp_buf, firmware->size, CSV_TP_SHORT_THRESHOLD,
		(s32 *)data_buf, 1, 7);
	if (ret < 0) {
		ts_info("Can't find short shortciurt_threshold, skip this item");
		ret = 0;
		test_params->test_items[GTP_SHORT_TEST] = false;
	} else {
		ts_info("parse_csvfile %s OK", CSV_TP_SHORT_THRESHOLD);
		test_params->test_items[GTP_SHORT_TEST] = true;
		test_params->short_threshold = data_buf[0];
		test_params->r_drv_drv_threshold = data_buf[1];
		test_params->r_drv_sen_threshold = data_buf[2];
		test_params->r_sen_sen_threshold = data_buf[3];
		test_params->r_drv_gnd_threshold = data_buf[4];
		test_params->r_sen_gnd_threshold = data_buf[5];
		test_params->avdd_value = data_buf[6];
	}

exit_free:
	vfree(temp_buf);
	if (firmware)
		release_firmware(firmware);
	return ret;
}

static int goodix_tptest_prepare(struct goodix_ts_test *ts_test)
{
	int ret;
	struct goodix_ic_config *cfg = &ts_test->test_config;

	ts_info("TP test prepare IN");

	goodix_init_params(ts_test);
	/* parse test limits from csv */
	ret = goodix_init_testlimits(ts_test);
	if (ret < 0) {
		ts_err("Failed to init testlimits from csv.");
		return ret;
	}

	/* disable irq */
	ts_test_irq_enable(ts_test, false);
	/* close esd */
	goodix_ts_esd_off(ts_test->ts);

	/* send test config if exist */
	if (cfg->len > 0) {
		ts_info("Test config exists and send it");
		ret = ts_test->ts->hw_ops->send_config(ts_test->ts, cfg->data, cfg->len);
		if (ret < 0) {
			ts_err("Send test config failed, exit");
			goodix_ts_esd_on(ts_test->ts);
			ts_test_irq_enable(ts_test, true);
			return ret;
		}
	}

	return 0;
}

static void goodix_tptest_finish(struct goodix_ts_test *ts_test)
{
	ts_info("TP test finish IN");
	/* reset chip */
	ts_test_reset(ts_test, 100);
	/* send normal config */
	if (ts_test->test_config.len > 0) {
		if (ts_test_send_config(ts_test, CONFIG_TYPE_NORMAL))
			ts_err("Send normal config failed");
	}

	/* open esd */
	goodix_ts_esd_on(ts_test->ts);
	/* enable irq */
	ts_test_irq_enable(ts_test, true);
}

#define SHORT_TEST_RUN_REG			0x10400
#define INSPECT_FW_SWITCH_CMD		0x85
#define TEST_FW_PID  				"OST"
static int goodix_short_test_prepare(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_cmd tmp_cmd;
	struct goodix_fw_version fw_ver;
	int ret;
	int retry;
	int resend = 3;

	ts_info("short test prepare IN");
	ts_test->test_result[GTP_SHORT_TEST] = GTP_TEST_SOFT_ERR;
	tmp_cmd.len = 4;
	tmp_cmd.cmd = INSPECT_FW_SWITCH_CMD;

resend_cmd:
	ret = ts_test_send_cmd(ts_test, &tmp_cmd);
	if (ret < 0) {
		ts_err("send test mode failed");
		return ret;
	}

	retry = 3;
	while (retry--) {
	msleep(40);
		ret = ts_test_read_version(ts_test, &fw_ver);
		if (ret < 0) {
			ts_err("read test version failed");
			return ret;
		}
		ret = memcmp(&(fw_ver.patch_pid[3]), TEST_FW_PID, strlen(TEST_FW_PID));
		if (ret == 0)
			return 0;
		else
			ts_info("patch ID dismatch %s != %s", fw_ver.patch_pid, TEST_FW_PID);
	}

	if (resend--) {
		ts_test_reset(ts_test, 100);
		goto resend_cmd;
	}

	return -EINVAL;
}

static u32 map_die2pin(struct ts_test_params *test_params, u32 chn_num)
{
	int i = 0;
	u32 res = 255;

	if (chn_num & DRV_CHANNEL_FLAG)
		chn_num = (chn_num & ~DRV_CHANNEL_FLAG) + test_params->params_info->max_sen_num;

	for (i = 0; i < test_params->params_info->max_sen_num; i++) {
		if (test_params->params_info->sen_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	/* res != 255 mean found the corresponding channel num */
	if (res != 255)
		return res;
	/* if cannot find in SenMap try find in DrvMap */
	for (i = 0; i < test_params->params_info->max_drv_num; i++) {
		if (test_params->params_info->drv_map[i] == chn_num) {
			res = i;
			break;
		}
	}
	if (i >= test_params->params_info->max_drv_num)
		ts_err("Faild found corrresponding channel num:%d", chn_num);
	else
		res |= DRV_CHANNEL_FLAG;

	return res;
}

static void goodix_save_short_res(struct ts_test_params *params,
	u16 chn1, u16 chn2, int r)
{
	int i;
	u8 repeat_cnt = 0;
	u8 repeat = 0;
	struct goodix_ts_test *ts_test = container_of(params,
		struct goodix_ts_test, test_params);
	struct ts_short_res *short_res = &ts_test->short_res;

	if (chn1 == chn2 || short_res->short_num >= MAX_SHORT_NUM)
		return;

	for (i = 0; i < short_res->short_num; i++) {
		repeat_cnt = 0;
		if (short_res->short_msg[4 * i] == chn1)
			repeat_cnt++;
		if (short_res->short_msg[4 * i] == chn2)
			repeat_cnt++;
		if (short_res->short_msg[4 * i + 1] == chn1)
			repeat_cnt++;
		if (short_res->short_msg[4 * i + 1] == chn2)
			repeat_cnt++;
		if (repeat_cnt >= 2){
			repeat = 1;
			break;
		}
	}
	if (repeat == 0) {
		short_res->short_msg[4 * short_res->short_num + 0] = chn1;
		short_res->short_msg[4 * short_res->short_num + 1] = chn2;
		short_res->short_msg[4 * short_res->short_num + 2] = (r >> 8) & 0xFF;
		short_res->short_msg[4 * short_res->short_num + 3] = r & 0xFF;
		if (short_res->short_num < MAX_SHORT_NUM)
			short_res->short_num++;
	}
}

static int gdix_check_tx_tx_shortcircut(struct goodix_ts_test *ts_test,
        u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->params_info->max_drv_num;
	int max_sen_num = test_params->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	data_reg = test_params->params_info->drv_drv_selfcode_reg;
	for (i = 0; i < short_ch_num; i++) {
		ret = ts_test_read(ts_test, data_reg, data_buf, size);
		if (ret < 0) {
			ts_err("Failed read Drv-to-Drv short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Drv-to-Drv adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_drv_drv_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		short_die_num -= max_sen_num;
		if (short_die_num >= max_drv_num) {
			ts_info("invalid short pad num:%d",
				short_die_num + max_sen_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_drv_num; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(ts_test, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num =
					map_die2pin(test_params, short_die_num + max_sen_num);
				slave_pin_num =
					map_die2pin(test_params, j + max_sen_num);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(test_params, master_pin_num,
					slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
							short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_rx_rx_shortcircut(struct goodix_ts_test *ts_test,
        u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf;
	u32 data_reg;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_sen_num = test_params->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_sen_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&drv shortcircut check */
	data_reg = test_params->params_info->sen_sen_selfcode_reg;
	for (i = 0; i < short_ch_num; i++) {
		ret = ts_test_read(ts_test, data_reg, data_buf, size);
		if (ret) {
			ts_err("Failed read Sen-to-Sen short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Sen-to-Sen adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_sen_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			ts_info("invalid short pad num:%d",	short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = short_die_num + 1; j < max_sen_num; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(ts_test, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(test_params, master_pin_num,
					slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
							short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_tx_rx_shortcircut(struct goodix_ts_test *ts_test,
        u8 short_ch_num)
{
	int ret = 0, err = 0;
	u32 r_threshold = 0, short_r = 0;
	int size = 0, i = 0, j = 0;
	u16 adc_signal = 0;
	u8 master_pin_num, slave_pin_num;
	u8 *data_buf = NULL;
	u32 data_reg;
	struct ts_test_params *test_params = &ts_test->test_params;
	int max_drv_num = test_params->params_info->max_drv_num;
	int max_sen_num = test_params->params_info->max_sen_num;
	u16 self_capdata, short_die_num = 0;

	size = 4 + max_drv_num * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* drv&sen shortcircut check */
	data_reg = test_params->params_info->drv_sen_selfcode_reg;
	for (i = 0; i < short_ch_num; i++) {
		ret = ts_test_read(ts_test, data_reg, data_buf, size);
		if (ret) {
			ts_err("Failed read Drv-to-Sen short rawdata");
			err = -EINVAL;
			break;
		}

		if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
			ts_err("Drv-to-Sen adc data checksum error");
			err = -EINVAL;
			break;
		}

		r_threshold = test_params->r_drv_sen_threshold;
		short_die_num = le16_to_cpup((__le16 *)&data_buf[0]);
		if (short_die_num >= max_sen_num) {
			ts_info("invalid short pad num:%d",	short_die_num);
			continue;
		}

		/* TODO: j start position need recheck */
		self_capdata = le16_to_cpup((__le16 *)&data_buf[2]);
		if (self_capdata == 0xffff || self_capdata == 0) {
			ts_info("invalid self_capdata:0x%x", self_capdata);
			continue;
		}

		for (j = 0; j < max_drv_num; j++) {
			adc_signal = le16_to_cpup((__le16 *)&data_buf[4 + j * 2]);

			if (adc_signal < test_params->short_threshold)
				continue;

			short_r = (u32)cal_cha_to_cha_res(ts_test, self_capdata, adc_signal);
			if (short_r < r_threshold) {
				master_pin_num = map_die2pin(test_params, short_die_num);
				slave_pin_num = map_die2pin(test_params, j + max_sen_num);
				if (master_pin_num == 0xFF || slave_pin_num == 0xFF) {
					ts_info("WARNNING invalid pin");
					continue;
				}
				goodix_save_short_res(test_params, master_pin_num,
					slave_pin_num, short_r);
				ts_err("short circut:R=%dK,R_Threshold=%dK",
							short_r, r_threshold);
				ts_err("%s%d--%s%d shortcircut",
					(master_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(master_pin_num & ~DRV_CHANNEL_FLAG),
					(slave_pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
					(slave_pin_num & ~DRV_CHANNEL_FLAG));
				err = -EINVAL;
			}
		}
		data_reg += size;
	}

	kfree(data_buf);
	return err;
}

static int gdix_check_resistance_to_gnd(struct ts_test_params *test_params,
        u16 adc_signal, u32 pos)
{
	long r = 0;
	u16 r_th = 0, avdd_value = 0;
	u16 chn_id_tmp = 0;
	u8 pin_num = 0;
	unsigned short short_type;
	struct goodix_ts_test *ts_test = container_of(test_params,
		struct goodix_ts_test, test_params);
	int max_drv_num = test_params->params_info->max_drv_num;
	int max_sen_num = test_params->params_info->max_sen_num;

	avdd_value = test_params->avdd_value;
	short_type = adc_signal & 0x8000;
	adc_signal &= ~0x8000;
	if (adc_signal == 0)
		adc_signal = 1;

	if (short_type == 0) {
		/* short to GND */
		r = cal_cha_to_gnd_res(ts_test, adc_signal);
	} else {
		/* short to VDD */
		r = cal_cha_to_avdd_res(ts_test, adc_signal, avdd_value);
	}

	if (pos < max_drv_num)
		r_th = test_params->r_drv_gnd_threshold;
	else
		r_th = test_params->r_sen_gnd_threshold;

	chn_id_tmp = pos;
	if (chn_id_tmp < max_drv_num)
		chn_id_tmp += max_sen_num;
	else
		chn_id_tmp -= max_drv_num;

	if (r < r_th) {
		pin_num = map_die2pin(test_params, chn_id_tmp);
		goodix_save_short_res(test_params, pin_num,
				short_type ? CHN_VDD : CHN_GND, r);
		ts_err("%s%d shortcircut to %s,R=%ldK,R_Threshold=%dK",
				(pin_num & DRV_CHANNEL_FLAG) ? "DRV" : "SEN",
				(pin_num & ~DRV_CHANNEL_FLAG),
				short_type ? "VDD" : "GND",
				r, r_th);

		return -EINVAL;
	}

	return 0;
}

static int gdix_check_gndvdd_shortcircut(struct goodix_ts_test *ts_test)
{
	int ret = 0, err = 0;
	int size = 0, i = 0;
	u16 adc_signal = 0;
	u32 data_reg;
	u8 *data_buf = NULL;
	int max_drv_num = ts_test->test_params.params_info->max_drv_num;
	int max_sen_num = ts_test->test_params.params_info->max_sen_num;

	size = (max_drv_num + max_sen_num) * 2 + 2;
	data_buf = kzalloc(size, GFP_KERNEL);
	if (!data_buf) {
		ts_err("Failed to alloc memory");
		return -ENOMEM;
	}
	/* read diff code, diff code will be used to calculate
		* resistance between channel and GND */
	data_reg = ts_test->test_params.params_info->diffcode_data_reg;
	ret = ts_test_read(ts_test, data_reg, data_buf, size);
	if (ret < 0) {
		ts_err("Failed read to-gnd rawdata");
		err = -EINVAL;
		goto err_out;
	}

	if (checksum_cmp(data_buf, size, CHECKSUM_MODE_U8_LE)) {
		ts_err("diff code checksum error");
		err = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < max_drv_num + max_sen_num; i++) {
		adc_signal = le16_to_cpup((__le16 *)&data_buf[i * 2]);
		ret = gdix_check_resistance_to_gnd(&ts_test->test_params,
					adc_signal, i);
		if (ret != 0) {
			ts_err("Resistance to-gnd/vdd short");
			err = ret;
		}
	}

err_out:
	kfree(data_buf);
	return err;
}

static int goodix_shortcircut_analysis(struct goodix_ts_test *ts_test)
{
	int ret;
	int err = 0;
	test_result_t test_result;

	ret = ts_test_read(ts_test, ts_test->test_params.params_info->short_test_result_reg,
		(u8 *)&test_result, sizeof(test_result));
	if (ret < 0) {
		ts_err("Read TEST_RESULT_REG failed");
		return ret;
	}

	if (checksum_cmp((u8 *)&test_result, sizeof(test_result),
		CHECKSUM_MODE_U8_LE)) {
		ts_err("shrot result checksum err");
		return -EINVAL;
	}

	if (!(test_result.result & 0x0F)) {
		ts_info(">>>>> No shortcircut");
		return 0;
	}
	ts_info("short flag 0x%02x, drv&drv:%d, sen&sen:%d, drv&sen:%d, drv/GNDVDD:%d, sen/GNDVDD:%d",
		test_result.result, test_result.drv_drv_num, test_result.sen_sen_num,
		test_result.drv_sen_num, test_result.drv_gnd_avdd_num, test_result.sen_gnd_avdd_num);

	if (test_result.drv_drv_num)
		err |= gdix_check_tx_tx_shortcircut(ts_test, test_result.drv_drv_num);
	if (test_result.sen_sen_num)
		err |= gdix_check_rx_rx_shortcircut(ts_test, test_result.sen_sen_num);
	if (test_result.drv_sen_num)
		err |= gdix_check_tx_rx_shortcircut(ts_test, test_result.drv_sen_num);
	if (test_result.drv_gnd_avdd_num || test_result.sen_gnd_avdd_num)
		err |= gdix_check_gndvdd_shortcircut(ts_test);

	ts_info(">>>>> short check return 0x%x", err);

	return err;
}

#define SHORT_FW_CMD_REG				0x10400
static int send_test_cmd(struct goodix_ts_test *ts_test,
    struct goodix_ts_cmd *cmd)
{
	int ret;
	u32 reg = SHORT_FW_CMD_REG;
	cmd->state = 0;
	cmd->ack = 0;
	goodix_append_checksum(&(cmd->buf[2]), cmd->len - 2,
            CHECKSUM_MODE_U8_LE);
	ret = ts_test_write(ts_test, reg, cmd->buf, cmd->len + 2);
	if (ret < 0)
		return ret;
	usleep_range(10000, 11000);
	return ret;
}


#define INSPECT_PARAM_CMD				0xAA
#define SHORT_TEST_FINISH_FLAG  		0x88
#define SHORT_TEST_THRESHOLD_REG		0x20402
static void goodix_shortcircut_test(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	int retry;
	u16 test_time;
	u8 status;
	int ic_type = ts_test->ts->bus->ic_type;
	struct goodix_ts_cmd test_parm_cmd;
	// u8 test_param[6];

	ts_info("---------------------- short_test begin ----------------------");
	ret = goodix_short_test_prepare(ts_test);
	if (ret < 0) {
		ts_err("Failed enter short test mode");
		return;
	}

	if (ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE) {
		ts_test_read(ts_test, 0x10400, &status, 1);
		if (status != 0xFD) {
			ts_err("marseille short status[0x%02x] != 0xFD", status);
			return;
		}
		status = 0;
		ts_test_write(ts_test, 0x10400, &status, 1);
		retry = 20;
		while (retry--) {
			msleep(20);
			ts_test_read(ts_test, 0x10400, &status, 1);
			if (status == 0xAA)
				break;
		}
		if (retry < 0) {
			ts_err("marseille short status[0x%02x] != 0xAA", status);
			return;
		}
	}

	msleep(500); //BerlinB quick check need to delay at least 500ms

	/* get short test time */
	ret = ts_test_read(ts_test, ts_test->test_params.params_info->short_test_time_reg, (u8 *)&test_time, 2);
	if (ret < 0) {
		ts_err("Failed to get test_time, default %dms", DEFAULT_TEST_TIME_MS);
		test_time = DEFAULT_TEST_TIME_MS;
	} else {
		if (ic_type == IC_TYPE_BERLIN_A)
			test_time /= 10;
		if (test_time > MAX_TEST_TIME_MS) {
			ts_info("test time too long %d > %d",
				test_time, MAX_TEST_TIME_MS);
			test_time = MAX_TEST_TIME_MS;
		}
		ts_info("get test time %dms", test_time);
	}

	/* start short test */
	if (ic_type == IC_TYPE_BERLIN_A) {
		test_parm_cmd.len = 0x0A;
		test_parm_cmd.cmd = INSPECT_PARAM_CMD;
		test_parm_cmd.data[0] = ts_test->test_params.params_info->dft_short_threshold & 0xFF;
		test_parm_cmd.data[1] = (ts_test->test_params.params_info->dft_short_threshold >> 8) & 0xFF;
		test_parm_cmd.data[2] = ts_test->test_params.params_info->short_diffcode_threshold & 0xFF;
		test_parm_cmd.data[3] = (ts_test->test_params.params_info->short_diffcode_threshold >> 8) & 0xFF;
		test_parm_cmd.data[4] = ts_test->test_params.params_info->short_test_dump_num & 0xFF;
		test_parm_cmd.data[5] = (ts_test->test_params.params_info->short_test_dump_num >> 8) & 0xFF;
		ret = send_test_cmd(ts_test, &test_parm_cmd);
		if (ret < 0) {
			ts_err("send INSPECT_PARAM_CMD failed");
			return;
		}
	} else {
		// test_param[0] = ts_test->test_params.params_info->dft_short_threshold & 0xFF;
		// test_param[1] = (ts_test->test_params.params_info->dft_short_threshold >> 8) & 0xFF;
		// test_param[2] = ts_test->test_params.params_info->short_diffcode_threshold & 0xFF;
		// test_param[3] = (ts_test->test_params.params_info->short_diffcode_threshold >> 8) & 0xFF;
		// test_param[4] = ts_test->test_params.params_info->short_test_dump_num & 0xFF;
		// test_param[5] = (ts_test->test_params.params_info->short_test_dump_num >> 8) & 0xFF;
		// ts_test_write(ts_test, SHORT_TEST_THRESHOLD_REG, test_param, sizeof(test_param));
		status = 0;
		ts_test_write(ts_test, SHORT_TEST_RUN_REG, &status, 1);
	}

	/* wait short test finish */
	if (test_time > 0)
		msleep(test_time);

	retry = 50;
	while (retry--) {
		ret = ts_test_read(ts_test, ts_test->test_params.params_info->short_test_status_reg, &status, 1);
		if (!ret && status == SHORT_TEST_FINISH_FLAG)
			break;
		msleep(50);
	}
	if (retry < 0) {
		ts_err("short test failed, status:0x%02x", status);
		return;
	}

	/* start analysis short result */
	ts_info("short_test finished, start analysis");
	ret = goodix_shortcircut_analysis(ts_test);
	if (ret < 0)
		ts_test->test_result[GTP_SHORT_TEST] = GTP_TEST_PANEL_ERR;
	else
		ts_test->test_result[GTP_SHORT_TEST] = GTP_TEST_OK;
}

#define GOODIX_CMD_RAWDATA	2
#define GOODIX_TOUCH_EVENT	0x80
static int goodix_cap_test_prepare(struct goodix_ts_test *ts_test)
{
	int ret;
	struct goodix_ts_cmd temp_cmd;

	ts_info("cap test prepare IN");

	/* switch rawdata mode */
	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D ||
			ts_test->ts->bus->ic_type == IC_TYPE_NOTTINGHAM ||
			ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE) {
		temp_cmd.cmd = 0x90;
		temp_cmd.data[0] = 0x81;
		temp_cmd.len = 5;
	} else {
		temp_cmd.cmd = GOODIX_CMD_RAWDATA;
		temp_cmd.len = 4;
	}
	ret = ts_test_send_cmd(ts_test, &temp_cmd);
	if (ret < 0)
		ts_err("Enter rawdata mode failed");

	return ret;
}

static int goodix_cap_test_finish(struct goodix_ts_test *ts_test)
{
	ts_info("cap_test finished");
	/* switch coor mode */
	ts_test_reset(ts_test, 100);
	return 0;
}

#define FRAME_DATA_MAX_SIZE 2500
static int goodix_poll_rawdata(struct goodix_ts_test *ts_test, u8 data_type, u8 *data, u32 data_num)
{
	struct goodix_ts_core *cd = ts_test->ts;
	u32 flag_addr = ts_test->ts->ic_info.misc.touch_data_addr;
	u32 data_addr;
	u8 *frame_buf;
	int offset = 0;
	int frame_len;
	u8 val;
	int ret;
	int retry;

	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D ||
			ts_test->ts->bus->ic_type == IC_TYPE_NOTTINGHAM ||
			ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE)
		flag_addr = ts_test->ts->ic_info.misc.frame_data_addr;

	if (data_type == GTP_MUTUAL_RAW) {
		data_addr = ts_test->test_params.rawdata_addr;
	} else if (data_type == GTP_MUTUAL_DIFF) {
		data_addr = ts_test->test_params.noisedata_addr;
	} else if (data_type == GTP_SELF_RAW) {
		data_addr = ts_test->test_params.self_rawdata_addr;
	} else {
		data_addr = ts_test->test_params.self_noisedata_addr;
	}

	val = 0;
	ret = ts_test_write(ts_test, flag_addr, &val, 1);
	if (ret < 0)
		return ret;
	retry = 20;
	while (retry--) {
		usleep_range(5000, 5100);
		ret = ts_test_read(ts_test, flag_addr, &val, 1);
		if (!ret && (val & 0x80))
			break;
	}
	if (retry < 0) {
		ts_err("rawdata is not ready val:0x%02x", val);
		return -EAGAIN;
	}

	frame_buf = kzalloc(FRAME_DATA_MAX_SIZE, GFP_KERNEL);
	if (cd->bus->ic_type == IC_TYPE_BERLIN_D ||
			cd->bus->ic_type == IC_TYPE_NOTTINGHAM ||
			cd->bus->ic_type == IC_TYPE_MARSEILLE) {
		ret = ts_test_read(ts_test, flag_addr, frame_buf, FRAME_DATA_MAX_SIZE);
		if (ret < 0)
			goto free_exit;

		if (checksum_cmp(frame_buf, cd->ic_info.misc.frame_data_head_len, CHECKSUM_MODE_U8_LE)) {
			ts_err("frame head checksum error");
			ret = -EAGAIN;
			goto free_exit;
		}

		frame_len = le16_to_cpup((__le16 *)&frame_buf[3]);
		if (checksum_cmp(frame_buf, frame_len, CHECKSUM_MODE_U16_LE)) {
			ts_err("frame body checksum error");
			ret = -EAGAIN;
			goto free_exit;
		}

		offset += cd->ic_info.misc.frame_data_head_len;
		offset += cd->ic_info.misc.fw_attr_len;
		offset += cd->ic_info.misc.fw_log_len;
		if (data_type == GTP_MUTUAL_RAW || data_type == GTP_MUTUAL_DIFF) {
			offset += 8;
		} else {
			offset += cd->ic_info.misc.mutual_struct_len;
			offset += 10;
		}
		memcpy(data, frame_buf + offset, data_num * 2);
	} else {
		ret = ts_test_read(ts_test, data_addr, data, data_num * 2);
	}

free_exit:
	kfree(frame_buf);
	return ret;
}

static int goodix_cache_rawdata(struct goodix_ts_test *ts_test)
{
	int ret;
	int i,j;
	u8 val;
	u32 sen_num = ts_test->test_params.sen_num;
	u32 drv_num = ts_test->test_params.drv_num;
	u32 data_size = sen_num * drv_num;
	u32 flag_addr = ts_test->ts->ic_info.misc.touch_data_addr;

	if (ts_test->ts->bus->ic_type == IC_TYPE_BERLIN_D ||
			ts_test->ts->bus->ic_type == IC_TYPE_NOTTINGHAM ||
			ts_test->ts->bus->ic_type == IC_TYPE_MARSEILLE)
		flag_addr = ts_test->ts->ic_info.misc.frame_data_addr;

	for (i = 0; i < DISCARD_FRAMES; i++) {
		val = 0;
		ts_test_write(ts_test, flag_addr, &val, 1);
		msleep(20);
	}

	for (i = 0; i < TOTAL_FRAME_NUM; i++) {
		for (j = 0; j < GOODIX_RETRY_3; j++) {
			ret = goodix_poll_rawdata(ts_test,
				GTP_MUTUAL_RAW, (u8 *)ts_test->rawdata[i].data, data_size);
			if (ret == 0) {
				ts_test->rawdata[i].size = data_size;
				goodix_rotate_abcd2cbad(drv_num, sen_num, ts_test->rawdata[i].data);
				break;
			}
		}
	}

	return ret;
}

static void goodix_cache_deltadata(struct goodix_ts_test *ts_test)
{
	u32 data_size;
	int tx = ts_test->test_params.drv_num;
	int i;
	int j;
	int max_val;
	int raw;
	int temp;

	for (i = 0; i < TOTAL_FRAME_NUM; i++) {
		data_size = ts_test->rawdata[i].size;
		if (data_size == 0)
			continue;
		for (j = 0; j < data_size; j++) {
			raw = ts_test->rawdata[i].data[j];
			max_val = 0;
			/* calcu delta with above node */
			if (j - tx >= 0) {
				temp = ts_test->rawdata[i].data[j - tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with bellow node */
			if (j + tx < data_size) {
				temp = ts_test->rawdata[i].data[j + tx];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with left node */
			if (j % tx) {
				temp = ts_test->rawdata[i].data[j - 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			/* calcu delta with right node */
			if ((j + 1) % tx) {
				temp = ts_test->rawdata[i].data[j + 1];
				temp = ABS(temp - raw);
				max_val = MAX(max_val, temp);
			}
			temp = max_val * 1000 / raw;
			ts_test->accord_arr[i].data[j] = (temp > 32767) ? 32767 : temp;
		}
		ts_test->accord_arr[i].size = data_size;
	}
}

static int goodix_cache_self_rawdata(struct goodix_ts_test *ts_test)
{
	int i;
	int ret;
	u32 sen_num = ts_test->test_params.sen_num;
	u32 drv_num = ts_test->test_params.drv_num;
	u32 data_size = sen_num + drv_num;

	for (i = 0; i < GOODIX_RETRY_3; i++) {
		ret = goodix_poll_rawdata(ts_test,
			GTP_SELF_RAW, (u8 *)ts_test->self_rawdata.data, data_size);
		if (ret == 0) {
			ts_test->self_rawdata.size = data_size;
			break;
		}
	}

	return ret;
}

static int goodix_cache_noisedata(struct goodix_ts_test *ts_test)
{
	int ret;
	int i,j;
	int cnt;
	u8 val;
	struct goodix_ts_cmd temp_cmd;
	struct goodix_ts_core *cd = ts_test->ts;
	u32 sen_num = ts_test->test_params.sen_num;
	u32 drv_num = ts_test->test_params.drv_num;
	u32 data_size = sen_num * drv_num;
	u32 flag_addr = ts_test->ts->ic_info.misc.touch_data_addr;

	if (cd->bus->ic_type == IC_TYPE_BERLIN_D ||
			cd->bus->ic_type == IC_TYPE_NOTTINGHAM ||
			cd->bus->ic_type == IC_TYPE_MARSEILLE) {
		flag_addr = ts_test->ts->ic_info.misc.frame_data_addr;
		temp_cmd.cmd = 0x90;
		temp_cmd.data[0] = 0x82;
		temp_cmd.len = 5;
		ret = ts_test_send_cmd(ts_test, &temp_cmd);
		if (ret < 0) {
			ts_err("switch diffdata mode failed, exit!");
			return ret;
		}
	}

	for (i = 0; i < DISCARD_FRAMES; i++) {
		val = 0;
		ts_test_write(ts_test, flag_addr, &val, 1);
		msleep(20);
	}

	for (cnt = 0; cnt < NOISEDATA_TEST_TIMES; cnt++) {
		for (j = 0; j < GOODIX_RETRY_3; j++) {
			ret = goodix_poll_rawdata(ts_test,
				GTP_MUTUAL_DIFF, (u8 *)ts_test->noisedata[cnt].data, data_size);
			if (ret == 0) {
				ts_test->noisedata[cnt].size = data_size;
				goodix_rotate_abcd2cbad(drv_num, sen_num, ts_test->noisedata[cnt].data);
				for (i = 0; i < data_size; i++)
					ts_test->noisedata[cnt].data[i] = ABS(ts_test->noisedata[cnt].data[i]);
				break;
			}
		}
	}

	return ret;
}

static int goodix_cache_self_noisedata(struct goodix_ts_test *ts_test)
{
	int ret;
	int i,j;
	u32 sen_num = ts_test->test_params.sen_num;
	u32 drv_num = ts_test->test_params.drv_num;
	u32 data_size = sen_num + drv_num;

	for (j = 0; j < GOODIX_RETRY_3; j++) {
		ret = goodix_poll_rawdata(ts_test,
			GTP_SELF_DIFF, (u8 *)ts_test->self_noisedata.data, data_size);
		if (ret == 0) {
			ts_test->self_noisedata.size = data_size;
			for (i = 0; i < data_size; i++) {
				ts_test->self_noisedata.data[i] = ABS(ts_test->self_noisedata.data[i]);
			}
			break;
		}
	}

	return ret;
}

static int goodix_analysis_rawdata(struct goodix_ts_test *ts_test)
{
	int i;
	int j;
	bool fail_flag = false;
	int err_cnt = 0;
	int times = TOTAL_FRAME_NUM;
	s16 val;
	u32 data_size = ts_test->rawdata[0].size;

	for (i = 0; i < times; i++) {
		for (j = 0; j < data_size; j++) {
			val = ts_test->rawdata[i].data[j];
			if (val < ts_test->test_params.min_limits[j]) {
				fail_flag = true;
				ts_test->open_res.beyond_min_limit_cnt[j]++;
			}
			if (val > ts_test->test_params.max_limits[j]) {
				fail_flag = true;
				ts_test->open_res.beyond_max_limit_cnt[j]++;
			}
		}
		if (fail_flag)
			err_cnt++;
		fail_flag = false;
	}

	if (err_cnt * 100 > times * 100 * 9 / 10) {
		ts_err("rawdata more than 90%%(%d) fail, test fail", err_cnt);
		return -1;
	}
	if (err_cnt * 100 > times * 100 * 1 / 10) {
		ts_info("rawdata more than 10%%(%d) fail, need test again", err_cnt);
		return 1;
	}

	ts_info("rawdata less than 10%%(%d) fail, test pass", err_cnt);
	return 0;
}

static int goodix_analysis_deltadata(struct goodix_ts_test *ts_test)
{
	int i;
	int j;
	int ret = 0;
	s32 val;
	u32 data_size = ts_test->accord_arr[0].size;

	for (i = 0; i < TOTAL_FRAME_NUM; i++) {
		for (j = 0; j < data_size; j++) {
			val = ts_test->accord_arr[i].data[j];
			if (val > ts_test->test_params.deviation_limits[j]) {
				ts_test->open_res.beyond_accord_limit_cnt[j]++;
				ret = -EINVAL;
			}
		}
	}

	return ret;
}

static int goodix_analysis_self_rawdata(struct goodix_ts_test *ts_test)
{
	int i;
	s16 val;
	u32 data_size = ts_test->self_rawdata.size;

	for (i = 0; i < data_size; i++) {
		val = ts_test->self_rawdata.data[i];
		if (val < ts_test->test_params.self_min_limits[i] ||
			val > ts_test->test_params.self_max_limits[i]) {
			ts_err("self_rawdata isn't in range, val:%d threshold:[%d,%d]",
				val, ts_test->test_params.self_min_limits[i],
				ts_test->test_params.self_max_limits[i]);
			return -EINVAL;
		}
	}

	return 0;
}

static int goodix_analysis_noisedata(struct goodix_ts_test *ts_test)
{
	int cnt;
	int i;
	bool fail_flag = false;
	int err_cnt = 0;
	int times = NOISEDATA_TEST_TIMES;
	s16 val;
	u32 data_size = ts_test->noisedata[0].size;

	for (cnt = 0; cnt < times; cnt++) {
		for (i = 0; i < data_size; i++) {
			val = ts_test->noisedata[cnt].data[i];
			if (val > ts_test->test_params.noise_threshold)
				fail_flag = true;
		}
		if (fail_flag)
			err_cnt++;
		fail_flag = false;
	}

	if (err_cnt > 0) {
		ts_err("noisedata have %d frames out of range", err_cnt);
		return -EINVAL;
	}

	return 0;
}

static int goodix_analysis_self_noisedata(struct goodix_ts_test *ts_test)
{
	int i;
	s16 val;
	u32 data_size = ts_test->self_noisedata.size;

	for (i = 0; i < data_size; i++) {
		val = ts_test->self_noisedata.data[i];
		if (val > ts_test->test_params.self_noise_threshold) {
			ts_err("self noisedata isn't in range, val:%d threshold:[0,%d]",
				val, ts_test->test_params.self_noise_threshold);
			return -EINVAL;
		}
	}

	return 0;
}

static void goodix_capacitance_test(struct goodix_ts_test *ts_test)
{
	int ret;
	int retry = 0;

	ts_info("---------------------- cap_test begin ----------------------");
	ret = goodix_cap_test_prepare(ts_test);
	if (ret < 0) {
		ts_err("cap_test prepare failed, exit");
		goto exit;
	}
	ts_info("cap rawdata prepare OK");

rawdata_test_start:
	/* obtain rawdata */
	ret = goodix_cache_rawdata(ts_test);
	if (ret < 0) {
		ts_err("Failed to read capdata");
	} else {
		ts_info("get rawdata finish, start analysis");
		ret = goodix_analysis_rawdata(ts_test);
		if (ret < 0) {
			ts_test->test_result[GTP_CAP_TEST] = GTP_TEST_PANEL_ERR;
		} else if (ret == 0) {
			ts_test->test_result[GTP_CAP_TEST] = GTP_TEST_OK;
		} else if (retry < GOODIX_RETRY_3) {
			retry++;
			ts_info("rawdata test retry[%d]", retry);
			goto rawdata_test_start;
		} else {
			ts_test->test_result[GTP_CAP_TEST] = GTP_TEST_PANEL_ERR;
		}
	}

	/* obtain delta_data */
	goodix_cache_deltadata(ts_test);
	ts_info("get deltadata finish, start analysis");
	ret = goodix_analysis_deltadata(ts_test);
	if (ret < 0)
		ts_test->test_result[GTP_DELTA_TEST] = GTP_TEST_PANEL_ERR;
	else
		ts_test->test_result[GTP_DELTA_TEST] = GTP_TEST_OK;

	/* obtain self_rawdata */
	if (ts_test->test_params.test_items[GTP_SELFCAP_TEST]) {
		ret = goodix_cache_self_rawdata(ts_test);
		if (ret < 0) {
			ts_err("Failed to read self_capdata");
		} else {
			ts_info("get self_rawdata finish, start analysis");
			ret = goodix_analysis_self_rawdata(ts_test);
			if (ret < 0)
				ts_test->test_result[GTP_SELFCAP_TEST] = GTP_TEST_PANEL_ERR;
			else
				ts_test->test_result[GTP_SELFCAP_TEST] = GTP_TEST_OK;
		}
	}

	/* obtain noisedata */
	if (ts_test->test_params.test_items[GTP_NOISE_TEST]) {
		ret = goodix_cache_noisedata(ts_test);
		if (ret < 0) {
			ts_err("Failed to read diffdata");
		} else {
			ts_info("get noisedata finish, start analysis");
			ret = goodix_analysis_noisedata(ts_test);
			if (ret < 0)
				ts_test->test_result[GTP_NOISE_TEST] = GTP_TEST_PANEL_ERR;
			else
				ts_test->test_result[GTP_NOISE_TEST] = GTP_TEST_OK;
		}
	}

	/* obtain self_noisedata */
	if (ts_test->test_params.test_items[GTP_SELFNOISE_TEST]) {
		ret = goodix_cache_self_noisedata(ts_test);
		if (ret < 0) {
			ts_err("Failed to read self_noisedata");
		} else {
			ts_info("get self_noisedata finish, start analysis");
			ret = goodix_analysis_self_noisedata(ts_test);
			if (ret < 0)
				ts_test->test_result[GTP_SELFNOISE_TEST] = GTP_TEST_PANEL_ERR;
			else
				ts_test->test_result[GTP_SELFNOISE_TEST] = GTP_TEST_OK;
		}
	}

exit:
	goodix_cap_test_finish(ts_test);
}

static void goodix_version_test(struct goodix_ts_test *ts_test)
{
	struct goodix_ts_core *cd = ts_test->ts;
	u8 rom_pid[8] = {0};

	memcpy(rom_pid, cd->fw_version.rom_pid, sizeof(cd->fw_version.rom_pid));
	sprintf(ts_test->cur_version, "%s-%02x.%02x.%02x_%s_%02x.%02x.%02x.%02x",
		rom_pid, cd->fw_version.rom_vid[0], cd->fw_version.rom_vid[1],
		cd->fw_version.rom_vid[2], cd->fw_version.patch_pid,
		cd->fw_version.patch_vid[0], cd->fw_version.patch_vid[1],
		cd->fw_version.patch_vid[2], cd->fw_version.patch_vid[3]);

	ts_test->test_result[GTP_VERSION_TEST] = GTP_TEST_OK;
}

#define FT_TEST_PASS_FLAG			0xAA
static void goodix_check_key_info_test(struct goodix_ts_test *ts_test)
{
	int ret;
	u32 chip_pid_addr;
	u32 chip_info_addr;
	u8 chip_info_buf[CHIP_KEY_INFO_MAX_LEN];
	u8 test_result = 0;
	u16 chksum;
	u16 sum = 0;
	int i;
	struct goodix_ts_core *cd = ts_test->ts;

	//check chip pid
	if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
		chip_pid_addr = 0x3F051;
	else
		chip_pid_addr = 0x1F031;

	for (i = 0; i < 3; i++) {
		ret = cd->hw_ops->read_flash(cd, chip_pid_addr, ts_test->chip_pid, sizeof(ts_test->chip_pid));
		if (ret == 0)
			break;
		msleep(100);
	}
	if (ret < 0) {
		ts_err("read chip pid from flash failed");
		return;
	}

	if (memcmp(ts_test->chip_pid, cd->fw_version.patch_pid, 7)) {
		test_result |= 0x01; // chip pid check failed
		ts_err("chip_pid:%s != patch_pid:%s", ts_test->chip_pid, cd->fw_version.patch_pid);
	}

	//read chip load info for berlin
	if (cd->bus->ic_type == IC_TYPE_BERLIN_B ||
			cd->bus->ic_type == IC_TYPE_BERLIN_D ||
			cd->bus->ic_type == IC_TYPE_NOTTINGHAM ||
			cd->bus->ic_type == IC_TYPE_MARSEILLE) {
		if (cd->bus->ic_type == IC_TYPE_BERLIN_B)
			chip_info_addr = 0x3F300;
		else
			chip_info_addr = 0x1F300;

		for (i = 0; i < 3; i++) {
			ret = cd->hw_ops->read_flash(cd, chip_info_addr, chip_info_buf, sizeof(chip_info_buf));
			if (ret == 0)
				break;
			msleep(100);
		}
		if (ret < 0) {
			ts_err("read chip info from flash failed");
			return;
		}
		chksum = chip_info_buf[CHIP_KEY_INFO_MAX_LEN - 2] + ((u16)chip_info_buf[CHIP_KEY_INFO_MAX_LEN - 1] << 8);
		for (i = 0; i < CHIP_KEY_INFO_MAX_LEN - 2; i++)
			sum += chip_info_buf[i];
		if ((sum & 0x00FF) != (chksum & 0x00FF)) {
			ts_err("chip info checksum error");
			return;
		}

		ts_test->chip_info.hw_ver = chip_info_buf[0];
		ts_test->chip_info.package_site = chip_info_buf[1];

		if (chip_info_buf[2 + CHIP_UID_MAX_LEN] == FT_TEST_PASS_FLAG) {
			memcpy(ts_test->chip_info.chip_uid, &chip_info_buf[2], CHIP_UID_MAX_LEN);//UID
			ts_test->chip_info.ft_test_result = chip_info_buf[2 + CHIP_UID_MAX_LEN];
		} else if (chip_info_buf[2] == FT_TEST_PASS_FLAG){
			ts_test->chip_info.ft_test_result = chip_info_buf[2];
			memcpy(ts_test->chip_info.chip_uid, &chip_info_buf[3], CHIP_UID_MAX_LEN);//UID
		} else {
			test_result |= 0x02;
			ts_err("chip info test failed");
		}

		ts_test->chip_info.substrate_tech = chip_info_buf[3 + CHIP_UID_MAX_LEN];
		ts_test->chip_info.mcu_fab = chip_info_buf[4 + CHIP_UID_MAX_LEN];
		ts_test->chip_info.vaild_len = chip_info_buf[CHIP_KEY_INFO_MAX_LEN - 3];
		ts_test->chip_info.check_sum = chksum;
	}

	if (test_result > 0)
		ts_test->test_result[GTP_CHIP_KEY_INFO_TEST] = GTP_TEST_PANEL_ERR;
	else
		ts_test->test_result[GTP_CHIP_KEY_INFO_TEST] = GTP_TEST_OK;
}

static void goodix_custom_info_test(struct goodix_ts_test *ts_test)
{
	int ret;
	struct goodix_ts_core *cd = ts_test->ts;
	u32 custom_info_addr;
	u32 custom_info_ram_addr;
	u32 custom_info_len = 16;

	if (cd->bus->ic_type == IC_TYPE_BERLIN_A) {
		custom_info_addr = 0x3F000;
		custom_info_ram_addr = 0x10028;
	} else if (cd->bus->ic_type == IC_TYPE_BERLIN_B) {
		custom_info_addr = 0x7F000;
		custom_info_ram_addr = 0x10030;
	} else {
		custom_info_addr = 0x1E000;
		custom_info_ram_addr = 0x10030;
	}

	ret = cd->hw_ops->read_flash(cd, custom_info_addr, ts_test->custom_info_flash, custom_info_len);
	if (ret < 0) {
		ts_err("read custom info from flash failed");
		return;
	}
	ret = cd->hw_ops->read(cd, custom_info_ram_addr, ts_test->custom_info_sram, custom_info_len);
	if (ret < 0) {
		ts_err("read custom info from sram failed");
		return;
	}

	ts_test->test_result[GTP_CUSTOM_INFO_TEST] = GTP_TEST_OK;
}

char *goodix_strncat(char *dest, char *src, size_t dest_size)
{
	size_t dest_len = 0;

	dest_len = strnlen(dest, dest_size);
	return strncat(&dest[dest_len], src, dest_size - dest_len - 1);
}

char *goodix_strncatint(char *dest, int src, char *format, size_t dest_size)
{
	char src_str[MAX_STR_LEN] = {0};

	snprintf(src_str, MAX_STR_LEN, format, src);
	return goodix_strncat(dest, src_str, dest_size);
}

static void goodix_data_cal(s16 *data, size_t data_size, s16 *stat_result)
{
	int i = 0;
	s16 avg = 0;
	s16 min = 0;
	s16 max = 0;
	long long sum = 0;

	min = data[0];
	max = data[0];
	for (i = 0; i < data_size; i++) {
		sum += data[i];
		if (max < data[i])
			max = data[i];
		if (min > data[i])
			min = data[i];
	}
	avg = div_s64(sum, data_size);
	stat_result[0] = avg;
	stat_result[1] = max;
	stat_result[2] = min;
}

static int goodix_save_test_config(struct goodix_ts_test *ts_test)
{
	int ret = 0;
	int i;
	struct goodix_ic_config *cfg = &ts_test->test_config;

	if (cfg->len <= 0)
		return 0;

	proc_append_string("<OrderConfig>\n");
	for (i = 0; i < cfg->len; i++)
		proc_append_string("0x%02X,", cfg->data[i]);
	proc_append_string("\n</OrderConfig>\n");
	return ret;
}

static int goodix_save_header(struct goodix_ts_test *ts_test)
{
	int i;
	bool result = false;
	struct goodix_ts_core *ts = ts_test->ts;

	proc_append_string("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
	proc_append_string("<TESTLOG>\n");
	proc_append_string("<Header>\n");

	/* sava test result */
	for (i = 0; i < MAX_TEST_ITEMS; i++) {
		if (ts_test->test_params.test_items[i]) {
			if (ts_test->test_result[i] != GTP_TEST_OK) {
				result = true;
				break;
			}
		}
	}
	proc_append_string("<Result>%s</Result>\n", result ? "NG" : "OK");
	proc_append_string("<DeviceType>GT%s</DeviceType>\n<SensorId>%d</SensorId>\n",
			ts->fw_version.patch_pid, ts->fw_version.sensor_id);
	auto_test_result = !result;
	ts_info("GT%s TP test total result: %d", ts->fw_version.patch_pid, auto_test_result);

	/* save test config */
	goodix_save_test_config(ts_test);

	proc_append_string("</Header>\n");

	/* item list */
	proc_append_string("<ItemList>\n");

	if (ts_test->test_params.test_items[GTP_VERSION_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_VERSION_TEST])
			proc_append_string("<Item name=\"Version Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Version Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Version Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_CHIP_KEY_INFO_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_CHIP_KEY_INFO_TEST])
			proc_append_string("<Item name=\"Check Chip Key Info\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Check Chip Key Info\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Check Chip Key Info\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_CUSTOM_INFO_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_CUSTOM_INFO_TEST])
			proc_append_string("<Item name=\"Custom Info Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Custom Info Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Custom Info Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_CAP_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_CAP_TEST])
			proc_append_string("<Item name=\"Rawdata MAX/MIN Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Rawdata MAX/MIN Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Rawdata MAX/MIN Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_DELTA_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_DELTA_TEST])
			proc_append_string("<Item name=\"Rawdata Adjcent Deviation Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Rawdata Adjcent Deviation Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Rawdata Adjcent Deviation Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_NOISE_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_NOISE_TEST])
			proc_append_string("<Item name=\"Diffdata Jitter Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Diffdata Jitter Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Diffdata Jitter Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_SELFNOISE_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_SELFNOISE_TEST])
			proc_append_string("<Item name=\"Self Diffdata Jitter Limit Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Self Diffdata Jitter Limit Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Self Diffdata Jitter Limit Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_SELFCAP_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_SELFCAP_TEST])
			proc_append_string("<Item name=\"Self Rawdata Upper Limit Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Self Rawdata Upper Limit Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Self Rawdata Upper Limit Test\" result=\"NG\"/>\n");
		}
	}

	if (ts_test->test_params.test_items[GTP_SHORT_TEST]) {
		if (GTP_TEST_OK == ts_test->test_result[GTP_SHORT_TEST])
			proc_append_string("<Item name=\"Short Test\" result=\"OK\"/>\n");
		else {
				proc_append_string("<Item name=\"Short Test\" result=\"NG\"/>\n");
				ts_err("<Item name=\"Short Test\" result=\"NG\"/>\n");
		}
	}

	proc_append_string("</ItemList>\n");
	return 0;
}

static int goodix_save_items(struct goodix_ts_test *ts_test)
{
	int i;
	int tx = ts_test->test_params.drv_num;
	int rx = ts_test->test_params.sen_num;
	int chn1;
	int chn2;
	int r;

	proc_append_string("<TestItems>\n");

	if (ts_test->test_params.test_items[GTP_VERSION_TEST]) {
		proc_append_string("<Item name=\"Version Test\">\n");
		proc_append_string("<CurVerDataString>%s</CurVerDataString>\n", ts_test->cur_version);
		proc_append_string("</Item>\n");
	}

	if (ts_test->test_params.test_items[GTP_CHIP_KEY_INFO_TEST]) {
		proc_append_string("<Item name=\"Check Chip Key Info\">\n");
		if (GTP_TEST_OK == ts_test->test_result[GTP_CHIP_KEY_INFO_TEST])
			proc_append_string("<TestInfo checkPID=\"true\"/>\n");
		else
			proc_append_string("<TestInfo checkPID=\"false\"/>\n");
		proc_append_string("<ChipInfo Pid=\"%s\" HwVer=\"0x%02x\" PackageSite=\"0x%02x\" FtTestResult=\"0x%02x\" SubstrateTechnology=\"0x%02x\" McuFab=\"0x%02x\">\n",
			ts_test->chip_pid, ts_test->chip_info.hw_ver,
			ts_test->chip_info.package_site, ts_test->chip_info.ft_test_result,
			ts_test->chip_info.substrate_tech, ts_test->chip_info.mcu_fab);
		proc_append_string("<ChipUID>");
		for (i = 0; i < sizeof(ts_test->chip_info.chip_uid); i++)
			proc_append_string("%02x", ts_test->chip_info.chip_uid[i]);
		proc_append_string("</ChipUID>\n");
		proc_append_string("</ChipInfo>\n");
		proc_append_string("</Item>\n");
	}

	if (ts_test->test_params.test_items[GTP_CUSTOM_INFO_TEST]) {
		proc_append_string("<Item name=\"Custom Info Test\">\n");
		proc_append_string("<CustomInfoInFlash>");
		for (i = 0; i < sizeof(ts_test->custom_info_flash); i++)
			proc_append_string("0x%02x,", ts_test->custom_info_flash[i]);
		proc_append_string("</CustomInfoInFlash>\n");
		proc_append_string("<CustomInfoInRam>");
		for (i = 0; i < sizeof(ts_test->custom_info_sram); i++)
			proc_append_string("0x%02x,", ts_test->custom_info_sram[i]);
		proc_append_string("</CustomInfoInRam>\n");
		proc_append_string("</Item>\n");
	}

	/* save short result */
	if (ts_test->test_params.test_items[GTP_SHORT_TEST]) {
		proc_append_string("<Item name=\"Short Test\">\n");
		proc_append_string("<ShortNum>%d</ShortNum>\n", ts_test->short_res.short_num);
		for (i = 0; i < ts_test->short_res.short_num; i++) {
			chn1 = ts_test->short_res.short_msg[4 * i];
			chn2 = ts_test->short_res.short_msg[4 * i + 1];
			r = (ts_test->short_res.short_msg[4 * i + 2] << 8) +
				ts_test->short_res.short_msg[4 * i + 3];
			if (chn1 == CHN_VDD)
				proc_append_string("<ShortMess Chn1=\"VDD\" ");
			else if (chn1 == CHN_GND)
				proc_append_string("<ShortMess Chn1=\"GND\" ");
			else if (chn1 & DRV_CHANNEL_FLAG)
				proc_append_string("<ShortMess Chn1=\"Tx%d\" ",
						chn1 & 0x7f);
			else
				proc_append_string("<ShortMess Chn1=\"Rx%d\" ",
						chn1 & 0x7f);
			if (chn2 == CHN_VDD)
				proc_append_string("Chn2=\"VDD\" ShortResistor= \"%dKom\"/>\n", r);
			else if (chn2 == CHN_GND)
				proc_append_string("Chn2=\"GND\" ShortResistor= \"%dKom\"/>\n", r);
			else if (chn2 & DRV_CHANNEL_FLAG)
				proc_append_string("Chn2=\"Tx%d\" ShortResistor= \"%dKom\"/>\n",
						chn2 & 0x7f, r);
			else
				proc_append_string("Chn2=\"Rx%d\" ShortResistor= \"%dKom\"/>\n",
						chn2 & 0x7f, r);
		}
		proc_append_string("</Item>\n");
	}

	/* rawdata max limit */
	proc_append_string("<Item name=\"Rawdata Test Sets\">\n");
	proc_append_string("<TotalFrameCnt>%d</TotalFrameCnt>\n", TOTAL_FRAME_NUM);
	proc_append_string("<MaxRawLimit>\n");
	for (i = 0; i < tx * rx; i++) {
		proc_append_string("%d,", ts_test->test_params.max_limits[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	proc_append_string("</MaxRawLimit>\n");
	/* BeyondRawdataUpperLimit */
	proc_append_string("<BeyondRawdataUpperLimitCnt>\n");
	for (i = 0; i < tx * rx; i++) {
		proc_append_string("%d,", ts_test->open_res.beyond_max_limit_cnt[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	proc_append_string("</BeyondRawdataUpperLimitCnt>\n");

	/* rawdata min limit */
	proc_append_string("<MinRawLimit>\n");
	for (i = 0; i < tx * rx; i++) {
		proc_append_string("%d,", ts_test->test_params.min_limits[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	proc_append_string("</MinRawLimit>\n");
	/* BeyondRawdataLower limit */
	proc_append_string("<BeyondRawdataLowerLimitCnt>\n");
	for (i = 0; i < tx * rx; i++) {
		proc_append_string("%d,", ts_test->open_res.beyond_min_limit_cnt[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	proc_append_string("</BeyondRawdataLowerLimitCnt>\n");

	/* Max Accord limit */
	proc_append_string("<MaxAccordLimit>\n");
	for (i = 0; i < tx * rx; i++) {
		proc_append_string("%d,", ts_test->test_params.deviation_limits[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	proc_append_string("</MaxAccordLimit>\n");
	/* BeyondAccordLimitCnt */
	proc_append_string("<BeyondAccordLimitCnt>\n");
	for (i = 0; i < tx * rx; i++) {
		proc_append_string("%d,", ts_test->open_res.beyond_accord_limit_cnt[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	proc_append_string("</BeyondAccordLimitCnt>\n");
	proc_append_string("</Item>\n");

	/* save noise limit */
	if (ts_test->test_params.test_items[GTP_NOISE_TEST]) {
		proc_append_string("<Item name=\"Diffdata Test Sets\">\n");
		proc_append_string("<TotalFrameCnt>%d</TotalFrameCnt>\n",
				NOISEDATA_TEST_TIMES);
		proc_append_string("<MaxJitterLimit>%d</MaxJitterLimit>\n",
				ts_test->test_params.noise_threshold);
		proc_append_string("</Item>\n");
	}

	/* save self rawdata limit */
	if (ts_test->test_params.test_items[GTP_SELFCAP_TEST]) {
		proc_append_string("<Item name=\"Self Rawdata Test Sets\">\n");
		proc_append_string("<TotalFrameCnt>1</TotalFrameCnt>\n");
		proc_append_string("<MaxRawLimit>\n");
		for (i = 0; i < tx + rx; i++) {
			proc_append_string("%d,",
				ts_test->test_params.self_max_limits[i]);
			if ((i + 1) % tx == 0)
				proc_append_string("\n");
		}
		if ((tx + rx) % tx != 0)
			proc_append_string("\n");
		proc_append_string("</MaxRawLimit>\n");
		proc_append_string("<MinRawLimit>\n");
		for (i = 0; i < tx + rx; i++) {
			proc_append_string("%d,",
				ts_test->test_params.self_min_limits[i]);
			if ((i + 1) % tx == 0)
				proc_append_string("\n");
		}
		if ((tx + rx) % tx != 0)
			proc_append_string("\n");
		proc_append_string("</MinRawLimit>\n");
		proc_append_string("</Item>\n");
	}

	/* save selfnoise limit */
	if (ts_test->test_params.test_items[GTP_SELFNOISE_TEST]) {
		proc_append_string("<Item name=\"Self Diffdata Test Sets\">\n");
		proc_append_string("<TotalFrameCnt>1</TotalFrameCnt>\n");
		proc_append_string("<MaxJitterLimit>%d</MaxJitterLimit>\n",
				ts_test->test_params.self_noise_threshold);
		proc_append_string("</Item>\n");
	}

	proc_append_string("</TestItems>\n");
	return 0;
}

static int goodix_save_rawdata(struct goodix_ts_test *ts_test)
{
	int i;
	int j;
	s16 stat_result[3];
	int tx = ts_test->test_params.drv_num;
	int rx = ts_test->test_params.sen_num;
	int len = tx * rx;

	proc_append_string("<RawDataRecord>\n");
	for (i = 0; i < TOTAL_FRAME_NUM; i++) {
		goodix_data_cal(ts_test->rawdata[i].data, len, stat_result);
		proc_append_string("<DataContent No=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
			i, len, stat_result[1], stat_result[2], stat_result[0]);
		for (j = 0; j < len; j++) {
			proc_append_string("%d,", ts_test->rawdata[i].data[j]);
			if ((j + 1) % tx == 0)
				proc_append_string("\n");
		}
		proc_append_string("</DataContent>\n");
		goodix_data_cal(ts_test->accord_arr[i].data, len, stat_result);
		proc_append_string("<RawAccord No=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
			i, len, stat_result[1], stat_result[2], stat_result[0]);
		for (j = 0; j < len; j++) {
			proc_append_string("%d,", ts_test->accord_arr[i].data[j]);
			if ((j + 1) % tx == 0)
				proc_append_string("\n");
		}
		proc_append_string("</RawAccord>\n");
	}

	proc_append_string("</RawDataRecord>\n");
	return 0;
}

static int goodix_save_noise_data(struct goodix_ts_test *ts_test)
{
	int i;
	int j;
	s16 stat_result[3];
	int tx = ts_test->test_params.drv_num;
	int rx = ts_test->test_params.sen_num;
	int len = tx * rx;

	proc_append_string("<DiffDataRecord>\n");
	for (i = 0; i < NOISEDATA_TEST_TIMES; i++) {
		goodix_data_cal(ts_test->noisedata[i].data, len, stat_result);
		proc_append_string("<DataContent No=\"%d\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
			i, len, stat_result[1], stat_result[2], stat_result[0]);
		for (j = 0; j < len; j++) {
			proc_append_string("%d,", ts_test->noisedata[i].data[j]);
			if ((j + 1) % tx == 0)
				proc_append_string("\n");
		}
		proc_append_string("</DataContent>\n");
	}

	proc_append_string("</DiffDataRecord>\n");
	return 0;
}

static int goodix_save_self_data(struct goodix_ts_test *ts_test,
		s16 *src_data, u8 *title, int len)
{
	int i;
	s16 stat_result[3];
	int tx = ts_test->test_params.drv_num;

	proc_append_string("<%s>\n", title);
	goodix_data_cal(src_data, len, stat_result);
	proc_append_string("<DataContent No=\"0\" DataCount=\"%d\" Maximum=\"%d\" Minimum=\"%d\" Average=\"%d\">\n",
		len, stat_result[1], stat_result[2], stat_result[0]);
	for (i = 0; i < len; i++) {
		proc_append_string("%d,", src_data[i]);
		if ((i + 1) % tx == 0)
			proc_append_string("\n");
	}
	if (len % tx != 0)
		proc_append_string("\n");
	proc_append_string("</DataContent>\n");
	proc_append_string("</%s>\n", title);
	return 0;
}

static int goodix_save_data(struct goodix_ts_test *ts_test)
{
	int ret;

	proc_append_string("<DataRecord>\n");

	ret = goodix_save_rawdata(ts_test);

	if (ts_test->test_params.test_items[GTP_NOISE_TEST]) {
		ret = goodix_save_noise_data(ts_test);
	}

	if (ts_test->test_params.test_items[GTP_SELFCAP_TEST]) {
		ret = goodix_save_self_data(ts_test,
				ts_test->self_rawdata.data,
				"selfDataRecord",
				ts_test->self_rawdata.size);
	}

	if (ts_test->test_params.test_items[GTP_SELFNOISE_TEST]) {
		ret = goodix_save_self_data(ts_test,
				ts_test->self_noisedata.data,
				"selfDiffDataRecord",
				ts_test->self_noisedata.size);
	}

	proc_append_string("</DataRecord>\n");
	return ret;
}

/* save end tag in csv file */
static int goodix_save_tail(struct goodix_ts_test *ts_test)
{
	proc_append_string("</TESTLOG>\n");
	return 0;
}

static void goodix_save_result_data(struct goodix_ts_test *ts_test)
{
	int ret;

	/* save header */
	ret = goodix_save_header(ts_test);
	if (ret < 0)
		return;

	/* save limits */
	ret = goodix_save_items(ts_test);
	if (ret < 0)
		return;

	/* save data */
	ret = goodix_save_data(ts_test);
	if (ret < 0)
		return;

	/* save tail */
	ret = goodix_save_tail(ts_test);
	if (ret < 0)
		return;
}

static int goodix_do_inspect(struct goodix_ts_core *cd)
{
	int ret;
	struct goodix_ts_test *ts_test = NULL;

	if (!cd) {
		ts_err("core_data or info is NULL");
		return -ENODEV;
	}

	ts_test = vzalloc(sizeof(*ts_test));
	if (!ts_test)
		return -ENOMEM;

	ts_test->ts = cd;
	ret = goodix_tptest_prepare(ts_test);
	if (ret < 0) {
		ts_err("Failed to prepare TP test, exit");
		goto exit_finish;
	}
	ts_info("TP test prepare OK");

	goodix_version_test(ts_test);
	goodix_check_key_info_test(ts_test);
	goodix_custom_info_test(ts_test);
	goodix_capacitance_test(ts_test);
	if (ts_test->test_params.test_items[GTP_SHORT_TEST])
		goodix_shortcircut_test(ts_test);
	goodix_save_result_data(ts_test);
	goodix_tptest_finish(ts_test);

exit_finish:
	vfree(ts_test);
	return ret;
}

static ssize_t auto_test_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
	struct seq_file *seq_file = (struct seq_file *)file->private_data;
	struct goodix_ts_core *cd = (struct goodix_ts_core *)seq_file->private;
	char tmp_buf[10];

	if (copy_from_user(tmp_buf, buf, count) != 0) {
		ts_err("copy from user failed");
		goto exit;
	}

	if (!(tmp_buf[0] == 1 || tmp_buf[0] == '1')) {
		ts_err("auto test input parameter error [%d]", tmp_buf[0]);
		goto exit;
	}

	output_offset = 0;
	vfree(output_buf);
	output_buf = (char *)vmalloc(MAX_PROC_OUTPUT_SIZE);

	goodix_do_inspect(cd);
exit:
	return count;
}

static void *seq_start(struct seq_file *s, loff_t *pos)
{
	if (*pos >= output_offset)
		return NULL;

	return output_buf + *pos;
}

static int seq_show(struct seq_file *s, void *v)
{
	//seq_printf(s, (u8 *)v);
	seq_printf(s, "%s", (u8 *)v);
	return 0;
}

static void *seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	*pos += output_offset;
	return NULL;
}

static void seq_stop(struct seq_file *s, void *v)
{
	if (s->read_pos >= output_offset) {
		output_offset = 0;
		vfree(output_buf);
	}
}

static const struct seq_operations seq_ops = {
	.start	= seq_start,
	.next	= seq_next,
	.stop	= seq_stop,
	.show	= seq_show
};

static int auto_test_open(struct inode *inode, struct file *file)
{
	struct seq_file *seq_file;
	int ret;

	ret = seq_open(file, &seq_ops);
	if (ret)
		return ret;

	seq_file = (struct seq_file *)file->private_data;
	if (seq_file) {
		seq_file->private = PDE_DATA(inode);
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops auto_test_ops = {
	.proc_open = auto_test_open,
	.proc_read = seq_read,
	.proc_write = auto_test_write,
	.proc_lseek = seq_lseek,
	.proc_release = seq_release,
};
#else
static const struct file_operations auto_test_ops = {
	.open = auto_test_open,
	.read = seq_read,
	.write = auto_test_write,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

static int auto_test_result_proc_show(struct seq_file *file, void *v)
{
	struct goodix_ts_core * core_data = (struct goodix_ts_core *)file->private;
	if (!core_data) {
	    ts_err("no goodix_ts_core set");
	    return -EIO;
	}

	seq_printf(file, "result=%d\n", auto_test_result);

	return 0;
}

static int auto_test_result_proc_open(struct inode *inode, struct file *file)
{
    return single_open_size(file, auto_test_result_proc_show, PDE_DATA(inode), PAGE_SIZE * 10);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0)
static const struct file_operations auto_test_result_ops = {
    .owner   = THIS_MODULE,
    .open    = auto_test_result_proc_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release,
};
#else
static const struct proc_ops auto_test_result_ops = {
    .proc_open = auto_test_result_proc_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};
#endif


int inspect_module_init(struct goodix_ts_core *core_data)
{
	struct proc_dir_entry *proc_entry;
	struct proc_dir_entry *result_proc_entry;

	proc_entry = proc_create_data("auto_test",
			0660, core_data->proc_dir_entry, &auto_test_ops, core_data);
	if (!proc_entry) {
		ts_err("failed to create proc entry");
		return -ENODEV;
	}

	result_proc_entry = proc_create_data("auto_test_result",
			0660, core_data->proc_dir_entry, &auto_test_result_ops, core_data);
	if (!result_proc_entry) {
		ts_err("failed to create auto_test_result proc entry");
		return -ENODEV;
	}

	ts_info("inspect module init success");
	return 0;
}

void inspect_module_exit(struct goodix_ts_core *core_data)
{
	ts_info("inspect module exit");
	remove_proc_entry("auto_test", core_data->proc_dir_entry);
}
