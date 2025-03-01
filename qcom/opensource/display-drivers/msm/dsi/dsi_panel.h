/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022-2023, Qualcomm Innovation Center, Inc. All rights reserved.
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 */

#ifndef _DSI_PANEL_H_
#define _DSI_PANEL_H_

#include <linux/of_device.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/backlight.h>
#include <drm/drm_panel.h>
#include <drm/msm_drm.h>
#include <drm/msm_drm_pp.h>

#include "dsi_defs.h"
#include "dsi_ctrl_hw.h"
#include "dsi_clk.h"
#include "dsi_pwr.h"
#include "dsi_parser.h"
#include "msm_drv.h"

#define MAX_BL_LEVEL 4096
#define MAX_BL_SCALE_LEVEL 1024
#define MAX_SV_BL_SCALE_LEVEL 65535
#define SV_BL_SCALE_CAP (MAX_SV_BL_SCALE_LEVEL * 4)
#define DSI_CMD_PPS_SIZE 135

#define DSI_CMD_PPS_HDR_SIZE 7
#define DSI_MODE_MAX 64

#define DSI_IS_FSC_PANEL(fsc_rgb_order) \
		(((!strcmp(fsc_rgb_order, "fsc_rgb")) || \
		(!strcmp(fsc_rgb_order, "fsc_rbg")) || \
		(!strcmp(fsc_rgb_order, "fsc_bgr")) || \
		(!strcmp(fsc_rgb_order, "fsc_brg")) || \
		(!strcmp(fsc_rgb_order, "fsc_gbr")) || \
		(!strcmp(fsc_rgb_order, "fsc_grb"))))

#define FSC_MODE_LABEL_SIZE	8

/*
 * Defining custom dsi msg flag.
 * Using upper byte of flag field for custom DSI flags.
 * Lower byte flags specified in drm_mipi_dsi.h.
 */
#define MIPI_DSI_MSG_ASYNC_OVERRIDE BIT(4)
#define MIPI_DSI_MSG_CMD_DMA_SCHED BIT(5)
#define MIPI_DSI_MSG_BATCH_COMMAND BIT(6)
#define MIPI_DSI_MSG_UNICAST_COMMAND BIT(7)

#define DSI_PANEL_MAX_PANEL_LEN        128
#define MAX_PARAM_NAME 10

#define BRIGHTNESS_HBM_ON	0xFFFFFFFE
#define BRIGHTNESS_HBM_OFF	(BRIGHTNESS_HBM_ON - 1)
#define HBM_BRIGHTNESS(value) ((value) == HBM_OFF_STATE ?\
			BRIGHTNESS_HBM_OFF : BRIGHTNESS_HBM_ON)

/* HBM implementation is different, depending on display and backlight hardware
 * design, which is classified into the following types:
 * HBM_TYPE_OLED: OLED panel, HBM is controlled by DSI register only, which
 *     is independent on brightness.
 * HBM_TYPE_LCD_DCS_WLED: LCD panel, HBM is controlled by DSI register, and
 *     brightness is decided by WLED IC on I2C/SPI bus.
 * HBM_TYPE_LCD_DCS_ONLY: LCD panel, brightness/HBM is controlled by DSI
 *     register only.
 * HBM_TYPE_LCD_WLED_ONLY: LCD panel, brightness/HBM is controlled by WLED
 *     IC only.
 * HBM_TYPE_LCD_DCS_GPIO: LCD panel, HBM  is controlled by GPIO, and brightness
 *     is controlled by DSI register.
 *
 * Note: brightness must be at maximum while enabling HBM for all LCD panels
 */

enum panel_hbm_type {
	HBM_TYPE_OLED = 0,
	HBM_TYPE_LCD_DCS_WLED,
	HBM_TYPE_LCD_DCS_ONLY,
	HBM_TYPE_LCD_WLED_ONLY,
	HBM_TYPE_LCD_DCS_GPIO
};

enum dsi_panel_rotation {
	DSI_PANEL_ROTATE_NONE = 0,
	DSI_PANEL_ROTATE_HV_FLIP,
	DSI_PANEL_ROTATE_H_FLIP,
	DSI_PANEL_ROTATE_V_FLIP
};

enum dsi_backlight_type {
	DSI_BACKLIGHT_PWM = 0,
	DSI_BACKLIGHT_WLED,
	DSI_BACKLIGHT_DCS,
	DSI_BACKLIGHT_DUMMY,
	DSI_BACKLIGHT_EXTERNAL,
	DSI_BACKLIGHT_I2C,
	DSI_BACKLIGHT_UNKNOWN,
	DSI_BACKLIGHT_MAX,
};

enum bl_update_flag {
	BL_UPDATE_DELAY_UNTIL_FIRST_FRAME,
	BL_UPDATE_NONE,
};

enum {
	MODE_GPIO_NOT_VALID = 0,
	MODE_SEL_DUAL_PORT,
	MODE_SEL_SINGLE_PORT,
	MODE_GPIO_HIGH,
	MODE_GPIO_LOW,
};

enum dsi_dms_mode {
	DSI_DMS_MODE_DISABLED = 0,
	DSI_DMS_MODE_RES_SWITCH_IMMEDIATE,
};

enum dsi_panel_physical_type {
	DSI_DISPLAY_PANEL_TYPE_LCD = 0,
	DSI_DISPLAY_PANEL_TYPE_OLED,
	DSI_DISPLAY_PANEL_TYPE_MAX,
};

struct dsi_dfps_capabilities {
	enum dsi_dfps_type type;
	u32 min_refresh_rate;
	u32 max_refresh_rate;
	u32 *dfps_list;
	u32 dfps_list_len;
	bool dfps_support;
	bool dfps_send_cmd_support;
	bool dfps_send_cmd_with_te_async;
	u32 panel_on_fps;
	u32 current_fps;
};

struct dsi_qsync_capabilities {
	bool qsync_support;
	u32 qsync_min_fps;
	u32 *qsync_min_fps_list;
	int qsync_min_fps_list_len;
};

struct dsi_avr_capabilities {
	u32 *avr_step_fps_list;
	u32 avr_step_fps_list_len;
};

struct dsi_dyn_clk_caps {
	bool dyn_clk_support;
	enum dsi_dyn_clk_feature_type type;
	bool maintain_const_fps;
};

struct dsi_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *active;
	struct pinctrl_state *suspend;
	struct pinctrl_state *pwm_pin;
};

struct dsi_panel_phy_props {
	u32 panel_width_mm;
	u32 panel_height_mm;
	enum dsi_panel_rotation rotation;
};

struct dsi_backlight_config {
	enum dsi_backlight_type type;
	enum bl_update_flag bl_update;

	bool bl_2bytes_enable;
	u32 bl_min_level;
	u32 bl_max_level;
	u32 brightness_max_level;
	/* current brightness value */
	u32 brightness;
	u32 brightness_default_level;
	u32 bl_level;
	u32 bl_scale;
	u32 bl_scale_sv;
	u32 bl_dcs_subtype;
	bool bl_inverted_dbv;
	/* digital dimming backlight LUT */
	struct drm_msm_dimming_bl_lut *dimming_bl_lut;
	u32 dimming_min_bl;
	u32 dimming_status;
	bool user_disable_notification;

	int en_gpio;
	/* PWM params */
	struct pwm_device *pwm_bl;
	bool pwm_enabled;
	u32 pwm_period_usecs;

	/* WLED params */
	struct led_trigger *wled;
	struct backlight_device *raw_bd;
	struct backlight_device *i2c_bd;

	/* DCS params */
	bool lp_mode;

	/* Use exponent backlight curve */
	bool bl_is_exponent;
};

struct dsi_reset_seq {
	u32 level;
	u32 sleep_ms;
};

struct dsi_panel_reset_config {
	struct dsi_reset_seq *sequence;
	u32 count;

	int reset_gpio;
	int disp_en_gpio;
	int lcd_mode_sel_gpio;
	u32 mode_sel_state;
	int vio_en_gpio;
	int vci_en_gpio;
	int touch_rst_gpio;
};

enum esd_check_status_mode {
	ESD_MODE_REG_READ,
	ESD_MODE_TE_CHK_REG_RD,
	ESD_MODE_SW_BTA,
	ESD_MODE_PANEL_TE,
	ESD_MODE_SW_SIM_SUCCESS,
	ESD_MODE_SW_SIM_FAILURE,
	ESD_MODE_MAX
};

struct drm_panel_esd_config {
	bool esd_enabled;

	enum esd_check_status_mode status_mode;
	struct dsi_panel_cmd_set status_cmd;
	u32 *status_cmds_rlen;
	u32 *status_valid_params;
	u32 *status_value;
	u8 *return_buf;
	u8 *status_buf;
	u32 groups;
};

struct drm_panel_cellid_config {
	bool cellid_enabled;

	struct dsi_panel_cmd_set cellid_cmd;
	u32 cellid_rlen;
	u32 cellid_offset;
	u8 *return_buf;
};

struct dsi_panel_spr_info {
	bool enable;
	enum msm_display_spr_pack_type pack_type;
};

struct dsi_panel_lhbm_config {
	bool enable;
	u32 dc_hybird_threshold;
	u32 dbv_level;
	u32 alpha_reg;
	u32 alpha_size;
	u32 *alpha;
	bool lhbm_wait_for_fps_valid;
	u32 lhbm_wait_for_fps_count;
	u32 lhbm_wait_for_fps_interval;
	u32 *lhbm_not_allowed_fps_list;
	u32 lhbm_not_allowed_fps_list_len;
};

enum panel_idx {
	MAIN_IDX = 0,
	SEC_INX,
	PANEL_IDX_MAX,
};

enum acl_state {
	ACL_OFF_STATE = 0,
	ACL_ON_STATE,
	ACL_STATE_NUM,
};

enum hbm_state {
	HBM_OFF_STATE = 0,
	HBM_ON_STATE,
	HBM_FOD_ON_STATE,
	HBM_STATE_NUM
};

enum cabc_state {
	CABC_UI_STATE,
	CABC_MV_STATE,
	CABC_DIS_STATE,
	CABC_STATE_NUM,
};

enum dc_state {
	DC_OFF_STATE = 0,
	DC_ON_STATE,
	DC_STATE_NUM,
};

enum color_state {
	COLOR_VBT_STATE = 0,
	COLOR_STD_STATE,
	COLOR_GAME_STATE,
	COLOR_NONE_STATE,
	COLOR_STATE_NUM,
};

struct panel_param_val_map {
	int state;
	enum dsi_cmd_set_type type;
	struct dsi_panel_cmd_set *cmds;
};

struct panel_param {
	const char *param_name;
	struct panel_param_val_map *val_map;
	u16 val_max;
	const u16 default_value;
	u16 value;
	bool is_supported;
};

struct dsi_panel;

struct dsi_panel_ops {
	int (*pinctrl_init)(struct dsi_panel *panel);
	int (*gpio_request)(struct dsi_panel *panel);
	int (*pinctrl_deinit)(struct dsi_panel *panel);
	int (*gpio_release)(struct dsi_panel *panel);
	int (*bl_register)(struct dsi_panel *panel);
	int (*bl_unregister)(struct dsi_panel *panel);
	int (*parse_gpios)(struct dsi_panel *panel);
	int (*parse_power_cfg)(struct dsi_panel *panel);
	int (*trigger_esd_attack)(struct dsi_panel *panel);
};
enum touch_state {
	TOUCH_DEEP_SLEEP_STATE = 0,
	TOUCH_LOW_POWER_STATE,
};

struct dsi_panel {
	const char *name;
	const char *type;
	struct device_node *panel_of_node;
	struct mipi_dsi_device mipi_device;
	struct device_node *rgb_left_led_node;
	struct device_node *rgb_right_led_node;

	struct mutex panel_lock;
	struct drm_panel drm_panel;
	struct mipi_dsi_host *host;
	struct device *parent;

	struct dsi_host_common_cfg host_config;
	struct dsi_video_engine_cfg video_config;
	struct dsi_cmd_engine_cfg cmd_config;
	enum dsi_op_mode panel_mode;
	bool panel_mode_switch_enabled;
	bool poms_align_vsync;

	struct dsi_dfps_capabilities dfps_caps;
	struct dsi_dyn_clk_caps dyn_clk_caps;
	struct dsi_panel_phy_props phy_props;
	bool dsc_switch_supported;

	struct dsi_display_mode *cur_mode;
	u32 num_timing_nodes;
	u32 num_display_modes;
	u32 refresh_rate_base;
	bool 	switch_rate_base;

	char fsc_rgb_order[FSC_MODE_LABEL_SIZE];

	struct dsi_regulator_info power_info;
	struct dsi_backlight_config bl_config;
	struct dsi_panel_reset_config reset_config;
	struct dsi_pinctrl_info pinctrl;
	struct drm_panel_hdr_properties hdr_props;
	struct drm_panel_esd_config esd_config;

	struct dsi_panel_lhbm_config lhbm_config;

	struct drm_panel_cellid_config cellid_config;

	struct dsi_parser_utils utils;

	bool lp11_init;
	bool ulps_feature_enabled;
	bool ulps_suspend_enabled;
	bool allow_phy_power_off;
	bool reset_gpio_always_on;
	bool need_execute_shutdown;
	atomic_t esd_recovery_pending;

	bool panel_initialized;
	bool te_using_watchdog_timer;
	struct dsi_qsync_capabilities qsync_caps;
	struct dsi_avr_capabilities avr_caps;

	char dce_pps_cmd[DSI_CMD_PPS_SIZE];
	enum dsi_dms_mode dms_mode;

	struct dsi_panel_spr_info spr_info;

	bool sync_broadcast_en;
	u32 dsc_count;
	u32 lm_count;

	int panel_test_gpio;
	int power_mode;
	enum dsi_panel_physical_type panel_type;

	struct dsi_panel_ops panel_ops;
	bool esd_utag_enable;
	u64 panel_id;
	u64 panel_ver;
	u32 panel_regDA;
	char panel_name[DSI_PANEL_MAX_PANEL_LEN];
	char panel_supplier[DSI_PANEL_MAX_PANEL_LEN];
	char panel_declare[DSI_PANEL_MAX_PANEL_LEN];

	u32 disp_on_chk_val;
	bool no_panel_on_read_support;

	bool panel_hbm_fod;
	bool panel_hbm_dim_off;

	enum panel_hbm_type hbm_type;
	u32  bl_lvl_during_hbm;

	struct panel_param *param_cmds;

	enum touch_state tp_state;
	bool tp_state_check_enable;
	bool tp_state_need_reset;

	int panel_recovery_retry;
	bool is_panel_dead;
       int paramVersion;
	int backlight_map_type;
	bool delect_dc_onoff;
	bool hbm_detect_fps;
	bool dc_on;
	bool mot_nt37701A_read_cellid;
	bool esd_first_check;
	int dc_state;
	int panel_power_cnt;
	struct msm_param_info curDCModeParaInfo;
	bool panel_send_cmd;
	bool nt37705_dc_detect_fps;
	bool rm690a0_backlight_config;
	bool check_pcd;
	int panelPcdCheck_enable;
	bool hbm_enabled;
	bool fod_hbm_enabled;
};

bool dsi_display_all_displays_dead(void);

static inline bool dsi_panel_ulps_feature_enabled(struct dsi_panel *panel)
{
	return panel->ulps_feature_enabled;
}

static inline bool dsi_panel_initialized(struct dsi_panel *panel)
{
	return panel->panel_initialized;
}

static inline void dsi_panel_acquire_panel_lock(struct dsi_panel *panel)
{
	mutex_lock(&panel->panel_lock);
}

static inline void dsi_panel_release_panel_lock(struct dsi_panel *panel)
{
	mutex_unlock(&panel->panel_lock);
}

static inline bool dsi_panel_is_type_oled(struct dsi_panel *panel)
{
	return (panel->panel_type == DSI_DISPLAY_PANEL_TYPE_OLED);
}

struct dsi_panel *dsi_panel_get(struct device *parent,
				struct device_node *of_node,
				struct device_node *parser_node,
				const char *type,
				int topology_override,
				bool trusted_vm_env,
				u32 panel_idx);

void dsi_panel_put(struct dsi_panel *panel);

int dsi_panel_drv_init(struct dsi_panel *panel, struct mipi_dsi_host *host);

int dsi_panel_drv_deinit(struct dsi_panel *panel);

int dsi_panel_get_mode_count(struct dsi_panel *panel);

void dsi_panel_put_mode(struct dsi_display_mode *mode);

int dsi_panel_get_mode(struct dsi_panel *panel,
		       u32 index,
		       struct dsi_display_mode *mode,
		       int topology_override);

int dsi_panel_validate_mode(struct dsi_panel *panel,
			    struct dsi_display_mode *mode);

int dsi_panel_get_host_cfg_for_mode(struct dsi_panel *panel,
				    struct dsi_display_mode *mode,
				    struct dsi_host_config *config);

int dsi_panel_get_phy_props(struct dsi_panel *panel,
			    struct dsi_panel_phy_props *phy_props);
int dsi_panel_get_dfps_caps(struct dsi_panel *panel,
			    struct dsi_dfps_capabilities *dfps_caps);

int dsi_panel_pre_prepare(struct dsi_panel *panel);

int dsi_panel_set_lp1(struct dsi_panel *panel);

int dsi_panel_set_lp2(struct dsi_panel *panel);

int dsi_panel_set_nolp(struct dsi_panel *panel);

int dsi_panel_prepare(struct dsi_panel *panel);

int dsi_panel_enable(struct dsi_panel *panel);

int dsi_panel_post_enable(struct dsi_panel *panel);

int dsi_panel_pre_disable(struct dsi_panel *panel);

int dsi_panel_disable(struct dsi_panel *panel);

int dsi_panel_unprepare(struct dsi_panel *panel);

int dsi_panel_post_unprepare(struct dsi_panel *panel);

int dsi_panel_set_backlight(struct dsi_panel *panel, u32 bl_lvl);

int dsi_panel_update_pps(struct dsi_panel *panel);

int dsi_panel_send_qsync_on_dcs(struct dsi_panel *panel,
		int ctrl_idx);
int dsi_panel_send_qsync_off_dcs(struct dsi_panel *panel,
		int ctrl_idx);

int dsi_panel_send_roi_dcs(struct dsi_panel *panel, int ctrl_idx,
		struct dsi_rect *roi);

int dsi_panel_switch_video_mode_out(struct dsi_panel *panel);

int dsi_panel_switch_cmd_mode_out(struct dsi_panel *panel);

int dsi_panel_switch_cmd_mode_in(struct dsi_panel *panel);

int dsi_panel_switch_video_mode_in(struct dsi_panel *panel);

int dsi_panel_switch(struct dsi_panel *panel);

int dsi_panel_post_switch(struct dsi_panel *panel);

void dsi_dsc_pclk_param_calc(struct msm_display_dsc_info *dsc, int intf_width);

void dsi_panel_bl_handoff(struct dsi_panel *panel);

struct dsi_panel *dsi_panel_ext_bridge_get(struct device *parent,
				struct device_node *of_node,
				int topology_override);

int dsi_panel_parse_esd_reg_read_configs(struct dsi_panel *panel);

int dsi_panel_parse_panel_cfg(struct dsi_panel *panel, bool is_primary);

void dsi_panel_ext_bridge_put(struct dsi_panel *panel);

int dsi_panel_get_io_resources(struct dsi_panel *panel,
		struct msm_io_res *io_res);

void dsi_panel_calc_dsi_transfer_time(struct dsi_host_common_cfg *config,
		struct dsi_display_mode *mode, u32 frame_threshold_us);

int dsi_panel_get_cmd_pkt_count(const char *data, u32 length, u32 *cnt);

int dsi_panel_alloc_cmd_packets(struct dsi_panel_cmd_set *cmd,
		u32 packet_count);

int dsi_panel_create_cmd_packets(const char *data, u32 length, u32 count,
					struct dsi_cmd_desc *cmd);

void dsi_panel_destroy_cmd_packets(struct dsi_panel_cmd_set *set);

void dsi_panel_dealloc_cmd_packets(struct dsi_panel_cmd_set *set);
int dsi_panel_set_param(struct dsi_panel *panel,
			struct msm_param_info *param_info);

void dsi_panel_reset_param(struct dsi_panel *panel);

int dsi_panel_get_elvss_data(struct dsi_panel *panel);
int dsi_panel_get_elvss_data_1(struct dsi_panel *panel);
int dsi_panel_set_elvss_dim_off(struct dsi_panel *panel, u8 val);
int dsi_panel_parse_elvss_config(struct dsi_panel *panel, u8 elv_vl);
void mot_update_hbmoff(struct dsi_panel *panel,
                        struct msm_param_info *param_info);
int dsi_panel_dfps_send_cmd(struct dsi_panel *panel);
int dsi_panel_tx_cellid_cmd(struct dsi_panel *panel);
int dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum dsi_cmd_set_type type);
void set_panelpcdcheck_enable(struct dsi_panel *panel);

#endif /* _DSI_PANEL_H_ */

