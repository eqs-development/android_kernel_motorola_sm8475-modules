/*
 * Copyright (c) 2012 - 2018 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */
/**
 * DOC: declare utility API related to the fw_offload component
 * called by other components
 */

#ifndef _WLAN_FW_OFFLOAD_MAIN_H_
#define _WLAN_FW_OFFLOAD_MAIN_H_

#include <wlan_objmgr_psoc_obj.h>
#include <wlan_objmgr_global_obj.h>
#include <wlan_cmn.h>

#include "cfg_ie_whitelist.h"

#define fwol_alert(params...) QDF_TRACE_FATAL(QDF_MODULE_ID_FWOL, params)
#define fwol_err(params...) QDF_TRACE_ERROR(QDF_MODULE_ID_FWOL, params)
#define fwol_warn(params...) QDF_TRACE_WARN(QDF_MODULE_ID_FWOL, params)
#define fwol_info(params...) QDF_TRACE_INFO(QDF_MODULE_ID_FWOL, params)
#define fwol_debug(params...) QDF_TRACE_DEBUG(QDF_MODULE_ID_FWOL, params)

/**
 * struct wlan_fwol_three_antenna_btc - Three antenna BTC config items
 * @btc_mode: Config BTC mode
 * @antenna_isolation: Antenna isolation
 * @max_tx_power_for_btc: Max wlan tx power in co-ex scenario
 * @wlan_low_rssi_threshold: Wlan low rssi threshold for BTC mode switching
 * @bt_low_rssi_threshold: BT low rssi threshold for BTC mode switching
 * @bt_interference_low_ll: Lower limit of low level BT interference
 * @bt_interference_low_ul: Upper limit of low level BT interference
 * @bt_interference_medium_ll: Lower limit of medium level BT interference
 * @bt_interference_medium_ul: Upper limit of medium level BT interference
 * @bt_interference_high_ll: Lower limit of high level BT interference
 * @bt_interference_high_ul: Upper limit of high level BT interference
 */
struct wlan_fwol_coex_config {
	uint8_t btc_mode;
	uint8_t antenna_isolation;
	uint8_t max_tx_power_for_btc;
	int16_t wlan_low_rssi_threshold;
	int16_t bt_low_rssi_threshold;
	int16_t bt_interference_low_ll;
	int16_t bt_interference_low_ul;
	int16_t bt_interference_medium_ll;
	int16_t bt_interference_medium_ul;
	int16_t bt_interference_high_ll;
	int16_t bt_interference_high_ul;
};

/*
 * struct wlan_fwol_thermal_temp - Thermal temperature config items
 * @thermal_temp_min_level0: Thermal temperature minimum level 0
 * @thermal_temp_max_level0: Thermal temperature maximum level 0
 * @thermal_temp_min_level1: Thermal temperature minimum level 1
 * @thermal_temp_max_level1: Thermal temperature maximum level 1
 * @thermal_temp_min_level2: Thermal temperature minimum level 2
 * @thermal_temp_max_level2: Thermal temperature maximum level 2
 * @thermal_temp_min_level3: Thermal temperature minimum level 3
 * @thermal_temp_max_level3: Thermal temperature maximum level 3
 */
struct wlan_fwol_thermal_temp {
	uint16_t thermal_temp_min_level0;
	uint16_t thermal_temp_max_level0;
	uint16_t thermal_temp_min_level1;
	uint16_t thermal_temp_max_level1;
	uint16_t thermal_temp_min_level2;
	uint16_t thermal_temp_max_level2;
	uint16_t thermal_temp_min_level3;
	uint16_t thermal_temp_max_level3;
};

/**
 * struct wlan_fwol_ie_whitelist - Probe request IE whitelist config items
 * @ie_whitelist: IE whitelist flag
 * @ie_bitmap_0: IE bitmap 0
 * @ie_bitmap_1: IE bitmap 1
 * @ie_bitmap_2: IE bitmap 2
 * @ie_bitmap_3: IE bitmap 3
 * @ie_bitmap_4: IE bitmap 4
 * @ie_bitmap_5: IE bitmap 5
 * @ie_bitmap_6: IE bitmap 6
 * @ie_bitmap_7: IE bitmap 7
 * @no_of_probe_req_ouis: Total number of ouis present in probe req
 * @probe_req_voui: Stores oui values after parsing probe req ouis
 */
struct wlan_fwol_ie_whitelist {
	bool ie_whitelist;
	uint32_t ie_bitmap_0;
	uint32_t ie_bitmap_1;
	uint32_t ie_bitmap_2;
	uint32_t ie_bitmap_3;
	uint32_t ie_bitmap_4;
	uint32_t ie_bitmap_5;
	uint32_t ie_bitmap_6;
	uint32_t ie_bitmap_7;
	uint32_t no_of_probe_req_ouis;
	uint32_t probe_req_voui[MAX_PROBE_REQ_OUIS];
};

/**
 * struct wlan_fwol_cfg - fwol config items
 * coex_config: coex config items
 * thermal_temp_cfg: Thermal temperature related config items
 * ie_whitelist_cfg: IE Whitelist related config items
 * @ani_enabled: ANI enable/disable
 * @enable_rts_sifsbursting: Enable RTS SIFS Bursting
 * @max_mpdus_inampdu: Max number of MPDUS
 * @arp_ac_category: ARP AC category
 * @enable_phy_reg_retention: Enable PHY reg retention
 * @upper_brssi_thresh: Upper BRSSI threshold
 * @lower_brssi_thresh: Lower BRSSI threshold
 * @enable_dtim_1chrx: Enable/disable DTIM 1 CHRX
 * @alternative_chainmask_enabled: Alternate chainmask
 * @smart_chainmask_enabled: Enable/disable chainmask
 * @get_rts_profile: Set the RTS profile
 * @enable_fw_log_level: Set the FW log level
 * @enable_fw_log_type: Set the FW log type
 * @is_rate_limit_enabled: Enable/disable RA rate limited
 * @tsf_gpio_pin: TSF GPIO Pin config
 * @enable_dhcp_server_offload: DHCP Offload is enabled or not
 * @dhcp_max_num_clients: Max number of DHCP client supported
 */
struct wlan_fwol_cfg {
	/* Add CFG and INI items here */
	struct wlan_fwol_coex_config coex_config;
	struct wlan_fwol_thermal_temp thermal_temp_cfg;
	struct wlan_fwol_ie_whitelist ie_whitelist_cfg;
	bool ani_enabled;
	bool enable_rts_sifsbursting;
	uint8_t max_mpdus_inampdu;
	uint32_t arp_ac_category;
	uint8_t enable_phy_reg_retention;
	uint16_t upper_brssi_thresh;
	uint16_t lower_brssi_thresh;
	bool enable_dtim_1chrx;
	bool alternative_chainmask_enabled;
	bool smart_chainmask_enabled;
	uint16_t get_rts_profile;
	uint16_t enable_fw_log_level;
	uint16_t enable_fw_log_type;
#ifdef FEATURE_WLAN_RA_FILTERING
	bool is_rate_limit_enabled;
#endif
#ifdef WLAN_FEATURE_TSF
	uint32_t tsf_gpio_pin;
#endif
#ifdef DHCP_SERVER_OFFLOAD
	bool enable_dhcp_server_offload;
	uint32_t dhcp_max_num_clients;
#endif
};

/**
 * struct wlan_fwol_psoc_obj - FW offload psoc priv object
 * @cfg:     cfg items
 */
struct wlan_fwol_psoc_obj {
	struct wlan_fwol_cfg cfg;
};

/**
 * wlan_psoc_get_fwol_obj() - private API to get fwol object from psoc
 * @psoc: psoc object
 *
 * Return: fwol object
 */
struct wlan_fwol_psoc_obj *fwol_get_psoc_obj(struct wlan_objmgr_psoc *psoc);

/*
 * fwol_cfg_on_psoc_enable() - Populate FWOL structure from CFG and INI
 * @psoc: pointer to the psoc object
 *
 * Populate the FWOL CFG structure from CFG and INI values using CFG APIs
 *
 * Return: QDF_STATUS
 */
QDF_STATUS fwol_cfg_on_psoc_enable(struct wlan_objmgr_psoc *psoc);

/*
 * fwol_cfg_on_psoc_disable() - Clear the CFG structure on psoc disable
 * @psoc: pointer to the psoc object
 *
 * Clear the FWOL CFG structure on psoc disable
 *
 * Return: QDF_STATUS
 */
QDF_STATUS fwol_cfg_on_psoc_disable(struct wlan_objmgr_psoc *psoc);
#endif
