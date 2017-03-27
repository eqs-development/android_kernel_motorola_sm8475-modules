/*
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
 *
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
 * DOC: reg_services.h
 * This file provides prototypes of the regulatory component
 * service functions
 */

#ifndef __REG_SERVICES_H_
#define __REG_SERVICES_H_

#include <qdf_types.h>
#include <qdf_status.h>
#include <wlan_objmgr_cmn.h>
#include <wlan_objmgr_global_obj.h>
#include <wlan_objmgr_psoc_obj.h>
#include <wlan_objmgr_pdev_obj.h>
#include "reg_db.h"
#include <reg_services_public_struct.h>

/**
 * enum channel_state - channel state
 * @CHANNEL_STATE_DISABLE: disabled state
 * @CHANNEL_STATE_PASSIVE: passive state
 * @CHANNEL_STATE_DFS: dfs state
 * @CHANNEL_STATE_ENABLE: enabled state
 * @CHANNEL_STATE_INVALID: invalid state
 */
enum channel_state {
	CHANNEL_STATE_DISABLE,
	CHANNEL_STATE_PASSIVE,
	CHANNEL_STATE_DFS,
	CHANNEL_STATE_ENABLE,
	CHANNEL_STATE_INVALID,
};

typedef enum {
	REGDOMAIN_FCC,
	REGDOMAIN_ETSI,
	REGDOMAIN_JAPAN,
	REGDOMAIN_WORLD,
	REGDOMAIN_COUNT
} v_REGDOMAIN_t;


/**
 * enum phy_ch_width - channel width
 * @CH_WIDTH_20MHZ: 20 mhz width
 * @CH_WIDTH_40MHZ: 40 mhz width
 * @CH_WIDTH_80MHZ: 80 mhz width
 * @CH_WIDTH_160MHZ: 160 mhz width
 * @CH_WIDTH_80P80HZ: 80+80 mhz width
 * @CH_WIDTH_5MHZ: 5 mhz width
 * @CH_WIDTH_10MHZ: 10 mhz width
 * @CH_WIDTH_INVALID: invalid width
 * @CH_WIDTH_MAX: max possible width
 */
enum phy_ch_width {
	CH_WIDTH_20MHZ = 0,
	CH_WIDTH_40MHZ,
	CH_WIDTH_80MHZ,
	CH_WIDTH_160MHZ,
	CH_WIDTH_80P80MHZ,
	CH_WIDTH_5MHZ,
	CH_WIDTH_10MHZ,
	CH_WIDTH_INVALID,
	CH_WIDTH_MAX
};

/**
 * struct ch_params
 * @ch_width: channel width
 * @sec_ch_offset: secondary channel offset
 * @center_freq_seg0: center freq for segment 0
 * @center_freq_seg1: center freq for segment 1
 */
struct ch_params_s {
	enum phy_ch_width ch_width;
	uint8_t sec_ch_offset;
	uint8_t center_freq_seg0;
	uint8_t center_freq_seg1;
};

/**
 * struct regulatory_channel
 * @center_freq: center frequency
 * @chan_num: channel number
 * @state: channel state
 * @chan_flags: channel flags
 * @tx_power: TX powers
 */
struct regulatory_channel {
	uint32_t center_freq;
	uint32_t chan_num;
	enum channel_state state;
	uint32_t chan_flags;
	uint32_t tx_power;
	uint16_t min_bw;
	uint16_t max_bw;
};

/**
 * struct channel_power
 * @chan_num: channel number
 * @tx_power: TX power
 */
struct channel_power {
	uint32_t chan_num;
	uint32_t tx_power;
};

/**
 * enum channel_enum - channel enumeration
 * @CHAN_ENUM_1:  channel number 1
 * @CHAN_ENUM_2:  channel number 2
 * @CHAN_ENUM_3:  channel number 3
 * @CHAN_ENUM_4:  channel number 4
 * @CHAN_ENUM_5:  channel number 5
 * @CHAN_ENUM_6:  channel number 6
 * @CHAN_ENUM_7:  channel number 7
 * @CHAN_ENUM_8:  channel number 8
 * @CHAN_ENUM_9:  channel number 9
 * @CHAN_ENUM_10:  channel number 10
 * @CHAN_ENUM_11:  channel number 11
 * @CHAN_ENUM_12:  channel number 12
 * @CHAN_ENUM_13:  channel number 13
 * @CHAN_ENUM_14:  channel number 14
 * @CHAN_ENUM_36:  channel number 36
 * @CHAN_ENUM_40:  channel number 40
 * @CHAN_ENUM_44:  channel number 44
 * @CHAN_ENUM_48:  channel number 48
 * @CHAN_ENUM_52:  channel number 52
 * @CHAN_ENUM_56:  channel number 56
 * @CHAN_ENUM_60:  channel number 60
 * @CHAN_ENUM_64:  channel number 64
 * @CHAN_ENUM_100:  channel number 100
 * @CHAN_ENUM_104:  channel number 104
 * @CHAN_ENUM_108:  channel number 108
 * @CHAN_ENUM_112:  channel number 112
 * @CHAN_ENUM_116:  channel number 116
 * @CHAN_ENUM_120:  channel number 120
 * @CHAN_ENUM_124:  channel number 124
 * @CHAN_ENUM_128:  channel number 128
 * @CHAN_ENUM_132:  channel number 132
 * @CHAN_ENUM_136:  channel number 136
 * @CHAN_ENUM_140:  channel number 140
 * @CHAN_ENUM_144:  channel number 144
 * @CHAN_ENUM_149:  channel number 149
 * @CHAN_ENUM_153:  channel number 153
 * @CHAN_ENUM_157:  channel number 157
 * @CHAN_ENUM_161:  channel number 161
 * @CHAN_ENUM_165:  channel number 165
 * @CHAN_ENUM_183:  channel number 183
 * @CHAN_ENUM_184:  channel number 184
 * @CHAN_ENUM_185:  channel number 185
 * @CHAN_ENUM_187:  channel number 187
 * @CHAN_ENUM_188:  channel number 188
 * @CHAN_ENUM_189:  channel number 189
 * @CHAN_ENUM_192:  channel number 192
 * @CHAN_ENUM_196:  channel number 196
 */
enum channel_enum {
	CHAN_ENUM_1,
	CHAN_ENUM_2,
	CHAN_ENUM_3,
	CHAN_ENUM_4,
	CHAN_ENUM_5,
	CHAN_ENUM_6,
	CHAN_ENUM_7,
	CHAN_ENUM_8,
	CHAN_ENUM_9,
	CHAN_ENUM_10,
	CHAN_ENUM_11,
	CHAN_ENUM_12,
	CHAN_ENUM_13,
	CHAN_ENUM_14,

	CHAN_ENUM_36,
	CHAN_ENUM_40,
	CHAN_ENUM_44,
	CHAN_ENUM_48,
	CHAN_ENUM_52,
	CHAN_ENUM_56,
	CHAN_ENUM_60,
	CHAN_ENUM_64,

	CHAN_ENUM_100,
	CHAN_ENUM_104,
	CHAN_ENUM_108,
	CHAN_ENUM_112,
	CHAN_ENUM_116,
	CHAN_ENUM_120,
	CHAN_ENUM_124,
	CHAN_ENUM_128,
	CHAN_ENUM_132,
	CHAN_ENUM_136,
	CHAN_ENUM_140,
	CHAN_ENUM_144,

	CHAN_ENUM_149,
	CHAN_ENUM_153,
	CHAN_ENUM_157,
	CHAN_ENUM_161,
	CHAN_ENUM_165,

	CHAN_ENUM_183,
	CHAN_ENUM_184,
	CHAN_ENUM_185,
	CHAN_ENUM_187,
	CHAN_ENUM_188,
	CHAN_ENUM_189,
	CHAN_ENUM_192,
	CHAN_ENUM_196,

	NUM_CHANNELS,

	MIN_24GHZ_CHANNEL = CHAN_ENUM_1,
	MAX_24GHZ_CHANNEL = CHAN_ENUM_14,
	NUM_24GHZ_CHANNELS = (MAX_24GHZ_CHANNEL - MIN_24GHZ_CHANNEL + 1),

	MIN_5GHZ_CHANNEL = CHAN_ENUM_36,
	MAX_5GHZ_CHANNEL = CHAN_ENUM_196,
	NUM_5GHZ_CHANNELS = (MAX_5GHZ_CHANNEL - MIN_5GHZ_CHANNEL + 1),

	MIN_49GHZ_CHANNEL = CHAN_ENUM_183,
	MAX_49GHZ_CHANNEL = CHAN_ENUM_196,

	INVALID_CHANNEL = 0xBAD,
};

/**
 * enum country_src: country source
 * @SOURCE_QUERY: source query
 * @SOURCE_CORE: source regulatory core
 * @SOURCE_DRIVER: source driver
 * @SOURCE_USERSPACE: source userspace
 * @SOURCE_11D: source 11D
 */
enum country_src {
	SOURCE_UNKNOWN,
	SOURCE_QUERY,
	SOURCE_CORE,
	SOURCE_DRIVER,
	SOURCE_USERSPACE,
	SOURCE_11D
};

/**
 * struct regulatory: regulatory information
 * @reg_domain: regulatory domain pair
 * @eeprom_rd_ext: eeprom value
 * @country_code: current country in integer
 * @alpha2: current alpha2
 * @def_country: default country alpha2
 * @def_region: DFS region
 * @ctl_2g: 2G CTL value
 * @ctl_5g: 5G CTL value
 * @reg_pair: pointer to regulatory pair
 * @cc_src: country code src
 * @reg_flags: kernel regulatory flags
 */
struct regulatory {
	uint32_t reg_domain;
	uint32_t eeprom_rd_ext;
	uint16_t country_code;
	uint8_t alpha2[REG_ALPHA2_LEN + 1];
	uint8_t ctl_2g;
	uint8_t ctl_5g;
	const void *regpair;
	enum country_src cc_src;
	uint32_t reg_flags;
};


/**
 * struct chan_map
 * @start_ch: center frequency
 * @chan_num: channel number
 */
struct chan_map {
	uint32_t center_freq;
	uint32_t chan_num;
};

/**
 * struct bonded_channel
 * @start_ch: start channel
 * @end_ch: end channel
 */
struct bonded_channel {
	uint16_t start_ch;
	uint16_t end_ch;
};

/**
 * enum ht_sec_ch_offset
 * @NO_SEC_CH: no secondary
 * @LOW_PRIMARY_CH: low primary
 * @HIGH_PRIMARY_CH: high primary
 */
enum ht_sec_ch_offset {
	NO_SEC_CH = 0,
	LOW_PRIMARY_CH = 1,
	HIGH_PRIMARY_CH = 3,
};


extern const struct chan_map channel_map[NUM_CHANNELS];

QDF_STATUS reg_get_channel_list_with_power(struct wlan_objmgr_psoc *psoc,
					   struct channel_power *ch_list,
					   uint8_t *num_chan);

void reg_read_default_country(struct wlan_objmgr_psoc *psoc,
		uint8_t *country);
enum channel_state reg_get_channel_state(struct wlan_objmgr_psoc *psoc,
		uint32_t ch);
enum channel_state reg_get_5g_bonded_channel_state(
		struct wlan_objmgr_psoc *psoc,
		uint8_t ch, enum phy_ch_width bw);
enum channel_state reg_get_2g_bonded_channel_state(
		struct wlan_objmgr_psoc *psoc,
		uint8_t oper_ch, uint8_t sec_ch,
		enum phy_ch_width bw);
void reg_set_channel_params(struct wlan_objmgr_psoc *psoc,
		uint8_t ch, struct ch_params_s *ch_params);
void reg_get_dfs_region(struct wlan_objmgr_psoc *psoc,
		enum dfs_reg *dfs_reg);
bool reg_is_dfs_ch(struct wlan_objmgr_psoc *psoc, uint8_t ch);

QDF_STATUS reg_process_master_chan_list(struct cur_regulatory_info *reg_info);

QDF_STATUS wlan_regulatory_psoc_obj_created_notification(
					    struct wlan_objmgr_psoc *psoc,
					    void *arg_list);

QDF_STATUS  wlan_regulatory_psoc_obj_destroyed_notification(
					    struct wlan_objmgr_psoc *psoc,
					    void *arg_list);

static inline struct wlan_lmac_if_reg_tx_ops *
get_reg_psoc_tx_ops(struct wlan_objmgr_psoc *psoc)
{
	return &((psoc->soc_cb.tx_ops.reg_ops));
}

#endif
