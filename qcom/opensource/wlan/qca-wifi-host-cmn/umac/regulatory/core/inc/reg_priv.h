/*
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
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
 * DOC: reg_priv.h
 * This file contains regulatory component private data structures.
 */

#define reg_log(level, args...) \
	QDF_TRACE(QDF_MODULE_ID_REGULATORY, level, ## args)
#define reg_logfl(level, format, args...) reg_log(level, FL(format), ## args)

#define reg_alert(format, args...) \
		reg_logfl(QDF_TRACE_LEVEL_FATAL, format, ## args)
#define reg_err(format, args...) \
		reg_logfl(QDF_TRACE_LEVEL_ERROR, format, ## args)
#define reg_warn(format, args...) \
		reg_logfl(QDF_TRACE_LEVEL_WARN, format, ## args)
#define reg_notice(format, args...) \
		reg_logfl(QDF_TRACE_LEVEL_INFO, format, ## args)
#define reg_info(format, args...) \
		reg_logfl(QDF_TRACE_LEVEL_INFO_HIGH, format, ## args)
#define reg_debug(format, args...) \
		reg_logfl(QDF_TRACE_LEVEL_DEBUG, format, ## args)

struct wlan_regulatory_psoc_priv_obj {
	struct regulatory_channel master_ch_list[NUM_CHANNELS];
	struct regulatory_channel current_ch_list[NUM_CHANNELS];
	bool offload_enabled;
	enum channel_enum nol_list[NUM_CHANNELS];
	char default_country[REG_ALPHA2_LEN + 1];
	char current_country[REG_ALPHA2_LEN + 1];
	uint32_t phybitmap;
	enum dfs_region dfs_region;
	char country_11d[REG_ALPHA2_LEN + 1];
	bool dfs_disable;
	bool set_fcc_channel;
	enum band_info band_capability;
	bool indoor_ch_enabled;
	bool enable_11d_supp_original;
	bool enable_11d_supp_current;
	bool userspace_country_priority;
};
