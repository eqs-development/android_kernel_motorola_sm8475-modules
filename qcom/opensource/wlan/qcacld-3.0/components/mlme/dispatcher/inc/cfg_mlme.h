/*
 * Copyright (c) 2018 The Linux Foundation. All rights reserved.
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

#ifndef __CFG_MLME_H
#define __CFG_MLME_H

#include "cfg_define.h"
#include "cfg_converged.h"
#include "qdf_types.h"
#include "cfg_mlme_wps_params.h"
#include "cfg_mlme_chainmask.h"
#include "cfg_mlme_edca_params.h"
#include "cfg_mlme_generic.h"
#include "cfg_mlme_acs.h"
#include "cfg_mlme_ht_caps.h"
#include "cfg_mlme_he_caps.h"
#include "cfg_mlme_lfr.h"
#include "cfg_mlme_obss_ht40.h"
#include "cfg_mlme_dfs.h"
#include "cfg_mlme_mbo.h"
#include "cfg_mlme_vht_caps.h"
#include "cfg_qos.h"
#include "cfg_mlme_timeout.h"
#include "cfg_mlme_rates.h"
#include "wlan_mlme_product_details_cfg.h"
#include "cfg_mlme_sta.h"
#include "cfg_sap_protection.h"
#include "cfg_mlme_fe_wmm.h"
#include "cfg_mlme_sap.h"
#include "cfg_mlme_scoring.h"
#include "cfg_mlme_oce.h"
#include "cfg_mlme_threshold.h"
#include "cfg_mlme_feature_flag.h"
#include "cfg_mlme_wep_params.h"

/* Please Maintain Alphabetic Order here */
#define CFG_MLME_ALL \
	CFG_ACS_ALL \
	CFG_CHAINMASK_ALL \
	CFG_DFS_ALL \
	CFG_EDCA_PARAMS_ALL \
	CFG_FEATURE_FLAG_ALL \
	CFG_GENERIC_ALL \
	CFG_HT_CAPS_ALL \
	CFG_HE_CAPS_ALL \
	CFG_LFR_ALL \
	CFG_MBO_ALL \
	CFG_MLME_PRODUCT_DETAILS_ALL \
	CFG_OBSS_HT40_ALL \
	CFG_OCE_ALL \
	CFG_QOS_ALL \
	CFG_RATES_ALL \
	CFG_WMM_PARAMS_ALL\
	CFG_SAP_ALL \
	CFG_SAP_PROTECTION_ALL \
	CFG_SCORING_ALL \
	CFG_STA_ALL \
	CFG_THRESHOLD_ALL \
	CFG_TIMEOUT_ALL \
	CFG_VHT_CAPS_ALL \
	CFG_WEP_PARAMS_ALL \
	CFG_WPS_ALL

#endif /* __CFG_MLME_H */
