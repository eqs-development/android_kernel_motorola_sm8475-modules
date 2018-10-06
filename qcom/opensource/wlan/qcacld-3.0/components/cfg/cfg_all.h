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

#include "cfg_define.h"
#include "cfg_converged.h"
#include "cfg_mlme.h"
#include "cfg_fwol.h"

#ifdef CONVERGED_P2P_ENABLE
#include "wlan_p2p_cfg.h"
#else
#define CFG_P2P_ALL
#endif

#ifdef CONVERGED_TDLS_ENABLE
#include "wlan_tdls_cfg.h"
#else
#define CFG_TDLS_ALL
#endif

#ifdef WLAN_FEATURE_NAN_CONVERGENCE
#include "cfg_nan.h"
#else
#define CFG_NAN_ALL
#endif

#include "wlan_pmo_cfg.h"
#include "hdd_config.h"
#include "hdd_dp_cfg.h"

/* Maintain Alphabetic order here while adding components */
#define CFG_ALL \
	CFG_CONVERGED_ALL \
	CFG_FWOL_ALL \
	CFG_HDD_ALL \
	CFG_HDD_DP_ALL \
	CFG_MLME_ALL \
	CFG_NAN_ALL \
	CFG_P2P_ALL \
	CFG_PMO_ALL \
	CFG_TDLS_ALL
