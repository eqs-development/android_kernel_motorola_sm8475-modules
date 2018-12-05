/*
 * Copyright (c) 2011-2018 The Linux Foundation. All rights reserved.
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

/*
 *
 * lim_send_messages.h: Provides functions to send messages or Indications to HAL.
 * Author:    Sunit Bhatia
 * Date:       09/21/2006
 * History:-
 * Date        Modified by            Modification Information
 *
 * --------------------------------------------------------------------------
 *
 */
#ifndef __LIM_SEND_MESSAGES_H
#define __LIM_SEND_MESSAGES_H

#include "ani_global.h"
#include "lim_types.h"
#include "wma_if.h"
#include "sir_params.h"
QDF_STATUS lim_send_cf_params(struct mac_context *mac, uint8_t bssIdx,
				 uint8_t cfpCount, uint8_t cfpPeriod);
QDF_STATUS lim_send_beacon_params(struct mac_context *mac,
				     tpUpdateBeaconParams pUpdatedBcnParams,
				     struct pe_session *pe_session);
/* QDF_STATUS lim_send_beacon_params(struct mac_context *mac, tpUpdateBeaconParams pUpdatedBcnParams); */
QDF_STATUS lim_send_mode_update(struct mac_context *mac,
				   tUpdateVHTOpMode *tempParam,
				   struct pe_session *pe_session);
QDF_STATUS lim_send_rx_nss_update(struct mac_context *mac,
				     tUpdateRxNss *tempParam,
				     struct pe_session *pe_session);

QDF_STATUS lim_set_membership(struct mac_context *mac,
				 tUpdateMembership *pTempParam,
				 struct pe_session *pe_session);

QDF_STATUS lim_set_user_pos(struct mac_context *mac,
			       tUpdateUserPos *pTempParam,
			       struct pe_session *pe_session);
QDF_STATUS lim_send_switch_chnl_params(struct mac_context *mac,
					  uint8_t chnlNumber,
					  uint8_t ch_center_freq_seg0,
					  uint8_t ch_center_freq_seg1,
					  enum phy_ch_width ch_width,
					  int8_t maxTxPower,
					  uint8_t peSessionId,
					  uint8_t is_restart,
					  uint32_t cac_duration_ms,
					  uint32_t dfs_regdomain);

QDF_STATUS lim_send_edca_params(struct mac_context *mac,
				   tSirMacEdcaParamRecord *pUpdatedEdcaParams,
				   uint16_t bssIdx, bool mu_edca);
QDF_STATUS lim_set_link_state(struct mac_context *mac, tSirLinkState state,
				 tSirMacAddr bssId, tSirMacAddr selfMac,
				 tpSetLinkStateCallback callback,
				 void *callbackArg);
extern QDF_STATUS lim_set_link_state_ft(struct mac_context *mac, tSirLinkState
					   state, tSirMacAddr bssId,
					   tSirMacAddr selfMacAddr, int ft,
					   struct pe_session *pe_session);
void lim_set_active_edca_params(struct mac_context *mac,
				tSirMacEdcaParamRecord *plocalEdcaParams,
				struct pe_session *pe_session);
#define CAPABILITY_FILTER_MASK  0x73CF
#define ERP_FILTER_MASK         0xF8
#define EDCA_FILTER_MASK        0xF0
#define QOS_FILTER_MASK         0xF0
#define HT_BYTE0_FILTER_MASK    0x0
#define HT_BYTE2_FILTER_MASK    0xEB
#define HT_BYTE5_FILTER_MASK    0xFD
#define DS_PARAM_CHANNEL_MASK   0x0
#define VHTOP_CHWIDTH_MASK      0xFC

#ifdef WLAN_FEATURE_11W
QDF_STATUS lim_send_exclude_unencrypt_ind(struct mac_context *mac,
					     bool excludeUnenc,
					     struct pe_session *pe_session);
#endif
QDF_STATUS lim_send_ht40_obss_scanind(struct mac_context *mac_ctx,
						struct pe_session *session);
void lim_handle_sme_join_result(struct mac_context *,
		tSirResultCodes, uint16_t, struct pe_session *);
#endif
