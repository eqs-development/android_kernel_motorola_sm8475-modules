/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
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
 *  DOC:    wlan_mgmt_txrx_tgt_api.c
 *  This file contains mgmt txrx public API definitions for
 *  southbound interface.
 */

#include "wlan_mgmt_txrx_tgt_api.h"
#include "wlan_mgmt_txrx_utils_api.h"
#include "wlan_mgmt_txrx_main_i.h"
#include "wlan_objmgr_psoc_obj.h"


/**
 * mgmt_get_spec_mgmt_action_subtype() - gets spec mgmt action subtype
 * @action_code: action code
 *
 * This function returns the subtype for spectrum management action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_spec_mgmt_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case ACTION_SPCT_MSR_REQ:
		frm_type = MGMT_ACTION_MEAS_REQUEST;
		break;
	case ACTION_SPCT_MSR_RPRT:
		frm_type = MGMT_ACTION_MEAS_REPORT;
		break;
	case ACTION_SPCT_TPC_REQ:
		frm_type = MGMT_ACTION_TPC_REQUEST;
		break;
	case ACTION_SPCT_TPC_RPRT:
		frm_type = MGMT_ACTION_TPC_REPORT;
		break;
	case ACTION_SPCT_CHL_SWITCH:
		frm_type = MGMT_ACTION_CHAN_SWITCH;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_qos_action_subtype() - gets qos action subtype
 * @action_code: action code
 *
 * This function returns the subtype for qos action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_qos_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case QOS_ADD_TS_REQ:
		frm_type = MGMT_ACTION_QOS_ADD_TS_REQ;
		break;
	case QOS_ADD_TS_RSP:
		frm_type = MGMT_ACTION_QOS_ADD_TS_RSP;
		break;
	case QOS_DEL_TS_REQ:
		frm_type = MGMT_ACTION_QOS_DEL_TS_REQ;
		break;
	case QOS_SCHEDULE:
		frm_type = MGMT_ACTION_QOS_SCHEDULE;
		break;
	case QOS_MAP_CONFIGURE:
		frm_type = MGMT_ACTION_QOS_MAP_CONFIGURE;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_dls_action_subtype() - gets dls action subtype
 * @action_code: action code
 *
 * This function returns the subtype for dls action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_dls_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case DLS_REQUEST:
		frm_type = MGMT_ACTION_DLS_REQUEST;
		break;
	case DLS_RESPONSE:
		frm_type = MGMT_ACTION_DLS_RESPONSE;
		break;
	case DLS_TEARDOWN:
		frm_type = MGMT_ACTION_DLS_TEARDOWN;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_back_action_subtype() - gets block ack action subtype
 * @action_code: action code
 *
 * This function returns the subtype for block ack action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_back_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case ADDBA_REQUEST:
		frm_type = MGMT_ACTION_BA_ADDBA_REQUEST;
		break;
	case ADDBA_RESPONSE:
		frm_type = MGMT_ACTION_BA_ADDBA_RESPONSE;
		break;
	case DELBA:
		frm_type = MGMT_ACTION_BA_DELBA;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_public_action_subtype() - gets public action subtype
 * @action_code: action code
 *
 * This function returns the subtype for public action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_public_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case PUB_ACTION_2040_BSS_COEXISTENCE:
		frm_type = MGMT_ACTION_2040_BSS_COEXISTENCE;
		break;
	case PUB_ACTION_P2P_SUBTYPE_PRESENCE_RSP:
		frm_type = MGMT_ACTION_P2P_SUBTYPE_PRESENCE_RSP;
		break;
	case PUB_ACTION_EXT_CHANNEL_SWITCH_ID:
		frm_type = MGMT_ACTION_EXT_CHANNEL_SWITCH_ID;
		break;
	case PUB_ACTION_VENDOR_SPECIFIC:
		frm_type = MGMT_ACTION_VENDOR_SPECIFIC;
		break;
	case PUB_ACTION_TDLS_DISCRESP:
		frm_type = MGMT_ACTION_TDLS_DISCRESP;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_rrm_action_subtype() - gets rrm action subtype
 * @action_code: action code
 *
 * This function returns the subtype for rrm action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_rrm_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case RRM_RADIO_MEASURE_REQ:
		frm_type = MGMT_ACTION_RRM_RADIO_MEASURE_REQ;
		break;
	case RRM_RADIO_MEASURE_RPT:
		frm_type = MGMT_ACTION_RRM_RADIO_MEASURE_RPT;
		break;
	case RRM_LINK_MEASUREMENT_REQ:
		frm_type = MGMT_ACTION_RRM_LINK_MEASUREMENT_REQ;
		break;
	case RRM_LINK_MEASUREMENT_RPT:
		frm_type = MGMT_ACTION_RRM_LINK_MEASUREMENT_RPT;
		break;
	case RRM_NEIGHBOR_REQ:
		frm_type = MGMT_ACTION_RRM_NEIGHBOR_REQ;
		break;
	case RRM_NEIGHBOR_RPT:
		frm_type = MGMT_ACTION_RRM_NEIGHBOR_RPT;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_ht_action_subtype() - gets ht action subtype
 * @action_code: action code
 *
 * This function returns the subtype for ht action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_ht_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case HT_ACTION_NOTIFY_CHANWIDTH:
		frm_type = MGMT_ACTION_HT_NOTIFY_CHANWIDTH;
		break;
	case HT_ACTION_SMPS:
		frm_type = MGMT_ACTION_HT_SMPS;
		break;
	case HT_ACTION_PSMP:
		frm_type = MGMT_ACTION_HT_PSMP;
		break;
	case HT_ACTION_PCO_PHASE:
		frm_type = MGMT_ACTION_HT_PCO_PHASE;
		break;
	case HT_ACTION_CSI:
		frm_type = MGMT_ACTION_HT_CSI;
		break;
	case HT_ACTION_NONCOMPRESSED_BF:
		frm_type = MGMT_ACTION_HT_NONCOMPRESSED_BF;
		break;
	case HT_ACTION_COMPRESSED_BF:
		frm_type = MGMT_ACTION_HT_COMPRESSED_BF;
		break;
	case HT_ACTION_ASEL_IDX_FEEDBACK:
		frm_type = MGMT_ACTION_HT_ASEL_IDX_FEEDBACK;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_sa_query_action_subtype() - gets sa query action subtype
 * @action_code: action code
 *
 * This function returns the subtype for sa query action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_sa_query_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case SA_QUERY_REQUEST:
		frm_type = MGMT_ACTION_SA_QUERY_REQUEST;
		break;
	case SA_QUERY_RESPONSE:
		frm_type = MGMT_ACTION_SA_QUERY_RESPONSE;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_pdpa_action_subtype() - gets pdpa action subtype
 * @action_code: action code
 *
 * This function returns the subtype for protected dual public
 * action category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_pdpa_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case PDPA_GAS_INIT_REQ:
		frm_type = MGMT_ACTION_PDPA_GAS_INIT_REQ;
		break;
	case PDPA_GAS_INIT_RSP:
		frm_type = MGMT_ACTION_PDPA_GAS_INIT_RSP;
		break;
	case PDPA_GAS_COMEBACK_REQ:
		frm_type = MGMT_ACTION_PDPA_GAS_COMEBACK_REQ;
		break;
	case PDPA_GAS_COMEBACK_RSP:
		frm_type = MGMT_ACTION_PDPA_GAS_COMEBACK_RSP;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_wnm_action_subtype() - gets wnm action subtype
 * @action_code: action code
 *
 * This function returns the subtype for wnm action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_wnm_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case WNM_BSS_TM_QUERY:
		frm_type = MGMT_ACTION_WNM_BSS_TM_QUERY;
		break;
	case WNM_BSS_TM_REQUEST:
		frm_type = MGMT_ACTION_WNM_BSS_TM_REQUEST;
		break;
	case WNM_BSS_TM_RESPONSE:
		frm_type = MGMT_ACTION_WNM_BSS_TM_RESPONSE;
		break;
	case WNM_NOTIF_REQUEST:
		frm_type = MGMT_ACTION_WNM_NOTIF_REQUEST;
		break;
	case WNM_NOTIF_RESPONSE:
		frm_type = MGMT_ACTION_WNM_NOTIF_RESPONSE;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_wnm_action_subtype() - gets tdls action subtype
 * @action_code: action code
 *
 * This function returns the subtype for tdls action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_tdls_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case TDLS_SETUP_REQUEST:
		frm_type = MGMT_ACTION_TDLS_SETUP_REQ;
		break;
	case TDLS_SETUP_RESPONSE:
		frm_type = MGMT_ACTION_TDLS_SETUP_RSP;
		break;
	case TDLS_SETUP_CONFIRM:
		frm_type = MGMT_ACTION_TDLS_SETUP_CNF;
		break;
	case TDLS_TEARDOWN:
		frm_type = MGMT_ACTION_TDLS_TEARDOWN;
		break;
	case TDLS_PEER_TRAFFIC_INDICATION:
		frm_type = MGMT_ACTION_TDLS_PEER_TRAFFIC_IND;
		break;
	case TDLS_CHANNEL_SWITCH_REQUEST:
		frm_type = MGMT_ACTION_TDLS_CH_SWITCH_REQ;
		break;
	case TDLS_CHANNEL_SWITCH_RESPONSE:
		frm_type = MGMT_ACTION_TDLS_CH_SWITCH_RSP;
		break;
	case TDLS_PEER_PSM_REQUEST:
		frm_type = MGMT_ACTION_TDLS_PEER_PSM_REQUEST;
		break;
	case TDLS_PEER_PSM_RESPONSE:
		frm_type = MGMT_ACTION_TDLS_PEER_PSM_RESPONSE;
		break;
	case TDLS_PEER_TRAFFIC_RESPONSE:
		frm_type = MGMT_ACTION_TDLS_PEER_TRAFFIC_RSP;
		break;
	case TDLS_DISCOVERY_REQUEST:
		frm_type = MGMT_ACTION_TDLS_DIS_REQ;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_mesh_action_subtype() - gets mesh action subtype
 * @action_code: action code
 *
 * This function returns the subtype for mesh action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_mesh_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case MESH_ACTION_LINK_METRIC_REPORT:
		frm_type = MGMT_ACTION_MESH_LINK_METRIC_REPORT;
		break;
	case MESH_ACTION_HWMP_PATH_SELECTION:
		frm_type = MGMT_ACTION_MESH_HWMP_PATH_SELECTION;
		break;
	case MESH_ACTION_GATE_ANNOUNCEMENT:
		frm_type = MGMT_ACTION_MESH_GATE_ANNOUNCEMENT;
		break;
	case MESH_ACTION_CONGESTION_CONTROL_NOTIFICATION:
		frm_type = MGMT_ACTION_MESH_CONGESTION_CONTROL_NOTIFICATION;
		break;
	case MESH_ACTION_MCCA_SETUP_REQUEST:
		frm_type = MGMT_ACTION_MESH_MCCA_SETUP_REQUEST;
		break;
	case MESH_ACTION_MCCA_SETUP_REPLY:
		frm_type = MGMT_ACTION_MESH_MCCA_SETUP_REPLY;
		break;
	case MESH_ACTION_MCCA_ADVERTISEMENT_REQUEST:
		frm_type = MGMT_ACTION_MESH_MCCA_ADVERTISEMENT_REQUEST;
		break;
	case MESH_ACTION_MCCA_ADVERTISEMENT:
		frm_type = MGMT_ACTION_MESH_MCCA_ADVERTISEMENT;
		break;
	case MESH_ACTION_MCCA_TEARDOWN:
		frm_type = MGMT_ACTION_MESH_MCCA_TEARDOWN;
		break;
	case MESH_ACTION_TBTT_ADJUSTMENT_REQUEST:
		frm_type = MGMT_ACTION_MESH_TBTT_ADJUSTMENT_REQUEST;
		break;
	case MESH_ACTION_TBTT_ADJUSTMENT_RESPONSE:
		frm_type = MGMT_ACTION_MESH_TBTT_ADJUSTMENT_RESPONSE;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_self_prot_action_subtype() - gets self prot. action subtype
 * @action_code: action code
 *
 * This function returns the subtype for self protected action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_self_prot_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case SP_MESH_PEERING_OPEN:
		frm_type = MGMT_ACTION_SP_MESH_PEERING_OPEN;
		break;
	case SP_MESH_PEERING_CONFIRM:
		frm_type = MGMT_ACTION_SP_MESH_PEERING_CONFIRM;
		break;
	case SP_MESH_PEERING_CLOSE:
		frm_type = MGMT_ACTION_SP_MESH_PEERING_CLOSE;
		break;
	case SP_MGK_INFORM:
		frm_type = MGMT_ACTION_SP_MGK_INFORM;
		break;
	case SP_MGK_ACK:
		frm_type = MGMT_ACTION_SP_MGK_ACK;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_wmm_action_subtype() - gets wmm action subtype
 * @action_code: action code
 *
 * This function returns the subtype for wmm action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_wmm_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case WMM_QOS_SETUP_REQ:
		frm_type = MGMT_ACTION_WMM_QOS_SETUP_REQ;
		break;
	case WMM_QOS_SETUP_RESP:
		frm_type = MGMT_ACTION_WMM_QOS_SETUP_RESP;
		break;
	case WMM_QOS_TEARDOWN:
		frm_type = MGMT_ACTION_WMM_QOS_TEARDOWN;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_get_vht_action_subtype() - gets vht action subtype
 * @action_code: action code
 *
 * This function returns the subtype for vht action
 * category.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_get_vht_action_subtype(uint8_t action_code)
{
	enum mgmt_frame_type frm_type;

	switch (action_code) {
	case VHT_ACTION_COMPRESSED_BF:
		frm_type = MGMT_ACTION_VHT_COMPRESSED_BF;
		break;
	case VHT_ACTION_GID_NOTIF:
		frm_type = MGMT_ACTION_VHT_GID_NOTIF;
		break;
	case VHT_ACTION_OPMODE_NOTIF:
		frm_type = MGMT_ACTION_VHT_OPMODE_NOTIF;
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_txrx_get_action_frm_subtype() - gets action frm subtype
 * @mpdu_data_ptr: pointer to mpdu data
 *
 * This function determines the action category of the frame
 * and calls respective function to get mgmt frame type.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_txrx_get_action_frm_subtype(uint8_t *mpdu_data_ptr)
{
	struct action_frm_hdr *action_hdr =
			(struct action_frm_hdr *)mpdu_data_ptr;
	enum mgmt_frame_type frm_type;

	switch (action_hdr->action_category) {
	case ACTION_CATEGORY_SPECTRUM_MGMT:
		frm_type = mgmt_get_spec_mgmt_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_QOS:
		frm_type = mgmt_get_qos_action_subtype(action_hdr->action_code);
		break;
	case ACTION_CATEGORY_DLS:
		frm_type = mgmt_get_dls_action_subtype(action_hdr->action_code);
		break;
	case ACTION_CATEGORY_BACK:
		frm_type = mgmt_get_back_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_PUBLIC:
		frm_type = mgmt_get_public_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_RRM:
		frm_type = mgmt_get_rrm_action_subtype(action_hdr->action_code);
		break;
	case ACTION_CATEGORY_HT:
		frm_type = mgmt_get_ht_action_subtype(action_hdr->action_code);
		break;
	case ACTION_CATEGORY_SA_QUERY:
		frm_type = mgmt_get_sa_query_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_PROTECTED_DUAL_OF_PUBLIC_ACTION:
		frm_type = mgmt_get_pdpa_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_WNM:
		frm_type = mgmt_get_wnm_action_subtype(action_hdr->action_code);
		break;
	case ACTION_CATEGORY_TDLS:
		frm_type = mgmt_get_tdls_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_MESH_ACTION:
		frm_type = mgmt_get_mesh_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_SELF_PROTECTED:
		frm_type = mgmt_get_self_prot_action_subtype(
						action_hdr->action_code);
		break;
	case ACTION_CATEGORY_WMM:
		frm_type = mgmt_get_wmm_action_subtype(action_hdr->action_code);
		break;
	case ACTION_CATEGORY_VHT:
		frm_type = mgmt_get_vht_action_subtype(action_hdr->action_code);
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

/**
 * mgmt_txrx_get_frm_type() - gets mgmt frm type
 * @mgmt_subtype: mgmt subtype
 * @mpdu_data_ptr: pointer to mpdu data
 *
 * This function returns mgmt frame type of the frame
 * based on the mgmt subtype.
 *
 * Return: mgmt frame type
 */
static enum mgmt_frame_type
mgmt_txrx_get_frm_type(uint8_t mgmt_subtype, uint8_t *mpdu_data_ptr)
{
	enum mgmt_frame_type frm_type;

	switch (mgmt_subtype) {
	case MGMT_SUBTYPE_ASSOC_REQ:
		frm_type = MGMT_ASSOC_REQ;
		break;
	case MGMT_SUBTYPE_ASSOC_RESP:
		frm_type = MGMT_ASSOC_RESP;
		break;
	case MGMT_SUBTYPE_REASSOC_REQ:
		frm_type = MGMT_ASSOC_REQ;
		break;
	case MGMT_SUBTYPE_REASSOC_RESP:
		frm_type = MGMT_REASSOC_RESP;
		break;
	case MGMT_SUBTYPE_PROBE_REQ:
		frm_type = MGMT_PROBE_REQ;
		break;
	case MGMT_SUBTYPE_PROBE_RESP:
		frm_type = MGMT_PROBE_RESP;
		break;
	case MGMT_SUBTYPE_BEACON:
		frm_type = MGMT_BEACON;
		break;
	case MGMT_SUBTYPE_ATIM:
		frm_type = MGMT_ATIM;
		break;
	case MGMT_SUBTYPE_DISASSOC:
		frm_type = MGMT_DISASSOC;
		break;
	case MGMT_SUBTYPE_AUTH:
		frm_type = MGMT_AUTH;
		break;
	case MGMT_SUBTYPE_DEAUTH:
		frm_type = MGMT_DEAUTH;
		break;
	case MGMT_SUBTYPE_ACTION:
	case MGMT_SUBTYPE_ACTION_NO_ACK:
		frm_type = mgmt_txrx_get_action_frm_subtype(mpdu_data_ptr);
		break;
	default:
		frm_type = MGMT_FRM_UNSPECIFIED;
		break;
	}

	return frm_type;
}

QDF_STATUS tgt_mgmt_txrx_rx_frame_handler(
			struct wlan_objmgr_psoc *psoc,
			qdf_nbuf_t buf, void *params)
{
	struct mgmt_txrx_priv_context *mgmt_txrx_ctx;
	struct ieee80211_frame *wh;
	qdf_nbuf_t copy_buf;
	struct wlan_objmgr_peer *peer = NULL;
	uint8_t mgmt_type, mgmt_subtype;
	uint8_t *mac_addr, *mpdu_data_ptr;
	enum mgmt_frame_type frm_type;
	struct mgmt_rx_handler *rx_handler, *rx_handler_node;
	struct mgmt_rx_handler *rx_handler_head = NULL, *rx_handler_tail = NULL;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (!psoc) {
		mgmt_txrx_err("psoc_ctx passed is NULL");
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_INVAL;
	}

	if (!buf) {
		mgmt_txrx_err("buffer passed is NULL");
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_INVAL;
	}

	wh = (struct ieee80211_frame *)qdf_nbuf_data(buf);

	/* peer can be NULL in following 2 scenarios:
	 * 1. broadcast frame received
	 * 2. operating in monitor mode
	 *
	 * and in both scenarios, the receiver of frame
	 * is expected to do processing accordingly considerng
	 * the fact that peer = NULL can be received and is a valid
	 * scenario.
	 */
	mac_addr = (uint8_t *)wh->i_addr2;
	peer = wlan_objmgr_find_peer(psoc, mac_addr);
	if (!peer) {
		mac_addr = (uint8_t *)wh->i_addr1;
		peer = wlan_objmgr_find_peer(psoc, mac_addr);
	}

	/**
	 * TO DO (calculate pdev)
	 * Waiting for a new parameter: pdev id to get added in rx event
	 */

	mgmt_type = (wh)->i_fc[0] & IEEE80211_FC0_TYPE_MASK;
	mgmt_subtype = (wh)->i_fc[0] & IEEE80211_FC0_SUBTYPE_MASK;

	if (mgmt_type != IEEE80211_FC0_TYPE_MGT) {
		mgmt_txrx_err("Rx event doesn't conatin a mgmt. packet, %d",
			mgmt_type);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	/* mpdu_data_ptr is pointer to action header */
	mpdu_data_ptr = (uint8_t *)qdf_nbuf_data(buf) +
			sizeof(struct ieee80211_frame);
	frm_type = mgmt_txrx_get_frm_type(mgmt_subtype, mpdu_data_ptr);
	if (frm_type == MGMT_FRM_UNSPECIFIED) {
		mgmt_txrx_err("Unspecified mgmt frame type");
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	mgmt_txrx_info("Rcvd mgmt frame, mgmt txrx frm type: %u, seq. no.: %u, peer: %p",
			frm_type, *(uint16_t *)wh->i_seq, peer);

	mgmt_txrx_ctx = (struct mgmt_txrx_priv_context *)
			wlan_objmgr_psoc_get_comp_private_obj(psoc,
				WLAN_UMAC_COMP_MGMT_TXRX);
	qdf_spin_lock_bh(&mgmt_txrx_ctx->mgmt_txrx_ctx_lock);
	rx_handler = mgmt_txrx_ctx->mgmt_rx_comp_cb[frm_type];
	if (!rx_handler) {
		qdf_spin_unlock_bh(&mgmt_txrx_ctx->mgmt_txrx_ctx_lock);
		mgmt_txrx_info("No rx callback registered for frm_type: %d",
			frm_type);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	while (rx_handler) {
		rx_handler_node = qdf_mem_malloc(sizeof(*rx_handler_node));
		if (!rx_handler_node) {
			qdf_spin_unlock_bh(&mgmt_txrx_ctx->mgmt_txrx_ctx_lock);
			mgmt_txrx_err("Couldn't allocate memory for rx handler node");
			qdf_nbuf_free(buf);
			status = QDF_STATUS_E_NOMEM;
			goto rx_handler_mem_free;
		}

		rx_handler_node->comp_id = rx_handler->comp_id;
		rx_handler_node->rx_cb = rx_handler->rx_cb;
		rx_handler_node->next = NULL;

		if (!rx_handler_head) {
			rx_handler_head = rx_handler_node;
			rx_handler_tail = rx_handler_head;
		} else {
			rx_handler_tail->next = rx_handler_node;
			rx_handler_tail = rx_handler_tail->next;
		}
		rx_handler = rx_handler->next;
	}
	qdf_spin_unlock_bh(&mgmt_txrx_ctx->mgmt_txrx_ctx_lock);

	rx_handler = rx_handler_head;
	while (rx_handler->next) {
		copy_buf = qdf_nbuf_clone(buf);
		rx_handler->rx_cb(psoc, peer, copy_buf,
					params, frm_type);
		rx_handler = rx_handler->next;
	}
	rx_handler->rx_cb(psoc, peer, buf,
				params, frm_type);

rx_handler_mem_free:
	while (rx_handler_head) {
		rx_handler = rx_handler_head;
		rx_handler_head = rx_handler_head->next;
		qdf_mem_free(rx_handler);
	}
	return status;
}

QDF_STATUS tgt_mgmt_txrx_tx_completion_handler(
			struct wlan_objmgr_psoc *psoc,
			uint32_t desc_id, uint32_t status,
			void *tx_compl_params)
{
	return QDF_STATUS_SUCCESS;
}

qdf_nbuf_t tgt_mgmt_txrx_get_nbuf_from_desc_id(
			struct wlan_objmgr_psoc *psoc,
			uint32_t desc_id)
{
	return NULL;
}

struct wlan_objmgr_peer *
tgt_mgmt_txrx_get_peer_from_desc_id(
			struct wlan_objmgr_psoc *psoc,
			uint32_t desc_id)
{
	return NULL;
}

uint8_t tgt_mgmt_txrx_get_vdev_id_from_desc_id(
			struct wlan_objmgr_psoc *psoc,
			uint32_t desc_id)
{
	return 0;
}
