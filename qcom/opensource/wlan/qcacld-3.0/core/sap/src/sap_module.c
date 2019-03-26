/*
 * Copyright (c) 2012-2019 The Linux Foundation. All rights reserved.
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
 * ===========================================================================
 *                     sap_module.c
 *  OVERVIEW:
 *  This software unit holds the implementation of the WLAN SAP modules
 *  functions providing EXTERNAL APIs. It is also where the global SAP module
 *  context gets initialised
 *  DEPENDENCIES:
 *  Are listed for each API below.
 * ===========================================================================
 */

/* $Header$ */

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "qdf_trace.h"
#include "qdf_util.h"
#include "qdf_atomic.h"
/* Pick up the sme callback registration API */
#include "sme_api.h"

/* SAP API header file */

#include "sap_internal.h"
#include "sme_inside.h"
#include "cds_ieee80211_common_i.h"
#include "cds_regdomain.h"
#include "wlan_policy_mgr_api.h"
#include <wlan_scan_ucfg_api.h>
#include "wlan_reg_services_api.h"
#include <wlan_dfs_utils_api.h>
#include <wlan_reg_ucfg_api.h>
#include <wlan_cfg80211_crypto.h>
#include <wlan_crypto_global_api.h>

#define SAP_DEBUG
static struct sap_context *gp_sap_ctx[SAP_MAX_NUM_SESSION];
static qdf_atomic_t sap_ctx_ref_count[SAP_MAX_NUM_SESSION];
static qdf_mutex_t sap_context_lock;

/**
 * wlansap_global_init() - Initialize SAP globals
 *
 * Initializes the SAP global data structures
 *
 * Return: QDF_STATUS
 */
QDF_STATUS wlansap_global_init(void)
{
	uint32_t i;

	if (QDF_IS_STATUS_ERROR(qdf_mutex_create(&sap_context_lock))) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "failed to init sap_context_lock");
		return QDF_STATUS_E_FAULT;
	}

	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		gp_sap_ctx[i] = NULL;
		qdf_atomic_init(&sap_ctx_ref_count[i]);
	}

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
			"%s: sap global context initialized", __func__);

	return QDF_STATUS_SUCCESS;
}

/**
 * wlansap_global_deinit() - De-initialize SAP globals
 *
 * De-initializes the SAP global data structures
 *
 * Return: QDF_STATUS
 */
QDF_STATUS wlansap_global_deinit(void)
{
	uint32_t i;

	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		if (gp_sap_ctx[i]) {
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				"we could be leaking context:%d", i);
		}
		gp_sap_ctx[i] = NULL;
		qdf_atomic_init(&sap_ctx_ref_count[i]);
	}

	if (QDF_IS_STATUS_ERROR(qdf_mutex_destroy(&sap_context_lock))) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				"failed to destroy sap_context_lock");
		return QDF_STATUS_E_FAULT;
	}

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
			"%s: sap global context deinitialized", __func__);

	return QDF_STATUS_SUCCESS;
}

/**
 * wlansap_save_context() - Save the context in global SAP context
 * @ctx: SAP context to be stored
 *
 * Stores the given SAP context in the global SAP context array
 *
 * Return: QDF_STATUS
 */
static QDF_STATUS wlansap_save_context(struct sap_context *ctx)
{
	uint32_t i;

	qdf_mutex_acquire(&sap_context_lock);
	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		if (!gp_sap_ctx[i]) {
			gp_sap_ctx[i] = ctx;
			qdf_atomic_inc(&sap_ctx_ref_count[i]);
			qdf_mutex_release(&sap_context_lock);
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
				"%s: sap context saved at index: %d",
				__func__, i);
			return QDF_STATUS_SUCCESS;
		}
	}
	qdf_mutex_release(&sap_context_lock);

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
		"%s: failed to save sap context", __func__);

	return QDF_STATUS_E_FAILURE;
}

/**
 * wlansap_context_get() - Verify SAP context and increment ref count
 * @ctx: Context to be checked
 *
 * Verifies the SAP context and increments the reference count maintained for
 * the corresponding SAP context.
 *
 * Return: QDF_STATUS
 */
QDF_STATUS wlansap_context_get(struct sap_context *ctx)
{
	uint32_t i;

	qdf_mutex_acquire(&sap_context_lock);
	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		if (ctx && (gp_sap_ctx[i] == ctx)) {
			qdf_atomic_inc(&sap_ctx_ref_count[i]);
			qdf_mutex_release(&sap_context_lock);
			return QDF_STATUS_SUCCESS;
		}
	}
	qdf_mutex_release(&sap_context_lock);

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
			"%s: sap session is not valid", __func__);
	return QDF_STATUS_E_FAILURE;
}

/**
 * wlansap_context_put() - Check the reference count and free SAP context
 * @ctx: SAP context to be checked and freed
 *
 * Checks the reference count and frees the SAP context
 *
 * Return: None
 */
void wlansap_context_put(struct sap_context *ctx)
{
	uint32_t i;

	if (!ctx)
		return;

	qdf_mutex_acquire(&sap_context_lock);
	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		if (gp_sap_ctx[i] == ctx) {
			if (qdf_atomic_dec_and_test(&sap_ctx_ref_count[i])) {
				if (ctx->channelList) {
					qdf_mem_free(ctx->channelList);
					ctx->channelList = NULL;
					ctx->num_of_channel = 0;
				}
				qdf_mem_free(ctx);
				gp_sap_ctx[i] = NULL;
				QDF_TRACE(QDF_MODULE_ID_SAP,
					QDF_TRACE_LEVEL_DEBUG,
					"%s: sap session freed: %d",
					__func__, i);
			}
			qdf_mutex_release(&sap_context_lock);
			return;
		}
	}
	qdf_mutex_release(&sap_context_lock);
}

struct sap_context *sap_create_ctx(void)
{
	struct sap_context *sap_ctx;
	QDF_STATUS status;

	/* dynamically allocate the sap_ctx */
	sap_ctx = qdf_mem_malloc(sizeof(*sap_ctx));
	if (!sap_ctx)
		return NULL;

	/* Clean up SAP control block, initialize all values */
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG, FL("Enter"));

	/* Save the SAP context pointer */
	status = wlansap_save_context(sap_ctx);
	if (QDF_IS_STATUS_ERROR(status)) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%s: failed to save SAP context", __func__);
		qdf_mem_free(sap_ctx);
		return NULL;
	}
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG, FL("Exit"));

	return sap_ctx;
} /* sap_create_ctx */

static QDF_STATUS wlansap_owe_init(struct sap_context *sap_ctx)
{
	qdf_list_create(&sap_ctx->owe_pending_assoc_ind_list, 0);

	return QDF_STATUS_SUCCESS;
}

static void wlansap_owe_cleanup(struct sap_context *sap_ctx)
{
	struct mac_context *mac;
	struct owe_assoc_ind *owe_assoc_ind;
	struct assoc_ind *assoc_ind = NULL;
	qdf_list_node_t *node = NULL, *next_node = NULL;
	QDF_STATUS status;

	if (!sap_ctx) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid SAP context");
		return;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return;
	}

	if (QDF_STATUS_SUCCESS !=
	    qdf_list_peek_front(&sap_ctx->owe_pending_assoc_ind_list,
				&node)) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
				"Failed to find assoc ind list");
		return;
	}

	while (node) {
		qdf_list_peek_next(&sap_ctx->owe_pending_assoc_ind_list,
				   node, &next_node);
		owe_assoc_ind = qdf_container_of(node, struct owe_assoc_ind,
						 node);
		status = qdf_list_remove_node(
					   &sap_ctx->owe_pending_assoc_ind_list,
					   node);
		if (status == QDF_STATUS_SUCCESS) {
			assoc_ind = owe_assoc_ind->assoc_ind;
			qdf_mem_free(owe_assoc_ind);
			assoc_ind->owe_ie = NULL;
			assoc_ind->owe_ie_len = 0;
			assoc_ind->owe_status = eSIR_MAC_UNSPEC_FAILURE_STATUS;
			status = sme_update_owe_info(mac, assoc_ind);
			qdf_mem_free(assoc_ind);
		} else {
			QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
					"Failed to remove assoc ind");
		}
		node = next_node;
		next_node = NULL;
	}
}

static void wlansap_owe_deinit(struct sap_context *sap_ctx)
{
	qdf_list_destroy(&sap_ctx->owe_pending_assoc_ind_list);
}

QDF_STATUS sap_init_ctx(struct sap_context *sap_ctx,
			 enum QDF_OPMODE mode,
			 uint8_t *addr, uint32_t session_id, bool reinit)
{
	QDF_STATUS status;
	struct mac_context *mac;

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "wlansap_start invoked successfully");

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	/* Now configure the roaming profile links. To SSID and bssid.*/
	/* We have room for two SSIDs. */
	sap_ctx->csr_roamProfile.SSIDs.numOfSSIDs = 1;   /* This is true for now. */
	sap_ctx->csr_roamProfile.SSIDs.SSIDList = sap_ctx->SSIDList;     /* Array of two */
	sap_ctx->csr_roamProfile.SSIDs.SSIDList[0].SSID.length = 0;
	sap_ctx->csr_roamProfile.SSIDs.SSIDList[0].handoffPermitted = false;
	sap_ctx->csr_roamProfile.SSIDs.SSIDList[0].ssidHidden =
		sap_ctx->SSIDList[0].ssidHidden;

	sap_ctx->csr_roamProfile.BSSIDs.numOfBSSIDs = 1; /* This is true for now. */
	sap_ctx->csr_roamProfile.BSSIDs.bssid = &sap_ctx->bssid;
	sap_ctx->csr_roamProfile.csrPersona = mode;
	qdf_mem_copy(sap_ctx->self_mac_addr, addr, QDF_MAC_ADDR_SIZE);

	/* Now configure the auth type in the roaming profile. To open. */
	sap_ctx->csr_roamProfile.negotiatedAuthType = eCSR_AUTH_TYPE_OPEN_SYSTEM;        /* open is the default */

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_INVAL;
	}

	status = sap_set_session_param(MAC_HANDLE(mac), sap_ctx, session_id);
	if (QDF_STATUS_SUCCESS != status) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"In %s calling sap_set_session_param status = %d",
			__func__, status);
		return QDF_STATUS_E_FAILURE;
	}
	/* Register with scan component only during init */
	if (!reinit)
		sap_ctx->req_id =
			ucfg_scan_register_requester(mac->psoc, "SAP",
					sap_scan_event_callback, sap_ctx);

	if (!reinit) {
		status = wlansap_owe_init(sap_ctx);
		if (QDF_STATUS_SUCCESS != status) {
			QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
					"OWE init failed");
			return QDF_STATUS_E_FAILURE;
		}
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS sap_deinit_ctx(struct sap_context *sap_ctx)
{
	struct mac_context *mac;

	/* Sanity check - Extract SAP control block */
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
		  "wlansap_stop invoked successfully ");

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	wlansap_owe_cleanup(sap_ctx);
	wlansap_owe_deinit(sap_ctx);

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}
	ucfg_scan_unregister_requester(mac->psoc, sap_ctx->req_id);

	if (sap_ctx->channelList) {
		qdf_mem_free(sap_ctx->channelList);
		sap_ctx->channelList = NULL;
		sap_ctx->num_of_channel = 0;
	}
	sap_free_roam_profile(&sap_ctx->csr_roamProfile);
	if (sap_ctx->sessionId != WLAN_UMAC_VDEV_ID_MAX) {
		/* empty queues/lists/pkts if any */
		sap_clear_session_param(MAC_HANDLE(mac), sap_ctx,
					sap_ctx->sessionId);
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS sap_destroy_ctx(struct sap_context *sap_ctx)
{
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  FL("Enter"));

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}
	/* Cleanup SAP control block */
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG, FL("Enter"));
	/*
	 * wlansap_context_put will release actual sap_ctx memory
	 * allocated during sap_create_ctx
	 */
	wlansap_context_put(sap_ctx);
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG, FL("Exit"));

	return QDF_STATUS_SUCCESS;
} /* sap_destroy_ctx */

bool wlansap_is_channel_in_nol_list(struct sap_context *sap_ctx,
				    uint8_t channelNumber,
				    ePhyChanBondState chanBondState)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			  "%s: Invalid SAP pointer from pCtx", __func__);
		return QDF_STATUS_E_FAULT;
	}

	return sap_dfs_is_channel_in_nol_list(sap_ctx, channelNumber,
					      chanBondState);
}

static QDF_STATUS wlansap_mark_leaking_channel(struct wlan_objmgr_pdev *pdev,
		uint8_t *leakage_adjusted_lst,
		uint8_t chan_bw)
{

	return utils_dfs_mark_leaking_ch(pdev, chan_bw, 1,
			leakage_adjusted_lst);
}

bool wlansap_is_channel_leaking_in_nol(struct sap_context *sap_ctx,
				       uint8_t channel,
				       uint8_t chan_bw)
{
	struct mac_context *mac_ctx;
	uint8_t leakage_adjusted_lst[1];

	leakage_adjusted_lst[0] = channel;
	mac_ctx = sap_get_mac_context();
	if (!mac_ctx) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}
	if (QDF_IS_STATUS_ERROR(wlansap_mark_leaking_channel(mac_ctx->pdev,
			leakage_adjusted_lst, chan_bw)))
		return true;

	if (!leakage_adjusted_lst[0])
		return true;

	return false;
}

#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
uint16_t wlansap_check_cc_intf(struct sap_context *sap_ctx)
{
	struct mac_context *mac;
	uint16_t intf_ch;
	eCsrPhyMode phy_mode;

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return 0;
	}
	phy_mode = sap_ctx->csr_roamProfile.phyMode;
	intf_ch = sme_check_concurrent_channel_overlap(MAC_HANDLE(mac),
						       sap_ctx->channel,
						       phy_mode,
						       sap_ctx->cc_switch_mode);
	return intf_ch;
}
#endif

 /**
  * wlansap_set_scan_acs_channel_params() - Config scan and channel parameters.
  * pconfig:                                Pointer to the SAP config
  * psap_ctx:                               Pointer to the SAP Context.
  * pusr_context:                           Parameter that will be passed
  *                                         back in all the SAP callback events.
  *
  * This api function is used to copy Scan and Channel parameters from sap
  * config to sap context.
  *
  * Return:                                 The result code associated with
  *                                         performing the operation
  */
static QDF_STATUS
wlansap_set_scan_acs_channel_params(tsap_config_t *pconfig,
				    struct sap_context *psap_ctx,
				    void *pusr_context)
{
	struct mac_context *mac;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (!pconfig) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%s: Invalid pconfig passed ", __func__);
		return QDF_STATUS_E_FAULT;
	}

	if (!psap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%s: Invalid pconfig passed ", __func__);
		return QDF_STATUS_E_FAULT;
	}

	/* Channel selection is auto or configured */
	psap_ctx->channel = pconfig->channel;
	psap_ctx->dfs_mode = pconfig->acs_dfs_mode;
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
	psap_ctx->cc_switch_mode = pconfig->cc_switch_mode;
#endif
	psap_ctx->auto_channel_select_weight =
		 pconfig->auto_channel_select_weight;
	psap_ctx->pUsrContext = pusr_context;
	psap_ctx->enableOverLapCh = pconfig->enOverLapCh;
	psap_ctx->acs_cfg = &pconfig->acs_cfg;
	psap_ctx->ch_width_orig = pconfig->acs_cfg.ch_width;
	psap_ctx->secondary_ch = pconfig->sec_ch;

	/*
	 * Set the BSSID to your "self MAC Addr" read
	 * the mac address from Configuation ITEM received
	 * from HDD
	 */
	psap_ctx->csr_roamProfile.BSSIDs.numOfBSSIDs = 1;

	/* Save a copy to SAP context */
	qdf_mem_copy(psap_ctx->csr_roamProfile.BSSIDs.bssid,
		pconfig->self_macaddr.bytes, QDF_MAC_ADDR_SIZE);
	qdf_mem_copy(psap_ctx->self_mac_addr,
		pconfig->self_macaddr.bytes, QDF_MAC_ADDR_SIZE);

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	return status;
}

/**
 * wlan_sap_get_roam_profile() - Returns sap roam profile.
 * @sap_ctx:	Pointer to Sap Context.
 *
 * This function provides the SAP roam profile.
 *
 * Return: SAP RoamProfile
 */
struct csr_roam_profile *wlan_sap_get_roam_profile(struct sap_context *sap_ctx)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP pointer from ctx"));
		return NULL;
	}
	return &sap_ctx->csr_roamProfile;
}

eCsrPhyMode wlan_sap_get_phymode(struct sap_context *sap_ctx)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP pointer from ctx"));
		return 0;
	}
	return sap_ctx->csr_roamProfile.phyMode;
}

uint32_t wlan_sap_get_vht_ch_width(struct sap_context *sap_ctx)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP pointer"));
		return 0;
	}

	return sap_ctx->ch_params.ch_width;
}

void wlan_sap_set_vht_ch_width(struct sap_context *sap_ctx,
			       uint32_t vht_channel_width)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP pointer"));
		return;
	}

	sap_ctx->ch_params.ch_width = vht_channel_width;
}

/**
 * wlan_sap_validate_channel_switch() - validate target channel switch w.r.t
 *      concurreny rules set to avoid channel interference.
 * @mac_handle: Opaque handle to the global MAC context
 * @sap_ch: channel to switch
 * @sap_context: sap session context
 *
 * Return: true if there is no channel interference else return false
 */
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
static bool wlan_sap_validate_channel_switch(mac_handle_t mac_handle,
					     uint16_t sap_ch,
					     struct sap_context *sap_context)
{
	return sme_validate_sap_channel_switch(
			mac_handle,
			sap_ch,
			sap_context->csr_roamProfile.phyMode,
			sap_context->cc_switch_mode,
			sap_context->sessionId);
}
#else
static bool wlan_sap_validate_channel_switch(mac_handle_t mac_handle,
					     uint16_t sap_ch,
					     struct sap_context *sap_context)
{
	return true;
}
#endif

void wlan_sap_set_sap_ctx_acs_cfg(struct sap_context *sap_ctx,
				  tsap_config_t *sap_config)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			  "%s: Invalid SAP pointer",
			  __func__);
		return;
	}

	sap_ctx->acs_cfg = &sap_config->acs_cfg;
}

QDF_STATUS wlansap_start_bss(struct sap_context *sap_ctx,
			     tpWLAN_SAPEventCB pSapEventCallback,
			     tsap_config_t *config, void *pUsrContext)
{
	tWLAN_SAPEvent sap_event;        /* State machine event */
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	struct mac_context *pmac = NULL;

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  FL("sap_ctx=%pK"), sap_ctx);

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			  "%s: Invalid SAP pointer",
			  __func__);
		return QDF_STATUS_E_FAULT;
	}
	sap_ctx->fsm_state = SAP_INIT;

	/* Channel selection is auto or configured */
	sap_ctx->channel = config->channel;
	sap_ctx->dfs_mode = config->acs_dfs_mode;
	sap_ctx->ch_params.ch_width = config->ch_params.ch_width;
	sap_ctx->ch_params.center_freq_seg0 =
		config->ch_params.center_freq_seg0;
	sap_ctx->ch_params.center_freq_seg1 =
		config->ch_params.center_freq_seg1;
	sap_ctx->ch_params.sec_ch_offset =
		config->ch_params.sec_ch_offset;
	sap_ctx->ch_width_orig = config->ch_width_orig;
#ifdef FEATURE_WLAN_MCC_TO_SCC_SWITCH
	sap_ctx->cc_switch_mode = config->cc_switch_mode;
#endif
	sap_ctx->auto_channel_select_weight =
		 config->auto_channel_select_weight;
	sap_ctx->pUsrContext = pUsrContext;
	sap_ctx->enableOverLapCh = config->enOverLapCh;
	sap_ctx->acs_cfg = &config->acs_cfg;
	sap_ctx->secondary_ch = config->sec_ch;
	sap_ctx->dfs_cac_offload = config->dfs_cac_offload;
	sap_ctx->isCacEndNotified = false;
	sap_ctx->is_chan_change_inprogress = false;

	/* Set the BSSID to your "self MAC Addr" read the mac address
		from Configuation ITEM received from HDD */
	sap_ctx->csr_roamProfile.BSSIDs.numOfBSSIDs = 1;
	qdf_mem_copy(sap_ctx->csr_roamProfile.BSSIDs.bssid,
		     sap_ctx->self_mac_addr, sizeof(struct qdf_mac_addr));

	/* Save a copy to SAP context */
	qdf_mem_copy(sap_ctx->csr_roamProfile.BSSIDs.bssid,
		     config->self_macaddr.bytes, QDF_MAC_ADDR_SIZE);
	qdf_mem_copy(sap_ctx->self_mac_addr,
		     config->self_macaddr.bytes, QDF_MAC_ADDR_SIZE);

	/* copy the configuration items to csrProfile */
	sapconvert_to_csr_profile(config, eCSR_BSS_TYPE_INFRA_AP,
			       &sap_ctx->csr_roamProfile);
	pmac = sap_get_mac_context();
	if (!pmac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		qdf_status = QDF_STATUS_E_FAULT;
		goto fail;
	}

	/*
	 * Copy the DFS Test Mode setting to pmac for
	 * access in lower layers
	 */
	pmac->sap.SapDfsInfo.sap_ch_switch_beacon_cnt =
				config->sap_chanswitch_beacon_cnt;
	pmac->sap.SapDfsInfo.sap_ch_switch_mode =
			config->sap_chanswitch_mode;

	pmac->sap.sapCtxList[sap_ctx->sessionId].sap_context = sap_ctx;
	pmac->sap.sapCtxList[sap_ctx->sessionId].sapPersona =
		sap_ctx->csr_roamProfile.csrPersona;
	pmac->sap.SapDfsInfo.reduced_beacon_interval =
				config->reduced_beacon_interval;

	/* Copy MAC filtering settings to sap context */
	sap_ctx->eSapMacAddrAclMode = config->SapMacaddr_acl;
	qdf_mem_copy(sap_ctx->acceptMacList, config->accept_mac,
		     sizeof(config->accept_mac));
	sap_ctx->nAcceptMac = config->num_accept_mac;
	sap_sort_mac_list(sap_ctx->acceptMacList, sap_ctx->nAcceptMac);
	qdf_mem_copy(sap_ctx->denyMacList, config->deny_mac,
		     sizeof(config->deny_mac));
	sap_ctx->nDenyMac = config->num_deny_mac;
	sap_sort_mac_list(sap_ctx->denyMacList, sap_ctx->nDenyMac);
	sap_ctx->beacon_tx_rate = config->beacon_tx_rate;

	/* Fill in the event structure for FSM */
	sap_event.event = eSAP_HDD_START_INFRA_BSS;
	sap_event.params = 0;    /* pSapPhysLinkCreate */

	/* Store the HDD callback in SAP context */
	sap_ctx->pfnSapEventCallback = pSapEventCallback;

	/* Handle event */
	qdf_status = sap_fsm(sap_ctx, &sap_event);
fail:
	if (QDF_IS_STATUS_ERROR(qdf_status))
		sap_free_roam_profile(&sap_ctx->csr_roamProfile);

	return qdf_status;
} /* wlansap_start_bss */

QDF_STATUS wlansap_set_mac_acl(struct sap_context *sap_ctx,
			       tsap_config_t *config)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "wlansap_set_mac_acl");

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}
	/* Copy MAC filtering settings to sap context */
	sap_ctx->eSapMacAddrAclMode = config->SapMacaddr_acl;

	if (eSAP_DENY_UNLESS_ACCEPTED == sap_ctx->eSapMacAddrAclMode) {
		qdf_mem_copy(sap_ctx->acceptMacList,
			     config->accept_mac,
			     sizeof(config->accept_mac));
		sap_ctx->nAcceptMac = config->num_accept_mac;
		sap_sort_mac_list(sap_ctx->acceptMacList,
			       sap_ctx->nAcceptMac);
	} else if (eSAP_ACCEPT_UNLESS_DENIED == sap_ctx->eSapMacAddrAclMode) {
		qdf_mem_copy(sap_ctx->denyMacList, config->deny_mac,
			     sizeof(config->deny_mac));
		sap_ctx->nDenyMac = config->num_deny_mac;
		sap_sort_mac_list(sap_ctx->denyMacList, sap_ctx->nDenyMac);
	}

	return qdf_status;
} /* wlansap_set_mac_acl */

QDF_STATUS wlansap_stop_bss(struct sap_context *sap_ctx)
{
	tWLAN_SAPEvent sap_event;        /* State machine event */
	QDF_STATUS qdf_status;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	/* Fill in the event structure for FSM */
	sap_event.event = eSAP_HDD_STOP_INFRA_BSS;
	sap_event.params = 0;

	/* Handle event */
	qdf_status = sap_fsm(sap_ctx, &sap_event);

	return qdf_status;
}

/* This routine will set the mode of operation for ACL dynamically*/
QDF_STATUS wlansap_set_acl_mode(struct sap_context *sap_ctx,
				eSapMacAddrACL mode)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	sap_ctx->eSapMacAddrAclMode = mode;
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_get_acl_mode(struct sap_context *sap_ctx,
				eSapMacAddrACL *mode)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	*mode = sap_ctx->eSapMacAddrAclMode;
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_get_acl_accept_list(struct sap_context *sap_ctx,
				       struct qdf_mac_addr *pAcceptList,
				       uint8_t *nAcceptList)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	memcpy(pAcceptList, sap_ctx->acceptMacList,
	       (sap_ctx->nAcceptMac * QDF_MAC_ADDR_SIZE));
	*nAcceptList = sap_ctx->nAcceptMac;
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_get_acl_deny_list(struct sap_context *sap_ctx,
				     struct qdf_mac_addr *pDenyList,
				     uint8_t *nDenyList)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer from p_cds_gctx", __func__);
		return QDF_STATUS_E_FAULT;
	}

	memcpy(pDenyList, sap_ctx->denyMacList,
	       (sap_ctx->nDenyMac * QDF_MAC_ADDR_SIZE));
	*nDenyList = sap_ctx->nDenyMac;
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_clear_acl(struct sap_context *sap_ctx)
{
	uint8_t i;

	if (!sap_ctx) {
		return QDF_STATUS_E_RESOURCES;
	}

	for (i = 0; i < (sap_ctx->nDenyMac - 1); i++) {
		qdf_mem_zero((sap_ctx->denyMacList + i)->bytes,
			     QDF_MAC_ADDR_SIZE);
	}

	sap_print_acl(sap_ctx->denyMacList, sap_ctx->nDenyMac);
	sap_ctx->nDenyMac = 0;

	for (i = 0; i < (sap_ctx->nAcceptMac - 1); i++) {
		qdf_mem_zero((sap_ctx->acceptMacList + i)->bytes,
			     QDF_MAC_ADDR_SIZE);
	}

	sap_print_acl(sap_ctx->acceptMacList, sap_ctx->nAcceptMac);
	sap_ctx->nAcceptMac = 0;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_modify_acl(struct sap_context *sap_ctx,
			      uint8_t *peer_sta_mac,
			      eSapACLType list_type, eSapACLCmdType cmd)
{
	bool sta_white_list = false, sta_black_list = false;
	uint8_t staWLIndex, staBLIndex;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP Context", __func__);
		return QDF_STATUS_E_FAULT;
	}
	if (qdf_mem_cmp(sap_ctx->bssid.bytes, peer_sta_mac,
			QDF_MAC_ADDR_SIZE) == 0) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "requested peer mac is" MAC_ADDRESS_STR
			  "our own SAP BSSID. Do not blacklist or whitelist this BSSID",
			  MAC_ADDR_ARRAY(peer_sta_mac));
		return QDF_STATUS_E_FAULT;
	}
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_LOW,
		  "Modify ACL entered\n" "Before modification of ACL\n"
		  "size of accept and deny lists %d %d", sap_ctx->nAcceptMac,
		  sap_ctx->nDenyMac);
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "*** WHITE LIST ***");
	sap_print_acl(sap_ctx->acceptMacList, sap_ctx->nAcceptMac);
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "*** BLACK LIST ***");
	sap_print_acl(sap_ctx->denyMacList, sap_ctx->nDenyMac);

	/* the expectation is a mac addr will not be in both the lists
	 * at the same time. It is the responsiblity of userspace to
	 * ensure this
	 */
	sta_white_list =
		sap_search_mac_list(sap_ctx->acceptMacList, sap_ctx->nAcceptMac,
				 peer_sta_mac, &staWLIndex);
	sta_black_list =
		sap_search_mac_list(sap_ctx->denyMacList, sap_ctx->nDenyMac,
				 peer_sta_mac, &staBLIndex);

	if (sta_white_list && sta_black_list) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "Peer mac " MAC_ADDRESS_STR
			  " found in white and black lists."
			  "Initial lists passed incorrect. Cannot execute this command.",
			  MAC_ADDR_ARRAY(peer_sta_mac));
		return QDF_STATUS_E_FAILURE;

	}
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_LOW,
		  "cmd %d", cmd);

	switch (list_type) {
	case eSAP_WHITE_LIST:
		if (cmd == ADD_STA_TO_ACL) {
			/* error check */
			/* if list is already at max, return failure */
			if (sap_ctx->nAcceptMac == MAX_ACL_MAC_ADDRESS) {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_ERROR,
					  "White list is already maxed out. Cannot accept "
					  MAC_ADDRESS_STR,
					  MAC_ADDR_ARRAY(peer_sta_mac));
				return QDF_STATUS_E_FAILURE;
			}
			if (sta_white_list) {
				/* Do nothing if already present in white list. Just print a warning */
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_WARN,
					  "MAC address already present in white list "
					  MAC_ADDRESS_STR,
					  MAC_ADDR_ARRAY(peer_sta_mac));
				return QDF_STATUS_SUCCESS;
			}
			if (sta_black_list) {
				/* remove it from black list before adding to the white list */
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_WARN,
					  "STA present in black list so first remove from it");
				sap_remove_mac_from_acl(sap_ctx->denyMacList,
						    &sap_ctx->nDenyMac,
						    staBLIndex);
			}
			QDF_TRACE(QDF_MODULE_ID_SAP,
				  QDF_TRACE_LEVEL_INFO,
				  "... Now add to the white list");
			sap_add_mac_to_acl(sap_ctx->acceptMacList,
					       &sap_ctx->nAcceptMac,
			       peer_sta_mac);
				QDF_TRACE(QDF_MODULE_ID_SAP,
				  QDF_TRACE_LEVEL_INFO_LOW,
				  "size of accept and deny lists %d %d",
				  sap_ctx->nAcceptMac,
				  sap_ctx->nDenyMac);
		} else if (cmd == DELETE_STA_FROM_ACL) {
			if (sta_white_list) {

				struct csr_del_sta_params delStaParams;

				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_INFO,
					  "Delete from white list");
				sap_remove_mac_from_acl(sap_ctx->acceptMacList,
						    &sap_ctx->nAcceptMac,
						    staWLIndex);
				/* If a client is deleted from white list and it is connected, send deauth */
				wlansap_populate_del_sta_params(peer_sta_mac,
					eCsrForcedDeauthSta,
					(SIR_MAC_MGMT_DEAUTH >> 4),
					&delStaParams);
				wlansap_deauth_sta(sap_ctx, &delStaParams);
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_INFO_LOW,
					  "size of accept and deny lists %d %d",
					  sap_ctx->nAcceptMac,
					  sap_ctx->nDenyMac);
			} else {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_WARN,
					  "MAC address to be deleted is not present in the white list "
					  MAC_ADDRESS_STR,
					  MAC_ADDR_ARRAY(peer_sta_mac));
				return QDF_STATUS_E_FAILURE;
			}
		} else {
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				  "Invalid cmd type passed");
			return QDF_STATUS_E_FAILURE;
		}
		break;

	case eSAP_BLACK_LIST:

		if (cmd == ADD_STA_TO_ACL) {
			struct csr_del_sta_params delStaParams;
			/* error check */
			/* if list is already at max, return failure */
			if (sap_ctx->nDenyMac == MAX_ACL_MAC_ADDRESS) {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_ERROR,
					  "Black list is already maxed out. Cannot accept "
					  MAC_ADDRESS_STR,
					  MAC_ADDR_ARRAY(peer_sta_mac));
				return QDF_STATUS_E_FAILURE;
			}
			if (sta_black_list) {
				/* Do nothing if already present in white list */
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_WARN,
					  "MAC address already present in black list "
					  MAC_ADDRESS_STR,
					  MAC_ADDR_ARRAY(peer_sta_mac));
				return QDF_STATUS_SUCCESS;
			}
			if (sta_white_list) {
				/* remove it from white list before adding to the black list */
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_WARN,
					  "Present in white list so first remove from it");
				sap_remove_mac_from_acl(sap_ctx->acceptMacList,
						    &sap_ctx->nAcceptMac,
						    staWLIndex);
			}
			/* If we are adding a client to the black list; if its connected, send deauth */
			wlansap_populate_del_sta_params(peer_sta_mac,
				eCsrForcedDeauthSta,
				(SIR_MAC_MGMT_DEAUTH >> 4),
				&delStaParams);
			wlansap_deauth_sta(sap_ctx, &delStaParams);
			QDF_TRACE(QDF_MODULE_ID_SAP,
				  QDF_TRACE_LEVEL_INFO,
				  "... Now add to black list");
			sap_add_mac_to_acl(sap_ctx->denyMacList,
				       &sap_ctx->nDenyMac, peer_sta_mac);
			QDF_TRACE(QDF_MODULE_ID_SAP,
				  QDF_TRACE_LEVEL_INFO_LOW,
				  "size of accept and deny lists %d %d",
				  sap_ctx->nAcceptMac,
				  sap_ctx->nDenyMac);
		} else if (cmd == DELETE_STA_FROM_ACL) {
			if (sta_black_list) {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_INFO,
					  "Delete from black list");
				sap_remove_mac_from_acl(sap_ctx->denyMacList,
						    &sap_ctx->nDenyMac,
						    staBLIndex);
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_INFO_LOW,
					  "no accept and deny mac %d %d",
					  sap_ctx->nAcceptMac,
					  sap_ctx->nDenyMac);
			} else {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_WARN,
					  "MAC address to be deleted is not present in the black list "
					  MAC_ADDRESS_STR,
					  MAC_ADDR_ARRAY(peer_sta_mac));
				return QDF_STATUS_E_FAILURE;
			}
		} else {
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				  "Invalid cmd type passed");
			return QDF_STATUS_E_FAILURE;
		}
		break;

	default:
	{
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "Invalid list type passed %d", list_type);
		return QDF_STATUS_E_FAILURE;
	}
	}
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_LOW,
		  "After modification of ACL");
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "*** WHITE LIST ***");
	sap_print_acl(sap_ctx->acceptMacList, sap_ctx->nAcceptMac);
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "*** BLACK LIST ***");
	sap_print_acl(sap_ctx->denyMacList, sap_ctx->nDenyMac);
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_disassoc_sta(struct sap_context *sap_ctx,
				struct csr_del_sta_params *params)
{
	struct mac_context *mac;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	return sme_roam_disconnect_sta(MAC_HANDLE(mac), sap_ctx->sessionId,
				       params);
}

QDF_STATUS wlansap_deauth_sta(struct sap_context *sap_ctx,
			      struct csr_del_sta_params *params)
{
	struct mac_context *mac;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	return sme_roam_deauth_sta(MAC_HANDLE(mac), sap_ctx->sessionId,
				   params);
}

/**
 * wlansap_update_csa_channel_params() - function to populate channel width and
 *                                        bonding modes.
 * @sap_context: sap adapter context
 * @channel: target channel
 *
 * Return: The QDF_STATUS code associated with performing the operation
 */
static QDF_STATUS
wlansap_update_csa_channel_params(struct sap_context *sap_context,
				  uint32_t channel)
{
	struct mac_context *mac_ctx;
	uint8_t bw;

	mac_ctx = sap_get_mac_context();
	if (!mac_ctx) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	if (channel <= CHAN_ENUM_14) {
		/*
		 * currently OBSS scan is done in hostapd, so to avoid
		 * SAP coming up in HT40 on channel switch we are
		 * disabling channel bonding in 2.4Ghz.
		 */
		mac_ctx->sap.SapDfsInfo.new_chanWidth = 0;

	} else {
		if (sap_context->csr_roamProfile.phyMode ==
		    eCSR_DOT11_MODE_11ac ||
		    sap_context->csr_roamProfile.phyMode ==
		    eCSR_DOT11_MODE_11ac_ONLY)
			bw = BW80;
		else
			bw = BW40_HIGH_PRIMARY;

		for (; bw >= BW20; bw--) {
			uint16_t op_class;

			op_class = wlan_reg_dmn_get_opclass_from_channel(
					mac_ctx->scan.countryCodeCurrent,
					channel, bw);
			/*
			 * Do not continue if bw is 20. This mean channel is not
			 * found and thus set BW20 for the channel.
			 */
			if (!op_class && bw > BW20)
				continue;

			if (bw == BW80) {
				mac_ctx->sap.SapDfsInfo.new_chanWidth =
					CH_WIDTH_80MHZ;
			} else if (bw == BW40_HIGH_PRIMARY) {
				mac_ctx->sap.SapDfsInfo.new_chanWidth =
					CH_WIDTH_40MHZ;
			} else if (bw == BW40_LOW_PRIMARY) {
				mac_ctx->sap.SapDfsInfo.new_chanWidth =
				   CH_WIDTH_40MHZ;
			} else {
				mac_ctx->sap.SapDfsInfo.new_chanWidth =
				   CH_WIDTH_20MHZ;
			}
			break;
		}

	}

	return QDF_STATUS_SUCCESS;
}

/**
 * sap_start_csa_restart() - send csa start event
 * @mac_ctx: mac ctx
 * @sap_ctx: SAP context
 *
 * Return: QDF_STATUS
 */
#ifdef CONFIG_VDEV_SM
static inline void sap_start_csa_restart(struct mac_context *mac,
					 struct sap_context *sap_ctx)
{
	sme_csa_restart(mac, sap_ctx->sessionId);
}
#else
static void sap_start_csa_restart(struct mac_context *mac,
				  struct sap_context *sap_ctx)
{
	tWLAN_SAPEvent sap_event;

	/*
	 * Post the eSAP_CHANNEL_SWITCH_ANNOUNCEMENT_START
	 * to SAP state machine to process the channel
	 * request with CSA IE set in the beacons.
	 */
	sap_event.event =
		eSAP_CHANNEL_SWITCH_ANNOUNCEMENT_START;
	sap_event.params = 0;
	sap_event.u1 = 0;
	sap_event.u2 = 0;

	sap_fsm(sap_ctx, &sap_event);
}
#endif
/**
 * wlansap_set_channel_change_with_csa() - Set channel change with CSA
 * @sap_ctx: Pointer to SAP context
 * @targetChannel: Target channel
 * @target_bw: Target bandwidth
 * @strict: if true switch to the requested channel always,
 *        SCC/MCC check will be ignored,
 *        fail otherwise
 *
 * This api function does a channel change to the target channel specified.
 * CSA IE is included in the beacons before doing a channel change.
 *
 * Return: QDF_STATUS
 */
QDF_STATUS wlansap_set_channel_change_with_csa(struct sap_context *sap_ctx,
					       uint32_t targetChannel,
					       enum phy_ch_width target_bw,
					       bool strict)
{
	struct mac_context *mac;
	mac_handle_t mac_handle;
	bool valid;
	QDF_STATUS status;
	bool sta_sap_scc_on_dfs_chan;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);

		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}
	mac_handle = MAC_HANDLE(mac);

	if (strict && !policy_mgr_is_safe_channel(mac->psoc, targetChannel)) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%u is unsafe channel", targetChannel);
		return QDF_STATUS_E_FAULT;
	}
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
		"%s: sap chan:%d target:%d conn on 5GHz:%d",
		__func__, sap_ctx->channel, targetChannel,
		policy_mgr_is_any_mode_active_on_band_along_with_session(
			mac->psoc, sap_ctx->sessionId, POLICY_MGR_BAND_5));

	sta_sap_scc_on_dfs_chan =
		policy_mgr_is_sta_sap_scc_allowed_on_dfs_chan(mac->psoc);
	/*
	 * Now, validate if the passed channel is valid in the
	 * current regulatory domain.
	 */
	if (sap_ctx->channel != targetChannel &&
		((wlan_reg_get_channel_state(mac->pdev, targetChannel) ==
			CHANNEL_STATE_ENABLE) ||
		(wlan_reg_get_channel_state(mac->pdev, targetChannel) ==
			CHANNEL_STATE_DFS &&
		(!policy_mgr_is_any_mode_active_on_band_along_with_session(
			mac->psoc, sap_ctx->sessionId,
			POLICY_MGR_BAND_5) ||
			sta_sap_scc_on_dfs_chan)))) {
		/*
		 * validate target channel switch w.r.t various concurrency
		 * rules set.
		 */
		if (!strict) {
			valid = wlan_sap_validate_channel_switch(mac_handle,
								 targetChannel,
								 sap_ctx);
			if (!valid) {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_ERROR,
					  FL("Channel switch to %u is not allowed due to concurrent channel interference"),
					  targetChannel);
				return QDF_STATUS_E_FAULT;
			}
		}
		/*
		 * Post a CSA IE request to SAP state machine with
		 * target channel information and also CSA IE required
		 * flag set in sap_ctx only, if SAP is in SAP_STARTED
		 * state.
		 */
		if (sap_ctx->fsm_state == SAP_STARTED) {
			status = wlansap_update_csa_channel_params(sap_ctx,
					targetChannel);
			if (status != QDF_STATUS_SUCCESS)
				return status;

			status = policy_mgr_set_hw_mode_before_channel_switch(
						mac->psoc,
						sap_ctx->sessionId,
						targetChannel);
			/*
			 * If status is QDF_STATUS_E_FAILURE that mean HW mode
			 * change was required but set HW mode change failed.
			 */
			if (status == QDF_STATUS_E_FAILURE) {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					  QDF_TRACE_LEVEL_ERROR,
					  FL("HW change required but failed to set hw mode"));
				return status;
			}

			/*
			 * Copy the requested target channel
			 * to sap context.
			 */
			mac->sap.SapDfsInfo.target_channel = targetChannel;
			mac->sap.SapDfsInfo.new_ch_params.ch_width =
				mac->sap.SapDfsInfo.new_chanWidth;

			/* By this time, the best bandwidth is calculated for
			 * the given target channel. Now, if there was a
			 * request from user to move to a selected bandwidth,
			 * we can see if it can be honored.
			 *
			 * Ex1: BW80 was selected for the target channel and
			 * user wants BW40, it can be allowed
			 * Ex2: BW40 was selected for the target channel and
			 * user wants BW80, it cannot be allowed for the given
			 * target channel.
			 *
			 * So, the MIN of the selected channel bandwidth and
			 * user input is used for the bandwidth
			 */
			if (target_bw != CH_WIDTH_MAX) {
				QDF_TRACE(QDF_MODULE_ID_SAP,
					QDF_TRACE_LEVEL_INFO,
					"%s: target bw:%d new width:%d",
					__func__, target_bw,
					mac->sap.SapDfsInfo.
					new_ch_params.ch_width);
				mac->sap.SapDfsInfo.new_ch_params.ch_width =
					mac->sap.SapDfsInfo.new_chanWidth =
					QDF_MIN(mac->sap.SapDfsInfo.
							new_ch_params.ch_width,
							target_bw);
			}
			wlan_reg_set_channel_params(mac->pdev, targetChannel,
				0, &mac->sap.SapDfsInfo.new_ch_params);
			/*
			 * Set the CSA IE required flag.
			 */
			mac->sap.SapDfsInfo.csaIERequired = true;

			/*
			 * Set the radar found status to allow the channel
			 * change to happen same as in the case of a radar
			 * detection. Since, this will allow SAP to be in
			 * correct state and also resume the netif queues
			 * that were suspended in HDD before the channel
			 * request was issued.
			 */
			mac->sap.SapDfsInfo.sap_radar_found_status = true;
			mac->sap.SapDfsInfo.cac_state =
					eSAP_DFS_DO_NOT_SKIP_CAC;
			sap_cac_reset_notify(mac_handle);
			sap_start_csa_restart(mac, sap_ctx);
		} else {
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				  "%s: Failed to request Channel Change, since SAP is not in SAP_STARTED state",
				  __func__);
			return QDF_STATUS_E_FAULT;
		}

	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Channel = %d is not valid in the current"
			  "regulatory domain", __func__, targetChannel);

		return QDF_STATUS_E_FAULT;
	}

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
		  "%s: Posted eSAP_CHANNEL_SWITCH_ANNOUNCEMENT_START successfully to sap_fsm for Channel = %d",
		  __func__, targetChannel);

	return QDF_STATUS_SUCCESS;
}

#ifdef CRYPTO_SET_KEY_CONVERGED
static QDF_STATUS wlan_sap_set_key_helper(struct sap_context *sap_ctx,
					  tCsrRoamSetKey *set_key_info)
{
	struct wlan_crypto_key *crypto_key;

	crypto_key = wlan_crypto_get_key(sap_ctx->vdev, 0);
	if (!crypto_key) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				"Crypto KEY is NULL");
		return QDF_STATUS_E_FAILURE;
	}

	return ucfg_crypto_set_key_req(sap_ctx->vdev, crypto_key,
				       WLAN_CRYPTO_KEY_TYPE_UNICAST);
}
#else
static QDF_STATUS wlan_sap_set_key_helper(struct sap_context *sap_ctx,
					  tCsrRoamSetKey *set_key_info)
{
	uint32_t roam_id = INVALID_ROAM_ID;
	struct mac_context *mac;

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	return sme_roam_set_key(MAC_HANDLE(mac), sap_ctx->sessionId,
				set_key_info, &roam_id);
}
#endif

QDF_STATUS wlansap_set_key_sta(struct sap_context *sap_ctx,
			       tCsrRoamSetKey *set_key_info)
{
	QDF_STATUS qdf_status;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer",
			  __func__);
		return QDF_STATUS_E_FAULT;
	}

	qdf_status = wlan_sap_set_key_helper(sap_ctx, set_key_info);
	if (qdf_status != QDF_STATUS_SUCCESS)
		qdf_status = QDF_STATUS_E_FAULT;

	return qdf_status;
}

QDF_STATUS wlan_sap_getstation_ie_information(struct sap_context *sap_ctx,
					      uint32_t *len, uint8_t *buf)
{
	QDF_STATUS qdf_status = QDF_STATUS_E_FAILURE;
	uint32_t ie_len = 0;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP pointer"));
		return QDF_STATUS_E_FAULT;
	}

	if (len) {
		ie_len = *len;
		*len = sap_ctx->nStaWPARSnReqIeLength;
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
			FL("WPAIE len : %x"), *len);
		if ((buf) && (ie_len >= sap_ctx->nStaWPARSnReqIeLength)) {
			qdf_mem_copy(buf,
				sap_ctx->pStaWpaRsnReqIE,
				sap_ctx->nStaWPARSnReqIeLength);
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
				FL("WPAIE: %02x:%02x:%02x:%02x:%02x:%02x"),
				buf[0], buf[1], buf[2], buf[3], buf[4],
				buf[5]);
			qdf_status = QDF_STATUS_SUCCESS;
		}
	}
	return qdf_status;
}

QDF_STATUS wlan_sap_update_next_channel(struct sap_context *sap_ctx,
					uint8_t channel,
					enum phy_ch_width chan_bw)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	sap_ctx->dfs_vendor_channel = channel;
	sap_ctx->dfs_vendor_chan_bw = chan_bw;

	return QDF_STATUS_SUCCESS;
}

#ifdef FEATURE_SAP_COND_CHAN_SWITCH
QDF_STATUS wlan_sap_set_pre_cac_status(struct sap_context *sap_ctx,
				       bool status)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	sap_ctx->is_pre_cac_on = status;
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
		"%s: is_pre_cac_on:%d", __func__, sap_ctx->is_pre_cac_on);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlan_sap_set_chan_before_pre_cac(struct sap_context *sap_ctx,
					    uint8_t chan_before_pre_cac)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	sap_ctx->chan_before_pre_cac = chan_before_pre_cac;
	return QDF_STATUS_SUCCESS;
}
#endif /* FEATURE_SAP_COND_CHAN_SWITCH */

QDF_STATUS wlan_sap_set_pre_cac_complete_status(struct sap_context *sap_ctx,
						bool status)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	sap_ctx->pre_cac_complete = status;

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
			"%s: pre cac complete status:%d session:%d",
			__func__, status, sap_ctx->sessionId);

	return QDF_STATUS_SUCCESS;
}

bool wlan_sap_is_pre_cac_context(struct sap_context *context)
{
	return context && context->is_pre_cac_on;
}

/**
 * wlan_sap_is_pre_cac_active() - Checks if pre cac in in progress
 * @handle: Global MAC handle
 *
 * Checks if pre cac is in progress in any of the SAP contexts
 *
 * Return: True is pre cac is active, false otherwise
 */
bool wlan_sap_is_pre_cac_active(mac_handle_t handle)
{
	struct mac_context *mac = NULL;
	struct sap_ctx_list *ctx_list;
	int i;

	mac = MAC_CONTEXT(handle);
	if (!mac) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			"%s: Invalid mac context", __func__);
		return false;
	}

	ctx_list = mac->sap.sapCtxList;
	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		if (wlan_sap_is_pre_cac_context(ctx_list[i].sap_context))
			return true;
	}

	return false;
}

/**
 * wlan_sap_get_pre_cac_vdev_id() - Get vdev id of the pre cac interface
 * @handle: Global handle
 * @vdev_id: vdev id
 *
 * Fetches the vdev id of the pre cac interface
 *
 * Return: QDF_STATUS
 */
QDF_STATUS wlan_sap_get_pre_cac_vdev_id(mac_handle_t handle, uint8_t *vdev_id)
{
	struct mac_context *mac = NULL;
	uint8_t i;

	mac = MAC_CONTEXT(handle);
	if (!mac) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			"%s: Invalid mac context", __func__);
		return QDF_STATUS_E_FAULT;
	}

	for (i = 0; i < SAP_MAX_NUM_SESSION; i++) {
		struct sap_context *context =
			mac->sap.sapCtxList[i].sap_context;
		if (context && context->is_pre_cac_on) {
			*vdev_id = i;
			return QDF_STATUS_SUCCESS;
		}
	}
	return QDF_STATUS_E_FAILURE;
}

void wlansap_get_sec_channel(uint8_t sec_ch_offset,
			     uint8_t op_channel,
			     uint8_t *sec_channel)
{
	switch (sec_ch_offset) {
	case LOW_PRIMARY_CH:
		*sec_channel = op_channel + 4;
		break;
	case HIGH_PRIMARY_CH:
		*sec_channel = op_channel - 4;
		break;
	default:
		*sec_channel = 0;
	}
	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
		  "%s: sec channel offset %d, sec channel %d",
		  __func__, sec_ch_offset, *sec_channel);
}

QDF_STATUS wlansap_channel_change_request(struct sap_context *sap_ctx,
					  uint8_t target_channel)
{
	QDF_STATUS status = QDF_STATUS_E_FAILURE;
	struct mac_context *mac_ctx;
	eCsrPhyMode phy_mode;
	struct ch_params *ch_params;

	if (!target_channel) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: channel 0 requested", __func__);
		return QDF_STATUS_E_FAULT;
	}

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac_ctx = sap_get_mac_context();
	if (!mac_ctx) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	phy_mode = sap_ctx->csr_roamProfile.phyMode;

	/* Update phy_mode if the target channel is in the other band */
	if (WLAN_CHAN_IS_5GHZ(target_channel) &&
	    ((phy_mode == eCSR_DOT11_MODE_11g) ||
	    (phy_mode == eCSR_DOT11_MODE_11g_ONLY)))
		phy_mode = eCSR_DOT11_MODE_11a;
	else if (WLAN_CHAN_IS_2GHZ(target_channel) &&
		 (phy_mode == eCSR_DOT11_MODE_11a))
		phy_mode = eCSR_DOT11_MODE_11g;

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
		  "%s: phy_mode: %d, target_channel: %d new phy_mode: %d",
		  __func__, sap_ctx->csr_roamProfile.phyMode,
		  target_channel, phy_mode);
	sap_ctx->csr_roamProfile.phyMode = phy_mode;

	if (sap_ctx->csr_roamProfile.ChannelInfo.numOfChannels == 0 ||
	    !sap_ctx->csr_roamProfile.ChannelInfo.ChannelList) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid channel list"));
		return QDF_STATUS_E_FAULT;
	}
	sap_ctx->csr_roamProfile.ChannelInfo.ChannelList[0] = target_channel;
	/*
	 * We are getting channel bonding mode from sapDfsInfor structure
	 * because we've implemented channel width fallback mechanism for DFS
	 * which will result in channel width changing dynamically.
	 */
	ch_params = &mac_ctx->sap.SapDfsInfo.new_ch_params;
	wlan_reg_set_channel_params(mac_ctx->pdev, target_channel,
			0, ch_params);
	sap_ctx->ch_params = *ch_params;
	/* Update the channel as this will be used to
	 * send event to supplicant
	 */
	sap_ctx->channel = target_channel;
	wlansap_get_sec_channel(ch_params->sec_ch_offset, target_channel,
				(uint8_t *)(&sap_ctx->secondary_ch));
	sap_ctx->csr_roamProfile.ch_params.ch_width = ch_params->ch_width;
	sap_ctx->csr_roamProfile.ch_params.sec_ch_offset =
						ch_params->sec_ch_offset;
	sap_ctx->csr_roamProfile.ch_params.center_freq_seg0 =
						ch_params->center_freq_seg0;
	sap_ctx->csr_roamProfile.ch_params.center_freq_seg1 =
						ch_params->center_freq_seg1;
	sap_dfs_set_current_channel(sap_ctx);

	status = sme_roam_channel_change_req(MAC_HANDLE(mac_ctx),
					     sap_ctx->bssid,
					     ch_params,
					     &sap_ctx->csr_roamProfile);

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
		"%s: chan:%d phy_mode %d width:%d offset:%d seg0:%d seg1:%d",
		__func__, sap_ctx->channel, phy_mode, ch_params->ch_width,
		ch_params->sec_ch_offset, ch_params->center_freq_seg0,
		ch_params->center_freq_seg1);

	if (QDF_IS_STATUS_SUCCESS(status))
		sap_signal_hdd_event(sap_ctx, NULL,
				     eSAP_CHANNEL_CHANGE_EVENT,
				     (void *) eSAP_STATUS_SUCCESS);

	return status;
}

QDF_STATUS wlansap_start_beacon_req(struct sap_context *sap_ctx)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	uint8_t dfs_cac_wait_status;
	struct mac_context *mac;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	/* No Radar was found during CAC WAIT, So start Beaconing */
	if (mac->sap.SapDfsInfo.sap_radar_found_status == false) {
		/* CAC Wait done without any Radar Detection */
		dfs_cac_wait_status = true;
		sap_ctx->pre_cac_complete = false;
		status = sme_roam_start_beacon_req(MAC_HANDLE(mac),
						   sap_ctx->bssid,
						   dfs_cac_wait_status);
	}

	return status;
}

QDF_STATUS wlansap_dfs_send_csa_ie_request(struct sap_context *sap_ctx)
{
	struct mac_context *mac;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	mac->sap.SapDfsInfo.new_ch_params.ch_width =
				mac->sap.SapDfsInfo.new_chanWidth;
	wlan_reg_set_channel_params(mac->pdev,
			mac->sap.SapDfsInfo.target_channel,
			0, &mac->sap.SapDfsInfo.new_ch_params);

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
			"%s: chan:%d req:%d width:%d off:%d",
			__func__, mac->sap.SapDfsInfo.target_channel,
			mac->sap.SapDfsInfo.csaIERequired,
			mac->sap.SapDfsInfo.new_ch_params.ch_width,
			mac->sap.SapDfsInfo.new_ch_params.sec_ch_offset);

	return sme_roam_csa_ie_request(MAC_HANDLE(mac),
				       sap_ctx->bssid,
				       mac->sap.SapDfsInfo.target_channel,
				       mac->sap.SapDfsInfo.csaIERequired,
				       &mac->sap.SapDfsInfo.new_ch_params);
}

QDF_STATUS wlansap_get_dfs_ignore_cac(mac_handle_t mac_handle,
				      uint8_t *ignore_cac)
{
	struct mac_context *mac = NULL;

	if (mac_handle) {
		mac = MAC_CONTEXT(mac_handle);
	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid mac_handle pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	*ignore_cac = mac->sap.SapDfsInfo.ignore_cac;
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_set_dfs_ignore_cac(mac_handle_t mac_handle,
				      uint8_t ignore_cac)
{
	struct mac_context *mac = NULL;

	if (mac_handle) {
		mac = MAC_CONTEXT(mac_handle);
	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid mac_handle pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac->sap.SapDfsInfo.ignore_cac = (ignore_cac >= true) ?
					  true : false;
	return QDF_STATUS_SUCCESS;
}

bool sap_is_auto_channel_select(struct sap_context *sapcontext)
{
	if (!sapcontext) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%s: Invalid SAP pointer", __func__);
		return false;
	}
	return sapcontext->channel == AUTO_CHANNEL_SELECT;
}

#ifdef FEATURE_AP_MCC_CH_AVOIDANCE
/**
 * wlan_sap_set_channel_avoidance() - sets sap mcc channel avoidance ini param
 * @mac_handle: Opaque handle to the global MAC context
 * @sap_channel_avoidance: ini parameter value
 *
 * sets sap mcc channel avoidance ini param, to be called in sap_start
 *
 * Return: success of failure of operation
 */
QDF_STATUS
wlan_sap_set_channel_avoidance(mac_handle_t mac_handle,
			       bool sap_channel_avoidance)
{
	struct mac_context *mac_ctx = NULL;

	if (mac_handle) {
		mac_ctx = MAC_CONTEXT(mac_handle);
	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP,
			  QDF_TRACE_LEVEL_ERROR,
			  FL("mac_handle or mac_ctx pointer NULL"));
		return QDF_STATUS_E_FAULT;
	}
	mac_ctx->sap.sap_channel_avoidance = sap_channel_avoidance;
	return QDF_STATUS_SUCCESS;
}
#endif /* FEATURE_AP_MCC_CH_AVOIDANCE */

/**
 * wlansap_set_dfs_preferred_channel_location() - set dfs preferred channel
 * @mac_handle: Opaque handle to the global MAC context
 * @dfs_Preferred_Channels_location :
 *       0 - Indicates No preferred channel location restrictions
 *       1 - Indicates SAP Indoor Channels operation only.
 *       2 - Indicates SAP Outdoor Channels operation only.
 *
 * This API is used to set sap preferred channels location
 * to resetrict the DFS random channel selection algorithm
 * either Indoor/Outdoor channels only.
 *
 * Return: The QDF_STATUS code associated with performing the operation
 *         QDF_STATUS_SUCCESS:  Success and error code otherwise.
 */
QDF_STATUS
wlansap_set_dfs_preferred_channel_location(mac_handle_t mac_handle,
					   uint8_t
					   dfs_Preferred_Channels_location)
{
	struct mac_context *mac = NULL;
	QDF_STATUS status;
	enum dfs_reg dfs_region;

	if (mac_handle) {
		mac = MAC_CONTEXT(mac_handle);
	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid mac_handle pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	wlan_reg_get_dfs_region(mac->pdev, &dfs_region);

	/*
	 * The Indoor/Outdoor only random channel selection
	 * restriction is currently enforeced only for
	 * JAPAN regulatory domain.
	 */
	if (DFS_MKK_REGION == dfs_region) {
		mac->sap.SapDfsInfo.sap_operating_chan_preferred_location =
			dfs_Preferred_Channels_location;
		QDF_TRACE(QDF_MODULE_ID_SAP,
			  QDF_TRACE_LEVEL_INFO_LOW,
			  FL
				  ("sapdfs:Set Preferred Operating Channel location=%d"),
			  mac->sap.SapDfsInfo.
			  sap_operating_chan_preferred_location);

		status = QDF_STATUS_SUCCESS;
	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  FL
				  ("sapdfs:NOT JAPAN REG, Invalid Set preferred chans location"));

		status = QDF_STATUS_E_FAULT;
	}

	return status;
}

QDF_STATUS wlansap_set_dfs_target_chnl(mac_handle_t mac_handle,
				       uint8_t target_channel)
{
	struct mac_context *mac = NULL;

	if (mac_handle) {
		mac = MAC_CONTEXT(mac_handle);
	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid mac_handle pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}
	if (target_channel > 0) {
		mac->sap.SapDfsInfo.user_provided_target_channel =
			target_channel;
	} else {
		mac->sap.SapDfsInfo.user_provided_target_channel = 0;
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
wlansap_update_sap_config_add_ie(tsap_config_t *config,
				 const uint8_t *pAdditionIEBuffer,
				 uint16_t additionIELength,
				 eUpdateIEsType updateType)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	uint8_t bufferValid = false;
	uint16_t bufferLength = 0;
	uint8_t *pBuffer = NULL;

	if (!config) {
		return QDF_STATUS_E_FAULT;
	}

	if ((pAdditionIEBuffer) && (additionIELength != 0)) {
		/* initialize the buffer pointer so that pe can copy */
		if (additionIELength > 0) {
			bufferLength = additionIELength;
			pBuffer = qdf_mem_malloc(bufferLength);
			if (!pBuffer)
				return QDF_STATUS_E_NOMEM;

			qdf_mem_copy(pBuffer, pAdditionIEBuffer, bufferLength);
			bufferValid = true;
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
				  FL("update_type: %d"), updateType);
			qdf_trace_hex_dump(QDF_MODULE_ID_SAP,
				QDF_TRACE_LEVEL_INFO, pBuffer, bufferLength);
		}
	}

	switch (updateType) {
	case eUPDATE_IE_PROBE_BCN:
		if (config->pProbeRespBcnIEsBuffer)
			qdf_mem_free(config->pProbeRespBcnIEsBuffer);
		if (bufferValid) {
			config->probeRespBcnIEsLen = bufferLength;
			config->pProbeRespBcnIEsBuffer = pBuffer;
		} else {
			config->probeRespBcnIEsLen = 0;
			config->pProbeRespBcnIEsBuffer = NULL;
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
				  FL("No Probe Resp beacone IE received in set beacon"));
		}
		break;
	case eUPDATE_IE_PROBE_RESP:
		if (config->pProbeRespIEsBuffer)
			qdf_mem_free(config->pProbeRespIEsBuffer);
		if (bufferValid) {
			config->probeRespIEsBufferLen = bufferLength;
			config->pProbeRespIEsBuffer = pBuffer;
		} else {
			config->probeRespIEsBufferLen = 0;
			config->pProbeRespIEsBuffer = NULL;
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
				  FL("No Probe Response IE received in set beacon"));
		}
		break;
	case eUPDATE_IE_ASSOC_RESP:
		if (config->pAssocRespIEsBuffer)
			qdf_mem_free(config->pAssocRespIEsBuffer);
		if (bufferValid) {
			config->assocRespIEsLen = bufferLength;
			config->pAssocRespIEsBuffer = pBuffer;
		} else {
			config->assocRespIEsLen = 0;
			config->pAssocRespIEsBuffer = NULL;
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
				  FL("No Assoc Response IE received in set beacon"));
		}
		break;
	default:
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO,
			  FL("No matching buffer type %d"), updateType);
		if (pBuffer)
			qdf_mem_free(pBuffer);
		break;
	}

	return status;
}

QDF_STATUS
wlansap_reset_sap_config_add_ie(tsap_config_t *config, eUpdateIEsType updateType)
{
	if (!config) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid Config pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	switch (updateType) {
	case eUPDATE_IE_ALL:    /*only used to reset */
	case eUPDATE_IE_PROBE_RESP:
		if (config->pProbeRespIEsBuffer) {
			qdf_mem_free(config->pProbeRespIEsBuffer);
			config->probeRespIEsBufferLen = 0;
			config->pProbeRespIEsBuffer = NULL;
		}
		if (eUPDATE_IE_ALL != updateType)
			break;

	case eUPDATE_IE_ASSOC_RESP:
		if (config->pAssocRespIEsBuffer) {
			qdf_mem_free(config->pAssocRespIEsBuffer);
			config->assocRespIEsLen = 0;
			config->pAssocRespIEsBuffer = NULL;
		}
		if (eUPDATE_IE_ALL != updateType)
			break;

	case eUPDATE_IE_PROBE_BCN:
		if (config->pProbeRespBcnIEsBuffer) {
			qdf_mem_free(config->pProbeRespBcnIEsBuffer);
			config->probeRespBcnIEsLen = 0;
			config->pProbeRespBcnIEsBuffer = NULL;
		}
		if (eUPDATE_IE_ALL != updateType)
			break;

	default:
		if (eUPDATE_IE_ALL != updateType)
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				  FL("Invalid buffer type %d"), updateType);
		break;
	}
	return QDF_STATUS_SUCCESS;
}

/*==========================================================================
   FUNCTION  wlansap_extend_to_acs_range

   DESCRIPTION Function extends give channel range to consider ACS chan bonding

   DEPENDENCIES PARAMETERS

   IN /OUT
   *startChannelNum : ACS extend start ch
   *endChannelNum   : ACS extended End ch
   *bandStartChannel: Band start ch
   *bandEndChannel  : Band end ch

   RETURN VALUE NONE

   SIDE EFFECTS
   ============================================================================*/
void wlansap_extend_to_acs_range(mac_handle_t mac_handle,
				 uint8_t *startChannelNum,
				 uint8_t *endChannelNum,
				 uint8_t *bandStartChannel,
				 uint8_t *bandEndChannel)
{
#define ACS_WLAN_20M_CH_INC 4
#define ACS_2G_EXTEND ACS_WLAN_20M_CH_INC
#define ACS_5G_EXTEND (ACS_WLAN_20M_CH_INC * 3)

	uint8_t tmp_startChannelNum = 0, tmp_endChannelNum = 0;
	struct mac_context *mac_ctx;

	mac_ctx = MAC_CONTEXT(mac_handle);
	if (!mac_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%s: Invalid mac_ctx", __func__);
		return;
	}
	if (*startChannelNum <= 14 && *endChannelNum <= 14) {
		*bandStartChannel = CHAN_ENUM_1;
		*bandEndChannel = CHAN_ENUM_14;
		tmp_startChannelNum = *startChannelNum > 5 ?
				   (*startChannelNum - ACS_2G_EXTEND) : 1;
		tmp_endChannelNum = (*endChannelNum + ACS_2G_EXTEND) <= 14 ?
				 (*endChannelNum + ACS_2G_EXTEND) : 14;
	} else if (*startChannelNum >= 36 && *endChannelNum >= 36) {
		*bandStartChannel = CHAN_ENUM_36;
		*bandEndChannel = CHAN_ENUM_173;
		tmp_startChannelNum = (*startChannelNum - ACS_5G_EXTEND) > 36 ?
				   (*startChannelNum - ACS_5G_EXTEND) : 36;
		tmp_endChannelNum = (*endChannelNum + ACS_5G_EXTEND) <=
				     WNI_CFG_CURRENT_CHANNEL_STAMAX ?
				     (*endChannelNum + ACS_5G_EXTEND) :
				     WNI_CFG_CURRENT_CHANNEL_STAMAX;
	} else {
		*bandStartChannel = CHAN_ENUM_1;
		*bandEndChannel = CHAN_ENUM_173;
		tmp_startChannelNum = *startChannelNum > 5 ?
			(*startChannelNum - ACS_2G_EXTEND) : 1;
		tmp_endChannelNum = (*endChannelNum + ACS_5G_EXTEND) <=
				     WNI_CFG_CURRENT_CHANNEL_STAMAX ?
				     (*endChannelNum + ACS_5G_EXTEND) :
				     WNI_CFG_CURRENT_CHANNEL_STAMAX;
	}

	/* Note if the ACS range include only DFS channels, do not cross range
	* Active scanning in adjacent non DFS channels results in transmission
	* spikes in DFS specturm channels which is due to emission spill.
	* Remove the active channels from extend ACS range for DFS only range
	*/
	if (wlan_reg_is_dfs_ch(mac_ctx->pdev, *startChannelNum)) {
		while (!wlan_reg_is_dfs_ch(mac_ctx->pdev,
					tmp_startChannelNum) &&
			tmp_startChannelNum < *startChannelNum)
			tmp_startChannelNum += ACS_WLAN_20M_CH_INC;

		*startChannelNum = tmp_startChannelNum;
	}
	if (wlan_reg_is_dfs_ch(mac_ctx->pdev, *endChannelNum)) {
		while (!wlan_reg_is_dfs_ch(mac_ctx->pdev,
					tmp_endChannelNum) &&
				 tmp_endChannelNum > *endChannelNum)
			tmp_endChannelNum -= ACS_WLAN_20M_CH_INC;

		*endChannelNum = tmp_endChannelNum;
	}
}

QDF_STATUS wlan_sap_set_vendor_acs(struct sap_context *sap_context,
				   bool is_vendor_acs)
{
	if (!sap_context) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}
	sap_context->vendor_acs_dfs_lte_enabled = is_vendor_acs;

	return QDF_STATUS_SUCCESS;
}

#ifdef DFS_COMPONENT_ENABLE
QDF_STATUS wlansap_set_dfs_nol(struct sap_context *sap_ctx,
			       eSapDfsNolType conf)
{
	struct mac_context *mac;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Invalid SAP pointer", __func__);
		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	if (conf == eSAP_DFS_NOL_CLEAR) {
		struct wlan_objmgr_pdev *pdev;

		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: clear the DFS NOL", __func__);

		pdev = mac->pdev;
		if (!pdev) {
			QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
				  "%s: null pdev", __func__);
			return QDF_STATUS_E_FAULT;
		}
		utils_dfs_clear_nol_channels(pdev);
	} else if (conf == eSAP_DFS_NOL_RANDOMIZE) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: Randomize the DFS NOL", __func__);

	} else {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  "%s: unsupport type %d", __func__, conf);
	}

	return QDF_STATUS_SUCCESS;
}
#endif

void wlansap_populate_del_sta_params(const uint8_t *mac,
				     uint16_t reason_code,
				     uint8_t subtype,
				     struct csr_del_sta_params *params)
{
	if (!mac)
		qdf_set_macaddr_broadcast(&params->peerMacAddr);
	else
		qdf_mem_copy(params->peerMacAddr.bytes, mac,
			     QDF_MAC_ADDR_SIZE);

	if (reason_code == 0)
		params->reason_code = eSIR_MAC_DEAUTH_LEAVING_BSS_REASON;
	else
		params->reason_code = reason_code;

	if (subtype == (SIR_MAC_MGMT_DEAUTH >> 4) ||
	    subtype == (SIR_MAC_MGMT_DISASSOC >> 4))
		params->subtype = subtype;
	else
		params->subtype = (SIR_MAC_MGMT_DEAUTH >> 4);

	QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_DEBUG,
		  FL("Delete STA with RC:%hu subtype:%hhu MAC::"
		     MAC_ADDRESS_STR),
		  params->reason_code, params->subtype,
		  MAC_ADDR_ARRAY(params->peerMacAddr.bytes));
}

QDF_STATUS wlansap_acs_chselect(struct sap_context *sap_context,
				tpWLAN_SAPEventCB pacs_event_callback,
				tsap_config_t *pconfig,
				void *pusr_context)
{
	QDF_STATUS qdf_status = QDF_STATUS_E_FAILURE;
	struct mac_context *mac;

	if (!sap_context) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"%s: Invalid SAP pointer", __func__);

		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	sap_context->acs_cfg = &pconfig->acs_cfg;
	sap_context->ch_width_orig = pconfig->acs_cfg.ch_width;
	sap_context->csr_roamProfile.phyMode = pconfig->acs_cfg.hw_mode;

	/*
	 * Now, configure the scan and ACS channel params
	 * to issue a scan request.
	 */
	wlansap_set_scan_acs_channel_params(pconfig, sap_context,
						pusr_context);

	/*
	 * Copy the HDD callback function to report the
	 * ACS result after scan in SAP context callback function.
	 */
	sap_context->pfnSapEventCallback = pacs_event_callback;
	/*
	 * init dfs channel nol
	 */
	sap_init_dfs_channel_nol_list(sap_context);

	/*
	 * Issue the scan request. This scan request is
	 * issued before the start BSS is done so
	 *
	 * 1. No need to pass the second parameter
	 * as the SAP state machine is not started yet
	 * and there is no need for any event posting.
	 *
	 * 2. Set third parameter to TRUE to indicate the
	 * channel selection function to register a
	 * different scan callback function to process
	 * the results pre start BSS.
	 */
	qdf_status = sap_channel_sel(sap_context);

	if (QDF_STATUS_E_ABORTED == qdf_status) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			"In %s,DFS not supported in the current operating mode",
			__func__);
		return QDF_STATUS_E_FAILURE;
	} else if (QDF_STATUS_E_CANCELED == qdf_status) {
		/*
		* ERROR is returned when either the SME scan request
		* failed or ACS is overridden due to other constrainst
		* So send selected channel to HDD
		*/
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Scan Req Failed/ACS Overridden"));
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Selected channel = %d"),
			sap_context->channel);

		return sap_signal_hdd_event(sap_context, NULL,
				eSAP_ACS_CHANNEL_SELECTED,
				(void *) eSAP_STATUS_SUCCESS);
	} else if (QDF_STATUS_SUCCESS == qdf_status) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_INFO_HIGH,
			FL("Successfully Issued a Pre Start Bss Scan Request"));
	}
	return qdf_status;
}

/**
 * wlan_sap_enable_phy_error_logs() - Enable DFS phy error logs
 * @mac_handle: Opaque handle to the global MAC context
 * @enable_log: value to set
 *
 * Since the frequency of DFS phy error is very high, enabling logs for them
 * all the times can cause crash and will also create lot of useless logs
 * causing difficulties in debugging other issue. This function will be called
 * from iwpriv cmd to enable such logs temporarily.
 *
 * Return: void
 */
void wlan_sap_enable_phy_error_logs(mac_handle_t mac_handle,
				    uint32_t enable_log)
{
	int error;

	struct mac_context *mac_ctx = MAC_CONTEXT(mac_handle);

	mac_ctx->sap.enable_dfs_phy_error_logs = !!enable_log;
	tgt_dfs_control(mac_ctx->pdev, DFS_SET_DEBUG_LEVEL, &enable_log,
			sizeof(uint32_t), NULL, NULL, &error);
}

uint32_t wlansap_get_chan_width(struct sap_context *sap_ctx)
{
	return wlan_sap_get_vht_ch_width(sap_ctx);
}

QDF_STATUS wlansap_set_invalid_session(struct sap_context *sap_ctx)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP pointer"));
		return QDF_STATUS_E_FAILURE;
	}

	sap_ctx->sessionId = WLAN_UMAC_VDEV_ID_MAX;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wlansap_release_vdev_ref(struct sap_context *sap_ctx)
{
	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			  FL("Invalid SAP pointer"));
		return QDF_STATUS_E_FAILURE;
	}

	sap_release_vdev_ref(sap_ctx);

	return QDF_STATUS_SUCCESS;
}

void wlansap_cleanup_cac_timer(struct sap_context *sap_ctx)
{
	struct mac_context *mac;

	if (!sap_ctx) {
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("Invalid SAP context"));
		return;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return;
	}

	if (mac->sap.SapDfsInfo.is_dfs_cac_timer_running) {
		qdf_mc_timer_stop(&mac->sap.SapDfsInfo.
				  sap_dfs_cac_timer);
		mac->sap.SapDfsInfo.is_dfs_cac_timer_running = 0;
		qdf_mc_timer_destroy(
			&mac->sap.SapDfsInfo.sap_dfs_cac_timer);
		QDF_TRACE(QDF_MODULE_ID_SAP, QDF_TRACE_LEVEL_ERROR,
			FL("sapdfs, force cleanup running dfs cac timer"));
	}
}

#define DH_OUI_TYPE	(0x20)
/**
 * wlansap_validate_owe_ie() - validate OWE IE
 * @ie: IE buffer
 * @remaining_ie_len: remaining IE length
 *
 * Return: validated IE length, negative for failure
 */
static int wlansap_validate_owe_ie(const uint8_t *ie, uint32_t remaining_ie_len)
{
	uint8_t ie_id, ie_len, ie_ext_id = 0;

	if (remaining_ie_len < 2) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "IE too short");
		return -EINVAL;
	}

	ie_id = ie[0];
	ie_len = ie[1];

	/* IEs that we are expecting in OWE IEs
	 * - RSN IE
	 * - DH IE
	 */
	switch (ie_id) {
	case DOT11F_EID_RSN:
		if (ie_len < DOT11F_IE_RSN_MIN_LEN ||
		    ie_len > DOT11F_IE_RSN_MAX_LEN) {
			QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
					"Invalid RSN IE len %d", ie_len);
			return -EINVAL;
		}
		ie_len += 2;
		break;
	case DOT11F_EID_DH_PARAMETER_ELEMENT:
		ie_ext_id = ie[2];
		if (ie_ext_id != DH_OUI_TYPE) {
			QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
					"Invalid DH IE ID %d", ie_ext_id);
			return -EINVAL;
		}
		if (ie_len < DOT11F_IE_DH_PARAMETER_ELEMENT_MIN_LEN ||
		    ie_len > DOT11F_IE_DH_PARAMETER_ELEMENT_MAX_LEN) {
			QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
					"Invalid DH IE len %d", ie_len);
			return -EINVAL;
		}
		ie_len += 2;
		break;
	default:
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid IE %d", ie_id);
		return -EINVAL;
	}

	if (ie_len > remaining_ie_len) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid IE len");
		return -EINVAL;
	}

	return ie_len;
}

/**
 * wlansap_validate_owe_ies() - validate OWE IEs
 * @ie: IE buffer
 * @ie_len: IE length
 *
 * Return: true if validated
 */
static bool wlansap_validate_owe_ies(const uint8_t *ie, uint32_t ie_len)
{
	const uint8_t *remaining_ie = ie;
	uint32_t remaining_ie_len = ie_len;
	int validated_len;
	bool validated = true;

	while (remaining_ie_len) {
		validated_len = wlansap_validate_owe_ie(remaining_ie,
							remaining_ie_len);
		if (validated_len < 0) {
			validated = false;
			break;
		}
		remaining_ie += validated_len;
		remaining_ie_len -= validated_len;
	}

	return validated;
}

QDF_STATUS wlansap_update_owe_info(struct sap_context *sap_ctx,
				   uint8_t *peer, const uint8_t *ie,
				   uint32_t ie_len, uint16_t owe_status)
{
	struct mac_context *mac;
	struct owe_assoc_ind *owe_assoc_ind;
	struct assoc_ind *assoc_ind = NULL;
	qdf_list_node_t *node = NULL, *next_node = NULL;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (!wlansap_validate_owe_ies(ie, ie_len)) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid OWE IE");
		return QDF_STATUS_E_FAULT;
	}

	if (!sap_ctx) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid SAP context");
		return QDF_STATUS_E_FAULT;
	}

	mac = sap_get_mac_context();
	if (!mac) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP, "Invalid MAC context");
		return QDF_STATUS_E_FAULT;
	}

	if (QDF_STATUS_SUCCESS !=
		qdf_list_peek_front(&sap_ctx->owe_pending_assoc_ind_list,
				    &next_node)) {
		QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
				"Failed to find assoc ind list");
		return QDF_STATUS_E_FAILURE;
	}

	do {
		node = next_node;
		owe_assoc_ind = qdf_container_of(node, struct owe_assoc_ind,
						 node);
		if (qdf_mem_cmp(peer,
				owe_assoc_ind->assoc_ind->peerMacAddr,
				QDF_MAC_ADDR_SIZE) == 0) {
			status = qdf_list_remove_node(
					   &sap_ctx->owe_pending_assoc_ind_list,
					   node);
			if (status != QDF_STATUS_SUCCESS) {
				QDF_TRACE_ERROR(QDF_MODULE_ID_SAP,
						"Failed to remove assoc ind");
				return status;
			}
			assoc_ind = owe_assoc_ind->assoc_ind;
			qdf_mem_free(owe_assoc_ind);
			break;
		}
	} while (QDF_STATUS_SUCCESS ==
		 qdf_list_peek_next(&sap_ctx->owe_pending_assoc_ind_list,
				    node, &next_node));

	if (assoc_ind) {
		assoc_ind->owe_ie = ie;
		assoc_ind->owe_ie_len = ie_len;
		assoc_ind->owe_status = owe_status;
		status = sme_update_owe_info(mac, assoc_ind);
		qdf_mem_free(assoc_ind);
	}

	return status;
}
