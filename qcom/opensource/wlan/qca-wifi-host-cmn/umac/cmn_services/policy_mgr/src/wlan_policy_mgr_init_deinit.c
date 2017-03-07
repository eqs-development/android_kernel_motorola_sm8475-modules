/*
 * Copyright (c) 2012-2017 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
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

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * DOC: wlan_policy_mgr_init_deinit.c
 *
 * WLAN Concurrenct Connection Management APIs
 *
 */

/* Include files */

#include "wlan_policy_mgr_api.h"
#include "wlan_policy_mgr_tables_1x1_dbs_i.h"
#include "wlan_policy_mgr_tables_2x2_dbs_i.h"
#include "wlan_policy_mgr_i.h"
#include "qdf_types.h"
#include "qdf_trace.h"
#include "wlan_objmgr_global_obj.h"

static QDF_STATUS policy_mgr_psoc_obj_create_cb(struct wlan_objmgr_psoc *psoc,
		void *data)
{
	struct policy_mgr_psoc_priv_obj *policy_mgr_ctx;

	policy_mgr_ctx = qdf_mem_malloc(
		sizeof(struct policy_mgr_psoc_priv_obj));
	if (!policy_mgr_ctx) {
		policy_mgr_err("memory allocation failed");
		return QDF_STATUS_E_FAILURE;
	}

	policy_mgr_ctx->psoc = psoc;

	wlan_objmgr_psoc_component_obj_attach(psoc,
			WLAN_UMAC_COMP_POLICY_MGR,
			policy_mgr_ctx,
			QDF_STATUS_SUCCESS);

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS policy_mgr_psoc_obj_destroy_cb(struct wlan_objmgr_psoc *psoc,
		void *data)
{
	struct policy_mgr_psoc_priv_obj *policy_mgr_ctx;

	policy_mgr_ctx = policy_mgr_get_context(psoc);
	wlan_objmgr_psoc_component_obj_detach(psoc,
					WLAN_UMAC_COMP_POLICY_MGR,
					policy_mgr_ctx);
	qdf_mem_free(policy_mgr_ctx);

	return QDF_STATUS_SUCCESS;
}

static void policy_mgr_psoc_obj_status_cb(struct wlan_objmgr_psoc *psoc,
		void *data, QDF_STATUS status)
{
	return;
}

static QDF_STATUS policy_mgr_vdev_obj_create_cb(struct wlan_objmgr_vdev *vdev,
		void *data)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS policy_mgr_vdev_obj_destroy_cb(struct wlan_objmgr_vdev *vdev,
		void *data)
{
	return QDF_STATUS_SUCCESS;
}

static void policy_mgr_vdev_obj_status_cb(struct wlan_objmgr_vdev *vdev,
		void *data, QDF_STATUS status)
{
	return;
}

QDF_STATUS policy_mgr_init(void)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	status = wlan_objmgr_register_psoc_create_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_psoc_obj_create_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS) {
		policy_mgr_err("Failed to register psoc obj create cback");
		goto err_psoc_create;
	}

	status = wlan_objmgr_register_psoc_destroy_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_psoc_obj_destroy_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS) {
		policy_mgr_err("Failed to register psoc obj delete cback");
		goto err_psoc_delete;
	}

	status = wlan_objmgr_register_psoc_status_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_psoc_obj_status_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS) {
		policy_mgr_err("Failed to register psoc obj status cback");
		goto err_psoc_status;
	}

	status = wlan_objmgr_register_vdev_create_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_vdev_obj_create_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS) {
		policy_mgr_err("Failed to register vdev obj create cback");
		goto err_vdev_create;
	}

	status = wlan_objmgr_register_vdev_destroy_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_vdev_obj_destroy_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS) {
		policy_mgr_err("Failed to register vdev obj delete cback");
		goto err_vdev_delete;
	}

	status = wlan_objmgr_register_vdev_status_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_vdev_obj_status_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS) {
		policy_mgr_err("Failed to register vdev obj status cback");
		goto err_vdev_status;
	}

	policy_mgr_notice("Callbacks registered with obj mgr");

	return QDF_STATUS_SUCCESS;

err_vdev_status:
	wlan_objmgr_unregister_vdev_destroy_handler(WLAN_UMAC_COMP_POLICY_MGR,
			policy_mgr_vdev_obj_destroy_cb, NULL);
err_vdev_delete:
	wlan_objmgr_unregister_vdev_create_handler(WLAN_UMAC_COMP_POLICY_MGR,
			policy_mgr_vdev_obj_create_cb, NULL);
err_vdev_create:
	wlan_objmgr_unregister_psoc_status_handler(WLAN_UMAC_COMP_POLICY_MGR,
			policy_mgr_psoc_obj_status_cb, NULL);
err_psoc_status:
	wlan_objmgr_unregister_psoc_destroy_handler(WLAN_UMAC_COMP_POLICY_MGR,
			policy_mgr_psoc_obj_destroy_cb, NULL);
err_psoc_delete:
	wlan_objmgr_unregister_psoc_create_handler(WLAN_UMAC_COMP_POLICY_MGR,
			policy_mgr_psoc_obj_create_cb, NULL);
err_psoc_create:
	return status;
}

QDF_STATUS policy_mgr_deinit(void)
{
	QDF_STATUS status;

	status = wlan_objmgr_unregister_psoc_create_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_psoc_obj_create_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS)
		policy_mgr_err("Failed to deregister psoc obj create cback");

	status = wlan_objmgr_unregister_psoc_destroy_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_psoc_obj_destroy_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS)
		policy_mgr_err("Failed to deregister psoc obj delete cback");

	status = wlan_objmgr_unregister_psoc_status_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_psoc_obj_status_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS)
		policy_mgr_err("Failed to deregister psoc obj status cback");

	status = wlan_objmgr_unregister_vdev_create_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_vdev_obj_create_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS)
		policy_mgr_err("Failed to deregister vdev obj create cback");

	status = wlan_objmgr_unregister_vdev_destroy_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_vdev_obj_destroy_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS)
		policy_mgr_err("Failed to deregister vdev obj delete cback");

	status = wlan_objmgr_unregister_vdev_status_handler(
				WLAN_UMAC_COMP_POLICY_MGR,
				policy_mgr_vdev_obj_status_cb,
				NULL);
	if (status != QDF_STATUS_SUCCESS)
		policy_mgr_err("Failed to deregister vdev obj status cback");

	policy_mgr_info("deregistered callbacks with obj mgr successfully");

	return status;
}

QDF_STATUS policy_mgr_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	/* placeholder for now */
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS policy_mgr_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	/* placeholder for now */
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS policy_mgr_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status;
	struct policy_mgr_psoc_priv_obj *pm_ctx;

	pm_ctx = policy_mgr_get_context(psoc);
	if (!pm_ctx) {
		policy_mgr_err("Invalid Context");
		return QDF_STATUS_E_FAILURE;
	}

	policy_mgr_debug("Initializing the policy manager");

	/* init pm_conc_connection_list */
	qdf_mem_zero(pm_conc_connection_list, sizeof(pm_conc_connection_list));

	/* init dbs_opportunistic_timer */
	status = qdf_mc_timer_init(&pm_ctx->dbs_opportunistic_timer,
				QDF_TIMER_TYPE_SW,
				pm_dbs_opportunistic_timer_handler,
				(void *)psoc);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		policy_mgr_err("Failed to init DBS opportunistic timer");
		return status;
	}

	/* init connection_update_done_evt */
	status = policy_mgr_init_connection_update(pm_ctx);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		policy_mgr_err("connection_update_done_evt init failed");
		return status;
	}

	pm_ctx->do_hw_mode_change = false;

	/* reset sap mandatory channels */
	status = policy_mgr_reset_sap_mandatory_channels(pm_ctx);
	if (QDF_IS_STATUS_ERROR(status)) {
		policy_mgr_err("failed to reset mandatory channels");
		return status;
	}

	/* init PCL table & function pointers based on HW capability */
	if (policy_mgr_is_hw_dbs_2x2_capable(psoc))
		policy_mgr_get_current_pref_hw_mode_ptr =
		policy_mgr_get_current_pref_hw_mode_dbs_2x2;
	else
		policy_mgr_get_current_pref_hw_mode_ptr =
		policy_mgr_get_current_pref_hw_mode_dbs_1x1;

	if (policy_mgr_is_hw_dbs_2x2_capable(psoc))
		second_connection_pcl_dbs_table =
		&pm_second_connection_pcl_dbs_2x2_table;
	else
		second_connection_pcl_dbs_table =
		&pm_second_connection_pcl_dbs_1x1_table;

	if (policy_mgr_is_hw_dbs_2x2_capable(psoc))
		third_connection_pcl_dbs_table =
		&pm_third_connection_pcl_dbs_2x2_table;
	else
		third_connection_pcl_dbs_table =
		&pm_third_connection_pcl_dbs_1x1_table;

	if (policy_mgr_is_hw_dbs_2x2_capable(psoc))
		next_action_two_connection_table =
		&pm_next_action_two_connection_dbs_2x2_table;
	else
		next_action_two_connection_table =
		&pm_next_action_two_connection_dbs_1x1_table;

	if (policy_mgr_is_hw_dbs_2x2_capable(psoc))
		next_action_three_connection_table =
		&pm_next_action_three_connection_dbs_2x2_table;
	else
		next_action_three_connection_table =
		&pm_next_action_three_connection_dbs_1x1_table;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS policy_mgr_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	struct policy_mgr_psoc_priv_obj *pm_ctx;

	pm_ctx = policy_mgr_get_context(psoc);
	if (!pm_ctx) {
		policy_mgr_err("Invalid Context");
		return QDF_STATUS_E_FAILURE;
	}

	/* destroy connection_update_done_evt */
	if (!QDF_IS_STATUS_SUCCESS(qdf_event_destroy
		(&pm_ctx->connection_update_done_evt))) {
		policy_mgr_err("Failed to destroy connection_update_done_evt");
		status = QDF_STATUS_E_FAILURE;
		QDF_ASSERT(0);
	}

	/* deallocate dbs_opportunistic_timer */
	if (QDF_TIMER_STATE_RUNNING ==
			qdf_mc_timer_get_current_state(
				&pm_ctx->dbs_opportunistic_timer)) {
		qdf_mc_timer_stop(&pm_ctx->dbs_opportunistic_timer);
	}

	if (!QDF_IS_STATUS_SUCCESS(qdf_mc_timer_destroy(
			&pm_ctx->dbs_opportunistic_timer))) {
		policy_mgr_err("Cannot deallocate dbs opportunistic timer");
		status = QDF_STATUS_E_FAILURE;
		QDF_ASSERT(0);
	}

	/* reset sap mandatory channels */
	if (QDF_IS_STATUS_ERROR(
		policy_mgr_reset_sap_mandatory_channels(pm_ctx))) {
		policy_mgr_err("failed to reset sap mandatory channels");
		status = QDF_STATUS_E_FAILURE;
		QDF_ASSERT(0);
	}

	/* deinit pm_conc_connection_list */
	qdf_mem_zero(pm_conc_connection_list, sizeof(pm_conc_connection_list));

	return status;
}

QDF_STATUS policy_mgr_register_sme_cb(struct wlan_objmgr_psoc *psoc,
		struct policy_mgr_sme_cbacks *sme_cbacks)
{
	struct policy_mgr_psoc_priv_obj *pm_ctx;

	pm_ctx = policy_mgr_get_context(psoc);
	if (!pm_ctx) {
		policy_mgr_err("Invalid Context");
		return QDF_STATUS_E_FAILURE;
	}

	pm_ctx->sme_cbacks.sme_get_nss_for_vdev =
		sme_cbacks->sme_get_nss_for_vdev;
	pm_ctx->sme_cbacks.sme_get_valid_channels =
		sme_cbacks->sme_get_valid_channels;
	pm_ctx->sme_cbacks.sme_nss_update_request =
		sme_cbacks->sme_nss_update_request;
	pm_ctx->sme_cbacks.sme_pdev_set_hw_mode =
		sme_cbacks->sme_pdev_set_hw_mode;
	pm_ctx->sme_cbacks.sme_pdev_set_pcl =
		sme_cbacks->sme_pdev_set_pcl;
	pm_ctx->sme_cbacks.sme_soc_set_dual_mac_config =
		sme_cbacks->sme_soc_set_dual_mac_config;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS policy_mgr_register_wma_cb(struct wlan_objmgr_psoc *psoc,
		struct policy_mgr_wma_cbacks *wma_cbacks)
{
	struct policy_mgr_psoc_priv_obj *pm_ctx;

	pm_ctx = policy_mgr_get_context(psoc);
	if (!pm_ctx) {
		policy_mgr_err("Invalid Context");
		return QDF_STATUS_E_FAILURE;
	}

	pm_ctx->wma_cbacks.wma_get_connection_info =
		wma_cbacks->wma_get_connection_info;
	pm_ctx->wma_cbacks.wma_is_service_enabled =
		wma_cbacks->wma_is_service_enabled;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS policy_mgr_register_cdp_cb(struct wlan_objmgr_psoc *psoc,
		struct policy_mgr_cdp_cbacks *cdp_cbacks)
{
	struct policy_mgr_psoc_priv_obj *pm_ctx;

	pm_ctx = policy_mgr_get_context(psoc);
	if (!pm_ctx) {
		policy_mgr_err("Invalid Context");
		return QDF_STATUS_E_FAILURE;
	}

	pm_ctx->cdp_cbacks.cdp_update_mac_id =
		cdp_cbacks->cdp_update_mac_id;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS policy_mgr_register_tdls_cb(struct wlan_objmgr_psoc *psoc,
		struct policy_mgr_tdls_cbacks *tdls_cbacks)
{
	struct policy_mgr_psoc_priv_obj *pm_ctx;

	pm_ctx = policy_mgr_get_context(psoc);
	if (!pm_ctx) {
		policy_mgr_err("Invalid Context");
		return QDF_STATUS_E_FAILURE;
	}

	pm_ctx->tdls_cbacks.check_is_tdls_allowed =
		tdls_cbacks->check_is_tdls_allowed;
	pm_ctx->tdls_cbacks.set_tdls_ct_mode =
		tdls_cbacks->set_tdls_ct_mode;

	return QDF_STATUS_SUCCESS;
}
