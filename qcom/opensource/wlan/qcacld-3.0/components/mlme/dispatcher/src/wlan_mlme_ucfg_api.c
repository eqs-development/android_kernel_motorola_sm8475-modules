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
/**
 * DOC: define UCFG APIs exposed by the mlme component
 */

#include "cfg_ucfg_api.h"
#include "cfg_mlme_sta.h"
#include "wlan_mlme_main.h"
#include "wlan_mlme_api.h"
#include "wlan_mlme_ucfg_api.h"
#include "wlan_objmgr_pdev_obj.h"
#include "wlan_mlme_vdev_mgr_interface.h"

QDF_STATUS ucfg_mlme_init(void)
{
	QDF_STATUS status;

	status = wlan_objmgr_register_psoc_create_handler(
			WLAN_UMAC_COMP_MLME,
			mlme_psoc_object_created_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		mlme_err("unable to register psoc create handle");
		return status;
	}

	status = wlan_objmgr_register_psoc_destroy_handler(
			WLAN_UMAC_COMP_MLME,
			mlme_psoc_object_destroyed_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS)
		mlme_err("unable to register psoc create handle");

	return status;
}

QDF_STATUS ucfg_mlme_deinit(void)
{
	QDF_STATUS status;

	status = wlan_objmgr_unregister_psoc_create_handler(
			WLAN_UMAC_COMP_MLME,
			mlme_psoc_object_created_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS) {
		mlme_err("unable to unregister psoc create handle");
		return status;
	}

	status = wlan_objmgr_unregister_psoc_destroy_handler(
			WLAN_UMAC_COMP_MLME,
			mlme_psoc_object_destroyed_notification,
			NULL);
	if (status != QDF_STATUS_SUCCESS)
		mlme_err("unable to unregister psoc destroy handle");

	return status;
}

QDF_STATUS ucfg_mlme_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status;

	status = mlme_cfg_on_psoc_enable(psoc);
	if (!QDF_IS_STATUS_SUCCESS(status))
		mlme_err("Failed to initialize MLME CFG");

	return status;
}

void ucfg_mlme_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	/* Clear the MLME CFG Structure */
}

#ifdef CONFIG_VDEV_SM
QDF_STATUS ucfg_mlme_pdev_open(struct wlan_objmgr_pdev *pdev)
{
	pdev->pdev_mlme.mlme_register_ops = mlme_register_vdev_mgr_ops;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS ucfg_mlme_pdev_close(struct wlan_objmgr_pdev *pdev)
{
	return QDF_STATUS_SUCCESS;
}
#endif

QDF_STATUS
ucfg_mlme_get_sta_keep_alive_period(struct wlan_objmgr_psoc *psoc,
				    uint32_t *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_INFRA_STA_KEEP_ALIVE_PERIOD);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.sta.sta_keep_alive_period;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_dfs_master_capability(struct wlan_objmgr_psoc *psoc,
				    bool *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_ENABLE_DFS_MASTER_CAPABILITY);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.dfs_cfg.dfs_master_capable;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_pmkid_modes(struct wlan_objmgr_psoc *psoc,
			  uint32_t *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_PMKID_MODES);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.sta.pmkid_modes;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_set_pmkid_modes(struct wlan_objmgr_psoc *psoc,
			  uint32_t val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	mlme_obj->cfg.sta.pmkid_modes = val;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_dot11p_mode(struct wlan_objmgr_psoc *psoc,
			  enum dot11p_mode *out_mode)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*out_mode = cfg_default(CFG_DOT11P_MODE);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*out_mode = mlme_obj->cfg.sta.dot11p_mode;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_go_cts2self_for_sta(struct wlan_objmgr_psoc *psoc,
				  bool *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_ENABLE_GO_CTS2SELF_FOR_STA);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.sta.enable_go_cts2self_for_sta;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_force_rsne_override(struct wlan_objmgr_psoc *psoc,
				  bool *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_FORCE_RSNE_OVERRIDE);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.sta.force_rsne_override;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_qcn_ie_support(struct wlan_objmgr_psoc *psoc,
			     bool *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_QCN_IE_SUPPORT);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.sta.qcn_ie_support;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_tgt_gtx_usr_cfg(struct wlan_objmgr_psoc *psoc,
			      uint32_t *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_TGT_GTX_USR_CFG);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.sta.tgt_gtx_usr_cfg;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_roaming_offload(struct wlan_objmgr_psoc *psoc,
			      bool *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_LFR3_ROAMING_OFFLOAD);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.lfr.lfr3_roaming_offload;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_set_roaming_offload(struct wlan_objmgr_psoc *psoc,
			      bool val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	mlme_obj->cfg.lfr.lfr3_roaming_offload = val;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_first_scan_bucket_threshold(struct wlan_objmgr_psoc *psoc,
					  uint8_t *val)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		*val = cfg_default(CFG_LFR_FIRST_SCAN_BUCKET_THRESHOLD);
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	*val = mlme_obj->cfg.lfr.first_scan_bucket_threshold;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_mlme_get_opr_rate_set(struct wlan_objmgr_psoc *psoc,
			   uint8_t *buf, qdf_size_t *len)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	return wlan_mlme_get_cfg_str(buf, &mlme_obj->cfg.rates.opr_rate_set,
				     len);
}

QDF_STATUS
ucfg_mlme_get_ext_opr_rate_set(struct wlan_objmgr_psoc *psoc,
			       uint8_t *buf, qdf_size_t *len)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	return wlan_mlme_get_cfg_str(buf, &mlme_obj->cfg.rates.ext_opr_rate_set,
				     len);
}

QDF_STATUS
ucfg_mlme_get_supported_mcs_set(struct wlan_objmgr_psoc *psoc,
				uint8_t *buf, qdf_size_t *len)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	return wlan_mlme_get_cfg_str(buf,
				     &mlme_obj->cfg.rates.supported_mcs_set,
				     len);
}

QDF_STATUS
ucfg_mlme_set_supported_mcs_set(struct wlan_objmgr_psoc *psoc,
				uint8_t *buf, qdf_size_t len)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	return wlan_mlme_set_cfg_str(buf,
				     &mlme_obj->cfg.rates.supported_mcs_set,
				     len);
}

QDF_STATUS
ucfg_mlme_get_current_mcs_set(struct wlan_objmgr_psoc *psoc,
			      uint8_t *buf, qdf_size_t *len)
{
	struct wlan_mlme_psoc_obj *mlme_obj;

	mlme_obj = mlme_get_psoc_obj(psoc);
	if (!mlme_obj) {
		mlme_err("mlme obj null");
		return QDF_STATUS_E_INVAL;
	}

	return wlan_mlme_get_cfg_str(buf,
				     &mlme_obj->cfg.rates.current_mcs_set,
				     len);
}
