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
 * DOC: wifi_pos_api.c
 * This file defines the APIs wifi_pos component.
 */

#include "wifi_pos_api.h"
#include "wifi_pos_utils_i.h"
#include "wifi_pos_main_i.h"
#include "os_if_wifi_pos.h"
#include "target_if_wifi_pos.h"
#include "wlan_objmgr_cmn.h"
#include "wlan_objmgr_global_obj.h"
#include "wlan_objmgr_psoc_obj.h"

QDF_STATUS wifi_pos_init(void)
{
	QDF_STATUS status;

	/* register psoc create handler functions. */
	status = wlan_objmgr_register_psoc_create_handler(
		WLAN_UMAC_COMP_WIFI_POS,
		wifi_pos_psoc_obj_created_notification,
		NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		wifi_pos_err("register_psoc_create_handler failed, status: %d",
			     status);
		return status;
	}

	/* register psoc delete handler functions. */
	status = wlan_objmgr_register_psoc_destroy_handler(
		WLAN_UMAC_COMP_WIFI_POS,
		wifi_pos_psoc_obj_destroyed_notification,
		NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		wifi_pos_err("register_psoc_destroy_handler failed, status: %d",
			     status);
	}

	return status;
}

QDF_STATUS wifi_pos_deinit(void)
{
	QDF_STATUS status;

	/* deregister psoc create handler functions. */
	status = wlan_objmgr_unregister_psoc_create_handler(
				WLAN_UMAC_COMP_WIFI_POS,
				wifi_pos_psoc_obj_created_notification,
				NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		wifi_pos_err("unregister_psoc_create_handler failed, status: %d",
			     status);
		return status;
	}

	/* deregister psoc delete handler functions. */
	status = wlan_objmgr_unregister_psoc_destroy_handler(
				WLAN_UMAC_COMP_WIFI_POS,
				wifi_pos_psoc_obj_destroyed_notification,
				NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		wifi_pos_err("unregister_psoc_destroy_handler failed, status: %d",
			     status);
	}
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wifi_pos_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status = target_if_wifi_pos_register_events(psoc);

	if (QDF_IS_STATUS_ERROR(status))
		wifi_pos_err("target_if_wifi_pos_register_events failed");

	return status;
}

QDF_STATUS wifi_pos_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status = target_if_wifi_pos_deregister_events(psoc);

	if (QDF_IS_STATUS_ERROR(status))
		wifi_pos_err("target_if_wifi_pos_deregister_events failed");

	return QDF_STATUS_SUCCESS;
}

void wifi_pos_set_oem_target_type(struct wlan_objmgr_psoc *psoc, uint32_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->oem_target_type = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_oem_fw_version(struct wlan_objmgr_psoc *psoc, uint32_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->oem_fw_version = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_drv_ver_major(struct wlan_objmgr_psoc *psoc, uint8_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->driver_version.major = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_drv_ver_minor(struct wlan_objmgr_psoc *psoc, uint8_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->driver_version.minor = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_drv_ver_patch(struct wlan_objmgr_psoc *psoc, uint8_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->driver_version.patch = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_drv_ver_build(struct wlan_objmgr_psoc *psoc, uint8_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->driver_version.build = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_dwell_time_min(struct wlan_objmgr_psoc *psoc, uint16_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->allowed_dwell_time_min = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}
void wifi_pos_set_dwell_time_max(struct wlan_objmgr_psoc *psoc, uint16_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->allowed_dwell_time_max = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_current_dwell_time_max(struct wlan_objmgr_psoc *psoc,
					 uint16_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->current_dwell_time_max = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}

void wifi_pos_set_current_dwell_time_min(struct wlan_objmgr_psoc *psoc,
					 uint16_t val)
{
	struct wifi_pos_psoc_priv_obj *wifi_pos_psoc =
			wifi_pos_get_psoc_priv_obj(psoc);

	qdf_spin_lock_bh(&wifi_pos_psoc->wifi_pos_lock);
	wifi_pos_psoc->current_dwell_time_max = val;
	qdf_spin_unlock_bh(&wifi_pos_psoc->wifi_pos_lock);
}
