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
 * DOC: offload lmac interface APIs definitions for scan
 */

#include <qdf_mem.h>
#include <qdf_status.h>
#include <wmi_unified_api.h>
#include <wmi_unified_priv.h>
#include <wmi_unified_param.h>
#include <wlan_objmgr_psoc_obj.h>
#include <wlan_scan_tgt_api.h>
#include <target_if.h>

static inline struct wlan_lmac_if_scan_rx_ops *
target_if_scan_get_rx_ops(struct wlan_objmgr_psoc *psoc)
{
	return &psoc->soc_cb.rx_ops.scan;
}

static int
target_if_scan_event_handler(ol_scn_t scn, uint8_t *data, uint32_t datalen)
{
	struct scan_event_info *event_info;
	struct wlan_objmgr_psoc *psoc;
	struct wmi_unified *wmi_handle;
	struct wlan_lmac_if_scan_rx_ops *scan_rx_ops;
	QDF_STATUS status;

	if (!scn || !data) {
		target_if_err("scn: 0x%p, data: 0x%p\n", scn, data);
		return -EINVAL;
	}
	psoc = target_if_get_psoc_from_scn_hdl(scn);
	if (!psoc) {
		target_if_err("null psoc\n");
		return -EINVAL;
	}
	wmi_handle = GET_WMI_HDL_FROM_PSOC(psoc);

	event_info = qdf_mem_malloc(sizeof(*event_info));

	if (!event_info) {
		target_if_err("%s: unable to allocate scan_event\n", __func__);
		return -ENOMEM;
	}

	if (wmi_extract_vdev_scan_ev_param(wmi_handle, data,
			&(event_info->event))) {
		target_if_err("%s: Failed to extract wmi scan event\n",
			__func__);
		qdf_mem_free(event_info);
		return -EINVAL;
	}

	scan_rx_ops = target_if_scan_get_rx_ops(psoc);
	if (scan_rx_ops->scan_ev_handler) {
		status = scan_rx_ops->scan_ev_handler(psoc, event_info);
		if (status != QDF_STATUS_SUCCESS) {
			qdf_mem_free(event_info);
			return -EINVAL;
		}
	}

	return 0;
}

QDF_STATUS
target_if_scan_register_event_handler(struct wlan_objmgr_psoc *psoc, void *arg)
{
	return wmi_unified_register_event_handler(psoc->tgt_if_handle,
		wmi_scan_event_id, target_if_scan_event_handler,
		WMI_RX_UMAC_CTX);
}

QDF_STATUS
target_if_scan_unregister_event_handler(struct wlan_objmgr_psoc *psoc,
		void *arg)
{
	return wmi_unified_unregister_event_handler(psoc->tgt_if_handle,
		wmi_scan_event_id);
}

QDF_STATUS
target_if_scan_start(struct wlan_objmgr_psoc *psoc,
		struct scan_start_request *req)
{
	return wmi_unified_scan_start_cmd_send(psoc->tgt_if_handle,
		&req->scan_req);
}


QDF_STATUS
target_if_scan_cancel(struct wlan_objmgr_psoc *psoc,
		struct scan_cancel_param *req)
{
	return wmi_unified_scan_stop_cmd_send(psoc->tgt_if_handle, req);
}
