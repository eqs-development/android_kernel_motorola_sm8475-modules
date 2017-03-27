/*
 * Copyright (c) 2017 The Linux Foundation. All rights reserved.
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

/**
 * DOC: Implement API's specific to Regulatory component.
 */

#include <qdf_status.h>
#include <wmi_unified_api.h>
#include <wmi_unified_priv.h>
#include <wmi_unified_reg_api.h>

QDF_STATUS wmi_extract_reg_chan_list_update_event(void *wmi_hdl,
						  uint8_t *evt_buf,
						  struct cur_regulatory_info
						  *reg_info,
						  uint32_t len)
{
	struct wmi_unified *wmi_handle = (struct wmi_unified *)wmi_hdl;

	if (wmi_handle->ops->extract_reg_chan_list_update_event)
		return wmi_handle->ops->extract_reg_chan_list_update_event
			(wmi_handle,
			 evt_buf, reg_info, len);

	return QDF_STATUS_E_FAILURE;
}
