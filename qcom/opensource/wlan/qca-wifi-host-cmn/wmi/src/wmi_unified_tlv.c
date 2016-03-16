/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
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

#include "wmi_unified_tlv.h"
#include "wmi_unified_api.h"
#include "wmi.h"
#include "wmi_unified_priv.h"
#include "wma_api.h"
#include "wma.h"
#include "wmi_version_whitelist.h"

/**
 * send_vdev_create_cmd_tlv() - send VDEV create command to fw
 * @wmi_handle: wmi handle
 * @param: pointer to hold vdev create parameter
 * @macaddr: vdev mac address
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_vdev_create_cmd_tlv(wmi_unified_t wmi_handle,
				 uint8_t macaddr[IEEE80211_ADDR_LEN],
				 struct vdev_create_params *param)
{
	wmi_vdev_create_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);
	int32_t ret;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_vdev_create_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_create_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_create_cmd_fixed_param));
	cmd->vdev_id = param->if_id;
	cmd->vdev_type = param->type;
	cmd->vdev_subtype = param->subtype;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(macaddr, &cmd->vdev_macaddr);
	WMI_LOGD("%s: ID = %d VAP Addr = %02x:%02x:%02x:%02x:%02x:%02x",
		 __func__, param->if_id,
		 macaddr[0], macaddr[1], macaddr[2],
		 macaddr[3], macaddr[4], macaddr[5]);
	ret = wmi_unified_cmd_send(wmi_handle, buf, len, WMI_VDEV_CREATE_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send WMI_VDEV_CREATE_CMDID");
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 * send_vdev_delete_cmd_tlv() - send VDEV delete command to fw
 * @wmi_handle: wmi handle
 * @if_id: vdev id
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_vdev_delete_cmd_tlv(wmi_unified_t wmi_handle,
					  uint8_t if_id)
{
	wmi_vdev_delete_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t ret;

	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGP("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}

	cmd = (wmi_vdev_delete_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_delete_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_delete_cmd_fixed_param));
	cmd->vdev_id = if_id;
	ret = wmi_unified_cmd_send(wmi_handle, buf,
				   sizeof(wmi_vdev_delete_cmd_fixed_param),
				   WMI_VDEV_DELETE_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send WMI_VDEV_DELETE_CMDID");
		wmi_buf_free(buf);
	}
	WMI_LOGD("%s:vdev id = %d", __func__, if_id);

	return ret;
}

/**
 * send_vdev_stop_cmd_tlv() - send vdev stop command to fw
 * @wmi: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or erro code
 */
QDF_STATUS send_vdev_stop_cmd_tlv(wmi_unified_t wmi,
					uint8_t vdev_id)
{
	wmi_vdev_stop_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi, len);
	if (!buf) {
		WMI_LOGP("%s : wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_vdev_stop_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_stop_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_vdev_stop_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	if (wmi_unified_cmd_send(wmi, buf, len, WMI_VDEV_STOP_CMDID)) {
		WMI_LOGP("%s: Failed to send vdev stop command", __func__);
		qdf_nbuf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_vdev_down_cmd_tlv() - send vdev down command to fw
 * @wmi: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_vdev_down_cmd_tlv(wmi_unified_t wmi, uint8_t vdev_id)
{
	wmi_vdev_down_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi, len);
	if (!buf) {
		WMI_LOGP("%s : wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_vdev_down_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_down_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_vdev_down_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	if (wmi_unified_cmd_send(wmi, buf, len, WMI_VDEV_DOWN_CMDID)) {
		WMI_LOGP("%s: Failed to send vdev down", __func__);
		qdf_nbuf_free(buf);
		return -EIO;
	}
	WMI_LOGD("%s: vdev_id %d", __func__, vdev_id);

	return 0;
}

/**
 * send_peer_flush_tids_cmd_tlv() - flush peer tids packets in fw
 * @wmi: wmi handle
 * @peer_addr: peer mac address
 * @param: pointer to hold peer flush tid parameter
 *
 * Return: 0 for sucess or error code
 */
QDF_STATUS send_peer_flush_tids_cmd_tlv(wmi_unified_t wmi,
					 uint8_t peer_addr[IEEE80211_ADDR_LEN],
					 struct peer_flush_params *param)
{
	wmi_peer_flush_tids_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_peer_flush_tids_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_flush_tids_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_peer_flush_tids_cmd_fixed_param));
	WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
	cmd->peer_tid_bitmap = param->peer_tid_bitmap;
	cmd->vdev_id = param->vdev_id;
	WMI_LOGD("%s: peer_addr %pM vdev_id %d and peer bitmap %d", __func__,
				peer_addr, param->vdev_id,
				param->peer_tid_bitmap);
	if (wmi_unified_cmd_send(wmi, buf, len, WMI_PEER_FLUSH_TIDS_CMDID)) {
		WMI_LOGP("%s: Failed to send flush tid command", __func__);
		qdf_nbuf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_peer_delete_cmd_tlv() - send PEER delete command to fw
 * @wmi: wmi handle
 * @peer_addr: peer mac addr
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_peer_delete_cmd_tlv(wmi_unified_t wmi,
				 uint8_t peer_addr[IEEE80211_ADDR_LEN],
				 uint8_t vdev_id)
{
	wmi_peer_delete_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_peer_delete_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_delete_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_peer_delete_cmd_fixed_param));
	WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
	cmd->vdev_id = vdev_id;

	if (wmi_unified_cmd_send(wmi, buf, len, WMI_PEER_DELETE_CMDID)) {
		WMI_LOGP("%s: Failed to send peer delete command", __func__);
		qdf_nbuf_free(buf);
		return -EIO;
	}
	WMI_LOGD("%s: peer_addr %pM vdev_id %d", __func__, peer_addr, vdev_id);

	return 0;
}

/**
 * send_peer_param_cmd_tlv() - set peer parameter in fw
 * @wmi: wmi handle
 * @peer_addr: peer mac address
 * @param    : pointer to hold peer set parameter
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_peer_param_cmd_tlv(wmi_unified_t wmi,
				uint8_t peer_addr[IEEE80211_ADDR_LEN],
				struct peer_set_params *param)
{
	wmi_peer_set_param_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t err;

	buf = wmi_buf_alloc(wmi, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send set_param cmd");
		return -ENOMEM;
	}
	cmd = (wmi_peer_set_param_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_set_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
				(wmi_peer_set_param_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
	cmd->param_id = param->param_id;
	cmd->param_value = param->param_value;
	err = wmi_unified_cmd_send(wmi, buf,
				   sizeof(wmi_peer_set_param_cmd_fixed_param),
				   WMI_PEER_SET_PARAM_CMDID);
	if (err) {
		WMI_LOGE("Failed to send set_param cmd");
		qdf_mem_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_vdev_up_cmd_tlv() - send vdev up command in fw
 * @wmi: wmi handle
 * @bssid: bssid
 * @vdev_up_params: pointer to hold vdev up parameter
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_vdev_up_cmd_tlv(wmi_unified_t wmi,
			     uint8_t bssid[IEEE80211_ADDR_LEN],
				 struct vdev_up_params *params)
{
	wmi_vdev_up_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	WMI_LOGD("%s: VDEV_UP", __func__);
	WMI_LOGD("%s: vdev_id %d aid %d bssid %pM", __func__,
		 params->vdev_id, params->assoc_id, bssid);
	buf = wmi_buf_alloc(wmi, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_vdev_up_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_up_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_vdev_up_cmd_fixed_param));
	cmd->vdev_id = params->vdev_id;
	cmd->vdev_assoc_id = params->assoc_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(bssid, &cmd->vdev_bssid);
	if (wmi_unified_cmd_send(wmi, buf, len, WMI_VDEV_UP_CMDID)) {
		WMI_LOGP("%s: Failed to send vdev up command", __func__);
		qdf_nbuf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_peer_create_cmd_tlv() - send peer create command to fw
 * @wmi: wmi handle
 * @peer_addr: peer mac address
 * @peer_type: peer type
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_peer_create_cmd_tlv(wmi_unified_t wmi,
					struct peer_create_params *param)
{
	wmi_peer_create_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_peer_create_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_create_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_peer_create_cmd_fixed_param));
	WMI_CHAR_ARRAY_TO_MAC_ADDR(param->peer_addr, &cmd->peer_macaddr);
	cmd->peer_type = param->peer_type;
	cmd->vdev_id = param->vdev_id;

	if (wmi_unified_cmd_send(wmi, buf, len, WMI_PEER_CREATE_CMDID)) {
		WMI_LOGP("%s: failed to send WMI_PEER_CREATE_CMDID", __func__);
		qdf_nbuf_free(buf);
		return -EIO;
	}
	WMI_LOGD("%s: peer_addr %pM vdev_id %d", __func__, param->peer_addr,
			param->vdev_id);

	return 0;
}

/**
 * send_green_ap_ps_cmd_tlv() - enable green ap powersave command
 * @wmi_handle: wmi handle
 * @value: value
 * @mac_id: mac id to have radio context
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_green_ap_ps_cmd_tlv(wmi_unified_t wmi_handle,
						uint32_t value, uint8_t mac_id)
{
	wmi_pdev_green_ap_ps_enable_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	WMI_LOGD("Set Green AP PS val %d", value);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: Green AP PS Mem Alloc Failed", __func__);
		return -ENOMEM;
	}

	cmd = (wmi_pdev_green_ap_ps_enable_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		   WMITLV_TAG_STRUC_wmi_pdev_green_ap_ps_enable_cmd_fixed_param,
		   WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_green_ap_ps_enable_cmd_fixed_param));
	cmd->reserved0 = 0;
	cmd->enable = value;

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_PDEV_GREEN_AP_PS_ENABLE_CMDID)) {
		WMI_LOGE("Set Green AP PS param Failed val %d", value);
		qdf_nbuf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_pdev_utf_cmd_tlv() - send utf command to fw
 * @wmi_handle: wmi handle
 * @param: pointer to pdev_utf_params
 * @mac_id: mac id to have radio context
 *
 * Return: 0 for success or error code
 */
QDF_STATUS
send_pdev_utf_cmd_tlv(wmi_unified_t wmi_handle,
				struct pdev_utf_params *param,
				uint8_t mac_id)
{
	wmi_buf_t buf;
	uint8_t *cmd;
	int32_t ret = 0;
	static uint8_t msgref = 1;
	uint8_t segNumber = 0, segInfo, numSegments;
	uint16_t chunk_len, total_bytes;
	uint8_t *bufpos;
	struct seg_hdr_info segHdrInfo;

	bufpos = param->utf_payload;
	total_bytes = param->len;
	ASSERT(total_bytes / MAX_WMI_UTF_LEN ==
	       (uint8_t) (total_bytes / MAX_WMI_UTF_LEN));
	numSegments = (uint8_t) (total_bytes / MAX_WMI_UTF_LEN);

	if (param->len - (numSegments * MAX_WMI_UTF_LEN))
		numSegments++;

	while (param->len) {
		if (param->len > MAX_WMI_UTF_LEN)
			chunk_len = MAX_WMI_UTF_LEN;    /* MAX messsage */
		else
			chunk_len = param->len;

		buf = wmi_buf_alloc(wmi_handle,
				    (chunk_len + sizeof(segHdrInfo) +
				     WMI_TLV_HDR_SIZE));
		if (!buf) {
			WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
			return -ENOMEM;
		}

		cmd = (uint8_t *) wmi_buf_data(buf);

		segHdrInfo.len = total_bytes;
		segHdrInfo.msgref = msgref;
		segInfo = ((numSegments << 4) & 0xF0) | (segNumber & 0xF);
		segHdrInfo.segmentInfo = segInfo;
		segHdrInfo.pad = 0;

		WMI_LOGD("%s:segHdrInfo.len = %d, segHdrInfo.msgref = %d,"
			 " segHdrInfo.segmentInfo = %d",
			 __func__, segHdrInfo.len, segHdrInfo.msgref,
			 segHdrInfo.segmentInfo);

		WMI_LOGD("%s:total_bytes %d segNumber %d totalSegments %d"
			 "chunk len %d", __func__, total_bytes, segNumber,
			 numSegments, chunk_len);

		segNumber++;

		WMITLV_SET_HDR(cmd, WMITLV_TAG_ARRAY_BYTE,
			       (chunk_len + sizeof(segHdrInfo)));
		cmd += WMI_TLV_HDR_SIZE;
		memcpy(cmd, &segHdrInfo, sizeof(segHdrInfo));   /* 4 bytes */
		memcpy(&cmd[sizeof(segHdrInfo)], bufpos, chunk_len);

		ret = wmi_unified_cmd_send(wmi_handle, buf,
					   (chunk_len + sizeof(segHdrInfo) +
					    WMI_TLV_HDR_SIZE),
					   WMI_PDEV_UTF_CMDID);

		if (ret != EOK) {
			WMI_LOGE("Failed to send WMI_PDEV_UTF_CMDID command");
			wmi_buf_free(buf);
			break;
		}

		param->len -= chunk_len;
		bufpos += chunk_len;
	}

	msgref++;

	return ret;
}

/**
 * send_pdev_param_cmd_tlv() - set pdev parameters
 * @wmi_handle: wmi handle
 * @param: pointer to pdev parameter
 * @mac_id: radio context
 *
 * Return: 0 on success, errno on failure
 */
QDF_STATUS
send_pdev_param_cmd_tlv(wmi_unified_t wmi_handle,
			   struct pdev_params *param,
				uint8_t mac_id)
{
	int32_t ret;
	wmi_pdev_set_param_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_pdev_set_param_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_pdev_set_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_set_param_cmd_fixed_param));
	cmd->reserved0 = 0;
	cmd->param_id = param->param_id;
	cmd->param_value = param->param_value;
	WMI_LOGD("Setting pdev param = %x, value = %u", param->param_id,
				param->param_value);
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_PDEV_SET_PARAM_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send set param command ret = %d", ret);
		wmi_buf_free(buf);
	}
	return ret;
}

/**
 * send_suspend_cmd_tlv() - WMI suspend function
 * @param wmi_handle      : handle to WMI.
 * @param param    : pointer to hold suspend parameter
 * @mac_id: radio context
 *
 * Return 0  on success and -ve on failure.
 */
QDF_STATUS send_suspend_cmd_tlv(wmi_unified_t wmi_handle,
				struct suspend_params *param,
				uint8_t mac_id)
{
	wmi_pdev_suspend_cmd_fixed_param *cmd;
	wmi_buf_t wmibuf;
	uint32_t len = sizeof(*cmd);
	int32_t ret;

	/*
	 * send the comand to Target to ignore the
	 * PCIE reset so as to ensure that Host and target
	 * states are in sync
	 */
	wmibuf = wmi_buf_alloc(wmi_handle, len);
	if (wmibuf == NULL)
		return -ENOMEM;

	cmd = (wmi_pdev_suspend_cmd_fixed_param *) wmi_buf_data(wmibuf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_pdev_suspend_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_suspend_cmd_fixed_param));
	if (param->disable_target_intr)
		cmd->suspend_opt = WMI_PDEV_SUSPEND_AND_DISABLE_INTR;
	else
		cmd->suspend_opt = WMI_PDEV_SUSPEND;
	ret = wmi_unified_cmd_send(wmi_handle, wmibuf, len,
				 WMI_PDEV_SUSPEND_CMDID);
	if (ret) {
		qdf_nbuf_free(wmibuf);
		WMA_LOGE("Failed to send WMI_PDEV_SUSPEND_CMDID command");
	}

	return ret;
}

/**
 * send_resume_cmd_tlv() - WMI resume function
 * @param wmi_handle      : handle to WMI.
 * @mac_id: radio context
 *
 * Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_resume_cmd_tlv(wmi_unified_t wmi_handle,
				uint8_t mac_id)
{
	wmi_buf_t wmibuf;
	wmi_pdev_resume_cmd_fixed_param *cmd;
	int32_t ret;

	wmibuf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (wmibuf == NULL)
		return -ENOMEM;
	cmd = (wmi_pdev_resume_cmd_fixed_param *) wmi_buf_data(wmibuf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_pdev_resume_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_resume_cmd_fixed_param));
	cmd->reserved0 = 0;
	ret = wmi_unified_cmd_send(wmi_handle, wmibuf, sizeof(*cmd),
				   WMI_PDEV_RESUME_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send WMI_PDEV_RESUME_CMDID command");
		wmi_buf_free(wmibuf);
	}

	return ret;
}

/**
 *  send_wow_enable_cmd_tlv() - WMI wow enable function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold wow enable parameter
 *  @mac_id: radio context
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_wow_enable_cmd_tlv(wmi_unified_t wmi_handle,
				struct wow_cmd_params *param,
				uint8_t mac_id)
{
	wmi_wow_enable_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len;
	int32_t ret;

	len = sizeof(wmi_wow_enable_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_wow_enable_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_wow_enable_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_wow_enable_cmd_fixed_param));
	cmd->enable = param->enable;
	if (param->can_suspend_link)
		cmd->pause_iface_config = WOW_IFACE_PAUSE_ENABLED;
	else
		cmd->pause_iface_config = WOW_IFACE_PAUSE_DISABLED;

	WMI_LOGI("suspend type: %s",
		cmd->pause_iface_config == WOW_IFACE_PAUSE_ENABLED ?
		"WOW_IFACE_PAUSE_ENABLED" : "WOW_IFACE_PAUSE_DISABLED");

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_WOW_ENABLE_CMDID);
	if (ret)
		wmi_buf_free(buf);

	return ret;
}

/**
 * send_set_ap_ps_param_cmd_tlv() - set ap powersave parameters
 * @wmi_handle: wmi handle
 * @peer_addr: peer mac address
 * @param: pointer to ap_ps parameter structure
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_set_ap_ps_param_cmd_tlv(wmi_unified_t wmi_handle,
					   uint8_t *peer_addr,
					   struct ap_ps_params *param)
{
	wmi_ap_ps_peer_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t err;

	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send set_ap_ps_param cmd");
		return -ENOMEM;
	}
	cmd = (wmi_ap_ps_peer_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_ap_ps_peer_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_ap_ps_peer_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(peer_addr, &cmd->peer_macaddr);
	cmd->param = param->param;
	cmd->value = param->value;
	err = wmi_unified_cmd_send(wmi_handle, buf,
				   sizeof(*cmd), WMI_AP_PS_PEER_PARAM_CMDID);
	if (err) {
		WMI_LOGE("Failed to send set_ap_ps_param cmd");
		qdf_mem_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_set_sta_ps_param_cmd_tlv() - set sta powersave parameters
 * @wmi_handle: wmi handle
 * @peer_addr: peer mac address
 * @param: pointer to sta_ps parameter structure
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_set_sta_ps_param_cmd_tlv(wmi_unified_t wmi_handle,
					   struct sta_ps_params *param)
{
	wmi_sta_powersave_param_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: Set Sta Ps param Mem Alloc Failed", __func__);
		return -ENOMEM;
	}

	cmd = (wmi_sta_powersave_param_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_sta_powersave_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_sta_powersave_param_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	cmd->param = param->param;
	cmd->value = param->value;

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_STA_POWERSAVE_PARAM_CMDID)) {
		WMI_LOGE("Set Sta Ps param Failed vdevId %d Param %d val %d",
			 param->vdev_id, param->param, param->value);
		qdf_nbuf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_crash_inject_cmd_tlv() - inject fw crash
 * @wmi_handle: wmi handle
 * @param: ponirt to crash inject paramter structure
 *
 * Return: 0 for success or return error
 */
QDF_STATUS send_crash_inject_cmd_tlv(wmi_unified_t wmi_handle,
			 struct crash_inject *param)
{
	int32_t ret = 0;
	WMI_FORCE_FW_HANG_CMD_fixed_param *cmd;
	uint16_t len = sizeof(*cmd);
	wmi_buf_t buf;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed!", __func__);
		return -ENOMEM;
	}

	cmd = (WMI_FORCE_FW_HANG_CMD_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_FORCE_FW_HANG_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_FORCE_FW_HANG_CMD_fixed_param));
	cmd->type = param->type;
	cmd->delay_time_ms = param->delay_time_ms;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
		WMI_FORCE_FW_HANG_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send set param command, ret = %d",
			 __func__, ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 *  send_dbglog_cmd_tlv() - set debug log level
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold dbglog level parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS
send_dbglog_cmd_tlv(wmi_unified_t wmi_handle,
				struct dbglog_params *dbglog_param)
{
	wmi_buf_t buf;
	wmi_debug_log_config_cmd_fixed_param *configmsg;
	A_STATUS status = A_OK;
	int32_t i;
	int32_t len;
	int8_t *buf_ptr;
	int32_t *module_id_bitmap_array;     /* Used to fomr the second tlv */

	ASSERT(bitmap_len < MAX_MODULE_ID_BITMAP_WORDS);

	/* Allocate size for 2 tlvs - including tlv hdr space for second tlv */
	len = sizeof(wmi_debug_log_config_cmd_fixed_param) + WMI_TLV_HDR_SIZE +
	      (sizeof(int32_t) * MAX_MODULE_ID_BITMAP_WORDS);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (buf == NULL)
		return A_NO_MEMORY;

	configmsg =
		(wmi_debug_log_config_cmd_fixed_param *) (wmi_buf_data(buf));
	buf_ptr = (int8_t *) configmsg;
	WMITLV_SET_HDR(&configmsg->tlv_header,
		       WMITLV_TAG_STRUC_wmi_debug_log_config_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_debug_log_config_cmd_fixed_param));
	configmsg->dbg_log_param = dbglog_param->param;
	configmsg->value = dbglog_param->val;
	/* Filling in the data part of second tlv -- should
	 * follow first tlv _ WMI_TLV_HDR_SIZE */
	module_id_bitmap_array = (A_UINT32 *) (buf_ptr +
				       sizeof
				       (wmi_debug_log_config_cmd_fixed_param)
				       + WMI_TLV_HDR_SIZE);
	WMITLV_SET_HDR(buf_ptr + sizeof(wmi_debug_log_config_cmd_fixed_param),
		       WMITLV_TAG_ARRAY_UINT32,
		       sizeof(A_UINT32) * MAX_MODULE_ID_BITMAP_WORDS);
	if (dbglog_param->module_id_bitmap) {
		for (i = 0; i < dbglog_param->bitmap_len; ++i) {
			module_id_bitmap_array[i] =
					dbglog_param->module_id_bitmap[i];
		}
	}

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_DBGLOG_CFG_CMDID);

	if (status != A_OK)
		qdf_nbuf_free(buf);

	return status;
}

/**
 *  send_vdev_set_param_cmd_tlv() - WMI vdev set parameter function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold vdev set parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_vdev_set_param_cmd_tlv(wmi_unified_t wmi_handle,
				struct vdev_set_params *param)
{
	int32_t ret;
	wmi_vdev_set_param_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_vdev_set_param_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_set_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_set_param_cmd_fixed_param));
	cmd->vdev_id = param->if_id;
	cmd->param_id = param->param_id;
	cmd->param_value = param->param_value;
	WMI_LOGD("Setting vdev %d param = %x, value = %u",
		 param->if_id, param->param_id, param->param_value);
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_VDEV_SET_PARAM_CMDID);
	if (ret < 0) {
		WMI_LOGE("Failed to send set param command ret = %d", ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 *  send_stats_request_cmd_tlv() - WMI request stats function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold stats request parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_stats_request_cmd_tlv(wmi_unified_t wmi_handle,
				uint8_t macaddr[IEEE80211_ADDR_LEN],
				struct stats_request_params *param)
{
	int32_t ret;
	wmi_request_stats_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(wmi_request_stats_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return -QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_request_stats_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_request_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_request_stats_cmd_fixed_param));
	cmd->stats_id = param->stats_id;
	cmd->vdev_id = param->vdev_id;
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
					 WMI_REQUEST_STATS_CMDID);
	if (ret) {
		WMI_LOGE("Failed to send status request to fw =%d", ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 *  send_packet_log_enable_cmd_tlv() - WMI request stats function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold stats request parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_packet_log_enable_cmd_tlv(wmi_unified_t wmi_handle,
				uint8_t macaddr[IEEE80211_ADDR_LEN],
				struct packet_enable_params *param)
{
	return 0;
}

/**
 *  send_beacon_send_cmd_tlv() - WMI beacon send function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold beacon send cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_beacon_send_cmd_tlv(wmi_unified_t wmi_handle,
				struct beacon_params *param)
{
	int32_t ret;
	wmi_bcn_tmpl_cmd_fixed_param *cmd;
	wmi_bcn_prb_info *bcn_prb_info;
	wmi_buf_t wmi_buf;
	uint8_t *buf_ptr;
	uint32_t wmi_buf_len;

	wmi_buf_len = sizeof(wmi_bcn_tmpl_cmd_fixed_param) +
		      sizeof(wmi_bcn_prb_info) + WMI_TLV_HDR_SIZE +
		      param->tmpl_len_aligned;
	wmi_buf = wmi_buf_alloc(wmi_handle, wmi_buf_len);
	if (!wmi_buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_bcn_tmpl_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_bcn_tmpl_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_bcn_tmpl_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	cmd->tim_ie_offset = param->tim_ie_offset;
	cmd->buf_len = param->tmpl_len;
	buf_ptr += sizeof(wmi_bcn_tmpl_cmd_fixed_param);

	bcn_prb_info = (wmi_bcn_prb_info *) buf_ptr;
	WMITLV_SET_HDR(&bcn_prb_info->tlv_header,
		       WMITLV_TAG_STRUC_wmi_bcn_prb_info,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_bcn_prb_info));
	bcn_prb_info->caps = 0;
	bcn_prb_info->erp = 0;
	buf_ptr += sizeof(wmi_bcn_prb_info);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, param->tmpl_len_aligned);
	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, param->frm, param->tmpl_len);

	ret = wmi_unified_cmd_send(wmi_handle,
				   wmi_buf, wmi_buf_len, WMI_BCN_TMPL_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send bcn tmpl: %d", __func__, ret);
		wmi_buf_free(wmi_buf);
	}
	return 0;
}

/**
 *  send_peer_assoc_cmd_tlv() - WMI peer assoc function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to peer assoc parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_peer_assoc_cmd_tlv(wmi_unified_t wmi_handle,
				struct peer_assoc_params *param)
{
	wmi_peer_assoc_complete_cmd_fixed_param *cmd;
	wmi_vht_rate_set *mcs;
	wmi_buf_t buf;
	int32_t len;
	uint8_t *buf_ptr;
	int ret;

	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE +
	      (param->num_peer_legacy_rates * sizeof(uint8_t)) +
	      WMI_TLV_HDR_SIZE +
	      (param->num_peer_ht_rates * sizeof(uint8_t)) +
	      sizeof(wmi_vht_rate_set);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_peer_assoc_complete_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_assoc_complete_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_peer_assoc_complete_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	cmd->peer_new_assoc = param->peer_new_assoc;
	cmd->peer_associd = param->peer_associd;
	cmd->peer_flags = param->peer_flags;
	cmd->peer_rate_caps = param->peer_rate_caps;
	cmd->peer_caps = param->peer_caps;
	cmd->peer_listen_intval = param->peer_listen_intval;
	cmd->peer_ht_caps = param->peer_ht_caps;
	cmd->peer_max_mpdu = param->peer_max_mpdu;
	cmd->peer_mpdu_density = param->peer_mpdu_density;
	cmd->num_peer_legacy_rates = param->num_peer_legacy_rates;
	cmd->num_peer_ht_rates = param->num_peer_ht_rates;
	cmd->peer_vht_caps = param->peer_vht_caps;
	cmd->peer_phymode = param->peer_phymode;

	/* Update peer legacy rate information */
	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE,
				   param->num_peer_legacy_rates);
	buf_ptr += WMI_TLV_HDR_SIZE;
	cmd->num_peer_legacy_rates = param->num_peer_legacy_rates;
	qdf_mem_copy(buf_ptr, param->peer_legacy_rates.rates,
		     param->peer_legacy_rates.num_rates);

	/* Update peer HT rate information */
	buf_ptr += param->num_peer_legacy_rates;
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE,
			  param->num_peer_ht_rates);
	buf_ptr += WMI_TLV_HDR_SIZE;
	cmd->num_peer_ht_rates = param->num_peer_ht_rates;
	qdf_mem_copy(buf_ptr, param->peer_ht_rates.rates,
				 param->peer_ht_rates.num_rates);

	/* VHT Rates */
	buf_ptr += param->num_peer_ht_rates;
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_STRUC_wmi_vht_rate_set,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_vht_rate_set));

	cmd->peer_nss = param->peer_nss;
	mcs = (wmi_vht_rate_set *) buf_ptr;
	if (param->vht_capable) {
		mcs->rx_max_rate = param->rx_max_rate;
		mcs->rx_mcs_set = param->rx_mcs_set;
		mcs->tx_max_rate = param->tx_max_rate;
		mcs->tx_mcs_set = param->tx_mcs_set;
	}

	WMI_LOGD("%s: vdev_id %d associd %d peer_flags %x rate_caps %x "
		 "peer_caps %x listen_intval %d ht_caps %x max_mpdu %d "
		 "nss %d phymode %d peer_mpdu_density %d "
		 "cmd->peer_vht_caps %x", __func__,
		 cmd->vdev_id, cmd->peer_associd, cmd->peer_flags,
		 cmd->peer_rate_caps, cmd->peer_caps,
		 cmd->peer_listen_intval, cmd->peer_ht_caps,
		 cmd->peer_max_mpdu, cmd->peer_nss, cmd->peer_phymode,
		 cmd->peer_mpdu_density,
		 cmd->peer_vht_caps);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_PEER_ASSOC_CMDID);
	if (ret != EOK) {
		WMI_LOGP("%s: Failed to send peer assoc command ret = %d",
			 __func__, ret);
		qdf_nbuf_free(buf);
	}

	return ret;
}

/**
 *  send_scan_start_cmd_tlv() - WMI scan start function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold scan start cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_scan_start_cmd_tlv(wmi_unified_t wmi_handle,
				struct scan_start_params *params)
{
	int32_t ret = 0;
	int32_t i;
	wmi_buf_t wmi_buf;
	wmi_start_scan_cmd_fixed_param *cmd;
	uint8_t *buf_ptr;
	uint32_t *tmp_ptr;
	wmi_ssid *ssid = NULL;
	wmi_mac_addr *bssid;
	int len = sizeof(*cmd);

	/* Length TLV placeholder for array of uint32_t */
	len += WMI_TLV_HDR_SIZE;
	/* calculate the length of buffer required */
	if (params->num_chan)
		len += params->num_chan * sizeof(uint32_t);

	/* Length TLV placeholder for array of wmi_ssid structures */
	len += WMI_TLV_HDR_SIZE;
	if (params->num_ssids)
		len += params->num_ssids * sizeof(wmi_ssid);

	/* Length TLV placeholder for array of wmi_mac_addr structures */
	len += WMI_TLV_HDR_SIZE;
	len += sizeof(wmi_mac_addr);

	/* Length TLV placeholder for array of bytes */
	len += WMI_TLV_HDR_SIZE;
	if (params->ie_len)
		len += roundup(params->ie_len, sizeof(uint32_t));

	/* Allocate the memory */
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGP("%s: failed to allocate memory for start scan cmd",
			 __func__);
		return QDF_STATUS_E_FAILURE;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_start_scan_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_start_scan_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_start_scan_cmd_fixed_param));

	cmd->scan_id = params->scan_id;
	cmd->scan_req_id = params->scan_req_id;
	cmd->vdev_id = params->vdev_id;
	cmd->scan_priority = params->scan_priority;
	cmd->notify_scan_events = params->notify_scan_events;
	cmd->dwell_time_active = params->dwell_time_active;
	cmd->dwell_time_passive = params->dwell_time_passive;
	cmd->min_rest_time = params->min_rest_time;
	cmd->max_rest_time = params->max_rest_time;
	cmd->repeat_probe_time = params->repeat_probe_time;
	cmd->probe_spacing_time = params->probe_spacing_time;
	cmd->idle_time = params->idle_time;
	cmd->max_scan_time = params->max_scan_time;
	cmd->probe_delay = params->probe_delay;
	cmd->scan_ctrl_flags = params->scan_ctrl_flags;
	cmd->burst_duration = params->burst_duration;
	cmd->num_chan = params->num_chan;
	cmd->num_bssid = params->num_bssid;
	cmd->num_ssids = params->num_ssids;
	cmd->ie_len = params->ie_len;
	cmd->n_probes = params->n_probes;
	buf_ptr += sizeof(*cmd);
	tmp_ptr = (uint32_t *) (buf_ptr + WMI_TLV_HDR_SIZE);
	for (i = 0; i < params->num_chan; ++i)
		tmp_ptr[i] = params->chan_list[i];

	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_UINT32,
		       (params->num_chan * sizeof(uint32_t)));
	buf_ptr += WMI_TLV_HDR_SIZE + (params->num_chan * sizeof(uint32_t));
	if (params->num_ssids > WMI_SCAN_MAX_NUM_SSID) {
		WMI_LOGE("Invalid value for numSsid");
		goto error;
	}

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_FIXED_STRUC,
	       (params->num_ssids * sizeof(wmi_ssid)));

	if (params->num_ssids) {
		ssid = (wmi_ssid *) (buf_ptr + WMI_TLV_HDR_SIZE);
		for (i = 0; i < params->num_ssids; ++i) {
			ssid->ssid_len = params->ssid[i].length;
			qdf_mem_copy(ssid->ssid, params->ssid[i].mac_ssid,
				     params->ssid[i].length);
			ssid++;
		}
	}
	buf_ptr += WMI_TLV_HDR_SIZE + (params->num_ssids * sizeof(wmi_ssid));

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_FIXED_STRUC,
		       (params->num_bssid * sizeof(wmi_mac_addr)));
	bssid = (wmi_mac_addr *) (buf_ptr + WMI_TLV_HDR_SIZE);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(params->mac_add_bytes, bssid);
	buf_ptr += WMI_TLV_HDR_SIZE + (params->num_bssid * sizeof(wmi_mac_addr));

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, params->ie_len_with_pad);
	if (params->ie_len) {
		qdf_mem_copy(buf_ptr + WMI_TLV_HDR_SIZE,
			     (uint8_t *) params->ie_base +
			     (params->uie_fieldOffset), params->ie_len);
	}
	buf_ptr += WMI_TLV_HDR_SIZE + params->ie_len_with_pad;

	ret = wmi_unified_cmd_send(wmi_handle, wmi_buf,
				      len, WMI_START_SCAN_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to start scan: %d", __func__, ret);
		wmi_buf_free(wmi_buf);
	}
	return ret;
error:
	qdf_nbuf_free(wmi_buf);
	return QDF_STATUS_E_FAILURE;
}

/**
 *  send_scan_stop_cmd_tlv() - WMI scan start function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold scan start cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_scan_stop_cmd_tlv(wmi_unified_t wmi_handle,
				struct scan_stop_params *param)
{
	wmi_stop_scan_cmd_fixed_param *cmd;
	int ret;
	int len = sizeof(*cmd);
	wmi_buf_t wmi_buf;

	/* Allocate the memory */
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGP("%s: failed to allocate memory for stop scan cmd",
			 __func__);
		ret = -ENOMEM;
		goto error;
	}

	cmd = (wmi_stop_scan_cmd_fixed_param *) wmi_buf_data(wmi_buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_stop_scan_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_stop_scan_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	cmd->requestor = param->requestor;
	cmd->scan_id = param->scan_id;
	/* stop the scan with the corresponding scan_id */
	cmd->req_type = param->req_type;
	ret = wmi_unified_cmd_send(wmi_handle, wmi_buf,
				      len, WMI_STOP_SCAN_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send stop scan: %d", __func__, ret);
		wmi_buf_free(wmi_buf);
	}

error:
	return ret;
}

/**
 *  send_scan_chan_list_cmd_tlv() - WMI scan channel list function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold scan channel list parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_scan_chan_list_cmd_tlv(wmi_unified_t wmi_handle,
				struct scan_chan_list_params *chan_list)
{
	wmi_buf_t buf;
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_scan_chan_list_cmd_fixed_param *cmd;
	int status, i;
	uint8_t *buf_ptr;
	wmi_channel *chan_info, *tchan_info;
	uint16_t len = sizeof(*cmd) + WMI_TLV_HDR_SIZE;

	len += sizeof(wmi_channel) * chan_list->num_scan_chans;
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("Failed to allocate memory");
		qdf_status = QDF_STATUS_E_NOMEM;
		goto end;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_scan_chan_list_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_scan_chan_list_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_scan_chan_list_cmd_fixed_param));

	WMI_LOGD("no of channels = %d, len = %d", chan_list->num_scan_chans, len);

	cmd->num_scan_chans = chan_list->num_scan_chans;
	WMITLV_SET_HDR((buf_ptr + sizeof(wmi_scan_chan_list_cmd_fixed_param)),
		       WMITLV_TAG_ARRAY_STRUC,
		       sizeof(wmi_channel) * chan_list->num_scan_chans);
	chan_info = (wmi_channel *) (buf_ptr + sizeof(*cmd) + WMI_TLV_HDR_SIZE);
	tchan_info = chan_list->chan_info;

	for (i = 0; i < chan_list->num_scan_chans; ++i) {
		WMITLV_SET_HDR(&chan_info->tlv_header,
			       WMITLV_TAG_STRUC_wmi_channel,
			       WMITLV_GET_STRUCT_TLVLEN(wmi_channel));
		chan_info->mhz = tchan_info->mhz;
		chan_info->band_center_freq1 =
				 tchan_info->band_center_freq1;
		chan_info->band_center_freq2 =
				tchan_info->band_center_freq2;
		chan_info->info = tchan_info->info;
		chan_info->reg_info_1 = tchan_info->reg_info_1;
		chan_info->reg_info_2 = tchan_info->reg_info_2;
		WMI_LOGD("chan[%d] = %u", i, chan_info->mhz);

		/*TODO: Set WMI_SET_CHANNEL_MIN_POWER */
		/*TODO: Set WMI_SET_CHANNEL_ANTENNA_MAX */
		/*TODO: WMI_SET_CHANNEL_REG_CLASSID */
		tchan_info++;
		chan_info++;
	}

	status = wmi_unified_cmd_send(wmi_handle, buf, len,
				      WMI_SCAN_CHAN_LIST_CMDID);

	if (status != EOK) {
		qdf_status = QDF_STATUS_E_FAILURE;
		WMI_LOGE("Failed to send WMI_SCAN_CHAN_LIST_CMDID");
		wmi_buf_free(buf);
	}
end:
	return qdf_status;
}

/**
 *  send_mgmt_cmd_tlv() - WMI scan start function
 *  @wmi_handle      : handle to WMI.
 *  @param    : pointer to hold mgmt cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
QDF_STATUS send_mgmt_cmd_tlv(wmi_unified_t wmi_handle,
				struct wmi_mgmt_params *param)
{
	wmi_buf_t buf;
	wmi_mgmt_tx_send_cmd_fixed_param *cmd;
	int32_t cmd_len;
	uint64_t dma_addr;
	struct wmi_desc_t *wmi_desc = NULL;
	void *qdf_ctx = param->qdf_ctx;
	uint8_t *bufp;
	int32_t bufp_len = (param->frm_len < mgmt_tx_dl_frm_len) ? param->frm_len :
		mgmt_tx_dl_frm_len;

	cmd_len = sizeof(wmi_mgmt_tx_send_cmd_fixed_param) +
		WMI_TLV_HDR_SIZE + roundup(bufp_len, sizeof(uint32_t));

	buf = wmi_buf_alloc(wmi_handle, cmd_len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_mgmt_tx_send_cmd_fixed_param *)wmi_buf_data(buf);
	bufp = (uint8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_mgmt_tx_send_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN
		(wmi_mgmt_tx_send_cmd_fixed_param));

	cmd->vdev_id = param->vdev_id;

	wmi_desc = param->wmi_desc;
	if (!wmi_desc) {
		WMI_LOGE("%s: Failed to get wmi_desc", __func__);
		goto err1;
	}
	wmi_desc->nbuf = param->tx_frame;
	wmi_desc->tx_cmpl_cb = param->tx_complete_cb;
	wmi_desc->ota_post_proc_cb = param->tx_ota_post_proc_cb;

	cmd->desc_id = wmi_desc->desc_id;
	cmd->chanfreq = param->chanfreq;
	bufp += sizeof(wmi_mgmt_tx_send_cmd_fixed_param);
	WMITLV_SET_HDR(bufp, WMITLV_TAG_ARRAY_BYTE, roundup(bufp_len,
							    sizeof(uint32_t)));
	bufp += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(bufp, param->pdata, bufp_len);
	qdf_nbuf_map_single(qdf_ctx, param->tx_frame, QDF_DMA_TO_DEVICE);
	dma_addr = qdf_nbuf_get_frag_paddr(param->tx_frame, 0);
	cmd->paddr_lo = (uint32_t)(dma_addr & 0xffffffff);
#if defined(HELIUMPLUS_PADDR64)
	cmd->paddr_hi = (uint32_t)((dma_addr >> 32) & 0x1F);
#endif
	cmd->frame_len = param->frm_len;
	cmd->buf_len = bufp_len;

	if (wmi_unified_cmd_send(wmi_handle, buf, cmd_len,
				      WMI_MGMT_TX_SEND_CMDID)) {
		WMI_LOGE("%s: Failed to send mgmt Tx", __func__);
		goto err1;
	}
	return QDF_STATUS_SUCCESS;

err1:
	wmi_buf_free(buf);
	return QDF_STATUS_E_FAILURE;
}

/**
 * send_modem_power_state_cmd_tlv() - set modem power state to fw
 * @wmi_handle: wmi handle
 * @param_value: parameter value
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_modem_power_state_cmd_tlv(wmi_unified_t wmi_handle,
		uint32_t param_value)
{
	int ret;
	wmi_modem_power_state_cmd_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_modem_power_state_cmd_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_modem_power_state_cmd_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_modem_power_state_cmd_param));
	cmd->modem_power_state = param_value;
	WMI_LOGD("%s: Setting cmd->modem_power_state = %u", __func__,
		 param_value);
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				     WMI_MODEM_POWER_STATE_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send notify cmd ret = %d", ret);
		wmi_buf_free(buf);
	}
	return ret;
}

/**
 * send_set_sta_ps_mode_cmd_tlv() - set sta powersave mode in fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @val: value
 *
 * Return: 0 for success or error code.
 */
QDF_STATUS send_set_sta_ps_mode_cmd_tlv(wmi_unified_t wmi_handle,
			       uint32_t vdev_id, uint8_t val)
{
	wmi_sta_powersave_mode_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	WMI_LOGD("Set Sta Mode Ps vdevId %d val %d", vdev_id, val);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: Set Sta Mode Ps Mem Alloc Failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_sta_powersave_mode_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_sta_powersave_mode_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_sta_powersave_mode_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	if (val)
		cmd->sta_ps_mode = WMI_STA_PS_MODE_ENABLED;
	else
		cmd->sta_ps_mode = WMI_STA_PS_MODE_DISABLED;

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_STA_POWERSAVE_MODE_CMDID)) {
		WMI_LOGE("Set Sta Mode Ps Failed vdevId %d val %d",
			 vdev_id, val);
		qdf_nbuf_free(buf);
		return -EIO;
	}
	return 0;
}

/**
 * send_set_mimops_cmd_tlv() - set MIMO powersave
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @value: value
 *
 * Return: QDF_STATUS_SUCCESS for success or error code.
 */
QDF_STATUS send_set_mimops_cmd_tlv(wmi_unified_t wmi_handle,
			uint8_t vdev_id, int value)
{
	int ret = QDF_STATUS_SUCCESS;
	wmi_sta_smps_force_mode_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_sta_smps_force_mode_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_sta_smps_force_mode_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_sta_smps_force_mode_cmd_fixed_param));

	cmd->vdev_id = vdev_id;

	switch (value) {
	case 0:
		cmd->forced_mode = WMI_SMPS_FORCED_MODE_NONE;
		break;
	case 1:
		cmd->forced_mode = WMI_SMPS_FORCED_MODE_DISABLED;
		break;
	case 2:
		cmd->forced_mode = WMI_SMPS_FORCED_MODE_STATIC;
		break;
	case 3:
		cmd->forced_mode = WMI_SMPS_FORCED_MODE_DYNAMIC;
		break;
	default:
		WMI_LOGE("%s:INVALID Mimo PS CONFIG", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGD("Setting vdev %d value = %u", vdev_id, value);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_STA_SMPS_FORCE_MODE_CMDID);
	if (ret < 0) {
		WMI_LOGE("Failed to send set Mimo PS ret = %d", ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 * send_set_smps_params_cmd_tlv() - set smps params
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @value: value
 *
 * Return: QDF_STATUS_SUCCESS for success or error code.
 */
QDF_STATUS send_set_smps_params_cmd_tlv(wmi_unified_t wmi_handle, uint8_t vdev_id,
			       int value)
{
	int ret = QDF_STATUS_SUCCESS;
	wmi_sta_smps_param_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_sta_smps_param_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_sta_smps_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_sta_smps_param_cmd_fixed_param));

	cmd->vdev_id = vdev_id;
	cmd->value = value & WMI_SMPS_MASK_LOWER_16BITS;
	cmd->param =
		(value >> WMI_SMPS_PARAM_VALUE_S) & WMI_SMPS_MASK_UPPER_3BITS;

	WMI_LOGD("Setting vdev %d value = %x param %x", vdev_id, cmd->value,
		 cmd->param);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_STA_SMPS_PARAM_CMDID);
	if (ret < 0) {
		WMI_LOGE("Failed to send set Mimo PS ret = %d", ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 * send_set_p2pgo_noa_req_cmd_tlv() - send p2p go noa request to fw
 * @wmi_handle: wmi handle
 * @noa: p2p power save parameters
 *
 * Return: CDF status
 */
QDF_STATUS send_set_p2pgo_noa_req_cmd_tlv(wmi_unified_t wmi_handle,
			struct p2p_ps_params *noa)
{
	wmi_p2p_set_noa_cmd_fixed_param *cmd;
	wmi_p2p_noa_descriptor *noa_discriptor;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint16_t len;
	int32_t status;
	uint32_t duration;

	WMI_LOGD("%s: Enter", __func__);
	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE + sizeof(*noa_discriptor);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("Failed to allocate memory");
		status = QDF_STATUS_E_FAILURE;
		goto end;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_p2p_set_noa_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_p2p_set_noa_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_p2p_set_noa_cmd_fixed_param));
	duration = (noa->count == 1) ? noa->single_noa_duration : noa->duration;
	cmd->vdev_id = noa->session_id;
	cmd->enable = (duration) ? true : false;
	cmd->num_noa = 1;

	WMITLV_SET_HDR((buf_ptr + sizeof(wmi_p2p_set_noa_cmd_fixed_param)),
		       WMITLV_TAG_ARRAY_STRUC, sizeof(wmi_p2p_noa_descriptor));
	noa_discriptor = (wmi_p2p_noa_descriptor *) (buf_ptr +
						     sizeof
						     (wmi_p2p_set_noa_cmd_fixed_param)
						     + WMI_TLV_HDR_SIZE);
	WMITLV_SET_HDR(&noa_discriptor->tlv_header,
		       WMITLV_TAG_STRUC_wmi_p2p_noa_descriptor,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_p2p_noa_descriptor));
	noa_discriptor->type_count = noa->count;
	noa_discriptor->duration = duration;
	noa_discriptor->interval = noa->interval;
	noa_discriptor->start_time = 0;

	WMI_LOGI("SET P2P GO NOA:vdev_id:%d count:%d duration:%d interval:%d",
		 cmd->vdev_id, noa->count, noa_discriptor->duration,
		 noa->interval);
	status = wmi_unified_cmd_send(wmi_handle, buf, len,
				      WMI_FWTEST_P2P_SET_NOA_PARAM_CMDID);
	if (status != EOK) {
		WMI_LOGE("Failed to send WMI_FWTEST_P2P_SET_NOA_PARAM_CMDID");
		wmi_buf_free(buf);
	}

end:
	WMI_LOGD("%s: Exit", __func__);
	return status;
}


/**
 * send_set_p2pgo_oppps_req_cmd_tlv() - send p2p go opp power save request to fw
 * @wmi_handle: wmi handle
 * @noa: p2p opp power save parameters
 *
 * Return: CDF status
 */
QDF_STATUS send_set_p2pgo_oppps_req_cmd_tlv(wmi_unified_t wmi_handle,
		struct p2p_ps_params *oppps)
{
	wmi_p2p_set_oppps_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t status;

	WMI_LOGD("%s: Enter", __func__);
	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate memory");
		status = QDF_STATUS_E_FAILURE;
		goto end;
	}

	cmd = (wmi_p2p_set_oppps_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_p2p_set_oppps_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_p2p_set_oppps_cmd_fixed_param));
	cmd->vdev_id = oppps->session_id;
	if (oppps->ctwindow)
		WMI_UNIFIED_OPPPS_ATTR_ENABLED_SET(cmd);

	WMI_UNIFIED_OPPPS_ATTR_CTWIN_SET(cmd, oppps->ctwindow);
	WMI_LOGI("SET P2P GO OPPPS:vdev_id:%d ctwindow:%d",
		 cmd->vdev_id, oppps->ctwindow);
	status = wmi_unified_cmd_send(wmi_handle, buf, sizeof(*cmd),
				      WMI_P2P_SET_OPPPS_PARAM_CMDID);
	if (status != EOK) {
		WMI_LOGE("Failed to send WMI_P2P_SET_OPPPS_PARAM_CMDID");
		wmi_buf_free(buf);
	}

end:
	WMI_LOGD("%s: Exit", __func__);
	return status;
}

/**
 * send_get_temperature_cmd_tlv() - get pdev temperature req
 * @wmi_handle: wmi handle
 *
 * Return: QDF_STATUS_SUCCESS for success or error code.
 */
QDF_STATUS send_get_temperature_cmd_tlv(wmi_unified_t wmi_handle)
{
	wmi_pdev_get_temperature_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len = sizeof(wmi_pdev_get_temperature_cmd_fixed_param);
	uint8_t *buf_ptr;

	if (!wmi_handle) {
		WMI_LOGE(FL("WMI is closed, can not issue cmd"));
		return QDF_STATUS_E_INVAL;
	}

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_pdev_get_temperature_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_pdev_get_temperature_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_get_temperature_cmd_fixed_param));

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_PDEV_GET_TEMPERATURE_CMDID)) {
		WMI_LOGE(FL("failed to send get temperature command"));
		wmi_buf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_sta_uapsd_auto_trig_cmd_tlv() - set uapsd auto trigger command
 * @wmi_handle: wmi handle
 * @vdevid: vdev id
 * @peer_addr: peer mac address
 * @auto_triggerparam: auto trigger parameters
 * @num_ac: number of access category
 *
 * This function sets the trigger
 * uapsd params such as service interval, delay interval
 * and suspend interval which will be used by the firmware
 * to send trigger frames periodically when there is no
 * traffic on the transmit side.
 *
 * Return: 0 for success or error code.
 */
QDF_STATUS send_set_sta_uapsd_auto_trig_cmd_tlv(wmi_unified_t wmi_handle,
				struct sta_uapsd_trig_params *param)
{
	wmi_sta_uapsd_auto_trig_cmd_fixed_param *cmd;
	int32_t ret;
	uint32_t param_len = param->num_ac * sizeof(wmi_sta_uapsd_auto_trig_param);
	uint32_t cmd_len = sizeof(*cmd) + param_len + WMI_TLV_HDR_SIZE;
	uint32_t i;
	wmi_buf_t buf;
	uint8_t *buf_ptr;

	buf = wmi_buf_alloc(wmi_handle, cmd_len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_sta_uapsd_auto_trig_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_sta_uapsd_auto_trig_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_sta_uapsd_auto_trig_cmd_fixed_param));
	cmd->vdev_id = param->vdevid;
	cmd->num_ac = param->num_ac;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(param->peer_addr, &cmd->peer_macaddr);

	/* TLV indicating array of structures to follow */
	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, param_len);

	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, param->auto_triggerparam, param_len);

	/*
	 * Update tag and length for uapsd auto trigger params (this will take
	 * care of updating tag and length if it is not pre-filled by caller).
	 */
	for (i = 0; i < param->num_ac; i++) {
		WMITLV_SET_HDR((buf_ptr +
				(i * sizeof(wmi_sta_uapsd_auto_trig_param))),
			       WMITLV_TAG_STRUC_wmi_sta_uapsd_auto_trig_param,
			       WMITLV_GET_STRUCT_TLVLEN
				       (wmi_sta_uapsd_auto_trig_param));
	}

	ret = wmi_unified_cmd_send(wmi_handle, buf, cmd_len,
				   WMI_STA_UAPSD_AUTO_TRIG_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send set uapsd param ret = %d", ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 * send_ocb_set_utc_time_cmd() - send the UTC time to the firmware
 * @wmi_handle: pointer to the wmi handle
 * @utc: pointer to the UTC time struct
 *
 * Return: 0 on succes
 */
QDF_STATUS send_ocb_set_utc_time_cmd_tlv(wmi_unified_t wmi_handle,
				struct ocb_utc_param *utc)
{
	int32_t ret;
	wmi_ocb_set_utc_time_cmd_fixed_param *cmd;
	uint8_t *buf_ptr;
	uint32_t len, i;
	wmi_buf_t buf;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *)wmi_buf_data(buf);
	cmd = (wmi_ocb_set_utc_time_cmd_fixed_param *)buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_ocb_set_utc_time_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(wmi_ocb_set_utc_time_cmd_fixed_param));
	cmd->vdev_id = utc->vdev_id;

	for (i = 0; i < SIZE_UTC_TIME; i++)
		WMI_UTC_TIME_SET(cmd, i, utc->utc_time[i]);

	for (i = 0; i < SIZE_UTC_TIME_ERROR; i++)
		WMI_TIME_ERROR_SET(cmd, i, utc->time_error[i]);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_OCB_SET_UTC_TIME_CMDID);
	if (ret != EOK) {
		WMI_LOGE(FL("Failed to set OCB UTC time"));
		wmi_buf_free(buf);
		return -EIO;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_ocb_start_timing_advert_cmd_tlv() - start sending the timing advertisement
 *				   frames on a channel
 * @wmi_handle: pointer to the wmi handle
 * @timing_advert: pointer to the timing advertisement struct
 *
 * Return: 0 on succes
 */
QDF_STATUS send_ocb_start_timing_advert_cmd_tlv(wmi_unified_t wmi_handle,
	struct ocb_timing_advert_param *timing_advert)
{
	int32_t ret;
	wmi_ocb_start_timing_advert_cmd_fixed_param *cmd;
	uint8_t *buf_ptr;
	uint32_t len, len_template;
	wmi_buf_t buf;

	len = sizeof(*cmd) +
		     WMI_TLV_HDR_SIZE;

	len_template = timing_advert->template_length;
	/* Add padding to the template if needed */
	if (len_template % 4 != 0)
		len_template += 4 - (len_template % 4);
	len += len_template;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *)wmi_buf_data(buf);
	cmd = (wmi_ocb_start_timing_advert_cmd_fixed_param *)buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_ocb_start_timing_advert_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_ocb_start_timing_advert_cmd_fixed_param));
	cmd->vdev_id = timing_advert->vdev_id;
	cmd->repeat_rate = timing_advert->repeat_rate;
	cmd->channel_freq = timing_advert->chan_freq;
	cmd->timestamp_offset = timing_advert->timestamp_offset;
	cmd->time_value_offset = timing_advert->time_value_offset;
	cmd->timing_advert_template_length = timing_advert->template_length;
	buf_ptr += sizeof(*cmd);

	/* Add the timing advert template */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE,
		       len_template);
	qdf_mem_copy(buf_ptr + WMI_TLV_HDR_SIZE,
		     (uint8_t *)timing_advert->template_value,
		     timing_advert->template_length);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_OCB_START_TIMING_ADVERT_CMDID);
	if (ret != EOK) {
		WMI_LOGE(FL("Failed to start OCB timing advert"));
		wmi_buf_free(buf);
		return -EIO;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_ocb_stop_timing_advert_cmd_tlv() - stop sending the timing advertisement frames
 *				  on a channel
 * @wmi_handle: pointer to the wmi handle
 * @timing_advert: pointer to the timing advertisement struct
 *
 * Return: 0 on succes
 */
QDF_STATUS send_ocb_stop_timing_advert_cmd_tlv(wmi_unified_t wmi_handle,
	struct ocb_timing_advert_param *timing_advert)
{
	int32_t ret;
	wmi_ocb_stop_timing_advert_cmd_fixed_param *cmd;
	uint8_t *buf_ptr;
	uint32_t len;
	wmi_buf_t buf;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *)wmi_buf_data(buf);
	cmd = (wmi_ocb_stop_timing_advert_cmd_fixed_param *)buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_ocb_stop_timing_advert_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_ocb_stop_timing_advert_cmd_fixed_param));
	cmd->vdev_id = timing_advert->vdev_id;
	cmd->channel_freq = timing_advert->chan_freq;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_OCB_STOP_TIMING_ADVERT_CMDID);
	if (ret != EOK) {
		WMI_LOGE(FL("Failed to stop OCB timing advert"));
		wmi_buf_free(buf);
		return -EIO;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_ocb_get_tsf_timer_cmd_tlv() - get ocb tsf timer val
 * @wmi_handle: pointer to the wmi handle
 * @request: pointer to the request
 *
 * Return: 0 on succes
 */
QDF_STATUS send_ocb_get_tsf_timer_cmd_tlv(wmi_unified_t wmi_handle,
			  uint8_t vdev_id)
{
	QDF_STATUS ret;
	wmi_ocb_get_tsf_timer_cmd_fixed_param *cmd;
	uint8_t *buf_ptr;
	wmi_buf_t buf;
	int32_t len;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}
	buf_ptr = (uint8_t *)wmi_buf_data(buf);

	cmd = (wmi_ocb_get_tsf_timer_cmd_fixed_param *)buf_ptr;
	qdf_mem_zero(cmd, len);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_ocb_get_tsf_timer_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_ocb_get_tsf_timer_cmd_fixed_param));
	cmd->vdev_id = vdev_id;

	/* Send the WMI command */
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_OCB_GET_TSF_TIMER_CMDID);
	/* If there is an error, set the completion event */
	if (ret != EOK) {
		WMI_LOGE(FL("Failed to send WMI message: %d"), ret);
		wmi_buf_free(buf);
		return -EIO;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_dcc_get_stats_cmd_tlv() - get the DCC channel stats
 * @wmi_handle: pointer to the wmi handle
 * @get_stats_param: pointer to the dcc stats
 *
 * Return: 0 on succes
 */
QDF_STATUS send_dcc_get_stats_cmd_tlv(wmi_unified_t wmi_handle,
		     struct dcc_get_stats_param *get_stats_param)
{
	int32_t ret;
	wmi_dcc_get_stats_cmd_fixed_param *cmd;
	wmi_dcc_channel_stats_request *channel_stats_array;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t len;
	uint32_t i;

	/* Validate the input */
	if (get_stats_param->request_array_len !=
	    get_stats_param->channel_count * sizeof(*channel_stats_array)) {
		WMI_LOGE(FL("Invalid parameter"));
		return -EINVAL;
	}

	/* Allocate memory for the WMI command */
	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE +
		get_stats_param->request_array_len;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);

	/* Populate the WMI command */
	cmd = (wmi_dcc_get_stats_cmd_fixed_param *)buf_ptr;
	buf_ptr += sizeof(*cmd);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_dcc_get_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
			   wmi_dcc_get_stats_cmd_fixed_param));
	cmd->vdev_id = get_stats_param->vdev_id;
	cmd->num_channels = get_stats_param->channel_count;

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       get_stats_param->request_array_len);
	buf_ptr += WMI_TLV_HDR_SIZE;

	channel_stats_array = (wmi_dcc_channel_stats_request *)buf_ptr;
	qdf_mem_copy(channel_stats_array, get_stats_param->request_array,
		     get_stats_param->request_array_len);
	for (i = 0; i < cmd->num_channels; i++)
		WMITLV_SET_HDR(&channel_stats_array[i].tlv_header,
			WMITLV_TAG_STRUC_wmi_dcc_channel_stats_request,
			WMITLV_GET_STRUCT_TLVLEN(
			    wmi_dcc_channel_stats_request));

	/* Send the WMI command */
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_DCC_GET_STATS_CMDID);

	if (ret != EOK) {
		WMI_LOGE(FL("Failed to send WMI message: %d"), ret);
		wmi_buf_free(buf);
		return -EIO;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_dcc_clear_stats_cmd_tlv() - command to clear the DCC stats
 * @wmi_handle: pointer to the wmi handle
 * @vdev_id: vdev id
 * @dcc_stats_bitmap: dcc status bitmap
 *
 * Return: 0 on succes
 */
QDF_STATUS send_dcc_clear_stats_cmd_tlv(wmi_unified_t wmi_handle,
				uint32_t vdev_id, uint32_t dcc_stats_bitmap)
{
	int32_t ret;
	wmi_dcc_clear_stats_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t len;

	/* Allocate memory for the WMI command */
	len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);

	/* Populate the WMI command */
	cmd = (wmi_dcc_clear_stats_cmd_fixed_param *)buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_dcc_clear_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
			   wmi_dcc_clear_stats_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->dcc_stats_bitmap = dcc_stats_bitmap;

	/* Send the WMI command */
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_DCC_CLEAR_STATS_CMDID);
	if (ret != EOK) {
		WMI_LOGE(FL("Failed to send the WMI command"));
		wmi_buf_free(buf);
		return -EIO;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_dcc_update_ndl_cmd_tlv() - command to update the NDL data
 * @wmi_handle: pointer to the wmi handle
 * @update_ndl_param: pointer to the request parameters
 *
 * Return: 0 on success
 */
QDF_STATUS send_dcc_update_ndl_cmd_tlv(wmi_unified_t wmi_handle,
		       struct dcc_update_ndl_param *update_ndl_param)
{
	QDF_STATUS qdf_status;
	wmi_dcc_update_ndl_cmd_fixed_param *cmd;
	wmi_dcc_ndl_chan *ndl_chan_array;
	wmi_dcc_ndl_active_state_config *ndl_active_state_array;
	uint32_t active_state_count;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t len;
	uint32_t i;

	/* validate the input */
	if (update_ndl_param->dcc_ndl_chan_list_len !=
	    update_ndl_param->channel_count * sizeof(*ndl_chan_array)) {
		WMI_LOGE(FL("Invalid parameter"));
		return QDF_STATUS_E_INVAL;
	}
	active_state_count = 0;
	ndl_chan_array = update_ndl_param->dcc_ndl_chan_list;
	for (i = 0; i < update_ndl_param->channel_count; i++)
		active_state_count +=
			WMI_NDL_NUM_ACTIVE_STATE_GET(&ndl_chan_array[i]);
	if (update_ndl_param->dcc_ndl_active_state_list_len !=
	    active_state_count * sizeof(*ndl_active_state_array)) {
		WMI_LOGE(FL("Invalid parameter"));
		return QDF_STATUS_E_INVAL;
	}

	/* Allocate memory for the WMI command */
	len = sizeof(*cmd) +
		WMI_TLV_HDR_SIZE + update_ndl_param->dcc_ndl_chan_list_len +
		WMI_TLV_HDR_SIZE +
		update_ndl_param->dcc_ndl_active_state_list_len;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);

	/* Populate the WMI command */
	cmd = (wmi_dcc_update_ndl_cmd_fixed_param *)buf_ptr;
	buf_ptr += sizeof(*cmd);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_dcc_update_ndl_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
			   wmi_dcc_update_ndl_cmd_fixed_param));
	cmd->vdev_id = update_ndl_param->vdev_id;
	cmd->num_channel = update_ndl_param->channel_count;

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       update_ndl_param->dcc_ndl_chan_list_len);
	buf_ptr += WMI_TLV_HDR_SIZE;

	ndl_chan_array = (wmi_dcc_ndl_chan *)buf_ptr;
	qdf_mem_copy(ndl_chan_array, update_ndl_param->dcc_ndl_chan_list,
		     update_ndl_param->dcc_ndl_chan_list_len);
	for (i = 0; i < cmd->num_channel; i++)
		WMITLV_SET_HDR(&ndl_chan_array[i].tlv_header,
			WMITLV_TAG_STRUC_wmi_dcc_ndl_chan,
			WMITLV_GET_STRUCT_TLVLEN(
			    wmi_dcc_ndl_chan));
	buf_ptr += update_ndl_param->dcc_ndl_chan_list_len;

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       update_ndl_param->dcc_ndl_active_state_list_len);
	buf_ptr += WMI_TLV_HDR_SIZE;

	ndl_active_state_array = (wmi_dcc_ndl_active_state_config *) buf_ptr;
	qdf_mem_copy(ndl_active_state_array,
		     update_ndl_param->dcc_ndl_active_state_list,
		     update_ndl_param->dcc_ndl_active_state_list_len);
	for (i = 0; i < active_state_count; i++) {
		WMITLV_SET_HDR(&ndl_active_state_array[i].tlv_header,
			WMITLV_TAG_STRUC_wmi_dcc_ndl_active_state_config,
			WMITLV_GET_STRUCT_TLVLEN(
			    wmi_dcc_ndl_active_state_config));
	}
	buf_ptr += update_ndl_param->dcc_ndl_active_state_list_len;

	/* Send the WMI command */
	qdf_status = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_DCC_UPDATE_NDL_CMDID);
	/* If there is an error, set the completion event */
	if (qdf_status) {
		WMI_LOGE(FL("Failed to send WMI message: %d"), qdf_status);
		wmi_buf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_ocb_set_config_cmd_tlv() - send the OCB config to the FW
 * @wmi_handle: pointer to the wmi handle
 * @config: the OCB configuration
 *
 * Return: 0 on success
 */
QDF_STATUS send_ocb_set_config_cmd_tlv(wmi_unified_t wmi_handle,
				struct ocb_config_param *config, uint32_t *ch_mhz)
{
	int32_t ret;
	wmi_ocb_set_config_cmd_fixed_param *cmd;
	wmi_channel *chan;
	wmi_ocb_channel *ocb_chan;
	wmi_qos_parameter *qos_param;
	wmi_dcc_ndl_chan *ndl_chan;
	wmi_dcc_ndl_active_state_config *ndl_active_config;
	wmi_ocb_schedule_element *sched_elem;
	uint8_t *buf_ptr;
	wmi_buf_t buf;
	int32_t len;
	int32_t i, j, active_state_count;

	/*
	 * Validate the dcc_ndl_chan_list_len and count the number of active
	 * states. Validate dcc_ndl_active_state_list_len.
	 */
	active_state_count = 0;
	if (config->dcc_ndl_chan_list_len) {
		if (!config->dcc_ndl_chan_list ||
			config->dcc_ndl_chan_list_len !=
			config->channel_count * sizeof(wmi_dcc_ndl_chan)) {
			WMI_LOGE(FL("NDL channel is invalid. List len: %d"),
				 config->dcc_ndl_chan_list_len);
			return -EINVAL;
		}

		for (i = 0, ndl_chan = config->dcc_ndl_chan_list;
				i < config->channel_count; ++i, ++ndl_chan)
			active_state_count +=
				WMI_NDL_NUM_ACTIVE_STATE_GET(ndl_chan);

		if (active_state_count) {
			if (!config->dcc_ndl_active_state_list ||
				config->dcc_ndl_active_state_list_len !=
				active_state_count *
				sizeof(wmi_dcc_ndl_active_state_config)) {
				WMI_LOGE(FL("NDL active state is invalid."));
				return -EINVAL;
			}
		}
	}

	len = sizeof(*cmd) +
		WMI_TLV_HDR_SIZE + config->channel_count *
			sizeof(wmi_channel) +
		WMI_TLV_HDR_SIZE + config->channel_count *
			sizeof(wmi_ocb_channel) +
		WMI_TLV_HDR_SIZE + config->channel_count *
			sizeof(wmi_qos_parameter) * WMI_MAX_NUM_AC +
		WMI_TLV_HDR_SIZE + config->dcc_ndl_chan_list_len +
		WMI_TLV_HDR_SIZE + active_state_count *
			sizeof(wmi_dcc_ndl_active_state_config) +
		WMI_TLV_HDR_SIZE + config->schedule_size *
			sizeof(wmi_ocb_schedule_element);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *)wmi_buf_data(buf);
	cmd = (wmi_ocb_set_config_cmd_fixed_param *)buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_ocb_set_config_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(wmi_ocb_set_config_cmd_fixed_param));
	cmd->vdev_id = config->session_id;
	cmd->channel_count = config->channel_count;
	cmd->schedule_size = config->schedule_size;
	cmd->flags = config->flags;
	buf_ptr += sizeof(*cmd);

	/* Add the wmi_channel info */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       config->channel_count*sizeof(wmi_channel));
	buf_ptr += WMI_TLV_HDR_SIZE;
	for (i = 0; i < config->channel_count; i++) {
		chan = (wmi_channel *)buf_ptr;
		WMITLV_SET_HDR(&chan->tlv_header,
				WMITLV_TAG_STRUC_wmi_channel,
				WMITLV_GET_STRUCT_TLVLEN(wmi_channel));
		chan->mhz = config->channels[i].chan_freq;
		chan->band_center_freq1 = config->channels[i].chan_freq;
		chan->band_center_freq2 = 0;
		chan->info = 0;

		WMI_SET_CHANNEL_MODE(chan, ch_mhz[i]);
		WMI_SET_CHANNEL_MAX_POWER(chan, config->channels[i].max_pwr);
		WMI_SET_CHANNEL_MIN_POWER(chan, config->channels[i].min_pwr);
		WMI_SET_CHANNEL_MAX_TX_POWER(chan, config->channels[i].max_pwr);
		WMI_SET_CHANNEL_REG_POWER(chan, config->channels[i].reg_pwr);
		WMI_SET_CHANNEL_ANTENNA_MAX(chan,
					    config->channels[i].antenna_max);

		if (config->channels[i].bandwidth < 10)
			WMI_SET_CHANNEL_FLAG(chan, WMI_CHAN_FLAG_QUARTER_RATE);
		else if (config->channels[i].bandwidth < 20)
			WMI_SET_CHANNEL_FLAG(chan, WMI_CHAN_FLAG_HALF_RATE);
		buf_ptr += sizeof(*chan);
	}

	/* Add the wmi_ocb_channel info */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       config->channel_count*sizeof(wmi_ocb_channel));
	buf_ptr += WMI_TLV_HDR_SIZE;
	for (i = 0; i < config->channel_count; i++) {
		ocb_chan = (wmi_ocb_channel *)buf_ptr;
		WMITLV_SET_HDR(&ocb_chan->tlv_header,
			       WMITLV_TAG_STRUC_wmi_ocb_channel,
			       WMITLV_GET_STRUCT_TLVLEN(wmi_ocb_channel));
		ocb_chan->bandwidth = config->channels[i].bandwidth;
		WMI_CHAR_ARRAY_TO_MAC_ADDR(
					config->channels[i].mac_address.bytes,
					&ocb_chan->mac_address);
		buf_ptr += sizeof(*ocb_chan);
	}

	/* Add the wmi_qos_parameter info */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		config->channel_count * sizeof(wmi_qos_parameter)*WMI_MAX_NUM_AC);
	buf_ptr += WMI_TLV_HDR_SIZE;
	/* WMI_MAX_NUM_AC parameters for each channel */
	for (i = 0; i < config->channel_count; i++) {
		for (j = 0; j < WMI_MAX_NUM_AC; j++) {
			qos_param = (wmi_qos_parameter *)buf_ptr;
			WMITLV_SET_HDR(&qos_param->tlv_header,
				WMITLV_TAG_STRUC_wmi_qos_parameter,
				WMITLV_GET_STRUCT_TLVLEN(wmi_qos_parameter));
			qos_param->aifsn =
				config->channels[i].qos_params[j].aifsn;
			qos_param->cwmin =
				config->channels[i].qos_params[j].cwmin;
			qos_param->cwmax =
				config->channels[i].qos_params[j].cwmax;
			buf_ptr += sizeof(*qos_param);
		}
	}

	/* Add the wmi_dcc_ndl_chan (per channel) */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       config->dcc_ndl_chan_list_len);
	buf_ptr += WMI_TLV_HDR_SIZE;
	if (config->dcc_ndl_chan_list_len) {
		ndl_chan = (wmi_dcc_ndl_chan *)buf_ptr;
		qdf_mem_copy(ndl_chan, config->dcc_ndl_chan_list,
			     config->dcc_ndl_chan_list_len);
		for (i = 0; i < config->channel_count; i++)
			WMITLV_SET_HDR(&(ndl_chan[i].tlv_header),
				WMITLV_TAG_STRUC_wmi_dcc_ndl_chan,
				WMITLV_GET_STRUCT_TLVLEN(wmi_dcc_ndl_chan));
		buf_ptr += config->dcc_ndl_chan_list_len;
	}

	/* Add the wmi_dcc_ndl_active_state_config */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, active_state_count *
		       sizeof(wmi_dcc_ndl_active_state_config));
	buf_ptr += WMI_TLV_HDR_SIZE;
	if (active_state_count) {
		ndl_active_config = (wmi_dcc_ndl_active_state_config *)buf_ptr;
		qdf_mem_copy(ndl_active_config,
			config->dcc_ndl_active_state_list,
			active_state_count * sizeof(*ndl_active_config));
		for (i = 0; i < active_state_count; ++i)
			WMITLV_SET_HDR(&(ndl_active_config[i].tlv_header),
			  WMITLV_TAG_STRUC_wmi_dcc_ndl_active_state_config,
			  WMITLV_GET_STRUCT_TLVLEN(
				wmi_dcc_ndl_active_state_config));
		buf_ptr += active_state_count *
			sizeof(*ndl_active_config);
	}

	/* Add the wmi_ocb_schedule_element info */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		config->schedule_size * sizeof(wmi_ocb_schedule_element));
	buf_ptr += WMI_TLV_HDR_SIZE;
	for (i = 0; i < config->schedule_size; i++) {
		sched_elem = (wmi_ocb_schedule_element *)buf_ptr;
		WMITLV_SET_HDR(&sched_elem->tlv_header,
			WMITLV_TAG_STRUC_wmi_ocb_schedule_element,
			WMITLV_GET_STRUCT_TLVLEN(wmi_ocb_schedule_element));
		sched_elem->channel_freq = config->schedule[i].chan_freq;
		sched_elem->total_duration = config->schedule[i].total_duration;
		sched_elem->guard_interval = config->schedule[i].guard_interval;
		buf_ptr += sizeof(*sched_elem);
	}


	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_OCB_SET_CONFIG_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to set OCB config");
		wmi_buf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_set_enable_disable_mcc_adaptive_scheduler_cmd_tlv() -enable/disable mcc scheduler
 * @wmi_handle: wmi handle
 * @mcc_adaptive_scheduler: enable/disable
 *
 * This function enable/disable mcc adaptive scheduler in fw.
 *
 * Return: QDF_STATUS_SUCCESS for sucess or error code
 */
QDF_STATUS send_set_enable_disable_mcc_adaptive_scheduler_cmd_tlv(
		   wmi_unified_t wmi_handle, uint32_t mcc_adaptive_scheduler)
{
	int ret = -1;
	wmi_buf_t buf = 0;
	wmi_resmgr_adaptive_ocs_enable_disable_cmd_fixed_param *cmd = NULL;
	uint16_t len =
		sizeof(wmi_resmgr_adaptive_ocs_enable_disable_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_resmgr_adaptive_ocs_enable_disable_cmd_fixed_param *)
		wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_resmgr_adaptive_ocs_enable_disable_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_resmgr_adaptive_ocs_enable_disable_cmd_fixed_param));
	cmd->enable = mcc_adaptive_scheduler;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_RESMGR_ADAPTIVE_OCS_ENABLE_DISABLE_CMDID);
	if (ret) {
		WMI_LOGP("%s: Failed to send enable/disable MCC"
			 " adaptive scheduler command", __func__);
		qdf_nbuf_free(buf);
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_mcc_channel_time_latency_cmd_tlv() -set MCC channel time latency
 * @wmi: wmi handle
 * @mcc_channel: mcc channel
 * @mcc_channel_time_latency: MCC channel time latency.
 *
 * Currently used to set time latency for an MCC vdev/adapter using operating
 * channel of it and channel number. The info is provided run time using
 * iwpriv command: iwpriv <wlan0 | p2p0> setMccLatency <latency in ms>.
 *
 * Return: CDF status
 */
QDF_STATUS send_set_mcc_channel_time_latency_cmd_tlv(wmi_unified_t wmi_handle,
	uint32_t mcc_channel_freq, uint32_t mcc_channel_time_latency)
{
	int ret = -1;
	wmi_buf_t buf = 0;
	wmi_resmgr_set_chan_latency_cmd_fixed_param *cmdTL = NULL;
	uint16_t len = 0;
	uint8_t *buf_ptr = NULL;
	wmi_resmgr_chan_latency chan_latency;
	/* Note: we only support MCC time latency for a single channel */
	uint32_t num_channels = 1;
	uint32_t chan1_freq = mcc_channel_freq;
	uint32_t latency_chan1 = mcc_channel_time_latency;


	/* If 0ms latency is provided, then FW will set to a default.
	 * Otherwise, latency must be at least 30ms.
	 */
	if ((latency_chan1 > 0) &&
	    (latency_chan1 < WMI_MCC_MIN_NON_ZERO_CHANNEL_LATENCY)) {
		WMI_LOGE("%s: Invalid time latency for Channel #1 = %dms "
			 "Minimum is 30ms (or 0 to use default value by "
			 "firmware)", __func__, latency_chan1);
		return QDF_STATUS_E_INVAL;
	}

	/*   Set WMI CMD for channel time latency here */
	len = sizeof(wmi_resmgr_set_chan_latency_cmd_fixed_param) +
	      WMI_TLV_HDR_SIZE +  /*Place holder for chan_time_latency array */
	      num_channels * sizeof(wmi_resmgr_chan_latency);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmdTL = (wmi_resmgr_set_chan_latency_cmd_fixed_param *)
		wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmdTL->tlv_header,
		WMITLV_TAG_STRUC_wmi_resmgr_set_chan_latency_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_resmgr_set_chan_latency_cmd_fixed_param));
	cmdTL->num_chans = num_channels;
	/* Update channel time latency information for home channel(s) */
	buf_ptr += sizeof(*cmdTL);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE,
		       num_channels * sizeof(wmi_resmgr_chan_latency));
	buf_ptr += WMI_TLV_HDR_SIZE;
	chan_latency.chan_mhz = chan1_freq;
	chan_latency.latency = latency_chan1;
	qdf_mem_copy(buf_ptr, &chan_latency, sizeof(chan_latency));
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_RESMGR_SET_CHAN_LATENCY_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send MCC Channel Time Latency command",
			 __func__);
		qdf_nbuf_free(buf);
		QDF_ASSERT(0);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_mcc_channel_time_quota_cmd_tlv() -set MCC channel time quota
 * @wmi: wmi handle
 * @adapter_1_chan_number: adapter 1 channel number
 * @adapter_1_quota: adapter 1 quota
 * @adapter_2_chan_number: adapter 2 channel number
 *
 * Return: CDF status
 */
QDF_STATUS send_set_mcc_channel_time_quota_cmd_tlv(wmi_unified_t wmi_handle,
	uint32_t adapter_1_chan_freq,
	uint32_t adapter_1_quota, uint32_t adapter_2_chan_freq)
{
	int ret = -1;
	wmi_buf_t buf = 0;
	uint16_t len = 0;
	uint8_t *buf_ptr = NULL;
	wmi_resmgr_set_chan_time_quota_cmd_fixed_param *cmdTQ = NULL;
	wmi_resmgr_chan_time_quota chan_quota;
	uint32_t quota_chan1 = adapter_1_quota;
	/* Knowing quota of 1st chan., derive quota for 2nd chan. */
	uint32_t quota_chan2 = 100 - quota_chan1;
	/* Note: setting time quota for MCC requires info for 2 channels */
	uint32_t num_channels = 2;
	uint32_t chan1_freq = adapter_1_chan_freq;
	uint32_t chan2_freq = adapter_2_chan_freq;

	WMI_LOGD("%s: freq1:%dMHz, Quota1:%dms, "
		 "freq2:%dMHz, Quota2:%dms", __func__,
		 chan1_freq, quota_chan1, chan2_freq,
		 quota_chan2);

	/*
	 * Perform sanity check on time quota values provided.
	 */
	if (quota_chan1 < WMI_MCC_MIN_CHANNEL_QUOTA ||
	    quota_chan1 > WMI_MCC_MAX_CHANNEL_QUOTA) {
		WMI_LOGE("%s: Invalid time quota for Channel #1=%dms. Minimum "
			 "is 20ms & maximum is 80ms", __func__, quota_chan1);
		return QDF_STATUS_E_INVAL;
	}
	/* Set WMI CMD for channel time quota here */
	len = sizeof(wmi_resmgr_set_chan_time_quota_cmd_fixed_param) +
	      WMI_TLV_HDR_SIZE +       /* Place holder for chan_time_quota array */
	      num_channels * sizeof(wmi_resmgr_chan_time_quota);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		QDF_ASSERT(0);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmdTQ = (wmi_resmgr_set_chan_time_quota_cmd_fixed_param *)
		wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmdTQ->tlv_header,
		       WMITLV_TAG_STRUC_wmi_resmgr_set_chan_time_quota_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_resmgr_set_chan_time_quota_cmd_fixed_param));
	cmdTQ->num_chans = num_channels;

	/* Update channel time quota information for home channel(s) */
	buf_ptr += sizeof(*cmdTQ);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE,
		       num_channels * sizeof(wmi_resmgr_chan_time_quota));
	buf_ptr += WMI_TLV_HDR_SIZE;
	chan_quota.chan_mhz = chan1_freq;
	chan_quota.channel_time_quota = quota_chan1;
	qdf_mem_copy(buf_ptr, &chan_quota, sizeof(chan_quota));
	/* Construct channel and quota record for the 2nd MCC mode. */
	buf_ptr += sizeof(chan_quota);
	chan_quota.chan_mhz = chan2_freq;
	chan_quota.channel_time_quota = quota_chan2;
	qdf_mem_copy(buf_ptr, &chan_quota, sizeof(chan_quota));

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_RESMGR_SET_CHAN_TIME_QUOTA_CMDID);
	if (ret) {
		WMI_LOGE("Failed to send MCC Channel Time Quota command");
		qdf_nbuf_free(buf);
		QDF_ASSERT(0);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_thermal_mgmt_cmd_tlv() - set thermal mgmt command to fw
 * @wmi_handle: Pointer to wmi handle
 * @thermal_info: Thermal command information
 *
 * This function sends the thermal management command
 * to the firmware
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 */
QDF_STATUS send_set_thermal_mgmt_cmd_tlv(wmi_unified_t wmi_handle,
				struct thermal_cmd_params *thermal_info)
{
	wmi_thermal_mgmt_cmd_fixed_param *cmd = NULL;
	wmi_buf_t buf = NULL;
	int status = 0;
	uint32_t len = 0;

	len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send set key cmd");
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_thermal_mgmt_cmd_fixed_param *) wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_thermal_mgmt_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_thermal_mgmt_cmd_fixed_param));

	cmd->lower_thresh_degreeC = thermal_info->min_temp;
	cmd->upper_thresh_degreeC = thermal_info->max_temp;
	cmd->enable = thermal_info->thermal_enable;

	WMI_LOGE("TM Sending thermal mgmt cmd: low temp %d, upper temp %d, enabled %d",
		cmd->lower_thresh_degreeC, cmd->upper_thresh_degreeC, cmd->enable);

	status = wmi_unified_cmd_send(wmi_handle, buf, len,
				      WMI_THERMAL_MGMT_CMDID);
	if (status) {
		qdf_nbuf_free(buf);
		WMI_LOGE("%s:Failed to send thermal mgmt command", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}


/**
 * send_lro_config_cmd_tlv() - process the LRO config command
 * @wmi_handle: Pointer to WMI handle
 * @wmi_lro_cmd: Pointer to LRO configuration parameters
 *
 * This function sends down the LRO configuration parameters to
 * the firmware to enable LRO, sets the TCP flags and sets the
 * seed values for the toeplitz hash generation
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 */
QDF_STATUS send_lro_config_cmd_tlv(wmi_unified_t wmi_handle,
	 struct wmi_lro_config_cmd_t *wmi_lro_cmd)
{
	wmi_lro_info_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int status;


	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send set key cmd");
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_lro_info_cmd_fixed_param *) wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
		 WMITLV_TAG_STRUC_wmi_lro_info_cmd_fixed_param,
		 WMITLV_GET_STRUCT_TLVLEN(wmi_lro_info_cmd_fixed_param));

	cmd->lro_enable = wmi_lro_cmd->lro_enable;
	WMI_LRO_INFO_TCP_FLAG_VALS_SET(cmd->tcp_flag_u32,
		 wmi_lro_cmd->tcp_flag);
	WMI_LRO_INFO_TCP_FLAGS_MASK_SET(cmd->tcp_flag_u32,
		 wmi_lro_cmd->tcp_flag_mask);
	cmd->toeplitz_hash_ipv4_0_3 =
		 wmi_lro_cmd->toeplitz_hash_ipv4[0];
	cmd->toeplitz_hash_ipv4_4_7 =
		 wmi_lro_cmd->toeplitz_hash_ipv4[1];
	cmd->toeplitz_hash_ipv4_8_11 =
		 wmi_lro_cmd->toeplitz_hash_ipv4[2];
	cmd->toeplitz_hash_ipv4_12_15 =
		 wmi_lro_cmd->toeplitz_hash_ipv4[3];
	cmd->toeplitz_hash_ipv4_16 =
		 wmi_lro_cmd->toeplitz_hash_ipv4[4];

	cmd->toeplitz_hash_ipv6_0_3 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[0];
	cmd->toeplitz_hash_ipv6_4_7 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[1];
	cmd->toeplitz_hash_ipv6_8_11 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[2];
	cmd->toeplitz_hash_ipv6_12_15 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[3];
	cmd->toeplitz_hash_ipv6_16_19 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[4];
	cmd->toeplitz_hash_ipv6_20_23 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[5];
	cmd->toeplitz_hash_ipv6_24_27 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[6];
	cmd->toeplitz_hash_ipv6_28_31 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[7];
	cmd->toeplitz_hash_ipv6_32_35 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[8];
	cmd->toeplitz_hash_ipv6_36_39 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[9];
	cmd->toeplitz_hash_ipv6_40 =
		 wmi_lro_cmd->toeplitz_hash_ipv6[10];

	WMI_LOGD("WMI_LRO_CONFIG: lro_enable %d, tcp_flag 0x%x",
		cmd->lro_enable, cmd->tcp_flag_u32);

	status = wmi_unified_cmd_send(wmi_handle, buf,
		 sizeof(*cmd), WMI_LRO_CONFIG_CMDID);
	if (status) {
		qdf_nbuf_free(buf);
		WMI_LOGE("%s:Failed to send WMI_LRO_CONFIG_CMDID", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_bcn_buf_ll_cmd_tlv() - prepare and send beacon buffer to fw for LL
 * @wmi_handle: wmi handle
 * @param: bcn ll cmd parameter
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 */
QDF_STATUS send_bcn_buf_ll_cmd_tlv(wmi_unified_t wmi_handle,
			wmi_bcn_send_from_host_cmd_fixed_param *param)
{
	wmi_bcn_send_from_host_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	QDF_STATUS ret;

	wmi_buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_bcn_send_from_host_cmd_fixed_param *) wmi_buf_data(wmi_buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_bcn_send_from_host_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_bcn_send_from_host_cmd_fixed_param));
	cmd->vdev_id = param->vdev_id;
	cmd->data_len = param->data_len;
	cmd->frame_ctrl = param->frame_ctrl;
	cmd->frag_ptr = param->frag_ptr;
	cmd->dtim_flag = param->dtim_flag;

	ret = wmi_unified_cmd_send(wmi_handle, wmi_buf, sizeof(*cmd),
				      WMI_PDEV_SEND_BCN_CMDID);

	if (ret != EOK) {
		WMI_LOGE("Failed to send WMI_PDEV_SEND_BCN_CMDID command");
		wmi_buf_free(wmi_buf);
	}

	return ret;
}

/**
 * send_set_sta_sa_query_param_cmd_tlv() - set sta sa query parameters
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @max_retries: max retries
 * @retry_interval: retry interval
 * This function sets sta query related parameters in fw.
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 */

QDF_STATUS send_set_sta_sa_query_param_cmd_tlv(wmi_unified_t wmi_handle,
				       uint8_t vdev_id, uint32_t max_retries,
					   uint32_t retry_interval)
{
	wmi_buf_t buf;
	WMI_PMF_OFFLOAD_SET_SA_QUERY_CMD_fixed_param *cmd;
	int len;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (WMI_PMF_OFFLOAD_SET_SA_QUERY_CMD_fixed_param *)wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_WMI_PMF_OFFLOAD_SET_SA_QUERY_CMD_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN
		(WMI_PMF_OFFLOAD_SET_SA_QUERY_CMD_fixed_param));


	cmd->vdev_id = vdev_id;
	cmd->sa_query_max_retry_count = max_retries;
	cmd->sa_query_retry_interval = retry_interval;

	WMI_LOGD(FL("STA sa query: vdev_id:%d interval:%u retry count:%d"),
		 vdev_id, retry_interval, max_retries);

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_PMF_OFFLOAD_SET_SA_QUERY_CMDID)) {
		WMI_LOGE(FL("Failed to offload STA SA Query"));
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGD(FL("Exit :"));
	return 0;
}

/**
 * send_set_sta_keep_alive_cmd_tlv() - set sta keep alive parameters
 * @wmi_handle: wmi handle
 * @params: sta keep alive parameter
 *
 * This function sets keep alive related parameters in fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_set_sta_keep_alive_cmd_tlv(wmi_unified_t wmi_handle,
				struct sta_params *params)
{
	wmi_buf_t buf;
	WMI_STA_KEEPALIVE_CMD_fixed_param *cmd;
	WMI_STA_KEEPALVE_ARP_RESPONSE *arp_rsp;
	uint8_t *buf_ptr;
	int len;

	WMI_LOGD("%s: Enter", __func__);

	if (params->timeperiod > WNI_CFG_INFRA_STA_KEEP_ALIVE_PERIOD_STAMAX) {
		WMI_LOGE("Invalid period %d Max limit %d", params->timeperiod,
			WNI_CFG_INFRA_STA_KEEP_ALIVE_PERIOD_STAMAX);
		return QDF_STATUS_E_FAILURE;
	}

	len = sizeof(*cmd) + sizeof(*arp_rsp);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("wmi_buf_alloc failed");
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (WMI_STA_KEEPALIVE_CMD_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_STA_KEEPALIVE_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_STA_KEEPALIVE_CMD_fixed_param));
	cmd->interval = params->timeperiod;
	cmd->enable = (params->timeperiod) ? 1 : 0;
	cmd->vdev_id = params->vdev_id;
	WMI_LOGD("Keep Alive: vdev_id:%d interval:%u method:%d", params->vdev_id,
		 params->timeperiod, params->method);
	arp_rsp = (WMI_STA_KEEPALVE_ARP_RESPONSE *) (buf_ptr + sizeof(*cmd));
	WMITLV_SET_HDR(&arp_rsp->tlv_header,
		       WMITLV_TAG_STRUC_WMI_STA_KEEPALVE_ARP_RESPONSE,
		       WMITLV_GET_STRUCT_TLVLEN(WMI_STA_KEEPALVE_ARP_RESPONSE));

	if (params->method == WMI_KEEP_ALIVE_UNSOLICIT_ARP_RSP) {
		if ((NULL == params->hostv4addr) ||
			(NULL == params->destv4addr) ||
			(NULL == params->destmac)) {
			WMI_LOGE("%s: received null pointer, hostv4addr:%p "
			   "destv4addr:%p destmac:%p ", __func__,
			   params->hostv4addr, params->destv4addr, params->destmac);
			qdf_nbuf_free(buf);
			return QDF_STATUS_E_FAILURE;
		}
		cmd->method = WMI_STA_KEEPALIVE_METHOD_UNSOLICITED_ARP_RESPONSE;
		qdf_mem_copy(&arp_rsp->sender_prot_addr, params->hostv4addr,
			     WMI_IPV4_ADDR_LEN);
		qdf_mem_copy(&arp_rsp->target_prot_addr, params->destv4addr,
			     WMI_IPV4_ADDR_LEN);
		WMI_CHAR_ARRAY_TO_MAC_ADDR(params->destmac, &arp_rsp->dest_mac_addr);
	} else {
		cmd->method = WMI_STA_KEEPALIVE_METHOD_NULL_FRAME;
	}

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_STA_KEEPALIVE_CMDID)) {
		WMI_LOGE("Failed to set KeepAlive");
		qdf_nbuf_free(buf);
	}

	WMI_LOGD("%s: Exit", __func__);
	return 0;
}

/**
 * send_vdev_set_gtx_cfg_cmd_tlv() - set GTX params
 * @wmi_handle: wmi handle
 * @if_id: vdev id
 * @gtx_info: GTX config params
 *
 * This function set GTX related params in firmware.
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_vdev_set_gtx_cfg_cmd_tlv(wmi_unified_t wmi_handle, uint32_t if_id,
				  struct wmi_gtx_config *gtx_info)
{
	wmi_vdev_set_gtx_params_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int len = sizeof(wmi_vdev_set_gtx_params_cmd_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_vdev_set_gtx_params_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_set_gtx_params_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_set_gtx_params_cmd_fixed_param));
	cmd->vdev_id = if_id;

	cmd->gtxRTMask[0] = gtx_info->gtx_rt_mask[0];
	cmd->gtxRTMask[1] = gtx_info->gtx_rt_mask[1];
	cmd->userGtxMask = gtx_info->gtx_usrcfg;
	cmd->gtxPERThreshold = gtx_info->gtx_threshold;
	cmd->gtxPERMargin = gtx_info->gtx_margin;
	cmd->gtxTPCstep = gtx_info->gtx_tpcstep;
	cmd->gtxTPCMin = gtx_info->gtx_tpcmin;
	cmd->gtxBWMask = gtx_info->gtx_bwmask;

	WMI_LOGD("Setting vdev%d GTX values:htmcs 0x%x, vhtmcs 0x%x, usermask 0x%x, \
		gtxPERThreshold %d, gtxPERMargin %d, gtxTPCstep %d, gtxTPCMin %d, \
		gtxBWMask 0x%x.", if_id, cmd->gtxRTMask[0], cmd->gtxRTMask[1],
		 cmd->userGtxMask, cmd->gtxPERThreshold, cmd->gtxPERMargin,
		 cmd->gtxTPCstep, cmd->gtxTPCMin, cmd->gtxBWMask);

	return wmi_unified_cmd_send(wmi_handle, buf, len,
				    WMI_VDEV_SET_GTX_PARAMS_CMDID);
}

/**
 * send_process_update_edca_param_cmd_tlv() - update EDCA params
 * @wmi_handle: wmi handle
 * @edca_params: edca parameters
 *
 * This function updates EDCA parameters to the target
 *
 * Return: CDF Status
 */
QDF_STATUS send_process_update_edca_param_cmd_tlv(wmi_unified_t wmi_handle,
				    uint8_t vdev_id,
				    wmi_wmm_vparams gwmm_param[WMI_MAX_NUM_AC])
{
	uint8_t *buf_ptr;
	wmi_buf_t buf;
	wmi_vdev_set_wmm_params_cmd_fixed_param *cmd;
	wmi_wmm_vparams *wmm_param, *twmm_param;
	int len = sizeof(*cmd);
	int ac;

	buf = wmi_buf_alloc(wmi_handle, len);

	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_vdev_set_wmm_params_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_set_wmm_params_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_set_wmm_params_cmd_fixed_param));
	cmd->vdev_id = vdev_id;

	for (ac = 0; ac < WMI_MAX_NUM_AC; ac++) {
		wmm_param = (wmi_wmm_vparams *) (&cmd->wmm_params[ac]);
		twmm_param = (wmi_wmm_vparams *) (&gwmm_param[ac]);
		WMITLV_SET_HDR(&wmm_param->tlv_header,
			       WMITLV_TAG_STRUC_wmi_vdev_set_wmm_params_cmd_fixed_param,
			       WMITLV_GET_STRUCT_TLVLEN(wmi_wmm_vparams));
		wmm_param->cwmin = twmm_param->cwmin;
		wmm_param->cwmax = twmm_param->cwmax;
		wmm_param->aifs = twmm_param->aifs;
		wmm_param->txoplimit = twmm_param->txoplimit;
		wmm_param->acm = twmm_param->acm;
		wmm_param->no_ack = twmm_param->no_ack;
	}

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_VDEV_SET_WMM_PARAMS_CMDID))
		goto fail;

	return QDF_STATUS_SUCCESS;

fail:
	wmi_buf_free(buf);
	WMI_LOGE("%s: Failed to set WMM Paremeters", __func__);
	return QDF_STATUS_E_FAILURE;
}

/**
 * send_probe_rsp_tmpl_send_cmd_tlv() - send probe response template to fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @probe_rsp_info: probe response info
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_probe_rsp_tmpl_send_cmd_tlv(wmi_unified_t wmi_handle,
				   uint8_t vdev_id,
				   struct wmi_probe_resp_params *probe_rsp_info,
				   uint8_t *frm)
{
	wmi_prb_tmpl_cmd_fixed_param *cmd;
	wmi_bcn_prb_info *bcn_prb_info;
	wmi_buf_t wmi_buf;
	uint32_t tmpl_len, tmpl_len_aligned, wmi_buf_len;
	uint8_t *buf_ptr;
	int ret;

	WMI_LOGD(FL("Send probe response template for vdev %d"), vdev_id);

	tmpl_len = probe_rsp_info->probeRespTemplateLen;
	tmpl_len_aligned = roundup(tmpl_len, sizeof(A_UINT32));

	wmi_buf_len = sizeof(wmi_prb_tmpl_cmd_fixed_param) +
			sizeof(wmi_bcn_prb_info) + WMI_TLV_HDR_SIZE +
			tmpl_len_aligned;

	if (wmi_buf_len > WMI_BEACON_TX_BUFFER_SIZE) {
		WMI_LOGE(FL("wmi_buf_len: %d > %d. Can't send wmi cmd"),
		wmi_buf_len, WMI_BEACON_TX_BUFFER_SIZE);
		return -EINVAL;
	}

	wmi_buf = wmi_buf_alloc(wmi_handle, wmi_buf_len);
	if (!wmi_buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_prb_tmpl_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_prb_tmpl_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_prb_tmpl_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->buf_len = tmpl_len;
	buf_ptr += sizeof(wmi_prb_tmpl_cmd_fixed_param);

	bcn_prb_info = (wmi_bcn_prb_info *) buf_ptr;
	WMITLV_SET_HDR(&bcn_prb_info->tlv_header,
		       WMITLV_TAG_STRUC_wmi_bcn_prb_info,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_bcn_prb_info));
	bcn_prb_info->caps = 0;
	bcn_prb_info->erp = 0;
	buf_ptr += sizeof(wmi_bcn_prb_info);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, tmpl_len_aligned);
	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, frm, tmpl_len);

	ret = wmi_unified_cmd_send(wmi_handle,
				   wmi_buf, wmi_buf_len, WMI_PRB_TMPL_CMDID);
	if (ret) {
		WMI_LOGE(FL("Failed to send PRB RSP tmpl: %d"), ret);
		wmi_buf_free(wmi_buf);
	}

	return ret;
}

/**
 * send_p2p_go_set_beacon_ie_cmd_tlv() - set beacon IE for p2p go
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @p2p_ie: p2p IE
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_p2p_go_set_beacon_ie_cmd_tlv(wmi_unified_t wmi_handle,
				    A_UINT32 vdev_id, uint8_t *p2p_ie)
{
	int ret;
	wmi_p2p_go_set_beacon_ie_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t ie_len, ie_len_aligned, wmi_buf_len;
	uint8_t *buf_ptr;

	ie_len = (uint32_t) (p2p_ie[1] + 2);

	/* More than one P2P IE may be included in a single frame.
	   If multiple P2P IEs are present, the complete P2P attribute
	   data consists of the concatenation of the P2P Attribute
	   fields of the P2P IEs. The P2P Attributes field of each
	   P2P IE may be any length up to the maximum (251 octets).
	   In this case host sends one P2P IE to firmware so the length
	   should not exceed more than 251 bytes
	 */
	if (ie_len > 251) {
		WMI_LOGE("%s : invalid p2p ie length %u", __func__, ie_len);
		return -EINVAL;
	}

	ie_len_aligned = roundup(ie_len, sizeof(A_UINT32));

	wmi_buf_len =
		sizeof(wmi_p2p_go_set_beacon_ie_fixed_param) + ie_len_aligned +
		WMI_TLV_HDR_SIZE;

	wmi_buf = wmi_buf_alloc(wmi_handle, wmi_buf_len);
	if (!wmi_buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_p2p_go_set_beacon_ie_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_p2p_go_set_beacon_ie_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_p2p_go_set_beacon_ie_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->ie_buf_len = ie_len;

	buf_ptr += sizeof(wmi_p2p_go_set_beacon_ie_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, ie_len_aligned);
	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, p2p_ie, ie_len);

	WMI_LOGI("%s: Sending WMI_P2P_GO_SET_BEACON_IE", __func__);

	ret = wmi_unified_cmd_send(wmi_handle,
				   wmi_buf, wmi_buf_len,
				   WMI_P2P_GO_SET_BEACON_IE);
	if (ret) {
		WMI_LOGE("Failed to send bcn tmpl: %d", ret);
		wmi_buf_free(wmi_buf);
	}

	WMI_LOGI("%s: Successfully sent WMI_P2P_GO_SET_BEACON_IE", __func__);
	return ret;
}

/**
 * send_set_gateway_params_cmd_tlv() - set gateway parameters
 * @wmi_handle: wmi handle
 * @req: gateway parameter update request structure
 *
 * This function reads the incoming @req and fill in the destination
 * WMI structure and sends down the gateway configs down to the firmware
 *
 * Return: QDF_STATUS
 */
QDF_STATUS send_set_gateway_params_cmd_tlv(wmi_unified_t wmi_handle,
				struct gateway_update_req_param *req)
{
	wmi_roam_subnet_change_config_fixed_param *cmd;
	wmi_buf_t buf;
	int ret;
	int len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_roam_subnet_change_config_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_roam_subnet_change_config_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_roam_subnet_change_config_fixed_param));

	cmd->vdev_id = req->session_id;
	qdf_mem_copy(&cmd->inet_gw_ip_v4_addr, req->ipv4_addr,
		QDF_IPV4_ADDR_SIZE);
	qdf_mem_copy(&cmd->inet_gw_ip_v6_addr, req->ipv6_addr,
		QDF_IPV6_ADDR_SIZE);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(req->gw_mac_addr.bytes,
		&cmd->inet_gw_mac_addr);
	cmd->max_retries = req->max_retries;
	cmd->timeout = req->timeout;
	cmd->num_skip_subnet_change_detection_bssid_list = 0;
	cmd->flag = 0;
	if (req->ipv4_addr_type)
		WMI_SET_ROAM_SUBNET_CHANGE_FLAG_IP4_ENABLED(cmd->flag);

	if (req->ipv6_addr_type)
		WMI_SET_ROAM_SUBNET_CHANGE_FLAG_IP6_ENABLED(cmd->flag);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_ROAM_SUBNET_CHANGE_CONFIG_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send gw config parameter to fw, ret: %d",
			ret);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_rssi_monitoring_cmd_tlv() - set rssi monitoring
 * @wmi_handle: wmi handle
 * @req: rssi monitoring request structure
 *
 * This function reads the incoming @req and fill in the destination
 * WMI structure and send down the rssi monitoring configs down to the firmware
 *
 * Return: 0 on success; error number otherwise
 */
QDF_STATUS send_set_rssi_monitoring_cmd_tlv(wmi_unified_t wmi_handle,
					struct rssi_monitor_param *req)
{
	wmi_rssi_breach_monitor_config_fixed_param *cmd;
	wmi_buf_t buf;
	int ret, len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_rssi_breach_monitor_config_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_rssi_breach_monitor_config_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_rssi_breach_monitor_config_fixed_param));

	cmd->vdev_id = req->session_id;
	cmd->request_id = req->request_id;
	cmd->lo_rssi_reenable_hysteresis = 0;
	cmd->hi_rssi_reenable_histeresis = 0;
	cmd->min_report_interval = 0;
	cmd->max_num_report = 1;
	if (req->control) {
		/* enable one threshold for each min/max */
		cmd->enabled_bitmap = 0x09;
		cmd->low_rssi_breach_threshold[0] = req->min_rssi;
		cmd->hi_rssi_breach_threshold[0] = req->max_rssi;
	} else {
		cmd->enabled_bitmap = 0;
		cmd->low_rssi_breach_threshold[0] = 0;
		cmd->hi_rssi_breach_threshold[0] = 0;
	}

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_RSSI_BREACH_MONITOR_CONFIG_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send WMI_RSSI_BREACH_MONITOR_CONFIG_CMDID");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGI("Sent WMI_RSSI_BREACH_MONITOR_CONFIG_CMDID to FW");
	return QDF_STATUS_SUCCESS;
}

/**
 * send_scan_probe_setoui_cmd_tlv() - set scan probe OUI
 * @wmi_handle: wmi handle
 * @psetoui: OUI parameters
 *
 * set scan probe OUI parameters in firmware
 *
 * Return: CDF status
 */
QDF_STATUS send_scan_probe_setoui_cmd_tlv(wmi_unified_t wmi_handle,
			  struct scan_mac_oui *psetoui)
{
	wmi_scan_prob_req_oui_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;
	uint32_t *oui_buf;

	len = sizeof(*cmd);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_scan_prob_req_oui_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_scan_prob_req_oui_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_scan_prob_req_oui_cmd_fixed_param));

	oui_buf = &cmd->prob_req_oui;
	qdf_mem_zero(oui_buf, sizeof(cmd->prob_req_oui));
	*oui_buf = psetoui->oui[0] << 16 | psetoui->oui[1] << 8
		   | psetoui->oui[2];
	WMI_LOGD("%s: wmi:oui received from hdd %08x", __func__,
		 cmd->prob_req_oui);

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_SCAN_PROB_REQ_OUI_CMDID)) {
		WMI_LOGE("%s: failed to send command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_reset_passpoint_network_list_cmd_tlv() - reset passpoint network list
 * @wmi_handle: wmi handle
 * @req: passpoint network request structure
 *
 * This function sends down WMI command with network id set to wildcard id.
 * firmware shall clear all the config entries
 *
 * Return: QDF_STATUS enumeration
 */
QDF_STATUS send_reset_passpoint_network_list_cmd_tlv(wmi_unified_t wmi_handle,
					struct wifi_passpoint_req_param *req)
{
	wmi_passpoint_config_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint32_t len;
	int ret;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_passpoint_config_cmd_fixed_param *) wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_passpoint_config_cmd_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(
			wmi_passpoint_config_cmd_fixed_param));
	cmd->id = WMI_PASSPOINT_NETWORK_ID_WILDCARD;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_PASSPOINT_LIST_CONFIG_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send reset passpoint network list wmi cmd",
			 __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_passpoint_network_list_cmd_tlv() - set passpoint network list
 * @wmi_handle: wmi handle
 * @req: passpoint network request structure
 *
 * This function reads the incoming @req and fill in the destination
 * WMI structure and send down the passpoint configs down to the firmware
 *
 * Return: QDF_STATUS enumeration
 */
QDF_STATUS send_set_passpoint_network_list_cmd_tlv(wmi_unified_t wmi_handle,
					struct wifi_passpoint_req_param *req)
{
	wmi_passpoint_config_cmd_fixed_param *cmd;
	u_int8_t i, j, *bytes;
	wmi_buf_t buf;
	uint32_t len;
	int ret;

	len = sizeof(*cmd);
	for (i = 0; i < req->num_networks; i++) {
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
			return QDF_STATUS_E_NOMEM;
		}

		cmd = (wmi_passpoint_config_cmd_fixed_param *)
				wmi_buf_data(buf);

		WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_passpoint_config_cmd_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(
			wmi_passpoint_config_cmd_fixed_param));
		cmd->id = req->networks[i].id;
		WMI_LOGD("%s: network id: %u", __func__, cmd->id);
		qdf_mem_copy(cmd->realm, req->networks[i].realm,
			strlen(req->networks[i].realm) + 1);
		WMI_LOGD("%s: realm: %s", __func__, cmd->realm);
		for (j = 0; j < PASSPOINT_ROAMING_CONSORTIUM_ID_NUM; j++) {
			bytes = (uint8_t *) &req->networks[i].roaming_consortium_ids[j];
			WMI_LOGD("index: %d rcids: %02x %02x %02x %02x %02x %02x %02x %02x",
				j, bytes[0], bytes[1], bytes[2], bytes[3],
				bytes[4], bytes[5], bytes[6], bytes[7]);

			qdf_mem_copy(&cmd->roaming_consortium_ids[j],
				&req->networks[i].roaming_consortium_ids[j],
				PASSPOINT_ROAMING_CONSORTIUM_ID_LEN);
		}
		qdf_mem_copy(cmd->plmn, req->networks[i].plmn,
				PASSPOINT_PLMN_ID_LEN);
		WMI_LOGD("%s: plmn: %02x:%02x:%02x", __func__,
			cmd->plmn[0], cmd->plmn[1], cmd->plmn[2]);

		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
					   WMI_PASSPOINT_LIST_CONFIG_CMDID);
		if (ret) {
			WMI_LOGE("%s: Failed to send set passpoint network list wmi cmd",
				 __func__);
			wmi_buf_free(buf);
			return QDF_STATUS_E_FAILURE;
		}
	}

	return QDF_STATUS_SUCCESS;
}

/** send_set_epno_network_list_cmd_tlv() - set epno network list
 * @wmi_handle: wmi handle
 * @req: epno config params request structure
 *
 * This function reads the incoming epno config request structure
 * and constructs the WMI message to the firmware.
 *
 * Returns: 0 on success, error number otherwise
 */
QDF_STATUS send_set_epno_network_list_cmd_tlv(wmi_unified_t wmi_handle,
		struct wifi_enhanched_pno_params *req)
{
	wmi_nlo_config_cmd_fixed_param *cmd;
	nlo_configured_parameters *nlo_list;
	u_int8_t i, *buf_ptr;
	wmi_buf_t buf;
	uint32_t len;
	int ret;

	/* TLV place holder for array of structures
	 * nlo_configured_parameters(nlo_list) */
	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE;
	len += sizeof(nlo_configured_parameters) *
				QDF_MIN(req->num_networks, WMI_NLO_MAX_SSIDS);
	len += WMI_TLV_HDR_SIZE; /* TLV for channel_list */
	len += WMI_TLV_HDR_SIZE; /* TLV for channel prediction cfg*/

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_nlo_config_cmd_fixed_param *) wmi_buf_data(buf);

	buf_ptr = (u_int8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_nlo_config_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
			       wmi_nlo_config_cmd_fixed_param));
	cmd->vdev_id = req->session_id;
	cmd->flags = WMI_NLO_CONFIG_ENLO;

	buf_ptr += sizeof(wmi_nlo_config_cmd_fixed_param);

	cmd->no_of_ssids = QDF_MIN(req->num_networks, WMI_NLO_MAX_SSIDS);
	WMI_LOGD("SSID count: %d", cmd->no_of_ssids);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       cmd->no_of_ssids * sizeof(nlo_configured_parameters));
	buf_ptr += WMI_TLV_HDR_SIZE;

	nlo_list = (nlo_configured_parameters *) buf_ptr;
	for (i = 0; i < cmd->no_of_ssids; i++) {
		WMITLV_SET_HDR(&nlo_list[i].tlv_header,
			WMITLV_TAG_ARRAY_BYTE,
			WMITLV_GET_STRUCT_TLVLEN(nlo_configured_parameters));
		/* Copy ssid and it's length */
		nlo_list[i].ssid.valid = true;
		nlo_list[i].ssid.ssid.ssid_len = req->networks[i].ssid.length;
		qdf_mem_copy(nlo_list[i].ssid.ssid.ssid,
			     req->networks[i].ssid.mac_ssid,
			     nlo_list[i].ssid.ssid.ssid_len);
		WMI_LOGD("index: %d ssid: %.*s len: %d", i,
			 nlo_list[i].ssid.ssid.ssid_len,
			 (char *) nlo_list[i].ssid.ssid.ssid,
			 nlo_list[i].ssid.ssid.ssid_len);

		/* Copy rssi threshold */
		nlo_list[i].rssi_cond.valid = true;
		nlo_list[i].rssi_cond.rssi =
			req->networks[i].rssi_threshold;
		WMI_LOGD("RSSI threshold : %d dBm",
			nlo_list[i].rssi_cond.rssi);

		/* Copy pno flags */
		nlo_list[i].bcast_nw_type.valid = true;
		nlo_list[i].bcast_nw_type.bcast_nw_type =
				req->networks[i].flags;
		WMI_LOGD("PNO flags (%u)",
				nlo_list[i].bcast_nw_type.bcast_nw_type);

		/* Copy auth bit field */
		nlo_list[i].auth_type.valid = true;
		nlo_list[i].auth_type.auth_type =
				req->networks[i].auth_bit_field;
		WMI_LOGD("Auth bit field (%u)",
				nlo_list[i].auth_type.auth_type);
	}

	buf_ptr += cmd->no_of_ssids * sizeof(nlo_configured_parameters);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_NETWORK_LIST_OFFLOAD_CONFIG_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send nlo wmi cmd", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGD("set ePNO list request sent successfully for vdev %d",
		 req->session_id);

	return QDF_STATUS_SUCCESS;
}

/** send_ipa_offload_control_cmd_tlv() - ipa offload control parameter
 * @wmi_handle: wmi handle
 * @ipa_offload: ipa offload control parameter
 *
 * Returns: 0 on success, error number otherwise
 */
QDF_STATUS send_ipa_offload_control_cmd_tlv(wmi_unified_t wmi_handle,
		struct ipa_offload_control_params *ipa_offload)
{
	wmi_ipa_offload_enable_disable_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	u_int8_t *buf_ptr;

	len  = sizeof(*cmd);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed (len=%d)", __func__, len);
		return QDF_STATUS_E_NOMEM;
	}

	WMI_LOGE("%s: offload_type=%d, enable=%d", __func__,
		ipa_offload->offload_type, ipa_offload->enable);

	buf_ptr = (u_int8_t *)wmi_buf_data(wmi_buf);

	cmd = (wmi_ipa_offload_enable_disable_cmd_fixed_param *)buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUCT_wmi_ipa_offload_enable_disable_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
		wmi_ipa_offload_enable_disable_cmd_fixed_param));

	cmd->offload_type = ipa_offload->offload_type;
	cmd->vdev_id = ipa_offload->vdev_id;
	cmd->enable = ipa_offload->enable;

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
		WMI_IPA_OFFLOAD_ENABLE_DISABLE_CMDID)) {
		WMI_LOGE("%s: failed to command", __func__);
		wmi_buf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_extscan_get_capabilities_cmd_tlv() - extscan get capabilities
 * @wmi_handle: wmi handle
 * @pgetcapab: get capabilities params
 *
 * This function send request to fw to get extscan capabilities.
 *
 * Return: CDF status
 */
QDF_STATUS send_extscan_get_capabilities_cmd_tlv(wmi_unified_t wmi_handle,
		    struct extscan_capabilities_params *pgetcapab)
{
	wmi_extscan_get_capabilities_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;

	len = sizeof(*cmd);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_extscan_get_capabilities_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_wmi_extscan_get_capabilities_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_extscan_get_capabilities_cmd_fixed_param));

	cmd->request_id = pgetcapab->request_id;

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_EXTSCAN_GET_CAPABILITIES_CMDID)) {
		WMI_LOGE("%s: failed to  command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_extscan_get_cached_results_cmd_tlv() - extscan get cached results
 * @wmi_handle: wmi handle
 * @pcached_results: cached results parameters
 *
 * This function send request to fw to get cached results.
 *
 * Return: CDF status
 */
QDF_STATUS send_extscan_get_cached_results_cmd_tlv(wmi_unified_t wmi_handle,
		  struct extscan_cached_result_params *pcached_results)
{
	wmi_extscan_get_cached_results_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;

	len = sizeof(*cmd);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_extscan_get_cached_results_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_extscan_get_cached_results_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN
		(wmi_extscan_get_cached_results_cmd_fixed_param));

	cmd->request_id = pcached_results->request_id;
	cmd->vdev_id = pcached_results->session_id;
	cmd->control_flags = pcached_results->flush;

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_EXTSCAN_GET_CACHED_RESULTS_CMDID)) {
		WMI_LOGE("%s: failed to  command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_extscan_stop_change_monitor_cmd_tlv() - send stop change monitor cmd
 * @wmi_handle: wmi handle
 * @reset_req: Reset change request params
 *
 * This function sends stop change monitor request to fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_extscan_stop_change_monitor_cmd_tlv(wmi_unified_t wmi_handle,
			struct extscan_capabilities_reset_params *reset_req)
{
	wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;
	int change_list = 0;

	len = sizeof(*cmd);

	/* reset significant change tlv is set to 0 */
	len += WMI_TLV_HDR_SIZE;
	len += change_list * sizeof(wmi_extscan_wlan_change_bssid_param);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param *)
		buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
	WMITLV_TAG_STRUC_wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN
		(wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param));

	cmd->request_id = reset_req->request_id;
	cmd->vdev_id = reset_req->session_id;
	cmd->mode = 0;

	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_STRUC,
		       change_list *
		       sizeof(wmi_extscan_wlan_change_bssid_param));
	buf_ptr += WMI_TLV_HDR_SIZE + (change_list *
				       sizeof
				       (wmi_extscan_wlan_change_bssid_param));

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
			 WMI_EXTSCAN_CONFIGURE_WLAN_CHANGE_MONITOR_CMDID)) {
		WMI_LOGE("%s: failed to  command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * wmi_get_buf_extscan_change_monitor_cmd() - fill change monitor request
 * @wmi_handle: wmi handle
 * @psigchange: change monitor request params
 * @buf: wmi buffer
 * @buf_len: buffer length
 *
 * This function fills elements of change monitor request buffer.
 *
 * Return: CDF status
 */
static QDF_STATUS wmi_get_buf_extscan_change_monitor_cmd(wmi_unified_t wmi_handle,
			struct extscan_set_sig_changereq_params
			*psigchange, wmi_buf_t *buf, int *buf_len)
{
	wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param *cmd;
	wmi_extscan_wlan_change_bssid_param *dest_chglist;
	uint8_t *buf_ptr;
	int j;
	int len = sizeof(*cmd);
	int numap = psigchange->num_ap;
	struct ap_threshold_params *src_ap = psigchange->ap;

	if (!numap) {
		WMI_LOGE("%s: Invalid number of bssid's", __func__);
		return QDF_STATUS_E_INVAL;
	}
	len += WMI_TLV_HDR_SIZE;
	len += numap * sizeof(wmi_extscan_wlan_change_bssid_param);

	*buf = wmi_buf_alloc(wmi_handle, len);
	if (!*buf) {
		WMI_LOGP("%s: failed to allocate memory for change monitor cmd",
			 __func__);
		return QDF_STATUS_E_FAILURE;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(*buf);
	cmd =
		(wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param *)
		buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
	WMITLV_TAG_STRUC_wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_extscan_configure_wlan_change_monitor_cmd_fixed_param));

	cmd->request_id = psigchange->request_id;
	cmd->vdev_id = psigchange->session_id;
	cmd->total_entries = numap;
	cmd->mode = 1;
	cmd->num_entries_in_page = numap;
	cmd->lost_ap_scan_count = psigchange->lostap_sample_size;
	cmd->max_rssi_samples = psigchange->rssi_sample_size;
	cmd->rssi_averaging_samples = psigchange->rssi_sample_size;
	cmd->max_out_of_range_count = psigchange->min_breaching;

	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_STRUC,
		       numap * sizeof(wmi_extscan_wlan_change_bssid_param));
	dest_chglist = (wmi_extscan_wlan_change_bssid_param *)
		       (buf_ptr + WMI_TLV_HDR_SIZE);

	for (j = 0; j < numap; j++) {
		WMITLV_SET_HDR(dest_chglist,
		       WMITLV_TAG_STRUC_wmi_extscan_bucket_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_extscan_wlan_change_bssid_param));

		dest_chglist->lower_rssi_limit = src_ap->low;
		dest_chglist->upper_rssi_limit = src_ap->high;
		WMI_CHAR_ARRAY_TO_MAC_ADDR(src_ap->bssid.bytes,
					   &dest_chglist->bssid);

		WMI_LOGD("%s: min_rssi %d", __func__,
			 dest_chglist->lower_rssi_limit);
		dest_chglist++;
		src_ap++;
	}
	buf_ptr += WMI_TLV_HDR_SIZE +
		   (numap * sizeof(wmi_extscan_wlan_change_bssid_param));
	*buf_len = len;
	return QDF_STATUS_SUCCESS;
}

/**
 * send_extscan_start_change_monitor_cmd_tlv() - send start change monitor cmd
 * @wmi_handle: wmi handle
 * @psigchange: change monitor request params
 *
 * This function sends start change monitor request to fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_extscan_start_change_monitor_cmd_tlv(wmi_unified_t wmi_handle,
			   struct extscan_set_sig_changereq_params *
			   psigchange)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_buf_t buf;
	int len;


	qdf_status = wmi_get_buf_extscan_change_monitor_cmd(wmi_handle,
			     psigchange, &buf,
			     &len);
	if (qdf_status != QDF_STATUS_SUCCESS) {
		WMI_LOGE("%s: Failed to get buffer for change monitor cmd",
			 __func__);
		return QDF_STATUS_E_FAILURE;
	}
	if (!buf) {
		WMI_LOGE("%s: Failed to get buffer", __func__);
		return QDF_STATUS_E_FAILURE;
	}
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
		 WMI_EXTSCAN_CONFIGURE_WLAN_CHANGE_MONITOR_CMDID)) {
		WMI_LOGE("%s: failed to send command", __func__);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_extscan_stop_hotlist_monitor_cmd_tlv() - stop hotlist monitor
 * @wmi_handle: wmi handle
 * @photlist_reset: hotlist reset params
 *
 * This function configures hotlist monitor to stop in fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_extscan_stop_hotlist_monitor_cmd_tlv(wmi_unified_t wmi_handle,
		  struct extscan_bssid_hotlist_reset_params *photlist_reset)
{
	wmi_extscan_configure_hotlist_monitor_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;
	int hotlist_entries = 0;

	len = sizeof(*cmd);

	/* reset bssid hotlist with tlv set to 0 */
	len += WMI_TLV_HDR_SIZE;
	len += hotlist_entries * sizeof(wmi_extscan_hotlist_entry);

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_extscan_configure_hotlist_monitor_cmd_fixed_param *)
	      buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
	WMITLV_TAG_STRUC_wmi_extscan_configure_hotlist_monitor_cmd_fixed_param,
	WMITLV_GET_STRUCT_TLVLEN
	(wmi_extscan_configure_hotlist_monitor_cmd_fixed_param));

	cmd->request_id = photlist_reset->request_id;
	cmd->vdev_id = photlist_reset->session_id;
	cmd->mode = 0;

	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_STRUC,
		       hotlist_entries * sizeof(wmi_extscan_hotlist_entry));
	buf_ptr += WMI_TLV_HDR_SIZE +
		   (hotlist_entries * sizeof(wmi_extscan_hotlist_entry));

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_EXTSCAN_CONFIGURE_HOTLIST_MONITOR_CMDID)) {
		WMI_LOGE("%s: failed to  command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_stop_extscan_cmd_tlv() - stop extscan command to fw.
 * @wmi_handle: wmi handle
 * @pstopcmd: stop scan command request params
 *
 * This function sends stop extscan request to fw.
 *
 * Return: CDF Status.
 */
QDF_STATUS send_stop_extscan_cmd_tlv(wmi_unified_t wmi_handle,
			  struct extscan_stop_req_params *pstopcmd)
{
	wmi_extscan_stop_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;

	len = sizeof(*cmd);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_extscan_stop_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_extscan_stop_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_extscan_stop_cmd_fixed_param));

	cmd->request_id = pstopcmd->request_id;
	cmd->vdev_id = pstopcmd->session_id;

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_EXTSCAN_STOP_CMDID)) {
		WMI_LOGE("%s: failed to  command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * wmi_get_buf_extscan_start_cmd() - Fill extscan start request
 * @wmi_handle: wmi handle
 * @pstart: scan command request params
 * @buf: event buffer
 * @buf_len: length of buffer
 *
 * This function fills individual elements of extscan request and
 * TLV for buckets, channel list.
 *
 * Return: CDF Status.
 */
QDF_STATUS wmi_get_buf_extscan_start_cmd(wmi_unified_t wmi_handle,
			 struct wifi_scan_cmd_req_params *pstart,
			 wmi_buf_t *buf, int *buf_len)
{
	wmi_extscan_start_cmd_fixed_param *cmd;
	wmi_extscan_bucket *dest_blist;
	wmi_extscan_bucket_channel *dest_clist;
	struct wifi_scan_bucket_params *src_bucket = pstart->buckets;
	struct wifi_scan_channelspec_params *src_channel = src_bucket->channels;
	struct wifi_scan_channelspec_params save_channel[WMI_WLAN_EXTSCAN_MAX_CHANNELS];

	uint8_t *buf_ptr;
	int i, k, count = 0;
	int len = sizeof(*cmd);
	int nbuckets = pstart->numBuckets;
	int nchannels = 0;

	/* These TLV's are are NULL by default */
	uint32_t ie_len_with_pad = 0;
	int num_ssid = 0;
	int num_bssid = 0;
	int ie_len = 0;

	uint32_t base_period = pstart->basePeriod;

	/* TLV placeholder for ssid_list (NULL) */
	len += WMI_TLV_HDR_SIZE;
	len += num_ssid * sizeof(wmi_ssid);

	/* TLV placeholder for bssid_list (NULL) */
	len += WMI_TLV_HDR_SIZE;
	len += num_bssid * sizeof(wmi_mac_addr);

	/* TLV placeholder for ie_data (NULL) */
	len += WMI_TLV_HDR_SIZE;
	len += ie_len * sizeof(uint32_t);

	/* TLV placeholder for bucket */
	len += WMI_TLV_HDR_SIZE;
	len += nbuckets * sizeof(wmi_extscan_bucket);

	/* TLV channel placeholder */
	len += WMI_TLV_HDR_SIZE;
	for (i = 0; i < nbuckets; i++) {
		nchannels += src_bucket->numChannels;
		src_bucket++;
	}

	WMI_LOGD("%s: Total buckets: %d total #of channels is %d",
		__func__, nbuckets, nchannels);
	len += nchannels * sizeof(wmi_extscan_bucket_channel);
	/* Allocate the memory */
	*buf = wmi_buf_alloc(wmi_handle, len);
	if (!*buf) {
		WMI_LOGP("%s: failed to allocate memory"
			 " for start extscan cmd", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(*buf);
	cmd = (wmi_extscan_start_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_extscan_start_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_extscan_start_cmd_fixed_param));

	cmd->request_id = pstart->requestId;
	cmd->vdev_id = pstart->sessionId;
	cmd->base_period = pstart->basePeriod;
	cmd->num_buckets = nbuckets;
	cmd->configuration_flags = 0;
	if (pstart->configuration_flags & EXTSCAN_LP_EXTENDED_BATCHING)
		cmd->configuration_flags |= WMI_EXTSCAN_EXTENDED_BATCHING_EN;
	WMI_LOGI("%s: configuration_flags: 0x%x", __func__,
			cmd->configuration_flags);

	cmd->min_rest_time = WMI_EXTSCAN_REST_TIME;
	cmd->max_rest_time = WMI_EXTSCAN_REST_TIME;
	cmd->max_bssids_per_scan_cycle = pstart->maxAPperScan;

	/* The max dwell time is retrieved from the first channel
	 * of the first bucket and kept common for all channels.
	 */
	cmd->min_dwell_time_active = pstart->min_dwell_time_active;
	cmd->max_dwell_time_active = pstart->max_dwell_time_active;
	cmd->min_dwell_time_passive = pstart->min_dwell_time_passive;
	cmd->max_dwell_time_passive = pstart->max_dwell_time_passive;
	cmd->max_bssids_per_scan_cycle = pstart->maxAPperScan;
	cmd->max_table_usage = pstart->report_threshold_percent;
	cmd->report_threshold_num_scans = pstart->report_threshold_num_scans;

	cmd->repeat_probe_time = cmd->max_dwell_time_active /
					WMI_SCAN_NPROBES_DEFAULT;
	cmd->max_scan_time = WMI_EXTSCAN_MAX_SCAN_TIME;
	cmd->probe_delay = 0;
	cmd->probe_spacing_time = 0;
	cmd->idle_time = 0;
	cmd->burst_duration = WMI_EXTSCAN_BURST_DURATION;
	cmd->scan_ctrl_flags = WMI_SCAN_ADD_BCAST_PROBE_REQ |
			       WMI_SCAN_ADD_CCK_RATES |
			       WMI_SCAN_ADD_OFDM_RATES |
			       WMI_SCAN_ADD_SPOOFED_MAC_IN_PROBE_REQ |
			       WMI_SCAN_ADD_DS_IE_IN_PROBE_REQ;
	cmd->scan_priority = WMI_SCAN_PRIORITY_HIGH;
	cmd->num_ssids = 0;
	cmd->num_bssid = 0;
	cmd->ie_len = 0;
	cmd->n_probes = (cmd->repeat_probe_time > 0) ?
			cmd->max_dwell_time_active / cmd->repeat_probe_time : 0;

	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_FIXED_STRUC,
		       num_ssid * sizeof(wmi_ssid));
	buf_ptr += WMI_TLV_HDR_SIZE + (num_ssid * sizeof(wmi_ssid));

	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_FIXED_STRUC,
		       num_bssid * sizeof(wmi_mac_addr));
	buf_ptr += WMI_TLV_HDR_SIZE + (num_bssid * sizeof(wmi_mac_addr));

	ie_len_with_pad = 0;
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE,
			  ie_len_with_pad);
	buf_ptr += WMI_TLV_HDR_SIZE + ie_len_with_pad;

	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_STRUC,
		       nbuckets * sizeof(wmi_extscan_bucket));
	dest_blist = (wmi_extscan_bucket *)
		     (buf_ptr + WMI_TLV_HDR_SIZE);
	src_bucket = pstart->buckets;

	/* Retrieve scanning information from each bucket and
	 * channels and send it to the target
	 */
	for (i = 0; i < nbuckets; i++) {
		WMITLV_SET_HDR(dest_blist,
		      WMITLV_TAG_STRUC_wmi_extscan_bucket_cmd_fixed_param,
		      WMITLV_GET_STRUCT_TLVLEN(wmi_extscan_bucket));

		dest_blist->bucket_id = src_bucket->bucket;
		dest_blist->base_period_multiplier =
			src_bucket->period / base_period;
		dest_blist->min_period = src_bucket->period;
		dest_blist->max_period = src_bucket->max_period;
		dest_blist->exp_backoff = src_bucket->exponent;
		dest_blist->exp_max_step_count = src_bucket->step_count;
		dest_blist->channel_band = src_bucket->band;
		dest_blist->num_channels = src_bucket->numChannels;
		dest_blist->notify_extscan_events = 0;

		if (src_bucket->reportEvents & WMI_EXTSCAN_REPORT_EVENTS_EACH_SCAN)
			dest_blist->notify_extscan_events =
					WMI_EXTSCAN_BUCKET_COMPLETED_EVENT;

		if (src_bucket->reportEvents &
				WMI_EXTSCAN_REPORT_EVENTS_FULL_RESULTS) {
			dest_blist->forwarding_flags =
				WMI_EXTSCAN_FORWARD_FRAME_TO_HOST;
			dest_blist->notify_extscan_events |=
				WMI_EXTSCAN_BUCKET_COMPLETED_EVENT |
				WMI_EXTSCAN_CYCLE_STARTED_EVENT |
				WMI_EXTSCAN_CYCLE_COMPLETED_EVENT;
		} else {
			dest_blist->forwarding_flags =
				WMI_EXTSCAN_NO_FORWARDING;
		}

		if (src_bucket->reportEvents & WMI_EXTSCAN_REPORT_EVENTS_NO_BATCH)
			dest_blist->configuration_flags = 0;
		else
			dest_blist->configuration_flags =
				WMI_EXTSCAN_BUCKET_CACHE_RESULTS;

		WMI_LOGI("%s: ntfy_extscan_events:%u cfg_flags:%u fwd_flags:%u",
			__func__, dest_blist->notify_extscan_events,
			dest_blist->configuration_flags,
			dest_blist->forwarding_flags);

		dest_blist->min_dwell_time_active =
				   src_bucket->min_dwell_time_active;
		dest_blist->max_dwell_time_active =
				   src_bucket->max_dwell_time_active;
		dest_blist->min_dwell_time_passive =
				   src_bucket->min_dwell_time_passive;
		dest_blist->max_dwell_time_passive =
				   src_bucket->max_dwell_time_passive;
		src_channel = src_bucket->channels;

		/* save the channel info to later populate
		 * the  channel TLV
		 */
		for (k = 0; k < src_bucket->numChannels; k++) {
			save_channel[count++].channel = src_channel->channel;
			src_channel++;
		}
		dest_blist++;
		src_bucket++;
	}
	buf_ptr += WMI_TLV_HDR_SIZE + (nbuckets * sizeof(wmi_extscan_bucket));
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_STRUC,
		       nchannels * sizeof(wmi_extscan_bucket_channel));
	dest_clist = (wmi_extscan_bucket_channel *)
		     (buf_ptr + WMI_TLV_HDR_SIZE);

	/* Active or passive scan is based on the bucket dwell time
	 * and channel specific active,passive scans are not
	 * supported yet
	 */
	for (i = 0; i < nchannels; i++) {
		WMITLV_SET_HDR(dest_clist,
		WMITLV_TAG_STRUC_wmi_extscan_bucket_channel_event_fixed_param,
			   WMITLV_GET_STRUCT_TLVLEN
			   (wmi_extscan_bucket_channel));
		dest_clist->channel = save_channel[i].channel;
		dest_clist++;
	}
	buf_ptr += WMI_TLV_HDR_SIZE +
		   (nchannels * sizeof(wmi_extscan_bucket_channel));
	*buf_len = len;
	return QDF_STATUS_SUCCESS;
}

/**
 * send_start_extscan_cmd_tlv() - start extscan command to fw.
 * @wmi_handle: wmi handle
 * @pstart: scan command request params
 *
 * This function sends start extscan request to fw.
 *
 * Return: CDF Status.
 */
QDF_STATUS send_start_extscan_cmd_tlv(wmi_unified_t wmi_handle,
			  struct wifi_scan_cmd_req_params *pstart)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_buf_t buf;
	int len;

	/* Fill individual elements of extscan request and
	 * TLV for buckets, channel list.
	 */
	qdf_status = wmi_get_buf_extscan_start_cmd(wmi_handle,
			     pstart, &buf, &len);
	if (qdf_status != QDF_STATUS_SUCCESS) {
		WMI_LOGE("%s: Failed to get buffer for ext scan cmd", __func__);
		return QDF_STATUS_E_FAILURE;
	}
	if (!buf) {
		WMI_LOGE("%s:Failed to get buffer"
			 "for current extscan info", __func__);
		return QDF_STATUS_E_FAILURE;
	}
	if (wmi_unified_cmd_send(wmi_handle, buf,
				 len, WMI_EXTSCAN_START_CMDID)) {
		WMI_LOGE("%s: failed to send command", __func__);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_plm_stop_cmd_tlv() - plm stop request
 * @wmi_handle: wmi handle
 * @plm: plm request parameters
 *
 * This function request FW to stop PLM.
 *
 * Return: CDF status
 */
QDF_STATUS send_plm_stop_cmd_tlv(wmi_unified_t wmi_handle,
			  const struct plm_req_params *plm)
{
	wmi_vdev_plmreq_stop_cmd_fixed_param *cmd;
	int32_t len;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int ret;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_vdev_plmreq_stop_cmd_fixed_param *) wmi_buf_data(buf);

	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_plmreq_stop_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_vdev_plmreq_stop_cmd_fixed_param));

	cmd->vdev_id = plm->session_id;

	cmd->meas_token = plm->meas_token;
	WMI_LOGD("vdev %d meas token %d", cmd->vdev_id, cmd->meas_token);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_VDEV_PLMREQ_STOP_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send plm stop wmi cmd", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_plm_start_cmd_tlv() - plm start request
 * @wmi_handle: wmi handle
 * @plm: plm request parameters
 *
 * This function request FW to start PLM.
 *
 * Return: CDF status
 */
QDF_STATUS send_plm_start_cmd_tlv(wmi_unified_t wmi_handle,
			  const struct plm_req_params *plm,
			  uint32_t *gchannel_list)
{
	wmi_vdev_plmreq_start_cmd_fixed_param *cmd;
	uint32_t *channel_list;
	int32_t len;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint8_t count;
	int ret;

	/* TLV place holder for channel_list */
	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE;
	len += sizeof(uint32_t) * plm->plm_num_ch;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_vdev_plmreq_start_cmd_fixed_param *) wmi_buf_data(buf);

	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_plmreq_start_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_plmreq_start_cmd_fixed_param));

	cmd->vdev_id = plm->session_id;

	cmd->meas_token = plm->meas_token;
	cmd->dialog_token = plm->diag_token;
	cmd->number_bursts = plm->num_bursts;
	cmd->burst_interval = WMI_SEC_TO_MSEC(plm->burst_int);
	cmd->off_duration = plm->meas_duration;
	cmd->burst_cycle = plm->burst_len;
	cmd->tx_power = plm->desired_tx_pwr;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(plm->mac_addr.bytes, &cmd->dest_mac);
	cmd->num_chans = plm->plm_num_ch;

	buf_ptr += sizeof(wmi_vdev_plmreq_start_cmd_fixed_param);

	WMI_LOGD("vdev : %d measu token : %d", cmd->vdev_id, cmd->meas_token);
	WMI_LOGD("dialog_token: %d", cmd->dialog_token);
	WMI_LOGD("number_bursts: %d", cmd->number_bursts);
	WMI_LOGD("burst_interval: %d", cmd->burst_interval);
	WMI_LOGD("off_duration: %d", cmd->off_duration);
	WMI_LOGD("burst_cycle: %d", cmd->burst_cycle);
	WMI_LOGD("tx_power: %d", cmd->tx_power);
	WMI_LOGD("Number of channels : %d", cmd->num_chans);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
		       (cmd->num_chans * sizeof(uint32_t)));

	buf_ptr += WMI_TLV_HDR_SIZE;
	if (cmd->num_chans) {
		channel_list = (uint32_t *) buf_ptr;
		for (count = 0; count < cmd->num_chans; count++) {
			channel_list[count] = plm->plm_ch_list[count];
			if (channel_list[count] < WMI_NLO_FREQ_THRESH)
				channel_list[count] =
					gchannel_list[count];
			WMI_LOGD("Ch[%d]: %d MHz", count, channel_list[count]);
		}
		buf_ptr += cmd->num_chans * sizeof(uint32_t);
	}

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_VDEV_PLMREQ_START_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send plm start wmi cmd", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_pno_stop_cmd_tlv() - PNO stop request
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * This function request FW to stop ongoing PNO operation.
 *
 * Return: CDF status
 */
QDF_STATUS send_pno_stop_cmd_tlv(wmi_unified_t wmi_handle, uint8_t vdev_id)
{
	wmi_nlo_config_cmd_fixed_param *cmd;
	int32_t len = sizeof(*cmd);
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int ret;

	/*
	 * TLV place holder for array of structures nlo_configured_parameters
	 * TLV place holder for array of uint32_t channel_list
	 * TLV place holder for chnl prediction cfg
	 */
	len += WMI_TLV_HDR_SIZE + WMI_TLV_HDR_SIZE + WMI_TLV_HDR_SIZE;
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_nlo_config_cmd_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_nlo_config_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_nlo_config_cmd_fixed_param));

	cmd->vdev_id = vdev_id;
	cmd->flags = WMI_NLO_CONFIG_STOP;
	buf_ptr += sizeof(*cmd);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;


	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_NETWORK_LIST_OFFLOAD_CONFIG_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send nlo wmi cmd", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_pno_start_cmd_tlv() - PNO start request
 * @wmi_handle: wmi handle
 * @pno: PNO request
 *
 * This function request FW to start PNO request.
 * Request: CDF status
 */
QDF_STATUS send_pno_start_cmd_tlv(wmi_unified_t wmi_handle,
		   struct pno_scan_req_params *pno,
		   uint32_t *gchannel_freq_list)
{
	wmi_nlo_config_cmd_fixed_param *cmd;
	nlo_configured_parameters *nlo_list;
	uint32_t *channel_list;
	int32_t len;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint8_t i;
	int ret;

	/*
	 * TLV place holder for array nlo_configured_parameters(nlo_list)
	 * TLV place holder for array of uint32_t channel_list
	 * TLV place holder for chnnl prediction cfg
	 */
	len = sizeof(*cmd) +
		WMI_TLV_HDR_SIZE + WMI_TLV_HDR_SIZE + WMI_TLV_HDR_SIZE;

	len += sizeof(uint32_t) * QDF_MIN(pno->aNetworks[0].ucChannelCount,
					  WMI_NLO_MAX_CHAN);
	len += sizeof(nlo_configured_parameters) *
	       QDF_MIN(pno->ucNetworksCount, WMI_NLO_MAX_SSIDS);
	len += sizeof(nlo_channel_prediction_cfg);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_nlo_config_cmd_fixed_param *) wmi_buf_data(buf);

	buf_ptr = (uint8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_nlo_config_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_nlo_config_cmd_fixed_param));
	cmd->vdev_id = pno->sessionId;
	cmd->flags = WMI_NLO_CONFIG_START | WMI_NLO_CONFIG_SSID_HIDE_EN;

	/* Current FW does not support min-max range for dwell time */
	cmd->active_dwell_time = pno->active_max_time;
	cmd->passive_dwell_time = pno->passive_max_time;

	/* Copy scan interval */
	cmd->fast_scan_period = pno->fast_scan_period;
	cmd->slow_scan_period = pno->slow_scan_period;
	cmd->fast_scan_max_cycles = pno->fast_scan_max_cycles;
	WMI_LOGD("fast_scan_period: %d msec slow_scan_period: %d msec",
			cmd->fast_scan_period, cmd->slow_scan_period);
	WMI_LOGD("fast_scan_max_cycles: %d", cmd->fast_scan_max_cycles);

	buf_ptr += sizeof(wmi_nlo_config_cmd_fixed_param);

	cmd->no_of_ssids = QDF_MIN(pno->ucNetworksCount, WMI_NLO_MAX_SSIDS);
	WMI_LOGD("SSID count : %d", cmd->no_of_ssids);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       cmd->no_of_ssids * sizeof(nlo_configured_parameters));
	buf_ptr += WMI_TLV_HDR_SIZE;

	nlo_list = (nlo_configured_parameters *) buf_ptr;
	for (i = 0; i < cmd->no_of_ssids; i++) {
		WMITLV_SET_HDR(&nlo_list[i].tlv_header,
			       WMITLV_TAG_ARRAY_BYTE,
			       WMITLV_GET_STRUCT_TLVLEN
				       (nlo_configured_parameters));
		/* Copy ssid and it's length */
		nlo_list[i].ssid.valid = true;
		nlo_list[i].ssid.ssid.ssid_len = pno->aNetworks[i].ssid.length;
		qdf_mem_copy(nlo_list[i].ssid.ssid.ssid,
			     pno->aNetworks[i].ssid.mac_ssid,
			     nlo_list[i].ssid.ssid.ssid_len);
		WMI_LOGD("index: %d ssid: %.*s len: %d", i,
			 nlo_list[i].ssid.ssid.ssid_len,
			 (char *)nlo_list[i].ssid.ssid.ssid,
			 nlo_list[i].ssid.ssid.ssid_len);

		/* Copy rssi threshold */
		if (pno->aNetworks[i].rssiThreshold &&
		    pno->aNetworks[i].rssiThreshold > WMI_RSSI_THOLD_DEFAULT) {
			nlo_list[i].rssi_cond.valid = true;
			nlo_list[i].rssi_cond.rssi =
				pno->aNetworks[i].rssiThreshold;
			WMI_LOGD("RSSI threshold : %d dBm",
				 nlo_list[i].rssi_cond.rssi);
		}
		nlo_list[i].bcast_nw_type.valid = true;
		nlo_list[i].bcast_nw_type.bcast_nw_type =
			pno->aNetworks[i].bcastNetwType;
		WMI_LOGI("Broadcast NW type (%u)",
			 nlo_list[i].bcast_nw_type.bcast_nw_type);
	}
	buf_ptr += cmd->no_of_ssids * sizeof(nlo_configured_parameters);

	/* Copy channel info */
	cmd->num_of_channels = QDF_MIN(pno->aNetworks[0].ucChannelCount,
				       WMI_NLO_MAX_CHAN);
	WMI_LOGD("Channel count: %d", cmd->num_of_channels);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
		       (cmd->num_of_channels * sizeof(uint32_t)));
	buf_ptr += WMI_TLV_HDR_SIZE;

	channel_list = (uint32_t *) buf_ptr;
	for (i = 0; i < cmd->num_of_channels; i++) {
		channel_list[i] = pno->aNetworks[0].aChannels[i];

		if (channel_list[i] < WMI_NLO_FREQ_THRESH)
			channel_list[i] = gchannel_freq_list[i];

		WMI_LOGD("Ch[%d]: %d MHz", i, channel_list[i]);
	}
	buf_ptr += cmd->num_of_channels * sizeof(uint32_t);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
			sizeof(nlo_channel_prediction_cfg));
	buf_ptr += WMI_TLV_HDR_SIZE;
	buf_ptr += WMI_TLV_HDR_SIZE;
	/** TODO: Discrete firmware doesn't have command/option to configure
	 * App IE which comes from wpa_supplicant as of part PNO start request.
	 */
	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_NETWORK_LIST_OFFLOAD_CONFIG_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send nlo wmi cmd", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/* send_set_ric_req_cmd_tlv() - set ric request element
 * @wmi_handle: wmi handle
 * @msg: message
 * @is_add_ts: is addts required
 *
 * This function sets ric request element for 11r roaming.
 *
 * Return: CDF status
 */
QDF_STATUS send_set_ric_req_cmd_tlv(wmi_unified_t wmi_handle,
			void *msg, uint8_t is_add_ts)
{
	wmi_ric_request_fixed_param *cmd;
	wmi_ric_tspec *tspec_param;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	struct mac_tspec_ie *ptspecIE;
	int32_t len = sizeof(wmi_ric_request_fixed_param) +
		      WMI_TLV_HDR_SIZE + sizeof(wmi_ric_tspec);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);

	cmd = (wmi_ric_request_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		   WMITLV_TAG_STRUC_wmi_ric_request_fixed_param,
		   WMITLV_GET_STRUCT_TLVLEN(wmi_ric_request_fixed_param));
	if (is_add_ts)
		cmd->vdev_id = ((struct add_ts_param *) msg)->sessionId;
	else
		cmd->vdev_id = ((struct del_ts_params *) msg)->sessionId;
	cmd->num_ric_request = 1;
	cmd->is_add_ric = is_add_ts;

	buf_ptr += sizeof(wmi_ric_request_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, sizeof(wmi_ric_tspec));

	buf_ptr += WMI_TLV_HDR_SIZE;
	tspec_param = (wmi_ric_tspec *) buf_ptr;
	WMITLV_SET_HDR(&tspec_param->tlv_header,
		       WMITLV_TAG_STRUC_wmi_ric_tspec,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_ric_tspec));

	if (is_add_ts)
		ptspecIE = &(((struct add_ts_param *) msg)->tspec);
	else
		ptspecIE = &(((struct del_ts_params *) msg)->delTsInfo.tspec);

	/* Fill the tsinfo in the format expected by firmware */
#ifndef ANI_LITTLE_BIT_ENDIAN
	qdf_mem_copy(((uint8_t *) &tspec_param->ts_info) + 1,
		     ((uint8_t *) &ptspecIE->tsinfo) + 1, 2);
#else
	qdf_mem_copy(((uint8_t *) &tspec_param->ts_info),
		     ((uint8_t *) &ptspecIE->tsinfo) + 1, 2);
#endif /* ANI_LITTLE_BIT_ENDIAN */

	tspec_param->nominal_msdu_size = ptspecIE->nomMsduSz;
	tspec_param->maximum_msdu_size = ptspecIE->maxMsduSz;
	tspec_param->min_service_interval = ptspecIE->minSvcInterval;
	tspec_param->max_service_interval = ptspecIE->maxSvcInterval;
	tspec_param->inactivity_interval = ptspecIE->inactInterval;
	tspec_param->suspension_interval = ptspecIE->suspendInterval;
	tspec_param->svc_start_time = ptspecIE->svcStartTime;
	tspec_param->min_data_rate = ptspecIE->minDataRate;
	tspec_param->mean_data_rate = ptspecIE->meanDataRate;
	tspec_param->peak_data_rate = ptspecIE->peakDataRate;
	tspec_param->max_burst_size = ptspecIE->maxBurstSz;
	tspec_param->delay_bound = ptspecIE->delayBound;
	tspec_param->min_phy_rate = ptspecIE->minPhyRate;
	tspec_param->surplus_bw_allowance = ptspecIE->surplusBw;
	tspec_param->medium_time = 0;

	WMI_LOGI("%s: Set RIC Req is_add_ts:%d", __func__, is_add_ts);

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_ROAM_SET_RIC_REQUEST_CMDID)) {
		WMI_LOGP("%s: Failed to send vdev Set RIC Req command",
			 __func__);
		if (is_add_ts)
			((struct add_ts_param *) msg)->status =
					    QDF_STATUS_E_FAILURE;
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_process_ll_stats_clear_cmd_tlv() - clear link layer stats
 * @wmi_handle: wmi handle
 * @clear_req: ll stats clear request command params
 *
 * Return: QDF_STATUS_SUCCESS for success or error code
 */
QDF_STATUS send_process_ll_stats_clear_cmd_tlv(wmi_unified_t wmi_handle,
		const struct ll_stats_clear_params *clear_req,
		uint8_t addr[IEEE80211_ADDR_LEN])
{
	wmi_clear_link_stats_cmd_fixed_param *cmd;
	int32_t len;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int ret;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);

	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);
	cmd = (wmi_clear_link_stats_cmd_fixed_param *) buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_clear_link_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_clear_link_stats_cmd_fixed_param));

	cmd->stop_stats_collection_req = clear_req->stop_req;
	cmd->vdev_id = clear_req->sta_id;
	cmd->stats_clear_req_mask = clear_req->stats_clear_mask;

	WMI_CHAR_ARRAY_TO_MAC_ADDR(addr,
				   &cmd->peer_macaddr);

	WMI_LOGD("LINK_LAYER_STATS - Clear Request Params");
	WMI_LOGD("StopReq         : %d", cmd->stop_stats_collection_req);
	WMI_LOGD("Vdev Id         : %d", cmd->vdev_id);
	WMI_LOGD("Clear Stat Mask : %d", cmd->stats_clear_req_mask);
	/* WMI_LOGD("Peer MAC Addr   : %pM",
		 cmd->peer_macaddr); */

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_CLEAR_LINK_STATS_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send clear link stats req", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGD("Clear Link Layer Stats request sent successfully");
	return QDF_STATUS_SUCCESS;
}

/**
 * send_process_ll_stats_set_cmd_tlv() - link layer stats set request
 * @wmi_handle:       wmi handle
 * @setReq:  ll stats set request command params
 *
 * Return: QDF_STATUS_SUCCESS for success or error code
 */
QDF_STATUS send_process_ll_stats_set_cmd_tlv(wmi_unified_t wmi_handle,
		const struct ll_stats_set_params *set_req)
{
	wmi_start_link_stats_cmd_fixed_param *cmd;
	int32_t len;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int ret;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);

	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);
	cmd = (wmi_start_link_stats_cmd_fixed_param *) buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_start_link_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_start_link_stats_cmd_fixed_param));

	cmd->mpdu_size_threshold = set_req->mpdu_size_threshold;
	cmd->aggressive_statistics_gathering =
		set_req->aggressive_statistics_gathering;

	WMI_LOGD("LINK_LAYER_STATS - Start/Set Request Params");
	WMI_LOGD("MPDU Size Thresh : %d", cmd->mpdu_size_threshold);
	WMI_LOGD("Aggressive Gather: %d", cmd->aggressive_statistics_gathering);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_START_LINK_STATS_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send set link stats request", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_process_ll_stats_get_cmd_tlv() - link layer stats get request
 * @wmi_handle:wmi handle
 * @get_req:ll stats get request command params
 * @addr: mac address
 *
 * Return: QDF_STATUS_SUCCESS for success or error code
 */
QDF_STATUS send_process_ll_stats_get_cmd_tlv(wmi_unified_t wmi_handle,
		 const struct ll_stats_get_params  *get_req,
		 uint8_t addr[IEEE80211_ADDR_LEN])
{
	wmi_request_link_stats_cmd_fixed_param *cmd;
	int32_t len;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int ret;

	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);
	cmd = (wmi_request_link_stats_cmd_fixed_param *) buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_request_link_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_request_link_stats_cmd_fixed_param));

	cmd->request_id = get_req->req_id;
	cmd->stats_type = get_req->param_id_mask;
	cmd->vdev_id = get_req->sta_id;

	WMI_CHAR_ARRAY_TO_MAC_ADDR(addr,
				   &cmd->peer_macaddr);

	WMI_LOGD("LINK_LAYER_STATS - Get Request Params");
	WMI_LOGD("Request ID      : %d", cmd->request_id);
	WMI_LOGD("Stats Type      : %d", cmd->stats_type);
	WMI_LOGD("Vdev ID         : %d", cmd->vdev_id);
	WMI_LOGD("Peer MAC Addr   : %pM", addr);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_REQUEST_LINK_STATS_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send get link stats request", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_get_stats_cmd_tlv() - get stats request
 * @wmi_handle: wmi handle
 * @get_stats_param: stats params
 * @addr: mac address
 *
 * Return: CDF status
 */
QDF_STATUS send_get_stats_cmd_tlv(wmi_unified_t wmi_handle,
		       struct pe_stats_req  *get_stats_param,
			   uint8_t addr[IEEE80211_ADDR_LEN])
{
	wmi_buf_t buf;
	wmi_request_stats_cmd_fixed_param *cmd;
	uint8_t len = sizeof(wmi_request_stats_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed to allocate wmi buffer", __func__);
		return QDF_STATUS_E_FAILURE;
	}


	cmd = (wmi_request_stats_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_request_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_request_stats_cmd_fixed_param));
	cmd->stats_id =
		WMI_REQUEST_PEER_STAT | WMI_REQUEST_PDEV_STAT |
		WMI_REQUEST_VDEV_STAT;
	cmd->vdev_id = get_stats_param->session_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(addr, &cmd->peer_macaddr);
	WMI_LOGD("STATS REQ VDEV_ID:%d-->", cmd->vdev_id);
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_REQUEST_STATS_CMDID)) {

		WMI_LOGE("%s: Failed to send WMI_REQUEST_STATS_CMDID",
			 __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;

}

/**
 * send_snr_request_cmd_tlv() - send request to fw to get RSSI stats
 * @wmi_handle: wmi handle
 * @rssi_req: get RSSI request
 *
 * Return: CDF status
 */
QDF_STATUS send_snr_request_cmd_tlv(wmi_unified_t wmi_handle)
{
	wmi_buf_t buf;
	wmi_request_stats_cmd_fixed_param *cmd;
	uint8_t len = sizeof(wmi_request_stats_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_request_stats_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_request_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_request_stats_cmd_fixed_param));
	cmd->stats_id = WMI_REQUEST_VDEV_STAT;
	if (wmi_unified_cmd_send
		    (wmi_handle, buf, len, WMI_REQUEST_STATS_CMDID)) {
		WMI_LOGE("Failed to send host stats request to fw");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_snr_cmd_tlv() - get RSSI from fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: CDF status
 */
QDF_STATUS send_snr_cmd_tlv(wmi_unified_t wmi_handle, uint8_t vdev_id)
{
	wmi_buf_t buf;
	wmi_request_stats_cmd_fixed_param *cmd;
	uint8_t len = sizeof(wmi_request_stats_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_request_stats_cmd_fixed_param *) wmi_buf_data(buf);
	cmd->vdev_id = vdev_id;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_request_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_request_stats_cmd_fixed_param));
	cmd->stats_id = WMI_REQUEST_VDEV_STAT;
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_REQUEST_STATS_CMDID)) {
		WMI_LOGE("Failed to send host stats request to fw");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_link_status_req_cmd_tlv() - process link status request from UMAC
 * @wmi_handle: wmi handle
 * @link_status: get link params
 *
 * Return: CDF status
 */
QDF_STATUS send_link_status_req_cmd_tlv(wmi_unified_t wmi_handle,
				 struct link_status_params *link_status)
{
	wmi_buf_t buf;
	wmi_request_stats_cmd_fixed_param *cmd;
	uint8_t len = sizeof(wmi_request_stats_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_request_stats_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_request_stats_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_request_stats_cmd_fixed_param));
	cmd->stats_id = WMI_REQUEST_VDEV_RATE_STAT;
	cmd->vdev_id = link_status->session_id;
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_REQUEST_STATS_CMDID)) {
		WMI_LOGE("Failed to send WMI link  status request to fw");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

#ifdef FEATURE_WLAN_LPHB

/**
 * send_lphb_config_hbenable_cmd_tlv() - enable command of LPHB configuration
 * @wmi_handle: wmi handle
 * @lphb_conf_req: configuration info
 *
 * Return: CDF status
 */
QDF_STATUS send_lphb_config_hbenable_cmd_tlv(wmi_unified_t wmi_handle,
				wmi_hb_set_enable_cmd_fixed_param *params)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_hb_set_enable_cmd_fixed_param *hb_enable_fp;
	int len = sizeof(wmi_hb_set_enable_cmd_fixed_param);


	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	hb_enable_fp = (wmi_hb_set_enable_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&hb_enable_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_hb_set_enable_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_hb_set_enable_cmd_fixed_param));

	/* fill in values */
	hb_enable_fp->vdev_id = params->session;
	hb_enable_fp->enable = params->enable;
	hb_enable_fp->item = params->item;
	hb_enable_fp->session = params->session;

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_HB_SET_ENABLE_CMDID);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_HB_SET_ENABLE returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	return QDF_STATUS_SUCCESS;
error:
	return qdf_status;
}

/**
 * send_lphb_config_tcp_params_cmd_tlv() - set tcp params of LPHB configuration
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
QDF_STATUS send_lphb_config_tcp_params_cmd_tlv(wmi_unified_t wmi_handle,
	    wmi_hb_set_tcp_params_cmd_fixed_param *lphb_conf_req)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_hb_set_tcp_params_cmd_fixed_param *hb_tcp_params_fp;
	int len = sizeof(wmi_hb_set_tcp_params_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	hb_tcp_params_fp = (wmi_hb_set_tcp_params_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&hb_tcp_params_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_hb_set_tcp_params_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_hb_set_tcp_params_cmd_fixed_param));

	/* fill in values */
	hb_tcp_params_fp->vdev_id = lphb_conf_req->vdev_id;
	hb_tcp_params_fp->srv_ip = lphb_conf_req->srv_ip;
	hb_tcp_params_fp->dev_ip = lphb_conf_req->dev_ip;
	hb_tcp_params_fp->seq = lphb_conf_req->seq;
	hb_tcp_params_fp->src_port = lphb_conf_req->src_port;
	hb_tcp_params_fp->dst_port = lphb_conf_req->dst_port;
	hb_tcp_params_fp->interval = lphb_conf_req->interval;
	hb_tcp_params_fp->timeout = lphb_conf_req->timeout;
	hb_tcp_params_fp->session = lphb_conf_req->session;
	qdf_mem_copy(&hb_tcp_params_fp->gateway_mac,
				   &lphb_conf_req->gateway_mac,
				   sizeof(hb_tcp_params_fp->gateway_mac));

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_HB_SET_TCP_PARAMS_CMDID);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_HB_SET_TCP_PARAMS returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	return QDF_STATUS_SUCCESS;
error:
	return qdf_status;
}

/**
 * send_lphb_config_tcp_pkt_filter_cmd_tlv() - configure tcp packet filter cmd
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
QDF_STATUS send_lphb_config_tcp_pkt_filter_cmd_tlv(wmi_unified_t wmi_handle,
		wmi_hb_set_tcp_pkt_filter_cmd_fixed_param *g_hb_tcp_filter_fp)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_hb_set_tcp_pkt_filter_cmd_fixed_param *hb_tcp_filter_fp;
	int len = sizeof(wmi_hb_set_tcp_pkt_filter_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	hb_tcp_filter_fp =
		(wmi_hb_set_tcp_pkt_filter_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&hb_tcp_filter_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_hb_set_tcp_pkt_filter_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_hb_set_tcp_pkt_filter_cmd_fixed_param));

	/* fill in values */
	hb_tcp_filter_fp->vdev_id = g_hb_tcp_filter_fp->vdev_id;
	hb_tcp_filter_fp->length = g_hb_tcp_filter_fp->length;
	hb_tcp_filter_fp->offset = g_hb_tcp_filter_fp->offset;
	hb_tcp_filter_fp->session = g_hb_tcp_filter_fp->session;
	memcpy((void *)&hb_tcp_filter_fp->filter,
	       (void *)&g_hb_tcp_filter_fp->filter,
	       WMI_WLAN_HB_MAX_FILTER_SIZE);

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_HB_SET_TCP_PKT_FILTER_CMDID);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_HB_SET_TCP_PKT_FILTER returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	return QDF_STATUS_SUCCESS;
error:
	return qdf_status;
}

/**
 * send_lphb_config_udp_params_cmd_tlv() - configure udp param command of LPHB
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
QDF_STATUS send_lphb_config_udp_params_cmd_tlv(wmi_unified_t wmi_handle,
		   wmi_hb_set_udp_params_cmd_fixed_param *lphb_conf_req)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_hb_set_udp_params_cmd_fixed_param *hb_udp_params_fp;
	int len = sizeof(wmi_hb_set_udp_params_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	hb_udp_params_fp = (wmi_hb_set_udp_params_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&hb_udp_params_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_hb_set_udp_params_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_hb_set_udp_params_cmd_fixed_param));

	/* fill in values */
	hb_udp_params_fp->vdev_id = lphb_conf_req->vdev_id;
	hb_udp_params_fp->srv_ip = lphb_conf_req->srv_ip;
	hb_udp_params_fp->dev_ip = lphb_conf_req->dev_ip;
	hb_udp_params_fp->src_port = lphb_conf_req->src_port;
	hb_udp_params_fp->dst_port = lphb_conf_req->dst_port;
	hb_udp_params_fp->interval = lphb_conf_req->interval;
	hb_udp_params_fp->timeout = lphb_conf_req->timeout;
	hb_udp_params_fp->session = lphb_conf_req->session;
	qdf_mem_copy(&hb_udp_params_fp->gateway_mac,
				   &lphb_conf_req->gateway_mac,
				   sizeof(lphb_conf_req->gateway_mac));

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_HB_SET_UDP_PARAMS_CMDID);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_HB_SET_UDP_PARAMS returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	return QDF_STATUS_SUCCESS;
error:
	return qdf_status;
}

/**
 * send_lphb_config_udp_pkt_filter_cmd_tlv() - configure udp pkt filter command
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
QDF_STATUS send_lphb_config_udp_pkt_filter_cmd_tlv(wmi_unified_t wmi_handle,
		wmi_hb_set_udp_pkt_filter_cmd_fixed_param *lphb_conf_req)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_hb_set_udp_pkt_filter_cmd_fixed_param *hb_udp_filter_fp;
	int len = sizeof(wmi_hb_set_udp_pkt_filter_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	hb_udp_filter_fp =
		(wmi_hb_set_udp_pkt_filter_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&hb_udp_filter_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_hb_set_udp_pkt_filter_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_hb_set_udp_pkt_filter_cmd_fixed_param));

	/* fill in values */
	hb_udp_filter_fp->vdev_id = lphb_conf_req->vdev_id;
	hb_udp_filter_fp->length = lphb_conf_req->length;
	hb_udp_filter_fp->offset = lphb_conf_req->offset;
	hb_udp_filter_fp->session = lphb_conf_req->session;
	memcpy((void *)&hb_udp_filter_fp->filter,
	       (void *)&lphb_conf_req->filter,
	       WMI_WLAN_HB_MAX_FILTER_SIZE);

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_HB_SET_UDP_PKT_FILTER_CMDID);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_HB_SET_UDP_PKT_FILTER returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	return QDF_STATUS_SUCCESS;
error:
	return qdf_status;
}
#endif /* FEATURE_WLAN_LPHB */

/**
 * send_process_dhcp_ind_cmd_tlv() - process dhcp indication from SME
 * @wmi_handle: wmi handle
 * @ta_dhcp_ind: DHCP indication parameter
 *
 * Return: CDF Status
 */
QDF_STATUS send_process_dhcp_ind_cmd_tlv(wmi_unified_t wmi_handle,
				wmi_peer_set_param_cmd_fixed_param *ta_dhcp_ind)
{
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_peer_set_param_cmd_fixed_param *peer_set_param_fp;
	int len = sizeof(wmi_peer_set_param_cmd_fixed_param);


	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	peer_set_param_fp = (wmi_peer_set_param_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&peer_set_param_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_peer_set_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_peer_set_param_cmd_fixed_param));

	/* fill in values */
	peer_set_param_fp->vdev_id = ta_dhcp_ind->vdev_id;
	peer_set_param_fp->param_id = ta_dhcp_ind->param_id;
	peer_set_param_fp->param_value = ta_dhcp_ind->param_value;
	qdf_mem_copy(&peer_set_param_fp->peer_macaddr,
				   &ta_dhcp_ind->peer_macaddr,
				   sizeof(ta_dhcp_ind->peer_macaddr));

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_PEER_SET_PARAM_CMDID);
	if (status != EOK) {
		WMI_LOGE("%s: wmi_unified_cmd_send WMI_PEER_SET_PARAM_CMD"
			 " returned Error %d", __func__, status);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_get_link_speed_cmd_tlv() -send command to get linkspeed
 * @wmi_handle: wmi handle
 * @pLinkSpeed: link speed info
 *
 * Return: CDF status
 */
QDF_STATUS send_get_link_speed_cmd_tlv(wmi_unified_t wmi_handle,
		wmi_mac_addr peer_macaddr)
{
	wmi_peer_get_estimated_linkspeed_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;

	len = sizeof(wmi_peer_get_estimated_linkspeed_cmd_fixed_param);
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (wmi_peer_get_estimated_linkspeed_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_wmi_peer_get_estimated_linkspeed_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_peer_get_estimated_linkspeed_cmd_fixed_param));

	/* Copy the peer macaddress to the wma buffer */
	qdf_mem_copy(&cmd->peer_macaddr,
				   &peer_macaddr,
				   sizeof(peer_macaddr));


	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_PEER_GET_ESTIMATED_LINKSPEED_CMDID)) {
		WMI_LOGE("%s: failed to send link speed command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_egap_conf_params_cmd_tlv() - send wmi cmd of egap configuration params
 * @wmi_handle:	 wmi handler
 * @egap_params: pointer to egap_params
 *
 * Return:	 0 for success, otherwise appropriate error code
 */
QDF_STATUS send_egap_conf_params_cmd_tlv(wmi_unified_t wmi_handle,
		     wmi_ap_ps_egap_param_cmd_fixed_param *egap_params)
{
	wmi_ap_ps_egap_param_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t err;

	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send ap_ps_egap cmd");
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_ap_ps_egap_param_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_ap_ps_egap_param_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
			       wmi_ap_ps_egap_param_cmd_fixed_param));

	cmd->enable = egap_params->enable;
	cmd->inactivity_time = egap_params->inactivity_time;
	cmd->wait_time = egap_params->wait_time;
	cmd->flags = egap_params->flags;
	err = wmi_unified_cmd_send(wmi_handle, buf,
				   sizeof(*cmd), WMI_AP_PS_EGAP_PARAM_CMDID);
	if (err) {
		WMI_LOGE("Failed to send ap_ps_egap cmd");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_fw_profiling_cmd_tlv() - send FW profiling cmd to WLAN FW
 * @wmi_handl: wmi handle
 * @cmd: Profiling command index
 * @value1: parameter1 value
 * @value2: parameter2 value
 *
 * Return: 0 for success else error code
 */
QDF_STATUS send_fw_profiling_cmd_tlv(wmi_unified_t wmi_handle,
			uint32_t cmd, uint32_t value1, uint32_t value2)
{
	wmi_buf_t buf;
	int32_t len = 0;
	int ret;
	wmi_wlan_profile_trigger_cmd_fixed_param *prof_trig_cmd;
	wmi_wlan_profile_set_hist_intvl_cmd_fixed_param *hist_intvl_cmd;
	wmi_wlan_profile_enable_profile_id_cmd_fixed_param *profile_enable_cmd;
	wmi_wlan_profile_get_prof_data_cmd_fixed_param *profile_getdata_cmd;

	switch (cmd) {
	case WMI_WLAN_PROFILE_TRIGGER_CMDID:
		len = sizeof(wmi_wlan_profile_trigger_cmd_fixed_param);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGP("%s: wmi_buf_alloc Failed", __func__);
			return -ENOMEM;
		}
		prof_trig_cmd =
			(wmi_wlan_profile_trigger_cmd_fixed_param *)
				wmi_buf_data(buf);
		WMITLV_SET_HDR(&prof_trig_cmd->tlv_header,
		     WMITLV_TAG_STRUC_wmi_wlan_profile_trigger_cmd_fixed_param,
		     WMITLV_GET_STRUCT_TLVLEN
		    (wmi_wlan_profile_trigger_cmd_fixed_param));
		prof_trig_cmd->enable = value1;
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_WLAN_PROFILE_TRIGGER_CMDID);
		if (ret) {
			WMI_LOGE("PROFILE_TRIGGER cmd Failed with value %d",
					value1);
			qdf_nbuf_free(buf);
			return ret;
		}
		break;

	case WMI_WLAN_PROFILE_GET_PROFILE_DATA_CMDID:
		len = sizeof(wmi_wlan_profile_get_prof_data_cmd_fixed_param);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGP("%s: wmi_buf_alloc Failed", __func__);
			return -ENOMEM;
		}
		profile_getdata_cmd =
			(wmi_wlan_profile_get_prof_data_cmd_fixed_param *)
				wmi_buf_data(buf);
		WMITLV_SET_HDR(&profile_getdata_cmd->tlv_header,
		      WMITLV_TAG_STRUC_wmi_wlan_profile_get_prof_data_cmd_fixed_param,
		      WMITLV_GET_STRUCT_TLVLEN
		      (wmi_wlan_profile_get_prof_data_cmd_fixed_param));
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_WLAN_PROFILE_GET_PROFILE_DATA_CMDID);
		if (ret) {
			WMI_LOGE("PROFILE_DATA cmd Failed for id %d value %d",
					value1, value2);
			qdf_nbuf_free(buf);
			return ret;
		}
		break;

	case WMI_WLAN_PROFILE_SET_HIST_INTVL_CMDID:
		len = sizeof(wmi_wlan_profile_set_hist_intvl_cmd_fixed_param);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGP("%s: wmi_buf_alloc Failed", __func__);
			return -ENOMEM;
		}
		hist_intvl_cmd =
			(wmi_wlan_profile_set_hist_intvl_cmd_fixed_param *)
				wmi_buf_data(buf);
		WMITLV_SET_HDR(&hist_intvl_cmd->tlv_header,
		      WMITLV_TAG_STRUC_wmi_wlan_profile_set_hist_intvl_cmd_fixed_param,
		      WMITLV_GET_STRUCT_TLVLEN
		      (wmi_wlan_profile_set_hist_intvl_cmd_fixed_param));
		hist_intvl_cmd->profile_id = value1;
		hist_intvl_cmd->value = value2;
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_WLAN_PROFILE_SET_HIST_INTVL_CMDID);
		if (ret) {
			WMI_LOGE("HIST_INTVL cmd Failed for id %d value %d",
					value1, value2);
			qdf_nbuf_free(buf);
			return ret;
		}
		break;

	case WMI_WLAN_PROFILE_ENABLE_PROFILE_ID_CMDID:
		len =
		sizeof(wmi_wlan_profile_enable_profile_id_cmd_fixed_param);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGP("%s: wmi_buf_alloc Failed", __func__);
			return -ENOMEM;
		}
		profile_enable_cmd =
			(wmi_wlan_profile_enable_profile_id_cmd_fixed_param *)
				wmi_buf_data(buf);
		WMITLV_SET_HDR(&profile_enable_cmd->tlv_header,
		      WMITLV_TAG_STRUC_wmi_wlan_profile_enable_profile_id_cmd_fixed_param,
		      WMITLV_GET_STRUCT_TLVLEN
		      (wmi_wlan_profile_enable_profile_id_cmd_fixed_param));
		profile_enable_cmd->profile_id = value1;
		profile_enable_cmd->enable = value2;
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_WLAN_PROFILE_ENABLE_PROFILE_ID_CMDID);
		if (ret) {
			WMI_LOGE("enable cmd Failed for id %d value %d",
					value1, value2);
			qdf_nbuf_free(buf);
			return ret;
		}
		break;

	default:
		WMI_LOGD("%s: invalid profiling command", __func__);
		break;
	}

	return 0;
}

#ifdef FEATURE_WLAN_RA_FILTERING
/**
 * send_wow_sta_ra_filter_cmd_tlv() - set RA filter pattern in fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: CDF status
 */
QDF_STATUS send_wow_sta_ra_filter_cmd_tlv(wmi_unified_t wmi_handle,
		   uint8_t vdev_id, uint8_t default_pattern,
		   uint16_t rate_limit_interval)
{

	WMI_WOW_ADD_PATTERN_CMD_fixed_param *cmd;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int32_t len;
	int ret;

	len = sizeof(WMI_WOW_ADD_PATTERN_CMD_fixed_param) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_BITMAP_PATTERN_T) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_IPV4_SYNC_PATTERN_T) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_IPV6_SYNC_PATTERN_T) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_MAGIC_PATTERN_CMD) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(A_UINT32) + WMI_TLV_HDR_SIZE + 1 * sizeof(A_UINT32);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (WMI_WOW_ADD_PATTERN_CMD_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_WOW_ADD_PATTERN_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_WOW_ADD_PATTERN_CMD_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->pattern_id = default_pattern,
	cmd->pattern_type = WOW_IPV6_RA_PATTERN;
	buf_ptr += sizeof(WMI_WOW_ADD_PATTERN_CMD_fixed_param);

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_BITMAP_PATTERN_T but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_IPV4_SYNC_PATTERN_T but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_IPV6_SYNC_PATTERN_T but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_MAGIC_PATTERN_CMD but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for pattern_info_timeout but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for ra_ratelimit_interval. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32, sizeof(A_UINT32));
	buf_ptr += WMI_TLV_HDR_SIZE;

	*((A_UINT32 *) buf_ptr) = rate_limit_interval;

	WMI_LOGD("%s: send RA rate limit [%d] to fw vdev = %d", __func__,
		 rate_limit_interval, vdev_id);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_WOW_ADD_WAKE_PATTERN_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send RA rate limit to fw", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;

}
#endif /* FEATURE_WLAN_RA_FILTERING */

/**
 * send_nat_keepalive_en_cmd_tlv() - enable NAT keepalive filter
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_nat_keepalive_en_cmd_tlv(wmi_unified_t wmi_handle, uint8_t vdev_id)
{
	WMI_VDEV_IPSEC_NATKEEPALIVE_FILTER_CMD_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	WMI_LOGD("%s: vdev_id %d", __func__, vdev_id);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (WMI_VDEV_IPSEC_NATKEEPALIVE_FILTER_CMD_fixed_param *)
		wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
	WMITLV_TAG_STRUC_WMI_VDEV_IPSEC_NATKEEPALIVE_FILTER_CMD_fixed_param,
		  WMITLV_GET_STRUCT_TLVLEN
		  (WMI_VDEV_IPSEC_NATKEEPALIVE_FILTER_CMD_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->action = IPSEC_NATKEEPALIVE_FILTER_ENABLE;
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_VDEV_IPSEC_NATKEEPALIVE_FILTER_CMDID)) {
		WMI_LOGP("%s: Failed to send NAT keepalive enable command",
			 __func__);
		wmi_buf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * wmi_unified_csa_offload_enable() - sen CSA offload enable command
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_csa_offload_enable_cmd_tlv(wmi_unified_t wmi_handle,
			uint8_t vdev_id)
{
	wmi_csa_offload_enable_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	WMI_LOGD("%s: vdev_id %d", __func__, vdev_id);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_csa_offload_enable_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_csa_offload_enable_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_csa_offload_enable_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->csa_offload_enable = WMI_CSA_OFFLOAD_ENABLE;
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_CSA_OFFLOAD_ENABLE_CMDID)) {
		WMI_LOGP("%s: Failed to send CSA offload enable command",
			 __func__);
		wmi_buf_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_start_oem_data_cmd_tlv() - start OEM data request to target
 * @wmi_handle: wmi handle
 * @startOemDataReq: start request params
 *
 * Return: CDF status
 */
QDF_STATUS send_start_oem_data_cmd_tlv(wmi_unified_t wmi_handle,
			  uint8_t data_len,
			  uint8_t *data)
{
	wmi_buf_t buf;
	uint8_t *cmd;
	int ret = 0;

	buf = wmi_buf_alloc(wmi_handle,
			    (data_len + WMI_TLV_HDR_SIZE));
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (uint8_t *) wmi_buf_data(buf);

	WMITLV_SET_HDR(cmd, WMITLV_TAG_ARRAY_BYTE, data_len);
	cmd += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(cmd, data,
		     data_len);

	WMI_LOGI(FL("Sending OEM Data Request to target, data len %d"),
		 data_len);

	ret = wmi_unified_cmd_send(wmi_handle, buf,
				   (data_len +
				    WMI_TLV_HDR_SIZE), WMI_OEM_REQ_CMDID);

	if (ret != EOK) {
		WMI_LOGE(FL(":wmi cmd send failed"));
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_dfs_phyerr_filter_offload_en_cmd_tlv() - enable dfs phyerr filter
 * @wmi_handle: wmi handle
 * @dfs_phyerr_filter_offload: is dfs phyerr filter offload
 *
 * Send WMI_DFS_PHYERR_FILTER_ENA_CMDID or
 * WMI_DFS_PHYERR_FILTER_DIS_CMDID command
 * to firmware based on phyerr filtering
 * offload status.
 *
 * Return: 1 success, 0 failure
 */
QDF_STATUS
send_dfs_phyerr_filter_offload_en_cmd_tlv(wmi_unified_t wmi_handle,
			bool dfs_phyerr_filter_offload)
{
	wmi_dfs_phyerr_filter_ena_cmd_fixed_param *enable_phyerr_offload_cmd;
	wmi_dfs_phyerr_filter_dis_cmd_fixed_param *disable_phyerr_offload_cmd;
	wmi_buf_t buf;
	uint16_t len;
	int ret;


	if (dfs_phyerr_filter_offload) {
		WMI_LOGD("%s:Phyerror Filtering offload is Disabled in ini",
			 __func__);
		len = sizeof(*disable_phyerr_offload_cmd);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
			return 0;
		}
		disable_phyerr_offload_cmd =
			(wmi_dfs_phyerr_filter_dis_cmd_fixed_param *)
			wmi_buf_data(buf);

		WMITLV_SET_HDR(&disable_phyerr_offload_cmd->tlv_header,
		     WMITLV_TAG_STRUC_wmi_dfs_phyerr_filter_dis_cmd_fixed_param,
		     WMITLV_GET_STRUCT_TLVLEN
		     (wmi_dfs_phyerr_filter_dis_cmd_fixed_param));

		/*
		 * Send WMI_DFS_PHYERR_FILTER_DIS_CMDID
		 * to the firmware to disable the phyerror
		 * filtering offload.
		 */
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
					   WMI_DFS_PHYERR_FILTER_DIS_CMDID);
		if (ret < 0) {
			WMI_LOGE("%s: Failed to send WMI_DFS_PHYERR_FILTER_DIS_CMDID ret=%d",
				__func__, ret);
			wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
		}
		WMI_LOGD("%s: WMI_DFS_PHYERR_FILTER_DIS_CMDID Send Success",
			 __func__);
	} else {
		WMI_LOGD("%s:Phyerror Filtering offload is Enabled in ini",
			 __func__);

		len = sizeof(*enable_phyerr_offload_cmd);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
		}

		enable_phyerr_offload_cmd =
			(wmi_dfs_phyerr_filter_ena_cmd_fixed_param *)
			wmi_buf_data(buf);

		WMITLV_SET_HDR(&enable_phyerr_offload_cmd->tlv_header,
		     WMITLV_TAG_STRUC_wmi_dfs_phyerr_filter_ena_cmd_fixed_param,
		     WMITLV_GET_STRUCT_TLVLEN
		     (wmi_dfs_phyerr_filter_ena_cmd_fixed_param));

		/*
		 * Send a WMI_DFS_PHYERR_FILTER_ENA_CMDID
		 * to the firmware to enable the phyerror
		 * filtering offload.
		 */
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
					   WMI_DFS_PHYERR_FILTER_ENA_CMDID);

		if (ret < 0) {
			WMI_LOGE("%s: Failed to send DFS PHYERR CMD ret=%d",
				__func__, ret);
			wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
		}
		WMI_LOGD("%s: WMI_DFS_PHYERR_FILTER_ENA_CMDID Send Success",
			 __func__);
	}

	return QDF_STATUS_SUCCESS;
}

#if !defined(REMOVE_PKT_LOG)
/**
 * send_pktlog_wmi_send_cmd_tlv() - send pktlog enable/disable command to target
 * @wmi_handle: wmi handle
 * @pktlog_event: pktlog event
 * @cmd_id: pktlog cmd id
 *
 * Return: CDF status
 */
QDF_STATUS send_pktlog_wmi_send_cmd_tlv(wmi_unified_t wmi_handle,
				   WMI_PKTLOG_EVENT pktlog_event,
				   WMI_CMD_ID cmd_id)
{
	WMI_PKTLOG_EVENT PKTLOG_EVENT;
	WMI_CMD_ID CMD_ID;
	wmi_pdev_pktlog_enable_cmd_fixed_param *cmd;
	wmi_pdev_pktlog_disable_cmd_fixed_param *disable_cmd;
	int len = 0;
	wmi_buf_t buf;

	PKTLOG_EVENT = pktlog_event;
	CMD_ID = cmd_id;

	switch (CMD_ID) {
	case WMI_PDEV_PKTLOG_ENABLE_CMDID:
		len = sizeof(*cmd);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
			return QDF_STATUS_E_NOMEM;
		}
		cmd = (wmi_pdev_pktlog_enable_cmd_fixed_param *)
			wmi_buf_data(buf);
		WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_pdev_pktlog_enable_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
		       (wmi_pdev_pktlog_enable_cmd_fixed_param));
		cmd->evlist = PKTLOG_EVENT;
		if (wmi_unified_cmd_send(wmi_handle, buf, len,
					 WMI_PDEV_PKTLOG_ENABLE_CMDID)) {
			WMI_LOGE("failed to send pktlog enable cmdid");
			goto wmi_send_failed;
		}
		break;
	case WMI_PDEV_PKTLOG_DISABLE_CMDID:
		len = sizeof(*disable_cmd);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
			return QDF_STATUS_E_NOMEM;
		}
		disable_cmd = (wmi_pdev_pktlog_disable_cmd_fixed_param *)
			      wmi_buf_data(buf);
		WMITLV_SET_HDR(&disable_cmd->tlv_header,
		     WMITLV_TAG_STRUC_wmi_pdev_pktlog_disable_cmd_fixed_param,
		     WMITLV_GET_STRUCT_TLVLEN
		     (wmi_pdev_pktlog_disable_cmd_fixed_param));
		disable_cmd->reserved0 = 0;
		if (wmi_unified_cmd_send(wmi_handle, buf, len,
					 WMI_PDEV_PKTLOG_DISABLE_CMDID)) {
			WMI_LOGE("failed to send pktlog disable cmdid");
			goto wmi_send_failed;
		}
		break;
	default:
		WMI_LOGD("%s: invalid PKTLOG command", __func__);
		break;
	}

	return QDF_STATUS_SUCCESS;

wmi_send_failed:
	wmi_buf_free(buf);
	return QDF_STATUS_E_FAILURE;
}
#endif /* REMOVE_PKT_LOG */

/**
 * send_add_wow_wakeup_event_cmd_tlv() -  Configures wow wakeup events.
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @bitmap: Event bitmap
 * @enable: enable/disable
 *
 * Return: CDF status
 */
QDF_STATUS send_add_wow_wakeup_event_cmd_tlv(wmi_unified_t wmi_handle,
					uint32_t vdev_id,
					uint32_t bitmap,
					bool enable)
{
	WMI_WOW_ADD_DEL_EVT_CMD_fixed_param *cmd;
	uint16_t len;
	wmi_buf_t buf;
	int ret;

	len = sizeof(WMI_WOW_ADD_DEL_EVT_CMD_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (WMI_WOW_ADD_DEL_EVT_CMD_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_WOW_ADD_DEL_EVT_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_WOW_ADD_DEL_EVT_CMD_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->is_add = enable;
	cmd->event_bitmap = bitmap;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_WOW_ENABLE_DISABLE_WAKE_EVENT_CMDID);
	if (ret) {
		WMI_LOGE("Failed to config wow wakeup event");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGD("Wakeup pattern 0x%x %s in fw", bitmap,
		 enable ? "enabled" : "disabled");

	return QDF_STATUS_SUCCESS;
}

/**
 * send_wow_patterns_to_fw_cmd_tlv() - Sends WOW patterns to FW.
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @ptrn_id: pattern id
 * @ptrn: pattern
 * @ptrn_len: pattern length
 * @ptrn_offset: pattern offset
 * @mask: mask
 * @mask_len: mask length
 * @user: true for user configured pattern and false for default pattern
 * @default_patterns: default patterns
 *
 * Return: CDF status
 */
QDF_STATUS send_wow_patterns_to_fw_cmd_tlv(wmi_unified_t wmi_handle,
				uint8_t vdev_id, uint8_t ptrn_id,
				const uint8_t *ptrn, uint8_t ptrn_len,
				uint8_t ptrn_offset, const uint8_t *mask,
				uint8_t mask_len, bool user,
				uint8_t default_patterns)
{
	WMI_WOW_ADD_PATTERN_CMD_fixed_param *cmd;
	WOW_BITMAP_PATTERN_T *bitmap_pattern;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int32_t len;
	int ret;


	len = sizeof(WMI_WOW_ADD_PATTERN_CMD_fixed_param) +
	      WMI_TLV_HDR_SIZE +
	      1 * sizeof(WOW_BITMAP_PATTERN_T) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_IPV4_SYNC_PATTERN_T) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_IPV6_SYNC_PATTERN_T) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(WOW_MAGIC_PATTERN_CMD) +
	      WMI_TLV_HDR_SIZE +
	      0 * sizeof(A_UINT32) + WMI_TLV_HDR_SIZE + 1 * sizeof(A_UINT32);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (WMI_WOW_ADD_PATTERN_CMD_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_WOW_ADD_PATTERN_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_WOW_ADD_PATTERN_CMD_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->pattern_id = ptrn_id;

	cmd->pattern_type = WOW_BITMAP_PATTERN;
	buf_ptr += sizeof(WMI_WOW_ADD_PATTERN_CMD_fixed_param);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       sizeof(WOW_BITMAP_PATTERN_T));
	buf_ptr += WMI_TLV_HDR_SIZE;
	bitmap_pattern = (WOW_BITMAP_PATTERN_T *) buf_ptr;

	WMITLV_SET_HDR(&bitmap_pattern->tlv_header,
		       WMITLV_TAG_STRUC_WOW_BITMAP_PATTERN_T,
		       WMITLV_GET_STRUCT_TLVLEN(WOW_BITMAP_PATTERN_T));

	qdf_mem_copy(&bitmap_pattern->patternbuf[0], ptrn, ptrn_len);
	qdf_mem_copy(&bitmap_pattern->bitmaskbuf[0], mask, mask_len);

	bitmap_pattern->pattern_offset = ptrn_offset;
	bitmap_pattern->pattern_len = ptrn_len;

	if (bitmap_pattern->pattern_len > WOW_DEFAULT_BITMAP_PATTERN_SIZE)
		bitmap_pattern->pattern_len = WOW_DEFAULT_BITMAP_PATTERN_SIZE;

	if (bitmap_pattern->pattern_len > WOW_DEFAULT_BITMASK_SIZE)
		bitmap_pattern->pattern_len = WOW_DEFAULT_BITMASK_SIZE;

	bitmap_pattern->bitmask_len = bitmap_pattern->pattern_len;
	bitmap_pattern->pattern_id = ptrn_id;

	WMI_LOGI("vdev id : %d, ptrn id: %d, ptrn len: %d, ptrn offset: %d user %d",
		 cmd->vdev_id, cmd->pattern_id, bitmap_pattern->pattern_len,
		 bitmap_pattern->pattern_offset, user);

	WMI_LOGI("Pattern : ");
	QDF_TRACE_HEX_DUMP(QDF_MODULE_ID_WMI, QDF_TRACE_LEVEL_INFO,
		&bitmap_pattern->patternbuf[0], bitmap_pattern->pattern_len);

	WMI_LOGI("Mask : ");
	QDF_TRACE_HEX_DUMP(QDF_MODULE_ID_WMI, QDF_TRACE_LEVEL_INFO,
		&bitmap_pattern->bitmaskbuf[0], bitmap_pattern->pattern_len);

	buf_ptr += sizeof(WOW_BITMAP_PATTERN_T);

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_IPV4_SYNC_PATTERN_T but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_IPV6_SYNC_PATTERN_T but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for WMITLV_TAG_STRUC_WOW_MAGIC_PATTERN_CMD but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for pattern_info_timeout but no data. */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32, 0);
	buf_ptr += WMI_TLV_HDR_SIZE;

	/* Fill TLV for ratelimit_interval with dummy data as this fix elem */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32, 1 * sizeof(A_UINT32));
	buf_ptr += WMI_TLV_HDR_SIZE;
	*(A_UINT32 *) buf_ptr = 0;

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_WOW_ADD_WAKE_PATTERN_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to send wow ptrn to fw", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_wow_delete_pattern_cmd_tlv() - delete wow pattern in target
 * @wmi_handle: wmi handle
 * @ptrn_id: pattern id
 * @vdev_id: vdev id
 *
 * Return: CDF status
 */
QDF_STATUS send_wow_delete_pattern_cmd_tlv(wmi_unified_t wmi_handle, uint8_t ptrn_id,
					uint8_t vdev_id)
{
	WMI_WOW_DEL_PATTERN_CMD_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len;
	int ret;

	len = sizeof(WMI_WOW_DEL_PATTERN_CMD_fixed_param);


	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (WMI_WOW_DEL_PATTERN_CMD_fixed_param *) wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_WOW_DEL_PATTERN_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
				WMI_WOW_DEL_PATTERN_CMD_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->pattern_id = ptrn_id;
	cmd->pattern_type = WOW_BITMAP_PATTERN;

	WMI_LOGI("Deleting pattern id: %d vdev id %d in fw",
		cmd->pattern_id, vdev_id);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_WOW_DEL_WAKE_PATTERN_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to delete wow ptrn from fw", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_host_wakeup_ind_to_fw_cmd_tlv() - send wakeup ind to fw
 * @wmi_handle: wmi handle
 *
 * Sends host wakeup indication to FW. On receiving this indication,
 * FW will come out of WOW.
 *
 * Return: CDF status
 */
QDF_STATUS send_host_wakeup_ind_to_fw_cmd_tlv(wmi_unified_t wmi_handle)
{
	wmi_wow_hostwakeup_from_sleep_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	int32_t len;
	int ret;

	len = sizeof(wmi_wow_hostwakeup_from_sleep_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_wow_hostwakeup_from_sleep_cmd_fixed_param *)
	      wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_wow_hostwakeup_from_sleep_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN
	       (wmi_wow_hostwakeup_from_sleep_cmd_fixed_param));


	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_WOW_HOSTWAKEUP_FROM_SLEEP_CMDID);
	if (ret) {
		WMI_LOGE("Failed to send host wakeup indication to fw");
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return qdf_status;
}

/**
 * send_del_ts_cmd_tlv() - send DELTS request to fw
 * @wmi_handle: wmi handle
 * @msg: delts params
 *
 * Return: CDF status
 */
QDF_STATUS send_del_ts_cmd_tlv(wmi_unified_t wmi_handle, uint8_t vdev_id,
				uint8_t ac)
{
	wmi_vdev_wmm_delts_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_vdev_wmm_delts_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_wmm_delts_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_wmm_delts_cmd_fixed_param));
	cmd->vdev_id = vdev_id;
	cmd->ac = ac;

	WMI_LOGD("Delts vdev:%d, ac:%d, %s:%d",
		 cmd->vdev_id, cmd->ac, __func__, __LINE__);
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_VDEV_WMM_DELTS_CMDID)) {
		WMI_LOGP("%s: Failed to send vdev DELTS command", __func__);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_aggr_qos_cmd_tlv() - send aggr qos request to fw
 * @wmi_handle: handle to wmi
 * @aggr_qos_rsp_msg - combined struct for all ADD_TS requests.
 *
 * A function to handle WMI_AGGR_QOS_REQ. This will send out
 * ADD_TS requestes to firmware in loop for all the ACs with
 * active flow.
 *
 * Return: CDF status
 */
QDF_STATUS send_aggr_qos_cmd_tlv(wmi_unified_t wmi_handle,
		      struct aggr_add_ts_param *aggr_qos_rsp_msg)
{
	int i = 0;
	wmi_vdev_wmm_addts_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	for (i = 0; i < WMI_QOS_NUM_AC_MAX; i++) {
		/* if flow in this AC is active */
		if (((1 << i) & aggr_qos_rsp_msg->tspecIdx)) {
			/*
			 * as per implementation of wma_add_ts_req() we
			 * are not waiting any response from firmware so
			 * apart from sending ADDTS to firmware just send
			 * success to upper layers
			 */
			aggr_qos_rsp_msg->status[i] = QDF_STATUS_SUCCESS;

			buf = wmi_buf_alloc(wmi_handle, len);
			if (!buf) {
				WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
				return QDF_STATUS_E_NOMEM;
			}
			cmd = (wmi_vdev_wmm_addts_cmd_fixed_param *)
				wmi_buf_data(buf);
			WMITLV_SET_HDR(&cmd->tlv_header,
			       WMITLV_TAG_STRUC_wmi_vdev_wmm_addts_cmd_fixed_param,
			       WMITLV_GET_STRUCT_TLVLEN
				       (wmi_vdev_wmm_addts_cmd_fixed_param));
			cmd->vdev_id = aggr_qos_rsp_msg->sessionId;
			cmd->ac =
				TID_TO_WME_AC(aggr_qos_rsp_msg->tspec[i].tsinfo.
					      traffic.userPrio);
			cmd->medium_time_us =
				aggr_qos_rsp_msg->tspec[i].mediumTime * 32;
			cmd->downgrade_type = WMM_AC_DOWNGRADE_DEPRIO;
			WMI_LOGD("%s:%d: Addts vdev:%d, ac:%d, mediumTime:%d downgrade_type:%d",
				__func__, __LINE__, cmd->vdev_id, cmd->ac,
				cmd->medium_time_us, cmd->downgrade_type);
			if (wmi_unified_cmd_send
				    (wmi_handle, buf, len,
				    WMI_VDEV_WMM_ADDTS_CMDID)) {
				WMI_LOGP("%s: Failed to send vdev ADDTS command",
					__func__);
				aggr_qos_rsp_msg->status[i] =
					QDF_STATUS_E_FAILURE;
				qdf_nbuf_free(buf);
				return QDF_STATUS_E_FAILURE;
			}
		}
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_add_ts_cmd_tlv() - send ADDTS request to fw
 * @wmi_handle: wmi handle
 * @msg: ADDTS params
 *
 * Return: CDF status
 */
QDF_STATUS send_add_ts_cmd_tlv(wmi_unified_t wmi_handle,
		 struct add_ts_param *msg)
{
	wmi_vdev_wmm_addts_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len = sizeof(*cmd);

	msg->status = QDF_STATUS_SUCCESS;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_vdev_wmm_addts_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_wmm_addts_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_vdev_wmm_addts_cmd_fixed_param));
	cmd->vdev_id = msg->sme_session_id;
	cmd->ac = msg->tspec.tsinfo.traffic.userPrio;
	cmd->medium_time_us = msg->tspec.mediumTime * 32;
	cmd->downgrade_type = WMM_AC_DOWNGRADE_DROP;
	WMI_LOGD("Addts vdev:%d, ac:%d, mediumTime:%d, downgrade_type:%d %s:%d",
		 cmd->vdev_id, cmd->ac, cmd->medium_time_us,
		 cmd->downgrade_type, __func__, __LINE__);
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_VDEV_WMM_ADDTS_CMDID)) {
		WMI_LOGP("%s: Failed to send vdev ADDTS command", __func__);
		msg->status = QDF_STATUS_E_FAILURE;
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_enable_disable_packet_filter_cmd_tlv() - enable/disable packet filter in target
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @enable: Flag to enable/disable packet filter
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_enable_disable_packet_filter_cmd_tlv(wmi_unified_t wmi_handle,
					uint8_t vdev_id, bool enable)
{
	int32_t len;
	int ret = 0;
	wmi_buf_t buf;
	WMI_PACKET_FILTER_ENABLE_CMD_fixed_param *cmd;

	len = sizeof(WMI_PACKET_FILTER_ENABLE_CMD_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return -ENOMEM;
	}

	cmd = (WMI_PACKET_FILTER_ENABLE_CMD_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_packet_filter_enable_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
		WMI_PACKET_FILTER_ENABLE_CMD_fixed_param));

	cmd->vdev_id = vdev_id;
	if (enable)
		cmd->enable = PACKET_FILTER_SET_ENABLE;
	else
		cmd->enable = PACKET_FILTER_SET_DISABLE;

	WMI_LOGE("%s: Packet filter enable %d for vdev_id %d",
		__func__, cmd->enable, vdev_id);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
			 WMI_PACKET_FILTER_ENABLE_CMDID);
	if (ret)
		WMI_LOGE("Failed to send packet filter wmi cmd to fw");

	return ret;
}

/**
 * send_config_packet_filter_cmd_tlv() - configure packet filter in target
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @rcv_filter_param: Packet filter parameters
 * @filter_id: Filter id
 * @enable: Flag to add/delete packet filter configuration
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_config_packet_filter_cmd_tlv(wmi_unified_t wmi_handle,
		uint8_t vdev_id, struct rcv_pkt_filter_config *rcv_filter_param,
		uint8_t filter_id, bool enable)
{
	int len, i;
	int err = 0;
	wmi_buf_t buf;
	WMI_PACKET_FILTER_CONFIG_CMD_fixed_param *cmd;


	/* allocate the memory */
	len = sizeof(*cmd);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send set_param cmd");
		return -ENOMEM;
	}

	cmd = (WMI_PACKET_FILTER_CONFIG_CMD_fixed_param *)wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_packet_filter_config_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN
			       (WMI_PACKET_FILTER_CONFIG_CMD_fixed_param));

	cmd->vdev_id = vdev_id;
	cmd->filter_id = filter_id;
	if (enable)
		cmd->filter_action = PACKET_FILTER_SET_ACTIVE;
	else
		cmd->filter_action = PACKET_FILTER_SET_INACTIVE;

	if (enable) {
		cmd->num_params = QDF_MIN(
			WMI_PACKET_FILTER_MAX_CMP_PER_PACKET_FILTER,
			rcv_filter_param->numFieldParams);
		cmd->filter_type = rcv_filter_param->filterType;
		cmd->coalesce_time = rcv_filter_param->coalesceTime;

		for (i = 0; i < cmd->num_params; i++) {
			cmd->paramsData[i].proto_type =
				rcv_filter_param->paramsData[i].protocolLayer;
			cmd->paramsData[i].cmp_type =
				rcv_filter_param->paramsData[i].cmpFlag;
			cmd->paramsData[i].data_length =
				rcv_filter_param->paramsData[i].dataLength;
			cmd->paramsData[i].data_offset =
				rcv_filter_param->paramsData[i].dataOffset;
			memcpy(&cmd->paramsData[i].compareData,
				rcv_filter_param->paramsData[i].compareData,
				sizeof(cmd->paramsData[i].compareData));
			memcpy(&cmd->paramsData[i].dataMask,
				rcv_filter_param->paramsData[i].dataMask,
				sizeof(cmd->paramsData[i].dataMask));
		}
	}

	WMI_LOGE("Packet filter action %d filter with id: %d, num_params=%d",
		cmd->filter_action, cmd->filter_id, cmd->num_params);
	/* send the command along with data */
	err = wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_PACKET_FILTER_CONFIG_CMDID);
	if (err) {
		WMI_LOGE("Failed to send pkt_filter cmd");
		wmi_buf_free(buf);
		return -EIO;
	}


	return 0;
}

/**
 * send_add_clear_mcbc_filter_cmd_tlv() - set mcast filter command to fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @multicastAddr: mcast address
 * @clearList: clear list flag
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_add_clear_mcbc_filter_cmd_tlv(wmi_unified_t wmi_handle,
				     uint8_t vdev_id,
				     struct qdf_mac_addr multicast_addr,
				     bool clearList)
{
	WMI_SET_MCASTBCAST_FILTER_CMD_fixed_param *cmd;
	wmi_buf_t buf;
	int err;

	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send set_param cmd");
		return -ENOMEM;
	}

	cmd = (WMI_SET_MCASTBCAST_FILTER_CMD_fixed_param *) wmi_buf_data(buf);
	qdf_mem_zero(cmd, sizeof(*cmd));

	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_WMI_SET_MCASTBCAST_FILTER_CMD_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (WMI_SET_MCASTBCAST_FILTER_CMD_fixed_param));
	cmd->action =
		(clearList ? WMI_MCAST_FILTER_DELETE : WMI_MCAST_FILTER_SET);
	cmd->vdev_id = vdev_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(multicast_addr.bytes, &cmd->mcastbdcastaddr);
	err = wmi_unified_cmd_send(wmi_handle, buf,
				   sizeof(*cmd),
				   WMI_SET_MCASTBCAST_FILTER_CMDID);
	if (err) {
		WMI_LOGE("Failed to send set_param cmd");
		qdf_mem_free(buf);
		return -EIO;
	}
	WMI_LOGD("Action:%d; vdev_id:%d; clearList:%d",
		 cmd->action, vdev_id, clearList);
	WMI_LOGD("MCBC MAC Addr: "MAC_ADDRESS_STR,
		 MAC_ADDR_ARRAY(multicast_addr.bytes));

	return 0;
}

/**
 * send_gtk_offload_cmd_tlv() - send GTK offload command to fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @params: GTK offload parameters
 *
 * Return: CDF status
 */
QDF_STATUS send_gtk_offload_cmd_tlv(wmi_unified_t wmi_handle, uint8_t vdev_id,
					   struct gtk_offload_params *params,
					   bool enable_offload,
					   uint32_t gtk_offload_opcode)
{
	int len;
	wmi_buf_t buf;
	WMI_GTK_OFFLOAD_CMD_fixed_param *cmd;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	WMI_LOGD("%s Enter", __func__);

	len = sizeof(*cmd);

	/* alloc wmi buffer */
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("wmi_buf_alloc failed for WMI_GTK_OFFLOAD_CMD");
		status = QDF_STATUS_E_NOMEM;
		goto out;
	}

	cmd = (WMI_GTK_OFFLOAD_CMD_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_GTK_OFFLOAD_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_GTK_OFFLOAD_CMD_fixed_param));

	cmd->vdev_id = vdev_id;

	/* Request target to enable GTK offload */
	if (enable_offload) {
		cmd->flags = gtk_offload_opcode;

		/* Copy the keys and replay counter */
		qdf_mem_copy(cmd->KCK, params->aKCK, WMI_GTK_OFFLOAD_KCK_BYTES);
		qdf_mem_copy(cmd->KEK, params->aKEK, WMI_GTK_OFFLOAD_KEK_BYTES);
		qdf_mem_copy(cmd->replay_counter, &params->ullKeyReplayCounter,
			     GTK_REPLAY_COUNTER_BYTES);
	} else {
		cmd->flags = gtk_offload_opcode;
	}

	/* send the wmi command */
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_GTK_OFFLOAD_CMDID)) {
		WMI_LOGE("Failed to send WMI_GTK_OFFLOAD_CMDID");
		wmi_buf_free(buf);
		status = QDF_STATUS_E_FAILURE;
	}

	WMI_LOGD("VDEVID: %d, GTK_FLAGS: x%x", vdev_id, cmd->flags);
out:
	WMI_LOGD("%s Exit", __func__);
	return status;
}

/**
 * send_process_gtk_offload_getinfo_cmd_tlv() - send GTK offload cmd to fw
 * @wmi_handle: wmi handle
 * @params: GTK offload params
 *
 * Return: CDF status
 */
QDF_STATUS send_process_gtk_offload_getinfo_cmd_tlv(wmi_unified_t wmi_handle,
				uint8_t vdev_id,
				uint64_t offload_req_opcode)
{
	int len;
	wmi_buf_t buf;
	WMI_GTK_OFFLOAD_CMD_fixed_param *cmd;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	len = sizeof(*cmd);

	/* alloc wmi buffer */
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("wmi_buf_alloc failed for WMI_GTK_OFFLOAD_CMD");
		status = QDF_STATUS_E_NOMEM;
		goto out;
	}

	cmd = (WMI_GTK_OFFLOAD_CMD_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_GTK_OFFLOAD_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_GTK_OFFLOAD_CMD_fixed_param));

	/* Request for GTK offload status */
	cmd->flags = offload_req_opcode;
	cmd->vdev_id = vdev_id;

	/* send the wmi command */
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_GTK_OFFLOAD_CMDID)) {
		WMI_LOGE("Failed to send WMI_GTK_OFFLOAD_CMDID for req info");
		wmi_buf_free(buf);
		status = QDF_STATUS_E_FAILURE;
	}

out:
	return status;
}

/**
 * send_process_add_periodic_tx_ptrn_cmd_tlv - add periodic tx ptrn
 * @wmi_handle: wmi handle
 * @pAddPeriodicTxPtrnParams: tx ptrn params
 *
 * Retrun: CDF status
 */
QDF_STATUS send_process_add_periodic_tx_ptrn_cmd_tlv(wmi_unified_t wmi_handle,
						struct periodic_tx_pattern  *
						pAddPeriodicTxPtrnParams,
						uint8_t vdev_id)
{
	WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint8_t *buf_ptr;
	uint32_t ptrn_len, ptrn_len_aligned;
	int j;

	ptrn_len = pAddPeriodicTxPtrnParams->ucPtrnSize;
	ptrn_len_aligned = roundup(ptrn_len, sizeof(uint32_t));
	len = sizeof(WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param) +
	      WMI_TLV_HDR_SIZE + ptrn_len_aligned;

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);

	cmd = (WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param));

	/* Pass the pattern id to delete for the corresponding vdev id */
	cmd->vdev_id = vdev_id;
	cmd->pattern_id = pAddPeriodicTxPtrnParams->ucPtrnId;
	cmd->timeout = pAddPeriodicTxPtrnParams->usPtrnIntervalMs;
	cmd->length = pAddPeriodicTxPtrnParams->ucPtrnSize;

	/* Pattern info */
	buf_ptr += sizeof(WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, ptrn_len_aligned);
	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, pAddPeriodicTxPtrnParams->ucPattern, ptrn_len);
	for (j = 0; j < pAddPeriodicTxPtrnParams->ucPtrnSize; j++)
		WMI_LOGD("%s: Add Ptrn: %02x", __func__, buf_ptr[j] & 0xff);

	WMI_LOGD("%s: Add ptrn id: %d vdev_id: %d",
		 __func__, cmd->pattern_id, cmd->vdev_id);

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_ADD_PROACTIVE_ARP_RSP_PATTERN_CMDID)) {
		WMI_LOGE("%s: failed to add pattern set state command",
			 __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_process_del_periodic_tx_ptrn_cmd_tlv - del periodic tx ptrn
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @pattern_id: pattern id
 *
 * Retrun: CDF status
 */
QDF_STATUS send_process_del_periodic_tx_ptrn_cmd_tlv(wmi_unified_t wmi_handle,
						uint8_t vdev_id,
						uint8_t pattern_id)
{
	WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len =
		sizeof(WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param);

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param *)
		wmi_buf_data(wmi_buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMD_fixed_param));

	/* Pass the pattern id to delete for the corresponding vdev id */
	cmd->vdev_id = vdev_id;
	cmd->pattern_id = pattern_id;
	WMI_LOGD("%s: Del ptrn id: %d vdev_id: %d",
		 __func__, cmd->pattern_id, cmd->vdev_id);

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_DEL_PROACTIVE_ARP_RSP_PATTERN_CMDID)) {
		WMI_LOGE("%s: failed to send del pattern command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_stats_ext_req_cmd_tlv() - request ext stats from fw
 * @wmi_handle: wmi handle
 * @preq: stats ext params
 *
 * Return: CDF status
 */
QDF_STATUS send_stats_ext_req_cmd_tlv(wmi_unified_t wmi_handle,
			struct stats_ext_params *preq)
{
	int32_t ret;
	wmi_req_stats_ext_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint16_t len;
	uint8_t *buf_ptr;

	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE + preq->request_data_len;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_req_stats_ext_cmd_fixed_param *) buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_req_stats_ext_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_req_stats_ext_cmd_fixed_param));
	cmd->vdev_id = preq->vdev_id;
	cmd->data_len = preq->request_data_len;

	WMI_LOGD("%s: The data len value is %u and vdev id set is %u ",
		 __func__, preq->request_data_len, preq->vdev_id);

	buf_ptr += sizeof(wmi_req_stats_ext_cmd_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, cmd->data_len);

	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, preq->request_data, cmd->data_len);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_REQUEST_STATS_EXT_CMDID);
	if (ret != EOK) {
		WMI_LOGE("%s: Failed to send notify cmd ret = %d", __func__,
			 ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 * send_enable_ext_wow_cmd_tlv() - enable ext wow in fw
 * @wmi_handle: wmi handle
 * @params: ext wow params
 *
 * Return:0 for success or error code
 */
QDF_STATUS send_enable_ext_wow_cmd_tlv(wmi_unified_t wmi_handle,
			struct ext_wow_params *params)
{
	wmi_extwow_enable_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len;
	int ret;

	len = sizeof(wmi_extwow_enable_cmd_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_extwow_enable_cmd_fixed_param *) wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_extwow_enable_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_extwow_enable_cmd_fixed_param));

	cmd->vdev_id = params->vdev_id;
	cmd->type = params->type;
	cmd->wakeup_pin_num = params->wakeup_pin_num;

	WMI_LOGD("%s: vdev_id %d type %d Wakeup_pin_num %x",
		 __func__, cmd->vdev_id, cmd->type, cmd->wakeup_pin_num);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_EXTWOW_ENABLE_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to set EXTWOW Enable", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;

}

/**
 * send_app_type1_params_in_fw_cmd_tlv() - set app type1 params in fw
 * @wmi_handle: wmi handle
 * @app_type1_params: app type1 params
 *
 * Return: CDF status
 */
QDF_STATUS send_app_type1_params_in_fw_cmd_tlv(wmi_unified_t wmi_handle,
				   struct app_type1_params *app_type1_params)
{
	wmi_extwow_set_app_type1_params_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len;
	int ret;

	len = sizeof(wmi_extwow_set_app_type1_params_cmd_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_extwow_set_app_type1_params_cmd_fixed_param *)
	      wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_wmi_extwow_set_app_type1_params_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_extwow_set_app_type1_params_cmd_fixed_param));

	cmd->vdev_id = app_type1_params->vdev_id;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(app_type1_params->wakee_mac_addr.bytes,
				   &cmd->wakee_mac);
	qdf_mem_copy(cmd->ident, app_type1_params->identification_id, 8);
	cmd->ident_len = app_type1_params->id_length;
	qdf_mem_copy(cmd->passwd, app_type1_params->password, 16);
	cmd->passwd_len = app_type1_params->pass_length;

	WMI_LOGD("%s: vdev_id %d wakee_mac_addr %pM "
		 "identification_id %.8s id_length %u "
		 "password %.16s pass_length %u",
		 __func__, cmd->vdev_id, app_type1_params->wakee_mac_addr.bytes,
		 cmd->ident, cmd->ident_len, cmd->passwd, cmd->passwd_len);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_EXTWOW_SET_APP_TYPE1_PARAMS_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to set APP TYPE1 PARAMS", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_app_type2_params_in_fw_cmd_tlv() - set app type2 params in fw
 * @wmi_handle: wmi handle
 * @appType2Params: app type2 params
 *
 * Return: CDF status
 */
QDF_STATUS send_set_app_type2_params_in_fw_cmd_tlv(wmi_unified_t wmi_handle,
			  struct app_type2_params *appType2Params)
{
	wmi_extwow_set_app_type2_params_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int32_t len;
	int ret;

	len = sizeof(wmi_extwow_set_app_type2_params_cmd_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: Failed allocate wmi buffer", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_extwow_set_app_type2_params_cmd_fixed_param *)
	      wmi_buf_data(buf);

	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_wmi_extwow_set_app_type2_params_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_extwow_set_app_type2_params_cmd_fixed_param));

	cmd->vdev_id = appType2Params->vdev_id;

	qdf_mem_copy(cmd->rc4_key, appType2Params->rc4_key, 16);
	cmd->rc4_key_len = appType2Params->rc4_key_len;

	cmd->ip_id = appType2Params->ip_id;
	cmd->ip_device_ip = appType2Params->ip_device_ip;
	cmd->ip_server_ip = appType2Params->ip_server_ip;

	cmd->tcp_src_port = appType2Params->tcp_src_port;
	cmd->tcp_dst_port = appType2Params->tcp_dst_port;
	cmd->tcp_seq = appType2Params->tcp_seq;
	cmd->tcp_ack_seq = appType2Params->tcp_ack_seq;

	cmd->keepalive_init = appType2Params->keepalive_init;
	cmd->keepalive_min = appType2Params->keepalive_min;
	cmd->keepalive_max = appType2Params->keepalive_max;
	cmd->keepalive_inc = appType2Params->keepalive_inc;

	WMI_CHAR_ARRAY_TO_MAC_ADDR(appType2Params->gateway_mac.bytes,
				   &cmd->gateway_mac);
	cmd->tcp_tx_timeout_val = appType2Params->tcp_tx_timeout_val;
	cmd->tcp_rx_timeout_val = appType2Params->tcp_rx_timeout_val;

	WMI_LOGD("%s: vdev_id %d gateway_mac %pM "
		 "rc4_key %.16s rc4_key_len %u "
		 "ip_id %x ip_device_ip %x ip_server_ip %x "
		 "tcp_src_port %u tcp_dst_port %u tcp_seq %u "
		 "tcp_ack_seq %u keepalive_init %u keepalive_min %u "
		 "keepalive_max %u keepalive_inc %u "
		 "tcp_tx_timeout_val %u tcp_rx_timeout_val %u",
		 __func__, cmd->vdev_id, appType2Params->gateway_mac.bytes,
		 cmd->rc4_key, cmd->rc4_key_len,
		 cmd->ip_id, cmd->ip_device_ip, cmd->ip_server_ip,
		 cmd->tcp_src_port, cmd->tcp_dst_port, cmd->tcp_seq,
		 cmd->tcp_ack_seq, cmd->keepalive_init, cmd->keepalive_min,
		 cmd->keepalive_max, cmd->keepalive_inc,
		 cmd->tcp_tx_timeout_val, cmd->tcp_rx_timeout_val);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_EXTWOW_SET_APP_TYPE2_PARAMS_CMDID);
	if (ret) {
		WMI_LOGE("%s: Failed to set APP TYPE2 PARAMS", __func__);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;

}

/**
 * send_set_auto_shutdown_timer_cmd_tlv() - sets auto shutdown timer in firmware
 * @wmi_handle: wmi handle
 * @timer_val: auto shutdown timer value
 *
 * Return: CDF status
 */
QDF_STATUS send_set_auto_shutdown_timer_cmd_tlv(wmi_unified_t wmi_handle,
						  uint32_t timer_val)
{
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_host_auto_shutdown_cfg_cmd_fixed_param *wmi_auto_sh_cmd;
	int len = sizeof(wmi_host_auto_shutdown_cfg_cmd_fixed_param);

	WMI_LOGD("%s: Set WMI_HOST_AUTO_SHUTDOWN_CFG_CMDID:TIMER_VAL=%d",
		 __func__, timer_val);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	wmi_auto_sh_cmd =
		(wmi_host_auto_shutdown_cfg_cmd_fixed_param *) buf_ptr;
	wmi_auto_sh_cmd->timer_value = timer_val;

	WMITLV_SET_HDR(&wmi_auto_sh_cmd->tlv_header,
	       WMITLV_TAG_STRUC_wmi_host_auto_shutdown_cfg_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_host_auto_shutdown_cfg_cmd_fixed_param));

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_HOST_AUTO_SHUTDOWN_CFG_CMDID);
	if (status != EOK) {
		WMI_LOGE("%s: WMI_HOST_AUTO_SHUTDOWN_CFG_CMDID Err %d",
			 __func__, status);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_nan_req_cmd_tlv() - to send nan request to target
 * @wmi_handle: wmi handle
 * @nan_req: request data which will be non-null
 *
 * Return: CDF status
 */
QDF_STATUS send_nan_req_cmd_tlv(wmi_unified_t wmi_handle,
			struct nan_req_params *nan_req)
{
	int ret;
	wmi_nan_cmd_param *cmd;
	wmi_buf_t buf;
	uint16_t len = sizeof(*cmd);
	uint16_t nan_data_len, nan_data_len_aligned;
	uint8_t *buf_ptr;

	/*
	 *    <----- cmd ------------><-- WMI_TLV_HDR_SIZE --><--- data ---->
	 *    +------------+----------+-----------------------+--------------+
	 *    | tlv_header | data_len | WMITLV_TAG_ARRAY_BYTE | nan_req_data |
	 *    +------------+----------+-----------------------+--------------+
	 */
	if (!nan_req) {
		WMI_LOGE("%s:nan req is not valid", __func__);
		return QDF_STATUS_E_FAILURE;
	}
	nan_data_len = nan_req->request_data_len;
	nan_data_len_aligned = roundup(nan_req->request_data_len,
				       sizeof(uint32_t));
	len += WMI_TLV_HDR_SIZE + nan_data_len_aligned;
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s:wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_nan_cmd_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_nan_cmd_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_nan_cmd_param));
	cmd->data_len = nan_req->request_data_len;
	WMI_LOGD("%s: The data len value is %u",
		 __func__, nan_req->request_data_len);
	buf_ptr += sizeof(wmi_nan_cmd_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, nan_data_len_aligned);
	buf_ptr += WMI_TLV_HDR_SIZE;
	qdf_mem_copy(buf_ptr, nan_req->request_data, cmd->data_len);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_NAN_CMDID);
	if (ret != EOK) {
		WMI_LOGE("%s Failed to send set param command ret = %d",
			 __func__, ret);
		wmi_buf_free(buf);
	}

	return ret;
}

/**
 * send_process_dhcpserver_offload_cmd_tlv() - enable DHCP server offload
 * @wmi_handle: wmi handle
 * @pDhcpSrvOffloadInfo: DHCP server offload info
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_process_dhcpserver_offload_cmd_tlv(wmi_unified_t wmi_handle,
		struct dhcp_offload_info_params *pDhcpSrvOffloadInfo)
{
	wmi_set_dhcp_server_offload_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int err;

	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send "
			 "set_dhcp_server_offload cmd");
		return -ENOMEM;
	}

	cmd = (wmi_set_dhcp_server_offload_cmd_fixed_param *) wmi_buf_data(buf);
	qdf_mem_zero(cmd, sizeof(*cmd));

	WMITLV_SET_HDR(&cmd->tlv_header,
	       WMITLV_TAG_STRUC_wmi_set_dhcp_server_offload_cmd_fixed_param,
	       WMITLV_GET_STRUCT_TLVLEN
	       (wmi_set_dhcp_server_offload_cmd_fixed_param));
	cmd->vdev_id = pDhcpSrvOffloadInfo->vdev_id;
	cmd->enable = pDhcpSrvOffloadInfo->dhcpSrvOffloadEnabled;
	cmd->num_client = pDhcpSrvOffloadInfo->dhcpClientNum;
	cmd->srv_ipv4 = pDhcpSrvOffloadInfo->dhcpSrvIP;
	cmd->start_lsb = 0;
	err = wmi_unified_cmd_send(wmi_handle, buf,
				   sizeof(*cmd),
				   WMI_SET_DHCP_SERVER_OFFLOAD_CMDID);
	if (err) {
		WMI_LOGE("Failed to send set_dhcp_server_offload cmd");
		wmi_buf_free(buf);
		return -EIO;
	}
	WMI_LOGD("Set dhcp server offload to vdevId %d",
		 pDhcpSrvOffloadInfo->vdev_id);
	return 0;
}

/**
 * send_set_led_flashing_cmd_tlv() - set led flashing in fw
 * @wmi_handle: wmi handle
 * @flashing: flashing request
 *
 * Return: CDF status
 */
QDF_STATUS send_set_led_flashing_cmd_tlv(wmi_unified_t wmi_handle,
				struct flashing_req_params *flashing)
{
	wmi_set_led_flashing_cmd_fixed_param *cmd;
	int status = 0;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	int32_t len = sizeof(wmi_set_led_flashing_cmd_fixed_param);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP(FL("wmi_buf_alloc failed"));
		return -ENOMEM;
	}
	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_set_led_flashing_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_set_led_flashing_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_set_led_flashing_cmd_fixed_param));
	cmd->pattern_id = flashing->pattern_id;
	cmd->led_x0 = flashing->led_x0;
	cmd->led_x1 = flashing->led_x1;

	status = wmi_unified_cmd_send(wmi_handle, buf, len,
				      WMI_PDEV_SET_LED_FLASHING_CMDID);
	if (status != EOK) {
		WMI_LOGE("%s: wmi_unified_cmd_send WMI_PEER_SET_PARAM_CMD"
			 " returned Error %d", __func__, status);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_process_ch_avoid_update_cmd_tlv() - handles channel avoid update request
 * @wmi_handle: wmi handle
 * @ch_avoid_update_req: channel avoid update params
 *
 * Return: CDF status
 */
QDF_STATUS send_process_ch_avoid_update_cmd_tlv(wmi_unified_t wmi_handle)
{
	int status = 0;
	wmi_buf_t buf = NULL;
	uint8_t *buf_ptr;
	wmi_chan_avoid_update_cmd_param *ch_avoid_update_fp;
	int len = sizeof(wmi_chan_avoid_update_cmd_param);


	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	ch_avoid_update_fp = (wmi_chan_avoid_update_cmd_param *) buf_ptr;
	WMITLV_SET_HDR(&ch_avoid_update_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_chan_avoid_update_cmd_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_chan_avoid_update_cmd_param));

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_CHAN_AVOID_UPDATE_CMDID);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send"
			 " WMITLV_TABLE_WMI_CHAN_AVOID_UPDATE"
			 " returned Error %d", status);
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_regdomain_info_to_fw_cmd_tlv() - send regdomain info to fw
 * @wmi_handle: wmi handle
 * @reg_dmn: reg domain
 * @regdmn2G: 2G reg domain
 * @regdmn5G: 5G reg domain
 * @ctl2G: 2G test limit
 * @ctl5G: 5G test limit
 *
 * Return: none
 */
QDF_STATUS send_regdomain_info_to_fw_cmd_tlv(wmi_unified_t wmi_handle,
				   uint32_t reg_dmn, uint16_t regdmn2G,
				   uint16_t regdmn5G, int8_t ctl2G,
				   int8_t ctl5G)
{
	wmi_buf_t buf;
	wmi_pdev_set_regdomain_cmd_fixed_param *cmd;
	int32_t len = sizeof(*cmd);


	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_pdev_set_regdomain_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_pdev_set_regdomain_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_set_regdomain_cmd_fixed_param));
	cmd->reg_domain = reg_dmn;
	cmd->reg_domain_2G = regdmn2G;
	cmd->reg_domain_5G = regdmn5G;
	cmd->conformance_test_limit_2G = ctl2G;
	cmd->conformance_test_limit_5G = ctl5G;

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_PDEV_SET_REGDOMAIN_CMDID)) {
		WMI_LOGP("%s: Failed to send pdev set regdomain command",
			 __func__);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}


/**
 * send_set_tdls_offchan_mode_cmd_tlv() - set tdls off channel mode
 * @wmi_handle: wmi handle
 * @chan_switch_params: Pointer to tdls channel switch parameter structure
 *
 * This function sets tdls off channel mode
 *
 * Return: 0 on success; Negative errno otherwise
 */
QDF_STATUS send_set_tdls_offchan_mode_cmd_tlv(wmi_unified_t wmi_handle,
	      struct tdls_channel_switch_params *chan_switch_params)
{
	wmi_tdls_set_offchan_mode_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	u_int16_t len = sizeof(wmi_tdls_set_offchan_mode_cmd_fixed_param);

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return QDF_STATUS_E_FAILURE;
	}
	cmd = (wmi_tdls_set_offchan_mode_cmd_fixed_param *)
		wmi_buf_data(wmi_buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_tdls_set_offchan_mode_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_tdls_set_offchan_mode_cmd_fixed_param));

	WMI_CHAR_ARRAY_TO_MAC_ADDR(chan_switch_params->peer_mac_addr,
				&cmd->peer_macaddr);
	cmd->vdev_id = chan_switch_params->vdev_id;
	cmd->offchan_mode = chan_switch_params->tdls_sw_mode;
	cmd->is_peer_responder = chan_switch_params->is_responder;
	cmd->offchan_num = chan_switch_params->tdls_off_ch;
	cmd->offchan_bw_bitmap = chan_switch_params->tdls_off_ch_bw_offset;
	cmd->offchan_oper_class = chan_switch_params->oper_class;

	WMI_LOGD(FL("Peer MAC Addr mac_addr31to0: 0x%x, mac_addr47to32: 0x%x"),
		 cmd->peer_macaddr.mac_addr31to0,
		 cmd->peer_macaddr.mac_addr47to32);

	WMI_LOGD(FL(
		 "vdev_id: %d, off channel mode: %d, off channel Num: %d, "
		 "off channel offset: 0x%x, is_peer_responder: %d, operating class: %d"
		  ),
		 cmd->vdev_id,
		 cmd->offchan_mode,
		 cmd->offchan_num,
		 cmd->offchan_bw_bitmap,
		 cmd->is_peer_responder,
		 cmd->offchan_oper_class);

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
		WMI_TDLS_SET_OFFCHAN_MODE_CMDID)) {
		WMI_LOGP(FL("failed to send tdls off chan command"));
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}


	return QDF_STATUS_SUCCESS;
}

/**
 * send_update_fw_tdls_state_cmd_tlv() - send enable/disable tdls for a vdev
 * @wmi_handle: wmi handle
 * @pwmaTdlsparams: TDLS params
 *
 * Return: 0 for sucess or error code
 */
QDF_STATUS send_update_fw_tdls_state_cmd_tlv(wmi_unified_t wmi_handle,
					 void *tdls_param, uint8_t tdls_state)
{
	wmi_tdls_set_state_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;

	struct wmi_tdls_params *wmi_tdls = (struct wmi_tdls_params *) tdls_param;
	uint16_t len = sizeof(wmi_tdls_set_state_cmd_fixed_param);

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmai_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}
	cmd = (wmi_tdls_set_state_cmd_fixed_param *) wmi_buf_data(wmi_buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		  WMITLV_TAG_STRUC_wmi_tdls_set_state_cmd_fixed_param,
		  WMITLV_GET_STRUCT_TLVLEN
		  (wmi_tdls_set_state_cmd_fixed_param));
	cmd->vdev_id = wmi_tdls->vdev_id;
	cmd->state = tdls_state;
	cmd->notification_interval_ms = wmi_tdls->notification_interval_ms;
	cmd->tx_discovery_threshold = wmi_tdls->tx_discovery_threshold;
	cmd->tx_teardown_threshold = wmi_tdls->tx_teardown_threshold;
	cmd->rssi_teardown_threshold = wmi_tdls->rssi_teardown_threshold;
	cmd->rssi_delta = wmi_tdls->rssi_delta;
	cmd->tdls_options = wmi_tdls->tdls_options;
	cmd->tdls_peer_traffic_ind_window = wmi_tdls->peer_traffic_ind_window;
	cmd->tdls_peer_traffic_response_timeout_ms =
		wmi_tdls->peer_traffic_response_timeout;
	cmd->tdls_puapsd_mask = wmi_tdls->puapsd_mask;
	cmd->tdls_puapsd_inactivity_time_ms = wmi_tdls->puapsd_inactivity_time;
	cmd->tdls_puapsd_rx_frame_threshold =
		wmi_tdls->puapsd_rx_frame_threshold;
	cmd->teardown_notification_ms =
		wmi_tdls->teardown_notification_ms;
	cmd->tdls_peer_kickout_threshold =
		wmi_tdls->tdls_peer_kickout_threshold;

	WMI_LOGD("%s: tdls_state: %d, state: %d, "
		 "notification_interval_ms: %d, "
		 "tx_discovery_threshold: %d, "
		 "tx_teardown_threshold: %d, "
		 "rssi_teardown_threshold: %d, "
		 "rssi_delta: %d, "
		 "tdls_options: 0x%x, "
		 "tdls_peer_traffic_ind_window: %d, "
		 "tdls_peer_traffic_response_timeout: %d, "
		 "tdls_puapsd_mask: 0x%x, "
		 "tdls_puapsd_inactivity_time: %d, "
		 "tdls_puapsd_rx_frame_threshold: %d, "
		 "teardown_notification_ms: %d, "
		 "tdls_peer_kickout_threshold: %d",
		 __func__, tdls_state, cmd->state,
		 cmd->notification_interval_ms,
		 cmd->tx_discovery_threshold,
		 cmd->tx_teardown_threshold,
		 cmd->rssi_teardown_threshold,
		 cmd->rssi_delta,
		 cmd->tdls_options,
		 cmd->tdls_peer_traffic_ind_window,
		 cmd->tdls_peer_traffic_response_timeout_ms,
		 cmd->tdls_puapsd_mask,
		 cmd->tdls_puapsd_inactivity_time_ms,
		 cmd->tdls_puapsd_rx_frame_threshold,
		 cmd->teardown_notification_ms,
		 cmd->tdls_peer_kickout_threshold);

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_TDLS_SET_STATE_CMDID)) {
		WMI_LOGP("%s: failed to send tdls set state command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}
	WMI_LOGD("%s: vdev_id %d", __func__, wmi_tdls->vdev_id);

	return QDF_STATUS_SUCCESS;
}

/**
 * send_update_tdls_peer_state_cmd_tlv() - update TDLS peer state
 * @wmi_handle: wmi handle
 * @peerStateParams: TDLS peer state params
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_update_tdls_peer_state_cmd_tlv(wmi_unified_t wmi_handle,
			       struct tdls_peer_state_params *peerStateParams,
				   uint32_t *ch_mhz)
{
	wmi_tdls_peer_update_cmd_fixed_param *cmd;
	wmi_tdls_peer_capabilities *peer_cap;
	wmi_channel *chan_info;
	wmi_buf_t wmi_buf;
	uint8_t *buf_ptr;
	uint32_t i;
	int32_t len = sizeof(wmi_tdls_peer_update_cmd_fixed_param) +
		      sizeof(wmi_tdls_peer_capabilities);


	len += WMI_TLV_HDR_SIZE +
	       sizeof(wmi_channel) * peerStateParams->peerCap.peerChanLen;

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_tdls_peer_update_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_tdls_peer_update_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_tdls_peer_update_cmd_fixed_param));

	cmd->vdev_id = peerStateParams->vdevId;
	WMI_CHAR_ARRAY_TO_MAC_ADDR(peerStateParams->peerMacAddr,
				   &cmd->peer_macaddr);


	cmd->peer_state = peerStateParams->peerState;

	WMI_LOGD("%s: vdev_id: %d, peerStateParams->peerMacAddr: %pM, "
		 "peer_macaddr.mac_addr31to0: 0x%x, "
		 "peer_macaddr.mac_addr47to32: 0x%x, peer_state: %d",
		 __func__, cmd->vdev_id, peerStateParams->peerMacAddr,
		 cmd->peer_macaddr.mac_addr31to0,
		 cmd->peer_macaddr.mac_addr47to32, cmd->peer_state);

	buf_ptr += sizeof(wmi_tdls_peer_update_cmd_fixed_param);
	peer_cap = (wmi_tdls_peer_capabilities *) buf_ptr;
	WMITLV_SET_HDR(&peer_cap->tlv_header,
		       WMITLV_TAG_STRUC_wmi_tdls_peer_capabilities,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_tdls_peer_capabilities));

	if ((peerStateParams->peerCap.peerUapsdQueue & 0x08) >> 3)
		WMI_SET_TDLS_PEER_VO_UAPSD(peer_cap);
	if ((peerStateParams->peerCap.peerUapsdQueue & 0x04) >> 2)
		WMI_SET_TDLS_PEER_VI_UAPSD(peer_cap);
	if ((peerStateParams->peerCap.peerUapsdQueue & 0x02) >> 1)
		WMI_SET_TDLS_PEER_BK_UAPSD(peer_cap);
	if (peerStateParams->peerCap.peerUapsdQueue & 0x01)
		WMI_SET_TDLS_PEER_BE_UAPSD(peer_cap);

	/* Ack and More Data Ack are sent as 0, so no need to set
	 * but fill SP
	 */
	WMI_SET_TDLS_PEER_SP_UAPSD(peer_cap,
				   peerStateParams->peerCap.peerMaxSp);

	peer_cap->buff_sta_support =
		peerStateParams->peerCap.peerBuffStaSupport;
	peer_cap->off_chan_support =
		peerStateParams->peerCap.peerOffChanSupport;
	peer_cap->peer_curr_operclass =
		peerStateParams->peerCap.peerCurrOperClass;
	/* self curr operclass is not being used and so pass op class for
	 * preferred off chan in it.
	 */
	peer_cap->self_curr_operclass =
		peerStateParams->peerCap.opClassForPrefOffChan;
	peer_cap->peer_chan_len = peerStateParams->peerCap.peerChanLen;
	peer_cap->peer_operclass_len =
		peerStateParams->peerCap.peerOperClassLen;

	WMI_LOGD("%s: peer_operclass_len: %d",
		 __func__, peer_cap->peer_operclass_len);
	for (i = 0; i < WMI_TDLS_MAX_SUPP_OPER_CLASSES; i++) {
		peer_cap->peer_operclass[i] =
			peerStateParams->peerCap.peerOperClass[i];
		WMI_LOGD("%s: peer_operclass[%d]: %d",
			 __func__, i, peer_cap->peer_operclass[i]);
	}

	peer_cap->is_peer_responder = peerStateParams->peerCap.isPeerResponder;
	peer_cap->pref_offchan_num = peerStateParams->peerCap.prefOffChanNum;
	peer_cap->pref_offchan_bw =
		peerStateParams->peerCap.prefOffChanBandwidth;

	WMI_LOGD
		("%s: peer_qos: 0x%x, buff_sta_support: %d, off_chan_support: %d, "
		 "peer_curr_operclass: %d, self_curr_operclass: %d, peer_chan_len: "
		 "%d, peer_operclass_len: %d, is_peer_responder: %d, pref_offchan_num:"
		 " %d, pref_offchan_bw: %d",
		__func__, peer_cap->peer_qos, peer_cap->buff_sta_support,
		peer_cap->off_chan_support, peer_cap->peer_curr_operclass,
		peer_cap->self_curr_operclass, peer_cap->peer_chan_len,
		peer_cap->peer_operclass_len, peer_cap->is_peer_responder,
		peer_cap->pref_offchan_num, peer_cap->pref_offchan_bw);

	/* next fill variable size array of peer chan info */
	buf_ptr += sizeof(wmi_tdls_peer_capabilities);
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_ARRAY_STRUC,
		       sizeof(wmi_channel) *
		       peerStateParams->peerCap.peerChanLen);
	chan_info = (wmi_channel *) (buf_ptr + WMI_TLV_HDR_SIZE);

	for (i = 0; i < peerStateParams->peerCap.peerChanLen; ++i) {
		WMITLV_SET_HDR(&chan_info->tlv_header,
			       WMITLV_TAG_STRUC_wmi_channel,
			       WMITLV_GET_STRUCT_TLVLEN(wmi_channel));
		chan_info->mhz = ch_mhz[i];
		chan_info->band_center_freq1 = chan_info->mhz;
		chan_info->band_center_freq2 = 0;

		WMI_LOGD("%s: chan[%d] = %u", __func__, i, chan_info->mhz);

		if (peerStateParams->peerCap.peerChan[i].dfsSet) {
			WMI_SET_CHANNEL_FLAG(chan_info, WMI_CHAN_FLAG_PASSIVE);
			WMI_LOGI("chan[%d] DFS[%d]\n",
				 peerStateParams->peerCap.peerChan[i].chanId,
				 peerStateParams->peerCap.peerChan[i].dfsSet);
		}

		if (chan_info->mhz < WMI_2_4_GHZ_MAX_FREQ)
			WMI_SET_CHANNEL_MODE(chan_info, MODE_11G);
		else
			WMI_SET_CHANNEL_MODE(chan_info, MODE_11A);

		WMI_SET_CHANNEL_MAX_TX_POWER(chan_info,
					     peerStateParams->peerCap.
					     peerChan[i].pwr);

		WMI_SET_CHANNEL_REG_POWER(chan_info,
					  peerStateParams->peerCap.peerChan[i].
					  pwr);
		WMI_LOGD("Channel TX power[%d] = %u: %d", i, chan_info->mhz,
			 peerStateParams->peerCap.peerChan[i].pwr);

		chan_info++;
	}

	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_TDLS_PEER_UPDATE_CMDID)) {
		WMI_LOGE("%s: failed to send tdls peer update state command",
			 __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}


	return QDF_STATUS_SUCCESS;
}

/*
 * send_process_fw_mem_dump_cmd_tlv() - Function to request fw memory dump from
 * firmware
 * @wmi_handle:         Pointer to wmi handle
 * @mem_dump_req:       Pointer for mem_dump_req
 *
 * This function sends memory dump request to firmware
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 *
 */
QDF_STATUS send_process_fw_mem_dump_cmd_tlv(wmi_unified_t wmi_handle,
					struct fw_dump_req_param *mem_dump_req)
{
	wmi_get_fw_mem_dump_fixed_param *cmd;
	wmi_fw_mem_dump *dump_params;
	struct fw_dump_seg_req *seg_req;
	int32_t len;
	wmi_buf_t buf;
	u_int8_t *buf_ptr;
	int ret, loop;

	/*
	 * len = sizeof(fixed param) that includes tlv header +
	 *       tlv header for array of struc +
	 *       sizeof (each struct)
	 */
	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE;
	len += mem_dump_req->num_seg * sizeof(wmi_fw_mem_dump);
	buf = wmi_buf_alloc(wmi_handle, len);

	if (!buf) {
		WMI_LOGE(FL("Failed allocate wmi buffer"));
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (u_int8_t *) wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);
	cmd = (wmi_get_fw_mem_dump_fixed_param *) buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_get_fw_mem_dump_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(wmi_get_fw_mem_dump_fixed_param));

	cmd->request_id = mem_dump_req->request_id;
	cmd->num_fw_mem_dump_segs = mem_dump_req->num_seg;

	/* TLV indicating array of structures to follow */
	buf_ptr += sizeof(wmi_get_fw_mem_dump_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       sizeof(wmi_fw_mem_dump) *
		       cmd->num_fw_mem_dump_segs);

	buf_ptr += WMI_TLV_HDR_SIZE;
	dump_params = (wmi_fw_mem_dump *) buf_ptr;

	WMI_LOGI(FL("request_id:%d num_seg:%d"),
		    mem_dump_req->request_id, mem_dump_req->num_seg);
	for (loop = 0; loop < cmd->num_fw_mem_dump_segs; loop++) {
		seg_req = (struct fw_dump_seg_req *)
			  ((uint8_t *)(mem_dump_req->segment) +
			    loop * sizeof(*seg_req));
		WMITLV_SET_HDR(&dump_params->tlv_header,
			    WMITLV_TAG_STRUC_wmi_fw_mem_dump_params,
			    WMITLV_GET_STRUCT_TLVLEN(wmi_fw_mem_dump));
		dump_params->seg_id = seg_req->seg_id;
		dump_params->seg_start_addr_lo = seg_req->seg_start_addr_lo;
		dump_params->seg_start_addr_hi = seg_req->seg_start_addr_hi;
		dump_params->seg_length = seg_req->seg_length;
		dump_params->dest_addr_lo = seg_req->dst_addr_lo;
		dump_params->dest_addr_hi = seg_req->dst_addr_hi;
		WMI_LOGI(FL("seg_number:%d"), loop);
		WMI_LOGI(FL("seg_id:%d start_addr_lo:0x%x start_addr_hi:0x%x"),
			 dump_params->seg_id, dump_params->seg_start_addr_lo,
			 dump_params->seg_start_addr_hi);
		WMI_LOGI(FL("seg_length:%d dst_addr_lo:0x%x dst_addr_hi:0x%x"),
			 dump_params->seg_length, dump_params->dest_addr_lo,
			 dump_params->dest_addr_hi);
		dump_params++;
	}

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_GET_FW_MEM_DUMP_CMDID);
	if (ret) {
		WMI_LOGE(FL("Failed to send get firmware mem dump request"));
		wmi_buf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	WMI_LOGI(FL("Get firmware mem dump request sent successfully"));
	return QDF_STATUS_SUCCESS;
}

/*
 * send_process_set_ie_info_cmd_tlv() - Function to send IE info to firmware
 * @wmi_handle:    Pointer to WMi handle
 * @ie_data:       Pointer for ie data
 *
 * This function sends IE information to firmware
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 *
 */
QDF_STATUS send_process_set_ie_info_cmd_tlv(wmi_unified_t wmi_handle,
				   struct vdev_ie_info_param *ie_info)
{
	wmi_vdev_set_ie_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t len, ie_len_aligned;
	int ret;


	ie_len_aligned = roundup(ie_info->length, sizeof(uint32_t));
	/* Allocate memory for the WMI command */
	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE + ie_len_aligned;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE(FL("wmi_buf_alloc failed"));
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = wmi_buf_data(buf);
	qdf_mem_zero(buf_ptr, len);

	/* Populate the WMI command */
	cmd = (wmi_vdev_set_ie_cmd_fixed_param *)buf_ptr;

	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_vdev_set_ie_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(
			wmi_vdev_set_ie_cmd_fixed_param));
	cmd->vdev_id = ie_info->vdev_id;
	cmd->ie_id = ie_info->ie_id;
	cmd->ie_len = ie_info->length;

	WMI_LOGD(FL("IE:%d of size:%d sent for vdev:%d"), ie_info->ie_id,
		 ie_info->length, ie_info->vdev_id);

	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_BYTE, ie_len_aligned);
	buf_ptr += WMI_TLV_HDR_SIZE;

	qdf_mem_copy(buf_ptr, ie_info->data, cmd->ie_len);

	ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				   WMI_VDEV_SET_IE_CMDID);
	if (ret != EOK) {
		WMI_LOGE(FL("Failed to send set IE command ret = %d"), ret);
		wmi_buf_free(buf);
	}

	return ret;
}


void wmi_copy_resource_config(wmi_resource_config *resource_cfg,
				wmi_resource_config *tgt_res_cfg)
{
	resource_cfg->num_peers = tgt_res_cfg->num_peers;
	resource_cfg->num_offload_peers = tgt_res_cfg->num_offload_peers;
	resource_cfg->num_offload_reorder_buffs =
			tgt_res_cfg->num_offload_reorder_buffs;
	resource_cfg->num_peer_keys = tgt_res_cfg->num_peer_keys;
	resource_cfg->num_tids = tgt_res_cfg->num_tids;
	resource_cfg->ast_skid_limit = tgt_res_cfg->ast_skid_limit;
	resource_cfg->tx_chain_mask = tgt_res_cfg->tx_chain_mask;
	resource_cfg->rx_chain_mask = tgt_res_cfg->rx_chain_mask;
	resource_cfg->rx_timeout_pri[0] = tgt_res_cfg->rx_timeout_pri[0];
	resource_cfg->rx_timeout_pri[1] = tgt_res_cfg->rx_timeout_pri[1];
	resource_cfg->rx_timeout_pri[2] = tgt_res_cfg->rx_timeout_pri[2];
	resource_cfg->rx_timeout_pri[3] = tgt_res_cfg->rx_timeout_pri[3];
	resource_cfg->rx_decap_mode = tgt_res_cfg->rx_decap_mode;
	resource_cfg->scan_max_pending_req =
			tgt_res_cfg->scan_max_pending_req;
	resource_cfg->bmiss_offload_max_vdev =
			tgt_res_cfg->bmiss_offload_max_vdev;
	resource_cfg->roam_offload_max_vdev =
			tgt_res_cfg->roam_offload_max_vdev;
	resource_cfg->roam_offload_max_ap_profiles =
			tgt_res_cfg->roam_offload_max_ap_profiles;
	resource_cfg->num_mcast_groups = tgt_res_cfg->num_mcast_groups;
	resource_cfg->num_mcast_table_elems =
			tgt_res_cfg->num_mcast_table_elems;
	resource_cfg->mcast2ucast_mode = tgt_res_cfg->mcast2ucast_mode;
	resource_cfg->tx_dbg_log_size = tgt_res_cfg->tx_dbg_log_size;
	resource_cfg->num_wds_entries = tgt_res_cfg->num_wds_entries;
	resource_cfg->dma_burst_size = tgt_res_cfg->dma_burst_size;
	resource_cfg->mac_aggr_delim = tgt_res_cfg->mac_aggr_delim;
	resource_cfg->rx_skip_defrag_timeout_dup_detection_check =
		tgt_res_cfg->rx_skip_defrag_timeout_dup_detection_check;
	resource_cfg->vow_config = tgt_res_cfg->vow_config;
	resource_cfg->gtk_offload_max_vdev = tgt_res_cfg->gtk_offload_max_vdev;
	resource_cfg->num_msdu_desc = tgt_res_cfg->num_msdu_desc;
	resource_cfg->max_frag_entries = tgt_res_cfg->max_frag_entries;
	resource_cfg->num_tdls_vdevs = tgt_res_cfg->num_tdls_vdevs;
	resource_cfg->num_tdls_conn_table_entries =
			tgt_res_cfg->num_tdls_conn_table_entries;
	resource_cfg->beacon_tx_offload_max_vdev =
			tgt_res_cfg->beacon_tx_offload_max_vdev;
	resource_cfg->num_multicast_filter_entries =
			tgt_res_cfg->num_multicast_filter_entries;
	resource_cfg->num_wow_filters =
			tgt_res_cfg->num_wow_filters;
	resource_cfg->num_keep_alive_pattern =
			tgt_res_cfg->num_keep_alive_pattern;
	resource_cfg->keep_alive_pattern_size =
			tgt_res_cfg->keep_alive_pattern_size;
	resource_cfg->max_tdls_concurrent_sleep_sta =
			tgt_res_cfg->max_tdls_concurrent_sleep_sta;
	resource_cfg->max_tdls_concurrent_buffer_sta =
			tgt_res_cfg->max_tdls_concurrent_buffer_sta;
	resource_cfg->wmi_send_separate =
			tgt_res_cfg->wmi_send_separate;
	resource_cfg->num_ocb_vdevs =
			tgt_res_cfg->num_ocb_vdevs;
	resource_cfg->num_ocb_channels =
			tgt_res_cfg->num_ocb_channels;
	resource_cfg->num_ocb_schedules =
			tgt_res_cfg->num_ocb_schedules;

}

/**
 * send_init_cmd_tlv() - wmi init command
 * @wmi_handle:      pointer to wmi handle
 * @res_cfg:         resource config
 * @num_mem_chunks:  no of mem chunck
 * @mem_chunk:       pointer to mem chunck structure
 *
 * This function sends IE information to firmware
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 *
 */
QDF_STATUS send_init_cmd_tlv(wmi_unified_t wmi_handle,
		wmi_resource_config *tgt_res_cfg,
		uint8_t num_mem_chunks, struct wmi_host_mem_chunk *mem_chunks,
		bool action)
{
	wmi_buf_t buf;
	wmi_init_cmd_fixed_param *cmd;
	wmi_abi_version my_vers;
	int num_whitelist;
	uint8_t *buf_ptr;
	wmi_resource_config *resource_cfg;
	wlan_host_memory_chunk *host_mem_chunks;
	uint32_t mem_chunk_len = 0;
	uint16_t idx;
	int len;
	int ret;

	len = sizeof(*cmd) + sizeof(wmi_resource_config) + WMI_TLV_HDR_SIZE;
	mem_chunk_len = (sizeof(wlan_host_memory_chunk) * MAX_MEM_CHUNKS);
	buf = wmi_buf_alloc(wmi_handle, len + mem_chunk_len);
	if (!buf) {
		WMI_LOGD("%s: wmi_buf_alloc failed\n", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	cmd = (wmi_init_cmd_fixed_param *) buf_ptr;
	resource_cfg = (wmi_resource_config *) (buf_ptr + sizeof(*cmd));

	host_mem_chunks = (wlan_host_memory_chunk *)
		(buf_ptr + sizeof(*cmd) + sizeof(wmi_resource_config)
		 + WMI_TLV_HDR_SIZE);

	WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_init_cmd_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(wmi_init_cmd_fixed_param));

	qdf_mem_copy(resource_cfg, tgt_res_cfg, sizeof(wmi_resource_config));
	WMITLV_SET_HDR(&resource_cfg->tlv_header,
			WMITLV_TAG_STRUC_wmi_resource_config,
			WMITLV_GET_STRUCT_TLVLEN(wmi_resource_config));

	for (idx = 0; idx < num_mem_chunks; ++idx) {
		WMITLV_SET_HDR(&(host_mem_chunks[idx].tlv_header),
				WMITLV_TAG_STRUC_wlan_host_memory_chunk,
				WMITLV_GET_STRUCT_TLVLEN
				(wlan_host_memory_chunk));
		host_mem_chunks[idx].ptr = mem_chunks[idx].paddr;
		host_mem_chunks[idx].size = mem_chunks[idx].len;
		host_mem_chunks[idx].req_id = mem_chunks[idx].req_id;
		WMI_LOGD("chunk %d len %d requested ,ptr  0x%x ",
				idx, host_mem_chunks[idx].size,
				host_mem_chunks[idx].ptr);
	}
	cmd->num_host_mem_chunks = num_mem_chunks;
	len += (num_mem_chunks * sizeof(wlan_host_memory_chunk));
	WMITLV_SET_HDR((buf_ptr + sizeof(*cmd) + sizeof(wmi_resource_config)),
			WMITLV_TAG_ARRAY_STRUC,
			(sizeof(wlan_host_memory_chunk) *
			 num_mem_chunks));

	num_whitelist = sizeof(version_whitelist) /
		sizeof(wmi_whitelist_version_info);
	my_vers.abi_version_0 = WMI_ABI_VERSION_0;
	my_vers.abi_version_1 = WMI_ABI_VERSION_1;
	my_vers.abi_version_ns_0 = WMI_ABI_VERSION_NS_0;
	my_vers.abi_version_ns_1 = WMI_ABI_VERSION_NS_1;
	my_vers.abi_version_ns_2 = WMI_ABI_VERSION_NS_2;
	my_vers.abi_version_ns_3 = WMI_ABI_VERSION_NS_3;

	wmi_cmp_and_set_abi_version(num_whitelist, version_whitelist,
			&my_vers,
			&wmi_handle->fw_abi_version,
			&cmd->host_abi_vers);

	WMI_LOGD("%s: INIT_CMD version: %d, %d, 0x%x, 0x%x, 0x%x, 0x%x",
		__func__, WMI_VER_GET_MAJOR(cmd->host_abi_vers.abi_version_0),
			WMI_VER_GET_MINOR(cmd->host_abi_vers.abi_version_0),
			cmd->host_abi_vers.abi_version_ns_0,
			cmd->host_abi_vers.abi_version_ns_1,
			cmd->host_abi_vers.abi_version_ns_2,
			cmd->host_abi_vers.abi_version_ns_3);

	/* Save version sent from host -
	 * Will be used to check ready event
	 */
	qdf_mem_copy(&wmi_handle->final_abi_vers, &cmd->host_abi_vers,
			sizeof(wmi_abi_version));

	if (action) {
		ret = wmi_unified_cmd_send(wmi_handle, buf, len,
				 WMI_INIT_CMDID);
		if (ret) {
			WMI_LOGE(FL("Failed to send set WMI INIT command ret = %d"), ret);
			wmi_buf_free(buf);
			return QDF_STATUS_E_FAILURE;
		}
	} else {
		wmi_handle->saved_wmi_init_cmd.buf = buf;
		wmi_handle->saved_wmi_init_cmd.buf_len = len;
	}

	return QDF_STATUS_SUCCESS;

}

/**
 * send_saved_init_cmd_tlv() - wmi init command
 * @wmi_handle:      pointer to wmi handle
 *
 * This function sends IE information to firmware
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 *
 */
QDF_STATUS send_saved_init_cmd_tlv(wmi_unified_t wmi_handle)
{
	int status;

	if (!wmi_handle->saved_wmi_init_cmd.buf ||
			!wmi_handle->saved_wmi_init_cmd.buf_len) {
		WMI_LOGP("Service ready ext event w/o WMI_SERVICE_EXT_MSG!");
		return QDF_STATUS_E_FAILURE;
	}
	status = wmi_unified_cmd_send(wmi_handle,
				wmi_handle->saved_wmi_init_cmd.buf,
				wmi_handle->saved_wmi_init_cmd.buf_len,
				WMI_INIT_CMDID);
	if (status) {
		WMI_LOGE(FL("Failed to send set WMI INIT command ret = %d"), status);
		wmi_buf_free(wmi_handle->saved_wmi_init_cmd.buf);
		return QDF_STATUS_E_FAILURE;
	}
	wmi_handle->saved_wmi_init_cmd.buf = NULL;
	wmi_handle->saved_wmi_init_cmd.buf_len = 0;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS save_fw_version_cmd_tlv(wmi_unified_t wmi_handle, void *evt_buf)
{
	WMI_SERVICE_READY_EVENTID_param_tlvs *param_buf;
	wmi_service_ready_event_fixed_param *ev;


	param_buf = (WMI_SERVICE_READY_EVENTID_param_tlvs *) evt_buf;

	ev = (wmi_service_ready_event_fixed_param *) param_buf->fixed_param;
	if (!ev)
		qdf_assert(0);

	/*Save fw version from service ready message */
	/*This will be used while sending INIT message */
	qdf_mem_copy(&wmi_handle->fw_abi_version, &ev->fw_abi_vers,
			sizeof(wmi_handle->fw_abi_version));

	return QDF_STATUS_SUCCESS;
}

/**
 * wmi_unified_save_fw_version_cmd() - save fw version
 * @wmi_handle:      pointer to wmi handle
 * @res_cfg:         resource config
 * @num_mem_chunks:  no of mem chunck
 * @mem_chunk:       pointer to mem chunck structure
 *
 * This function sends IE information to firmware
 *
 * Return: QDF_STATUS_SUCCESS for success otherwise failure
 *
 */
QDF_STATUS check_and_update_fw_version_cmd_tlv(wmi_unified_t wmi_handle,
					  void *evt_buf)
{
	WMI_READY_EVENTID_param_tlvs *param_buf = NULL;
	wmi_ready_event_fixed_param *ev = NULL;

	param_buf = (WMI_READY_EVENTID_param_tlvs *) evt_buf;
	ev = param_buf->fixed_param;

	if (!wmi_versions_are_compatible(&wmi_handle->final_abi_vers,
				&ev->fw_abi_vers)) {
		/*
		 * Error: Our host version and the given firmware version
		 * are incompatible.
		 **/
		WMI_LOGD("%s: Error: Incompatible WMI version."
			"Host: %d,%d,0x%x 0x%x 0x%x 0x%x, FW: %d,%d,0x%x 0x%x 0x%x 0x%x\n",
				__func__,
			WMI_VER_GET_MAJOR(wmi_handle->final_abi_vers.
				abi_version_0),
			WMI_VER_GET_MINOR(wmi_handle->final_abi_vers.
				abi_version_0),
			wmi_handle->final_abi_vers.abi_version_ns_0,
			wmi_handle->final_abi_vers.abi_version_ns_1,
			wmi_handle->final_abi_vers.abi_version_ns_2,
			wmi_handle->final_abi_vers.abi_version_ns_3,
			WMI_VER_GET_MAJOR(ev->fw_abi_vers.abi_version_0),
			WMI_VER_GET_MINOR(ev->fw_abi_vers.abi_version_0),
			ev->fw_abi_vers.abi_version_ns_0,
			ev->fw_abi_vers.abi_version_ns_1,
			ev->fw_abi_vers.abi_version_ns_2,
			ev->fw_abi_vers.abi_version_ns_3);

		return QDF_STATUS_E_FAILURE;
	}
	qdf_mem_copy(&wmi_handle->final_abi_vers, &ev->fw_abi_vers,
			sizeof(wmi_abi_version));
	qdf_mem_copy(&wmi_handle->fw_abi_version, &ev->fw_abi_vers,
			sizeof(wmi_abi_version));


	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_base_macaddr_indicate_cmd_tlv() - set base mac address in fw
 * @wmi_handle: wmi handle
 * @custom_addr: base mac address
 *
 * Return: 0 for success or error code
 */
QDF_STATUS send_set_base_macaddr_indicate_cmd_tlv(wmi_unified_t wmi_handle,
					 uint8_t *custom_addr)
{
	wmi_pdev_set_base_macaddr_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	int err;

	buf = wmi_buf_alloc(wmi_handle, sizeof(*cmd));
	if (!buf) {
		WMI_LOGE("Failed to allocate buffer to send base macaddr cmd");
		return -ENOMEM;
	}

	cmd = (wmi_pdev_set_base_macaddr_cmd_fixed_param *) wmi_buf_data(buf);
	qdf_mem_zero(cmd, sizeof(*cmd));

	WMITLV_SET_HDR(&cmd->tlv_header,
		   WMITLV_TAG_STRUC_wmi_pdev_set_base_macaddr_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_pdev_set_base_macaddr_cmd_fixed_param));
	WMI_CHAR_ARRAY_TO_MAC_ADDR(custom_addr, &cmd->base_macaddr);
	err = wmi_unified_cmd_send(wmi_handle, buf,
				   sizeof(*cmd),
				   WMI_PDEV_SET_BASE_MACADDR_CMDID);
	if (err) {
		WMI_LOGE("Failed to send set_base_macaddr cmd");
		qdf_mem_free(buf);
		return -EIO;
	}

	return 0;
}

/**
 * send_log_supported_evt_cmd_tlv() - Enable/Disable FW diag/log events
 * @handle: wmi handle
 * @event:  Event received from FW
 * @len:    Length of the event
 *
 * Enables the low frequency events and disables the high frequency
 * events. Bit 17 indicates if the event if low/high frequency.
 * 1 - high frequency, 0 - low frequency
 *
 * Return: 0 on successfully enabling/disabling the events
 */
QDF_STATUS send_log_supported_evt_cmd_tlv(wmi_unified_t wmi_handle,
		uint8_t *event,
		uint32_t len)
{
	uint32_t num_of_diag_events_logs;
	wmi_diag_event_log_config_fixed_param *cmd;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t *cmd_args, *evt_args;
	uint32_t buf_len, i;

	WMI_DIAG_EVENT_LOG_SUPPORTED_EVENTID_param_tlvs *param_buf;
	wmi_diag_event_log_supported_event_fixed_params *wmi_event;

	WMI_LOGI("Received WMI_DIAG_EVENT_LOG_SUPPORTED_EVENTID");

	param_buf = (WMI_DIAG_EVENT_LOG_SUPPORTED_EVENTID_param_tlvs *) event;
	if (!param_buf) {
		WMI_LOGE("Invalid log supported event buffer");
		return -EINVAL;
	}
	wmi_event = param_buf->fixed_param;
	num_of_diag_events_logs = wmi_event->num_of_diag_events_logs;
	evt_args = param_buf->diag_events_logs_list;
	if (!evt_args) {
		WMI_LOGE("%s: Event list is empty, num_of_diag_events_logs=%d",
				__func__, num_of_diag_events_logs);
		return -EINVAL;
	}

	WMI_LOGD("%s: num_of_diag_events_logs=%d",
			__func__, num_of_diag_events_logs);

	/* Free any previous allocation */
	if (wmi_handle->events_logs_list)
		qdf_mem_free(wmi_handle->events_logs_list);

	/* Store the event list for run time enable/disable */
	wmi_handle->events_logs_list = qdf_mem_malloc(num_of_diag_events_logs *
			sizeof(uint32_t));
	if (!wmi_handle->events_logs_list) {
		WMI_LOGE("%s: event log list memory allocation failed",
				__func__);
		return -ENOMEM;
	}
	wmi_handle->num_of_diag_events_logs = num_of_diag_events_logs;

	/* Prepare the send buffer */
	buf_len = sizeof(*cmd) + WMI_TLV_HDR_SIZE +
		(num_of_diag_events_logs * sizeof(uint32_t));

	buf = wmi_buf_alloc(wmi_handle, buf_len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		qdf_mem_free(wmi_handle->events_logs_list);
		wmi_handle->events_logs_list = NULL;
		return -ENOMEM;
	}

	cmd = (wmi_diag_event_log_config_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_diag_event_log_config_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(
				wmi_diag_event_log_config_fixed_param));

	cmd->num_of_diag_events_logs = num_of_diag_events_logs;

	buf_ptr += sizeof(wmi_diag_event_log_config_fixed_param);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
			(num_of_diag_events_logs * sizeof(uint32_t)));

	cmd_args = (uint32_t *) (buf_ptr + WMI_TLV_HDR_SIZE);

	/* Populate the events */
	for (i = 0; i < num_of_diag_events_logs; i++) {
		/* Low freq (0) - Enable (1) the event
		 * High freq (1) - Disable (0) the event
		 */
		WMI_DIAG_ID_ENABLED_DISABLED_SET(cmd_args[i],
				!(WMI_DIAG_FREQUENCY_GET(evt_args[i])));
		/* Set the event ID */
		WMI_DIAG_ID_SET(cmd_args[i],
				WMI_DIAG_ID_GET(evt_args[i]));
		/* Set the type */
		WMI_DIAG_TYPE_SET(cmd_args[i],
				WMI_DIAG_TYPE_GET(evt_args[i]));
		/* Storing the event/log list in WMI */
		wmi_handle->events_logs_list[i] = evt_args[i];
	}

	if (wmi_unified_cmd_send(wmi_handle, buf, buf_len,
				WMI_DIAG_EVENT_LOG_CONFIG_CMDID)) {
		WMI_LOGE("%s: WMI_DIAG_EVENT_LOG_CONFIG_CMDID failed",
				__func__);
		wmi_buf_free(buf);
		/* Not clearing events_logs_list, though wmi cmd failed.
		 * Host can still have this list
		 */
		return -EINVAL;
	}

	return 0;
}

/**
 * send_enable_specific_fw_logs_cmd_tlv() - Start/Stop logging of diag log id
 * @wmi_handle: wmi handle
 * @start_log: Start logging related parameters
 *
 * Send the command to the FW based on which specific logging of diag
 * event/log id can be started/stopped
 *
 * Return: None
 */
QDF_STATUS send_enable_specific_fw_logs_cmd_tlv(wmi_unified_t wmi_handle,
		struct wmi_wifi_start_log *start_log)
{
	wmi_diag_event_log_config_fixed_param *cmd;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t len, count, log_level, i;
	uint32_t *cmd_args;
	uint32_t total_len;
	count = 0;

	if (!wmi_handle->events_logs_list) {
		WMI_LOGE("%s: Not received event/log list from FW, yet",
				__func__);
		return -ENOMEM;
	}
	/* total_len stores the number of events where BITS 17 and 18 are set.
	 * i.e., events of high frequency (17) and for extended debugging (18)
	 */
	total_len = 0;
	for (i = 0; i < wmi_handle->num_of_diag_events_logs; i++) {
		if ((WMI_DIAG_FREQUENCY_GET(wmi_handle->events_logs_list[i])) &&
		    (WMI_DIAG_EXT_FEATURE_GET(wmi_handle->events_logs_list[i])))
			total_len++;
	}

	len = sizeof(*cmd) + WMI_TLV_HDR_SIZE +
		(total_len * sizeof(uint32_t));

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}
	cmd = (wmi_diag_event_log_config_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;

	WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_diag_event_log_config_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(
				wmi_diag_event_log_config_fixed_param));

	cmd->num_of_diag_events_logs = total_len;

	buf_ptr += sizeof(wmi_diag_event_log_config_fixed_param);

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
			(total_len * sizeof(uint32_t)));

	cmd_args = (uint32_t *) (buf_ptr + WMI_TLV_HDR_SIZE);

	if (start_log->verbose_level >= LOG_LEVEL_ACTIVE)
		log_level = 1;
	else
		log_level = 0;

	WMI_LOGD("%s: Length:%d, Log_level:%d", __func__, total_len, log_level);
	for (i = 0; i < wmi_handle->num_of_diag_events_logs; i++) {
		uint32_t val = wmi_handle->events_logs_list[i];
		if ((WMI_DIAG_FREQUENCY_GET(val)) &&
				(WMI_DIAG_EXT_FEATURE_GET(val))) {

			WMI_DIAG_ID_SET(cmd_args[count],
					WMI_DIAG_ID_GET(val));
			WMI_DIAG_TYPE_SET(cmd_args[count],
					WMI_DIAG_TYPE_GET(val));
			WMI_DIAG_ID_ENABLED_DISABLED_SET(cmd_args[count],
					log_level);
			WMI_LOGD("%s: Idx:%d, val:%x", __func__, i, val);
			count++;
		}
	}

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_DIAG_EVENT_LOG_CONFIG_CMDID)) {
		WMI_LOGE("%s: WMI_DIAG_EVENT_LOG_CONFIG_CMDID failed",
				__func__);
		wmi_buf_free(buf);
		return -EINVAL;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_flush_logs_to_fw_cmd_tlv() - Send log flush command to FW
 * @wmi_handle: WMI handle
 *
 * This function is used to send the flush command to the FW,
 * that will flush the fw logs that are residue in the FW
 *
 * Return: None
 */
QDF_STATUS send_flush_logs_to_fw_cmd_tlv(wmi_unified_t wmi_handle)
{
	wmi_debug_mesg_flush_fixed_param *cmd;
	wmi_buf_t buf;
	int len = sizeof(*cmd);
	int ret;

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
		return -ENOMEM;
	}

	cmd = (wmi_debug_mesg_flush_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
			WMITLV_TAG_STRUC_wmi_debug_mesg_flush_fixed_param,
			WMITLV_GET_STRUCT_TLVLEN(
				wmi_debug_mesg_flush_fixed_param));
	cmd->reserved0 = 0;

	ret = wmi_unified_cmd_send(wmi_handle,
			buf,
			len,
			WMI_DEBUG_MESG_FLUSH_CMDID);
	if (ret != EOK) {
		WMI_LOGE("Failed to send WMI_DEBUG_MESG_FLUSH_CMDID");
		wmi_buf_free(buf);
		return -EINVAL;
	}
	WMI_LOGI("Sent WMI_DEBUG_MESG_FLUSH_CMDID to FW");

	return QDF_STATUS_SUCCESS;
}

/**
 * send_soc_set_pcl_cmd_tlv() - Send WMI_SOC_SET_PCL_CMDID to FW
 * @wmi_handle: wmi handle
 * @msg: PCL structure containing the PCL and the number of channels
 *
 * WMI_SOC_SET_PCL_CMDID provides a Preferred Channel List (PCL) to the WLAN
 * firmware. The DBS Manager is the consumer of this information in the WLAN
 * firmware. The channel list will be used when a Virtual DEVice (VDEV) needs
 * to migrate to a new channel without host driver involvement. An example of
 * this behavior is Legacy Fast Roaming (LFR 3.0). Generally, the host will
 * manage the channel selection without firmware involvement.
 *
 * Return: Success if the cmd is sent successfully to the firmware
 */
QDF_STATUS send_soc_set_pcl_cmd_tlv(wmi_unified_t wmi_handle,
				struct wmi_pcl_list *msg)
{
	wmi_soc_set_pcl_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint8_t *buf_ptr;
	uint32_t *cmd_args, i, len;

	len = sizeof(*cmd) +
		WMI_TLV_HDR_SIZE + (msg->pcl_len * sizeof(uint32_t));

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_soc_set_pcl_cmd_fixed_param *) wmi_buf_data(buf);
	buf_ptr = (uint8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_soc_set_pcl_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(wmi_soc_set_pcl_cmd_fixed_param));
	cmd->num_chan = msg->pcl_len;
	WMI_LOGI("%s: PCL len:%d", __func__, cmd->num_chan);

	buf_ptr += sizeof(wmi_soc_set_pcl_cmd_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
			(msg->pcl_len * sizeof(uint32_t)));
	cmd_args = (uint32_t *) (buf_ptr + WMI_TLV_HDR_SIZE);
	for (i = 0; i < msg->pcl_len ; i++) {
		cmd_args[i] = msg->pcl_list[i];
		WMI_LOGI("%s: PCL chan:%d", __func__, cmd_args[i]);
	}
	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_SOC_SET_PCL_CMDID)) {
		WMI_LOGE("%s: Failed to send WMI_SOC_SET_PCL_CMDID", __func__);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_soc_set_hw_mode_cmd_tlv() - Send WMI_SOC_SET_HW_MODE_CMDID to FW
 * @wmi_handle: wmi handle
 * @msg: Structure containing the following parameters
 *
 * - hw_mode_index: The HW_Mode field is a enumerated type that is selected
 * from the HW_Mode table, which is returned in the WMI_SERVICE_READY_EVENTID.
 *
 * Provides notification to the WLAN firmware that host driver is requesting a
 * HardWare (HW) Mode change. This command is needed to support iHelium in the
 * configurations that include the Dual Band Simultaneous (DBS) feature.
 *
 * Return: Success if the cmd is sent successfully to the firmware
 */
QDF_STATUS send_soc_set_hw_mode_cmd_tlv(wmi_unified_t wmi_handle,
				uint32_t hw_mode_index)
{
	wmi_soc_set_hw_mode_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint32_t len;

	len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_soc_set_hw_mode_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_soc_set_hw_mode_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(wmi_soc_set_hw_mode_cmd_fixed_param));
	cmd->hw_mode_index = hw_mode_index;
	WMI_LOGI("%s: HW mode index:%d", __func__, cmd->hw_mode_index);

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_SOC_SET_HW_MODE_CMDID)) {
		WMI_LOGE("%s: Failed to send WMI_SOC_SET_HW_MODE_CMDID",
			__func__);
		qdf_nbuf_free(buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_soc_set_dual_mac_config_cmd_tlv() - Set dual mac config to FW
 * @wmi_handle: wmi handle
 * @msg: Dual MAC config parameters
 *
 * Configures WLAN firmware with the dual MAC features
 *
 * Return: QDF_STATUS. 0 on success.
 */
QDF_STATUS send_soc_set_dual_mac_config_cmd_tlv(wmi_unified_t wmi_handle,
		struct wmi_dual_mac_config *msg)
{
	wmi_soc_set_dual_mac_config_cmd_fixed_param *cmd;
	wmi_buf_t buf;
	uint32_t len;

	len = sizeof(*cmd);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_FAILURE;
	}

	cmd = (wmi_soc_set_dual_mac_config_cmd_fixed_param *) wmi_buf_data(buf);
	WMITLV_SET_HDR(&cmd->tlv_header,
		WMITLV_TAG_STRUC_wmi_soc_set_dual_mac_config_cmd_fixed_param,
		WMITLV_GET_STRUCT_TLVLEN(
			wmi_soc_set_dual_mac_config_cmd_fixed_param));
	cmd->concurrent_scan_config_bits = msg->scan_config;
	cmd->fw_mode_config_bits = msg->fw_mode_config;
	WMI_LOGI("%s: scan_config:%x fw_mode_config:%x",
			__func__, msg->scan_config, msg->fw_mode_config);

	if (wmi_unified_cmd_send(wmi_handle, buf, len,
				WMI_SOC_SET_DUAL_MAC_CONFIG_CMDID)) {
		WMI_LOGE("%s: Failed to send WMI_SOC_SET_DUAL_MAC_CONFIG_CMDID",
				__func__);
		qdf_nbuf_free(buf);
	}
	return QDF_STATUS_SUCCESS;
}

/**
 * send_enable_arp_ns_offload_cmd_tlv() - enable ARP NS offload
 * @wma: wmi handle
 * @tpSirHostOffloadReq: offload request
 * @arp_only: flag
 *
 * To configure ARP NS off load data to firmware
 * when target goes to wow mode.
 *
 * Return: CDF Status
 */
QDF_STATUS send_enable_arp_ns_offload_cmd_tlv(wmi_unified_t wmi_handle,
			   struct host_offload_req_param *param, bool arp_only,
			   uint8_t vdev_id)
{
	int32_t i;
	int32_t res;
	WMI_SET_ARP_NS_OFFLOAD_CMD_fixed_param *cmd;
	WMI_NS_OFFLOAD_TUPLE *ns_tuple;
	WMI_ARP_OFFLOAD_TUPLE *arp_tuple;
	A_UINT8 *buf_ptr;
	wmi_buf_t buf;
	int32_t len;
	uint32_t count = 0, num_ns_ext_tuples = 0;


	if (!arp_only)
		count = param->num_ns_offload_count;
	/*
	 * TLV place holder size for array of NS tuples
	 * TLV place holder size for array of ARP tuples
	 */
	len = sizeof(WMI_SET_ARP_NS_OFFLOAD_CMD_fixed_param) + WMI_TLV_HDR_SIZE +
	      WMI_MAX_NS_OFFLOADS * sizeof(WMI_NS_OFFLOAD_TUPLE) + WMI_TLV_HDR_SIZE +
	      WMI_MAX_ARP_OFFLOADS * sizeof(WMI_ARP_OFFLOAD_TUPLE);

	/*
	 * If there are more than WMI_MAX_NS_OFFLOADS addresses then allocate
	 * extra length for extended NS offload tuples which follows ARP offload
	 * tuples. Host needs to fill this structure in following format:
	 * 2 NS ofload tuples
	 * 2 ARP offload tuples
	 * N numbers of extended NS offload tuples if HDD has given more than
	 * 2 NS offload addresses
	 */
	if (!arp_only && count > WMI_MAX_NS_OFFLOADS) {
		num_ns_ext_tuples = count - WMI_MAX_NS_OFFLOADS;
		len += WMI_TLV_HDR_SIZE + num_ns_ext_tuples *
					sizeof(WMI_NS_OFFLOAD_TUPLE);
	}

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		qdf_mem_free(param);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (A_UINT8 *) wmi_buf_data(buf);
	cmd = (WMI_SET_ARP_NS_OFFLOAD_CMD_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_WMI_SET_ARP_NS_OFFLOAD_CMD_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (WMI_SET_ARP_NS_OFFLOAD_CMD_fixed_param));
	cmd->flags = 0;
	cmd->vdev_id = vdev_id;
	if (!arp_only)
		cmd->num_ns_ext_tuples = num_ns_ext_tuples;

	WMI_LOGD("ARP NS Offload vdev_id: %d", cmd->vdev_id);

	/* Have copy of arp info to send along with NS, Since FW expects
	 * both ARP and NS info in single cmd */
	if (arp_only)
		qdf_mem_copy(&wmi_handle->arp_info, param,
			     sizeof(tSirHostOffloadReq));

	buf_ptr += sizeof(WMI_SET_ARP_NS_OFFLOAD_CMD_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       (WMI_MAX_NS_OFFLOADS * sizeof(WMI_NS_OFFLOAD_TUPLE)));
	buf_ptr += WMI_TLV_HDR_SIZE;
	for (i = 0; i < WMI_MAX_NS_OFFLOADS; i++) {
		ns_tuple = (WMI_NS_OFFLOAD_TUPLE *) buf_ptr;
		WMITLV_SET_HDR(&ns_tuple->tlv_header,
			       WMITLV_TAG_STRUC_WMI_NS_OFFLOAD_TUPLE,
			       (sizeof(WMI_NS_OFFLOAD_TUPLE) -
				WMI_TLV_HDR_SIZE));

		/* Fill data only for NS offload in the first ARP tuple for LA */
		if (!arp_only &&
		    ((param->enableOrDisable & SIR_OFFLOAD_ENABLE))) {
			ns_tuple->flags |= WMI_NSOFF_FLAGS_VALID;

#ifdef WLAN_NS_OFFLOAD
			/*Copy the target/solicitation/remote ip addr */
			if (param->nsOffloadInfo.
			    targetIPv6AddrValid[i])
				A_MEMCPY(&ns_tuple->target_ipaddr[0],
					 &param->nsOffloadInfo.
					 targetIPv6Addr[i],
					 sizeof(WMI_IPV6_ADDR));
			A_MEMCPY(&ns_tuple->solicitation_ipaddr,
				 &param->nsOffloadInfo.
				 selfIPv6Addr[i], sizeof(WMI_IPV6_ADDR));
			WMI_LOGD("NS solicitedIp: %pI6, targetIp: %pI6",
				 &param->nsOffloadInfo.selfIPv6Addr[i],
				 &param->nsOffloadInfo.
				 targetIPv6Addr[i]);

			/* target MAC is optional, check if it is valid,
			 * if this is not valid, the target will use the known
			 * local MAC address rather than the tuple
			 */
			WMI_CHAR_ARRAY_TO_MAC_ADDR(param->
					nsOffloadInfo.self_macaddr.bytes,
					&ns_tuple->target_mac);
#endif /* WLAN_NS_OFFLOAD */
			if ((ns_tuple->target_mac.mac_addr31to0 != 0) ||
			    (ns_tuple->target_mac.mac_addr47to32 != 0)) {
				ns_tuple->flags |= WMI_NSOFF_FLAGS_MAC_VALID;
			}
		}
		buf_ptr += sizeof(WMI_NS_OFFLOAD_TUPLE);
	}

	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
		       (WMI_MAX_ARP_OFFLOADS * sizeof(WMI_ARP_OFFLOAD_TUPLE)));
	buf_ptr += WMI_TLV_HDR_SIZE;
	for (i = 0; i < WMI_MAX_ARP_OFFLOADS; i++) {
		arp_tuple = (WMI_ARP_OFFLOAD_TUPLE *) buf_ptr;
		WMITLV_SET_HDR(&arp_tuple->tlv_header,
			       WMITLV_TAG_STRUC_WMI_ARP_OFFLOAD_TUPLE,
			       WMITLV_GET_STRUCT_TLVLEN(WMI_ARP_OFFLOAD_TUPLE));

		/* Fill data for ARP and NS in the first tupple for LA */
		if ((wmi_handle->arp_info.enableOrDisable & SIR_OFFLOAD_ENABLE)
		    && (i == 0)) {
			/*Copy the target ip addr and flags */
			arp_tuple->flags = WMI_ARPOFF_FLAGS_VALID;
			A_MEMCPY(&arp_tuple->target_ipaddr,
				 wmi_handle->arp_info.params.hostIpv4Addr,
				 SIR_IPV4_ADDR_LEN);
			WMI_LOGD("ARPOffload IP4 address: %pI4",
				 wmi_handle->arp_info.params.hostIpv4Addr);
		}
		buf_ptr += sizeof(WMI_ARP_OFFLOAD_TUPLE);
	}

	/* Populate extended NS offload tuples */
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC,
					(num_ns_ext_tuples*sizeof(WMI_NS_OFFLOAD_TUPLE)));
	buf_ptr += WMI_TLV_HDR_SIZE;

	if (num_ns_ext_tuples) {
		for (i = WMI_MAX_NS_OFFLOADS; i < count; i++) {
			ns_tuple = (WMI_NS_OFFLOAD_TUPLE *)buf_ptr;
			WMITLV_SET_HDR(&ns_tuple->tlv_header,
				WMITLV_TAG_STRUC_WMI_NS_OFFLOAD_TUPLE,
				(sizeof(WMI_NS_OFFLOAD_TUPLE)-WMI_TLV_HDR_SIZE));

			/* Fill data only for NS offload in the first ARP tuple for LA */
			if (!arp_only  &&
				((param->enableOrDisable & SIR_OFFLOAD_ENABLE))) {
				ns_tuple->flags |= WMI_NSOFF_FLAGS_VALID;
#ifdef WLAN_NS_OFFLOAD
				/*Copy the target/solicitation/remote ip addr */
				if (param->nsOffloadInfo.targetIPv6AddrValid[i])
					A_MEMCPY(&ns_tuple->target_ipaddr[0],
						&param->nsOffloadInfo.targetIPv6Addr[i],
						sizeof(WMI_IPV6_ADDR));
				A_MEMCPY(&ns_tuple->solicitation_ipaddr,
					&param->nsOffloadInfo.selfIPv6Addr[i],
					sizeof(WMI_IPV6_ADDR));
				WMI_LOGD("Index %d NS solicitedIp: %pI6, targetIp: %pI6", i,
					&param->nsOffloadInfo.selfIPv6Addr[i],
					&param->nsOffloadInfo.targetIPv6Addr[i]);

				/* target MAC is optional, check if it is valid, if this is not valid,
				 * the target will use the known local MAC address rather than the tuple */
				 WMI_CHAR_ARRAY_TO_MAC_ADDR(
					param->nsOffloadInfo.self_macaddr.bytes,
					&ns_tuple->target_mac);
#endif
				if ((ns_tuple->target_mac.mac_addr31to0 != 0) ||
					(ns_tuple->target_mac.mac_addr47to32 != 0)) {
					ns_tuple->flags |= WMI_NSOFF_FLAGS_MAC_VALID;
				}
			}
			buf_ptr += sizeof(WMI_NS_OFFLOAD_TUPLE);
		}
	}

	res = wmi_unified_cmd_send(wmi_handle, buf, len,
				     WMI_SET_ARP_NS_OFFLOAD_CMDID);
	if (res) {
		WMI_LOGE("Failed to enable ARP NDP/NSffload");
		wmi_buf_free(buf);
		qdf_mem_free(param);
		return QDF_STATUS_E_FAILURE;
	}

	qdf_mem_free(param);
	return QDF_STATUS_SUCCESS;
}

/**
 * send_set_ssid_hotlist_cmd_tlv() - Handle an SSID hotlist set request
 * @wmi_handle: wmi handle
 * @request: SSID hotlist set request
 *
 * Return: QDF_STATUS enumeration
 */
QDF_STATUS
send_set_ssid_hotlist_cmd_tlv(wmi_unified_t wmi_handle,
		     struct ssid_hotlist_request_params *request)
{
	wmi_extscan_configure_hotlist_ssid_monitor_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint32_t len;
	uint32_t array_size;
	uint8_t *buf_ptr;

	/* length of fixed portion */
	len = sizeof(*cmd);

	/* length of variable portion */
	array_size =
		request->ssid_count * sizeof(wmi_extscan_hotlist_ssid_entry);
	len += WMI_TLV_HDR_SIZE + array_size;

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(wmi_buf);
	cmd = (wmi_extscan_configure_hotlist_ssid_monitor_cmd_fixed_param *)
						buf_ptr;
	WMITLV_SET_HDR
		(&cmd->tlv_header,
		 WMITLV_TAG_STRUC_wmi_extscan_configure_hotlist_ssid_monitor_cmd_fixed_param,
		 WMITLV_GET_STRUCT_TLVLEN
			(wmi_extscan_configure_hotlist_ssid_monitor_cmd_fixed_param));

	cmd->request_id = request->request_id;
	cmd->requestor_id = 0;
	cmd->vdev_id = request->session_id;
	cmd->table_id = 0;
	cmd->lost_ap_scan_count = request->lost_ssid_sample_size;
	cmd->total_entries = request->ssid_count;
	cmd->num_entries_in_page = request->ssid_count;
	cmd->first_entry_index = 0;

	buf_ptr += sizeof(*cmd);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_STRUC, array_size);

	if (request->ssid_count) {
		wmi_extscan_hotlist_ssid_entry *entry;
		int i;

		buf_ptr += WMI_TLV_HDR_SIZE;
		entry = (wmi_extscan_hotlist_ssid_entry *)buf_ptr;
		for (i = 0; i < request->ssid_count; i++) {
			WMITLV_SET_HDR
				(entry,
				 WMITLV_TAG_ARRAY_STRUC,
				 WMITLV_GET_STRUCT_TLVLEN
					(wmi_extscan_hotlist_ssid_entry));
			entry->ssid.ssid_len = request->ssids[i].ssid.length;
			qdf_mem_copy(entry->ssid.ssid,
				     request->ssids[i].ssid.mac_ssid,
				     request->ssids[i].ssid.length);
			entry->band = request->ssids[i].band;
			entry->min_rssi = request->ssids[i].rssi_low;
			entry->max_rssi = request->ssids[i].rssi_high;
			entry++;
		}
		cmd->mode = WMI_EXTSCAN_MODE_START;
	} else {
		cmd->mode = WMI_EXTSCAN_MODE_STOP;
	}

	if (wmi_unified_cmd_send
		(wmi_handle, wmi_buf, len,
		 WMI_EXTSCAN_CONFIGURE_HOTLIST_SSID_MONITOR_CMDID)) {
		WMI_LOGE("%s: failed to send command", __func__);
		wmi_buf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_process_roam_synch_complete_cmd_tlv() - roam synch complete command to fw.
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * This function sends roam synch complete event to fw.
 *
 * Return: CDF STATUS
 */
QDF_STATUS send_process_roam_synch_complete_cmd_tlv(wmi_unified_t wmi_handle,
		 uint8_t vdev_id)
{
	wmi_roam_synch_complete_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint8_t *buf_ptr;
	uint16_t len;
	len = sizeof(wmi_roam_synch_complete_fixed_param);

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}
	cmd = (wmi_roam_synch_complete_fixed_param *) wmi_buf_data(wmi_buf);
	buf_ptr = (uint8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_roam_synch_complete_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_roam_synch_complete_fixed_param));
	cmd->vdev_id = vdev_id;
	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_ROAM_SYNCH_COMPLETE)) {
		WMI_LOGP("%s: failed to send roam synch confirmation",
			 __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_unit_test_cmd_tlv() - send unit test command to fw.
 * @wmi_handle: wmi handle
 * @wmi_utest: unit test command
 *
 * This function send unit test command to fw.
 *
 * Return: CDF STATUS
 */
QDF_STATUS send_unit_test_cmd_tlv(wmi_unified_t wmi_handle,
			       struct wmi_unit_test_cmd *wmi_utest)
{
	wmi_unit_test_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	uint8_t *buf_ptr;
	int i;
	uint16_t len, args_tlv_len;
	A_UINT32 *unit_test_cmd_args;

	args_tlv_len =
		WMI_TLV_HDR_SIZE + wmi_utest->num_args * sizeof(A_UINT32);
	len = sizeof(wmi_unit_test_cmd_fixed_param) + args_tlv_len;

	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmai_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_unit_test_cmd_fixed_param *) wmi_buf_data(wmi_buf);
	buf_ptr = (uint8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
		       WMITLV_TAG_STRUC_wmi_unit_test_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_unit_test_cmd_fixed_param));
	cmd->vdev_id = wmi_utest->vdev_id;
	cmd->module_id = wmi_utest->module_id;
	cmd->num_args = wmi_utest->num_args;
	buf_ptr += sizeof(wmi_unit_test_cmd_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
		       (wmi_utest->num_args * sizeof(uint32_t)));
	unit_test_cmd_args = (A_UINT32 *) (buf_ptr + WMI_TLV_HDR_SIZE);
	WMI_LOGI("%s: %d num of args = ", __func__, wmi_utest->num_args);
	for (i = 0; (i < wmi_utest->num_args && i < WMI_MAX_NUM_ARGS); i++) {
		unit_test_cmd_args[i] = wmi_utest->args[i];
		WMI_LOGI("%d,", wmi_utest->args[i]);
	}
	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
				 WMI_UNIT_TEST_CMDID)) {
		WMI_LOGP("%s: failed to send unit test command", __func__);
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_roam_invoke_cmd_tlv() - send roam invoke command to fw.
 * @wmi_handle: wma handle
 * @roaminvoke: roam invoke command
 *
 * Send roam invoke command to fw for fastreassoc.
 *
 * Return: CDF STATUS
 */
QDF_STATUS send_roam_invoke_cmd_tlv(wmi_unified_t wmi_handle,
		struct wmi_roam_invoke_cmd *roaminvoke,
		uint32_t ch_hz)
{
	wmi_roam_invoke_cmd_fixed_param *cmd;
	wmi_buf_t wmi_buf;
	u_int8_t *buf_ptr;
	u_int16_t len, args_tlv_len;
	A_UINT32 *channel_list;
	wmi_mac_addr *bssid_list;

	/* Host sends only one channel and one bssid */
	args_tlv_len = 2 * WMI_TLV_HDR_SIZE + sizeof(A_UINT32) +
			sizeof(wmi_mac_addr);
	len = sizeof(wmi_roam_invoke_cmd_fixed_param) + args_tlv_len;
	wmi_buf = wmi_buf_alloc(wmi_handle, len);
	if (!wmi_buf) {
		WMI_LOGE("%s: wmai_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	cmd = (wmi_roam_invoke_cmd_fixed_param *)wmi_buf_data(wmi_buf);
	buf_ptr = (u_int8_t *) cmd;
	WMITLV_SET_HDR(&cmd->tlv_header,
	WMITLV_TAG_STRUC_wmi_roam_invoke_cmd_fixed_param,
	WMITLV_GET_STRUCT_TLVLEN(wmi_roam_invoke_cmd_fixed_param));
	cmd->vdev_id = roaminvoke->vdev_id;
	cmd->flags = 0;
	cmd->roam_scan_mode = 0;
	cmd->roam_ap_sel_mode = 0;
	cmd->roam_delay = 0;
	cmd->num_chan = 1;
	cmd->num_bssid = 1;
	buf_ptr += sizeof(wmi_roam_invoke_cmd_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
				(sizeof(u_int32_t)));
	channel_list = (A_UINT32 *)(buf_ptr + WMI_TLV_HDR_SIZE);
	*channel_list = ch_hz;
	buf_ptr += sizeof(A_UINT32) + WMI_TLV_HDR_SIZE;
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_FIXED_STRUC,
				(sizeof(wmi_mac_addr)));
	bssid_list = (wmi_mac_addr *)(buf_ptr + WMI_TLV_HDR_SIZE);
	WMI_CHAR_ARRAY_TO_MAC_ADDR(roaminvoke->bssid, bssid_list);
	if (wmi_unified_cmd_send(wmi_handle, wmi_buf, len,
					WMI_ROAM_INVOKE_CMDID)) {
		WMI_LOGP("%s: failed to send roam invoke command", __func__);
		wmi_buf_free(wmi_buf);
		return QDF_STATUS_E_FAILURE;
	}

	return QDF_STATUS_SUCCESS;
}

/**
 * send_roam_scan_offload_cmd_tlv() - set roam offload command
 * @wmi_handle: wmi handle
 * @command: command
 * @vdev_id: vdev id
 *
 * This function set roam offload command to fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_roam_scan_offload_cmd_tlv(wmi_unified_t wmi_handle,
					 uint32_t command, uint32_t vdev_id)
{
	QDF_STATUS qdf_status;
	wmi_roam_scan_cmd_fixed_param *cmd_fp;
	wmi_buf_t buf = NULL;
	int status = 0;
	int len;
	uint8_t *buf_ptr;

	len = sizeof(wmi_roam_scan_cmd_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);

	cmd_fp = (wmi_roam_scan_cmd_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&cmd_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_roam_scan_cmd_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_roam_scan_cmd_fixed_param));
	cmd_fp->vdev_id = vdev_id;
	cmd_fp->command_arg = command;

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_ROAM_SCAN_CMD);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_ROAM_SCAN_CMD returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	WMI_LOGI("%s: WMI --> WMI_ROAM_SCAN_CMD", __func__);
	return QDF_STATUS_SUCCESS;

error:
	wmi_buf_free(buf);

	return qdf_status;
}

/**
 * send_roam_scan_offload_ap_profile_cmd_tlv() - set roam ap profile in fw
 * @wmi_handle: wmi handle
 * @ap_profile_p: ap profile
 * @vdev_id: vdev id
 *
 * Send WMI_ROAM_AP_PROFILE to firmware
 *
 * Return: CDF status
 */
QDF_STATUS send_roam_scan_offload_ap_profile_cmd_tlv(wmi_unified_t wmi_handle,
					    wmi_ap_profile *ap_profile_p,
					    uint32_t vdev_id)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_buf_t buf = NULL;
	int status = 0;
	int len;
	uint8_t *buf_ptr;
	wmi_roam_ap_profile_fixed_param *roam_ap_profile_fp;

	len = sizeof(wmi_roam_ap_profile_fixed_param) + sizeof(wmi_ap_profile);

	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	roam_ap_profile_fp = (wmi_roam_ap_profile_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&roam_ap_profile_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_roam_ap_profile_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_roam_ap_profile_fixed_param));
	/* fill in threshold values */
	roam_ap_profile_fp->vdev_id = vdev_id;
	roam_ap_profile_fp->id = 0;
	buf_ptr += sizeof(wmi_roam_ap_profile_fixed_param);

	qdf_mem_copy(buf_ptr, ap_profile_p, sizeof(wmi_ap_profile));
	WMITLV_SET_HDR(buf_ptr,
		       WMITLV_TAG_STRUC_wmi_ap_profile,
		       WMITLV_GET_STRUCT_TLVLEN(wmi_ap_profile));
	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_ROAM_AP_PROFILE);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_ROAM_AP_PROFILE returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	WMI_LOGI("WMI --> WMI_ROAM_AP_PROFILE and other parameters");
	return QDF_STATUS_SUCCESS;
error:
	wmi_buf_free(buf);

	return qdf_status;
}

/**
 * send_roam_scan_offload_scan_period_cmd_tlv() - set roam offload scan period
 * @wmi_handle: wmi handle
 * @scan_period: scan period
 * @scan_age: scan age
 * @vdev_id: vdev id
 *
 * Send WMI_ROAM_SCAN_PERIOD parameters to fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_roam_scan_offload_scan_period_cmd_tlv(wmi_unified_t wmi_handle,
					     uint32_t scan_period,
					     uint32_t scan_age,
					     uint32_t vdev_id)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_buf_t buf = NULL;
	int status = 0;
	int len;
	uint8_t *buf_ptr;
	wmi_roam_scan_period_fixed_param *scan_period_fp;

	/* Send scan period values */
	len = sizeof(wmi_roam_scan_period_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	scan_period_fp = (wmi_roam_scan_period_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&scan_period_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_roam_scan_period_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_roam_scan_period_fixed_param));
	/* fill in scan period values */
	scan_period_fp->vdev_id = vdev_id;
	scan_period_fp->roam_scan_period = scan_period; /* 20 seconds */
	scan_period_fp->roam_scan_age = scan_age;

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_ROAM_SCAN_PERIOD);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_ROAM_SCAN_PERIOD returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	WMI_LOGI("%s: WMI --> WMI_ROAM_SCAN_PERIOD roam_scan_period=%d, roam_scan_age=%d",
		__func__, scan_period, scan_age);
	return QDF_STATUS_SUCCESS;
error:
	wmi_buf_free(buf);

	return qdf_status;
}

/**
 * send_roam_scan_offload_chan_list_cmd_tlv() - set roam offload channel list
 * @wmi_handle: wmi handle
 * @chan_count: channel count
 * @chan_list: channel list
 * @list_type: list type
 * @vdev_id: vdev id
 *
 * Set roam offload channel list.
 *
 * Return: CDF status
 */
QDF_STATUS send_roam_scan_offload_chan_list_cmd_tlv(wmi_unified_t wmi_handle,
				   uint8_t chan_count,
				   uint8_t *chan_list,
				   uint8_t list_type, uint32_t vdev_id)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_buf_t buf = NULL;
	int status = 0;
	int len, list_tlv_len;
	int i;
	uint8_t *buf_ptr;
	wmi_roam_chan_list_fixed_param *chan_list_fp;
	A_UINT32 *roam_chan_list_array;

	if (chan_count == 0) {
		WMI_LOGD("%s : invalid number of channels %d", __func__,
			 chan_count);
		return QDF_STATUS_E_EMPTY;
	}
	/* Channel list is a table of 2 TLV's */
	list_tlv_len = WMI_TLV_HDR_SIZE + chan_count * sizeof(A_UINT32);
	len = sizeof(wmi_roam_chan_list_fixed_param) + list_tlv_len;
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	chan_list_fp = (wmi_roam_chan_list_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&chan_list_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_roam_chan_list_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_roam_chan_list_fixed_param));
	chan_list_fp->vdev_id = vdev_id;
	chan_list_fp->num_chan = chan_count;
	if (chan_count > 0 && list_type == CHANNEL_LIST_STATIC) {
		/* external app is controlling channel list */
		chan_list_fp->chan_list_type =
			WMI_ROAM_SCAN_CHAN_LIST_TYPE_STATIC;
	} else {
		/* umac supplied occupied channel list in LFR */
		chan_list_fp->chan_list_type =
			WMI_ROAM_SCAN_CHAN_LIST_TYPE_DYNAMIC;
	}

	buf_ptr += sizeof(wmi_roam_chan_list_fixed_param);
	WMITLV_SET_HDR(buf_ptr, WMITLV_TAG_ARRAY_UINT32,
		       (chan_list_fp->num_chan * sizeof(uint32_t)));
	roam_chan_list_array = (A_UINT32 *) (buf_ptr + WMI_TLV_HDR_SIZE);
	WMI_LOGI("%s: %d channels = ", __func__, chan_list_fp->num_chan);
	for (i = 0; ((i < chan_list_fp->num_chan) &&
		     (i < WMI_ROAM_MAX_CHANNELS)); i++) {
		roam_chan_list_array[i] = chan_list[i];
		WMI_LOGI("%d,", roam_chan_list_array[i]);
	}

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_ROAM_CHAN_LIST);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_ROAM_CHAN_LIST returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	WMI_LOGI("%s: WMI --> WMI_ROAM_SCAN_CHAN_LIST", __func__);
	return QDF_STATUS_SUCCESS;
error:
	wmi_buf_free(buf);

	return qdf_status;
}

/**
 * send_roam_scan_offload_rssi_change_cmd_tlv() - set roam offload RSSI th
 * @wmi_handle: wmi handle
 * @rssi_change_thresh: RSSI Change threshold
 * @bcn_rssi_weight: beacon RSSI weight
 * @vdev_id: vdev id
 *
 * Send WMI_ROAM_SCAN_RSSI_CHANGE_THRESHOLD parameters to fw.
 *
 * Return: CDF status
 */
QDF_STATUS send_roam_scan_offload_rssi_change_cmd_tlv(wmi_unified_t wmi_handle,
	uint32_t vdev_id,
	int32_t rssi_change_thresh,
	uint32_t bcn_rssi_weight,
	uint32_t hirssi_delay_btw_scans)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	wmi_buf_t buf = NULL;
	int status = 0;
	int len;
	uint8_t *buf_ptr;
	wmi_roam_scan_rssi_change_threshold_fixed_param *rssi_change_fp;

	/* Send rssi change parameters */
	len = sizeof(wmi_roam_scan_rssi_change_threshold_fixed_param);
	buf = wmi_buf_alloc(wmi_handle, len);
	if (!buf) {
		WMI_LOGE("%s : wmi_buf_alloc failed", __func__);
		return QDF_STATUS_E_NOMEM;
	}

	buf_ptr = (uint8_t *) wmi_buf_data(buf);
	rssi_change_fp =
		(wmi_roam_scan_rssi_change_threshold_fixed_param *) buf_ptr;
	WMITLV_SET_HDR(&rssi_change_fp->tlv_header,
		       WMITLV_TAG_STRUC_wmi_roam_scan_rssi_change_threshold_fixed_param,
		       WMITLV_GET_STRUCT_TLVLEN
			       (wmi_roam_scan_rssi_change_threshold_fixed_param));
	/* fill in rssi change threshold (hysteresis) values */
	rssi_change_fp->vdev_id = vdev_id;
	rssi_change_fp->roam_scan_rssi_change_thresh = rssi_change_thresh;
	rssi_change_fp->bcn_rssi_weight = bcn_rssi_weight;
	rssi_change_fp->hirssi_delay_btw_scans = hirssi_delay_btw_scans;

	status = wmi_unified_cmd_send(wmi_handle, buf,
				      len, WMI_ROAM_SCAN_RSSI_CHANGE_THRESHOLD);
	if (status != EOK) {
		WMI_LOGE("wmi_unified_cmd_send WMI_ROAM_SCAN_RSSI_CHANGE_THRESHOLD returned Error %d",
			status);
		qdf_status = QDF_STATUS_E_FAILURE;
		goto error;
	}

	WMI_LOGI(FL("roam_scan_rssi_change_thresh=%d, bcn_rssi_weight=%d"),
		rssi_change_thresh, bcn_rssi_weight);
	WMI_LOGI(FL("hirssi_delay_btw_scans=%d"), hirssi_delay_btw_scans);
	return QDF_STATUS_SUCCESS;
error:
	wmi_buf_free(buf);

	return qdf_status;
}

/** wmi_get_hotlist_entries_per_page() - hotlist entries per page
 * @wmi_handle: wmi handle.
 * @cmd: size of command structure.
 * @per_entry_size: per entry size.
 *
 * This utility function calculates how many hotlist entries can
 * fit in one page.
 *
 * Return: number of entries
 */
static inline int wmi_get_hotlist_entries_per_page(wmi_unified_t wmi_handle,
						   size_t cmd_size,
						   size_t per_entry_size)
{
	uint32_t avail_space = 0;
	int num_entries = 0;
	uint16_t max_msg_len = wmi_get_max_msg_len(wmi_handle);

	/* Calculate number of hotlist entries that can
	 * be passed in wma message request.
	 */
	avail_space = max_msg_len - cmd_size;
	num_entries = avail_space / per_entry_size;
	return num_entries;
}

/**
 * send_get_buf_extscan_hotlist_cmd_tlv() - prepare hotlist command
 * @wmi_handle: wmi handle
 * @photlist: hotlist command params
 * @buf_len: buffer length
 *
 * This function fills individual elements for  hotlist request and
 * TLV for bssid entries
 *
 * Return: CDF Status.
 */
QDF_STATUS send_get_buf_extscan_hotlist_cmd_tlv(wmi_unified_t wmi_handle,
					   struct ext_scan_setbssi_hotlist_params *
					   photlist, int *buf_len)
{
	wmi_extscan_configure_hotlist_monitor_cmd_fixed_param *cmd = NULL;
	wmi_extscan_hotlist_entry *dest_hotlist;
	struct ap_threshold_params *src_ap = photlist->ap;
	wmi_buf_t buf;
	uint8_t *buf_ptr;

	int j, index = 0;
	int cmd_len = 0;
	int num_entries;
	int min_entries = 0;
	int numap = photlist->numAp;
	int len = sizeof(*cmd);

	len += WMI_TLV_HDR_SIZE;
	cmd_len = len;

	num_entries = wmi_get_hotlist_entries_per_page(wmi_handle,
							cmd_len,
							sizeof(*dest_hotlist));
	/* setbssid hotlist expects the bssid list
	 * to be non zero value
	 */
	if (!numap) {
		WMI_LOGE("%s: Invalid number of bssid's", __func__);
		return QDF_STATUS_E_INVAL;
	}

	/* Split the hot list entry pages and send multiple command
	 * requests if the buffer reaches the maximum request size
	 */
	while (index < numap) {
		min_entries = QDF_MIN(num_entries, numap);
		len += min_entries * sizeof(wmi_extscan_hotlist_entry);
		buf = wmi_buf_alloc(wmi_handle, len);
		if (!buf) {
			WMI_LOGP("%s: wmi_buf_alloc failed", __func__);
			return QDF_STATUS_E_FAILURE;
		}
		buf_ptr = (uint8_t *) wmi_buf_data(buf);
		cmd = (wmi_extscan_configure_hotlist_monitor_cmd_fixed_param *)
		      buf_ptr;
		WMITLV_SET_HDR(&cmd->tlv_header,
			       WMITLV_TAG_STRUC_wmi_extscan_configure_hotlist_monitor_cmd_fixed_param,
			       WMITLV_GET_STRUCT_TLVLEN
				       (wmi_extscan_configure_hotlist_monitor_cmd_fixed_param));

		/* Multiple requests are sent until the num_entries_in_page
		 * matches the total_entries
		 */
		cmd->request_id = photlist->requestId;
		cmd->vdev_id = photlist->sessionId;
		cmd->total_entries = numap;
		cmd->mode = 1;
		cmd->num_entries_in_page = min_entries;
		cmd->lost_ap_scan_count = photlist->lost_ap_sample_size;
		cmd->first_entry_index = index;

		WMI_LOGD("%s: vdev id:%d total_entries: %d num_entries: %d lost_ap_sample_size: %d",
			__func__, cmd->vdev_id, cmd->total_entries,
			cmd->num_entries_in_page,
			cmd->lost_ap_scan_count);

		buf_ptr += sizeof(*cmd);
		WMITLV_SET_HDR(buf_ptr,
			       WMITLV_TAG_ARRAY_STRUC,
			       min_entries * sizeof(wmi_extscan_hotlist_entry));
		dest_hotlist = (wmi_extscan_hotlist_entry *)
			       (buf_ptr + WMI_TLV_HDR_SIZE);

		/* Populate bssid, channel info and rssi
		 * for the bssid's that are sent as hotlists.
		 */
		for (j = 0; j < min_entries; j++) {
			WMITLV_SET_HDR(dest_hotlist,
				       WMITLV_TAG_STRUC_wmi_extscan_bucket_cmd_fixed_param,
				       WMITLV_GET_STRUCT_TLVLEN
					       (wmi_extscan_hotlist_entry));

			dest_hotlist->min_rssi = src_ap->low;
			WMI_CHAR_ARRAY_TO_MAC_ADDR(src_ap->bssid.bytes,
						   &dest_hotlist->bssid);

			WMI_LOGD("%s:channel:%d min_rssi %d",
				 __func__, dest_hotlist->channel,
				 dest_hotlist->min_rssi);
			WMI_LOGD
				("%s: bssid mac_addr31to0: 0x%x, mac_addr47to32: 0x%x",
				__func__, dest_hotlist->bssid.mac_addr31to0,
				dest_hotlist->bssid.mac_addr47to32);
			dest_hotlist++;
			src_ap++;
		}
		buf_ptr += WMI_TLV_HDR_SIZE +
			   (min_entries * sizeof(wmi_extscan_hotlist_entry));

		if (wmi_unified_cmd_send(wmi_handle, buf, len,
					 WMI_EXTSCAN_CONFIGURE_HOTLIST_MONITOR_CMDID)) {
			WMI_LOGE("%s: failed to send command", __func__);
			qdf_nbuf_free(buf);
			return QDF_STATUS_E_FAILURE;
		}
		index = index + min_entries;
		num_entries = numap - min_entries;
		len = cmd_len;
	}
	return QDF_STATUS_SUCCESS;
}

struct wmi_ops tlv_ops =  {
	.send_vdev_create_cmd = send_vdev_create_cmd_tlv,
	.send_vdev_delete_cmd = send_vdev_delete_cmd_tlv,
	.send_vdev_down_cmd = send_vdev_down_cmd_tlv,
	.send_peer_flush_tids_cmd = send_peer_flush_tids_cmd_tlv,
	.send_peer_param_cmd = send_peer_param_cmd_tlv,
	.send_vdev_up_cmd = send_vdev_up_cmd_tlv,
	.send_vdev_stop_cmd = send_vdev_stop_cmd_tlv,
	.send_peer_create_cmd = send_peer_create_cmd_tlv,
	.send_peer_delete_cmd = send_peer_delete_cmd_tlv,
	.send_green_ap_ps_cmd = send_green_ap_ps_cmd_tlv,
	.send_pdev_utf_cmd = send_pdev_utf_cmd_tlv,
	.send_pdev_param_cmd = send_pdev_param_cmd_tlv,
	.send_suspend_cmd = send_suspend_cmd_tlv,
	.send_resume_cmd = send_resume_cmd_tlv,
	.send_wow_enable_cmd = send_wow_enable_cmd_tlv,
	.send_set_ap_ps_param_cmd = send_set_ap_ps_param_cmd_tlv,
	.send_set_sta_ps_param_cmd = send_set_sta_ps_param_cmd_tlv,
	.send_crash_inject_cmd = send_crash_inject_cmd_tlv,
	.send_dbglog_cmd = send_dbglog_cmd_tlv,
	.send_vdev_set_param_cmd = send_vdev_set_param_cmd_tlv,
	.send_stats_request_cmd = send_stats_request_cmd_tlv,
	.send_packet_log_enable_cmd = send_packet_log_enable_cmd_tlv,
	.send_beacon_send_cmd = send_beacon_send_cmd_tlv,
	.send_peer_assoc_cmd = send_peer_assoc_cmd_tlv,
	.send_scan_start_cmd = send_scan_start_cmd_tlv,
	.send_scan_stop_cmd = send_scan_stop_cmd_tlv,
	.send_scan_chan_list_cmd = send_scan_chan_list_cmd_tlv,
	.send_mgmt_cmd = send_mgmt_cmd_tlv,
	.send_modem_power_state_cmd = send_modem_power_state_cmd_tlv,
	.send_set_sta_ps_mode_cmd = send_set_sta_ps_mode_cmd_tlv,
	.send_set_sta_uapsd_auto_trig_cmd =
		send_set_sta_uapsd_auto_trig_cmd_tlv,
	.send_get_temperature_cmd = send_get_temperature_cmd_tlv,
	.send_set_p2pgo_oppps_req_cmd = send_set_p2pgo_oppps_req_cmd_tlv,
	.send_set_p2pgo_noa_req_cmd = send_set_p2pgo_noa_req_cmd_tlv,
	.send_set_smps_params_cmd = send_set_smps_params_cmd_tlv,
	.send_set_mimops_cmd = send_set_mimops_cmd_tlv,
	.send_ocb_set_utc_time_cmd = send_ocb_set_utc_time_cmd_tlv,
	.send_ocb_get_tsf_timer_cmd = send_ocb_get_tsf_timer_cmd_tlv,
	.send_dcc_clear_stats_cmd = send_dcc_clear_stats_cmd_tlv,
	.send_dcc_get_stats_cmd = send_dcc_get_stats_cmd_tlv,
	.send_dcc_update_ndl_cmd = send_dcc_update_ndl_cmd_tlv,
	.send_ocb_set_config_cmd = send_ocb_set_config_cmd_tlv,
	.send_ocb_stop_timing_advert_cmd = send_ocb_stop_timing_advert_cmd_tlv,
	.send_ocb_start_timing_advert_cmd =
		send_ocb_start_timing_advert_cmd_tlv,
	.send_set_enable_disable_mcc_adaptive_scheduler_cmd =
		 send_set_enable_disable_mcc_adaptive_scheduler_cmd_tlv,
	.send_set_mcc_channel_time_latency_cmd =
			 send_set_mcc_channel_time_latency_cmd_tlv,
	.send_set_mcc_channel_time_quota_cmd =
			 send_set_mcc_channel_time_quota_cmd_tlv,
	.send_set_thermal_mgmt_cmd = send_set_thermal_mgmt_cmd_tlv,
	.send_lro_config_cmd = send_lro_config_cmd_tlv,
	.send_bcn_buf_ll_cmd = send_bcn_buf_ll_cmd_tlv,
	.send_set_sta_sa_query_param_cmd = send_set_sta_sa_query_param_cmd_tlv,
	.send_set_sta_keep_alive_cmd = send_set_sta_keep_alive_cmd_tlv,
	.send_vdev_set_gtx_cfg_cmd = send_vdev_set_gtx_cfg_cmd_tlv,
	.send_process_update_edca_param_cmd =
				 send_process_update_edca_param_cmd_tlv,
	.send_probe_rsp_tmpl_send_cmd =
				send_probe_rsp_tmpl_send_cmd_tlv,
	.send_p2p_go_set_beacon_ie_cmd =
				send_p2p_go_set_beacon_ie_cmd_tlv,
	.send_set_gateway_params_cmd =
				send_set_gateway_params_cmd_tlv,
	.send_set_rssi_monitoring_cmd =
			 send_set_rssi_monitoring_cmd_tlv,
	.send_scan_probe_setoui_cmd =
				send_scan_probe_setoui_cmd_tlv,
	.send_reset_passpoint_network_list_cmd =
				send_reset_passpoint_network_list_cmd_tlv,
	.send_set_passpoint_network_list_cmd =
			 send_set_passpoint_network_list_cmd_tlv,
	.send_set_epno_network_list_cmd =
			 send_set_epno_network_list_cmd_tlv,
	.send_ipa_offload_control_cmd =
			 send_ipa_offload_control_cmd_tlv,
	.send_extscan_get_capabilities_cmd =
			 send_extscan_get_capabilities_cmd_tlv,
	.send_extscan_get_cached_results_cmd =
		 send_extscan_get_cached_results_cmd_tlv,
	.send_extscan_stop_change_monitor_cmd =
		  send_extscan_stop_change_monitor_cmd_tlv,
	.send_extscan_start_change_monitor_cmd =
		  send_extscan_start_change_monitor_cmd_tlv,
	.send_extscan_stop_hotlist_monitor_cmd =
		  send_extscan_stop_hotlist_monitor_cmd_tlv,
	.send_stop_extscan_cmd = send_stop_extscan_cmd_tlv,
	.send_start_extscan_cmd = send_start_extscan_cmd_tlv,
	.send_plm_stop_cmd = send_plm_stop_cmd_tlv,
	.send_plm_start_cmd = send_plm_start_cmd_tlv,
	.send_pno_stop_cmd = send_pno_stop_cmd_tlv,
	.send_pno_start_cmd = send_pno_start_cmd_tlv,
	.send_set_ric_req_cmd = send_set_ric_req_cmd_tlv,
	.send_process_ll_stats_clear_cmd = send_process_ll_stats_clear_cmd_tlv,
	.send_process_ll_stats_set_cmd = send_process_ll_stats_set_cmd_tlv,
	.send_process_ll_stats_get_cmd = send_process_ll_stats_get_cmd_tlv,
	.send_get_stats_cmd = send_get_stats_cmd_tlv,
	.send_snr_request_cmd = send_snr_request_cmd_tlv,
	.send_snr_cmd = send_snr_cmd_tlv,
	.send_link_status_req_cmd = send_link_status_req_cmd_tlv,
	.send_lphb_config_hbenable_cmd = send_lphb_config_hbenable_cmd_tlv,
	.send_lphb_config_tcp_params_cmd = send_lphb_config_tcp_params_cmd_tlv,
	.send_lphb_config_udp_params_cmd = send_lphb_config_udp_params_cmd_tlv,
	.send_lphb_config_udp_pkt_filter_cmd =
		send_lphb_config_udp_pkt_filter_cmd_tlv,
	.send_process_dhcp_ind_cmd = send_process_dhcp_ind_cmd_tlv,
	.send_get_link_speed_cmd = send_get_link_speed_cmd_tlv,
	.send_egap_conf_params_cmd = send_egap_conf_params_cmd_tlv,
	.send_fw_profiling_cmd = send_fw_profiling_cmd_tlv,
	.send_csa_offload_enable_cmd = send_csa_offload_enable_cmd_tlv,
	.send_wow_sta_ra_filter_cmd = send_wow_sta_ra_filter_cmd_tlv,
	.send_nat_keepalive_en_cmd = send_nat_keepalive_en_cmd_tlv,
	.send_start_oem_data_cmd = send_start_oem_data_cmd_tlv,
	.send_dfs_phyerr_filter_offload_en_cmd =
		 send_dfs_phyerr_filter_offload_en_cmd_tlv,
	.send_pktlog_wmi_send_cmd = send_pktlog_wmi_send_cmd_tlv,
	.send_add_wow_wakeup_event_cmd = send_add_wow_wakeup_event_cmd_tlv,
	.send_wow_patterns_to_fw_cmd = send_wow_patterns_to_fw_cmd_tlv,
	.send_wow_delete_pattern_cmd = send_wow_delete_pattern_cmd_tlv,
	.send_host_wakeup_ind_to_fw_cmd = send_host_wakeup_ind_to_fw_cmd_tlv,
	.send_del_ts_cmd = send_del_ts_cmd_tlv,
	.send_aggr_qos_cmd = send_aggr_qos_cmd_tlv,
	.send_add_ts_cmd = send_add_ts_cmd_tlv,
	.send_enable_disable_packet_filter_cmd =
		send_enable_disable_packet_filter_cmd_tlv,
	.send_config_packet_filter_cmd = send_config_packet_filter_cmd_tlv,
	.send_add_clear_mcbc_filter_cmd = send_add_clear_mcbc_filter_cmd_tlv,
	.send_gtk_offload_cmd = send_gtk_offload_cmd_tlv,
	.send_process_gtk_offload_getinfo_cmd =
			send_process_gtk_offload_getinfo_cmd_tlv,
	.send_process_add_periodic_tx_ptrn_cmd =
		send_process_add_periodic_tx_ptrn_cmd_tlv,
	.send_process_del_periodic_tx_ptrn_cmd =
		send_process_del_periodic_tx_ptrn_cmd_tlv,
	.send_stats_ext_req_cmd = send_stats_ext_req_cmd_tlv,
	.send_enable_ext_wow_cmd = send_enable_ext_wow_cmd_tlv,
	.send_set_app_type2_params_in_fw_cmd =
		send_set_app_type2_params_in_fw_cmd_tlv,
	.send_set_auto_shutdown_timer_cmd =
		send_set_auto_shutdown_timer_cmd_tlv,
	.send_nan_req_cmd = send_nan_req_cmd_tlv,
	.send_process_dhcpserver_offload_cmd =
		send_process_dhcpserver_offload_cmd_tlv,
	.send_set_led_flashing_cmd = send_set_led_flashing_cmd_tlv,
	.send_process_ch_avoid_update_cmd =
		send_process_ch_avoid_update_cmd_tlv,
	.send_regdomain_info_to_fw_cmd = send_regdomain_info_to_fw_cmd_tlv,
	.send_set_tdls_offchan_mode_cmd = send_set_tdls_offchan_mode_cmd_tlv,
	.send_update_fw_tdls_state_cmd = send_update_fw_tdls_state_cmd_tlv,
	.send_update_tdls_peer_state_cmd = send_update_tdls_peer_state_cmd_tlv,
	.send_process_fw_mem_dump_cmd = send_process_fw_mem_dump_cmd_tlv,
	.send_process_set_ie_info_cmd = send_process_set_ie_info_cmd_tlv,
	.send_init_cmd = send_init_cmd_tlv,
	.save_fw_version_cmd = save_fw_version_cmd_tlv,
	.check_and_update_fw_version_cmd =
		 check_and_update_fw_version_cmd_tlv,
	.send_saved_init_cmd = send_saved_init_cmd_tlv,
	.send_set_base_macaddr_indicate_cmd =
		 send_set_base_macaddr_indicate_cmd_tlv,
	.send_log_supported_evt_cmd = send_log_supported_evt_cmd_tlv,
	.send_enable_specific_fw_logs_cmd =
		 send_enable_specific_fw_logs_cmd_tlv,
	.send_flush_logs_to_fw_cmd = send_flush_logs_to_fw_cmd_tlv,
	.send_soc_set_pcl_cmd = send_soc_set_pcl_cmd_tlv,
	.send_soc_set_hw_mode_cmd = send_soc_set_hw_mode_cmd_tlv,
	.send_soc_set_dual_mac_config_cmd =
		 send_soc_set_dual_mac_config_cmd_tlv,
	.send_enable_arp_ns_offload_cmd =
		 send_enable_arp_ns_offload_cmd_tlv,
	.send_app_type1_params_in_fw_cmd =
		 send_app_type1_params_in_fw_cmd_tlv,
	.send_set_ssid_hotlist_cmd = send_set_ssid_hotlist_cmd_tlv,
	.send_process_roam_synch_complete_cmd =
		 send_process_roam_synch_complete_cmd_tlv,
	.send_unit_test_cmd = send_unit_test_cmd_tlv,
	.send_roam_invoke_cmd = send_roam_invoke_cmd_tlv,
	.send_roam_scan_offload_cmd = send_roam_scan_offload_cmd_tlv,
	.send_roam_scan_offload_ap_profile_cmd =
			send_roam_scan_offload_ap_profile_cmd_tlv,
	.send_roam_scan_offload_scan_period_cmd =
		 send_roam_scan_offload_scan_period_cmd_tlv,
	.send_roam_scan_offload_chan_list_cmd =
		 send_roam_scan_offload_chan_list_cmd_tlv,
	.send_roam_scan_offload_rssi_change_cmd =
		 send_roam_scan_offload_rssi_change_cmd_tlv,
	.send_get_buf_extscan_hotlist_cmd =
		 send_get_buf_extscan_hotlist_cmd_tlv,
	/* TODO - Add other tlv apis here */
};

/**
 * wmi_get_tlv_ops() - gives pointer to wmi tlv ops
 *
 * Return: pointer to wmi tlv ops
 */
struct wmi_ops *wmi_get_tlv_ops(void)
{
	return &tlv_ops;
}

