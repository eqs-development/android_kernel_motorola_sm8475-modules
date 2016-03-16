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
#include "athdefs.h"
#include "osapi_linux.h"
#include "a_types.h"
#include "a_debug.h"
#include "ol_if_athvar.h"
#include "ol_defines.h"
#include "wmi.h"
#include "wmi_unified_priv.h"
#include "wma_api.h"
#include "wmi_unified_param.h"

/**
 * wmi_unified_vdev_create_send() - send VDEV create command to fw
 * @wmi_handle: wmi handle
 * @param: pointer to hold vdev create parameter
 * @macaddr: vdev mac address
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_vdev_create_send(void *wmi_hdl,
				 uint8_t macaddr[IEEE80211_ADDR_LEN],
				 struct vdev_create_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_create_cmd)
		return wmi_handle->ops->send_vdev_create_cmd(wmi_handle,
			   macaddr, param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_vdev_delete_send() - send VDEV delete command to fw
 * @wmi_handle: wmi handle
 * @if_id: vdev id
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_vdev_delete_send(void *wmi_hdl,
					  uint8_t if_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_delete_cmd)
		return wmi_handle->ops->send_vdev_delete_cmd(wmi_handle,
			   if_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_vdev_stop_send() - send vdev stop command to fw
 * @wmi: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or erro code
 */
CDF_STATUS wmi_unified_vdev_stop_send(void *wmi_hdl,
					uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_stop_cmd)
		return wmi_handle->ops->send_vdev_stop_cmd(wmi_handle,
			   vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_vdev_down_send() - send vdev down command to fw
 * @wmi: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_vdev_down_send(void *wmi_hdl, uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_down_cmd)
		return wmi_handle->ops->send_vdev_down_cmd(wmi_handle, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_peer_flush_tids_send() - flush peer tids packets in fw
 * @wmi: wmi handle
 * @peer_addr: peer mac address
 * @param: pointer to hold peer flush tid parameter
 *
 * Return: 0 for sucess or error code
 */
CDF_STATUS wmi_unified_peer_flush_tids_send(void *wmi_hdl,
					 uint8_t peer_addr[IEEE80211_ADDR_LEN],
					 struct peer_flush_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_peer_flush_tids_cmd)
		return wmi_handle->ops->send_peer_flush_tids_cmd(wmi_handle,
				  peer_addr, param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_peer_delete_send() - send PEER delete command to fw
 * @wmi: wmi handle
 * @peer_addr: peer mac addr
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_peer_delete_send(void *wmi_hdl,
				    uint8_t
				    peer_addr[IEEE80211_ADDR_LEN],
				    uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_peer_delete_cmd)
		return wmi_handle->ops->send_peer_delete_cmd(wmi_handle,
				  peer_addr, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_set_peer_param() - set peer parameter in fw
 * @wmi_ctx: wmi handle
 * @peer_addr: peer mac address
 * @param    : pointer to hold peer set parameter
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_set_peer_param_send(void *wmi_hdl,
				uint8_t peer_addr[IEEE80211_ADDR_LEN],
				struct peer_set_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_peer_param_cmd)
		return wmi_handle->ops->send_peer_param_cmd(wmi_handle,
				peer_addr, param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_vdev_up_send() - send vdev up command in fw
 * @wmi: wmi handle
 * @bssid: bssid
 * @vdev_up_params: pointer to hold vdev up parameter
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_vdev_up_send(void *wmi_hdl,
			     uint8_t bssid[IEEE80211_ADDR_LEN],
				 struct vdev_up_params *params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_up_cmd)
		return wmi_handle->ops->send_vdev_up_cmd(wmi_handle, bssid,
					params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_peer_create_send() - send peer create command to fw
 * @wmi: wmi handle
 * @peer_addr: peer mac address
 * @peer_type: peer type
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_peer_create_send(void *wmi_hdl,
					struct peer_create_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_peer_create_cmd)
		return wmi_handle->ops->send_peer_create_cmd(wmi_handle, param);

	return CDF_STATUS_E_FAILURE;
}

#ifdef FEATURE_GREEN_AP
/**
 * wmi_unified_green_ap_ps_send() - enable green ap powersave command
 * @wmi_handle: wmi handle
 * @value: value
 * @mac_id: mac id to have radio context
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_green_ap_ps_send(void *wmi_hdl,
						uint32_t value, uint8_t mac_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_green_ap_ps_cmd)
		return wmi_handle->ops->send_green_ap_ps_cmd(wmi_handle, value,
				  mac_id);

	return CDF_STATUS_E_FAILURE;
}
#else
CDF_STATUS wmi_unified_green_ap_ps_send(void *wmi_hdl,
						uint32_t value, uint8_t mac_id)
{
	return 0;
}
#endif /* FEATURE_GREEN_AP */

/**
 * wmi_unified_pdev_utf_cmd() - send utf command to fw
 * @wmi_handle: wmi handle
 * @param: pointer to pdev_utf_params
 * @mac_id: mac id to have radio context
 *
 * Return: 0 for success or error code
 */
CDF_STATUS
wmi_unified_pdev_utf_cmd_send(void *wmi_hdl,
				struct pdev_utf_params *param,
				uint8_t mac_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_pdev_utf_cmd)
		return wmi_handle->ops->send_pdev_utf_cmd(wmi_handle, param,
				  mac_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_pdev_set_param() - set pdev parameters
 * @wmi_handle: wmi handle
 * @param: pointer to pdev parameter
 * @mac_id: radio context
 *
 * Return: 0 on success, errno on failure
 */
CDF_STATUS
wmi_unified_pdev_param_send(void *wmi_hdl,
			   struct pdev_params *param,
				uint8_t mac_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_pdev_param_cmd)
		return wmi_handle->ops->send_pdev_param_cmd(wmi_handle, param,
				  mac_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_suspend_send() - WMI suspend function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold suspend parameter
 *  @mac_id: radio context
 *
 *  Return: 0 on success and -ve on failure.
 */
CDF_STATUS wmi_unified_suspend_send(void *wmi_hdl,
				struct suspend_params *param,
				uint8_t mac_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_suspend_cmd)
		return wmi_handle->ops->send_suspend_cmd(wmi_handle, param,
				  mac_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_resume_send - WMI resume function
 *  @param wmi_handle      : handle to WMI.
 *  @mac_id: radio context
 *
 *  Return: 0 on success and -ve on failure.
 */
CDF_STATUS wmi_unified_resume_send(void *wmi_hdl,
				uint8_t mac_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_resume_cmd)
		return wmi_handle->ops->send_resume_cmd(wmi_handle,
				  mac_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_wow_enable_send() - WMI wow enable function
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold wow enable parameter
 *  @mac_id: radio context
 *
 *  Return: 0 on success and -ve on failure.
 */
CDF_STATUS wmi_unified_wow_enable_send(void *wmi_hdl,
				struct wow_cmd_params *param,
				uint8_t mac_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_wow_enable_cmd)
		return wmi_handle->ops->send_wow_enable_cmd(wmi_handle, param,
				  mac_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_ap_ps_cmd_send() - set ap powersave parameters
 * @wma_ctx: wma context
 * @peer_addr: peer mac address
 * @param: pointer to ap_ps parameter structure
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_ap_ps_cmd_send(void *wmi_hdl,
					   uint8_t *peer_addr,
					   struct ap_ps_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_ap_ps_param_cmd)
		return wmi_handle->ops->send_set_ap_ps_param_cmd(wmi_handle,
				  peer_addr,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_sta_ps_cmd_send() - set sta powersave parameters
 * @wma_ctx: wma context
 * @peer_addr: peer mac address
 * @param: pointer to sta_ps parameter structure
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_sta_ps_cmd_send(void *wmi_hdl,
					   struct sta_ps_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_sta_ps_param_cmd)
		return wmi_handle->ops->send_set_sta_ps_param_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_crash_inject() - inject fw crash
 * @wma_handle: wma handle
 * @param: ponirt to crash inject paramter structure
 *
 * Return: 0 for success or return error
 */
CDF_STATUS wmi_crash_inject(void *wmi_hdl,
			 struct crash_inject *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_crash_inject_cmd)
		return wmi_handle->ops->send_crash_inject_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_dbglog_cmd_send() - set debug log level
 *  @param wmi_handle      : handle to WMI.
 *  @param param    : pointer to hold dbglog level parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS
wmi_unified_dbglog_cmd_send(void *wmi_hdl,
				struct dbglog_params *dbglog_param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_dbglog_cmd)
		return wmi_handle->ops->send_dbglog_cmd(wmi_handle,
				  dbglog_param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_vdev_set_param_send() - WMI vdev set parameter function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold vdev set parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_vdev_set_param_send(void *wmi_hdl,
				struct vdev_set_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_set_param_cmd)
		return wmi_handle->ops->send_vdev_set_param_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_stats_request_send() - WMI request stats function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold stats request parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_stats_request_send(void *wmi_hdl,
				uint8_t macaddr[IEEE80211_ADDR_LEN],
				struct stats_request_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_stats_request_cmd)
		return wmi_handle->ops->send_stats_request_cmd(wmi_handle,
				   macaddr, param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_stats_request_send() - WMI request stats function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold stats request parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_packet_log_enable_send(void *wmi_hdl,
				uint8_t macaddr[IEEE80211_ADDR_LEN],
				struct packet_enable_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_packet_log_enable_cmd)
		return wmi_handle->ops->send_packet_log_enable_cmd(wmi_handle,
				  macaddr, param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_beacon_send_cmd() - WMI beacon send function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold beacon send cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_beacon_send_cmd(void *wmi_hdl,
				struct beacon_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_beacon_send_cmd)
		return wmi_handle->ops->send_beacon_send_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_peer_assoc_send() - WMI peer assoc function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to peer assoc parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_peer_assoc_send(void *wmi_hdl,
				struct peer_assoc_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_peer_assoc_cmd)
		return wmi_handle->ops->send_peer_assoc_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_scan_start_cmd_send() - WMI scan start function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold scan start cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_scan_start_cmd_send(void *wmi_hdl,
				struct scan_start_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_scan_start_cmd)
		return wmi_handle->ops->send_scan_start_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_scan_stop_cmd_send() - WMI scan start function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold scan start cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_scan_stop_cmd_send(void *wmi_hdl,
				struct scan_stop_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_scan_stop_cmd)
		return wmi_handle->ops->send_scan_stop_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_unified_scan_chan_list_cmd_send() - WMI scan channel list function
 *  @param wmi_handle      : handle to WMI.
 *  @param macaddr        : MAC address
 *  @param param    : pointer to hold scan channel list parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_unified_scan_chan_list_cmd_send(void *wmi_hdl,
				struct scan_chan_list_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_scan_chan_list_cmd)
		return wmi_handle->ops->send_scan_chan_list_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 *  wmi_mgmt_unified_cmd_send() - management cmd over wmi layer
 *  @wmi_hdl      : handle to WMI.
 *  @param    : pointer to hold mgmt cmd parameter
 *
 *  Return: 0  on success and -ve on failure.
 */
CDF_STATUS wmi_mgmt_unified_cmd_send(void *wmi_hdl,
				struct wmi_mgmt_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_mgmt_cmd)
		return wmi_handle->ops->send_mgmt_cmd(wmi_handle,
				  param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_modem_power_state() - set modem power state to fw
 * @wmi_hdl: wmi handle
 * @param_value: parameter value
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_modem_power_state(void *wmi_hdl,
		uint32_t param_value)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_modem_power_state_cmd)
		return wmi_handle->ops->send_modem_power_state_cmd(wmi_handle,
				  param_value);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_sta_ps_mode() - set sta powersave params in fw
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 * @val: value
 *
 * Return: 0 for success or error code.
 */
CDF_STATUS wmi_unified_set_sta_ps_mode(void *wmi_hdl,
			       uint32_t vdev_id, uint8_t val)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_sta_ps_mode_cmd)
		return wmi_handle->ops->send_set_sta_ps_mode_cmd(wmi_handle,
				  vdev_id, val);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_set_mimops() - set MIMO powersave
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 * @value: value
 *
 * Return: CDF_STATUS_SUCCESS for success or error code.
 */
CDF_STATUS wmi_unified_set_mimops(void *wmi_hdl, uint8_t vdev_id, int value)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_mimops_cmd)
		return wmi_handle->ops->send_set_mimops_cmd(wmi_handle,
				  vdev_id, value);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_set_smps_params() - set smps params
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 * @value: value
 *
 * Return: CDF_STATUS_SUCCESS for success or error code.
 */
CDF_STATUS wmi_unified_set_smps_params(void *wmi_hdl, uint8_t vdev_id,
			       int value)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_smps_params_cmd)
		return wmi_handle->ops->send_set_smps_params_cmd(wmi_handle,
				  vdev_id, value);

	return CDF_STATUS_E_FAILURE;
}


/**
 * wmi_set_p2pgo_oppps_req() - send p2p go opp power save request to fw
 * @wmi_hdl: wmi handle
 * @opps: p2p opp power save parameters
 *
 * Return: none
 */
CDF_STATUS wmi_unified_set_p2pgo_oppps_req(void *wmi_hdl,
		struct p2p_ps_params *oppps)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_p2pgo_oppps_req_cmd)
		return wmi_handle->ops->send_set_p2pgo_oppps_req_cmd(wmi_handle,
				  oppps);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_p2pgo_noa_req_cmd() - send p2p go noa request to fw
 * @wmi_hdl: wmi handle
 * @noa: p2p power save parameters
 *
 * Return: none
 */
CDF_STATUS wmi_unified_set_p2pgo_noa_req_cmd(void *wmi_hdl,
			struct p2p_ps_params *noa)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_p2pgo_noa_req_cmd)
		return wmi_handle->ops->send_set_p2pgo_noa_req_cmd(wmi_handle,
				  noa);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_get_temperature() - get pdev temperature req
 * @wmi_hdl: wmi handle
 *
 * Return: CDF_STATUS_SUCCESS for success or error code.
 */
CDF_STATUS wmi_unified_get_temperature(void *wmi_hdl)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_get_temperature_cmd)
		return wmi_handle->ops->send_get_temperature_cmd(wmi_handle);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_sta_uapsd_auto_trig_cmd() - set uapsd auto trigger command
 * @wmi_hdl: wmi handle
 * @end_set_sta_ps_mode_cmd: cmd paramter strcture
 *
 * This function sets the trigger
 * uapsd params such as service interval, delay interval
 * and suspend interval which will be used by the firmware
 * to send trigger frames periodically when there is no
 * traffic on the transmit side.
 *
 * Return: 0 for success or error code.
 */
CDF_STATUS
wmi_unified_set_sta_uapsd_auto_trig_cmd(void *wmi_hdl,
				struct sta_uapsd_trig_params *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_sta_uapsd_auto_trig_cmd)
		return wmi_handle->ops->send_set_sta_uapsd_auto_trig_cmd(wmi_handle,
					param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_ocb_start_timing_advert() - start sending the timing advertisement
 *			   frames on a channel
 * @wmi_handle: pointer to the wmi handle
 * @timing_advert: pointer to the timing advertisement struct
 *
 * Return: 0 on succes
 */
CDF_STATUS wmi_unified_ocb_start_timing_advert(void *wmi_hdl,
	struct ocb_timing_advert_param *timing_advert)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_ocb_start_timing_advert_cmd)
		return wmi_handle->ops->send_ocb_start_timing_advert_cmd(wmi_handle,
				timing_advert);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_ocb_stop_timing_advert() - stop sending the timing advertisement
 *			frames on a channel
 * @wmi_handle: pointer to the wmi handle
 * @timing_advert: pointer to the timing advertisement struct
 *
 * Return: 0 on succes
 */
CDF_STATUS wmi_unified_ocb_stop_timing_advert(void *wmi_hdl,
	struct ocb_timing_advert_param *timing_advert)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_ocb_stop_timing_advert_cmd)
		return wmi_handle->ops->send_ocb_stop_timing_advert_cmd(wmi_handle,
					timing_advert);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_ocb_set_utc_time_cmd() - get ocb tsf timer val
 * @wmi_handle: pointer to the wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 on succes
 */
CDF_STATUS wmi_unified_ocb_set_utc_time_cmd(void *wmi_hdl,
			struct ocb_utc_param *utc)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_ocb_set_utc_time_cmd)
		return wmi_handle->ops->send_ocb_set_utc_time_cmd(wmi_handle,
				utc);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_ocb_get_tsf_timer() - get ocb tsf timer val
 * @wmi_handle: pointer to the wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 on succes
 */
CDF_STATUS wmi_unified_ocb_get_tsf_timer(void *wmi_hdl,
			uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_ocb_get_tsf_timer_cmd)
		return wmi_handle->ops->send_ocb_get_tsf_timer_cmd(wmi_handle,
					vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_dcc_get_stats_cmd() - get the DCC channel stats
 * @wmi_handle: pointer to the wmi handle
 * @get_stats_param: pointer to the dcc stats
 *
 * Return: 0 on succes
 */
CDF_STATUS wmi_unified_dcc_get_stats_cmd(void *wmi_hdl,
			struct dcc_get_stats_param *get_stats_param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_dcc_get_stats_cmd)
		return wmi_handle->ops->send_dcc_get_stats_cmd(wmi_handle,
					get_stats_param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_dcc_clear_stats() - command to clear the DCC stats
 * @wmi_handle: pointer to the wmi handle
 * @clear_stats_param: parameters to the command
 *
 * Return: 0 on succes
 */
CDF_STATUS wmi_unified_dcc_clear_stats(void *wmi_hdl,
			uint32_t vdev_id, uint32_t dcc_stats_bitmap)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_dcc_clear_stats_cmd)
		return wmi_handle->ops->send_dcc_clear_stats_cmd(wmi_handle,
					vdev_id, dcc_stats_bitmap);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_dcc_update_ndl() - command to update the NDL data
 * @wmi_handle: pointer to the wmi handle
 * @update_ndl_param: pointer to the request parameters
 *
 * Return: 0 on success
 */
CDF_STATUS wmi_unified_dcc_update_ndl(void *wmi_hdl,
			struct dcc_update_ndl_param *update_ndl_param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_dcc_update_ndl_cmd)
		return wmi_handle->ops->send_dcc_update_ndl_cmd(wmi_handle,
					update_ndl_param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_ocb_set_config() - send the OCB config to the FW
 * @wmi_handle: pointer to the wmi handle
 * @config: the OCB configuration
 *
 * Return: 0 on success
 */
CDF_STATUS wmi_unified_ocb_set_config(void *wmi_hdl,
			struct ocb_config_param *config, uint32_t *ch_mhz)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_ocb_set_config_cmd)
		return wmi_handle->ops->send_ocb_set_config_cmd(wmi_handle,
					config, ch_mhz);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_enable_disable_mcc_adaptive_scheduler_cmd() - control mcc scheduler
 * @wmi_handle: wmi handle
 * @mcc_adaptive_scheduler: enable/disable
 *
 * This function enable/disable mcc adaptive scheduler in fw.
 *
 * Return: CDF_STATUS_SUCCESS for sucess or error code
 */
CDF_STATUS wmi_unified_set_enable_disable_mcc_adaptive_scheduler_cmd(
		void *wmi_hdl, uint32_t mcc_adaptive_scheduler)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_enable_disable_mcc_adaptive_scheduler_cmd)
		return wmi_handle->ops->send_set_enable_disable_mcc_adaptive_scheduler_cmd(wmi_handle,
					mcc_adaptive_scheduler);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_mcc_channel_time_latency_cmd() - set MCC channel time latency
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
CDF_STATUS wmi_unified_set_mcc_channel_time_latency_cmd(void *wmi_hdl,
	uint32_t mcc_channel_freq, uint32_t mcc_channel_time_latency)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_mcc_channel_time_latency_cmd)
		return wmi_handle->ops->send_set_mcc_channel_time_latency_cmd(wmi_handle,
					mcc_channel_freq,
					mcc_channel_time_latency);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_mcc_channel_time_quota_cmd() - set MCC channel time quota
 * @wmi: wmi handle
 * @adapter_1_chan_number: adapter 1 channel number
 * @adapter_1_quota: adapter 1 quota
 * @adapter_2_chan_number: adapter 2 channel number
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_set_mcc_channel_time_quota_cmd(void *wmi_hdl,
			 uint32_t adapter_1_chan_freq,
			 uint32_t adapter_1_quota, uint32_t adapter_2_chan_freq)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_mcc_channel_time_quota_cmd)
		return wmi_handle->ops->send_set_mcc_channel_time_quota_cmd(wmi_handle,
						adapter_1_chan_freq,
						adapter_1_quota,
						adapter_2_chan_freq);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_thermal_mgmt_cmd() - set thermal mgmt command to fw
 * @wmi_handle: Pointer to wmi handle
 * @thermal_info: Thermal command information
 *
 * This function sends the thermal management command
 * to the firmware
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 */
CDF_STATUS wmi_unified_set_thermal_mgmt_cmd(void *wmi_hdl,
				struct thermal_cmd_params *thermal_info)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_thermal_mgmt_cmd)
		return wmi_handle->ops->send_set_thermal_mgmt_cmd(wmi_handle,
					thermal_info);

	return CDF_STATUS_E_FAILURE;
}


/**
 * wmi_unified_lro_config_cmd() - process the LRO config command
 * @wmi: Pointer to wmi handle
 * @wmi_lro_cmd: Pointer to LRO configuration parameters
 *
 * This function sends down the LRO configuration parameters to
 * the firmware to enable LRO, sets the TCP flags and sets the
 * seed values for the toeplitz hash generation
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 */
CDF_STATUS wmi_unified_lro_config_cmd(void *wmi_hdl,
	 struct wmi_lro_config_cmd_t *wmi_lro_cmd)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_lro_config_cmd)
		return wmi_handle->ops->send_lro_config_cmd(wmi_handle,
					wmi_lro_cmd);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_bcn_buf_ll_cmd() - prepare and send beacon buffer to fw for LL
 * @wmi_hdl: wmi handle
 * @param: bcn ll cmd parameter
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 */
CDF_STATUS wmi_unified_bcn_buf_ll_cmd(void *wmi_hdl,
			wmi_bcn_send_from_host_cmd_fixed_param *param)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_bcn_buf_ll_cmd)
		return wmi_handle->ops->send_bcn_buf_ll_cmd(wmi_handle,
						param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_sta_sa_query_param_cmd() - set sta sa query parameters
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 * @max_retries: max retries
 * @retry_interval: retry interval
 * This function sets sta query related parameters in fw.
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 */

CDF_STATUS wmi_unified_set_sta_sa_query_param_cmd(void *wmi_hdl,
					uint8_t vdev_id, uint32_t max_retries,
					uint32_t retry_interval)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_sta_sa_query_param_cmd)
		return wmi_handle->ops->send_set_sta_sa_query_param_cmd(wmi_handle,
						vdev_id, max_retries,
						retry_interval);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_sta_keep_alive_cmd() - set sta keep alive parameters
 * @wmi_hdl: wmi handle
 * @params: sta keep alive parameter
 *
 * This function sets keep alive related parameters in fw.
 *
 * Return: none
 */
CDF_STATUS wmi_unified_set_sta_keep_alive_cmd(void *wmi_hdl,
				struct sta_params *params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_sta_keep_alive_cmd)
		return wmi_handle->ops->send_set_sta_keep_alive_cmd(wmi_handle,
						params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_vdev_set_gtx_cfg_cmd() - set GTX params
 * @wmi_hdl: wmi handle
 * @if_id: vdev id
 * @gtx_info: GTX config params
 *
 * This function set GTX related params in firmware.
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_vdev_set_gtx_cfg_cmd(void *wmi_hdl, uint32_t if_id,
			struct wmi_gtx_config *gtx_info)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_vdev_set_gtx_cfg_cmd)
		return wmi_handle->ops->send_vdev_set_gtx_cfg_cmd(wmi_handle,
					if_id, gtx_info);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_update_edca_param() - update EDCA params
 * @wmi_hdl: wmi handle
 * @edca_params: edca parameters
 *
 * This function updates EDCA parameters to the target
 *
 * Return: CDF Status
 */
CDF_STATUS wmi_unified_process_update_edca_param(void *wmi_hdl,
				uint8_t vdev_id,
				wmi_wmm_vparams gwmm_param[WMI_MAX_NUM_AC])
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_update_edca_param_cmd)
		return wmi_handle->ops->send_process_update_edca_param_cmd(wmi_handle,
					 vdev_id, gwmm_param);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_probe_rsp_tmpl_send_cmd() - send probe response template to fw
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 * @probe_rsp_info: probe response info
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_probe_rsp_tmpl_send_cmd(void *wmi_hdl,
				uint8_t vdev_id,
				struct wmi_probe_resp_params *probe_rsp_info,
				uint8_t *frm)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_probe_rsp_tmpl_send_cmd)
		return wmi_handle->ops->send_probe_rsp_tmpl_send_cmd(wmi_handle,
						 vdev_id, probe_rsp_info,
						 frm);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_p2p_go_set_beacon_ie_cmd() - set beacon IE for p2p go
 * @wma_handle: wma handle
 * @vdev_id: vdev id
 * @p2p_ie: p2p IE
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_p2p_go_set_beacon_ie_cmd(void *wmi_hdl,
				    A_UINT32 vdev_id, uint8_t *p2p_ie)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_p2p_go_set_beacon_ie_cmd)
		return wmi_handle->ops->send_p2p_go_set_beacon_ie_cmd(wmi_handle,
						 vdev_id, p2p_ie);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_gateway_params_cmd() - set gateway parameters
 * @wmi_hdl: wmi handle
 * @req: gateway parameter update request structure
 *
 * This function reads the incoming @req and fill in the destination
 * WMI structure and sends down the gateway configs down to the firmware
 *
 * Return: CDF_STATUS
 */
CDF_STATUS wmi_unified_set_gateway_params_cmd(void *wmi_hdl,
					struct gateway_update_req_param *req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_gateway_params_cmd)
		return wmi_handle->ops->send_set_gateway_params_cmd(wmi_handle,
				  req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_rssi_monitoring_cmd() - set rssi monitoring
 * @wmi_hdl: wmi handle
 * @req: rssi monitoring request structure
 *
 * This function reads the incoming @req and fill in the destination
 * WMI structure and send down the rssi monitoring configs down to the firmware
 *
 * Return: 0 on success; error number otherwise
 */
CDF_STATUS wmi_unified_set_rssi_monitoring_cmd(void *wmi_hdl,
					struct rssi_monitor_param *req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_pno_start_cmd)
		return wmi_handle->ops->send_set_rssi_monitoring_cmd(wmi_handle,
			    req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_scan_probe_setoui_cmd() - set scan probe OUI
 * @wmi_hdl: wmi handle
 * @psetoui: OUI parameters
 *
 * set scan probe OUI parameters in firmware
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_scan_probe_setoui_cmd(void *wmi_hdl,
			  struct scan_mac_oui *psetoui)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_scan_probe_setoui_cmd)
		return wmi_handle->ops->send_scan_probe_setoui_cmd(wmi_handle,
			    psetoui);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_reset_passpoint_network_list_cmd() - reset passpoint network list
 * @wmi_hdl: wmi handle
 * @req: passpoint network request structure
 *
 * This function sends down WMI command with network id set to wildcard id.
 * firmware shall clear all the config entries
 *
 * Return: CDF_STATUS enumeration
 */
CDF_STATUS wmi_unified_reset_passpoint_network_list_cmd(void *wmi_hdl,
					struct wifi_passpoint_req_param *req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_reset_passpoint_network_list_cmd)
		return wmi_handle->ops->send_reset_passpoint_network_list_cmd(wmi_handle,
			    req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_passpoint_network_list_cmd() - set passpoint network list
 * @wmi_hdl: wmi handle
 * @req: passpoint network request structure
 *
 * This function reads the incoming @req and fill in the destination
 * WMI structure and send down the passpoint configs down to the firmware
 *
 * Return: CDF_STATUS enumeration
 */
CDF_STATUS wmi_unified_set_passpoint_network_list_cmd(void *wmi_hdl,
					struct wifi_passpoint_req_param *req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_passpoint_network_list_cmd)
		return wmi_handle->ops->send_set_passpoint_network_list_cmd(wmi_handle,
			    req);

	return CDF_STATUS_E_FAILURE;
}

/** wmi_unified_set_epno_network_list_cmd() - set epno network list
 * @wmi_hdl: wmi handle
 * @req: epno config params request structure
 *
 * This function reads the incoming epno config request structure
 * and constructs the WMI message to the firmware.
 *
 * Returns: 0 on success, error number otherwise
 */
CDF_STATUS wmi_unified_set_epno_network_list_cmd(void *wmi_hdl,
		struct wifi_enhanched_pno_params *req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_epno_network_list_cmd)
		return wmi_handle->ops->send_set_epno_network_list_cmd(wmi_handle,
			    req);

	return CDF_STATUS_E_FAILURE;
}

/** wmi_unified_ipa_offload_control_cmd() - ipa offload control parameter
 * @wmi_hdl: wmi handle
 * @ipa_offload: ipa offload control parameter
 *
 * Returns: 0 on success, error number otherwise
 */
CDF_STATUS  wmi_unified_ipa_offload_control_cmd(void *wmi_hdl,
		struct ipa_offload_control_params *ipa_offload)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_ipa_offload_control_cmd)
		return wmi_handle->ops->send_ipa_offload_control_cmd(wmi_handle,
			    ipa_offload);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_extscan_get_capabilities_cmd() - extscan get capabilities
 * @wmi_hdl: wmi handle
 * @pgetcapab: get capabilities params
 *
 * This function send request to fw to get extscan capabilities.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_extscan_get_capabilities_cmd(void *wmi_hdl,
			  struct extscan_capabilities_params *pgetcapab)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_extscan_get_capabilities_cmd)
		return wmi_handle->ops->send_extscan_get_capabilities_cmd(wmi_handle,
			    pgetcapab);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_extscan_get_cached_results_cmd() - extscan get cached results
 * @wmi_hdl: wmi handle
 * @pcached_results: cached results parameters
 *
 * This function send request to fw to get cached results.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_extscan_get_cached_results_cmd(void *wmi_hdl,
			  struct extscan_cached_result_params *pcached_results)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_extscan_get_cached_results_cmd)
		return wmi_handle->ops->send_extscan_get_cached_results_cmd(wmi_handle,
			    pcached_results);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_extscan_stop_change_monitor_cmd() - send stop change monitor cmd
 * @wmi_hdl: wmi handle
 * @reset_req: Reset change request params
 *
 * This function sends stop change monitor request to fw.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_extscan_stop_change_monitor_cmd(void *wmi_hdl,
			  struct extscan_capabilities_reset_params *reset_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_extscan_stop_change_monitor_cmd)
		return wmi_handle->ops->send_extscan_stop_change_monitor_cmd(wmi_handle,
			    reset_req);

	return CDF_STATUS_E_FAILURE;
}



/**
 * wmi_unified_extscan_start_change_monitor_cmd() - start change monitor cmd
 * @wmi_hdl: wmi handle
 * @psigchange: change monitor request params
 *
 * This function sends start change monitor request to fw.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_extscan_start_change_monitor_cmd(void *wmi_hdl,
				   struct extscan_set_sig_changereq_params *
				   psigchange)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_extscan_start_change_monitor_cmd)
		return wmi_handle->ops->send_extscan_start_change_monitor_cmd(wmi_handle,
			    psigchange);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_extscan_stop_hotlist_monitor_cmd() - stop hotlist monitor
 * @wmi_hdl: wmi handle
 * @photlist_reset: hotlist reset params
 *
 * This function configures hotlist monitor to stop in fw.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_extscan_stop_hotlist_monitor_cmd(void *wmi_hdl,
		  struct extscan_bssid_hotlist_reset_params *photlist_reset)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_extscan_stop_hotlist_monitor_cmd)
		return wmi_handle->ops->send_extscan_stop_hotlist_monitor_cmd(wmi_handle,
			    photlist_reset);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_stop_extscan_cmd() - stop extscan command to fw.
 * @wmi_hdl: wmi handle
 * @pstopcmd: stop scan command request params
 *
 * This function sends stop extscan request to fw.
 *
 * Return: CDF Status.
 */
CDF_STATUS wmi_unified_stop_extscan_cmd(void *wmi_hdl,
			  struct extscan_stop_req_params *pstopcmd)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_stop_extscan_cmd)
		return wmi_handle->ops->send_stop_extscan_cmd(wmi_handle,
			    pstopcmd);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_start_extscan_cmd() - start extscan command to fw.
 * @wmi_hdl: wmi handle
 * @pstart: scan command request params
 *
 * This function sends start extscan request to fw.
 *
 * Return: CDF Status.
 */
CDF_STATUS wmi_unified_start_extscan_cmd(void *wmi_hdl,
			  struct wifi_scan_cmd_req_params *pstart)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_start_extscan_cmd)
		return wmi_handle->ops->send_start_extscan_cmd(wmi_handle,
			    pstart);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_plm_stop_cmd() - plm stop request
 * @wmi_hdl: wmi handle
 * @plm: plm request parameters
 *
 * This function request FW to stop PLM.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_plm_stop_cmd(void *wmi_hdl,
			  const struct plm_req_params *plm)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_plm_stop_cmd)
		return wmi_handle->ops->send_plm_stop_cmd(wmi_handle,
			    plm);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_plm_start_cmd() - plm start request
 * @wmi_hdl: wmi handle
 * @plm: plm request parameters
 *
 * This function request FW to start PLM.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_plm_start_cmd(void *wmi_hdl,
			  const struct plm_req_params *plm,
			  uint32_t *gchannel_list)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_plm_start_cmd)
		return wmi_handle->ops->send_plm_start_cmd(wmi_handle,
			    plm, gchannel_list);

	return CDF_STATUS_E_FAILURE;
}

/**
 * send_pno_stop_cmd() - PNO stop request
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 *
 * This function request FW to stop ongoing PNO operation.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_pno_stop_cmd(void *wmi_hdl, uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_pno_stop_cmd)
		return wmi_handle->ops->send_pno_stop_cmd(wmi_handle,
			    vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_pno_start_cmd() - PNO start request
 * @wmi_hdl: wmi handle
 * @pno: PNO request
 * @gchannel_freq_list: channel frequency list
 *
 * This function request FW to start PNO request.
 * Request: CDF status
 */
CDF_STATUS wmi_unified_pno_start_cmd(void *wmi_hdl,
		   struct pno_scan_req_params *pno,
		   uint32_t *gchannel_freq_list)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_pno_start_cmd)
		return wmi_handle->ops->send_pno_start_cmd(wmi_handle,
			    pno, gchannel_freq_list);

	return CDF_STATUS_E_FAILURE;
}

/* wmi_unified_set_ric_req_cmd() - set ric request element
 * @wmi_hdl: wmi handle
 * @msg: message
 * @is_add_ts: is addts required
 *
 * This function sets ric request element for 11r roaming.
 *
 * Return: none
 */
CDF_STATUS wmi_unified_set_ric_req_cmd(void *wmi_hdl, void *msg,
		uint8_t is_add_ts)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_ric_req_cmd)
		return wmi_handle->ops->send_set_ric_req_cmd(wmi_handle, msg,
			    is_add_ts);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_ll_stats_clear_cmd() - clear link layer stats
 * @wmi_hdl: wmi handle
 * @clear_req: ll stats clear request command params
 * @addr: mac address
 *
 * Return: CDF_STATUS_SUCCESS for success or error code
 */
CDF_STATUS wmi_unified_process_ll_stats_clear_cmd(void *wmi_hdl,
	 const struct ll_stats_clear_params *clear_req,
	 uint8_t addr[IEEE80211_ADDR_LEN])
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_ll_stats_clear_cmd)
		return wmi_handle->ops->send_process_ll_stats_clear_cmd(wmi_handle,
			   clear_req,  addr);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_ll_stats_get_cmd() - link layer stats get request
 * @wmi_hdl:wmi handle
 * @get_req:ll stats get request command params
 *
 * Return: CDF_STATUS_SUCCESS for success or error code
 */
CDF_STATUS wmi_unified_process_ll_stats_get_cmd(void *wmi_hdl,
		 const struct ll_stats_get_params  *get_req,
		 uint8_t addr[IEEE80211_ADDR_LEN])
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_ll_stats_get_cmd)
		return wmi_handle->ops->send_process_ll_stats_get_cmd(wmi_handle,
			   get_req,  addr);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_get_stats_cmd() - get stats request
 * @wmi_hdl: wma handle
 * @get_stats_param: stats params
 * @addr: mac address
 *
 * Return: none
 */
CDF_STATUS wmi_unified_get_stats_cmd(void *wmi_hdl,
		       struct pe_stats_req  *get_stats_param,
			   uint8_t addr[IEEE80211_ADDR_LEN])
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_get_stats_cmd)
		return wmi_handle->ops->send_get_stats_cmd(wmi_handle,
			   get_stats_param,  addr);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_ll_stats_set_cmd() - link layer stats set request
 * @wmi_handle:       wmi handle
 * @set_req:  ll stats set request command params
 *
 * Return: CDF_STATUS_SUCCESS for success or error code
 */
CDF_STATUS wmi_unified_process_ll_stats_set_cmd(void *wmi_hdl,
		const struct ll_stats_set_params *set_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_ll_stats_set_cmd)
		return wmi_handle->ops->send_process_ll_stats_set_cmd(wmi_handle,
			   set_req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_snr_request_cmd() - send request to fw to get RSSI stats
 * @wmi_handle: wmi handle
 * @rssi_req: get RSSI request
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_snr_request_cmd(void *wmi_hdl)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_snr_request_cmd)
		return wmi_handle->ops->send_snr_request_cmd(wmi_handle);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_snr_cmd() - get RSSI from fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_snr_cmd(void *wmi_hdl, uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_snr_cmd)
		return wmi_handle->ops->send_snr_cmd(wmi_handle,
			    vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_link_status_req_cmd() - process link status request from UMAC
 * @wmi_handle: wmi handle
 * @link_status: get link params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_link_status_req_cmd(void *wmi_hdl,
				 struct link_status_params *link_status)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_link_status_req_cmd)
		return wmi_handle->ops->send_link_status_req_cmd(wmi_handle,
			    link_status);

	return CDF_STATUS_E_FAILURE;
}

#ifdef FEATURE_WLAN_LPHB

/**
 * wmi_unified_lphb_config_hbenable_cmd() - enable command of LPHB configuration requests
 * @wmi_handle: wmi handle
 * @lphb_conf_req: configuration info
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_lphb_config_hbenable_cmd(void *wmi_hdl,
				wmi_hb_set_enable_cmd_fixed_param *params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_lphb_config_hbenable_cmd)
		return wmi_handle->ops->send_lphb_config_hbenable_cmd(wmi_handle,
			    params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_lphb_config_tcp_params_cmd() - set tcp params of LPHB configuration requests
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_lphb_config_tcp_params_cmd(void *wmi_hdl,
				    wmi_hb_set_tcp_params_cmd_fixed_param *lphb_conf_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_lphb_config_tcp_params_cmd)
		return wmi_handle->ops->send_lphb_config_tcp_params_cmd(wmi_handle,
			    lphb_conf_req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_lphb_config_tcp_pkt_filter_cmd() - configure tcp packet filter command of LPHB
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_lphb_config_tcp_pkt_filter_cmd(void *wmi_hdl,
					wmi_hb_set_tcp_pkt_filter_cmd_fixed_param *g_hb_tcp_filter_fp)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_lphb_config_tcp_pkt_filter_cmd)
		return wmi_handle->ops->send_lphb_config_tcp_pkt_filter_cmd(wmi_handle,
			    g_hb_tcp_filter_fp);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_lphb_config_udp_params_cmd() - configure udp param command of LPHB
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_lphb_config_udp_params_cmd(void *wmi_hdl,
				    wmi_hb_set_udp_params_cmd_fixed_param *lphb_conf_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_lphb_config_udp_params_cmd)
		return wmi_handle->ops->send_lphb_config_udp_params_cmd(wmi_handle,
			    lphb_conf_req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_lphb_config_udp_pkt_filter_cmd() - configure udp pkt filter command of LPHB
 * @wmi_handle: wmi handle
 * @lphb_conf_req: lphb config request
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_lphb_config_udp_pkt_filter_cmd(void *wmi_hdl,
					wmi_hb_set_udp_pkt_filter_cmd_fixed_param *lphb_conf_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_lphb_config_udp_pkt_filter_cmd)
		return wmi_handle->ops->send_lphb_config_udp_pkt_filter_cmd(wmi_handle,
			    lphb_conf_req);

	return CDF_STATUS_E_FAILURE;
}
#endif /* FEATURE_WLAN_LPHB */

/**
 * wmi_unified_process_dhcp_ind() - process dhcp indication from SME
 * @wmi_handle: wmi handle
 * @ta_dhcp_ind: DHCP indication parameter
 *
 * Return: CDF Status
 */
CDF_STATUS wmi_unified_process_dhcp_ind(void *wmi_hdl,
				wmi_peer_set_param_cmd_fixed_param *ta_dhcp_ind)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_dhcp_ind_cmd)
		return wmi_handle->ops->send_process_dhcp_ind_cmd(wmi_handle,
			    ta_dhcp_ind);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_get_link_speed_cmd() -send command to get linkspeed
 * @wmi_handle: wmi handle
 * @pLinkSpeed: link speed info
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_get_link_speed_cmd(void *wmi_hdl,
			wmi_mac_addr peer_macaddr)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_get_link_speed_cmd)
		return wmi_handle->ops->send_get_link_speed_cmd(wmi_handle,
			    peer_macaddr);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_egap_conf_params_cmd() - send wmi cmd of egap configuration params
 * @wmi_handle:	 wmi handler
 * @egap_params: pointer to egap_params
 *
 * Return:	 0 for success, otherwise appropriate error code
 */
CDF_STATUS wmi_unified_egap_conf_params_cmd(void *wmi_hdl,
				     wmi_ap_ps_egap_param_cmd_fixed_param *egap_params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_egap_conf_params_cmd)
		return wmi_handle->ops->send_egap_conf_params_cmd(wmi_handle,
			    egap_params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_fw_profiling_data_cmd() - send FW profiling cmd to WLAN FW
 * @wmi_handl: wmi handle
 * @cmd: Profiling command index
 * @value1: parameter1 value
 * @value2: parameter2 value
 *
 * Return: 0 for success else error code
 */
CDF_STATUS wmi_unified_fw_profiling_data_cmd(void *wmi_hdl,
			uint32_t cmd, uint32_t value1, uint32_t value2)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_fw_profiling_cmd)
		return wmi_handle->ops->send_fw_profiling_cmd(wmi_handle,
			    cmd, value1, value2);

	return CDF_STATUS_E_FAILURE;
}

#ifdef FEATURE_WLAN_RA_FILTERING
/**
 * wmi_unified_wow_sta_ra_filter_cmd() - set RA filter pattern in fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_wow_sta_ra_filter_cmd(void *wmi_hdl,
				uint8_t vdev_id, uint8_t default_pattern,
				uint16_t rate_limit_interval)
{

	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_wow_sta_ra_filter_cmd)
		return wmi_handle->ops->send_wow_sta_ra_filter_cmd(wmi_handle,
			    vdev_id, default_pattern, default_pattern);

	return CDF_STATUS_E_FAILURE;

}
#endif /* FEATURE_WLAN_RA_FILTERING */

/**
 * wmi_unified_nat_keepalive_en_cmd() - enable NAT keepalive filter
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_nat_keepalive_en_cmd(void *wmi_hdl, uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_nat_keepalive_en_cmd)
		return wmi_handle->ops->send_nat_keepalive_en_cmd(wmi_handle,
			    vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_csa_offload_enable() - send CSA offload enable command
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_csa_offload_enable(void *wmi_hdl, uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_csa_offload_enable_cmd)
		return wmi_handle->ops->send_csa_offload_enable_cmd(wmi_handle,
			    vdev_id);

	return CDF_STATUS_E_FAILURE;
}
/**
 * wmi_unified_start_oem_data_cmd() - start OEM data request to target
 * @wmi_handle: wmi handle
 * @startOemDataReq: start request params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_start_oem_data_cmd(void *wmi_hdl,
			  uint8_t data_len,
			  uint8_t *data)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_start_oem_data_cmd)
		return wmi_handle->ops->send_start_oem_data_cmd(wmi_handle,
			    data_len, data);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_dfs_phyerr_filter_offload_en_cmd() - enable dfs phyerr filter
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
CDF_STATUS
wmi_unified_dfs_phyerr_filter_offload_en_cmd(void *wmi_hdl,
			bool dfs_phyerr_filter_offload)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_dfs_phyerr_filter_offload_en_cmd)
		return wmi_handle->ops->send_dfs_phyerr_filter_offload_en_cmd(wmi_handle,
			    dfs_phyerr_filter_offload);

	return CDF_STATUS_E_FAILURE;
}

#if !defined(REMOVE_PKT_LOG)
/**
 * wmi_unified_pktlog_wmi_send_cmd() - send pktlog enable/disable command to target
 * @wmi_handle: wmi handle
 * @pktlog_event: pktlog event
 * @cmd_id: pktlog cmd id
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_pktlog_wmi_send_cmd(void *wmi_hdl,
				   WMI_PKTLOG_EVENT pktlog_event,
				   WMI_CMD_ID cmd_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_pktlog_wmi_send_cmd)
		return wmi_handle->ops->send_pktlog_wmi_send_cmd(wmi_handle,
			    pktlog_event, pktlog_event);

	return CDF_STATUS_E_FAILURE;
}
#endif /* REMOVE_PKT_LOG */

/**
 * wmi_unified_add_wow_wakeup_event_cmd() -  Configures wow wakeup events.
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @bitmap: Event bitmap
 * @enable: enable/disable
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_add_wow_wakeup_event_cmd(void *wmi_hdl,
					uint32_t vdev_id,
					uint32_t bitmap,
					bool enable)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_add_wow_wakeup_event_cmd)
		return wmi_handle->ops->send_add_wow_wakeup_event_cmd(wmi_handle,
			    vdev_id, vdev_id, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_wow_patterns_to_fw_cmd() - Sends WOW patterns to FW.
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
CDF_STATUS wmi_unified_wow_patterns_to_fw_cmd(void *wmi_hdl,
				uint8_t vdev_id, uint8_t ptrn_id,
				const uint8_t *ptrn, uint8_t ptrn_len,
				uint8_t ptrn_offset, const uint8_t *mask,
				uint8_t mask_len, bool user,
				uint8_t default_patterns)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_wow_patterns_to_fw_cmd)
		return wmi_handle->ops->send_wow_patterns_to_fw_cmd(wmi_handle,
			    vdev_id, ptrn_id, ptrn,
				ptrn_len, ptrn_offset, mask,
				mask_len, user, default_patterns);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_wow_delete_pattern_cmd() - delete wow pattern in target
 * @wmi_handle: wmi handle
 * @ptrn_id: pattern id
 * @vdev_id: vdev id
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_wow_delete_pattern_cmd(void *wmi_hdl, uint8_t ptrn_id,
					uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_wow_delete_pattern_cmd)
		return wmi_handle->ops->send_wow_delete_pattern_cmd(wmi_handle,
			    ptrn_id, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_host_wakeup_ind_to_fw_cmd() - send wakeup ind to fw
 * @wmi_handle: wmi handle
 *
 * Sends host wakeup indication to FW. On receiving this indication,
 * FW will come out of WOW.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_host_wakeup_ind_to_fw_cmd(void *wmi_hdl)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_host_wakeup_ind_to_fw_cmd)
		return wmi_handle->ops->send_host_wakeup_ind_to_fw_cmd(wmi_handle);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_del_ts_cmd() - send DELTS request to fw
 * @wmi_handle: wmi handle
 * @msg: delts params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_del_ts_cmd(void *wmi_hdl, uint8_t vdev_id,
				uint8_t ac)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_del_ts_cmd)
		return wmi_handle->ops->send_del_ts_cmd(wmi_handle,
			    vdev_id, ac);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_aggr_qos_cmd() - send aggr qos request to fw
 * @wmi_handle: handle to wmi
 * @aggr_qos_rsp_msg - combined struct for all ADD_TS requests.
 *
 * A function to handle WMA_AGGR_QOS_REQ. This will send out
 * ADD_TS requestes to firmware in loop for all the ACs with
 * active flow.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_aggr_qos_cmd(void *wmi_hdl,
		      struct aggr_add_ts_param *aggr_qos_rsp_msg)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_aggr_qos_cmd)
		return wmi_handle->ops->send_aggr_qos_cmd(wmi_handle,
			    aggr_qos_rsp_msg);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_add_ts_cmd() - send ADDTS request to fw
 * @wmi_handle: wmi handle
 * @msg: ADDTS params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_add_ts_cmd(void *wmi_hdl,
		 struct add_ts_param *msg)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_add_ts_cmd)
		return wmi_handle->ops->send_add_ts_cmd(wmi_handle,
			    msg);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_enable_disable_packet_filter_cmd() - enable/disable packet filter in target
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @enable: Flag to enable/disable packet filter
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_enable_disable_packet_filter_cmd(void *wmi_hdl,
					uint8_t vdev_id, bool enable)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_enable_disable_packet_filter_cmd)
		return wmi_handle->ops->send_enable_disable_packet_filter_cmd(wmi_handle,
			    vdev_id, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_config_packet_filter_cmd() - configure packet filter in target
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @rcv_filter_param: Packet filter parameters
 * @filter_id: Filter id
 * @enable: Flag to add/delete packet filter configuration
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_config_packet_filter_cmd(void *wmi_hdl,
		uint8_t vdev_id, struct rcv_pkt_filter_config *rcv_filter_param,
		uint8_t filter_id, bool enable)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_config_packet_filter_cmd)
		return wmi_handle->ops->send_config_packet_filter_cmd(wmi_handle,
			    vdev_id, rcv_filter_param,
				filter_id, enable);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_add_clear_mcbc_filter_cmd() - set mcast filter command to fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @multicastAddr: mcast address
 * @clearList: clear list flag
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_add_clear_mcbc_filter_cmd(void *wmi_hdl,
				     uint8_t vdev_id,
				     struct cdf_mac_addr multicast_addr,
				     bool clearList)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_add_clear_mcbc_filter_cmd)
		return wmi_handle->ops->send_add_clear_mcbc_filter_cmd(wmi_handle,
			    vdev_id, multicast_addr, clearList);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_send_gtk_offload_cmd() - send GTK offload command to fw
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @params: GTK offload parameters
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_send_gtk_offload_cmd(void *wmi_hdl, uint8_t vdev_id,
					   struct gtk_offload_params *params,
					   bool enable_offload,
					   uint32_t gtk_offload_opcode)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_gtk_offload_cmd)
		return wmi_handle->ops->send_gtk_offload_cmd(wmi_handle,
			    vdev_id, params,
				enable_offload, gtk_offload_opcode);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_gtk_offload_getinfo_cmd() - send GTK offload cmd to fw
 * @wmi_handle: wmi handle
 * @params: GTK offload params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_process_gtk_offload_getinfo_cmd(void *wmi_hdl,
				uint8_t vdev_id,
				uint64_t offload_req_opcode)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_gtk_offload_getinfo_cmd)
		return wmi_handle->ops->send_process_gtk_offload_getinfo_cmd(wmi_handle,
			    vdev_id,
				offload_req_opcode);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_add_periodic_tx_ptrn_cmd - add periodic tx ptrn
 * @wmi_handle: wmi handle
 * @pAddPeriodicTxPtrnParams: tx ptrn params
 *
 * Retrun: CDF status
 */
CDF_STATUS wmi_unified_process_add_periodic_tx_ptrn_cmd(void *wmi_hdl,
						struct periodic_tx_pattern  *
						pAddPeriodicTxPtrnParams,
						uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_add_periodic_tx_ptrn_cmd)
		return wmi_handle->ops->send_process_add_periodic_tx_ptrn_cmd(wmi_handle,
			    pAddPeriodicTxPtrnParams,
				vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_del_periodic_tx_ptrn_cmd - del periodic tx ptrn
 * @wmi_handle: wmi handle
 * @vdev_id: vdev id
 * @pattern_id: pattern id
 *
 * Retrun: CDF status
 */
CDF_STATUS wmi_unified_process_del_periodic_tx_ptrn_cmd(void *wmi_hdl,
						uint8_t vdev_id,
						uint8_t pattern_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_del_periodic_tx_ptrn_cmd)
		return wmi_handle->ops->send_process_del_periodic_tx_ptrn_cmd(wmi_handle,
			    vdev_id,
				pattern_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_stats_ext_req_cmd() - request ext stats from fw
 * @wmi_handle: wmi handle
 * @preq: stats ext params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_stats_ext_req_cmd(void *wmi_hdl,
			struct stats_ext_params *preq)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_stats_ext_req_cmd)
		return wmi_handle->ops->send_stats_ext_req_cmd(wmi_handle,
			    preq);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_enable_ext_wow_cmd() - enable ext wow in fw
 * @wmi_handle: wmi handle
 * @params: ext wow params
 *
 * Return:0 for success or error code
 */
CDF_STATUS wmi_unified_enable_ext_wow_cmd(void *wmi_hdl,
			struct ext_wow_params *params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_enable_ext_wow_cmd)
		return wmi_handle->ops->send_enable_ext_wow_cmd(wmi_handle,
			    params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_app_type2_params_in_fw_cmd() - set app type2 params in fw
 * @wmi_handle: wmi handle
 * @appType2Params: app type2 params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_set_app_type2_params_in_fw_cmd(void *wmi_hdl,
					  struct app_type2_params *appType2Params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_app_type2_params_in_fw_cmd)
		return wmi_handle->ops->send_set_app_type2_params_in_fw_cmd(wmi_handle,
			     appType2Params);

	return CDF_STATUS_E_FAILURE;

}

/**
 * wmi_unified_set_auto_shutdown_timer_cmd() - sets auto shutdown timer in firmware
 * @wmi_handle: wmi handle
 * @timer_val: auto shutdown timer value
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_set_auto_shutdown_timer_cmd(void *wmi_hdl,
						  uint32_t timer_val)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_auto_shutdown_timer_cmd)
		return wmi_handle->ops->send_set_auto_shutdown_timer_cmd(wmi_handle,
			    timer_val);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_nan_req_cmd() - to send nan request to target
 * @wmi_handle: wmi handle
 * @nan_req: request data which will be non-null
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_nan_req_cmd(void *wmi_hdl,
			struct nan_req_params *nan_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_nan_req_cmd)
		return wmi_handle->ops->send_nan_req_cmd(wmi_handle,
			    nan_req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_dhcpserver_offload_cmd() - enable DHCP server offload
 * @wmi_handle: wmi handle
 * @pDhcpSrvOffloadInfo: DHCP server offload info
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_process_dhcpserver_offload_cmd(void *wmi_hdl,
				struct dhcp_offload_info_params *pDhcpSrvOffloadInfo)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_dhcpserver_offload_cmd)
		return wmi_handle->ops->send_process_dhcpserver_offload_cmd(wmi_handle,
			    pDhcpSrvOffloadInfo);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_ch_avoid_update_cmd() - handles channel avoid update request
 * @wmi_handle: wmi handle
 * @ch_avoid_update_req: channel avoid update params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_process_ch_avoid_update_cmd(void *wmi_hdl)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_ch_avoid_update_cmd)
		return wmi_handle->ops->send_process_ch_avoid_update_cmd(wmi_handle);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_send_regdomain_info_to_fw_cmd() - send regdomain info to fw
 * @wmi_handle: wmi handle
 * @reg_dmn: reg domain
 * @regdmn2G: 2G reg domain
 * @regdmn5G: 5G reg domain
 * @ctl2G: 2G test limit
 * @ctl5G: 5G test limit
 *
 * Return: none
 */
CDF_STATUS wmi_unified_send_regdomain_info_to_fw_cmd(void *wmi_hdl,
				   uint32_t reg_dmn, uint16_t regdmn2G,
				   uint16_t regdmn5G, int8_t ctl2G,
				   int8_t ctl5G)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_regdomain_info_to_fw_cmd)
		return wmi_handle->ops->send_regdomain_info_to_fw_cmd(wmi_handle,
			    reg_dmn, regdmn2G,
				regdmn5G, ctl2G,
				ctl5G);

	return CDF_STATUS_E_FAILURE;
}


/**
 * wmi_unified_set_tdls_offchan_mode_cmd() - set tdls off channel mode
 * @wmi_handle: wmi handle
 * @chan_switch_params: Pointer to tdls channel switch parameter structure
 *
 * This function sets tdls off channel mode
 *
 * Return: 0 on success; Negative errno otherwise
 */
CDF_STATUS wmi_unified_set_tdls_offchan_mode_cmd(void *wmi_hdl,
			      struct tdls_channel_switch_params *chan_switch_params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_tdls_offchan_mode_cmd)
		return wmi_handle->ops->send_set_tdls_offchan_mode_cmd(wmi_handle,
			    chan_switch_params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_update_fw_tdls_state_cmd() - send enable/disable tdls for a vdev
 * @wmi_handle: wmi handle
 * @pwmaTdlsparams: TDLS params
 *
 * Return: 0 for sucess or error code
 */
CDF_STATUS wmi_unified_update_fw_tdls_state_cmd(void *wmi_hdl,
					 void *tdls_param, uint8_t tdls_state)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_update_fw_tdls_state_cmd)
		return wmi_handle->ops->send_update_fw_tdls_state_cmd(wmi_handle,
			    tdls_param, tdls_state);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_update_tdls_peer_state_cmd() - update TDLS peer state
 * @wmi_handle: wmi handle
 * @peerStateParams: TDLS peer state params
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_update_tdls_peer_state_cmd(void *wmi_hdl,
			       struct tdls_peer_state_params *peerStateParams,
				   uint32_t *ch_mhz)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_update_tdls_peer_state_cmd)
		return wmi_handle->ops->send_update_tdls_peer_state_cmd(wmi_handle,
			    peerStateParams, ch_mhz);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_fw_mem_dump_cmd() - Function to request fw memory dump from
 * firmware
 * @wmi_handle:         Pointer to wmi handle
 * @mem_dump_req:       Pointer for mem_dump_req
 *
 * This function sends memory dump request to firmware
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 *
 */
CDF_STATUS wmi_unified_process_fw_mem_dump_cmd(void *wmi_hdl,
					struct fw_dump_req_param *mem_dump_req)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_fw_mem_dump_cmd)
		return wmi_handle->ops->send_process_fw_mem_dump_cmd(wmi_handle,
			    mem_dump_req);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_process_set_ie_info_cmd() - Function to send IE info to firmware
 * @wmi_handle:    Pointer to WMi handle
 * @ie_data:       Pointer for ie data
 *
 * This function sends IE information to firmware
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 *
 */
CDF_STATUS wmi_unified_process_set_ie_info_cmd(void *wmi_hdl,
				   struct vdev_ie_info_param *ie_info)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_set_ie_info_cmd)
		return wmi_handle->ops->send_process_set_ie_info_cmd(wmi_handle,
			    ie_info);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_send_init_cmd() - wmi init command
 * @wmi_handle:      pointer to wmi handle
 * @res_cfg:         resource config
 * @num_mem_chunks:  no of mem chunck
 * @mem_chunk:       pointer to mem chunck structure
 *
 * This function sends IE information to firmware
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 *
 */
CDF_STATUS wmi_unified_send_init_cmd(void *wmi_hdl,
		wmi_resource_config *res_cfg,
		uint8_t num_mem_chunks, struct wmi_host_mem_chunk *mem_chunk,
		bool action)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_init_cmd)
		return wmi_handle->ops->send_init_cmd(wmi_handle,
			    res_cfg, num_mem_chunks, mem_chunk, action);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_send_saved_init_cmd() - wmi init command
 * @wmi_handle:      pointer to wmi handle
 *
 * This function sends IE information to firmware
 *
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 *
 */
CDF_STATUS wmi_unified_send_saved_init_cmd(void *wmi_hdl)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_saved_init_cmd)
		return wmi_handle->ops->send_saved_init_cmd(wmi_handle);

	return CDF_STATUS_E_FAILURE;
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
 * Return: CDF_STATUS_SUCCESS for success otherwise failure
 *
 */
CDF_STATUS wmi_unified_save_fw_version_cmd(void *wmi_hdl,
		void *evt_buf)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->save_fw_version_cmd)
		return wmi_handle->ops->save_fw_version_cmd(wmi_handle,
			    evt_buf);

	return CDF_STATUS_E_FAILURE;
}

/**
 * send_set_base_macaddr_indicate_cmd() - set base mac address in fw
 * @wmi_hdl: wmi handle
 * @custom_addr: base mac address
 *
 * Return: 0 for success or error code
 */
CDF_STATUS wmi_unified_set_base_macaddr_indicate_cmd(void *wmi_hdl,
					 uint8_t *custom_addr)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_base_macaddr_indicate_cmd)
		return wmi_handle->ops->send_set_base_macaddr_indicate_cmd(wmi_handle,
			    custom_addr);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_log_supported_evt_cmd() - Enable/Disable FW diag/log events
 * @wmi_hdl: wmi handle
 * @event:  Event received from FW
 * @len:    Length of the event
 *
 * Enables the low frequency events and disables the high frequency
 * events. Bit 17 indicates if the event if low/high frequency.
 * 1 - high frequency, 0 - low frequency
 *
 * Return: 0 on successfully enabling/disabling the events
 */
CDF_STATUS wmi_unified_log_supported_evt_cmd(void *wmi_hdl,
		uint8_t *event,
		uint32_t len)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_log_supported_evt_cmd)
		return wmi_handle->ops->send_log_supported_evt_cmd(wmi_handle,
			    event, len);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_enable_specific_fw_logs_cmd() - Start/Stop logging of diag log id
 * @wmi_hdl: wmi handle
 * @start_log: Start logging related parameters
 *
 * Send the command to the FW based on which specific logging of diag
 * event/log id can be started/stopped
 *
 * Return: None
 */
CDF_STATUS wmi_unified_enable_specific_fw_logs_cmd(void *wmi_hdl,
		struct wmi_wifi_start_log *start_log)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_enable_specific_fw_logs_cmd)
		return wmi_handle->ops->send_enable_specific_fw_logs_cmd(wmi_handle,
			    start_log);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_flush_logs_to_fw_cmd() - Send log flush command to FW
 * @wmi_hdl: WMI handle
 *
 * This function is used to send the flush command to the FW,
 * that will flush the fw logs that are residue in the FW
 *
 * Return: None
 */
CDF_STATUS wmi_unified_flush_logs_to_fw_cmd(void *wmi_hdl)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_flush_logs_to_fw_cmd)
		return wmi_handle->ops->send_flush_logs_to_fw_cmd(wmi_handle);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_soc_set_pcl_cmd() - Send WMI_SOC_SET_PCL_CMDID to FW
 * @wmi_hdl: wmi handle
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
CDF_STATUS wmi_unified_soc_set_pcl_cmd(void *wmi_hdl,
				struct wmi_pcl_list *msg)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_soc_set_pcl_cmd)
		return wmi_handle->ops->send_soc_set_pcl_cmd(wmi_handle, msg);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_soc_set_hw_mode_cmd() - Send WMI_SOC_SET_HW_MODE_CMDID to FW
 * @wmi_hdl: wmi handle
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
CDF_STATUS wmi_unified_soc_set_hw_mode_cmd(void *wmi_hdl,
				uint32_t hw_mode_index)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_soc_set_hw_mode_cmd)
		return wmi_handle->ops->send_soc_set_hw_mode_cmd(wmi_handle,
				  hw_mode_index);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_soc_set_dual_mac_config_cmd() - Set dual mac config to FW
 * @wmi_hdl: wmi handle
 * @msg: Dual MAC config parameters
 *
 * Configures WLAN firmware with the dual MAC features
 *
 * Return: CDF_STATUS. 0 on success.
 */
CDF_STATUS wmi_unified_soc_set_dual_mac_config_cmd(void *wmi_hdl,
		struct wmi_dual_mac_config *msg)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_soc_set_dual_mac_config_cmd)
		return wmi_handle->ops->send_soc_set_dual_mac_config_cmd(wmi_handle,
				  msg);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_enable_arp_ns_offload_cmd() - enable ARP NS offload
 * @wmi_hdl: wmi handle
 * @param: offload request
 * @arp_only: flag
 *
 * To configure ARP NS off load data to firmware
 * when target goes to wow mode.
 *
 * Return: CDF Status
 */
CDF_STATUS wmi_unified_enable_arp_ns_offload_cmd(void *wmi_hdl,
			   struct host_offload_req_param *param, bool arp_only,
			   uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_enable_arp_ns_offload_cmd)
		return wmi_handle->ops->send_enable_arp_ns_offload_cmd(wmi_handle,
				  param, arp_only,
				  vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_led_flashing_cmd() - set led flashing in fw
 * @wmi_hdl: wmi handle
 * @flashing: flashing request
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_set_led_flashing_cmd(void *wmi_hdl,
				struct flashing_req_params *flashing)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_led_flashing_cmd)
		return wmi_handle->ops->send_set_led_flashing_cmd(wmi_handle,
				  flashing);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_app_type1_params_in_fw_cmd() - set app type1 params in fw
 * @wmi_hdl: wmi handle
 * @appType1Params: app type1 params
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_app_type1_params_in_fw_cmd(void *wmi_hdl,
				   struct app_type1_params *app_type1_params)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_app_type1_params_in_fw_cmd)
		return wmi_handle->ops->send_app_type1_params_in_fw_cmd(wmi_handle,
				  app_type1_params);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_set_ssid_hotlist_cmd() - Handle an SSID hotlist set request
 * @wmi_hdl: wmi handle
 * @request: SSID hotlist set request
 *
 * Return: CDF_STATUS enumeration
 */
CDF_STATUS
wmi_unified_set_ssid_hotlist_cmd(void *wmi_hdl,
		     struct ssid_hotlist_request_params *request)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_set_ssid_hotlist_cmd)
		return wmi_handle->ops->send_set_ssid_hotlist_cmd(wmi_handle,
				  request);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_roam_synch_complete_cmd() - roam synch complete command to fw.
 * @wmi_hdl: wmi handle
 * @vdev_id: vdev id
 *
 * This function sends roam synch complete event to fw.
 *
 * Return: CDF STATUS
 */
CDF_STATUS wmi_unified_roam_synch_complete_cmd(void *wmi_hdl,
		 uint8_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_process_roam_synch_complete_cmd)
		return wmi_handle->ops->send_process_roam_synch_complete_cmd(wmi_handle,
				  vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_unit_test_cmd() - send unit test command to fw.
 * @wmi_hdl: wmi handle
 * @wmi_utest: unit test command
 *
 * This function send unit test command to fw.
 *
 * Return: CDF STATUS
 */
CDF_STATUS wmi_unified_unit_test_cmd(void *wmi_hdl,
			       struct wmi_unit_test_cmd *wmi_utest)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_unit_test_cmd)
		return wmi_handle->ops->send_unit_test_cmd(wmi_handle,
				  wmi_utest);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified__roam_invoke_cmd() - send roam invoke command to fw.
 * @wmi_hdl: wmi handle
 * @roaminvoke: roam invoke command
 *
 * Send roam invoke command to fw for fastreassoc.
 *
 * Return: none
 */
CDF_STATUS wmi_unified_roam_invoke_cmd(void *wmi_hdl,
		struct wmi_roam_invoke_cmd *roaminvoke,
		uint32_t ch_hz)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_roam_invoke_cmd)
		return wmi_handle->ops->send_roam_invoke_cmd(wmi_handle,
				  roaminvoke, ch_hz);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_roam_scan_offload_cmd() - set roam offload command
 * @wmi_hdl: wmi handle
 * @command: command
 * @vdev_id: vdev id
 *
 * This function set roam offload command to fw.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_roam_scan_offload_cmd(void *wmi_hdl,
					 uint32_t command, uint32_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_roam_scan_offload_cmd)
		return wmi_handle->ops->send_roam_scan_offload_cmd(wmi_handle,
				  command, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_send_roam_scan_offload_ap_cmd() - set roam ap profile in fw
 * @wmi_hdl: wmi handle
 * @ap_profile_p: ap profile
 * @vdev_id: vdev id
 *
 * Send WMI_ROAM_AP_PROFILE to firmware
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_send_roam_scan_offload_ap_cmd(void *wmi_hdl,
					    wmi_ap_profile *ap_profile_p,
					    uint32_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_roam_scan_offload_ap_profile_cmd)
		return wmi_handle->ops->send_roam_scan_offload_ap_profile_cmd(wmi_handle,
				  ap_profile_p, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_roam_scan_offload_scan_period() - set roam offload scan period
 * @wmi_handle: wmi handle
 * @scan_period: scan period
 * @scan_age: scan age
 * @vdev_id: vdev id
 *
 * Send WMI_ROAM_SCAN_PERIOD parameters to fw.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_roam_scan_offload_scan_period(void *wmi_hdl,
					     uint32_t scan_period,
					     uint32_t scan_age,
					     uint32_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_roam_scan_offload_scan_period_cmd)
		return wmi_handle->ops->send_roam_scan_offload_scan_period_cmd(wmi_handle,
				  scan_period, scan_age, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_roam_scan_offload_chan_list_cmd() - set roam offload channel list
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
CDF_STATUS wmi_unified_roam_scan_offload_chan_list_cmd(void *wmi_hdl,
				   uint8_t chan_count,
				   uint8_t *chan_list,
				   uint8_t list_type, uint32_t vdev_id)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_roam_scan_offload_chan_list_cmd)
		return wmi_handle->ops->send_roam_scan_offload_chan_list_cmd(wmi_handle,
				  chan_count, chan_list,
				  list_type, vdev_id);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_roam_scan_offload_rssi_change_cmd() - set roam offload RSSI th
 * @wmi_hdl: wmi handle
 * @rssi_change_thresh: RSSI Change threshold
 * @bcn_rssi_weight: beacon RSSI weight
 * @vdev_id: vdev id
 *
 * Send WMI_ROAM_SCAN_RSSI_CHANGE_THRESHOLD parameters to fw.
 *
 * Return: CDF status
 */
CDF_STATUS wmi_unified_roam_scan_offload_rssi_change_cmd(void *wmi_hdl,
	uint32_t vdev_id,
	int32_t rssi_change_thresh,
	uint32_t bcn_rssi_weight,
	uint32_t hirssi_delay_btw_scans)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_roam_scan_offload_rssi_change_cmd)
		return wmi_handle->ops->send_roam_scan_offload_rssi_change_cmd(wmi_handle,
				  vdev_id, rssi_change_thresh,
				  bcn_rssi_weight, hirssi_delay_btw_scans);

	return CDF_STATUS_E_FAILURE;
}

/**
 * wmi_unified_get_buf_extscan_hotlist_cmd() - prepare hotlist command
 * @wmi_hdl: wmi handle
 * @photlist: hotlist command params
 * @buf_len: buffer length
 *
 * This function fills individual elements for  hotlist request and
 * TLV for bssid entries
 *
 * Return: CDF Status.
 */
CDF_STATUS wmi_unified_get_buf_extscan_hotlist_cmd(void *wmi_hdl,
				   struct ext_scan_setbssi_hotlist_params *
				   photlist, int *buf_len)
{
	wmi_unified_t wmi_handle = (wmi_unified_t) wmi_hdl;

	if (wmi_handle->ops->send_get_buf_extscan_hotlist_cmd)
		return wmi_handle->ops->send_get_buf_extscan_hotlist_cmd(wmi_handle,
				  photlist, buf_len);

	return CDF_STATUS_E_FAILURE;
}

