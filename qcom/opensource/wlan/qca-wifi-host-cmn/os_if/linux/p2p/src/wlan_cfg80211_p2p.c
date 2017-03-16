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
 * DOC: defines driver functions interfacing with linux kernel
 */

#include <qdf_util.h>
#include <wlan_objmgr_psoc_obj.h>
#include <wlan_objmgr_global_obj.h>
#include <wlan_objmgr_pdev_obj.h>
#include <wlan_objmgr_vdev_obj.h>
#include <wlan_objmgr_peer_obj.h>
#include <wlan_p2p_public_struct.h>
#include <wlan_p2p_ucfg_api.h>
#include <wlan_utility.h>
#include <wlan_osif_priv.h>
#include "wlan_cfg80211.h"
#include "wlan_cfg80211_p2p.h"

#define MAX_NO_OF_2_4_CHANNELS 14

/**
 * wlan_p2p_rx_callback() - Callback for rx mgmt frame
 * @user_data: pointer to soc object
 * @rx_frame: RX mgmt frame information
 *
 * This callback will be used to rx frames in os interface.
 *
 * Return: None
 */
static void wlan_p2p_rx_callback(void *user_data,
	struct p2p_rx_mgmt_frame *rx_frame)
{
	struct wlan_objmgr_psoc *psoc;
	struct wlan_objmgr_vdev *vdev;
	struct vdev_osif_priv *osif_priv;
	struct wireless_dev *wdev;
	uint16_t freq;

	cfg80211_debug("user data:%p, vdev id:%d, rssi:%d, buf:%p, len:%d",
		user_data, rx_frame->vdev_id, rx_frame->rx_rssi,
		rx_frame->buf, rx_frame->frame_len);

	psoc = user_data;
	if (!psoc) {
		cfg80211_err("psoc is null");
		return;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc,
		rx_frame->vdev_id, WLAN_P2P_ID);
	if (!vdev) {
		cfg80211_err("vdev is null");
		return;
	}

	wlan_vdev_obj_lock(vdev);
	osif_priv = wlan_vdev_get_ospriv(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!osif_priv) {
		cfg80211_err("osif_priv is null");
		goto fail;
	}

	wdev = osif_priv->wdev;
	if (!wdev) {
		cfg80211_err("wdev is null");
		goto fail;
	}

	if (rx_frame->rx_chan <= MAX_NO_OF_2_4_CHANNELS)
		freq = ieee80211_channel_to_frequency(
			rx_frame->rx_chan, NL80211_BAND_2GHZ);
	else
		freq = ieee80211_channel_to_frequency(
			rx_frame->rx_chan, NL80211_BAND_5GHZ);

	cfg80211_notice("Indicate frame over nl80211, vdev id:%d, idx:%d",
		   rx_frame->vdev_id, wdev->netdev->ifindex);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0))
	cfg80211_rx_mgmt(wdev, freq, rx_frame->rx_rssi * 100,
		rx_frame->buf, rx_frame->frame_len,
		NL80211_RXMGMT_FLAG_ANSWERED);
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 12, 0))
	cfg80211_rx_mgmt(wdev, freq, rx_frame->rx_rssi * 100,
		rx_frame->buf, rx_frame->frame_len,
		NL80211_RXMGMT_FLAG_ANSWERED, GFP_ATOMIC);
#else
	cfg80211_rx_mgmt(wdev, freq, rx_frame->rx_rssi * 100,
		rx_frame->buf, rx_frame->frame_len, GFP_ATOMIC);
#endif /* LINUX_VERSION_CODE */
fail:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_P2P_ID);
}

/**
 * wlan_p2p_action_tx_cnf_callback() - Callback for tx confirmation
 * @user_data: pointer to soc object
 * @tx_cnf: tx confirmation information
 *
 * This callback will be used to give tx mgmt frame confirmation to
 * os interface.
 *
 * Return: None
 */
static void wlan_p2p_action_tx_cnf_callback(void *user_data,
	struct p2p_tx_cnf *tx_cnf)
{
	struct wlan_objmgr_psoc *psoc;
	struct wlan_objmgr_vdev *vdev;
	struct vdev_osif_priv *osif_priv;
	struct wireless_dev *wdev;

	cfg80211_info("user data:%p, action cookie:%llx, buf:%p, len:%d, tx status:%d",
		user_data, tx_cnf->action_cookie, tx_cnf->buf,
		tx_cnf->buf_len, tx_cnf->status);

	psoc = user_data;
	if (!psoc) {
		cfg80211_err("psoc is null");
		return;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc,
		tx_cnf->vdev_id, WLAN_P2P_ID);
	if (!vdev) {
		cfg80211_err("vdev is null");
		return;
	}

	wlan_vdev_obj_lock(vdev);
	osif_priv = wlan_vdev_get_ospriv(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!osif_priv) {
		cfg80211_err("osif_priv is null");
		goto fail;
	}

	wdev = osif_priv->wdev;
	if (!wdev) {
		cfg80211_err("wireless dev is null");
		goto fail;
	}

	cfg80211_mgmt_tx_status(
		wdev,
		tx_cnf->action_cookie,
		tx_cnf->buf, tx_cnf->buf_len,
		(bool)tx_cnf->status, GFP_KERNEL);
fail:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_P2P_ID);
}

/**
 * wlan_p2p_lo_event_callback() - Callback for listen offload event
 * @user_data: pointer to soc object
 * @p2p_lo_event: listen offload event information
 *
 * This callback will be used to give listen offload event to os interface.
 *
 * Return: None
 */
static void wlan_p2p_lo_event_callback(void *user_data,
	struct p2p_lo_event *p2p_lo_event)
{
	struct wlan_objmgr_psoc *psoc;
	struct wlan_objmgr_vdev *vdev;
	struct vdev_osif_priv *osif_priv;
	struct wireless_dev *wdev;
	struct sk_buff *vendor_event;

	cfg80211_debug("user data:%p, vdev id:%d, reason code:%d",
		user_data, p2p_lo_event->vdev_id,
		p2p_lo_event->reason_code);

	psoc = user_data;
	if (!psoc) {
		cfg80211_err("psoc is null");
		return;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc,
		p2p_lo_event->vdev_id, WLAN_P2P_ID);
	if (!vdev) {
		cfg80211_err("vdev is null");
		return;
	}

	wlan_vdev_obj_lock(vdev);
	osif_priv = wlan_vdev_get_ospriv(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!osif_priv) {
		cfg80211_err("osif_priv is null");
		goto fail;
	}

	wdev = osif_priv->wdev;
	if (!wdev) {
		cfg80211_err("wireless dev is null");
		goto fail;
	}

	vendor_event = cfg80211_vendor_event_alloc(wdev->wiphy, NULL,
			sizeof(uint32_t) + NLMSG_HDRLEN,
			QCA_NL80211_VENDOR_SUBCMD_P2P_LO_EVENT_INDEX,
			GFP_KERNEL);
	if (!vendor_event) {
		cfg80211_err("cfg80211_vendor_event_alloc failed");
		goto fail;
	}

	if (nla_put_u32(vendor_event,
		QCA_WLAN_VENDOR_ATTR_P2P_LISTEN_OFFLOAD_STOP_REASON,
		p2p_lo_event->reason_code)) {
		cfg80211_err("nla put failed");
		kfree_skb(vendor_event);
		goto fail;
	}

	cfg80211_vendor_event(vendor_event, GFP_KERNEL);

fail:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_P2P_ID);
}

/**
 * wlan_p2p_event_callback() - Callback for P2P event
 * @user_data: pointer to soc object
 * @p2p_event: p2p event information
 *
 * This callback will be used to give p2p event to os interface.
 *
 * Return: None
 */
static void wlan_p2p_event_callback(void *user_data,
	struct p2p_event *p2p_event)
{
	struct wlan_objmgr_psoc *psoc;
	struct wlan_objmgr_vdev *vdev;
	struct ieee80211_channel *chan;
	struct vdev_osif_priv *osif_priv;
	struct wireless_dev *wdev;

	cfg80211_debug("user data:%p, vdev id:%d, event type:%d",
		user_data, p2p_event->vdev_id, p2p_event->roc_event);

	psoc = user_data;
	if (!psoc) {
		cfg80211_err("psoc is null");
		return;
	}

	vdev = wlan_objmgr_get_vdev_by_id_from_psoc(psoc,
		p2p_event->vdev_id, WLAN_P2P_ID);
	if (!vdev) {
		cfg80211_err("vdev is null");
		return;
	}

	wlan_vdev_obj_lock(vdev);
	osif_priv = wlan_vdev_get_ospriv(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!osif_priv) {
		cfg80211_err("osif_priv is null");
		goto fail;
	}

	wdev = osif_priv->wdev;
	if (!wdev) {
		cfg80211_err("wireless dev is null");
		goto fail;
	}

	chan = __ieee80211_get_channel(wdev->wiphy,
			wlan_chan_to_freq(p2p_event->chan));
	if (p2p_event->roc_event == ROC_EVENT_READY_ON_CHAN) {
		cfg80211_ready_on_channel(wdev,
			p2p_event->cookie, chan,
			p2p_event->duration, GFP_KERNEL);
	} else if (p2p_event->roc_event == ROC_EVENT_COMPLETED) {
		cfg80211_remain_on_channel_expired(wdev,
			p2p_event->cookie, chan, GFP_KERNEL);
	} else {
		cfg80211_err("Invalid p2p event");
	}

fail:
	wlan_objmgr_vdev_release_ref(vdev, WLAN_P2P_ID);
}

QDF_STATUS wlan_p2p_start(struct wlan_objmgr_psoc *psoc)
{
	struct p2p_start_param start_param;

	if (!psoc) {
		cfg80211_err("psoc null");
		return QDF_STATUS_E_INVAL;
	}

	start_param.rx_cb = wlan_p2p_rx_callback;
	start_param.rx_cb_data = psoc;
	start_param.event_cb = wlan_p2p_event_callback;
	start_param.event_cb_data = psoc;
	start_param.tx_cnf_cb = wlan_p2p_action_tx_cnf_callback;
	start_param.tx_cnf_cb_data = psoc;
	start_param.lo_event_cb = wlan_p2p_lo_event_callback;
	start_param.lo_event_cb_data = psoc;

	return ucfg_p2p_psoc_start(psoc, &start_param);
}

QDF_STATUS wlan_p2p_stop(struct wlan_objmgr_psoc *psoc)
{
	if (!psoc) {
		cfg80211_err("psoc null");
		return QDF_STATUS_E_INVAL;
	}

	return ucfg_p2p_psoc_stop(psoc);
}

int wlan_cfg80211_roc(struct wlan_objmgr_vdev *vdev,
	struct ieee80211_channel *chan, uint32_t duration,
	uint64_t *cookie)
{
	struct p2p_roc_req roc_req;
	struct wlan_objmgr_psoc *psoc;
	uint8_t vdev_id;

	if (!vdev) {
		cfg80211_err("invalid vdev object");
		return -EINVAL;
	}

	if (!chan) {
		cfg80211_err("invalid channel");
		return -EINVAL;
	}

	wlan_vdev_obj_lock(vdev);
	psoc = wlan_vdev_get_psoc(vdev);
	vdev_id = wlan_vdev_get_id(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!psoc) {
		cfg80211_err("psoc handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	roc_req.chan = (uint32_t)wlan_freq_to_chan(chan->center_freq);
	roc_req.duration = duration;
	roc_req.vdev_id = (uint32_t)vdev_id;

	return qdf_status_to_os_return(
		ucfg_p2p_roc_req(psoc, &roc_req, cookie));
}

int wlan_cfg80211_cancel_roc(struct wlan_objmgr_vdev *vdev,
		uint64_t cookie)
{
	struct wlan_objmgr_psoc *psoc;

	if (!vdev) {
		cfg80211_err("invalid vdev object");
		return -EINVAL;
	}

	wlan_vdev_obj_lock(vdev);
	psoc = wlan_vdev_get_psoc(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!psoc) {
		cfg80211_err("psoc handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	return qdf_status_to_os_return(
		ucfg_p2p_roc_cancel_req(psoc, cookie));
}

int wlan_cfg80211_mgmt_tx(struct wlan_objmgr_vdev *vdev,
		struct ieee80211_channel *chan, bool offchan,
		unsigned int wait,
		const uint8_t *buf, uint32_t len, bool no_cck,
		bool dont_wait_for_ack, uint64_t *cookie)
{
	struct p2p_mgmt_tx mgmt_tx;
	struct wlan_objmgr_psoc *psoc;
	uint8_t vdev_id;

	if (!vdev) {
		cfg80211_err("invalid vdev object");
		return -EINVAL;
	}

	if (!chan) {
		cfg80211_err("invalid channel");
		return -EINVAL;
	}

	wlan_vdev_obj_lock(vdev);
	psoc = wlan_vdev_get_psoc(vdev);
	vdev_id = wlan_vdev_get_id(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!psoc) {
		cfg80211_err("psoc handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	mgmt_tx.vdev_id = (uint32_t)vdev_id;
	mgmt_tx.chan = (uint32_t)wlan_freq_to_chan(chan->center_freq);
	mgmt_tx.wait = wait;
	mgmt_tx.len = len;
	mgmt_tx.no_cck = (uint32_t)no_cck;
	mgmt_tx.dont_wait_for_ack = (uint32_t)dont_wait_for_ack;
	mgmt_tx.buf = buf;

	return qdf_status_to_os_return(
		ucfg_p2p_mgmt_tx(psoc, &mgmt_tx, cookie));
}

int wlan_cfg80211_mgmt_tx_cancel(struct wlan_objmgr_vdev *vdev,
	uint64_t cookie)
{
	struct wlan_objmgr_psoc *psoc;

	if (!vdev) {
		cfg80211_err("invalid vdev object");
		return -EINVAL;
	}

	wlan_vdev_obj_lock(vdev);
	psoc = wlan_vdev_get_psoc(vdev);
	wlan_vdev_obj_unlock(vdev);
	if (!psoc) {
		cfg80211_err("psoc handle is NULL");
		return QDF_STATUS_E_INVAL;
	}

	return qdf_status_to_os_return(
		ucfg_p2p_mgmt_tx_cancel(psoc, cookie));
}
