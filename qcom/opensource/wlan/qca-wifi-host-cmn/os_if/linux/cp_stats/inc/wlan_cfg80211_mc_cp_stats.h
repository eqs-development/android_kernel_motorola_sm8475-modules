/*
 * Copyright (c) 2011-2018 The Linux Foundation. All rights reserved.
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
 * DOC: wlan_cfg80211_mc_cp_stats.h
 *
 * This Header file provide declaration for cfg80211 command handler API
 * registered cp stats and specific with ic
 */

#ifndef __WLAN_CFG80211_MC_CP_STATS_H__
#define __WLAN_CFG80211_MC_CP_STATS_H__

#ifdef QCA_SUPPORT_CP_STATS

/* forward declaration */
struct wiphy;
struct wlan_objmgr_psoc;

/**
 * wlan_cfg80211_mc_cp_stats_get_wakelock_stats() - API to request wake lock
 * stats. Stats are returned to user space via vender event
 * @psoc:    Pointer to psoc
 * @wiphy:   wiphy pointer
 *
 * Return: 0 on success, negative value on failure
 */
int wlan_cfg80211_mc_cp_stats_get_wakelock_stats(struct wlan_objmgr_psoc *psoc,
						 struct wiphy *wiphy);

/**
 * wlan_cfg80211_mc_cp_stats_get_tx_power() - API to fetch tx power
 * @vdev:    Pointer to vdev
 * @dbm:     Pointer to TX power in dbm
 *
 * Return: 0 on success, negative value on failure
 */
int wlan_cfg80211_mc_cp_stats_get_tx_power(struct wlan_objmgr_vdev *vdev,
					   int *dbm);

/**
 * wlan_cfg80211_mc_cp_stats_get_peer_rssi() - API to fetch peer rssi
 * @vdev:    Pointer to vdev
 * @macaddress: mac address
 * @rssi_info: stats structure within which rssi info will be populated
 *
 * Return: 0 on success, negative value on failure
 */
int wlan_cfg80211_mc_cp_stats_get_peer_rssi(struct wlan_objmgr_vdev *vdev,
					    uint8_t *macaddress,
					    struct stats_event *rssi_info);

#endif /* QCA_SUPPORT_CP_STATS */
#endif /* __WLAN_CFG80211_MC_CP_STATS_H__ */
