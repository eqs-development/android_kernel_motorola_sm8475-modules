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
 * DOC: wlan_tdls_main.h
 *
 * TDLS core function declaration
 */

#if !defined(_WLAN_TDLS_MAIN_H_)
#define _WLAN_TDLS_MAIN_H_

#include <qdf_trace.h>
#include <qdf_list.h>
#include <wlan_objmgr_psoc_obj.h>
#include <wlan_objmgr_pdev_obj.h>
#include <wlan_objmgr_vdev_obj.h>
#include <wlan_objmgr_peer_obj.h>
#include <wlan_tdls_public_structs.h>
#include <scheduler_api.h>

/* Bit mask flag for tdls_option to FW */
#define ENA_TDLS_OFFCHAN      (1 << 0)  /* TDLS Off Channel support */
#define ENA_TDLS_BUFFER_STA   (1 << 1)  /* TDLS Buffer STA support */
#define ENA_TDLS_SLEEP_STA    (1 << 2)  /* TDLS Sleep STA support */

#define tdls_log(level, args...) \
	QDF_TRACE(QDF_MODULE_ID_TDLS, level, ## args)
#define tdls_logfl(level, format, args...) \
	tdls_log(level, FL(format), ## args)

#define tdls_debug(format, args...) \
	tdls_logfl(QDF_TRACE_LEVEL_DEBUG, format, ## args)
#define tdls_notice(format, args...) \
	tdls_logfl(QDF_TRACE_LEVEL_INFO, format, ## args)
#define tdls_warn(format, args...) \
	tdls_logfl(QDF_TRACE_LEVEL_WARN, format, ## args)
#define tdls_err(format, args...) \
	tdls_logfl(QDF_TRACE_LEVEL_ERROR, format, ## args)
#define tdls_alert(format, args...) \
	tdls_logfl(QDF_TRACE_LEVEL_FATAL, format, ## args)

#define TDLS_IS_CONNECTED(peer)  \
	((TDLS_LINK_CONNECTED == (peer)->link_status) || \
	 (TDLS_LINK_TEARING == (peer)->link_status))

#define SET_BIT(value, mask) ((value) |= (1 << (mask)))
#define CLEAR_BIT(value, mask) ((value) &= ~(1 << (mask)))
#define CHECK_BIT(value, mask) ((value) & (1 << (mask)))
/**
 * struct tdls_conn_info - TDLS connection record
 * @session_id: session id
 * @sta_id: sta id
 * @peer_mac: peer address
 */
struct tdls_conn_info {
	uint8_t session_id;
	uint8_t sta_id;
	struct qdf_mac_addr peer_mac;
};

/**
 * enum tdls_nss_transition_state - TDLS NSS transition states
 * @TDLS_NSS_TRANSITION_UNKNOWN: default state
 * @TDLS_NSS_TRANSITION_2x2_to_1x1: transition from 2x2 to 1x1 stream
 * @TDLS_NSS_TRANSITION_1x1_to_2x2: transition from 1x1 to 2x2 stream
 */
enum tdls_nss_transition_state {
	TDLS_NSS_TRANSITION_S_UNKNOWN = 0,
	TDLS_NSS_TRANSITION_S_2x2_to_1x1,
	TDLS_NSS_TRANSITION_S_1x1_to_2x2,
};

/**
 * enum tdls_command_type - TDLS command type
 * @TDLS_CMD_TX_ACTION: send tdls action frame
 * @TDLS_CMD_ADD_STA: add tdls peer
 * @TDLS_CMD_CHANGE_STA: change tdls peer
 * @TDLS_CMD_ENABLE_LINK: enable tdls link
 * @TDLS_CMD_DISABLE_LINK: disable tdls link
 * @TDLS_CMD_CONFIG_FORCE_PEER: config external peer
 * @TDLS_CMD_REMOVE_FORCE_PEER: remove external peer
 * @TDLS_CMD_STATS_UPDATE: update tdls stats
 * @TDLS_CMD_CONFIG_UPDATE: config tdls
 */
enum tdls_commmand_type {
	TDLS_CMD_TX_ACTION = 1,
	TDLS_CMD_ADD_STA,
	TDLS_CMD_CHANGE_STA,
	TDLS_CMD_ENABLE_LINK,
	TDLS_CMD_DISABLE_LINK,
	TDLS_CMD_CONFIG_FORCE_PEER,
	TDLS_CMD_REMOVE_FORCE_PEER,
	TDLS_CMD_STATS_UPDATE,
	TDLS_CMD_CONFIG_UPDATE
};

/**
 * struct tdls_conn_tracker_mac_table - connection tracker peer table
 * @mac_address: peer mac address
 * @tx_packet_cnt: number of tx pkts
 * @rx_packet_cnt: number of rx pkts
 * @peer_timestamp_ms: time stamp of latest peer traffic
 */
struct tdls_conn_tracker_mac_table {
	struct qdf_mac_addr mac_address;
	uint32_t tx_packet_cnt;
	uint32_t rx_packet_cnt;
	uint32_t peer_timestamp_ms;
};

/**
 * struct tdls_set_state_db - to record set tdls state command, we need to
 * set correct tdls state to firmware:
 * 1. enable tdls in firmware before tdls connection;
 * 2. disable tdls if concurrency happen, before disable tdls, all active peer
 * should be deleted in firmware.
 *
 * @set_state_cnt: tdls set state count
 * @vdev_id: vdev id of last set state command
 */
struct tdls_set_state_info {
	uint8_t set_state_cnt;
	uint8_t vdev_id;
};

/**
 * struct tdls_psoc_priv_ctx - tdls context
 * @soc: objmgr psoc
 * @tdls_current_mode: current tdls mode
 * @tdls_user_config_mode: user configure tdls mode
 * @tdls_conn_info: this tdls_conn_info can be removed and we can use peer type
 *                of peer object to get the active tdls peers
 * @tdls_configs: tdls user configure
 * @max_num_tdls_sta: maximum TDLS station number allowed upon runtime condition
 * @connected_peer_count: tdls peer connected count
 * @tdls_off_channel: tdls off channel number
 * @tdls_channel_offset: tdls channel offset
 * @tdls_fw_off_chan_mode: tdls fw off channel mode
 * @enable_tdls_connection_tracker: enable tdls connection tracker
 * @tdls_external_peer_count: external tdls peer count
 * @tdls_nss_switch_in_progress: tdls antenna switch in progress
 * @tdls_nss_teardown_complete: tdls tear down complete
 * @tdls_nss_transition_mode: tdls nss transition mode
 * @tdls_teardown_peers_cnt: tdls tear down peer count
 * @tdls_tx_cnf_cb: callback registered by hdd to receive the ack cnf
 * @set_state_info: set tdls state info
 * @tx_ack_cnf_cb_data: user data to tdls_tx_cnf_cb
 * @tdls_event_cb: tdls event callback
 * @tdls_evt_cb_data: tdls event user data
 * @tx_q_ack: queue for tx frames waiting for ack
 * @tdls_con_cap: tdls concurrency support
 */
struct tdls_soc_priv_obj {
	struct wlan_objmgr_psoc *soc;
	enum tdls_support_mode tdls_current_mode;
	enum tdls_support_mode tdls_user_config_mode;
	struct tdls_conn_info tdls_conn_info[WLAN_TDLS_STA_MAX_NUM];
	struct tdls_user_config tdls_configs;
	uint16_t max_num_tdls_sta;
	uint16_t connected_peer_count;
	uint8_t tdls_off_channel;
	uint16_t tdls_channel_offset;
	int32_t tdls_fw_off_chan_mode;
	bool enable_tdls_connection_tracker;
	uint8_t tdls_external_peer_count;
	bool tdls_nss_switch_in_progress;
	bool tdls_nss_teardown_complete;
	enum tdls_nss_transition_state tdls_nss_transition_mode;
	int32_t tdls_teardown_peers_cnt;
	struct tdls_set_state_info set_state_info;
	tdls_tx_ack_cnf_callback tdls_tx_cnf_cb;
	void *tx_ack_cnf_cb_data;
	tdls_evt_callback tdls_event_cb;
	void *tdls_evt_cb_data;
	qdf_list_t tx_q_ack;
	enum tdls_conc_cap tdls_con_cap;
};

/**
 * struct tdls_vdev_priv_obj - tdls private vdev object
 * @vdev: vdev objmgr object
 * @peer_list: tdls peer list on this vdev
 * @peer_update_timer: connection tracker timer
 * @peer_dicovery_timer: peer discovery timer
 * @threshold_config: threshold config
 * @discovery_peer_cnt: discovery peer count
 * @discovery_sent_cnt: discovery sent count
 * @ap_rssi: ap rssi
 * @curr_candidate: current candidate
 * @ct_peer_table: linear mac address table for counting the packets
 * @valid_mac_entries: number of valid mac entry in @ct_peer_mac_table
 * @magic: magic
 * @tx_queue: tx frame queue
 */
struct tdls_vdev_priv_obj {
	struct wlan_objmgr_vdev *vdev;
	qdf_list_t peer_list[WLAN_TDLS_PEER_LIST_SIZE];
	qdf_mc_timer_t peer_update_timer;
	qdf_mc_timer_t peer_discovery_timer;
	struct tdls_config_params threshold_config;
	int32_t discovery_peer_cnt;
	uint32_t discovery_sent_cnt;
	int8_t ap_rssi;
	struct tdls_peer *curr_candidate;
	struct tdls_conn_tracker_mac_table
			ct_peer_table[WLAN_TDLS_CT_TABLE_SIZE];
	uint8_t valid_mac_entries;
	uint32_t magic;
	qdf_list_t tx_queue;
};

/**
 * struct tdls_peer - tdls peer data
 * @node: node
 * @vdev_priv: tdls vdev priv obj
 * @peer_mac: peer mac address
 * @sta_id: station identifier
 * @rssi: rssi
 * @tdls_support: tdls support
 * @link_status: tdls link status
 * @signature: signature
 * @is_responder: is responder
 * @discovery_processed: dicovery processed
 * @discovery_attempt: discovery attempt
 * @tx_pkt: tx packet
 * @rx_pkt: rx packet
 * @uapsd_queues: uapsd queues
 * @max_sp: max sp
 * @buf_sta_capable: is buffer sta
 * @off_channel_capable: is offchannel supported flag
 * @supported_channels_len: supported channels length
 * @supported_channels: supported channels
 * @supported_oper_classes_len: supported operation classes length
 * @supported_oper_classes: supported operation classes
 * @is_forced_peer: is forced peer
 * @op_class_for_pref_off_chan: op class for preferred off channel
 * @pref_off_chan_num: preferred off channel number
 * @op_class_for_pref_off_chan_is_set: op class for preferred off channel set
 * @peer_idle_timer: time to check idle traffic in tdls peers
 * @is_peer_idle_timer_initialised: Flag to check idle timer init
 * @spatial_streams: Number of TX/RX spatial streams for TDLS
 * @reason: reason
 * @state_change_notification: state change notification
 * @qos: QOS capability of TDLS link
 */
struct tdls_peer {
	qdf_list_node_t node;
	struct tdls_vdev_priv_obj *vdev_priv;
	struct qdf_mac_addr peer_mac;
	uint16_t sta_id;
	int8_t rssi;
	enum tdls_peer_capab tdls_support;
	enum tdls_link_status link_status;
	uint8_t signature;
	uint8_t is_responder;
	uint8_t discovery_processed;
	uint16_t discovery_attempt;
	uint16_t tx_pkt;
	uint16_t rx_pkt;
	uint8_t uapsd_queues;
	uint8_t max_sp;
	uint8_t buf_sta_capable;
	uint8_t off_channel_capable;
	uint8_t supported_channels_len;
	uint8_t supported_channels[WLAN_MAC_MAX_SUPP_CHANNELS];
	uint8_t supported_oper_classes_len;
	uint8_t supported_oper_classes[WLAN_MAX_SUPP_OPER_CLASSES];
	bool is_forced_peer;
	uint8_t op_class_for_pref_off_chan;
	uint8_t pref_off_chan_num;
	uint8_t op_class_for_pref_off_chan_is_set;
	qdf_mc_timer_t peer_idle_timer;
	bool is_peer_idle_timer_initialised;
	uint8_t spatial_streams;
	enum tdls_link_reason reason;
	tdls_state_change_callback state_change_notification;
	uint8_t qos;
};

/**
 * wlan_vdev_get_tdls_soc_obj - private API to get tdls soc object from vdev
 * @vdev: vdev object
 *
 * Return: tdls soc object
 */
static inline struct tdls_soc_priv_obj *
wlan_vdev_get_tdls_soc_obj(struct wlan_objmgr_vdev *vdev)
{
	struct wlan_objmgr_psoc *psoc =
		wlan_pdev_get_psoc(wlan_vdev_get_pdev(vdev));

	return (struct tdls_soc_priv_obj *)
		wlan_objmgr_psoc_get_comp_private_obj(psoc,
						      WLAN_UMAC_COMP_TDLS);
}

/**
 * wlan_psoc_get_tdls_soc_obj - private API to get tdls soc object from psoc
 * @psoc: psoc object
 *
 * Return: tdls soc object
 */
static inline struct tdls_soc_priv_obj *
wlan_psoc_get_tdls_soc_obj(struct wlan_objmgr_psoc *psoc)
{
	return (struct tdls_soc_priv_obj *)
		wlan_objmgr_psoc_get_comp_private_obj(psoc,
						      WLAN_UMAC_COMP_TDLS);
}

/**
 * wlan_vdev_get_tdls_vdev_obj - private API to get tdls vdev object from vdev
 * @vdev: vdev object
 *
 * Return: tdls vdev object
 */
static inline struct tdls_vdev_priv_obj *
wlan_vdev_get_tdls_vdev_obj(struct wlan_objmgr_vdev *vdev)
{
	return (struct tdls_vdev_priv_obj *)
		wlan_objmgr_vdev_get_comp_private_obj(vdev,
						      WLAN_UMAC_COMP_TDLS);
}

/**
 * tdls_set_link_status - tdls set link status
 * @vdev: vdev object
 * @mac: mac address of tdls peer
 * @link_status: tdls link status
 * @link_reason: reason
 */
void tdls_set_link_status(struct tdls_vdev_priv_obj *vdev,
			  const uint8_t *mac,
			  enum tdls_link_status link_status,
			  enum tdls_link_reason link_reason);
/**
 * tdls_psoc_obj_create_notification() - tdls psoc create notification handler
 * @psoc: psoc object
 * @arg_list: Argument list
 *
 * Return: QDF_STATUS
 */
QDF_STATUS tdls_psoc_obj_create_notification(struct wlan_objmgr_psoc *psoc,
					     void *arg_list);

/**
 * tdls_psoc_obj_destroy_notification() - tdls psoc destroy notification handler
 * @psoc: psoc object
 * @arg_list: Argument list
 *
 * Return: QDF_STATUS
 */
QDF_STATUS tdls_psoc_obj_destroy_notification(struct wlan_objmgr_psoc *psoc,
					      void *arg_list);

/**
 * tdls_vdev_obj_create_notification() - tdls vdev create notification handler
 * @vdev: vdev object
 * @arg_list: Argument list
 *
 * Return: QDF_STATUS
 */
QDF_STATUS tdls_vdev_obj_create_notification(struct wlan_objmgr_vdev *vdev,
					     void *arg_list);

/**
 * tdls_vdev_obj_destroy_notification() - tdls vdev destroy notification handler
 * @vdev: vdev object
 * @arg_list: Argument list
 *
 * Return: QDF_STATUS
 */
QDF_STATUS tdls_vdev_obj_destroy_notification(struct wlan_objmgr_vdev *vdev,
					      void *arg_list);

/**
 * tdls_process_cmd() - tdls main command process function
 * @msg: scheduler msg
 *
 * Return: QDF_STATUS
 */
QDF_STATUS tdls_process_cmd(struct scheduler_msg *msg);

/**
 * tdls_process_evt() - tdls main event process function
 * @msg: scheduler msg
 *
 * Return: QDF_STATUS
 */
QDF_STATUS tdls_process_evt(struct scheduler_msg *msg);
#endif
