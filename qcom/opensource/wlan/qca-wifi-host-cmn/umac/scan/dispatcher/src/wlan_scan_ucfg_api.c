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

/*
 * DOC: contains scan north bound interface definitions
 */

#include <scheduler_api.h>
#include <wlan_scan_ucfg_api.h>
#include <wlan_objmgr_global_obj.h>
#include <wlan_objmgr_cmn.h>
#include <wlan_serialization_api.h>
#include <wlan_scan_tgt_api.h>
#include "../../core/src/wlan_scan_main.h"
#include "../../core/src/wlan_scan_manager.h"
#include "../../core/src/wlan_scan_cache_db.h"

QDF_STATUS ucfg_scan_register_bcn_cb(struct wlan_objmgr_psoc *psoc,
	update_beacon_cb cb, enum scan_cb_type type)
{
	return scm_scan_register_bcn_cb(psoc, cb, type);
}

qdf_list_t *ucfg_scan_get_result(struct wlan_objmgr_pdev *pdev,
	struct scan_filter *filter)
{
	return scm_get_scan_result(pdev, filter);
}

QDF_STATUS ucfg_scan_db_iterate(struct wlan_objmgr_pdev *pdev,
	scan_iterator_func func, void *arg)
{
	return scm_iterate_scan_db(pdev, func, arg);
}

QDF_STATUS ucfg_scan_purge_results(qdf_list_t *scan_list)
{
	return scm_purge_scan_results(scan_list);
}

QDF_STATUS ucfg_scan_flush_results(struct wlan_objmgr_pdev *pdev,
	struct scan_filter *filter)
{
	return scm_flush_results(pdev, filter);
}

void ucfg_scan_filter_valid_channel(struct wlan_objmgr_pdev *pdev,
	uint8_t *chan_list, uint32_t num_chan)
{
	scm_filter_valid_channel(pdev, chan_list, num_chan);
}

QDF_STATUS ucfg_scan_init(void)
{
	QDF_STATUS status;

	status = wlan_objmgr_register_psoc_create_handler(WLAN_UMAC_COMP_SCAN,
		wlan_scan_psoc_created_notification, NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		scm_err("Failed to register psoc create handler");
		goto fail_create_psoc;
	}

	status = wlan_objmgr_register_psoc_destroy_handler(WLAN_UMAC_COMP_SCAN,
		wlan_scan_psoc_destroyed_notification, NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		scm_err("Failed to create psoc delete handler");
		goto fail_psoc_destroy;
	}
	scm_info("scan psoc create and delete handler registered with objmgr");

	status = wlan_objmgr_register_vdev_create_handler(WLAN_UMAC_COMP_SCAN,
		wlan_scan_vdev_created_notification, NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		scm_err("Failed to register vdev create handler");
		goto fail_pdev_create;
	}

	status = wlan_objmgr_register_vdev_destroy_handler(WLAN_UMAC_COMP_SCAN,
		wlan_scan_vdev_destroyed_notification, NULL);
	if (QDF_IS_STATUS_SUCCESS(status)) {
		scm_info("scan vdev create and delete handler registered with objmgr");
		return QDF_STATUS_SUCCESS;
	}

	scm_err("Failed to destroy vdev delete handler");
	wlan_objmgr_unregister_vdev_create_handler(WLAN_UMAC_COMP_SCAN,
				wlan_scan_vdev_created_notification, NULL);
fail_pdev_create:
	wlan_objmgr_unregister_psoc_destroy_handler(WLAN_UMAC_COMP_SCAN,
				wlan_scan_psoc_destroyed_notification, NULL);
fail_psoc_destroy:
	wlan_objmgr_unregister_psoc_create_handler(WLAN_UMAC_COMP_SCAN,
			wlan_scan_psoc_created_notification, NULL);
fail_create_psoc:
	return status;
}

QDF_STATUS ucfg_scan_deinit(void)
{
	QDF_STATUS status;

	status = wlan_objmgr_unregister_psoc_create_handler(WLAN_UMAC_COMP_SCAN,
		wlan_scan_psoc_created_notification, NULL);
	if (status != QDF_STATUS_SUCCESS)
		scm_err("Failed to unregister psoc create handler");

	status = wlan_objmgr_unregister_psoc_destroy_handler(
				WLAN_UMAC_COMP_SCAN,
				wlan_scan_psoc_destroyed_notification, NULL);
	if (status != QDF_STATUS_SUCCESS)
		scm_err("Failed to unregister psoc delete handler");

	status = wlan_objmgr_unregister_vdev_create_handler(WLAN_UMAC_COMP_SCAN,
		wlan_scan_vdev_created_notification, NULL);
	if (status != QDF_STATUS_SUCCESS)
		scm_err("Failed to unregister vdev create handler");

	status = wlan_objmgr_unregister_vdev_destroy_handler(
			WLAN_UMAC_COMP_SCAN,
			wlan_scan_vdev_destroyed_notification, NULL);
	if (status != QDF_STATUS_SUCCESS)
		scm_err("Failed to unregister vdev delete handler");

	return status;
}

#ifdef FEATURE_WLAN_SCAN_PNO

QDF_STATUS ucfg_scan_pno_start(struct wlan_objmgr_vdev *vdev,
	struct pno_scan_req_params *req)
{
	struct scan_vdev_obj *scan_vdev_obj;
	QDF_STATUS status;

	scan_vdev_obj = wlan_get_vdev_scan_obj(vdev);
	if (!scan_vdev_obj) {
		scm_err("null scan_vdev_obj");
		return QDF_STATUS_E_INVAL;
	}
	if (scan_vdev_obj->pno_in_progress) {
		scm_err("pno already in progress");
		return QDF_STATUS_E_ALREADY;
	}

	status = tgt_scan_pno_start(vdev, req);
	if (QDF_IS_STATUS_ERROR(status))
		scm_err("pno start failed");
	else
		scan_vdev_obj->pno_in_progress = true;

	return status;
}

QDF_STATUS ucfg_scan_pno_stop(struct wlan_objmgr_vdev *vdev)
{
	struct scan_vdev_obj *scan_vdev_obj;
	QDF_STATUS status;

	scan_vdev_obj = wlan_get_vdev_scan_obj(vdev);
	if (!scan_vdev_obj) {
		scm_err("null scan_vdev_obj");
		return QDF_STATUS_E_INVAL;
	}
	if (!scan_vdev_obj->pno_in_progress) {
		scm_err("pno already stopped");
		return QDF_STATUS_E_ALREADY;
	}

	status = tgt_scan_pno_stop(vdev, wlan_vdev_get_id(vdev));
	if (QDF_IS_STATUS_ERROR(status))
		scm_err("pno start failed");
	else
		scan_vdev_obj->pno_in_progress = false;

	return status;
}

bool ucfg_scan_get_pno_in_progress(struct wlan_objmgr_vdev *vdev)
{
	struct scan_vdev_obj *scan_vdev_obj;

	scan_vdev_obj = wlan_get_vdev_scan_obj(vdev);
	if (!scan_vdev_obj) {
		scm_err("null scan_vdev_obj");
		return false;
	}

	return scan_vdev_obj->pno_in_progress;
}

bool ucfg_scan_get_pno_match(struct wlan_objmgr_vdev *vdev)
{
	struct scan_vdev_obj *scan_vdev_obj;

	scan_vdev_obj = wlan_get_vdev_scan_obj(vdev);
	if (!scan_vdev_obj) {
		scm_err("null scan_vdev_obj");
		return false;
	}

	return scan_vdev_obj->pno_match_evt_received;
}

static QDF_STATUS
wlan_pno_global_init(struct pno_def_config *pno_def)
{
	qdf_wake_lock_create(&pno_def->pno_wake_lock, "wlan_pno_wl");
	pno_def->channel_prediction = SCAN_PNO_CHANNEL_PREDICTION;
	pno_def->top_k_num_of_channels = SCAN_TOP_K_NUM_OF_CHANNELS;
	pno_def->stationary_thresh = SCAN_STATIONARY_THRESHOLD;
	pno_def->channel_prediction_full_scan =
			SCAN_CHANNEL_PREDICTION_FULL_SCAN_MS;
	pno_def->adaptive_dwell_mode = SCAN_ADAPTIVE_PNOSCAN_DWELL_MODE;

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS
wlan_pno_global_deinit(struct pno_def_config *pno_def)
{
	qdf_wake_lock_destroy(&pno_def->pno_wake_lock);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_get_pno_def_params(struct wlan_objmgr_vdev *vdev,
	struct pno_scan_req_params *req)
{
	struct scan_default_params *scan_def;
	struct wlan_scan_obj *scan = wlan_vdev_get_scan_obj(vdev);
	struct pno_def_config *pno_def;

	if (!vdev | !req | !scan) {
		scm_err("vdev: 0x%p, req: 0x%p scan_obj: 0x%p",
			vdev, req, scan);
		return QDF_STATUS_E_INVAL;
	}

	scan_def = wlan_vdev_get_def_scan_params(vdev);
	pno_def = &scan->pno_cfg;

	req->active_dwell_time = scan_def->active_dwell;
	req->passive_dwell_time = scan_def->passive_dwell;

	req->adaptive_dwell_mode = pno_def->adaptive_dwell_mode;

	req->pno_channel_prediction = pno_def->adaptive_dwell_mode;
	req->top_k_num_of_channels = pno_def->top_k_num_of_channels;
	req->stationary_thresh = pno_def->stationary_thresh;
	req->channel_prediction_full_scan =
			pno_def->channel_prediction_full_scan;

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS ucfg_scan_update_pno_config(struct pno_def_config *pno,
	struct pno_user_cfg *pno_cfg)
{
	pno->channel_prediction = pno_cfg->channel_prediction;
	pno->top_k_num_of_channels = pno_cfg->top_k_num_of_channels;
	pno->stationary_thresh = pno_cfg->stationary_thresh;
	pno->adaptive_dwell_mode = pno_cfg->adaptive_dwell_mode;
	pno->channel_prediction_full_scan =
		pno_cfg->channel_prediction_full_scan;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_register_pno_cb(struct wlan_objmgr_psoc *psoc,
	scan_event_handler event_cb, void *arg)
{
	struct wlan_scan_obj *scan;

	if (!psoc) {
		scm_err("null psoc");
		return QDF_STATUS_E_INVAL;
	}
	scan = wlan_psoc_get_scan_obj(psoc);
	qdf_spin_lock_bh(&scan->lock);
	scan->pno_cfg.pno_cb.func = event_cb;
	scan->pno_cfg.pno_cb.arg = arg;
	qdf_spin_unlock_bh(&scan->lock);
	scm_info("event_cb: 0x%p, arg: 0x%p", event_cb, arg);

	return QDF_STATUS_SUCCESS;
}

#else

static inline QDF_STATUS
wlan_pno_global_init(struct pno_def_config *pno_def)
{
	return QDF_STATUS_SUCCESS;
}
static inline QDF_STATUS
wlan_pno_global_deinit(struct pno_def_config *pno_def)
{
	return QDF_STATUS_SUCCESS;
}

static inline QDF_STATUS
ucfg_scan_update_pno_config(struct pno_def_config *pno,
	struct pno_user_cfg *pno_cfg)
{
	return QDF_STATUS_SUCCESS;
}

#endif


QDF_STATUS
ucfg_scan_start(struct scan_start_request *req)
{
	struct scheduler_msg msg = {0, };
	QDF_STATUS status;

	if (!req || !req->vdev) {
		scm_err("vdev: %p, req: %p", req->vdev, req);
		return QDF_STATUS_E_NULL_VALUE;
	}
	scm_info("reqid: %d, scanid: %d, vdevid: %d",
		req->scan_req.scan_req_id, req->scan_req.scan_id,
		req->scan_req.vdev_id);
	msg.bodyptr = req;
	msg.callback = scm_scan_start_req;

	status = scheduler_post_msg(QDF_MODULE_ID_OS_IF, &msg);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		scm_err("failed to post to QDF_MODULE_ID_OS_IF");
		qdf_mem_free(req);
	}

	return status;
}

QDF_STATUS
ucfg_scan_cancel(struct scan_cancel_request *req)
{
	struct scheduler_msg msg = {0, };
	QDF_STATUS status;

	if (!req || !req->vdev) {
		scm_err("vdev: %p, req: %p", req->vdev, req);
		if (req)
			qdf_mem_free(req);
		return QDF_STATUS_E_NULL_VALUE;
	}
	scm_info("reqid: %d, scanid: %d, vdevid: %d, type: %d",
		req->cancel_req.requester, req->cancel_req.scan_id,
		req->cancel_req.vdev_id, req->cancel_req.req_type);
	msg.bodyptr = req;
	msg.callback = scm_scan_cancel_req;

	status = scheduler_post_msg(QDF_MODULE_ID_OS_IF, &msg);
	if (!QDF_IS_STATUS_SUCCESS(status)) {
		scm_err("failed to post to QDF_MODULE_ID_OS_IF");
		qdf_mem_free(req);
	}

	return status;
}

wlan_scan_requester
ucfg_scan_register_requester(struct wlan_objmgr_psoc *psoc,
	uint8_t *name, scan_event_handler event_cb, void *arg)
{
	int i, j;
	struct wlan_scan_obj *scan;
	struct scan_requester_info *requesters;
	wlan_scan_requester requester = {0};

	if (!psoc) {
		scm_err("null psoc");
		return 0;
	}
	scan = wlan_psoc_get_scan_obj(psoc);
	requesters = scan->requesters;
	qdf_spin_lock_bh(&scan->lock);
	for (i = 0; i < WLAN_MAX_REQUESTORS; ++i) {
		if (requesters[i].requester == 0) {
			requesters[i].requester =
				WLAN_SCAN_REQUESTER_ID_PREFIX | i;
			j = 0;
			while (name[j] && (j < (WLAN_MAX_MODULE_NAME - 1))) {
				requesters[i].module[j] = name[j];
				++j;
			}
			requesters[i].module[j] = 0;
			requesters[i].ev_handler.func = event_cb;
			requesters[i].ev_handler.arg = arg;
			requester = requesters[i].requester;
			break;
		}
	}
	qdf_spin_unlock_bh(&scan->lock);
	scm_info("module: %s, event_cb: 0x%p, arg: 0x%p, reqid: %d",
		name, event_cb, arg, requester);

	return requester;
}

void
ucfg_scan_unregister_requester(struct wlan_objmgr_psoc *psoc,
	wlan_scan_requester requester)
{
	int idx = requester & ~WLAN_SCAN_REQUESTER_ID_PREFIX;
	struct wlan_scan_obj *scan;
	struct scan_requester_info *requesters;

	if (!psoc) {
		scm_err("null psoc");
		return;
	}
	scan = wlan_psoc_get_scan_obj(psoc);
	requesters = scan->requesters;
	scm_info("reqid: %d", requester);

	qdf_spin_lock_bh(&scan->lock);
	requesters[idx].requester = 0;
	requesters[idx].module[0] = 0;
	requesters[idx].ev_handler.func = NULL;
	requesters[idx].ev_handler.arg = NULL;
	qdf_spin_unlock_bh(&scan->lock);
}

uint8_t*
ucfg_get_scan_requester_name(struct wlan_objmgr_psoc *psoc,
	wlan_scan_requester requester)
{
	int idx = requester & ~WLAN_SCAN_REQUESTER_ID_PREFIX;
	struct wlan_scan_obj *scan;
	struct scan_requester_info *requesters;

	if (!psoc) {
		scm_err("null psoc");
		return "null";
	}
	scan = wlan_psoc_get_scan_obj(psoc);
	requesters = scan->requesters;

	if ((idx < WLAN_MAX_REQUESTORS) &&
		(requesters[idx].requester == requester)) {
		return requesters[idx].module;
	}

	return (uint8_t *)"unknown";
}

wlan_scan_id
ucfg_scan_get_scan_id(struct wlan_objmgr_psoc *psoc)
{
	wlan_scan_id id;
	struct wlan_scan_obj *scan;

	if (!psoc) {
		QDF_ASSERT(0);
		scm_err("null psoc");
		return 0;
	}
	scan = wlan_psoc_get_scan_obj(psoc);

	id = qdf_atomic_inc_return(&scan->scan_ids);
	id =  id & WLAN_SCAN_ID_MASK;
	/* Mark this scan request as triggered by host
	 * by setting WLAN_HOST_SCAN_REQ_ID_PREFIX flag.
	 */
	id =  id | WLAN_HOST_SCAN_REQ_ID_PREFIX;
	scm_info("scan_id: 0x%x", id);

	return id;
}

static QDF_STATUS
scm_add_scan_event_handler(struct pdev_scan_ev_handler *pdev_ev_handler,
	scan_event_handler event_cb, void *arg)
{
	struct cb_handler *cb_handler;
	uint32_t handler_cnt = pdev_ev_handler->handler_cnt;

	/* Assign next available slot to this registration request */
	cb_handler = &(pdev_ev_handler->cb_handlers[handler_cnt]);
	cb_handler->func = event_cb;
	cb_handler->arg = arg;
	pdev_ev_handler->handler_cnt++;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_register_event_handler(struct wlan_objmgr_pdev *pdev,
	scan_event_handler event_cb, void *arg)
{
	uint32_t idx;
	struct wlan_scan_obj *scan;
	struct pdev_scan_ev_handler *pdev_ev_handler;
	struct cb_handler *cb_handler;

	/* scan event handler call back can't be NULL */
	if (!pdev || !event_cb) {
		scm_err("pdev: %p, event_cb: %p", pdev, event_cb);
		return QDF_STATUS_E_NULL_VALUE;
	}

	scm_info("pdev: %p, event_cb: %p, arg: %p\n", pdev, event_cb, arg);

	scan = wlan_pdev_get_scan_obj(pdev);
	pdev_ev_handler = wlan_pdev_get_pdev_scan_ev_handlers(pdev);
	cb_handler = &(pdev_ev_handler->cb_handlers[0]);

	qdf_spin_lock_bh(&scan->lock);
	/* Ensure its not a duplicate registration request */
	for (idx = 0; idx < MAX_SCAN_EVENT_HANDLERS_PER_PDEV;
		idx++, cb_handler++) {
		if ((cb_handler->func == event_cb) &&
			(cb_handler->arg == arg)) {
			qdf_spin_unlock_bh(&scan->lock);
			scm_warn("func: %p, arg: %p already exists",
				event_cb, arg);
			return QDF_STATUS_SUCCESS;
		}
	}

	QDF_ASSERT(pdev_ev_handler->handler_cnt <
			MAX_SCAN_EVENT_HANDLERS_PER_PDEV);

	if (pdev_ev_handler->handler_cnt >= MAX_SCAN_EVENT_HANDLERS_PER_PDEV) {
		qdf_spin_unlock_bh(&scan->lock);
		scm_warn("No more registrations possible");
		return QDF_STATUS_E_NOMEM;
	}

	scm_add_scan_event_handler(pdev_ev_handler, event_cb, arg);
	qdf_spin_unlock_bh(&scan->lock);

	scm_info("event_cb: 0x%p, arg: 0x%p", event_cb, arg);

	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS
wlan_scan_global_init(struct wlan_scan_obj *scan_obj)
{
	scan_obj->scan_def.active_dwell = SCAN_ACTIVE_DWELL_TIME;
	scan_obj->scan_def.passive_dwell = SCAN_PASSIVE_DWELL_TIME;
	scan_obj->scan_def.max_rest_time = SCAN_MAX_REST_TIME;
	scan_obj->scan_def.min_rest_time = SCAN_MIN_REST_TIME;
	scan_obj->scan_def.conc_active_dwell = SCAN_CONC_ACTIVE_DWELL_TIME;
	scan_obj->scan_def.conc_passive_dwell = SCAN_CONC_PASSIVE_DWELL_TIME;
	scan_obj->scan_def.conc_max_rest_time = SCAN_CONC_MAX_REST_TIME;
	scan_obj->scan_def.conc_min_rest_time = SCAN_CONC_MIN_REST_TIME;
	scan_obj->scan_def.conc_idle_time = SCAN_CONC_IDLE_TIME;
	scan_obj->scan_def.repeat_probe_time = SCAN_REPEAT_PROBE_TIME;
	scan_obj->scan_def.probe_spacing_time = SCAN_PROBE_SPACING_TIME;
	scan_obj->scan_def.probe_delay = SCAN_PROBE_DELAY;
	scan_obj->scan_def.burst_duration = SCAN_BURST_DURATION;
	scan_obj->scan_def.max_scan_time = SCAN_MAX_SCAN_TIME;
	scan_obj->scan_def.num_probes = SCAN_NUM_PROBES;
	scan_obj->scan_def.scan_cache_aging_time = SCAN_CACHE_AGING_TIME;
	scan_obj->scan_def.max_bss_per_pdev = SCAN_MAX_BSS_PDEV;
	scan_obj->scan_def.max_num_scan_allowed = SCAN_MAX_NUM_SCAN_ALLOWED;
	scan_obj->scan_def.scan_priority = SCAN_PRIORITY;
	scan_obj->scan_def.idle_time = SCAN_NETWORK_IDLE_TIMEOUT;
	scan_obj->scan_def.adaptive_dwell_time_mode = SCAN_DWELL_MODE_DEFAULT;
	/* scan contrl flags */
	scan_obj->scan_def.scan_f_passive = true;
	scan_obj->scan_def.scan_f_ofdm_rates = true;
	scan_obj->scan_def.scan_f_2ghz = true;
	scan_obj->scan_def.scan_f_5ghz = true;
	/* scan event flags */
	scan_obj->scan_def.scan_ev_started = true;
	scan_obj->scan_def.scan_ev_completed = true;
	scan_obj->scan_def.scan_ev_bss_chan = true;
	scan_obj->scan_def.scan_ev_foreign_chan = true;
	scan_obj->scan_def.scan_ev_dequeued = true;
	scan_obj->scan_def.scan_ev_preempted = true;
	scan_obj->scan_def.scan_ev_start_failed = true;
	scan_obj->scan_def.scan_ev_restarted = true;
	/* init scan id seed */
	qdf_atomic_init(&scan_obj->scan_ids);

	return wlan_pno_global_init(&scan_obj->pno_cfg);
}

static QDF_STATUS
scm_remove_scan_event_handler(struct pdev_scan_ev_handler *pdev_ev_handler,
	struct cb_handler *entry)
{
	struct cb_handler *last_entry;
	uint32_t handler_cnt = pdev_ev_handler->handler_cnt;

	/* Replace event handler being deleted
	 * with the last one in the list.
	 */
	last_entry = &(pdev_ev_handler->cb_handlers[handler_cnt - 1]);
	entry->func = last_entry->func;
	entry->arg = last_entry->arg;

	/* Clear our last entry */
	last_entry->func = NULL;
	last_entry->arg = NULL;
	pdev_ev_handler->handler_cnt--;

	return QDF_STATUS_SUCCESS;
}

void
ucfg_scan_unregister_event_handler(struct wlan_objmgr_pdev *pdev,
	scan_event_handler event_cb, void *arg)
{
	uint8_t found = false;
	uint32_t idx;
	uint32_t handler_cnt;
	struct wlan_scan_obj *scan;
	struct cb_handler *cb_handler;
	struct pdev_scan_ev_handler *pdev_ev_handler;

	scm_info("pdev: %p, event_cb: 0x%p, arg: 0x%p", pdev, event_cb, arg);
	if (!pdev) {
		scm_err("null pdev");
		return;
	}
	scan = wlan_pdev_get_scan_obj(pdev);
	pdev_ev_handler = wlan_pdev_get_pdev_scan_ev_handlers(pdev);
	cb_handler = &(pdev_ev_handler->cb_handlers[0]);

	qdf_spin_lock_bh(&scan->lock);
	handler_cnt = pdev_ev_handler->handler_cnt;
	if (!handler_cnt) {
		qdf_spin_unlock_bh(&scan->lock);
		scm_info("No event handlers registered");
		return;
	}

	for (idx = 0; idx < MAX_SCAN_EVENT_HANDLERS_PER_PDEV;
		idx++, cb_handler++) {
		if ((cb_handler->func == event_cb) &&
			(cb_handler->arg == arg)) {
			/* Event handler found, remove it
			 * from event handler list.
			 */
			found = true;
			scm_remove_scan_event_handler(pdev_ev_handler,
				cb_handler);
			handler_cnt--;
			break;
		}
	}
	qdf_spin_unlock_bh(&scan->lock);

	scm_info("event handler %s, remaining handlers: %d",
		(found ? "removed" : "not found"), handler_cnt);
}

QDF_STATUS
ucfg_scan_init_default_params(struct wlan_objmgr_vdev *vdev,
	struct scan_start_request *req)
{
	struct scan_default_params *def;

	if (!vdev | !req) {
		scm_err("vdev: 0x%p, req: 0x%p", vdev, req);
		return QDF_STATUS_E_INVAL;
	}
	def = wlan_vdev_get_def_scan_params(vdev);

	/* Zero out everything and explicitly set fields as required */
	qdf_mem_zero(req, sizeof(*req));

	req->vdev = vdev;
	req->scan_req.vdev_id = wlan_vdev_get_id(vdev);
	req->scan_req.scan_priority = def->scan_priority;
	req->scan_req.dwell_time_active = def->active_dwell;
	req->scan_req.dwell_time_passive = def->passive_dwell;
	req->scan_req.min_rest_time = def->min_rest_time;
	req->scan_req.max_rest_time = def->max_rest_time;
	req->scan_req.repeat_probe_time = def->repeat_probe_time;
	req->scan_req.probe_spacing_time = def->probe_spacing_time;
	req->scan_req.idle_time = def->idle_time;
	req->scan_req.max_scan_time = def->max_scan_time;
	req->scan_req.probe_delay = def->probe_delay;
	req->scan_req.burst_duration = def->burst_duration;
	req->scan_req.n_probes = def->num_probes;
	req->scan_req.adaptive_dwell_time_mode =
		def->adaptive_dwell_time_mode;
	req->scan_req.scan_flags = def->scan_flags;
	req->scan_req.scan_events = def->scan_events;

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_init_ssid_params(struct scan_start_request *req,
		uint32_t num_ssid, struct wlan_ssid *ssid_list)
{
	uint32_t max_ssid = sizeof(req->scan_req.ssid) /
				sizeof(req->scan_req.ssid[0]);

	if (!req) {
		scm_err("null request");
		return QDF_STATUS_E_NULL_VALUE;
	}
	if (!num_ssid) {
		/* empty channel list provided */
		req->scan_req.num_ssids = 0;
		qdf_mem_zero(&req->scan_req.ssid[0],
			sizeof(req->scan_req.ssid));
		return QDF_STATUS_SUCCESS;
	}
	if (!ssid_list) {
		scm_err("null ssid_list while num_ssid: %d", num_ssid);
		return QDF_STATUS_E_NULL_VALUE;
	}
	if (num_ssid > max_ssid) {
		/* got a big list. alert and continue */
		scm_warn("overflow: received %d, max supported : %d",
			num_ssid, max_ssid);
		return QDF_STATUS_E_E2BIG;
	}

	if (max_ssid > num_ssid)
		max_ssid = num_ssid;

	req->scan_req.num_ssids = max_ssid;
	qdf_mem_copy(&req->scan_req.ssid[0], ssid_list,
		(req->scan_req.num_ssids * sizeof(req->scan_req.ssid[0])));

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_init_bssid_params(struct scan_start_request *req,
		uint32_t num_bssid, struct qdf_mac_addr *bssid_list)
{
	uint32_t max_bssid = sizeof(req->scan_req.bssid_list) /
				sizeof(req->scan_req.bssid_list[0]);

	if (!req) {
		scm_err("null request");
		return QDF_STATUS_E_NULL_VALUE;
	}
	if (!num_bssid) {
		/* empty channel list provided */
		req->scan_req.num_bssid = 0;
		qdf_mem_zero(&req->scan_req.bssid_list[0],
			sizeof(req->scan_req.bssid_list));
		return QDF_STATUS_SUCCESS;
	}
	if (!bssid_list) {
		scm_err("null bssid_list while num_bssid: %d", num_bssid);
		return QDF_STATUS_E_NULL_VALUE;
	}
	if (num_bssid > max_bssid) {
		/* got a big list. alert and continue */
		scm_warn("overflow: received %d, max supported : %d",
			num_bssid, max_bssid);
		return QDF_STATUS_E_E2BIG;
	}

	if (max_bssid > num_bssid)
		max_bssid = num_bssid;

	req->scan_req.num_bssid = max_bssid;
	qdf_mem_copy(&req->scan_req.bssid_list[0], bssid_list,
		req->scan_req.num_bssid * sizeof(req->scan_req.bssid_list[0]));

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_init_chanlist_params(struct scan_start_request *req,
		uint32_t num_chans, uint32_t *chan_list)
{

	uint32_t max_chans = sizeof(req->scan_req.chan_list) /
				sizeof(req->scan_req.chan_list[0]);
	if (!req) {
		scm_err("null request");
		return QDF_STATUS_E_NULL_VALUE;
	}
	if (!num_chans) {
		/* empty channel list provided */
		req->scan_req.num_chan = 0;
		qdf_mem_zero(&req->scan_req.chan_list[0],
			sizeof(req->scan_req.chan_list));
		return QDF_STATUS_SUCCESS;
	}
	if (!chan_list) {
		scm_err("null chan_list while num_chans: %d", num_chans);
		return QDF_STATUS_E_NULL_VALUE;
	}

	if (num_chans > max_chans) {
		/* got a big list. alert and continue */
		scm_warn("overflow: received %d, max supported : %d",
			num_chans, max_chans);
		return QDF_STATUS_E_E2BIG;
	}

	if (max_chans > num_chans)
		max_chans = num_chans;

	req->scan_req.num_chan = max_chans;
	qdf_mem_copy(&req->scan_req.chan_list[0], chan_list,
		req->scan_req.num_chan * sizeof(req->scan_req.chan_list[0]));

	return QDF_STATUS_SUCCESS;
}

static inline enum scm_scan_status
get_scan_status_from_serialization_status(
	enum wlan_serialization_cmd_status status)
{
	enum scm_scan_status scan_status;

	switch (status) {
	case WLAN_SER_CMD_IN_PENDING_LIST:
		scan_status = SCAN_IS_PENDING;
		break;
	case WLAN_SER_CMD_IN_ACTIVE_LIST:
		scan_status = SCAN_IS_ACTIVE;
		break;
	case WLAN_SER_CMDS_IN_ALL_LISTS:
		scan_status = SCAN_IS_ACTIVE_AND_PENDING;
		break;
	case WLAN_SER_CMD_NOT_FOUND:
		scan_status = SCAN_NOT_IN_PROGRESS;
		break;
	default:
		scm_warn("invalid serialization status %d", status);
		QDF_ASSERT(0);
		scan_status = SCAN_NOT_IN_PROGRESS;
		break;
	}

	return scan_status;
}

enum scm_scan_status
ucfg_scan_get_vdev_status(struct wlan_objmgr_vdev *vdev)
{
	enum wlan_serialization_cmd_status status;

	if (!vdev) {
		scm_err("null vdev");
		return QDF_STATUS_E_NULL_VALUE;
	}
	status = wlan_serialization_vdev_scan_status(vdev);

	return get_scan_status_from_serialization_status(status);
}

enum scm_scan_status
ucfg_scan_get_pdev_status(struct wlan_objmgr_pdev *pdev)
{
	enum wlan_serialization_cmd_status status;

	if (!pdev) {
		scm_err("null pdev");
		return QDF_STATUS_E_NULL_VALUE;
	}
	status = wlan_serialization_pdev_scan_status(pdev);

	return get_scan_status_from_serialization_status(status);
}

static void
ucfg_scan_register_unregister_bcn_cb(struct wlan_objmgr_psoc *psoc,
	bool enable)
{
	QDF_STATUS status;
	struct mgmt_txrx_mgmt_frame_cb_info cb_info[2];

	cb_info[0].frm_type = MGMT_PROBE_RESP;
	cb_info[0].mgmt_rx_cb = tgt_scan_bcn_probe_rx_callback;
	cb_info[1].frm_type = MGMT_BEACON;
	cb_info[1].mgmt_rx_cb = tgt_scan_bcn_probe_rx_callback;

	if (enable)
		status = wlan_mgmt_txrx_register_rx_cb(psoc,
					 WLAN_UMAC_COMP_SCAN, cb_info, 2);
	else
		status = wlan_mgmt_txrx_deregister_rx_cb(psoc,
					 WLAN_UMAC_COMP_SCAN, cb_info, 2);
	if (status != QDF_STATUS_SUCCESS)
		scm_err("%s the Handle with MGMT TXRX layer has failed",
			enable ? "Registering" : "Deregistering");
}

QDF_STATUS ucfg_scan_update_user_config(struct wlan_objmgr_psoc *psoc,
	struct scan_user_cfg *scan_cfg)
{
	struct wlan_scan_obj *scan_obj;
	struct scan_default_params *scan_def;

	if (!psoc) {
		scm_err("null psoc");
		return QDF_STATUS_E_FAILURE;
	}
	scan_obj = wlan_psoc_get_scan_obj(psoc);
	if (scan_obj == NULL) {
		scm_err("Failed to get scan object");
		return QDF_STATUS_E_FAILURE;
	}

	scan_def = &scan_obj->scan_def;
	scan_def->active_dwell = scan_cfg->active_dwell;
	scan_def->passive_dwell = scan_cfg->passive_dwell;
	scan_def->conc_active_dwell = scan_cfg->conc_active_dwell;
	scan_def->conc_passive_dwell = scan_cfg->conc_passive_dwell;
	scan_def->conc_max_rest_time = scan_cfg->conc_max_rest_time;
	scan_def->conc_min_rest_time = scan_cfg->conc_min_rest_time;
	scan_def->conc_idle_time = scan_cfg->conc_idle_time;
	scan_def->scan_cache_aging_time = scan_cfg->scan_cache_aging_time;
	scan_def->adaptive_dwell_time_mode = scan_cfg->scan_dwell_time_mode;
	scan_def->scan_f_chan_stat_evnt = scan_cfg->is_snr_monitoring_enabled;

	return ucfg_scan_update_pno_config(&scan_obj->pno_cfg,
		&scan_cfg->pno_cfg);
}

QDF_STATUS
ucfg_scan_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	struct wlan_scan_obj *scan_obj;

	scm_info("psoc open: 0x%p", psoc);
	if (!psoc) {
		scm_err("null psoc");
		return QDF_STATUS_E_FAILURE;
	}
	scan_obj = wlan_psoc_get_scan_obj(psoc);
	if (scan_obj == NULL) {
		scm_err("Failed to get scan object");
		return QDF_STATUS_E_FAILURE;
	}
	/* Initialize the scan Globals */
	wlan_scan_global_init(scan_obj);
	qdf_spinlock_create(&scan_obj->lock);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	struct wlan_scan_obj *scan_obj;

	scm_info("psoc close: 0x%p", psoc);
	if (!psoc) {
		scm_err("null psoc");
		return QDF_STATUS_E_FAILURE;
	}
	scan_obj = wlan_psoc_get_scan_obj(psoc);
	if (scan_obj == NULL) {
		scm_err("Failed to get scan object");
		return QDF_STATUS_E_FAILURE;
	}
	qdf_spinlock_destroy(&scan_obj->lock);
	wlan_pno_global_deinit(&scan_obj->pno_cfg);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS
ucfg_scan_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status;

	scm_info("psoc enable: 0x%p", psoc);
	if (!psoc) {
		scm_err("null psoc");
		return QDF_STATUS_E_FAILURE;
	}
	/* Subscribe for scan events from lmac layesr */
	status = tgt_scan_register_ev_handler(psoc);
	QDF_ASSERT(status == QDF_STATUS_SUCCESS);
	scm_db_init(psoc);
	ucfg_scan_register_unregister_bcn_cb(psoc, true);

	return status;
}

QDF_STATUS
ucfg_scan_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	QDF_STATUS status;

	scm_info("psoc disable: 0x%p", psoc);
	if (!psoc) {
		scm_err("null psoc");
		return QDF_STATUS_E_FAILURE;
	}
	/* Unsubscribe for scan events from lmac layesr */
	status = tgt_scan_unregister_ev_handler(psoc);
	QDF_ASSERT(status == QDF_STATUS_SUCCESS);
	ucfg_scan_register_unregister_bcn_cb(psoc, false);
	scm_db_deinit(psoc);

	return status;
}
