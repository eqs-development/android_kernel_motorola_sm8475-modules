// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 */
#include <linux/slab.h>
#include <dt-bindings/regulator/qcom,rpmh-regulator-levels.h>
#include <linux/clk.h>
#include <linux/clk/qcom.h>

#include "mmrm_debug.h"
#include "mmrm_clk_rsrc_mgr.h"
#include "mmrm_fixedpoint.h"

#define Q16_INT(q) ((q) >> 16)
#define Q16_FRAC(q) ((((q) & 0xFFFF) * 100) >> 16)
#define CLK_RATE_STEP 1000000
#define CLIENT_CB_TIMEOUT msecs_to_jiffies(100)


static int mmrm_sw_update_freq(
	struct mmrm_sw_clk_mgr_info *sinfo, struct mmrm_sw_clk_client_tbl_entry *tbl_entry)
{
	int rc = 0;
	u32 i;
	struct mmrm_driver_data *drv_data = (struct mmrm_driver_data *)sinfo->driver_data;
	struct mmrm_clk_platform_resources *cres = &drv_data->clk_res;
	struct voltage_corner_set *cset = &cres->corner_set;
	long clk_val_min, clk_val_max, clk_val, clk_val_round;
	int voltage_corner;

	clk_val_min = clk_round_rate(tbl_entry->clk, 1);
	clk_val_max = clk_round_rate(tbl_entry->clk, ~0UL);
	d_mpr_h("%s: csid(0x%x): min_clk_rate(%llu) max_clk_rate(%llu)\n",
		__func__,
		tbl_entry->clk_src_id,
		clk_val_min,
		clk_val_max);

	/* init with min val */
	for (i = 0; i < MMRM_VDD_LEVEL_MAX; i++) {
		tbl_entry->freq[i] = clk_val_min;
	}

	/* step through rates */
	for (clk_val = clk_val_min; clk_val < clk_val_max; clk_val += CLK_RATE_STEP) {
		/* get next clk rate */
		clk_val_round = clk_round_rate(tbl_entry->clk, clk_val);
		if (clk_val_round > clk_val_min) {
			clk_val_min = clk_val_round;

			/* Get voltage corner */
			voltage_corner = qcom_clk_get_voltage(tbl_entry->clk, clk_val_round);
			if (voltage_corner < 0 || voltage_corner > mmrm_sw_vdd_corner[MMRM_VDD_LEVEL_TURBO]) {
				break;
			}

			/* voltage corner is below svsl1 */
			if (voltage_corner < mmrm_sw_vdd_corner[MMRM_VDD_LEVEL_SVS_L1]) {
				voltage_corner = mmrm_sw_vdd_corner[MMRM_VDD_LEVEL_SVS_L1];
			}

			/* match vdd level */
			for (i = 0; i < MMRM_VDD_LEVEL_MAX; i++) {
				if (voltage_corner == mmrm_sw_vdd_corner[i])
					break;
			}

			/* update freq */
			while (i < MMRM_VDD_LEVEL_MAX) {
				tbl_entry->freq[i++] = clk_val_round;
			}
		}
	}

	/* print results */
	for (i = 0; i < MMRM_VDD_LEVEL_MAX; i++) {
		d_mpr_h("%s: csid(0x%x) corner(%s) clk_rate(%llu)\n",
			__func__,
			tbl_entry->clk_src_id,
			cset->corner_tbl[i].name,
			tbl_entry->freq[i]);
	}

	return rc;
}

static void mmrm_sw_print_client_data(struct mmrm_sw_clk_mgr_info *sinfo,
			struct mmrm_sw_clk_client_tbl_entry *tbl_entry)
{
	struct mmrm_driver_data *drv_data = (struct mmrm_driver_data *)sinfo->driver_data;
	struct mmrm_clk_platform_resources *cres = &drv_data->clk_res;
	struct voltage_corner_set *cset = &cres->corner_set;
	u32 i, j;

	for (i = 0; i < MMRM_VDD_LEVEL_MAX; i++) {
		d_mpr_h("%s: csid(0x%x) corner(%s) dyn_pwr(%zu) leak_pwr(%zu) \n",
				__func__,
				tbl_entry->clk_src_id,
				cset->corner_tbl[i].name,
				tbl_entry->dyn_pwr[i],
				tbl_entry->leak_pwr[i]);

		for (j = 0; j < MMRM_VDD_LEVEL_MAX; j++) {
			d_mpr_h("%s: csid(0x%x) total_pwr(%zu) cur_ma(%zu)\n",
				__func__,
				tbl_entry->clk_src_id,
				(tbl_entry->dyn_pwr[i] + tbl_entry->leak_pwr[i]),
				tbl_entry->current_ma[i][j]);
		}
	}
}

static int mmrm_sw_update_curr(struct mmrm_sw_clk_mgr_info *sinfo,
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry)
{
	u32 i, j;
	struct mmrm_driver_data *drv_data = (struct mmrm_driver_data *)sinfo->driver_data;
	struct mmrm_clk_platform_resources *cres = &drv_data->clk_res;
	struct voltage_corner_set *cset = &cres->corner_set;
	u32 scaling_factor = 0, voltage_factor = 0;
	fp_t nom_dyn_pwr, nom_leak_pwr, dyn_sc, leak_sc,
		volt, dyn_pwr, leak_pwr, pwr_mw, nom_freq;
	u32 c;
	struct nom_clk_src_info *nom_tbl_entry = NULL;

	for (c = 0; c < sinfo->tot_clk_clients; c++) {
		if (tbl_entry->clk_src_id == sinfo->clk_client_tbl[c].clk_src_id) {
			nom_tbl_entry = &cres->nom_clk_set.clk_src_tbl[c];
			break;
		}
	}
	if (nom_tbl_entry == NULL) {
		d_mpr_h("%s: can't find 0x%x clock src ID\n",
			__func__,
			tbl_entry->clk_src_id);
		return -EINVAL;
	}

	nom_dyn_pwr = FP(Q16_INT(nom_tbl_entry->nom_dyn_pwr),
		Q16_FRAC(nom_tbl_entry->nom_dyn_pwr), 100);

	nom_leak_pwr = FP(Q16_INT(nom_tbl_entry->nom_leak_pwr),
		Q16_FRAC(nom_tbl_entry->nom_leak_pwr), 100);

	nom_freq = tbl_entry->freq[MMRM_VDD_LEVEL_NOM];

	/* update power & current entries for all levels */
	for (i = 0; i < MMRM_VDD_LEVEL_MAX; i++) {
		scaling_factor = cset->corner_tbl[i].scaling_factor_dyn;
		dyn_sc = FP(
			Q16_INT(scaling_factor), Q16_FRAC(scaling_factor), 100);

		scaling_factor = cset->corner_tbl[i].scaling_factor_leak;
		leak_sc = FP(
			Q16_INT(scaling_factor), Q16_FRAC(scaling_factor), 100);

		/* Frequency scaling */
		pwr_mw = fp_mult(nom_dyn_pwr, tbl_entry->freq[i]);
		pwr_mw = fp_div(pwr_mw, nom_freq);

		/* Scaling factor */
		dyn_pwr = fp_mult(pwr_mw, dyn_sc);
		leak_pwr = fp_mult(nom_leak_pwr, leak_sc);

		tbl_entry->dyn_pwr[i] = fp_round(dyn_pwr);
		tbl_entry->leak_pwr[i] = fp_round(leak_pwr);

		for (j = 0; j < MMRM_VDD_LEVEL_MAX; j++) {
			voltage_factor = cset->corner_tbl[j].volt_factor;
			volt = FP(Q16_INT(voltage_factor), Q16_FRAC(voltage_factor), 100);

			tbl_entry->current_ma[i][j] = fp_round(fp_div((dyn_pwr+leak_pwr), volt));
		}
	}
	mmrm_sw_print_client_data(sinfo, tbl_entry);
	return 0;
}

static struct mmrm_client *mmrm_sw_clk_client_register(
	struct mmrm_clk_mgr *sw_clk_mgr,
	struct mmrm_clk_client_desc clk_desc,
	enum mmrm_client_priority priority,
	void *pvt_data,
	notifier_callback_fn_t not_fn_cb)
{
	int rc = 0;
	struct mmrm_client *clk_client = NULL;
	struct mmrm_sw_clk_mgr_info *sinfo = &(sw_clk_mgr->data.sw_info);
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;

	u32 c = 0;
	u32 clk_client_src_id = 0;

	d_mpr_h("%s: entering\n", __func__);

	mutex_lock(&sw_clk_mgr->lock);

	/* check if entry is free in table */
	if (sinfo->tot_clk_clients == sinfo->enabled_clk_clients) {
		d_mpr_e("%s: no free entry to register a clk client\n",
			__func__);
		rc = -EINVAL;
		goto err_nofree_entry;
	}

	/* look for entry that matches domain and id */
	clk_client_src_id = (clk_desc.client_domain << 16 | clk_desc.client_id);
	for (c = 0; c < sinfo->tot_clk_clients; c++) {
		if (clk_client_src_id == sinfo->clk_client_tbl[c].clk_src_id)
			break;
	}

	/* entry not found */
	if (c == sinfo->tot_clk_clients) {
		d_mpr_e("%s: unknown clk client 0x%x\n",
			__func__, clk_client_src_id);
		rc = -EINVAL;
		goto err_nofree_entry;
	}

	tbl_entry = &sinfo->clk_client_tbl[c];

	/* entry already registered */
	if (tbl_entry->client) {
		if (msm_mmrm_allow_multiple_register) {
			tbl_entry->ref_count++;
			d_mpr_h("%s: client csid(0x%x) already registered ref:%d\n",
				__func__, tbl_entry->clk_src_id, tbl_entry->ref_count);
			clk_client = tbl_entry->client;

			mmrm_sw_print_client_data(sinfo, tbl_entry);

			goto exit_found;
		}

		d_mpr_e("%s: client csid(0x%x) already registered\n",
			__func__, tbl_entry->clk_src_id);
		rc = -EINVAL;
		goto err_already_registered;
	}

	/* populate the entry */
	clk_client = kzalloc(sizeof(*clk_client), GFP_KERNEL);
	if (!clk_client) {
		d_mpr_e("%s: failed to allocate memory for clk_client\n",
			__func__);
		rc = -ENOMEM;
		goto err_fail_alloc_clk_client;
	}

	clk_client->client_uid = c;
	clk_client->client_type = MMRM_CLIENT_CLOCK;
	tbl_entry->ref_count = 1;

	/* copy the entries provided by client */
	tbl_entry->client = clk_client;
	strlcpy(tbl_entry->name, clk_desc.name, MMRM_CLK_CLIENT_NAME_SIZE);
	tbl_entry->clk = clk_desc.clk;
	tbl_entry->pri = priority;
	tbl_entry->pvt_data = pvt_data;
	tbl_entry->notifier_cb_fn = not_fn_cb;

	/* print table entry */
	d_mpr_h("%s: csid(0x%x) name(%s) pri(%d) pvt(%p) notifier(%p)\n",
		__func__,
		tbl_entry->clk_src_id,
		tbl_entry->name,
		tbl_entry->pri,
		tbl_entry->pvt_data,
		tbl_entry->notifier_cb_fn);

	/* determine full range of clock freq */
	rc = mmrm_sw_update_freq(sinfo, tbl_entry);
	if (rc) {
		d_mpr_e("%s: csid(0x%x) failed to update freq\n",
			__func__, tbl_entry->clk_src_id);
		goto err_fail_update_entry;
	}

	/* calculate current & scale power for other levels */
	rc = mmrm_sw_update_curr(sinfo, tbl_entry);
	if (rc) {
		d_mpr_e("%s: csid(0x%x) failed to update current\n",
			__func__, tbl_entry->clk_src_id);
		goto err_fail_update_entry;
	}

exit_found:
	mutex_unlock(&sw_clk_mgr->lock);
	d_mpr_h("%s: exiting with success\n", __func__);
	return clk_client;

err_fail_update_entry:
	kfree(clk_client);

err_fail_alloc_clk_client:
	tbl_entry->client = NULL;
	tbl_entry->clk = NULL;
	tbl_entry->pri = 0x0;
	tbl_entry->pvt_data = NULL;
	tbl_entry->notifier_cb_fn = NULL;
err_nofree_entry:
err_already_registered:
	mutex_unlock(&sw_clk_mgr->lock);

	d_mpr_h("%s: error exit\n", __func__);
	return NULL;
}

static int mmrm_sw_clk_client_deregister(struct mmrm_clk_mgr *sw_clk_mgr,
	struct mmrm_client *client)
{
	int rc =  0;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;
	struct mmrm_sw_clk_mgr_info *sinfo = &(sw_clk_mgr->data.sw_info);

	d_mpr_h("%s: entering\n", __func__);

	/* validate the client ptr */
	if (!client) {
		d_mpr_e("%s: invalid client\n");
		rc = -EINVAL;
		goto err_invalid_client;
	}

	if (client->client_uid >= sinfo->tot_clk_clients) {
		d_mpr_e("%s: invalid client uid (%d)\n",
			__func__, client->client_uid);
		rc = -EINVAL;
		goto err_invalid_client;
	}

	mutex_lock(&sw_clk_mgr->lock);

	tbl_entry = &sinfo->clk_client_tbl[client->client_uid];
	if (tbl_entry->ref_count > 0) {
		tbl_entry->ref_count--;
	}

	if (tbl_entry->ref_count == 0) {

		kfree(tbl_entry->client);

		tbl_entry->client = NULL;
		tbl_entry->clk = NULL;
		tbl_entry->pri = 0x0;
		tbl_entry->pvt_data = NULL;
		tbl_entry->notifier_cb_fn = NULL;
	}

	mutex_unlock(&sw_clk_mgr->lock);

	d_mpr_h("%s: exiting with success\n", __func__);
	return rc;

err_invalid_client:
	d_mpr_h("%s: error exit\n", __func__);
	return rc;
}

static int mmrm_sw_get_req_level(
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry,
	unsigned long clk_val, u32 *req_level)
{
	int rc = 0;
	int voltage_corner;
	u32 level;

	/* get voltage corner */
	voltage_corner = qcom_clk_get_voltage(tbl_entry->clk, clk_val);
	if (voltage_corner < 0 || voltage_corner > mmrm_sw_vdd_corner[MMRM_VDD_LEVEL_TURBO]) {
		d_mpr_e("%s: csid(0x%x): invalid voltage corner(%d) for clk rate(%llu)\n",
			__func__,
			tbl_entry->clk_src_id,
			voltage_corner,
			clk_val);
		rc = voltage_corner;
		goto err_invalid_corner;
	}

	/* voltage corner is below svsl1 */
	if (voltage_corner < mmrm_sw_vdd_corner[MMRM_VDD_LEVEL_SVS_L1]) {
		d_mpr_h("%s: csid(0x%x): lower voltage corner(%d)\n",
			__func__,
			tbl_entry->clk_src_id,
			voltage_corner);
		*req_level = MMRM_VDD_LEVEL_SVS_L1;
		goto exit_no_err;
	}

	/* match vdd level */
	for (level = 0; level < MMRM_VDD_LEVEL_MAX; level++) {
		if (voltage_corner == mmrm_sw_vdd_corner[level])
			break;
	}

	if (level == MMRM_VDD_LEVEL_MAX) {
		d_mpr_e("%s: csid(0x%x): invalid voltage corner(%d) for clk rate(%llu)\n",
			__func__,
			tbl_entry->clk_src_id,
			voltage_corner,
			clk_val);
		rc = -EINVAL;
		goto err_invalid_corner;
	}

	*req_level = level;
	d_mpr_h("%s: req_level(%d)\n", __func__, level);

exit_no_err:
	return rc;

err_invalid_corner:
	return rc;
}

static int mmrm_sw_check_req_level(
	struct mmrm_sw_clk_mgr_info *sinfo,
	u32 clk_src_id, u32 req_level, u32 *adj_level)
{
	int rc = 0;
	struct mmrm_sw_peak_current_data *peak_data = &sinfo->peak_cur_data;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;
	u32 c, level = req_level;

	if (req_level >= MMRM_VDD_LEVEL_MAX) {
		d_mpr_e("%s: invalid level %lu\n", __func__, req_level);
		rc = -EINVAL;
		goto err_invalid_level;
	}

	/* req_level is rejected when another client has a higher level */
	if (req_level < peak_data->aggreg_level) {
		for (c = 0; c < sinfo->tot_clk_clients; c++) {
			tbl_entry = &sinfo->clk_client_tbl[c];
			if (IS_ERR_OR_NULL(tbl_entry->clk) || !tbl_entry->clk_rate ||
				tbl_entry->clk_src_id == clk_src_id) {
				continue;
			}
			if (tbl_entry->vdd_level == peak_data->aggreg_level) {
				break;
			}
		}
		/* reject req level */
		if (c < sinfo->tot_clk_clients) {
			level = peak_data->aggreg_level;
		}
	}

	*adj_level = level;
	d_mpr_h("%s: adj_level(%d)\n", __func__, level);
	return rc;

err_invalid_level:
	return rc;
}

static int mmrm_sw_calculate_total_current(
	struct mmrm_sw_clk_mgr_info *sinfo,
	u32 req_level, u32 *total_cur)
{
	int rc = 0;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;
	u32 c, sum_cur = 0;

	if (req_level >= MMRM_VDD_LEVEL_MAX) {
		d_mpr_e("%s: invalid level %lu\n", __func__, req_level);
		rc = -EINVAL;
		goto err_invalid_level;
	}

	/* calculate sum of values (scaled by volt) */
	for (c = 0; c < sinfo->tot_clk_clients; c++) {
		tbl_entry = &sinfo->clk_client_tbl[c];
		if (IS_ERR_OR_NULL(tbl_entry->clk) || !tbl_entry->clk_rate) {
			continue;
		}
		sum_cur += tbl_entry->current_ma[tbl_entry->vdd_level][req_level];
	}

	*total_cur = sum_cur;
	d_mpr_h("%s: total_cur(%d)\n", __func__, sum_cur);
	return rc;

err_invalid_level:
	return rc;
}

static int mmrm_sw_throttle_low_priority_client(
	struct mmrm_sw_clk_mgr_info *sinfo, u32 *delta_cur)
{
	int rc = 0, c = 0;
	bool found_client_throttle = false;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry_throttle_client;
	struct mmrm_client_notifier_data notifier_data;
	struct completion timeout;
	struct mmrm_sw_peak_current_data *peak_data = &sinfo->peak_cur_data;
	u32 now_cur_ma, min_cur_ma;
	long clk_min_level = MMRM_VDD_LEVEL_SVS_L1;

	d_mpr_h("%s: entering\n", __func__);
	init_completion(&timeout);
	for (c = 0; c < sinfo->tot_clk_clients; c++) {
		tbl_entry_throttle_client = &sinfo->clk_client_tbl[c];
		now_cur_ma = tbl_entry_throttle_client->current_ma
			[tbl_entry_throttle_client->vdd_level][peak_data->aggreg_level];
		min_cur_ma = tbl_entry_throttle_client->current_ma[clk_min_level]
						[peak_data->aggreg_level];

		d_mpr_h("%s:csid(0x%x) name(%s) now_cur_ma(%llu) min_cur_ma(%llu) delta_cur(%d)\n",
			__func__, tbl_entry_throttle_client->clk_src_id,
			tbl_entry_throttle_client->name, now_cur_ma, min_cur_ma,
			*delta_cur);

		if (tbl_entry_throttle_client
			&& (tbl_entry_throttle_client->pri == MMRM_CLIENT_PRIOR_LOW)
			&& (now_cur_ma > min_cur_ma) && (now_cur_ma - min_cur_ma > *delta_cur)) {
			found_client_throttle = true;
			d_mpr_h("%s: Throttle client csid(0x%x) name(%s)\n", __func__,
				tbl_entry_throttle_client->clk_src_id,
				tbl_entry_throttle_client->name);
			d_mpr_h("%s: now_cur_ma(%llu) - min_cur_ma(%llu) > delta_cur(%d)\n",
				__func__, now_cur_ma, min_cur_ma, *delta_cur);

			/* found client to throttle, break from here. */
			break;
		}
	}

	/*Client to throttle is found, Throttle this client now to minimum clock rate*/
	if (found_client_throttle) {
		/* Setup notifier */

		notifier_data.cb_type = MMRM_CLIENT_RESOURCE_VALUE_CHANGE;
		notifier_data.cb_data.val_chng.old_val =
			tbl_entry_throttle_client->freq[tbl_entry_throttle_client->vdd_level];
		notifier_data.cb_data.val_chng.new_val =
			tbl_entry_throttle_client->freq[clk_min_level];
		notifier_data.pvt_data = NULL;

		if (tbl_entry_throttle_client->notifier_cb_fn)
			rc = tbl_entry_throttle_client->notifier_cb_fn(&notifier_data);

		if (!wait_for_completion_timeout(&timeout, CLIENT_CB_TIMEOUT))
			d_mpr_h("Intentionally added to wait for 100ms\n", __func__);

		if (rc) {
			d_mpr_e("%s: Client failed to send SUCCESS in callback(%d)\n",
					__func__, tbl_entry_throttle_client->clk_src_id);
			rc = -EINVAL;
			goto err_clk_set_fail;
		}

		rc = clk_set_rate(tbl_entry_throttle_client->clk,
					tbl_entry_throttle_client->freq[clk_min_level]);
		if (rc) {
			d_mpr_e("%s: Failed to throttle the clk csid(%d)\n",
					__func__, tbl_entry_throttle_client->clk_src_id);
			rc = -EINVAL;
			goto err_clk_set_fail;
		} else {
			*delta_cur = now_cur_ma - min_cur_ma;
		}
		/* Store the throttled clock rate of client */
		tbl_entry_throttle_client->clk_rate =
					tbl_entry_throttle_client->freq[clk_min_level];

		/* Store the corner level of throttled client */
		tbl_entry_throttle_client->vdd_level = clk_min_level;

		/* Clearing the reserve flag */
		tbl_entry_throttle_client->reserve = tbl_entry_throttle_client->reserve & 0;
	}
err_clk_set_fail:
	return rc;
}


static int mmrm_sw_check_peak_current(struct mmrm_sw_clk_mgr_info *sinfo,
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry,
	u32 req_level, u32 clk_val)
{
	int rc = 0;
	struct mmrm_sw_peak_current_data *peak_data = &sinfo->peak_cur_data;
	u32 adj_level = req_level;
	u32 peak_cur = peak_data->aggreg_val;
	u32 old_cur = 0, new_cur = 0;
	int delta_cur;

	/* check the req level and adjust according to tbl entries */
	rc = mmrm_sw_check_req_level(sinfo, tbl_entry->clk_src_id, req_level, &adj_level);
	if (rc) {
		goto err_invalid_level;
	}

	/* recalculate aggregated current with adj level */
	if (adj_level != peak_data->aggreg_level) {
		rc = mmrm_sw_calculate_total_current(sinfo, adj_level, &peak_cur);
		if (rc) {
			goto err_invalid_level;
		}
	}

	/* calculate delta cur */
	if (tbl_entry->clk_rate) {
		old_cur = tbl_entry->current_ma[tbl_entry->vdd_level][adj_level];
	}

	if (clk_val) {
		new_cur = tbl_entry->current_ma[req_level][adj_level];
	}

	delta_cur = (signed)new_cur - old_cur;

	/* negative value, update peak data */
	if ((signed)peak_cur + delta_cur <= 0) {
		peak_data->aggreg_val = 0;
		peak_data->aggreg_level = adj_level;
		goto exit_no_err;
	}

	/* peak overshoot, do not update peak data */
	if ((signed)peak_cur + delta_cur >= peak_data->threshold) {
		/* Find low prority client and throttle it*/
		if ((tbl_entry->pri == MMRM_CLIENT_PRIOR_HIGH)
			&& (msm_mmrm_enable_throttle_feature > 0)) {
			rc = mmrm_sw_throttle_low_priority_client(sinfo, &delta_cur);
			if (rc != 0) {
				d_mpr_e("%s: Failed to throttle the low priority client\n",
						__func__);
				goto err_peak_overshoot;
			}
		} else {
			d_mpr_e("%s: Client csid(0x%x) name(%s) can't request throtlling\n",
				__func__, tbl_entry->clk_src_id, tbl_entry->name);
			rc = -EINVAL;
			goto err_peak_overshoot;
		}
	}
	/* update peak data */
	peak_data->aggreg_val = peak_cur + delta_cur;
	peak_data->aggreg_level = adj_level;

exit_no_err:
	d_mpr_h("%s: aggreg_val(%lu) aggreg_level(%lu)\n",
		__func__,
		peak_data->aggreg_val,
		peak_data->aggreg_level);
	return rc;

err_invalid_level:
err_peak_overshoot:
	return rc;
}

static int mmrm_sw_clk_client_setval(struct mmrm_clk_mgr *sw_clk_mgr,
	struct mmrm_client *client,
	struct mmrm_client_data *client_data,
	unsigned long clk_val)
{
	int rc = 0;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;
	struct mmrm_sw_clk_mgr_info *sinfo = &(sw_clk_mgr->data.sw_info);
	bool req_reserve;
	u32 req_level;

	d_mpr_h("%s: entering\n", __func__);

	/* validate input params */
	if (!client) {
		d_mpr_e("%s: invalid client\n");
		rc = -EINVAL;
		goto err_invalid_client;
	}

	if (client->client_uid >= sinfo->tot_clk_clients) {
		d_mpr_e("%s: invalid client uid (%d)\n",
			__func__, client->client_uid);
		rc = -EINVAL;
		goto err_invalid_client;
	}

	if (!client_data) {
		d_mpr_e("%s: invalid client data\n", __func__);
		rc = -EINVAL;
		goto err_invalid_client_data;
	}

	/* get table entry */
	tbl_entry = &sinfo->clk_client_tbl[client->client_uid];
	if (IS_ERR_OR_NULL(tbl_entry->clk)) {
		d_mpr_e("%s: clk src not registered\n");
		rc = -EINVAL;
		goto err_invalid_client;
	}

	/* Check if the requested clk rate is the same as the current clk rate.
	 * When clk rates are the same, compare this with the current state.
	 * Skip when duplicate calculations will be made.
	 * --- current ---- requested --- action ---
	 * a.  reserve  &&  req_reserve:  skip
	 * b. !reserve  && !req_reserve:  skip
	 * c. !reserve  &&  req_reserve:  skip
	 * d.  reserve  && !req_reserve:  set clk rate
	 */
	req_reserve = client_data->flags & MMRM_CLIENT_DATA_FLAG_RESERVE_ONLY;
	if (tbl_entry->clk_rate == clk_val) {
		d_mpr_h("%s: csid(0x%x) same as previous clk rate %llu\n",
			__func__, tbl_entry->clk_src_id, clk_val);

		/* a & b */
		if (tbl_entry->reserve == req_reserve)
			goto exit_no_err;

		/* c & d */
		mutex_lock(&sw_clk_mgr->lock);
		tbl_entry->reserve = req_reserve;
		mutex_unlock(&sw_clk_mgr->lock);

		/* skip or set clk rate */
		if (req_reserve)
			goto exit_no_err;
		else
			goto set_clk_rate;
	}

	/* get corresponding level */
	if (clk_val) {
		rc = mmrm_sw_get_req_level(tbl_entry, clk_val, &req_level);
		if (rc || req_level >= MMRM_VDD_LEVEL_MAX) {
			d_mpr_e("%s: csid(0x%x) unable to get level for clk rate %llu\n",
				__func__, tbl_entry->clk_src_id, clk_val);
			rc = -EINVAL;
			goto err_invalid_clk_val;
		}
	} else {
		req_level = 0;
	}

	mutex_lock(&sw_clk_mgr->lock);

	/* check and update for peak current */
	rc = mmrm_sw_check_peak_current(sinfo, tbl_entry, req_level, clk_val);
	if (rc) {
		d_mpr_e("%s: csid (0x%x) peak overshoot peak_cur(%lu)\n",
			__func__, tbl_entry->clk_src_id,
			sinfo->peak_cur_data.aggreg_val);
		mutex_unlock(&sw_clk_mgr->lock);
		goto err_peak_overshoot;
	}

	/* update table entry */
	tbl_entry->clk_rate = clk_val;
	tbl_entry->vdd_level = req_level;
	tbl_entry->reserve = req_reserve;

	mutex_unlock(&sw_clk_mgr->lock);

	/* check reserve only flag (skip set clock rate) */
	if (req_reserve) {
		d_mpr_h("%s: csid(0x%x) skip setting clk rate\n",
		__func__, tbl_entry->clk_src_id);
		rc = 0;
		goto exit_no_err;
	}

set_clk_rate:
	d_mpr_h("%s: csid(0x%x) setting clk rate %llu\n",
		__func__, tbl_entry->clk_src_id, clk_val);
	rc = clk_set_rate(tbl_entry->clk, clk_val);
	if (rc) {
		d_mpr_e("%s: csid(0x%x) failed to set clk rate %llu\n",
			__func__, tbl_entry->clk_src_id, clk_val);
		rc = -EINVAL;
		/* TBD: incase of failure clk_rate is invalid */
		goto err_clk_set_fail;
	}

exit_no_err:
	d_mpr_h("%s: exiting with success\n", __func__);
	return rc;

err_invalid_client:
err_invalid_client_data:
err_invalid_clk_val:
err_peak_overshoot:
err_clk_set_fail:
	d_mpr_h("%s: error exit\n", __func__);
	return rc;
}

static int mmrm_sw_clk_client_setval_inrange(struct mmrm_clk_mgr *sw_clk_mgr,
		struct mmrm_client *client,
		struct mmrm_client_data *client_data,
		struct mmrm_client_res_value *val)
{
	d_mpr_h("%s: entering\n", __func__);

	/* TBD: add support for set val in range */
	return mmrm_sw_clk_client_setval(sw_clk_mgr, client, client_data,
		val->cur);
}

static int mmrm_sw_clk_client_getval(struct mmrm_clk_mgr *sw_clk_mgr,
	struct mmrm_client *client,
	struct mmrm_client_res_value *val)
{
	int rc = 0;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;
	struct mmrm_sw_clk_mgr_info *sinfo = &(sw_clk_mgr->data.sw_info);

	d_mpr_h("%s: entering\n", __func__);

	/* validate input params */
	if (!client) {
		d_mpr_e("%s: invalid client\n");
		rc = -EINVAL;
		goto err_invalid_client;
	}

	if (client->client_uid >= sinfo->tot_clk_clients) {
		d_mpr_e("%s: invalid client uid (%d)\n",
			__func__, client->client_uid);
		rc = -EINVAL;
		goto err_invalid_client;
	}

	tbl_entry = &sinfo->clk_client_tbl[client->client_uid];
	if (!tbl_entry->clk) {
		d_mpr_e("%s: clk src not registered\n");
		rc = -EINVAL;
		goto err_invalid_client;
	}

	/* return previously configured value */
	/* TBD: Identify the min & max values */
	val->min = tbl_entry->clk_rate;
	val->cur = tbl_entry->clk_rate;
	val->max = tbl_entry->clk_rate;

	d_mpr_h("%s: exiting with success\n", __func__);
	return rc;

err_invalid_client:
	d_mpr_h("%s: error exit\n", __func__);
	return rc;
}

static struct mmrm_clk_mgr_client_ops clk_client_swops = {
	.clk_client_reg = mmrm_sw_clk_client_register,
	.clk_client_dereg = mmrm_sw_clk_client_deregister,
	.clk_client_setval = mmrm_sw_clk_client_setval,
	.clk_client_setval_inrange = mmrm_sw_clk_client_setval_inrange,
	.clk_client_getval = mmrm_sw_clk_client_getval,
};

static int mmrm_sw_prepare_table(struct mmrm_clk_platform_resources *cres,
	struct mmrm_sw_clk_mgr_info *sinfo)
{
	int rc = 0;
	u32 c;
	struct mmrm_sw_clk_client_tbl_entry *tbl_entry;
	struct nom_clk_src_info *nom_tbl_entry;

	d_mpr_h("%s: entering\n", __func__);

	/* read all resource entries */
	for (c = 0; c < sinfo->tot_clk_clients; c++) {
		tbl_entry = &sinfo->clk_client_tbl[c];
		nom_tbl_entry = &cres->nom_clk_set.clk_src_tbl[c];

		tbl_entry->clk_src_id = (nom_tbl_entry->domain << 16 |
			nom_tbl_entry->clk_src_id);
		tbl_entry->dyn_pwr[MMRM_VDD_LEVEL_NOM] =
			nom_tbl_entry->nom_dyn_pwr;
		tbl_entry->leak_pwr[MMRM_VDD_LEVEL_NOM] =
			nom_tbl_entry->nom_leak_pwr;

		d_mpr_h("%s: updating csid(0x%x) dyn_pwr(%d) leak_pwr(%d)\n",
			__func__,
			tbl_entry->clk_src_id,
			tbl_entry->dyn_pwr[MMRM_VDD_LEVEL_NOM],
			tbl_entry->leak_pwr[MMRM_VDD_LEVEL_NOM]);
	}

	d_mpr_h("%s: exiting\n", __func__);
	return rc;
}

int mmrm_init_sw_clk_mgr(void *driver_data)
{
	int rc = 0;
	struct mmrm_driver_data *drv_data =
		(struct mmrm_driver_data *)driver_data;
	struct mmrm_clk_platform_resources *cres = &drv_data->clk_res;
	struct mmrm_sw_clk_mgr_info *sinfo = NULL;
	struct mmrm_clk_mgr *sw_clk_mgr = NULL;
	u32 tbl_size = 0;

	d_mpr_h("%s: entering\n", __func__);

	/* mmrm_sw_clk_mgr */
	sw_clk_mgr = kzalloc(sizeof(*sw_clk_mgr), GFP_KERNEL);
	if (!sw_clk_mgr) {
		d_mpr_e("%s: failed to allocate memory for sw_clk_mgr\n",
			__func__);
		rc = -ENOMEM;
		goto err_fail_sw_clk_mgr;
	}

	/* initialize the tables */
	tbl_size = sizeof(struct mmrm_sw_clk_client_tbl_entry) *
		cres->nom_clk_set.count;

	sinfo = &(sw_clk_mgr->data.sw_info);
	sinfo->driver_data = drv_data;
	sinfo->clk_client_tbl = kzalloc(tbl_size, GFP_KERNEL);
	if (!sinfo->clk_client_tbl) {
		d_mpr_e(
			"%s: failed to allocate memory for clk_client_tbl (%d)\n",
			__func__, cres->nom_clk_set.count);
		rc = -ENOMEM;
		goto err_fail_clk_tbl;
	}
	sinfo->tot_clk_clients = cres->nom_clk_set.count;
	sinfo->enabled_clk_clients = 0;

	/* prepare table entries */
	rc = mmrm_sw_prepare_table(cres, sinfo);
	if (rc) {
		d_mpr_e("%s: failed to prepare clk table\n", __func__);
		rc = -ENOMEM;
		goto err_fail_prep_tbl;
	}

	/* update the peak current threshold */
	sinfo->peak_cur_data.threshold = cres->threshold;
	sinfo->peak_cur_data.aggreg_val = 0;
	sinfo->peak_cur_data.aggreg_level = MMRM_VDD_LEVEL_SVS_L1;

	/* initialize mutex for sw clk mgr */
	mutex_init(&sw_clk_mgr->lock);
	sw_clk_mgr->scheme = drv_data->clk_res.scheme;

	/* clk client operations */
	sw_clk_mgr->clk_client_ops = &clk_client_swops;
	drv_data->clk_mgr = sw_clk_mgr;

	d_mpr_h("%s: exiting with success\n", __func__);
	return rc;

err_fail_prep_tbl:
	kfree(sinfo->clk_client_tbl);
err_fail_clk_tbl:
	kfree(sw_clk_mgr);
	drv_data->clk_mgr = NULL;
err_fail_sw_clk_mgr:
	d_mpr_h("%s: error exit\n", __func__);
	return rc;
}

int mmrm_destroy_sw_clk_mgr(struct mmrm_clk_mgr *sw_clk_mgr)
{
	int rc = 0;

	if (!sw_clk_mgr) {
		d_mpr_e("%s: sw_clk_mgr null\n", __func__);
		return -EINVAL;
	}

	kfree(sw_clk_mgr->data.sw_info.clk_client_tbl);
	mutex_destroy(&sw_clk_mgr->lock);
	kfree(sw_clk_mgr);

	return rc;
}
