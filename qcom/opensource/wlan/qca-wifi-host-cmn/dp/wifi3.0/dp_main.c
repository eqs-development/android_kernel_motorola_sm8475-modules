/*
 * Copyright (c) 2016-2017 The Linux Foundation. All rights reserved.
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

#include <qdf_types.h>
#include <qdf_lock.h>
#include <hal_api.h>
#include <hif.h>
#include <htt.h>
#include <wdi_event.h>
#include <queue.h>
#include "dp_htt.h"
#include "dp_types.h"
#include "dp_internal.h"
#include "dp_tx.h"
#include "dp_rx.h"
#include "../../wlan_cfg/wlan_cfg.h"

#define DP_INTR_POLL_TIMER_MS	100
/**
 * dp_setup_srng - Internal function to setup SRNG rings used by data path
 */
static int dp_srng_setup(struct dp_soc *soc, struct dp_srng *srng,
	int ring_type, int ring_num, int pdev_id, uint32_t num_entries)
{
	void *hal_soc = soc->hal_soc;
	uint32_t entry_size = hal_srng_get_entrysize(hal_soc, ring_type);
	/* TODO: See if we should get align size from hal */
	uint32_t ring_base_align = 8;
	struct hal_srng_params ring_params;


	srng->hal_srng = NULL;
	srng->alloc_size = (num_entries * entry_size) + ring_base_align - 1;
	srng->base_vaddr_unaligned = qdf_mem_alloc_consistent(
		soc->osdev, soc->osdev->dev, srng->alloc_size,
		&(srng->base_paddr_unaligned));

	if (!srng->base_vaddr_unaligned) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("alloc failed - ring_type: %d, ring_num %d"),
			ring_type, ring_num);
		return QDF_STATUS_E_NOMEM;
	}

	ring_params.ring_base_vaddr = srng->base_vaddr_unaligned +
		((unsigned long)srng->base_vaddr_unaligned % ring_base_align);
	ring_params.ring_base_paddr = srng->base_paddr_unaligned +
		((unsigned long)(ring_params.ring_base_vaddr) -
		(unsigned long)srng->base_vaddr_unaligned);
	ring_params.num_entries = num_entries;

	/* TODO: Check MSI support and get MSI settings from HIF layer */
	ring_params.msi_data = 0;
	ring_params.msi_addr = 0;

	/* TODO: Setup interrupt timer and batch counter thresholds for
	 * interrupt mitigation based on ring type
	 */
	ring_params.intr_timer_thres_us = 8;
	ring_params.intr_batch_cntr_thres_entries = 1;

	/* TODO: Currently hal layer takes care of endianness related settings.
	 * See if these settings need to passed from DP layer
	 */
	ring_params.flags = 0;

	/* Enable low threshold interrupts for rx buffer rings (regular and
	 * monitor buffer rings.
	 * TODO: See if this is required for any other ring
	 */
	if ((ring_type == RXDMA_BUF) || (ring_type == RXDMA_MONITOR_BUF)) {
		/* TODO: Setting low threshold to 1/8th of ring size
		 * see if this needs to be configurable
		 */
		ring_params.low_threshold = num_entries >> 3;
		ring_params.flags |= HAL_SRNG_LOW_THRES_INTR_ENABLE;
	}

	srng->hal_srng = hal_srng_setup(hal_soc, ring_type, ring_num,
		pdev_id, &ring_params);
	return 0;
}

/**
 * dp_srng_cleanup - Internal function to cleanup SRNG rings used by data path
 * Any buffers allocated and attached to ring entries are expected to be freed
 * before calling this function.
 */
static void dp_srng_cleanup(struct dp_soc *soc, struct dp_srng *srng,
	int ring_type, int ring_num)
{
	if (!srng->hal_srng) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("Ring type: %d, num:%d not setup"),
			ring_type, ring_num);
		return;
	}

	hal_srng_cleanup(soc->hal_soc, srng->hal_srng);

	qdf_mem_free_consistent(soc->osdev, soc->osdev->dev,
				srng->alloc_size,
				srng->base_vaddr_unaligned,
				srng->base_paddr_unaligned, 0);
}

/* TODO: Need this interface from HIF */
void *hif_get_hal_handle(void *hif_handle);

/*
 * dp_service_srngs() - Top level interrupt handler for DP Ring interrupts
 * @dp_ctx: DP SOC handle
 * @budget: Number of frames/descriptors that can be processed in one shot
 *
 * Return: remaining budget/quota for the soc device
 */
static uint32_t dp_service_srngs(void *dp_ctx, uint32_t dp_budget)
{
	struct dp_intr *int_ctx = (struct dp_intr *)dp_ctx;
	struct dp_soc *soc = int_ctx->soc;
	int ring = 0;
	uint32_t work_done  = 0;
	uint32_t budget = dp_budget;
	uint8_t tx_mask = int_ctx->tx_ring_mask;
	uint8_t rx_mask = int_ctx->rx_ring_mask;
	uint8_t rx_err_mask = int_ctx->rx_err_ring_mask;
	uint8_t rx_wbm_rel_mask = int_ctx->rx_wbm_rel_ring_mask;
	uint8_t reo_status_mask = int_ctx->reo_status_ring_mask;

	/* Process Tx completion interrupts first to return back buffers */
	if (tx_mask) {
		for (ring = 0; ring < soc->num_tcl_data_rings; ring++) {
			if (tx_mask & (1 << ring)) {
				work_done =
					dp_tx_comp_handler(soc, ring, budget);
				budget -= work_done;
				if (work_done)
					QDF_TRACE(QDF_MODULE_ID_DP,
						QDF_TRACE_LEVEL_INFO,
						"tx mask 0x%x ring %d,"
						"budget %d",
						tx_mask, ring, budget);
				if (budget <= 0)
					goto budget_done;
			}
		}
	}

	/* Process REO Exception ring interrupt */
	if (rx_err_mask) {
		work_done = dp_rx_err_process(soc,
				soc->reo_exception_ring.hal_srng, budget);
		budget -=  work_done;

		if (work_done)
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO,
				"REO Exception Ring: work_done %d budget %d",
				work_done, budget);
		if (budget <= 0) {
			goto budget_done;
		}
	}

	/* Process Rx WBM release ring interrupt */
	if (rx_wbm_rel_mask) {
		work_done = dp_rx_wbm_err_process(soc,
				soc->rx_rel_ring.hal_srng, budget);
		budget -=  work_done;

		if (work_done)
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO,
				"WBM Release Ring: work_done %d budget %d",
				work_done, budget);
		if (budget <= 0) {
			goto budget_done;
		}
	}

	/* Process Rx interrupts */
	if (rx_mask) {
		for (ring = 0; ring < soc->num_reo_dest_rings; ring++) {
			if (rx_mask & (1 << ring)) {
				work_done =
					dp_rx_process(soc,
					    soc->reo_dest_ring[ring].hal_srng,
					    budget);
				budget -=  work_done;
				if (work_done)
					QDF_TRACE(QDF_MODULE_ID_DP,
						QDF_TRACE_LEVEL_INFO,
						"rx mask 0x%x ring %d,"
						"budget %d",
						tx_mask, ring, budget);
				if (budget <= 0)
					goto budget_done;
			}
		}
	}

	if (reo_status_mask)
		dp_reo_status_ring_handler(soc);

budget_done:
	return dp_budget - budget;
}

/* dp_interrupt_timer()- timer poll for interrupts
 *
 * @arg: SoC Handle
 *
 * Return:
 *
 */
#ifdef DP_INTR_POLL_BASED
static void dp_interrupt_timer(void *arg)
{
	struct dp_soc *soc = (struct dp_soc *) arg;
	int i;

	for (i = 0 ; i < wlan_cfg_get_num_contexts(soc->wlan_cfg_ctx); i++)
		dp_service_srngs(&soc->intr_ctx[i], 0xffff);

	qdf_timer_mod(&soc->int_timer, DP_INTR_POLL_TIMER_MS);
}

/*
 * dp_soc_interrupt_attach() - Register handlers for DP interrupts
 * @txrx_soc: DP SOC handle
 *
 * Host driver will register for “DP_NUM_INTERRUPT_CONTEXTS” number of NAPI
 * contexts. Each NAPI context will have a tx_ring_mask , rx_ring_mask ,and
 * rx_monitor_ring mask to indicate the rings that are processed by the handler.
 *
 * Return: 0 for success. nonzero for failure.
 */
static QDF_STATUS dp_soc_interrupt_attach(void *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;
	int i;

	for (i = 0; i < wlan_cfg_get_num_contexts(soc->wlan_cfg_ctx); i++) {
		soc->intr_ctx[i].tx_ring_mask = 0xF;
		soc->intr_ctx[i].rx_ring_mask = 0xF;
		soc->intr_ctx[i].rx_mon_ring_mask = 0xF;
		soc->intr_ctx[i].rx_err_ring_mask = 0x1;
		soc->intr_ctx[i].rx_wbm_rel_ring_mask = 0x1;
		soc->intr_ctx[i].reo_status_ring_mask = 0x1;
		soc->intr_ctx[i].soc = soc;
	}

	qdf_timer_init(soc->osdev, &soc->int_timer,
			dp_interrupt_timer, (void *)soc,
			QDF_TIMER_TYPE_WAKE_APPS);

	return QDF_STATUS_SUCCESS;
}

/*
 * dp_soc_interrupt_detach() - Deregister any allocations done for interrupts
 * @txrx_soc: DP SOC handle
 *
 * Return: void
 */
static void dp_soc_interrupt_detach(void *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;

	qdf_timer_stop(&soc->int_timer);

	qdf_timer_free(&soc->int_timer);
}
#else
/*
 * dp_soc_interrupt_attach() - Register handlers for DP interrupts
 * @txrx_soc: DP SOC handle
 *
 * Host driver will register for “DP_NUM_INTERRUPT_CONTEXTS” number of NAPI
 * contexts. Each NAPI context will have a tx_ring_mask , rx_ring_mask ,and
 * rx_monitor_ring mask to indicate the rings that are processed by the handler.
 *
 * Return: 0 for success. nonzero for failure.
 */
static QDF_STATUS dp_soc_interrupt_attach(void *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;

	int i = 0;
	int num_irq = 0;


	for (i = 0; i < wlan_cfg_get_num_contexts(soc->wlan_cfg_ctx); i++) {
		int j = 0;
		int ret = 0;

		/* Map of IRQ ids registered with one interrupt context */
		int irq_id_map[HIF_MAX_GRP_IRQ];

		int tx_mask =
			wlan_cfg_get_tx_ring_mask(soc->wlan_cfg_ctx, i);
		int rx_mask =
			wlan_cfg_get_rx_ring_mask(soc->wlan_cfg_ctx, i);
		int rx_mon_mask =
			wlan_cfg_get_rx_mon_ring_mask(soc->wlan_cfg_ctx, i);

		soc->intr_ctx[i].tx_ring_mask = tx_mask;
		soc->intr_ctx[i].rx_ring_mask = rx_mask;
		soc->intr_ctx[i].rx_mon_ring_mask = rx_mon_mask;
		soc->intr_ctx[i].soc = soc;

		num_irq = 0;

		for (j = 0; j < HIF_MAX_GRP_IRQ; j++) {

			if (tx_mask & (1 << j)) {
				irq_id_map[num_irq++] =
					(wbm2host_tx_completions_ring1 - j);
			}

			if (rx_mask & (1 << j)) {
				irq_id_map[num_irq++] =
					(reo2host_destination_ring1 - j);
			}

			if (rx_mon_mask & (1 << j)) {
				irq_id_map[num_irq++] =
					(rxdma2host_monitor_destination_mac1
					 - j);
			}
		}


		ret = hif_register_ext_group_int_handler(soc->hif_handle,
				num_irq, irq_id_map,
				dp_service_srngs,
				&soc->intr_ctx[i]);

		if (ret) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("failed, ret = %d"), ret);

			return QDF_STATUS_E_FAILURE;
		}
	}

	return QDF_STATUS_SUCCESS;
}

/*
 * dp_soc_interrupt_detach() - Deregister any allocations done for interrupts
 * @txrx_soc: DP SOC handle
 *
 * Return: void
 */
static void dp_soc_interrupt_detach(void *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;
	int i;

	for (i = 0; i < wlan_cfg_get_num_contexts(soc->wlan_cfg_ctx); i++) {
		soc->intr_ctx[i].tx_ring_mask = 0;
		soc->intr_ctx[i].rx_ring_mask = 0;
		soc->intr_ctx[i].rx_mon_ring_mask = 0;
	}
}
#endif

#define AVG_MAX_MPDUS_PER_TID 128
#define AVG_TIDS_PER_CLIENT 2
#define AVG_FLOWS_PER_TID 2
#define AVG_MSDUS_PER_FLOW 128
#define AVG_MSDUS_PER_MPDU 4

/*
 * Allocate and setup link descriptor pool that will be used by HW for
 * various link and queue descriptors and managed by WBM
 */
static int dp_hw_link_desc_pool_setup(struct dp_soc *soc)
{
	int link_desc_size = hal_get_link_desc_size(soc->hal_soc);
	int link_desc_align = hal_get_link_desc_align(soc->hal_soc);
	uint32_t max_clients = wlan_cfg_get_max_clients(soc->wlan_cfg_ctx);
	uint32_t num_mpdus_per_link_desc =
		hal_num_mpdus_per_link_desc(soc->hal_soc);
	uint32_t num_msdus_per_link_desc =
		hal_num_msdus_per_link_desc(soc->hal_soc);
	uint32_t num_mpdu_links_per_queue_desc =
		hal_num_mpdu_links_per_queue_desc(soc->hal_soc);
	uint32_t max_alloc_size = wlan_cfg_max_alloc_size(soc->wlan_cfg_ctx);
	uint32_t total_link_descs, total_mem_size;
	uint32_t num_mpdu_link_descs, num_mpdu_queue_descs;
	uint32_t num_tx_msdu_link_descs, num_rx_msdu_link_descs;
	uint32_t num_link_desc_banks;
	uint32_t last_bank_size = 0;
	uint32_t entry_size, num_entries;
	int i;

	/* Only Tx queue descriptors are allocated from common link descriptor
	 * pool Rx queue descriptors are not included in this because (REO queue
	 * extension descriptors) they are expected to be allocated contiguously
	 * with REO queue descriptors
	 */
	num_mpdu_link_descs = (max_clients * AVG_TIDS_PER_CLIENT *
		AVG_MAX_MPDUS_PER_TID) / num_mpdus_per_link_desc;

	num_mpdu_queue_descs = num_mpdu_link_descs /
		num_mpdu_links_per_queue_desc;

	num_tx_msdu_link_descs = (max_clients * AVG_TIDS_PER_CLIENT *
		AVG_FLOWS_PER_TID * AVG_MSDUS_PER_FLOW) /
		num_msdus_per_link_desc;

	num_rx_msdu_link_descs = (max_clients * AVG_TIDS_PER_CLIENT *
		AVG_MAX_MPDUS_PER_TID * AVG_MSDUS_PER_MPDU) / 6;

	num_entries = num_mpdu_link_descs + num_mpdu_queue_descs +
		num_tx_msdu_link_descs + num_rx_msdu_link_descs;

	/* Round up to power of 2 */
	total_link_descs = 1;
	while (total_link_descs < num_entries)
		total_link_descs <<= 1;

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
		FL("total_link_descs: %u, link_desc_size: %d"),
		total_link_descs, link_desc_size);
	total_mem_size =  total_link_descs * link_desc_size;

	total_mem_size += link_desc_align;

	if (total_mem_size <= max_alloc_size) {
		num_link_desc_banks = 0;
		last_bank_size = total_mem_size;
	} else {
		num_link_desc_banks = (total_mem_size) /
			(max_alloc_size - link_desc_align);
		last_bank_size = total_mem_size %
			(max_alloc_size - link_desc_align);
	}

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
		FL("total_mem_size: %d, num_link_desc_banks: %u"),
		total_mem_size, num_link_desc_banks);

	for (i = 0; i < num_link_desc_banks; i++) {
		soc->link_desc_banks[i].base_vaddr_unaligned =
			qdf_mem_alloc_consistent(soc->osdev, soc->osdev->dev,
			max_alloc_size,
			&(soc->link_desc_banks[i].base_paddr_unaligned));
		soc->link_desc_banks[i].size = max_alloc_size;

		soc->link_desc_banks[i].base_vaddr = (void *)((unsigned long)(
			soc->link_desc_banks[i].base_vaddr_unaligned) +
			((unsigned long)(
			soc->link_desc_banks[i].base_vaddr_unaligned) %
			link_desc_align));

		soc->link_desc_banks[i].base_paddr = (unsigned long)(
			soc->link_desc_banks[i].base_paddr_unaligned) +
			((unsigned long)(soc->link_desc_banks[i].base_vaddr) -
			(unsigned long)(
			soc->link_desc_banks[i].base_vaddr_unaligned));

		if (!soc->link_desc_banks[i].base_vaddr_unaligned) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("Link descriptor memory alloc failed"));
			goto fail;
		}
	}

	if (last_bank_size) {
		/* Allocate last bank in case total memory required is not exact
		 * multiple of max_alloc_size
		 */
		soc->link_desc_banks[i].base_vaddr_unaligned =
			qdf_mem_alloc_consistent(soc->osdev, soc->osdev->dev,
			last_bank_size,
			&(soc->link_desc_banks[i].base_paddr_unaligned));
		soc->link_desc_banks[i].size = last_bank_size;

		soc->link_desc_banks[i].base_vaddr = (void *)((unsigned long)
			(soc->link_desc_banks[i].base_vaddr_unaligned) +
			((unsigned long)(
			soc->link_desc_banks[i].base_vaddr_unaligned) %
			link_desc_align));

		soc->link_desc_banks[i].base_paddr =
			(unsigned long)(
			soc->link_desc_banks[i].base_paddr_unaligned) +
			((unsigned long)(soc->link_desc_banks[i].base_vaddr) -
			(unsigned long)(
			soc->link_desc_banks[i].base_vaddr_unaligned));
	}


	/* Allocate and setup link descriptor idle list for HW internal use */
	entry_size = hal_srng_get_entrysize(soc->hal_soc, WBM_IDLE_LINK);
	total_mem_size = entry_size * total_link_descs;

	if (total_mem_size <= max_alloc_size) {
		void *desc;

		if (dp_srng_setup(soc, &soc->wbm_idle_link_ring,
			WBM_IDLE_LINK, 0, 0, total_link_descs)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("Link desc idle ring setup failed"));
			goto fail;
		}

		hal_srng_access_start_unlocked(soc->hal_soc,
			soc->wbm_idle_link_ring.hal_srng);

		for (i = 0; i < MAX_LINK_DESC_BANKS &&
			soc->link_desc_banks[i].base_paddr; i++) {
			uint32_t num_entries = (soc->link_desc_banks[i].size -
				(unsigned long)(
				soc->link_desc_banks[i].base_vaddr) -
				(unsigned long)(
				soc->link_desc_banks[i].base_vaddr_unaligned))
				/ link_desc_size;
			unsigned long paddr = (unsigned long)(
				soc->link_desc_banks[i].base_paddr);

			while (num_entries && (desc = hal_srng_src_get_next(
				soc->hal_soc,
				soc->wbm_idle_link_ring.hal_srng))) {
				hal_set_link_desc_addr(desc, i, paddr);
				num_entries--;
				paddr += link_desc_size;
			}
		}
		hal_srng_access_end_unlocked(soc->hal_soc,
			soc->wbm_idle_link_ring.hal_srng);
	} else {
		uint32_t num_scatter_bufs;
		uint32_t num_entries_per_buf;
		uint32_t rem_entries;
		uint8_t *scatter_buf_ptr;
		uint16_t scatter_buf_num;

		soc->wbm_idle_scatter_buf_size =
			hal_idle_list_scatter_buf_size(soc->hal_soc);
		num_entries_per_buf = hal_idle_scatter_buf_num_entries(
			soc->hal_soc, soc->wbm_idle_scatter_buf_size);
		num_scatter_bufs = (total_mem_size /
			soc->wbm_idle_scatter_buf_size) + (total_mem_size %
				soc->wbm_idle_scatter_buf_size) ? 1 : 0;

		for (i = 0; i < num_scatter_bufs; i++) {
			soc->wbm_idle_scatter_buf_base_vaddr[i] =
				qdf_mem_alloc_consistent(soc->osdev, soc->osdev->dev,
				soc->wbm_idle_scatter_buf_size,
				&(soc->wbm_idle_scatter_buf_base_paddr[i]));
			if (soc->wbm_idle_scatter_buf_base_vaddr[i] == NULL) {
				QDF_TRACE(QDF_MODULE_ID_DP,
					QDF_TRACE_LEVEL_ERROR,
					FL("Scatter list memory alloc failed"));
				goto fail;
			}
		}

		/* Populate idle list scatter buffers with link descriptor
		 * pointers
		 */
		scatter_buf_num = 0;
		scatter_buf_ptr = (uint8_t *)(
			soc->wbm_idle_scatter_buf_base_vaddr[scatter_buf_num]);
		rem_entries = num_entries_per_buf;

		for (i = 0; i < MAX_LINK_DESC_BANKS &&
			soc->link_desc_banks[i].base_paddr; i++) {
			uint32_t num_link_descs =
				(soc->link_desc_banks[i].size -
				(unsigned long)(
				soc->link_desc_banks[i].base_vaddr) -
				(unsigned long)(
				soc->link_desc_banks[i].base_vaddr_unaligned)) /
				link_desc_size;
			unsigned long paddr = (unsigned long)(
				soc->link_desc_banks[i].base_paddr);
			void *desc = NULL;

			while (num_link_descs && (desc =
				hal_srng_src_get_next(soc->hal_soc,
				soc->wbm_idle_link_ring.hal_srng))) {
				hal_set_link_desc_addr((void *)scatter_buf_ptr,
					i, paddr);
				num_link_descs--;
				paddr += link_desc_size;
				if (rem_entries) {
					rem_entries--;
					scatter_buf_ptr += link_desc_size;
				} else {
					rem_entries = num_entries_per_buf;
					scatter_buf_num++;
					scatter_buf_ptr = (uint8_t *)(
						soc->wbm_idle_scatter_buf_base_vaddr[
						scatter_buf_num]);
				}
			}
		}
		/* Setup link descriptor idle list in HW */
		hal_setup_link_idle_list(soc->hal_soc,
			soc->wbm_idle_scatter_buf_base_paddr,
			soc->wbm_idle_scatter_buf_base_vaddr,
			num_scatter_bufs, soc->wbm_idle_scatter_buf_size,
			(uint32_t)(scatter_buf_ptr -
					(uint8_t *)(soc->wbm_idle_scatter_buf_base_vaddr[
			scatter_buf_num])));
	}
	return 0;

fail:
	if (soc->wbm_idle_link_ring.hal_srng) {
		dp_srng_cleanup(soc->hal_soc, &soc->wbm_idle_link_ring,
			WBM_IDLE_LINK, 0);
	}

	for (i = 0; i < MAX_IDLE_SCATTER_BUFS; i++) {
		if (soc->wbm_idle_scatter_buf_base_vaddr[i]) {
			qdf_mem_free_consistent(soc->osdev, soc->osdev->dev,
				soc->wbm_idle_scatter_buf_size,
				soc->wbm_idle_scatter_buf_base_vaddr[i],
				soc->wbm_idle_scatter_buf_base_paddr[i], 0);
		}
	}

	for (i = 0; i < MAX_LINK_DESC_BANKS; i++) {
		if (soc->link_desc_banks[i].base_vaddr_unaligned) {
			qdf_mem_free_consistent(soc->osdev, soc->osdev->dev,
				soc->link_desc_banks[i].size,
				soc->link_desc_banks[i].base_vaddr_unaligned,
				soc->link_desc_banks[i].base_paddr_unaligned,
				0);
		}
	}
	return QDF_STATUS_E_FAILURE;
}

#ifdef notused
/*
 * Free link descriptor pool that was setup HW
 */
static void dp_hw_link_desc_pool_cleanup(struct dp_soc *soc)
{
	int i;

	if (soc->wbm_idle_link_ring.hal_srng) {
		dp_srng_cleanup(soc->hal_soc, &soc->wbm_idle_link_ring,
			WBM_IDLE_LINK, 0);
	}

	for (i = 0; i < MAX_IDLE_SCATTER_BUFS; i++) {
		if (soc->wbm_idle_scatter_buf_base_vaddr[i]) {
			qdf_mem_free_consistent(soc->osdev, soc->osdev->dev,
				soc->wbm_idle_scatter_buf_size,
				soc->wbm_idle_scatter_buf_base_vaddr[i],
				soc->wbm_idle_scatter_buf_base_paddr[i], 0);
		}
	}

	for (i = 0; i < MAX_LINK_DESC_BANKS; i++) {
		if (soc->link_desc_banks[i].base_vaddr_unaligned) {
			qdf_mem_free_consistent(soc->osdev, soc->osdev->dev,
				soc->link_desc_banks[i].size,
				soc->link_desc_banks[i].base_vaddr_unaligned,
				soc->link_desc_banks[i].base_paddr_unaligned,
				0);
		}
	}
}
#endif /* notused */

/* TODO: Following should be configurable */
#define WBM_RELEASE_RING_SIZE 64
#define TCL_DATA_RING_SIZE 512
#define TCL_CMD_RING_SIZE 32
#define TCL_STATUS_RING_SIZE 32
#define REO_DST_RING_SIZE 2048
#define REO_REINJECT_RING_SIZE 32
#define RX_RELEASE_RING_SIZE 256
#define REO_EXCEPTION_RING_SIZE 128
#define REO_CMD_RING_SIZE 32
#define REO_STATUS_RING_SIZE 32
#define RXDMA_BUF_RING_SIZE 2048
#define RXDMA_MONITOR_BUF_RING_SIZE 2048
#define RXDMA_MONITOR_DST_RING_SIZE 2048
#define RXDMA_MONITOR_STATUS_RING_SIZE 2048

/*
 * dp_soc_cmn_setup() - Common SoC level initializion
 * @soc:		Datapath SOC handle
 *
 * This is an internal function used to setup common SOC data structures,
 * to be called from PDEV attach after receiving HW mode capabilities from FW
 */
static int dp_soc_cmn_setup(struct dp_soc *soc)
{
	int i;

	if (soc->cmn_init_done)
		return 0;

	if (dp_peer_find_attach(soc))
		goto fail0;

	if (dp_hw_link_desc_pool_setup(soc))
		goto fail1;

	/* Setup SRNG rings */
	/* Common rings */
	if (dp_srng_setup(soc, &soc->wbm_desc_rel_ring, SW2WBM_RELEASE, 0, 0,
		WBM_RELEASE_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for wbm_desc_rel_ring"));
		goto fail1;
	}


	soc->num_tcl_data_rings = 0;
	/* Tx data rings */
	if (!wlan_cfg_per_pdev_tx_ring(soc->wlan_cfg_ctx)) {
		soc->num_tcl_data_rings =
			wlan_cfg_num_tcl_data_rings(soc->wlan_cfg_ctx);
		for (i = 0; i < soc->num_tcl_data_rings; i++) {
			if (dp_srng_setup(soc, &soc->tcl_data_ring[i],
				TCL_DATA, i, 0, TCL_DATA_RING_SIZE)) {
				QDF_TRACE(QDF_MODULE_ID_DP,
					QDF_TRACE_LEVEL_ERROR,
					FL("dp_srng_setup failed for tcl_data_ring[%d]"), i);
				goto fail1;
			}
			if (dp_srng_setup(soc, &soc->tx_comp_ring[i],
				WBM2SW_RELEASE, i, 0, TCL_DATA_RING_SIZE)) {
				QDF_TRACE(QDF_MODULE_ID_DP,
					QDF_TRACE_LEVEL_ERROR,
					FL("dp_srng_setup failed for tx_comp_ring[%d]"), i);
				goto fail1;
			}
		}
	} else {
		/* This will be incremented during per pdev ring setup */
		soc->num_tcl_data_rings = 0;
	}

	if (dp_tx_soc_attach(soc)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("dp_tx_soc_attach failed"));
		goto fail1;
	}

	/* TCL command and status rings */
	if (dp_srng_setup(soc, &soc->tcl_cmd_ring, TCL_CMD, 0, 0,
		TCL_CMD_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for tcl_cmd_ring"));
		goto fail1;
	}

	if (dp_srng_setup(soc, &soc->tcl_status_ring, TCL_STATUS, 0, 0,
		TCL_STATUS_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for tcl_status_ring"));
		goto fail1;
	}


	/* TBD: call dp_tx_init to setup Tx SW descriptors and MSDU extension
	 * descriptors
	 */

	/* Rx data rings */
	if (!wlan_cfg_per_pdev_rx_ring(soc->wlan_cfg_ctx)) {
		soc->num_reo_dest_rings =
			wlan_cfg_num_reo_dest_rings(soc->wlan_cfg_ctx);
		for (i = 0; i < soc->num_reo_dest_rings; i++) {
			if (dp_srng_setup(soc, &soc->reo_dest_ring[i], REO_DST,
				i, 0, REO_DST_RING_SIZE)) {
				QDF_TRACE(QDF_MODULE_ID_DP,
					QDF_TRACE_LEVEL_ERROR,
					FL("dp_srng_setup failed for reo_dest_ring[%d]"), i);
				goto fail1;
			}
		}
	} else {
		/* This will be incremented during per pdev ring setup */
		soc->num_reo_dest_rings = 0;
	}

	/* TBD: call dp_rx_init to setup Rx SW descriptors */

	/* REO reinjection ring */
	if (dp_srng_setup(soc, &soc->reo_reinject_ring, REO_REINJECT, 0, 0,
		REO_REINJECT_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for reo_reinject_ring"));
		goto fail1;
	}


	/* Rx release ring */
	if (dp_srng_setup(soc, &soc->rx_rel_ring, WBM2SW_RELEASE, 3, 0,
		RX_RELEASE_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for rx_rel_ring"));
		goto fail1;
	}


	/* Rx exception ring */
	if (dp_srng_setup(soc, &soc->reo_exception_ring, REO_EXCEPTION, 0,
		MAX_REO_DEST_RINGS, REO_EXCEPTION_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for reo_exception_ring"));
		goto fail1;
	}


	/* REO command and status rings */
	if (dp_srng_setup(soc, &soc->reo_cmd_ring, REO_CMD, 0, 0,
		REO_CMD_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for reo_cmd_ring"));
		goto fail1;
	}

	hal_reo_init_cmd_ring(soc->hal_soc, soc->reo_cmd_ring.hal_srng);
	TAILQ_INIT(&soc->rx.reo_cmd_list);
	qdf_spinlock_create(&soc->rx.reo_cmd_lock);

	if (dp_srng_setup(soc, &soc->reo_status_ring, REO_STATUS, 0, 0,
		REO_STATUS_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for reo_status_ring"));
		goto fail1;
	}

	dp_soc_interrupt_attach(soc);
	/* Setup HW REO */
	hal_reo_setup(soc->hal_soc);

	soc->cmn_init_done = 1;
	return 0;
fail1:
	/*
	 * Cleanup will be done as part of soc_detach, which will
	 * be called on pdev attach failure
	 */
fail0:
	return QDF_STATUS_E_FAILURE;
}

static void dp_pdev_detach_wifi3(void *txrx_pdev, int force);

/*
* dp_pdev_attach_wifi3() - attach txrx pdev
* @osif_pdev: Opaque PDEV handle from OSIF/HDD
* @txrx_soc: Datapath SOC handle
* @htc_handle: HTC handle for host-target interface
* @qdf_osdev: QDF OS device
* @pdev_id: PDEV ID
*
* Return: DP PDEV handle on success, NULL on failure
*/
static void *dp_pdev_attach_wifi3(struct cdp_soc_t *txrx_soc, void *ctrl_pdev,
	HTC_HANDLE htc_handle, qdf_device_t qdf_osdev, uint8_t pdev_id)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;
	struct dp_pdev *pdev = qdf_mem_malloc(sizeof(*pdev));

	if (!pdev) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("DP PDEV memory allocation failed"));
		goto fail0;
	}

	pdev->wlan_cfg_ctx = wlan_cfg_pdev_attach();

	if (!pdev->wlan_cfg_ctx) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("pdev cfg_attach failed"));

		qdf_mem_free(pdev);
		goto fail0;
	}

	pdev->soc = soc;
	pdev->osif_pdev = ctrl_pdev;
	pdev->pdev_id = pdev_id;
	soc->pdev_list[pdev_id] = pdev;

	TAILQ_INIT(&pdev->vdev_list);
	pdev->vdev_count = 0;

	if (dp_soc_cmn_setup(soc)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_soc_cmn_setup failed"));
		goto fail1;
	}

	/* Setup per PDEV TCL rings if configured */
	if (wlan_cfg_per_pdev_tx_ring(soc->wlan_cfg_ctx)) {
		if (dp_srng_setup(soc, &soc->tcl_data_ring[pdev_id], TCL_DATA,
			pdev_id, pdev_id, TCL_DATA_RING_SIZE)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("dp_srng_setup failed for tcl_data_ring"));
			goto fail1;
		}
		if (dp_srng_setup(soc, &soc->tx_comp_ring[pdev_id],
			WBM2SW_RELEASE, pdev_id, pdev_id, TCL_DATA_RING_SIZE)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("dp_srng_setup failed for tx_comp_ring"));
			goto fail1;
		}
		soc->num_tcl_data_rings++;
	}

	/* Tx specific init */
	if (dp_tx_pdev_attach(pdev)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_tx_pdev_attach failed"));
		goto fail1;
	}

	/* Setup per PDEV REO rings if configured */
	if (wlan_cfg_per_pdev_rx_ring(soc->wlan_cfg_ctx)) {
		if (dp_srng_setup(soc, &soc->reo_dest_ring[pdev_id], REO_DST,
			pdev_id, pdev_id, REO_DST_RING_SIZE)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("dp_srng_setup failed for reo_dest_ringn"));
			goto fail1;
		}
		soc->num_reo_dest_rings++;

	}

	if (dp_srng_setup(soc, &pdev->rx_refill_buf_ring, RXDMA_BUF, 0, pdev_id,
		RXDMA_BUF_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			 FL("dp_srng_setup failed rx refill ring"));
		goto fail1;
	}
#ifdef QCA_HOST2FW_RXBUF_RING
	if (dp_srng_setup(soc, &pdev->rx_mac_buf_ring, RXDMA_BUF, 1, pdev_id,
		RXDMA_BUF_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			 FL("dp_srng_setup failed rx mac ring"));
		goto fail1;
	}
#endif
	/* TODO: RXDMA destination ring is not planned to be used currently.
	 * Setup the ring when required
	 */
	if (dp_srng_setup(soc, &pdev->rxdma_mon_buf_ring, RXDMA_MONITOR_BUF, 0,
		pdev_id, RXDMA_MONITOR_BUF_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for rxdma_mon_buf_ring"));
		goto fail1;
	}

	if (dp_srng_setup(soc, &pdev->rxdma_mon_dst_ring, RXDMA_MONITOR_DST, 0,
		pdev_id, RXDMA_MONITOR_DST_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for rxdma_mon_dst_ring"));
		goto fail1;
	}


	if (dp_srng_setup(soc, &pdev->rxdma_mon_status_ring,
		RXDMA_MONITOR_STATUS, 0, pdev_id,
		RXDMA_MONITOR_STATUS_RING_SIZE)) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("dp_srng_setup failed for rxdma_mon_status_ring"));
		goto fail1;
	}

	/* Rx specific init */
	if (dp_rx_pdev_attach(pdev)) {
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("dp_rx_pdev_attach failed "));
			goto fail0;
	}

#ifndef CONFIG_WIN
	/* MCL */
	dp_local_peer_id_pool_init(pdev);
#endif

	return (void *)pdev;

fail1:
	dp_pdev_detach_wifi3((void *)pdev, 0);

fail0:
	return NULL;
}

/*
* dp_pdev_detach_wifi3() - detach txrx pdev
* @txrx_pdev: Datapath PDEV handle
* @force: Force detach
*
*/
static void dp_pdev_detach_wifi3(void *txrx_pdev, int force)
{
	struct dp_pdev *pdev = (struct dp_pdev *)txrx_pdev;
	struct dp_soc *soc = pdev->soc;

	dp_tx_pdev_detach(pdev);

	if (wlan_cfg_per_pdev_tx_ring(soc->wlan_cfg_ctx)) {
		dp_srng_cleanup(soc, &soc->tcl_data_ring[pdev->pdev_id],
			TCL_DATA, pdev->pdev_id);
		dp_srng_cleanup(soc, &soc->tx_comp_ring[pdev->pdev_id],
			WBM2SW_RELEASE, pdev->pdev_id);
	}

	dp_rx_pdev_detach(pdev);

	/* Setup per PDEV REO rings if configured */
	if (wlan_cfg_per_pdev_rx_ring(soc->wlan_cfg_ctx)) {
		dp_srng_cleanup(soc, &soc->reo_dest_ring[pdev->pdev_id],
			REO_DST, pdev->pdev_id);
	}

	dp_srng_cleanup(soc, &pdev->rx_refill_buf_ring, RXDMA_BUF, 0);

#ifdef QCA_HOST2FW_RXBUF_RING
	dp_srng_cleanup(soc, &pdev->rx_mac_buf_ring, RXDMA_BUF, 1);
#endif
	dp_srng_cleanup(soc, &pdev->rxdma_mon_buf_ring, RXDMA_MONITOR_BUF, 0);

	dp_srng_cleanup(soc, &pdev->rxdma_mon_dst_ring, RXDMA_MONITOR_DST, 0);

	dp_srng_cleanup(soc, &pdev->rxdma_mon_status_ring,
		RXDMA_MONITOR_STATUS, 0);

	soc->pdev_list[pdev->pdev_id] = NULL;

	qdf_mem_free(pdev);
}

/*
 * dp_soc_detach_wifi3() - Detach txrx SOC
 * @txrx_soc: DP SOC handle
 *
 */
static void dp_soc_detach_wifi3(void *txrx_soc)
{
	struct dp_soc *soc = (struct dp_soc *)txrx_soc;
	struct dp_pdev *pdev = qdf_mem_malloc(sizeof(*pdev));
	int i;

	soc->cmn_init_done = 0;

	dp_soc_interrupt_detach(soc);

	for (i = 0; i < MAX_PDEV_CNT; i++) {
		if (soc->pdev_list[i])
			dp_pdev_detach_wifi3((void *)pdev, 1);
	}

	dp_peer_find_detach(soc);

	/* TBD: Call Tx and Rx cleanup functions to free buffers and
	 * SW descriptors
	 */

	/* Free the ring memories */
	/* Common rings */
	dp_srng_cleanup(soc, &soc->wbm_desc_rel_ring, SW2WBM_RELEASE, 0);

	/* Tx data rings */
	if (!wlan_cfg_per_pdev_tx_ring(soc->wlan_cfg_ctx)) {
		dp_tx_soc_detach(soc);

		for (i = 0; i < soc->num_tcl_data_rings; i++) {
			dp_srng_cleanup(soc, &soc->tcl_data_ring[i],
				TCL_DATA, i);
			dp_srng_cleanup(soc, &soc->tx_comp_ring[i],
				WBM2SW_RELEASE, i);
		}
	}

	/* TCL command and status rings */
	dp_srng_cleanup(soc, &soc->tcl_cmd_ring, TCL_CMD, 0);
	dp_srng_cleanup(soc, &soc->tcl_status_ring, TCL_STATUS, 0);

	/* Rx data rings */
	if (!wlan_cfg_per_pdev_rx_ring(soc->wlan_cfg_ctx)) {
		soc->num_reo_dest_rings =
			wlan_cfg_num_reo_dest_rings(soc->wlan_cfg_ctx);
		for (i = 0; i < soc->num_reo_dest_rings; i++) {
			/* TODO: Get number of rings and ring sizes
			 * from wlan_cfg
			 */
			dp_srng_cleanup(soc, &soc->reo_dest_ring[i],
				REO_DST, i);
		}
	}
	/* REO reinjection ring */
	dp_srng_cleanup(soc, &soc->reo_reinject_ring, REO_REINJECT, 0);

	/* Rx release ring */
	dp_srng_cleanup(soc, &soc->rx_rel_ring, WBM2SW_RELEASE, 0);

	/* Rx exception ring */
	/* TODO: Better to store ring_type and ring_num in
	 * dp_srng during setup
	 */
	dp_srng_cleanup(soc, &soc->reo_exception_ring, REO_EXCEPTION, 0);

	/* REO command and status rings */
	dp_srng_cleanup(soc, &soc->reo_cmd_ring, REO_CMD, 0);
	dp_srng_cleanup(soc, &soc->reo_status_ring, REO_STATUS, 0);
	qdf_spinlock_destroy(&soc->rx.reo_cmd_lock);

	qdf_spinlock_destroy(&soc->peer_ref_mutex);
	htt_soc_detach(soc->htt_handle);
}

/*
 * dp_soc_attach_target_wifi3() - SOC initialization in the target
 * @txrx_soc: Datapath SOC handle
 */
static int dp_soc_attach_target_wifi3(struct cdp_soc_t *cdp_soc)
{
	struct dp_soc *soc = (struct dp_soc *)cdp_soc;
	int i;

	htt_soc_attach_target(soc->htt_handle);

	for (i = 0; i < MAX_PDEV_CNT; i++) {
		struct dp_pdev *pdev = soc->pdev_list[i];
		if (pdev) {
			htt_srng_setup(soc->htt_handle, i,
				pdev->rx_refill_buf_ring.hal_srng, RXDMA_BUF);
#ifdef QCA_HOST2FW_RXBUF_RING
			htt_srng_setup(soc->htt_handle, i,
				pdev->rx_mac_buf_ring.hal_srng, RXDMA_BUF);
#endif
#ifdef notyet /* FW doesn't handle monitor rings yet */
			htt_srng_setup(soc->htt_handle, i,
				pdev->rxdma_mon_buf_ring.hal_srng,
				RXDMA_MONITOR_BUF);
			htt_srng_setup(soc->htt_handle, i,
				pdev->rxdma_mon_dst_ring.hal_srng,
				RXDMA_MONITOR_DST);
			htt_srng_setup(soc->htt_handle, i,
				pdev->rxdma_mon_status_ring.hal_srng,
				RXDMA_MONITOR_STATUS);
#endif
		}
	}
	return 0;
}

/*
* dp_vdev_attach_wifi3() - attach txrx vdev
* @txrx_pdev: Datapath PDEV handle
* @vdev_mac_addr: MAC address of the virtual interface
* @vdev_id: VDEV Id
* @wlan_op_mode: VDEV operating mode
*
* Return: DP VDEV handle on success, NULL on failure
*/
static void *dp_vdev_attach_wifi3(void *txrx_pdev,
	uint8_t *vdev_mac_addr, uint8_t vdev_id, enum wlan_op_mode op_mode)
{
	struct dp_pdev *pdev = (struct dp_pdev *)txrx_pdev;
	struct dp_soc *soc = pdev->soc;
	struct dp_vdev *vdev = qdf_mem_malloc(sizeof(*vdev));

	if (!vdev) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("DP VDEV memory allocation failed"));
		goto fail0;
	}

	vdev->pdev = pdev;
	vdev->vdev_id = vdev_id;
	vdev->opmode = op_mode;
	vdev->osdev = soc->osdev;

	vdev->osif_rx = NULL;
	vdev->osif_rsim_rx_decap = NULL;
	vdev->osif_rx_mon = NULL;
	vdev->osif_vdev = NULL;

	vdev->delete.pending = 0;
	vdev->safemode = 0;
	vdev->drop_unenc = 1;
#ifdef notyet
	vdev->filters_num = 0;
#endif

	qdf_mem_copy(
		&vdev->mac_addr.raw[0], vdev_mac_addr, OL_TXRX_MAC_ADDR_LEN);

	vdev->tx_encap_type = wlan_cfg_pkt_type(soc->wlan_cfg_ctx);
	vdev->rx_decap_type = wlan_cfg_pkt_type(soc->wlan_cfg_ctx);

	/* TODO: Initialize default HTT meta data that will be used in
	 * TCL descriptors for packets transmitted from this VDEV
	 */

	TAILQ_INIT(&vdev->peer_list);

	/* add this vdev into the pdev's list */
	TAILQ_INSERT_TAIL(&pdev->vdev_list, vdev, vdev_list_elem);
	pdev->vdev_count++;

	dp_tx_vdev_attach(vdev);

#ifdef DP_INTR_POLL_BASED
	if (pdev->vdev_count == 1)
		qdf_timer_mod(&soc->int_timer, DP_INTR_POLL_TIMER_MS);
#endif

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
		"Created vdev %p (%pM)", vdev, vdev->mac_addr.raw);

	return (void *)vdev;

fail0:
	return NULL;
}

/**
 * dp_vdev_register_wifi3() - Register VDEV operations from osif layer
 * @vdev: Datapath VDEV handle
 * @osif_vdev: OSIF vdev handle
 * @txrx_ops: Tx and Rx operations
 *
 * Return: DP VDEV handle on success, NULL on failure
 */
static void dp_vdev_register_wifi3(void *vdev_handle, void *osif_vdev,
	struct ol_txrx_ops *txrx_ops)
{
	struct dp_vdev *vdev = (struct dp_vdev *)vdev_handle;
	vdev->osif_vdev = osif_vdev;
	vdev->osif_rx = txrx_ops->rx.rx;
	vdev->osif_rsim_rx_decap = txrx_ops->rx.rsim_rx_decap;
	vdev->osif_rx_mon = txrx_ops->rx.mon;
#ifdef notyet
#if ATH_SUPPORT_WAPI
	vdev->osif_check_wai = txrx_ops->rx.wai_check;
#endif
#if UMAC_SUPPORT_PROXY_ARP
	vdev->osif_proxy_arp = txrx_ops->proxy_arp;
#endif
#endif
	/* TODO: Enable the following once Tx code is integrated */
	txrx_ops->tx.tx = dp_tx_send;

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO,
		"DP Vdev Register success");
}

/*
 * dp_vdev_detach_wifi3() - Detach txrx vdev
 * @txrx_vdev:		Datapath VDEV handle
 * @callback:		Callback OL_IF on completion of detach
 * @cb_context:	Callback context
 *
 */
static void dp_vdev_detach_wifi3(void *vdev_handle,
	ol_txrx_vdev_delete_cb callback, void *cb_context)
{
	struct dp_vdev *vdev = (struct dp_vdev *)vdev_handle;
	struct dp_pdev *pdev = vdev->pdev;
	struct dp_soc *soc = pdev->soc;

	/* preconditions */
	qdf_assert(vdev);

	/* remove the vdev from its parent pdev's list */
	TAILQ_REMOVE(&pdev->vdev_list, vdev, vdev_list_elem);

	/*
	 * Use peer_ref_mutex while accessing peer_list, in case
	 * a peer is in the process of being removed from the list.
	 */
	qdf_spin_lock_bh(&soc->peer_ref_mutex);
	/* check that the vdev has no peers allocated */
	if (!TAILQ_EMPTY(&vdev->peer_list)) {
		/* debug print - will be removed later */
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_WARN,
			FL("not deleting vdev object %p (%pM)"
			"until deletion finishes for all its peers"),
			vdev, vdev->mac_addr.raw);
		/* indicate that the vdev needs to be deleted */
		vdev->delete.pending = 1;
		vdev->delete.callback = callback;
		vdev->delete.context = cb_context;
		qdf_spin_unlock_bh(&soc->peer_ref_mutex);
		return;
	}
	qdf_spin_unlock_bh(&soc->peer_ref_mutex);

	dp_tx_vdev_detach(vdev);
	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
		FL("deleting vdev object %p (%pM)"), vdev, vdev->mac_addr.raw);

	qdf_mem_free(vdev);

	if (callback)
		callback(cb_context);
}

/*
 * dp_peer_create_wifi3() - attach txrx peer
 * @txrx_vdev: Datapath VDEV handle
 * @peer_mac_addr: Peer MAC address
 *
 * Return: DP peeer handle on success, NULL on failure
 */
static void *dp_peer_create_wifi3(void *vdev_handle, uint8_t *peer_mac_addr)
{
	struct dp_peer *peer;
	int i;
	struct dp_vdev *vdev = (struct dp_vdev *)vdev_handle;
	struct dp_pdev *pdev;
	struct dp_soc *soc;

	/* preconditions */
	qdf_assert(vdev);
	qdf_assert(peer_mac_addr);

	pdev = vdev->pdev;
	soc = pdev->soc;
#ifdef notyet
	peer = (struct dp_peer *)qdf_mempool_alloc(soc->osdev,
		soc->mempool_ol_ath_peer);
#else
	peer = (struct dp_peer *)qdf_mem_malloc(sizeof(*peer));
#endif

	if (!peer)
		return NULL; /* failure */

	qdf_mem_zero(peer, sizeof(struct dp_peer));
	qdf_spinlock_create(&peer->peer_info_lock);

	/* store provided params */
	peer->vdev = vdev;
	qdf_mem_copy(
		&peer->mac_addr.raw[0], peer_mac_addr, OL_TXRX_MAC_ADDR_LEN);

	/* TODO: See of rx_opt_proc is really required */
	peer->rx_opt_proc = soc->rx_opt_proc;

	/* initialize the peer_id */
	for (i = 0; i < MAX_NUM_PEER_ID_PER_PEER; i++)
		peer->peer_ids[i] = HTT_INVALID_PEER;

	qdf_spin_lock_bh(&soc->peer_ref_mutex);

	qdf_atomic_init(&peer->ref_cnt);

	/* keep one reference for attach */
	qdf_atomic_inc(&peer->ref_cnt);

	/* add this peer into the vdev's list */
	TAILQ_INSERT_TAIL(&vdev->peer_list, peer, peer_list_elem);
	qdf_spin_unlock_bh(&soc->peer_ref_mutex);

	/* TODO: See if hash based search is required */
	dp_peer_find_hash_add(soc, peer);

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
		"vdev %p created peer %p (%pM)",
		vdev, peer, peer->mac_addr.raw);
	/*
	 * For every peer MAp message search and set if bss_peer
	 */
	if (memcmp(peer->mac_addr.raw, vdev->mac_addr.raw, 6) == 0) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
			"vdev bss_peer!!!!");
		peer->bss_peer = 1;
		vdev->vap_bss_peer = peer;
	}

#ifndef CONFIG_WIN
	dp_local_peer_id_alloc(pdev, peer);
#endif
	return (void *)peer;
}

/*
 * dp_peer_setup_wifi3() - initialize the peer
 * @vdev_hdl: virtual device object
 * @peer: Peer object
 *
 * Return: void
 */
static void dp_peer_setup_wifi3(void *vdev_hdl, void *peer_hdl)
{
	struct dp_peer *peer = (struct dp_peer *)peer_hdl;
	struct dp_vdev *vdev = (struct dp_vdev *)vdev_hdl;
	struct dp_pdev *pdev;
	struct dp_soc *soc;

	/* preconditions */
	qdf_assert(vdev);
	qdf_assert(peer);

	pdev = vdev->pdev;
	soc = pdev->soc;

	dp_peer_rx_init(pdev, peer);

	if (soc->cdp_soc.ol_ops->peer_set_default_routing) {
		/* TODO: Check the destination ring number to be passed to FW */
		soc->cdp_soc.ol_ops->peer_set_default_routing(soc->osif_soc,
			 peer->mac_addr.raw, peer->vdev->vdev_id, 0, 1);
	}
	return;
}

/*
 * dp_peer_authorize() - authorize txrx peer
 * @peer_handle:		Datapath peer handle
 * @authorize
 *
 */
static void dp_peer_authorize(void *peer_handle, uint32_t authorize)
{
	struct dp_peer *peer = (struct dp_peer *)peer_handle;
	struct dp_soc *soc;

	if (peer != NULL) {
		soc = peer->vdev->pdev->soc;

		qdf_spin_lock_bh(&soc->peer_ref_mutex);
		peer->authorize = authorize ? 1 : 0;
#ifdef notyet /* ATH_BAND_STEERING */
		peer->peer_bs_inact_flag = 0;
		peer->peer_bs_inact = soc->pdev_bs_inact_reload;
#endif
		qdf_spin_unlock_bh(&soc->peer_ref_mutex);
	}
}

/*
 * dp_peer_unref_delete() - unref and delete peer
 * @peer_handle:		Datapath peer handle
 *
 */
void dp_peer_unref_delete(void *peer_handle)
{
	struct dp_peer *peer = (struct dp_peer *)peer_handle;
	struct dp_vdev *vdev = peer->vdev;
	struct dp_soc *soc = vdev->pdev->soc;
	struct dp_peer *tmppeer;
	int found = 0;
	uint16_t peer_id;

	/*
	 * Hold the lock all the way from checking if the peer ref count
	 * is zero until the peer references are removed from the hash
	 * table and vdev list (if the peer ref count is zero).
	 * This protects against a new HL tx operation starting to use the
	 * peer object just after this function concludes it's done being used.
	 * Furthermore, the lock needs to be held while checking whether the
	 * vdev's list of peers is empty, to make sure that list is not modified
	 * concurrently with the empty check.
	 */
	qdf_spin_lock_bh(&soc->peer_ref_mutex);
	if (qdf_atomic_dec_and_test(&peer->ref_cnt)) {
		peer_id = peer->peer_ids[0];

		/*
		 * Make sure that the reference to the peer in
		 * peer object map is removed
		 */
		if (peer_id != HTT_INVALID_PEER)
			soc->peer_id_to_obj_map[peer_id] = NULL;

		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
			"Deleting peer %p (%pM)", peer, peer->mac_addr.raw);

		/* remove the reference to the peer from the hash table */
		dp_peer_find_hash_remove(soc, peer);

		TAILQ_FOREACH(tmppeer, &peer->vdev->peer_list, peer_list_elem) {
			if (tmppeer == peer) {
				found = 1;
				break;
			}
		}
		if (found) {
			TAILQ_REMOVE(&peer->vdev->peer_list, peer,
				peer_list_elem);
		} else {
			/*Ignoring the remove operation as peer not found*/
			QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_WARN,
				"peer %p not found in vdev (%p)->peer_list:%p",
				peer, vdev, &peer->vdev->peer_list);
		}

		/* cleanup the Rx reorder queues for this peer */
		dp_peer_rx_cleanup(vdev, peer);

		/* check whether the parent vdev has no peers left */
		if (TAILQ_EMPTY(&vdev->peer_list)) {
			/*
			 * Now that there are no references to the peer, we can
			 * release the peer reference lock.
			 */
			qdf_spin_unlock_bh(&soc->peer_ref_mutex);
			/*
			 * Check if the parent vdev was waiting for its peers
			 * to be deleted, in order for it to be deleted too.
			 */
			if (vdev->delete.pending) {
				ol_txrx_vdev_delete_cb vdev_delete_cb =
					vdev->delete.callback;
				void *vdev_delete_context =
					vdev->delete.context;

				QDF_TRACE(QDF_MODULE_ID_DP,
					QDF_TRACE_LEVEL_INFO_HIGH,
					FL("deleting vdev object %p (%pM)"
					" - its last peer is done"),
					vdev, vdev->mac_addr.raw);
				/* all peers are gone, go ahead and delete it */
				qdf_mem_free(vdev);
				if (vdev_delete_cb)
					vdev_delete_cb(vdev_delete_context);
			}
		} else {
			qdf_spin_unlock_bh(&soc->peer_ref_mutex);
		}
#ifdef notyet
		qdf_mempool_free(soc->osdev, soc->mempool_ol_ath_peer, peer);
#else
		qdf_mem_free(peer);
#endif
		if (soc->cdp_soc.ol_ops->peer_unref_delete) {
			soc->cdp_soc.ol_ops->peer_unref_delete(soc->osif_soc);
		}
	} else {
		qdf_spin_unlock_bh(&soc->peer_ref_mutex);
	}
}

/*
 * dp_peer_detach_wifi3() – Detach txrx peer
 * @peer_handle:		Datapath peer handle
 *
 */
static void dp_peer_delete_wifi3(void *peer_handle)
{
	struct dp_peer *peer = (struct dp_peer *)peer_handle;

	/* redirect the peer's rx delivery function to point to a
	 * discard func
	 */
	peer->rx_opt_proc = dp_rx_discard;

	QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_INFO_HIGH,
		FL("peer %p (%pM)"),  peer, peer->mac_addr.raw);

	/*
	 * Remove the reference added during peer_attach.
	 * The peer will still be left allocated until the
	 * PEER_UNMAP message arrives to remove the other
	 * reference, added by the PEER_MAP message.
	 */
	dp_peer_unref_delete(peer_handle);
#ifndef CONFIG_WIN
	dp_local_peer_id_free(peer->vdev->pdev, peer);
#endif
	qdf_spinlock_destroy(&peer->peer_info_lock);
}

/*
 * dp_get_vdev_mac_addr_wifi3() – Detach txrx peer
 * @peer_handle:		Datapath peer handle
 *
 */
static uint8 *dp_get_vdev_mac_addr_wifi3(void *pvdev)
{
	struct dp_vdev *vdev = pvdev;

	return vdev->mac_addr.raw;
}

/*
 * dp_get_vdev_from_vdev_id_wifi3() – Detach txrx peer
 * @peer_handle:		Datapath peer handle
 *
 */
static void *dp_get_vdev_from_vdev_id_wifi3(void *dev, uint8_t vdev_id)
{
	struct dp_pdev *pdev = dev;
	struct dp_vdev *vdev = NULL;

	if (qdf_unlikely(!pdev))
		return NULL;

	TAILQ_FOREACH(vdev, &pdev->vdev_list, vdev_list_elem) {
		if (vdev->vdev_id == vdev_id)
			break;
	}

	return vdev;
}

static int dp_get_opmode(void *vdev_handle)
{
	struct dp_vdev *vdev = vdev_handle;

	return vdev->opmode;
}

static void *dp_get_ctrl_pdev_from_vdev_wifi3(void *pvdev)
{
	struct dp_vdev *vdev = pvdev;
	struct dp_pdev *pdev = vdev->pdev;

	return (void *)pdev->wlan_cfg_ctx;
}

static struct cdp_cmn_ops dp_ops_cmn = {
	.txrx_soc_attach_target = dp_soc_attach_target_wifi3,
	.txrx_vdev_attach = dp_vdev_attach_wifi3,
	.txrx_vdev_detach = dp_vdev_detach_wifi3,
	.txrx_pdev_attach = dp_pdev_attach_wifi3,
	.txrx_pdev_detach = dp_pdev_detach_wifi3,
	.txrx_peer_create = dp_peer_create_wifi3,
	.txrx_peer_setup = dp_peer_setup_wifi3,
	.txrx_peer_teardown = NULL,
	.txrx_peer_delete = dp_peer_delete_wifi3,
	.txrx_vdev_register = dp_vdev_register_wifi3,
	.txrx_soc_detach = dp_soc_detach_wifi3,
	.txrx_get_vdev_mac_addr = dp_get_vdev_mac_addr_wifi3,
	.txrx_get_vdev_from_vdev_id = dp_get_vdev_from_vdev_id_wifi3,
	.txrx_get_ctrl_pdev_from_vdev = dp_get_ctrl_pdev_from_vdev_wifi3,
	.addba_requestprocess = dp_addba_requestprocess_wifi3,
	.addba_responsesetup = dp_addba_responsesetup_wifi3,
	.delba_process = dp_delba_process_wifi3,
	/* TODO: Add other functions */
};

static struct cdp_ctrl_ops dp_ops_ctrl = {
	.txrx_peer_authorize = dp_peer_authorize,
	/* TODO: Add other functions */
};

static struct cdp_me_ops dp_ops_me = {
	/* TODO */
};

static struct cdp_mon_ops dp_ops_mon = {
	/* TODO */
};

static struct cdp_host_stats_ops dp_ops_host_stats = {
	/* TODO */
};

static struct cdp_wds_ops dp_ops_wds = {
	/* TODO */
};

static struct cdp_raw_ops dp_ops_raw = {
	/* TODO */
};

#ifdef CONFIG_WIN
static struct cdp_pflow_ops dp_ops_pflow = {
	/* TODO */
};
#endif /* CONFIG_WIN */

#ifndef CONFIG_WIN
static struct cdp_misc_ops dp_ops_misc = {
	.get_opmode = dp_get_opmode,
};

static struct cdp_flowctl_ops dp_ops_flowctl = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

static struct cdp_lflowctl_ops dp_ops_l_flowctl = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

static struct cdp_ipa_ops dp_ops_ipa = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

static struct cdp_lro_ops dp_ops_lro = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

/**
 * dp_dummy_bus_suspend() - dummy bus suspend op
 *
 * FIXME - This is a placeholder for the actual logic!
 *
 * Return: QDF_STATUS_SUCCESS
 */
inline QDF_STATUS dp_dummy_bus_suspend(void)
{
	return QDF_STATUS_SUCCESS;
}

/**
 * dp_dummy_bus_resume() - dummy bus resume
 *
 * FIXME - This is a placeholder for the actual logic!
 *
 * Return: QDF_STATUS_SUCCESS
 */
inline QDF_STATUS dp_dummy_bus_resume(void)
{
	return QDF_STATUS_SUCCESS;
}

static struct cdp_bus_ops dp_ops_bus = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
	.bus_suspend = dp_dummy_bus_suspend,
	.bus_resume = dp_dummy_bus_resume
};

static struct cdp_ocb_ops dp_ops_ocb = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};


static struct cdp_throttle_ops dp_ops_throttle = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

static struct cdp_mob_stats_ops dp_ops_mob_stats = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

static struct cdp_cfg_ops dp_ops_cfg = {
	/* WIFI 3.0 DP NOT IMPLEMENTED YET */
};

static struct cdp_peer_ops dp_ops_peer = {
	.register_peer = dp_register_peer,
	.clear_peer = dp_clear_peer,
	.find_peer_by_addr = dp_find_peer_by_addr,
	.find_peer_by_addr_and_vdev = dp_find_peer_by_addr_and_vdev,
	.local_peer_id = dp_local_peer_id,
	.peer_find_by_local_id = dp_peer_find_by_local_id,
	.peer_state_update = dp_peer_state_update,
	.get_vdevid = dp_get_vdevid,
	.peer_get_peer_mac_addr = dp_peer_get_peer_mac_addr,
	.get_vdev_for_peer = dp_get_vdev_for_peer,
	.get_peer_state = dp_get_peer_state,
};
#endif

static struct cdp_ops dp_txrx_ops = {
	.cmn_drv_ops = &dp_ops_cmn,
	.ctrl_ops = &dp_ops_ctrl,
	.me_ops = &dp_ops_me,
	.mon_ops = &dp_ops_mon,
	.host_stats_ops = &dp_ops_host_stats,
	.wds_ops = &dp_ops_wds,
	.raw_ops = &dp_ops_raw,
#ifdef CONFIG_WIN
	.pflow_ops = &dp_ops_pflow,
#endif /* CONFIG_WIN */
#ifndef CONFIG_WIN
	.misc_ops = &dp_ops_misc,
	.cfg_ops = &dp_ops_cfg,
	.flowctl_ops = &dp_ops_flowctl,
	.l_flowctl_ops = &dp_ops_l_flowctl,
	.ipa_ops = &dp_ops_ipa,
	.lro_ops = &dp_ops_lro,
	.bus_ops = &dp_ops_bus,
	.ocb_ops = &dp_ops_ocb,
	.peer_ops = &dp_ops_peer,
	.throttle_ops = &dp_ops_throttle,
	.mob_stats_ops = &dp_ops_mob_stats,
#endif
};

/*
 * dp_soc_attach_wifi3() - Attach txrx SOC
 * @osif_soc:		Opaque SOC handle from OSIF/HDD
 * @htc_handle:	Opaque HTC handle
 * @hif_handle:	Opaque HIF handle
 * @qdf_osdev:	QDF device
 *
 * Return: DP SOC handle on success, NULL on failure
 */
/*
 * Local prototype added to temporarily address warning caused by
 * -Wmissing-prototypes. A more correct solution, namely to expose
 * a prototype in an appropriate header file, will come later.
 */
void *dp_soc_attach_wifi3(void *osif_soc, void *hif_handle,
	HTC_HANDLE htc_handle, qdf_device_t qdf_osdev,
	struct ol_if_ops *ol_ops);
void *dp_soc_attach_wifi3(void *osif_soc, void *hif_handle,
	HTC_HANDLE htc_handle, qdf_device_t qdf_osdev,
	struct ol_if_ops *ol_ops)
{
	struct dp_soc *soc = qdf_mem_malloc(sizeof(*soc));

	if (!soc) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("DP SOC memory allocation failed"));
		goto fail0;
	}

	soc->cdp_soc.ops = &dp_txrx_ops;
	soc->cdp_soc.ol_ops = ol_ops;
	soc->osif_soc = osif_soc;
	soc->osdev = qdf_osdev;
	soc->hif_handle = hif_handle;

	soc->hal_soc = hif_get_hal_handle(hif_handle);
	soc->htt_handle = htt_soc_attach(soc, osif_soc, htc_handle,
		soc->hal_soc, qdf_osdev);
	if (!soc->htt_handle) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
			FL("HTT attach failed"));
		goto fail1;
	}

	soc->wlan_cfg_ctx = wlan_cfg_soc_attach();
	if (!soc->wlan_cfg_ctx) {
		QDF_TRACE(QDF_MODULE_ID_DP, QDF_TRACE_LEVEL_ERROR,
				FL("wlan_cfg_soc_attach failed"));
		goto fail2;
	}
	qdf_spinlock_create(&soc->peer_ref_mutex);

	if (dp_soc_interrupt_attach(soc) != QDF_STATUS_SUCCESS) {
		goto fail2;
	}

	return (void *)soc;

fail2:
	htt_soc_detach(soc->htt_handle);
fail1:
	qdf_mem_free(soc);
fail0:
	return NULL;
}
