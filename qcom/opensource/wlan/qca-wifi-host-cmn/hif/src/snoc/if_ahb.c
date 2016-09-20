/*
 * Copyright (c) 2013-2016 The Linux Foundation. All rights reserved.
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
 * DOC: if_ahb.c
 *
 * c file for ahb specific implementations.
 */

#include "hif.h"
#include "hif_main.h"
#include "hif_debug.h"
#include "hif_io32.h"
#include "ce_main.h"
#include "ce_tasklet.h"
#include "if_ahb.h"
#include "if_pci.h"
#include "ahb_api.h"
#include "pci_api.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
#define IRQF_DISABLED 0x00000020
#endif

#define HIF_IC_CE0_IRQ_OFFSET 4
#define HIF_IC_MAX_IRQ 54

/* integrated chip irq names */
const char *ic_irqname[HIF_IC_MAX_IRQ] = {
"misc_pulse1",
"misc_latch",
"sw_exception",
"watchdog",
"ce0",
"ce1",
"ce2",
"ce3",
"ce4",
"ce5",
"ce6",
"ce7",
"ce8",
"ce9",
"ce10",
"ce11",
"ce12",
"ce13",
"host2wbm_desc_feed",
"host2reo_re_injection",
"host2reo_command",
"host2rxdma_monitor_ring3",
"host2rxdma_monitor_ring2",
"host2rxdma_monitor_ring1",
"reo2ost_exception",
"wbm2host_rx_release",
"reo2host_status",
"reo2host_destination_ring4",
"reo2host_destination_ring3",
"reo2host_destination_ring2",
"reo2host_destination_ring1",
"rxdma2host_monitor_destination_mac3",
"rxdma2host_monitor_destination_mac2",
"rxdma2host_monitor_destination_mac1",
"ppdu_end_interrupts_mac3",
"ppdu_end_interrupts_mac2",
"ppdu_end_interrupts_mac1",
"rxdma2host_monitor_status_ring_mac3",
"rxdma2host_monitor_status_ring_mac2",
"rxdma2host_monitor_status_ring_mac1",
"host2rxdma_host_buf_ring_mac3",
"host2rxdma_host_buf_ring_mac2",
"host2rxdma_host_buf_ring_mac1",
"rxdma2host_destination_ring_mac3",
"rxdma2host_destination_ring_mac2",
"rxdma2host_destination_ring_mac1",
"host2tcl_input_ring4",
"host2tcl_input_ring3",
"host2tcl_input_ring2",
"host2tcl_input_ring1",
"wbm2host_tx_completions_ring3",
"wbm2host_tx_completions_ring2",
"wbm2host_tx_completions_ring1",
"tcl2host_status_ring",
};

irqreturn_t hif_ahb_interrupt_handler(int irq, void *context);
/**
 * hif_disable_isr() - disable isr
 *
 * This function disables isr and kills tasklets
 *
 * @hif_ctx: struct hif_softc
 *
 * Return: void
 */
void hif_ahb_disable_isr(struct hif_softc *scn)
{
	struct hif_pci_softc *sc = HIF_GET_PCI_SOFTC(scn);

	hif_nointrs(scn);
	ce_tasklet_kill(scn);
	tasklet_kill(&sc->intr_tq);
	qdf_atomic_set(&scn->active_tasklet_cnt, 0);
}

/**
 * hif_dump_registers() - dump bus debug registers
 * @scn: struct hif_opaque_softc
 *
 * This function dumps hif bus debug registers
 *
 * Return: 0 for success or error code
 */
int hif_ahb_dump_registers(struct hif_softc *hif_ctx)
{
	int status;
	struct hif_softc *scn = HIF_GET_SOFTC(hif_ctx);

	status = hif_dump_ce_registers(scn);
	if (status)
		HIF_ERROR("%s: Dump CE Registers Failed status %d", __func__,
							status);

	return 0;
}

/**
 * hif_ahb_close() - hif_bus_close
 * @scn: pointer to the hif context.
 *
 * This is a callback function for hif_bus_close.
 *
 *
 * Return: n/a
 */
void hif_ahb_close(struct hif_softc *scn)
{
	hif_ce_close(scn);
}

/**
 * hif_bus_open() - hif_ahb open
 * @hif_ctx: hif context
 * @bus_type: bus type
 *
 * This is a callback function for hif_bus_open.
 *
 * Return: n/a
 */
QDF_STATUS hif_ahb_open(struct hif_softc *hif_ctx, enum qdf_bus_type bus_type)
{

	struct hif_pci_softc *sc = HIF_GET_PCI_SOFTC(hif_ctx);

	qdf_spinlock_create(&sc->irq_lock);
	return hif_ce_open(hif_ctx);
}

/**
 * hif_bus_configure() - Configure the bus
 * @scn: pointer to the hif context.
 *
 * This function configure the ahb bus
 *
 * return: 0 for success. nonzero for failure.
 */
int hif_ahb_bus_configure(struct hif_softc *scn)
{
	return hif_pci_bus_configure(scn);
}

/**
 * hif_configure_msi_ahb - Configure MSI interrupts
 * @sc : pointer to the hif context
 *
 * return: 0 for success. nonzero for failure.
 */

int hif_configure_msi_ahb(struct hif_pci_softc *sc)
{
	return 0;
}

/**
 * hif_ahb_configure_legacy_irq() - Configure Legacy IRQ
 * @sc: pointer to the hif context.
 *
 * This function registers the irq handler and enables legacy interrupts
 *
 * return: 0 for success. nonzero for failure.
 */
int hif_ahb_configure_legacy_irq(struct hif_pci_softc *sc)
{
	int ret = 0;
	struct hif_softc *scn = HIF_GET_SOFTC(sc);
	struct platform_device *pdev = (struct platform_device *)sc->pdev;
	int irq = 0;

	/* do not support MSI or MSI IRQ failed */
	tasklet_init(&sc->intr_tq, wlan_tasklet, (unsigned long)sc);
	irq = platform_get_irq_byname(pdev, "legacy");
	if (irq < 0) {
		dev_err(&pdev->dev, "Unable to get irq\n");
		ret = -1;
		goto end;
	}
	ret = request_irq(irq, hif_pci_interrupt_handler,
				IRQF_DISABLED, "wlan_ahb", sc);
	if (ret) {
		dev_err(&pdev->dev, "ath_request_irq failed\n");
		ret = -1;
		goto end;
	}
	sc->irq = irq;

	/* Use Legacy PCI Interrupts */
	hif_write32_mb(sc->mem+(SOC_CORE_BASE_ADDRESS |
				PCIE_INTR_ENABLE_ADDRESS),
			PCIE_INTR_FIRMWARE_MASK | PCIE_INTR_CE_MASK_ALL);
	/* read once to flush */
	hif_read32_mb(sc->mem+(SOC_CORE_BASE_ADDRESS |
				PCIE_INTR_ENABLE_ADDRESS)
		     );

end:
	return ret;
}

int hif_ahb_configure_irq(struct hif_pci_softc *sc)
{
	int ret = 0;
	struct hif_softc *scn = HIF_GET_SOFTC(sc);
	struct platform_device *pdev = (struct platform_device *)sc->pdev;
	struct HIF_CE_state *hif_ce_state = HIF_GET_CE_STATE(scn);
	int irq = 0;
	int i;

	/* configure per CE interrupts */
	for (i = 0; i < scn->ce_count; i++) {
		irq = platform_get_irq_byname(pdev, ic_irqname[HIF_IC_CE0_IRQ_OFFSET + i]);
		ret = request_irq(irq ,
				hif_ahb_interrupt_handler,
				IRQF_TRIGGER_RISING, ic_irqname[HIF_IC_CE0_IRQ_OFFSET + i],
				&hif_ce_state->tasklets[i]);
		if (ret) {
			dev_err(&pdev->dev, "ath_request_irq failed\n");
			ret = -1;
			goto end;
		}
		hif_ahb_irq_enable(scn, i);
	}

end:
	return ret;
}

irqreturn_t hif_ahb_interrupt_handler(int irq, void *context)
{
	struct ce_tasklet_entry *tasklet_entry = context;
	return ce_dispatch_interrupt(tasklet_entry->ce_id, tasklet_entry);
}


/**
 * hif_target_sync() : ensure the target is ready
 * @scn: hif control structure
 *
 * Informs fw that we plan to use legacy interupts so that
 * it can begin booting. Ensures that the fw finishes booting
 * before continuing. Should be called before trying to write
 * to the targets other registers for the first time.
 *
 * Return: none
 */
int hif_target_sync_ahb(struct hif_softc *scn)
{
	hif_write32_mb(scn->mem + FW_INDICATOR_ADDRESS, FW_IND_HOST_READY);
	if (HAS_FW_INDICATOR) {
		int wait_limit = 500;
		int fw_ind = 0;

		while (1) {
			fw_ind = hif_read32_mb(scn->mem +
					FW_INDICATOR_ADDRESS);
			if (fw_ind & FW_IND_INITIALIZED)
				break;
			if (wait_limit-- < 0)
				break;
			hif_write32_mb(scn->mem+(SOC_CORE_BASE_ADDRESS |
				PCIE_INTR_ENABLE_ADDRESS),
				PCIE_INTR_FIRMWARE_MASK);
			qdf_mdelay(10);
		}
		if (wait_limit < 0) {
			HIF_TRACE("%s: FW signal timed out", __func__);
			return -EIO;
		} else {
			HIF_TRACE("%s: Got FW signal, retries = %x", __func__,
							500-wait_limit);
		}
	}

	return 0;
}

/**
 * hif_disable_bus() - Disable the bus
 * @scn : pointer to the hif context
 *
 * This function disables the bus and helds the target in reset state
 *
 * Return: none
 */
void hif_ahb_disable_bus(struct hif_softc *scn)
{
	struct hif_pci_softc *sc = HIF_GET_PCI_SOFTC(scn);
	void __iomem *mem;
	struct platform_device *pdev = (struct platform_device *)sc->pdev;
	struct resource *memres = NULL;
	int mem_pa_size = 0;

	/*Disable WIFI clock input*/
	if (sc->mem) {
		memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!memres) {
			HIF_INFO("%s: Failed to get IORESOURCE_MEM\n",
								__func__);
			return;
		}
		mem_pa_size = memres->end - memres->start + 1;

		hif_ahb_clk_enable_disable(&pdev->dev, 0);

		hif_ahb_device_reset(scn);
		mem = (void __iomem *)sc->mem;
		if (mem) {
			devm_iounmap(&pdev->dev, mem);
			devm_release_mem_region(&pdev->dev, scn->mem_pa,
								mem_pa_size);
			sc->mem = NULL;
		}
	}
	scn->mem = NULL;
}

/**
 * hif_enable_bus() - Enable the bus
 * @dev: dev
 * @bdev: bus dev
 * @bid: bus id
 * @type: bus type
 *
 * This function enables the radio bus by enabling necessary
 * clocks and waits for the target to get ready to proceed futher
 *
 * Return: QDF_STATUS
 */
QDF_STATUS hif_ahb_enable_bus(struct hif_softc *ol_sc,
		struct device *dev, void *bdev,
		const hif_bus_id *bid,
		enum hif_enable_type type)
{
	int ret = 0;
	int hif_type;
	int target_type;
	const struct platform_device_id *id = (struct platform_device_id *)bid;
	struct platform_device *pdev = bdev;
	struct hif_target_info *tgt_info = NULL;
	struct resource *memres = NULL;
	void __iomem *mem = NULL;
	uint32_t revision_id = 0;
	struct hif_pci_softc *sc = HIF_GET_PCI_SOFTC(ol_sc);

	sc->pdev = (struct pci_dev *)pdev;
	sc->dev = &pdev->dev;
	sc->devid = id->driver_data;

	ret = hif_get_device_type(id->driver_data, revision_id,
			&hif_type, &target_type);
	if (ret < 0) {
		HIF_ERROR("%s: invalid device  ret %d id %d revision_id %d",
			__func__, ret, (int)id->driver_data, revision_id);
		return QDF_STATUS_E_FAILURE;
	}

	memres = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!memres) {
		HIF_INFO("%s: Failed to get IORESOURCE_MEM\n", __func__);
		return -EIO;
	}

	ret = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		HIF_INFO("ath: 32-bit DMA not available\n");
		goto err_cleanup1;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
#else
	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
#endif
	if (ret) {
		HIF_ERROR("%s: failed to set dma mask error = %d",
				__func__, ret);
		return ret;
	}

	/* Arrange for access to Target SoC registers. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)
	mem = devm_ioremap_resource(&pdev->dev, memres);
#else
	mem = devm_request_and_ioremap(&pdev->dev, memres);
#endif
	if (IS_ERR(mem)) {
		HIF_INFO("ath: ioremap error\n");
		ret = PTR_ERR(mem);
		goto err_cleanup1;
	}

	sc->mem = mem;
	ol_sc->mem = mem;
	ol_sc->mem_pa = memres->start;

	tgt_info = hif_get_target_info_handle((struct hif_opaque_softc *)ol_sc);

	tgt_info->target_type = target_type;
	hif_register_tbl_attach(ol_sc, hif_type);
	hif_target_register_tbl_attach(ol_sc, target_type);

	if (hif_ahb_enable_radio(sc, pdev, id) != 0) {
		HIF_INFO("error in enabling soc\n");
		return -EIO;
	}

	if (hif_target_sync_ahb(ol_sc) < 0) {
		ret = -EIO;
		goto err_target_sync;
	}
	HIF_TRACE("%s: X - hif_type = 0x%x, target_type = 0x%x",
			__func__, hif_type, target_type);

	return QDF_STATUS_SUCCESS;
err_target_sync:
	HIF_INFO("Error: Disabling target\n");
	hif_ahb_disable_bus(ol_sc);
err_cleanup1:
	return ret;
}


/**
 * hif_reset_soc() - reset soc
 *
 * @hif_ctx: HIF context
 *
 * This function resets soc and helds the
 * target in reset state
 *
 * Return: void
 */
/* Function to reset SoC */
void hif_ahb_reset_soc(struct hif_softc *hif_ctx)
{
	hif_ahb_device_reset(hif_ctx);
}


/**
 * hif_nointrs() - disable IRQ
 *
 * @scn: struct hif_softc
 *
 * This function stops interrupt(s)
 *
 * Return: none
 */
void hif_ahb_nointrs(struct hif_softc *scn)
{
	int i;
	int irq;
	struct hif_pci_softc *sc = HIF_GET_PCI_SOFTC(scn);
	struct platform_device *pdev = (struct platform_device *)sc->pdev;
	struct HIF_CE_state *hif_state = HIF_GET_CE_STATE(scn);

	if (scn->request_irq_done == false)
		return;

	if (sc->num_msi_intrs > 0) {
		/* MSI interrupt(s) */
		for (i = 0; i < sc->num_msi_intrs; i++) {
			free_irq(sc->irq + i, sc);
		}
		sc->num_msi_intrs = 0;
	} else {
		if (!scn->per_ce_irq) {
			free_irq(sc->irq, sc);
		} else {
			for (i = 0; i < scn->ce_count; i++) {
				irq = platform_get_irq_byname(pdev, ic_irqname[HIF_IC_CE0_IRQ_OFFSET + i]);
				free_irq(irq, sc);
			}
		}
	}
	ce_unregister_irq(hif_state, CE_ALL_BITMAP);
	scn->request_irq_done = false;
}

/**
 * ce_irq_enable() - enable copy engine IRQ
 * @scn: struct hif_softc
 * @ce_id: ce_id
 *
 * This function enables the interrupt for the radio.
 *
 * Return: N/A
 */
void hif_ahb_irq_enable(struct hif_softc *scn, int ce_id)
{
	uint32_t regval;
	uint32_t reg_offset = 0;
	struct HIF_CE_state *hif_state = HIF_GET_CE_STATE(scn);
	struct CE_pipe_config *target_ce_conf = &hif_state->target_ce_config[ce_id];

	if (scn->per_ce_irq) {
		if (target_ce_conf->pipedir & PIPEDIR_OUT) {
			reg_offset = HOST_IE_ADDRESS;
			qdf_spin_lock_irqsave(&hif_state->irq_reg_lock);
			regval = hif_read32_mb(scn->mem + reg_offset);
			regval |= (1 << ce_id);
			hif_write32_mb(scn->mem + reg_offset, regval);
			qdf_spin_unlock_irqrestore(&hif_state->irq_reg_lock);
		}
		if (target_ce_conf->pipedir & PIPEDIR_IN) {
			reg_offset = HOST_IE_ADDRESS_2;
			qdf_spin_lock_irqsave(&hif_state->irq_reg_lock);
			regval = hif_read32_mb(scn->mem + reg_offset);
			regval |= (1 << ce_id);
			hif_write32_mb(scn->mem + reg_offset, regval);
			qdf_spin_unlock_irqrestore(&hif_state->irq_reg_lock);
		}
	} else {
		hif_pci_irq_enable(scn, ce_id);
	}
}

/**
 * ce_irq_disable() - disable copy engine IRQ
 * @scn: struct hif_softc
 * @ce_id: ce_id
 *
 * Return: N/A
 */
void hif_ahb_irq_disable(struct hif_softc *scn, int ce_id)
{
	uint32_t regval;
	uint32_t reg_offset = 0;
	struct HIF_CE_state *hif_state = HIF_GET_CE_STATE(scn);
	struct CE_pipe_config *target_ce_conf = &hif_state->target_ce_config[ce_id];

	if (scn->per_ce_irq) {
		if (target_ce_conf->pipedir & PIPEDIR_OUT) {
			reg_offset = HOST_IE_ADDRESS;
			qdf_spin_lock_irqsave(&hif_state->irq_reg_lock);
			regval = hif_read32_mb(scn->mem + reg_offset);
			regval &= ~(1 << ce_id);
			hif_write32_mb(scn->mem + reg_offset, regval);
			qdf_spin_unlock_irqrestore(&hif_state->irq_reg_lock);
		}
		if (target_ce_conf->pipedir & PIPEDIR_IN) {
			reg_offset = HOST_IE_ADDRESS_2;
			qdf_spin_lock_irqsave(&hif_state->irq_reg_lock);
			regval = hif_read32_mb(scn->mem + reg_offset);
			regval &= ~(1 << ce_id);
			hif_write32_mb(scn->mem + reg_offset, regval);
			qdf_spin_unlock_irqrestore(&hif_state->irq_reg_lock);
		}
	}
}
