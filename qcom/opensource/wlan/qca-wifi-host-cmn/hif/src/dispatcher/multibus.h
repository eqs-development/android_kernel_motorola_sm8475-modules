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

#ifndef _MULTIBUS_H_
#define _MULTIBUS_H_

#include "osdep.h"
#include "qdf_status.h"
#include "hif_debug.h"

struct hif_softc;

struct hif_bus_ops {
	QDF_STATUS (*hif_bus_open)(struct hif_softc *hif_sc,
				   enum qdf_bus_type bus_type);
	void (*hif_bus_close)(struct hif_softc *hif_sc);
	void (*hif_bus_prevent_linkdown)(struct hif_softc *hif_sc, bool flag);
	void (*hif_reset_soc)(struct hif_softc *hif_sc);
	int (*hif_bus_suspend)(struct hif_softc *hif_ctx);
	int (*hif_bus_resume)(struct hif_softc *hif_ctx);
	int (*hif_target_sleep_state_adjust)(struct hif_softc *scn,
			bool sleep_ok, bool wait_for_it);
	void (*hif_disable_isr)(struct hif_softc *hif_sc);
	void (*hif_nointrs)(struct hif_softc *hif_sc);
	QDF_STATUS (*hif_enable_bus)(struct hif_softc *hif_sc,
			struct device *dev, void *bdev, const hif_bus_id *bid,
			enum hif_enable_type type);
	void (*hif_disable_bus)(struct hif_softc *hif_sc);
	int (*hif_bus_configure)(struct hif_softc *hif_sc);
	void (*hif_irq_disable)(struct hif_softc *hif_sc, int ce_id);
	void (*hif_irq_enable)(struct hif_softc *hif_sc, int ce_id);
	int (*hif_dump_registers)(struct hif_softc *hif_sc);
};

#ifdef HIF_SNOC
QDF_STATUS hif_initialize_snoc_ops(struct hif_bus_ops *hif_sc);
int hif_snoc_get_context_size(void);
#else
static inline QDF_STATUS hif_initialize_snoc_ops(struct hif_bus_ops *hif_sc)
{
	HIF_ERROR("%s: not supported", __func__);
	return QDF_STATUS_E_NOSUPPORT;
}
/**
 * hif_snoc_get_context_size() - dummy when snoc isn't supported
 *
 * Return: 0 as an invalid size to indicate no support
 */
static inline int hif_snoc_get_context_size(void)
{
	return 0;
}
#endif /* HIF_SNOC */

#ifdef HIF_PCI
QDF_STATUS hif_initialize_pci_ops(struct hif_softc *hif_sc);
int hif_pci_get_context_size(void);
#else
static inline QDF_STATUS hif_initialize_pci_ops(struct hif_softc *hif_sc)
{
	HIF_ERROR("%s: not supported", __func__);
	return QDF_STATUS_E_NOSUPPORT;
}
/**
 * hif_pci_get_context_size() - dummy when pci isn't supported
 *
 * Return: 0 as an invalid size to indicate no support
 */
static inline int hif_pci_get_context_size(void)
{
	return 0;
}
#endif /* HIF_PCI */

#endif /* _MULTIBUS_H_ */
