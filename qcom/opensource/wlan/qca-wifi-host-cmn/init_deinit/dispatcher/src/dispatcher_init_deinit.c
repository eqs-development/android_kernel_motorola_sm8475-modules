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
#include <qdf_trace.h>
#include <qdf_threads.h>
#include <dispatcher_init_deinit.h>
#include <scheduler_api.h>
#include <wlan_mgmt_txrx_utils_api.h>
#include <wlan_serialization_api.h>
#ifdef WLAN_PMO_ENABLE
#include "wlan_pmo_obj_mgmt_api.h"
#endif
#ifdef WLAN_POLICY_MGR_ENABLE
#include "wlan_policy_mgr_api.h"
#endif
#ifdef WLAN_ATF_ENABLE
#include <wlan_atf_utils_api.h>
#endif
#ifdef WIFI_POS_CONVERGED
#include "wifi_pos_api.h"
#endif /* WIFI_POS_CONVERGED */
#ifdef WLAN_FEATURE_NAN_CONVERGENCE
#include "wlan_nan_api.h"
#endif /* WLAN_FEATURE_NAN_CONVERGENCE */

#ifdef WLAN_CONV_CRYPTO_SUPPORTED
#include "wlan_crypto_main.h"
#endif
/**
 * DOC: This file provides various init/deinit trigger point for new
 * components.
 */

/* All new components needs to replace their dummy init/deinit
 * psoc_open, psco_close, psoc_enable and psoc_disable APIs once
 * thier actual handlers are ready
 */

static QDF_STATUS scm_init(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS scm_deinit(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS p2p_init(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS p2p_deinit(void)
{
	return QDF_STATUS_SUCCESS;
}


static QDF_STATUS tdls_init(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS tdls_deinit(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS scm_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS scm_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS p2p_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS p2p_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS tdls_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS tdls_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS scm_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS scm_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}


static QDF_STATUS p2p_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS p2p_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}


static QDF_STATUS tdls_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}


static QDF_STATUS tdls_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

#ifdef WLAN_PMO_ENABLE
static QDF_STATUS dispatcher_init_pmo(void)
{
	return pmo_init();
}

static QDF_STATUS dispatcher_deinit_pmo(void)
{
	return pmo_deinit();
}
#else
static QDF_STATUS dispatcher_init_pmo(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_deinit_pmo(void)
{
	return QDF_STATUS_SUCCESS;
}
#endif /* END of WLAN_PMO_ENABLE */

#ifdef WLAN_POLICY_MGR_ENABLE
static QDF_STATUS dispatcher_policy_mgr_init(void)
{
	return policy_mgr_init();
}

static QDF_STATUS dispatcher_policy_mgr_deinit(void)
{
	return policy_mgr_deinit();
}

static QDF_STATUS dispatcher_policy_mgr_psoc_enable(
	struct wlan_objmgr_psoc *psoc)
{
	return policy_mgr_psoc_enable(psoc);
}

static QDF_STATUS dispatcher_policy_mgr_psoc_disable(
	struct wlan_objmgr_psoc *psoc)
{
	return policy_mgr_psoc_disable(psoc);
}
#else
static QDF_STATUS dispatcher_policy_mgr_init(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_policy_mgr_deinit(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_policy_mgr_psoc_enable(
	struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_policy_mgr_psoc_disable(
	struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}
#endif /* END of WLAN_POLICY_MGR_ENABLE */

#ifdef WLAN_ATF_ENABLE
static QDF_STATUS dispatcher_init_atf(void)
{
	return wlan_atf_init();
}

static QDF_STATUS dispatcher_deinit_atf(void)
{
	return wlan_atf_deinit();
}

static QDF_STATUS atf_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	return wlan_atf_open(psoc);
}

static QDF_STATUS atf_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	return wlan_atf_close(psoc);
}

static QDF_STATUS atf_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return wlan_atf_enable(psoc);
}

static QDF_STATUS atf_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return wlan_atf_disable(psoc);
}
#else
static QDF_STATUS dispatcher_init_atf(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_deinit_atf(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS atf_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS atf_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS atf_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS atf_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}
#endif /* END of WLAN_ATF_ENABLE */

#ifdef WLAN_CONV_CRYPTO_SUPPORTED
static QDF_STATUS dispatcher_init_crypto(void)
{
	return wlan_crypto_init();
}

static QDF_STATUS dispatcher_deinit_crypto(void)
{
	return wlan_crypto_deinit();
}
#else
static QDF_STATUS dispatcher_init_crypto(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_deinit_crypto(void)
{
	return QDF_STATUS_SUCCESS;
}
#endif /* END of WLAN_CONV_CRYPTO_SUPPORTED */

#ifdef WIFI_POS_CONVERGED
static QDF_STATUS dispatcher_init_wifi_pos(void)
{
	return wifi_pos_init();
}

static QDF_STATUS dispatcher_deinit_wifi_pos(void)
{
	return wifi_pos_deinit();
}

static QDF_STATUS dispatcher_wifi_pos_enable(struct wlan_objmgr_psoc *psoc)
{
	return wifi_pos_psoc_enable(psoc);
}

static QDF_STATUS dispatcher_wifi_pos_disable(struct wlan_objmgr_psoc *psoc)
{
	return wifi_pos_psoc_disable(psoc);
}
#else
static QDF_STATUS dispatcher_init_wifi_pos(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_deinit_wifi_pos(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_wifi_pos_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_wifi_pos_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}
#endif

#ifdef WLAN_FEATURE_NAN_CONVERGENCE
static QDF_STATUS dispatcher_init_nan(void)
{
	return nan_init();
}

static QDF_STATUS dispatcher_deinit_nan(void)
{
	return nan_deinit();
}

static QDF_STATUS dispatcher_nan_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return nan_psoc_enable(psoc);
}

static QDF_STATUS dispatcher_nan_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return nan_psoc_disable(psoc);
}
#else
static QDF_STATUS dispatcher_init_nan(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_deinit_nan(void)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_nan_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS dispatcher_nan_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	return QDF_STATUS_SUCCESS;
}
#endif

QDF_STATUS dispatcher_init(void)
{
	if (QDF_STATUS_SUCCESS != wlan_objmgr_global_obj_init())
		goto out;

	if (QDF_STATUS_SUCCESS != wlan_mgmt_txrx_init())
		goto mgmt_txrx_init_fail;

	if (QDF_STATUS_SUCCESS != scm_init())
		goto scm_init_fail;

	if (QDF_STATUS_SUCCESS != p2p_init())
		goto p2p_init_fail;

	if (QDF_STATUS_SUCCESS != tdls_init())
		goto tdls_init_fail;

	if (QDF_STATUS_SUCCESS != wlan_serialization_init())
		goto serialization_init_fail;

	if (QDF_STATUS_SUCCESS != scheduler_init())
		goto scheduler_init_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_init_pmo())
		goto pmo_init_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_init_crypto())
		goto crypto_init_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_policy_mgr_init())
		goto policy_mgr_init_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_init_atf())
		goto atf_init_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_init_wifi_pos())
		goto wifi_pos_init_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_init_nan())
		goto nan_init_fail;

	return QDF_STATUS_SUCCESS;

nan_init_fail:
	dispatcher_deinit_wifi_pos();
wifi_pos_init_fail:
	dispatcher_deinit_atf();
atf_init_fail:
	dispatcher_policy_mgr_deinit();
policy_mgr_init_fail:
	dispatcher_deinit_crypto();
crypto_init_fail:
	dispatcher_deinit_pmo();
pmo_init_fail:
	scheduler_deinit();
scheduler_init_fail:
	wlan_serialization_deinit();
serialization_init_fail:
	tdls_deinit();
tdls_init_fail:
	p2p_deinit();
p2p_init_fail:
	scm_deinit();
scm_init_fail:
	wlan_mgmt_txrx_deinit();
mgmt_txrx_init_fail:
	wlan_objmgr_global_obj_deinit();

out:
	return QDF_STATUS_E_FAILURE;
}
EXPORT_SYMBOL(dispatcher_init);

QDF_STATUS dispatcher_deinit(void)
{
	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_deinit_nan());

	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_deinit_wifi_pos());

	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_deinit_atf());

	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_policy_mgr_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_deinit_crypto());

	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_deinit_pmo());

	QDF_BUG(QDF_STATUS_SUCCESS == scheduler_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == wlan_serialization_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == tdls_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == p2p_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == scm_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == wlan_mgmt_txrx_deinit());

	QDF_BUG(QDF_STATUS_SUCCESS == wlan_objmgr_global_obj_deinit());

	return QDF_STATUS_SUCCESS;
}
EXPORT_SYMBOL(dispatcher_deinit);

QDF_STATUS dispatcher_psoc_open(struct wlan_objmgr_psoc *psoc)
{
	if (QDF_STATUS_SUCCESS != scm_psoc_open(psoc))
		goto out;

	if (QDF_STATUS_SUCCESS != p2p_psoc_open(psoc))
		goto p2p_psoc_open_fail;

	if (QDF_STATUS_SUCCESS != tdls_psoc_open(psoc))
		goto tdls_psoc_open_fail;

	if (QDF_STATUS_SUCCESS != wlan_serialization_psoc_open(psoc))
		goto serialization_psoc_open_fail;

	if (QDF_STATUS_SUCCESS != atf_psoc_open(psoc))
		goto atf_psoc_open_fail;

	return QDF_STATUS_SUCCESS;

atf_psoc_open_fail:
	wlan_serialization_psoc_close(psoc);
serialization_psoc_open_fail:
	tdls_psoc_close(psoc);
tdls_psoc_open_fail:
	p2p_psoc_close(psoc);
p2p_psoc_open_fail:
	scm_psoc_close(psoc);

out:
	return QDF_STATUS_E_FAILURE;
}
EXPORT_SYMBOL(dispatcher_psoc_open);

QDF_STATUS dispatcher_psoc_close(struct wlan_objmgr_psoc *psoc)
{
	QDF_BUG(QDF_STATUS_SUCCESS == atf_psoc_close(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == wlan_serialization_psoc_close(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == tdls_psoc_close(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == p2p_psoc_close(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == scm_psoc_close(psoc));

	return QDF_STATUS_SUCCESS;
}
EXPORT_SYMBOL(dispatcher_psoc_close);

QDF_STATUS dispatcher_psoc_enable(struct wlan_objmgr_psoc *psoc)
{
	if (QDF_STATUS_SUCCESS != scm_psoc_enable(psoc))
		goto out;

	if (QDF_STATUS_SUCCESS != p2p_psoc_enable(psoc))
		goto p2p_psoc_enable_fail;

	if (QDF_STATUS_SUCCESS != tdls_psoc_enable(psoc))
		goto tdls_psoc_enable_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_policy_mgr_psoc_enable(psoc))
		goto policy_mgr_psoc_enable_fail;

	if (QDF_STATUS_SUCCESS != atf_psoc_enable(psoc))
		goto atf_psoc_enable_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_wifi_pos_enable(psoc))
		goto wifi_pos_psoc_enable_fail;

	if (QDF_STATUS_SUCCESS != dispatcher_nan_psoc_enable(psoc))
		goto nan_psoc_enable_fail;

	return QDF_STATUS_SUCCESS;

nan_psoc_enable_fail:
	dispatcher_wifi_pos_disable(psoc);
wifi_pos_psoc_enable_fail:
	atf_psoc_disable(psoc);
atf_psoc_enable_fail:
	dispatcher_policy_mgr_psoc_disable(psoc);
policy_mgr_psoc_enable_fail:
	tdls_psoc_disable(psoc);
tdls_psoc_enable_fail:
	p2p_psoc_disable(psoc);
p2p_psoc_enable_fail:
	scm_psoc_disable(psoc);

out:
	return QDF_STATUS_E_FAILURE;
}
EXPORT_SYMBOL(dispatcher_psoc_enable);

QDF_STATUS dispatcher_psoc_disable(struct wlan_objmgr_psoc *psoc)
{
	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_nan_psoc_disable(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == dispatcher_wifi_pos_disable(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == atf_psoc_disable(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS ==
		dispatcher_policy_mgr_psoc_disable(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == tdls_psoc_disable(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == p2p_psoc_disable(psoc));

	QDF_BUG(QDF_STATUS_SUCCESS == scm_psoc_disable(psoc));

	return QDF_STATUS_SUCCESS;
}
EXPORT_SYMBOL(dispatcher_psoc_disable);
