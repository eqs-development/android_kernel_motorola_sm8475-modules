// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include "msm_vidc_vb2.h"
#include "msm_vidc_core.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_driver.h"
#include "msm_vdec.h"
#include "msm_venc.h"
#include "msm_vidc_debug.h"

void *msm_vb2_get_userptr(struct device *dev, unsigned long vaddr,
			unsigned long size, enum dma_data_direction dma_dir)
{
	return (void *)0xdeadbeef;
}

void msm_vb2_put_userptr(void *buf_priv)
{
}

int msm_vidc_queue_setup(struct vb2_queue *q,
		unsigned int *num_buffers, unsigned int *num_planes,
		unsigned int sizes[], struct device *alloc_devs[])
{
	int rc = 0;
	struct msm_vidc_inst *inst;
	int port;

	if (!q || !num_buffers || !num_planes
		|| !sizes || !q->drv_priv) {
		d_vpr_e("%s: invalid params, q = %pK, %pK, %pK\n",
			__func__, q, num_buffers, num_planes);
		return -EINVAL;
	}
	inst = q->drv_priv;
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params %pK\n", __func__, inst);
		return -EINVAL;
	}
	if (inst->state == MSM_VIDC_START) {
		d_vpr_e("%s: invalid state %d\n", __func__, inst->state);
		return -EINVAL;
	}

	port = msm_vidc_get_port_from_v4l2_type(inst, q->type, __func__);
	if (port < 0)
		return -EINVAL;

	if (port == INPUT_PORT || port == INPUT_META_PORT) {
		if (inst->state == MSM_VIDC_START_INPUT) {
			d_vpr_e("%s: input invalid state %d\n",
				__func__, inst->state);
			return -EINVAL;
		}
	} else if (port == OUTPUT_PORT || port == OUTPUT_META_PORT) {
		if (inst->state == MSM_VIDC_START_OUTPUT) {
			d_vpr_e("%s: output invalid state %d\n",
				__func__, inst->state);
			return -EINVAL;
		}
	}

	*num_planes = 1;
	if (*num_buffers < inst->buffers.input.min_count +
		inst->buffers.input.extra_count)
		*num_buffers = inst->buffers.input.min_count +
			inst->buffers.input.extra_count;
	sizes[0] = inst->fmts[port].fmt.pix.sizeimage;
	inst->buffers.input.actual_count = *num_buffers;

	s_vpr_h(inst->sid,
		"queue_setup: type %d num_buffers %d sizes[0] %d\n",
		q->type, *num_buffers, sizes[0]);
	return rc;
}

int msm_vidc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	int rc = 0;
	struct msm_vidc_inst *inst;

	if (!q || !q->drv_priv) {
		d_vpr_e("%s: invalid input, q = %pK\n", q);
		return -EINVAL;
	}
	inst = q->drv_priv;
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	if (q->type == INPUT_META_PLANE || q->type == OUTPUT_META_PLANE) {
		s_vpr_h(inst->sid, "%s: nothing to start on meta port %d\n",
			__func__, q->type);
		return 0;
	}
	if (!is_decode_session(inst) && !is_encode_session(inst)) {
		s_vpr_e(inst->sid, "%s: invalid session %d\n",
			__func__, inst->domain);
		return -EINVAL;
	}
	s_vpr_h(inst->sid, "Streamon: %d\n", q->type);

	if (!inst->session_created) {
		rc = msm_vidc_session_open(inst);
		if (rc)
			return -EINVAL;
	}

	if (q->type == INPUT_PLANE) {
		if (is_decode_session(inst))
			rc = msm_vdec_start_input(inst);
		//else if (is_encode_session(inst))
		//	rc = msm_venc_start_input(inst);
	} else if (q->type == OUTPUT_PLANE) {
		if (is_decode_session(inst))
			rc = msm_vdec_start_output(inst);
		//else if (is_encode_session(inst))
		//	rc = msm_venc_start_output(inst);
	} else {
		s_vpr_e(inst->sid, "%s: invalid type %d\n", __func__, q->type);
		rc = -EINVAL;
	}

	return rc;
}

void msm_vidc_stop_streaming(struct vb2_queue *q)
{
}

void msm_vidc_buf_queue(struct vb2_buffer *vb2)
{
}

void msm_vidc_buf_cleanup(struct vb2_buffer *vb)
{
}