// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <media/msm_vidc_utils.h>
#include <media/msm_media_info.h>

#include "msm_venc.h"
#include "msm_vidc_core.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_platform.h"
#include "msm_vidc_debug.h"

static int msm_venc_codec_change(struct msm_vidc_inst *inst, u32 codec)
{
	int rc = 0;

	d_vpr_h("%s()\n", __func__);

	inst->codec = get_vidc_codec_from_v4l2(codec);
	rc = msm_vidc_get_inst_capability(inst);
	return rc;
}

int msm_venc_s_fmt(struct msm_vidc_inst *inst, struct v4l2_format *f)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct v4l2_format *fmt;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	if (inst->state == MSM_VIDC_START) {
		d_vpr_e("%s: invalid state %d\n", __func__, inst->state);
		return -EINVAL;
	}

	if (f->type == INPUT_PLANE) {
		if (inst->state == MSM_VIDC_START_INPUT) {
			d_vpr_e("%s: invalid state %d\n", __func__, inst->state);
			return -EINVAL;
		}
		fmt = &inst->fmts[INPUT_PORT];
		fmt->type = INPUT_PLANE;
		fmt->fmt.pix.pixelformat = f->fmt.pix.pixelformat;
		fmt->fmt.pix.width = VENUS_Y_STRIDE(
			get_media_colorformat_from_v4l2(fmt->fmt.pix.pixelformat),
			f->fmt.pix.width);
		fmt->fmt.pix.height = VENUS_Y_SCANLINES(
			get_media_colorformat_from_v4l2(fmt->fmt.pix.pixelformat),
			f->fmt.pix.height);
		fmt->fmt.pix.bytesperline = fmt->fmt.pix.width;
		fmt->fmt.pix.sizeimage = call_session_op(core, buffer_size,
				inst, MSM_VIDC_BUF_INPUT);
		inst->buffers.input.min_count =
			call_session_op(core, min_count, inst, MSM_VIDC_BUF_INPUT);
		inst->buffers.input.extra_count =
			call_session_op(core, extra_count, inst, MSM_VIDC_BUF_INPUT);
		if (inst->buffers.input.actual_count <
			inst->buffers.input.min_count +
			inst->buffers.input.extra_count) {
			inst->buffers.input.actual_count =
				inst->buffers.input.min_count +
				inst->buffers.input.extra_count;
		}
		inst->buffers.input.size = fmt->fmt.pix.sizeimage;

		//rc = msm_vidc_check_session_supported(inst);
		if (rc)
			goto err_invalid_fmt;
		//update_log_ctxt(inst->sid, inst->session_type,
		//	mplane->pixelformat);
		s_vpr_h(inst->sid,
			"%s: input: codec %#x width %d height %d size %d min_count %d extra_count %d\n",
			__func__, f->fmt.pix.pixelformat, f->fmt.pix.width,
			f->fmt.pix.height, fmt->fmt.pix.sizeimage,
			inst->buffers.input.min_count,
			inst->buffers.input.extra_count);

		//msm_vidc_update_dcvs(inst);
		//msm_vidc_update_batching(inst);

	} else if (f->type == INPUT_META_PLANE) {
		if (inst->state == MSM_VIDC_START_INPUT) {
			d_vpr_e("%s: invalid state %d\n", __func__, inst->state);
			return -EINVAL;
		}
		fmt = &inst->fmts[INPUT_META_PORT];
		fmt->type = INPUT_META_PLANE;
		fmt->fmt.meta.dataformat = V4L2_PIX_FMT_VIDC_META;
		fmt->fmt.meta.buffersize = call_session_op(core, buffer_size,
				inst, MSM_VIDC_BUF_INPUT_META);
		inst->buffers.input_meta.min_count =
				inst->buffers.input.min_count;
		inst->buffers.input_meta.extra_count =
				inst->buffers.input.extra_count;
		inst->buffers.input_meta.actual_count =
				inst->buffers.input.actual_count;
		inst->buffers.input_meta.size = fmt->fmt.meta.buffersize;
		s_vpr_h(inst->sid,
			"%s: input meta: size %d min_count %d extra_count %d\n",
			__func__, fmt->fmt.meta.buffersize,
			inst->buffers.input_meta.min_count,
			inst->buffers.input_meta.extra_count);
	} else if (f->type == OUTPUT_PLANE) {
		if (inst->state == MSM_VIDC_START_OUTPUT) {
			d_vpr_e("%s: invalid state %d\n", __func__, inst->state);
			return -EINVAL;
		}
		fmt = &inst->fmts[OUTPUT_PORT];
		if (fmt->fmt.pix.pixelformat != f->fmt.pix.pixelformat) {
			s_vpr_e(inst->sid,
				"%s: codec changed from %#x to %#x\n", __func__,
				fmt->fmt.pix.pixelformat, f->fmt.pix.pixelformat);
			rc = msm_venc_codec_change(inst, f->fmt.pix.pixelformat);
			if (rc)
				goto err_invalid_fmt;
		}
		fmt->type = OUTPUT_PLANE;
		fmt->fmt.pix.width = ALIGN(f->fmt.pix.width, 16);
		fmt->fmt.pix.height = ALIGN(f->fmt.pix.height, 16);
		fmt->fmt.pix.pixelformat = f->fmt.pix.pixelformat;
		fmt->fmt.pix.bytesperline = 0;
		fmt->fmt.pix.sizeimage = call_session_op(core, buffer_size,
				inst, MSM_VIDC_BUF_OUTPUT);
		inst->buffers.output.min_count =
			call_session_op(core, min_count, inst, MSM_VIDC_BUF_OUTPUT);
		inst->buffers.output.extra_count =
			call_session_op(core, extra_count, inst, MSM_VIDC_BUF_OUTPUT);
		if (inst->buffers.output.actual_count <
			inst->buffers.output.min_count +
			inst->buffers.output.extra_count) {
			inst->buffers.output.actual_count =
				inst->buffers.output.min_count +
				inst->buffers.output.extra_count;
		}
		inst->buffers.output.size = fmt->fmt.pix.sizeimage;

		//rc = msm_vidc_check_session_supported(inst);
		if (rc)
			goto err_invalid_fmt;

		/* update crop dimensions */
		inst->crop.x = inst->crop.y = 0;
		inst->crop.width = f->fmt.pix.width;
		inst->crop.height = f->fmt.pix.height;

		//update_log_ctxt(inst->sid, inst->session_type,
		//	mplane->pixelformat);

		s_vpr_h(inst->sid,
			"%s: output: format %#x width %d height %d size %d min_count %d extra_count %d\n",
			__func__, fmt->fmt.pix.pixelformat, fmt->fmt.pix.width,
			fmt->fmt.pix.height, fmt->fmt.pix.sizeimage,
			inst->buffers.output.min_count,
			inst->buffers.output.extra_count);
	} else if (f->type == OUTPUT_META_PLANE) {
		if (inst->state == MSM_VIDC_START_OUTPUT) {
			d_vpr_e("%s: invalid state %d\n", __func__, inst->state);
			return -EINVAL;
		}
		fmt = &inst->fmts[OUTPUT_META_PORT];
		fmt->type = OUTPUT_META_PLANE;
		fmt->fmt.meta.dataformat = V4L2_PIX_FMT_VIDC_META;
		fmt->fmt.meta.buffersize = call_session_op(core, buffer_size,
				inst, MSM_VIDC_BUF_OUTPUT_META);
		inst->buffers.output_meta.min_count =
				inst->buffers.output.min_count;
		inst->buffers.output_meta.extra_count =
				inst->buffers.output.extra_count;
		inst->buffers.output_meta.actual_count =
				inst->buffers.output.actual_count;
		inst->buffers.output_meta.size = fmt->fmt.meta.buffersize;
		s_vpr_h(inst->sid,
			"%s: output meta: size %d min_count %d extra_count %d\n",
			__func__, fmt->fmt.meta.buffersize,
			inst->buffers.output_meta.min_count,
			inst->buffers.output_meta.extra_count);
	} else {
		s_vpr_e(inst->sid, "%s: invalid type %d\n", __func__, f->type);
		goto err_invalid_fmt;
	}
	memcpy(f, fmt, sizeof(struct v4l2_format));

err_invalid_fmt:
	return rc;
}

int msm_venc_g_fmt(struct msm_vidc_inst *inst, struct v4l2_format *f)
{
	int rc = 0;
	int port;

	d_vpr_h("%s()\n", __func__);
	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	port = msm_vidc_get_port_from_v4l2_type(inst, f->type, __func__);
	if (port < 0)
		return -EINVAL;

	memcpy(f, &inst->fmts[port], sizeof(struct v4l2_format));

	return rc;
}

int msm_venc_enum_fmt(struct msm_vidc_inst *inst, struct v4l2_fmtdesc *f)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 array[32] = {0};
	u32 i = 0, idx = 0;

	if (!inst || !inst->core || !inst->capabilities || !f) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	if (f->type == OUTPUT_PLANE) {
		u32 codecs = core->capabilities[DEC_CODECS].value;

		while (codecs) {
			if (idx > 31)
				break;
			if (codecs & BIT(i)) {
				array[idx] = codecs & BIT(i);
				idx++;
			}
			i++;
			codecs >>= 1;
		}
		f->pixelformat = get_v4l2_codec_from_vidc(array[f->index]);
		if (!f->pixelformat)
			return -EINVAL;
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
		strlcpy(f->description, "codec", sizeof(f->description));
	} else if (f->type == INPUT_PLANE) {
		u32 formats = inst->capabilities->cap[PIX_FMTS].step_or_mask;

		while (formats) {
			if (idx > 31)
				break;
			if (formats & BIT(i)) {
				array[idx] = formats & BIT(i);
				idx++;
			}
			i++;
			formats >>= 1;
		}
		f->pixelformat = get_v4l2_colorformat_from_vidc(array[f->index]);
		if (!f->pixelformat)
			return -EINVAL;
		strlcpy(f->description, "colorformat", sizeof(f->description));
	} else if (f->type == INPUT_META_PLANE || f->type == OUTPUT_META_PLANE) {
		if (!f->index) {
			f->pixelformat = V4L2_PIX_FMT_VIDC_META;
			strlcpy(f->description, "metadata", sizeof(f->description));
		} else {
			return -EINVAL;
		}
	}
	memset(f->reserved, 0, sizeof(f->reserved));

	s_vpr_h(inst->sid, "%s: index %d, %s : %#x, flags %#x\n",
		__func__, f->index, f->description, f->pixelformat, f->flags);
	return rc;
}

int msm_venc_inst_init(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct v4l2_format *f;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	f = &inst->fmts[OUTPUT_PORT];
	f->type = OUTPUT_PLANE;
	f->fmt.pix.width = DEFAULT_WIDTH;
	f->fmt.pix.height = DEFAULT_HEIGHT;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
	f->fmt.pix.bytesperline = 0;
	f->fmt.pix.sizeimage = call_session_op(core, buffer_size,
			inst, MSM_VIDC_BUF_OUTPUT);
	inst->buffers.output.min_count =
			call_session_op(core, min_count, inst, MSM_VIDC_BUF_OUTPUT);
	inst->buffers.output.extra_count =
			call_session_op(core, extra_count, inst, MSM_VIDC_BUF_OUTPUT);
	inst->buffers.output.actual_count =
			inst->buffers.output.min_count +
			inst->buffers.output.extra_count;
	inst->buffers.output.size = f->fmt.pix.sizeimage;

	inst->crop.x = inst->crop.y = 0;
	inst->crop.width = f->fmt.pix.width;
	inst->crop.height = f->fmt.pix.height;

	f = &inst->fmts[OUTPUT_META_PORT];
	f->type = OUTPUT_META_PLANE;
	f->fmt.meta.dataformat = V4L2_PIX_FMT_VIDC_META;
	f->fmt.meta.buffersize = call_session_op(core, buffer_size,
			inst, MSM_VIDC_BUF_OUTPUT_META);
	inst->buffers.output_meta.min_count = inst->buffers.output.min_count;
	inst->buffers.output_meta.extra_count = inst->buffers.output.extra_count;
	inst->buffers.output_meta.actual_count = inst->buffers.output.actual_count;
	inst->buffers.output_meta.size = f->fmt.meta.buffersize;			

	f = &inst->fmts[INPUT_PORT];
	f->type = INPUT_PLANE;
	f->fmt.pix.pixelformat = V4L2_PIX_FMT_NV12_UBWC;
	f->fmt.pix.width = VENUS_Y_STRIDE(
		get_media_colorformat_from_v4l2(f->fmt.pix.pixelformat), DEFAULT_WIDTH);
	f->fmt.pix.height = VENUS_Y_SCANLINES(
		get_media_colorformat_from_v4l2(f->fmt.pix.pixelformat), DEFAULT_HEIGHT);
	f->fmt.pix.bytesperline = f->fmt.pix.width;
	f->fmt.pix.sizeimage = call_session_op(core, buffer_size,
			inst, MSM_VIDC_BUF_INPUT);
	inst->buffers.input.min_count =
			call_session_op(core, min_count, inst, MSM_VIDC_BUF_INPUT);
	inst->buffers.input.extra_count =
			call_session_op(core, extra_count, inst, MSM_VIDC_BUF_INPUT);
	inst->buffers.input.actual_count =
			inst->buffers.input.min_count +
			inst->buffers.input.extra_count;
	inst->buffers.input.size = f->fmt.pix.sizeimage;

	f = &inst->fmts[INPUT_META_PORT];
	f->type = INPUT_META_PLANE;
	f->fmt.meta.dataformat = V4L2_PIX_FMT_VIDC_META;
	f->fmt.meta.buffersize = call_session_op(core, buffer_size,
			inst, MSM_VIDC_BUF_INPUT_META);
	inst->buffers.input_meta.min_count = inst->buffers.input.min_count;
	inst->buffers.input_meta.extra_count = inst->buffers.input.extra_count;
	inst->buffers.input_meta.actual_count = inst->buffers.input.actual_count;
	inst->buffers.input_meta.size = f->fmt.meta.buffersize;

	inst->prop.frame_rate = DEFAULT_FPS << 16;
	inst->prop.operating_rate = DEFAULT_FPS << 16;

	rc = msm_venc_codec_change(inst,
			inst->fmts[OUTPUT_PORT].fmt.pix.pixelformat);

	return rc;
}
