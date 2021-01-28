// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <media/v4l2_vidc_extensions.h>
#include "msm_media_info.h"
#include <linux/v4l2-common.h>

#include "msm_vdec.h"
#include "msm_vidc_core.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_driver.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_platform.h"
#include "msm_vidc_control.h"
#include "msm_vidc_debug.h"
#include "msm_vidc_control.h"
#include "venus_hfi.h"
#include "hfi_packet.h"

u32 msm_vdec_subscribe_for_port_settings_change[] = {
	HFI_PROP_BITSTREAM_RESOLUTION,
	HFI_PROP_CROP_OFFSETS,
	HFI_PROP_LUMA_CHROMA_BIT_DEPTH,
	HFI_PROP_CODED_FRAMES,
	HFI_PROP_BUFFER_FW_MIN_OUTPUT_COUNT,
	HFI_PROP_PIC_ORDER_CNT_TYPE,
	HFI_PROP_SIGNAL_COLOR_INFO,
	HFI_PROP_PROFILE,
	HFI_PROP_LEVEL,
	HFI_PROP_TIER,
};

u32 msm_vdec_subscribe_for_properties[] = {
	HFI_PROP_NO_OUTPUT,
	HFI_PROP_CABAC_SESSION,
};

static int msm_vdec_codec_change(struct msm_vidc_inst *inst, u32 v4l2_codec)
{
	int rc = 0;

	if (inst->codec && inst->fmts[INPUT_PORT].fmt.pix_mp.pixelformat == v4l2_codec)
		return 0;

	s_vpr_h(inst->sid, "%s: codec changed from %#x to %#x\n",
		__func__, inst->fmts[INPUT_PORT].fmt.pix_mp.pixelformat, v4l2_codec);

	inst->codec = v4l2_codec_to_driver(v4l2_codec, __func__);
	rc = msm_vidc_get_inst_capability(inst);
	if (rc)
		goto exit;

	rc = msm_vidc_ctrl_deinit(inst);
	if (rc)
		goto exit;

	rc = msm_vidc_ctrl_init(inst);
	if(rc)
		goto exit;

exit:
	return rc;
}

static int msm_vdec_set_bitstream_resolution(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 resolution;

	resolution = inst->fmts[INPUT_PORT].fmt.pix_mp.width << 16 |
		inst->fmts[INPUT_PORT].fmt.pix_mp.height;
	s_vpr_h(inst->sid, "%s: width: %d height: %d\n", __func__,
			inst->fmts[INPUT_PORT].fmt.pix_mp.width,
			inst->fmts[INPUT_PORT].fmt.pix_mp.height);
	inst->subcr_params[port].bitstream_resolution = resolution;
	rc = venus_hfi_session_property(inst,
			HFI_PROP_BITSTREAM_RESOLUTION,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&resolution,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_linear_stride_scanline(struct msm_vidc_inst *inst)
{
	int rc = 0;
	u32 stride_y, scanline_y, stride_uv, scanline_uv;
	u32 payload[2];

	if (inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat !=
		V4L2_PIX_FMT_NV12 &&
		inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat !=
		V4L2_PIX_FMT_VIDC_P010)
		return 0;

	stride_y = inst->fmts[OUTPUT_PORT].fmt.pix_mp.width;
	scanline_y = inst->fmts[OUTPUT_PORT].fmt.pix_mp.height;
	stride_uv = stride_y;
	scanline_uv = scanline_y / 2;

	payload[0] = stride_y << 16 | scanline_y;
	payload[1] = stride_uv << 16 | scanline_uv;
	s_vpr_h(inst->sid, "%s: stride_y: %d scanline_y: %d "
		"stride_uv: %d, scanline_uv: %d", __func__,
		stride_y, scanline_y, stride_uv, scanline_uv);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_LINEAR_ALIGNMENT_FACTOR,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, OUTPUT_PORT),
			HFI_PAYLOAD_U64,
			&payload,
			sizeof(u64));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_crop_offsets(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 left_offset, top_offset, right_offset, bottom_offset;
	u64 payload;

	if (inst->fmts[INPUT_PORT].fmt.pix_mp.width <
		inst->crop.width)
		return -EINVAL;

	if (inst->fmts[INPUT_PORT].fmt.pix_mp.height <
		inst->crop.height)
		return -EINVAL;

	left_offset = inst->crop.left;
	top_offset = inst->crop.top;
	right_offset = (inst->fmts[INPUT_PORT].fmt.pix_mp.width -
		inst->crop.width);
	bottom_offset = (inst->fmts[INPUT_PORT].fmt.pix_mp.height -
		inst->crop.height);

	payload = (u64)right_offset << 48 | (u64)bottom_offset << 32 |
		(u64)left_offset << 16 | top_offset;
	s_vpr_h(inst->sid, "%s: left_offset: %d top_offset: %d "
		"right_offset: %d bottom_offset: %d", __func__,
		left_offset, top_offset, right_offset, bottom_offset);
	inst->subcr_params[port].crop_offsets = payload;
	rc = venus_hfi_session_property(inst,
			HFI_PROP_CROP_OFFSETS,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_64_PACKED,
			&payload,
			sizeof(u64));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_bit_depth(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 colorformat;
	u32 bitdepth = 8 << 16 | 8;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	colorformat = inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat;
	if (colorformat == V4L2_PIX_FMT_VIDC_P010 ||
	    colorformat == V4L2_PIX_FMT_VIDC_TP10C)
		bitdepth = 10 << 16 | 10;

	inst->subcr_params[port].bit_depth = bitdepth;
	inst->capabilities->cap[BIT_DEPTH].value = bitdepth;
	s_vpr_h(inst->sid, "%s: bit depth: %d", __func__, bitdepth);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_LUMA_CHROMA_BIT_DEPTH,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&bitdepth,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}
//todo: enable when needed
/*
static int msm_vdec_set_cabac(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 cabac = 0;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	cabac = inst->capabilities->cap[ENTROPY_MODE].value;
	inst->subcr_params[port].cabac = cabac;
	s_vpr_h(inst->sid, "%s: entropy mode: %d", __func__, cabac);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_CABAC_SESSION,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&cabac,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}
*/
static int msm_vdec_set_coded_frames(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 coded_frames;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	coded_frames = inst->capabilities->cap[CODED_FRAMES].value;
	inst->subcr_params[port].coded_frames = coded_frames;
	s_vpr_h(inst->sid, "%s: coded frames: %d", __func__, coded_frames);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_CODED_FRAMES,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&coded_frames,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_min_output_count(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 min_output;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	min_output = inst->buffers.output.min_count;
	inst->subcr_params[port].fw_min_count = min_output;
	s_vpr_h(inst->sid, "%s: firmware min output count: %d",
		__func__, min_output);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_BUFFER_FW_MIN_OUTPUT_COUNT,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&min_output,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_picture_order_count(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 poc = 0;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	inst->subcr_params[port].pic_order_cnt = poc;
	s_vpr_h(inst->sid, "%s: picture order count: %d", __func__, poc);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_PIC_ORDER_CNT_TYPE,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&poc,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_colorspace(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 colorspace, xfer_func, ycbcr_enc, color_info;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	colorspace = inst->fmts[OUTPUT_PORT].fmt.pix_mp.colorspace;
	xfer_func = inst->fmts[OUTPUT_PORT].fmt.pix_mp.xfer_func;
	ycbcr_enc = inst->fmts[OUTPUT_PORT].fmt.pix_mp.ycbcr_enc;

	color_info = ((ycbcr_enc << 16) & 0xFF0000) |
		((xfer_func << 8) & 0xFF00) | (colorspace & 0xFF);
	inst->subcr_params[port].color_info = color_info;
	s_vpr_h(inst->sid, "%s: color info: %d", __func__, color_info);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_SIGNAL_COLOR_INFO,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&color_info,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_profile(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 profile;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	profile = inst->capabilities->cap[PROFILE].value;
	inst->subcr_params[port].profile = profile;
	s_vpr_h(inst->sid, "%s: profile: %d", __func__, profile);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_PROFILE,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32_ENUM,
			&profile,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_level(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 level;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	level = inst->capabilities->cap[LEVEL].value;
	inst->subcr_params[port].level = level;
	s_vpr_h(inst->sid, "%s: level: %d", __func__, level);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_LEVEL,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32_ENUM,
			&level,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_tier(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 tier;

	if (port != INPUT_PORT && port != OUTPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	tier = inst->capabilities->cap[HEVC_TIER].value;
	inst->subcr_params[port].tier = tier;
	s_vpr_h(inst->sid, "%s: tier: %d", __func__, tier);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_TIER,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32_ENUM,
			&tier,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_colorformat(struct msm_vidc_inst *inst)
{
	int rc = 0;
	u32 pixelformat;
	enum msm_vidc_colorformat_type colorformat;
	u32 hfi_colorformat;

	pixelformat = inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat;
	colorformat = v4l2_colorformat_to_driver(pixelformat, __func__);
	hfi_colorformat = get_hfi_colorformat(inst, colorformat);
	s_vpr_h(inst->sid, "%s: hfi colorformat: %d",
		__func__, hfi_colorformat);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_COLOR_FORMAT,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, OUTPUT_PORT),
			HFI_PAYLOAD_U32,
			&hfi_colorformat,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_stage(struct msm_vidc_inst *inst)
{
	int rc = 0;
	u32 stage = 0;
	struct msm_vidc_core *core = inst->core;
	struct msm_vidc_inst_capability *capability = inst->capabilities;

	rc = call_session_op(core, decide_work_mode, inst);
	if (rc) {
		s_vpr_e(inst->sid, "%s: decide_work_mode failed %d\n",
			__func__);
		return -EINVAL;
	}

	stage = capability->cap[STAGE].value;
	s_vpr_h(inst->sid, "%s: stage: %d", __func__, stage);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_STAGE,
			HFI_HOST_FLAGS_NONE,
			HFI_PORT_NONE,
			HFI_PAYLOAD_U32,
			&stage,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_pipe(struct msm_vidc_inst *inst)
{
	int rc = 0;
	u32 pipe;
	struct msm_vidc_core *core = inst->core;
	struct msm_vidc_inst_capability *capability = inst->capabilities;

	rc = call_session_op(core, decide_work_route, inst);
	if (rc) {
		s_vpr_e(inst->sid, "%s: decide_work_route failed\n",
			__func__);
		return -EINVAL;
	}

	pipe = capability->cap[PIPE].value;
	s_vpr_h(inst->sid, "%s: pipe: %d", __func__, pipe);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_PIPE,
			HFI_HOST_FLAGS_NONE,
			HFI_PORT_NONE,
			HFI_PAYLOAD_U32,
			&pipe,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_output_order(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 output_order;

	if (port != INPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	output_order = inst->capabilities->cap[DISPLAY_DELAY_ENABLE].value;
	s_vpr_h(inst->sid, "%s: output order: %d", __func__, output_order);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_DECODE_ORDER_OUTPUT,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&output_order,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_secure_mode(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 secure_mode;

	secure_mode = inst->capabilities->cap[SECURE_MODE].value;
	s_vpr_h(inst->sid, "%s: secure mode: %d", __func__, secure_mode);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_SECURE,
			HFI_HOST_FLAGS_NONE,
			HFI_PORT_NONE,
			HFI_PAYLOAD_U32,
			&secure_mode,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_thumbnail_mode(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 thumbnail_mode = 0;

	if (port != INPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	s_vpr_h(inst->sid, "%s: thumbnail mode: %d", __func__, thumbnail_mode);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_THUMBNAIL_MODE,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&thumbnail_mode,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_realtime(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 realtime = 1;  //todo

	if (port != INPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	s_vpr_h(inst->sid, "%s: priority: %d", __func__, realtime);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_REALTIME,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_U32,
			&realtime,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_conceal_color_8bit(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 conceal_color_8bit;

	if (port != INPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	conceal_color_8bit = inst->capabilities->cap[CONCEAL_COLOR_8BIT].value;
	s_vpr_h(inst->sid, "%s: conceal color 8bit: %#x",
		__func__, conceal_color_8bit);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_CONCEAL_COLOR_8BIT,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_32_PACKED,
			&conceal_color_8bit,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_conceal_color_10bit(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 conceal_color_10bit;

	if (port != INPUT_PORT) {
		s_vpr_e(inst->sid, "%s: invalid port %d\n", __func__, port);
		return -EINVAL;
	}

	conceal_color_10bit = inst->capabilities->cap[CONCEAL_COLOR_8BIT].value;
	s_vpr_h(inst->sid, "%s: conceal color 10bit: %#x",
		__func__, conceal_color_10bit);
	rc = venus_hfi_session_property(inst,
			HFI_PROP_CONCEAL_COLOR_10BIT,
			HFI_HOST_FLAGS_NONE,
			get_hfi_port(inst, port),
			HFI_PAYLOAD_32_PACKED,
			&conceal_color_10bit,
			sizeof(u32));
	if (rc)
		s_vpr_e(inst->sid, "%s: set property failed\n", __func__);

	return rc;
}

static int msm_vdec_set_input_properties(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vdec_set_output_order(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vdec_set_secure_mode(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vdec_set_thumbnail_mode(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vdec_set_realtime(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vdec_set_conceal_color_8bit(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vdec_set_conceal_color_10bit(inst, INPUT_PORT);
	if (rc)
		return rc;

	return rc;
}

static int msm_vdec_set_output_properties(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vdec_set_colorformat(inst);
	if (rc)
		return rc;

	rc = msm_vdec_set_stage(inst);
	if (rc)
		return rc;

	rc = msm_vdec_set_pipe(inst);
	if (rc)
		return rc;

	rc = msm_vdec_set_linear_stride_scanline(inst);
	if (rc)
		return rc;

	return rc;
}

static int msm_vdec_get_input_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_get_internal_buffers(inst, MSM_VIDC_BUF_BIN);
	if (rc)
		return rc;

	rc = msm_vidc_get_internal_buffers(inst, MSM_VIDC_BUF_COMV);
	if (rc)
		return rc;

	rc = msm_vidc_get_internal_buffers(inst, MSM_VIDC_BUF_NON_COMV);
	if (rc)
		return rc;

	rc = msm_vidc_get_internal_buffers(inst, MSM_VIDC_BUF_LINE);
	if (rc)
		return rc;

	rc = msm_vidc_get_internal_buffers(inst, MSM_VIDC_BUF_PERSIST);
	if (rc)
		return rc;

	s_vpr_h(inst->sid, "input internal buffer: min     size     reuse\n");
	s_vpr_h(inst->sid, "bin  buffer: %d      %d      %d\n",
		inst->buffers.bin.min_count,
		inst->buffers.bin.size,
		inst->buffers.bin.reuse);
	s_vpr_h(inst->sid, "comv  buffer: %d      %d      %d\n",
		inst->buffers.comv.min_count,
		inst->buffers.comv.size,
		inst->buffers.comv.reuse);
	s_vpr_h(inst->sid, "non_comv  buffer: %d      %d      %d\n",
		inst->buffers.non_comv.min_count,
		inst->buffers.non_comv.size,
		inst->buffers.non_comv.reuse);
	s_vpr_h(inst->sid, "line buffer: %d      %d      %d\n",
		inst->buffers.line.min_count,
		inst->buffers.line.size,
		inst->buffers.line.reuse);
	s_vpr_h(inst->sid, "persist buffer: %d      %d      %d\n",
		inst->buffers.persist.min_count,
		inst->buffers.persist.size,
		inst->buffers.persist.reuse);

	return rc;
}

static int msm_vdec_get_output_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_get_internal_buffers(inst, MSM_VIDC_BUF_DPB);
	if (rc)
		return rc;

	s_vpr_h(inst->sid, "output internal buffer: min     size     reuse\n");
	s_vpr_h(inst->sid, "dpb  buffer: %d      %d      %d\n",
		inst->buffers.dpb.min_count,
		inst->buffers.dpb.size,
		inst->buffers.dpb.reuse);

	return rc;
}

static int msm_vdec_create_input_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_create_internal_buffers(inst, MSM_VIDC_BUF_BIN);
	if (rc)
		return rc;
	rc = msm_vidc_create_internal_buffers(inst, MSM_VIDC_BUF_COMV);
	if (rc)
		return rc;
	rc = msm_vidc_create_internal_buffers(inst, MSM_VIDC_BUF_NON_COMV);
	if (rc)
		return rc;
	rc = msm_vidc_create_internal_buffers(inst, MSM_VIDC_BUF_LINE);
	if (rc)
		return rc;
	rc = msm_vidc_create_internal_buffers(inst, MSM_VIDC_BUF_PERSIST);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_create_output_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_create_internal_buffers(inst, MSM_VIDC_BUF_DPB);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_queue_input_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_queue_internal_buffers(inst, MSM_VIDC_BUF_BIN);
	if (rc)
		return rc;
	rc = msm_vidc_queue_internal_buffers(inst, MSM_VIDC_BUF_COMV);
	if (rc)
		return rc;
	rc = msm_vidc_queue_internal_buffers(inst, MSM_VIDC_BUF_NON_COMV);
	if (rc)
		return rc;
	rc = msm_vidc_queue_internal_buffers(inst, MSM_VIDC_BUF_LINE);
	if (rc)
		return rc;
	rc = msm_vidc_queue_internal_buffers(inst, MSM_VIDC_BUF_PERSIST);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_queue_output_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_queue_internal_buffers(inst, MSM_VIDC_BUF_DPB);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_release_input_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	s_vpr_h(inst->sid, "%s()\n",__func__);

	rc = msm_vidc_release_internal_buffers(inst, MSM_VIDC_BUF_BIN);
	if (rc)
		return rc;
	rc = msm_vidc_release_internal_buffers(inst, MSM_VIDC_BUF_COMV);
	if (rc)
		return rc;
	rc = msm_vidc_release_internal_buffers(inst, MSM_VIDC_BUF_NON_COMV);
	if (rc)
		return rc;
	rc = msm_vidc_release_internal_buffers(inst, MSM_VIDC_BUF_LINE);
	if (rc)
		return rc;
	rc = msm_vidc_release_internal_buffers(inst, MSM_VIDC_BUF_PERSIST);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_release_output_internal_buffers(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	s_vpr_h(inst->sid, "%s()\n",__func__);

	rc = msm_vidc_release_internal_buffers(inst, MSM_VIDC_BUF_DPB);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_subscribe_input_port_settings_change(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 payload[32] = {0};
	u32 i;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	d_vpr_h("%s()\n", __func__);

	payload[0] = HFI_MODE_PORT_SETTINGS_CHANGE;
	for (i = 0; i < ARRAY_SIZE(msm_vdec_subscribe_for_port_settings_change);
	     i++)
		payload[i + 1] = msm_vdec_subscribe_for_port_settings_change[i];

	rc = venus_hfi_session_command(inst,
			HFI_CMD_SUBSCRIBE_MODE,
			port,
			HFI_PAYLOAD_U32_ARRAY,
			&payload[0],
			(ARRAY_SIZE(msm_vdec_subscribe_for_port_settings_change) + 1) *
			sizeof(u32));

	for (i = 0; i < ARRAY_SIZE(msm_vdec_subscribe_for_port_settings_change);
		i++) {
		switch (msm_vdec_subscribe_for_port_settings_change[i]) {
		case HFI_PROP_BITSTREAM_RESOLUTION:
			rc = msm_vdec_set_bitstream_resolution(inst, port);
			break;
		case HFI_PROP_CROP_OFFSETS:
			rc = msm_vdec_set_crop_offsets(inst, port);
			break;
		case HFI_PROP_LUMA_CHROMA_BIT_DEPTH:
			rc = msm_vdec_set_bit_depth(inst, port);
			break;
		case HFI_PROP_CODED_FRAMES:
			rc = msm_vdec_set_coded_frames(inst, port);
			break;
		case HFI_PROP_BUFFER_FW_MIN_OUTPUT_COUNT:
			rc = msm_vdec_set_min_output_count(inst, port);
			break;
		case HFI_PROP_PIC_ORDER_CNT_TYPE:
			rc = msm_vdec_set_picture_order_count(inst, port);
			break;
		case HFI_PROP_SIGNAL_COLOR_INFO:
			rc = msm_vdec_set_colorspace(inst, port);
			break;
		case HFI_PROP_PROFILE:
			rc = msm_vdec_set_profile(inst, port);
			break;
		case HFI_PROP_LEVEL:
			rc = msm_vdec_set_level(inst, port);
			break;
		case HFI_PROP_TIER:
			rc = msm_vdec_set_tier(inst, port);
			break;
		default:
			d_vpr_e("%s: unknown property %#x\n", __func__,
				msm_vdec_subscribe_for_port_settings_change[i]);
			rc = -EINVAL;
			break;
		}
	}

	return rc;
}

static int msm_vdec_subscribe_property(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 payload[32] = {0};
	u32 i;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	d_vpr_h("%s()\n", __func__);

	payload[0] = HFI_MODE_PROPERTY;
	for (i = 0; i < ARRAY_SIZE(msm_vdec_subscribe_for_properties); i++)
		payload[i + 1] = msm_vdec_subscribe_for_properties[i];

	rc = venus_hfi_session_command(inst,
			HFI_CMD_SUBSCRIBE_MODE,
			port,
			HFI_PAYLOAD_U32_ARRAY,
			&payload[0],
			(ARRAY_SIZE(msm_vdec_subscribe_for_properties) + 1) *
			sizeof(u32));

	return rc;
}

static int msm_vdec_subscribe_metadata(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 payload[32] = {0};
	u32 i, count = 0;
	struct msm_vidc_inst_capability *capability;
	u32 metadata_list[] = {
		META_DPB_MISR,
		META_OPB_MISR,
		META_INTERLACE,
		META_TIMESTAMP,
		META_CONCEALED_MB_CNT,
		META_HIST_INFO,
		META_SEI_MASTERING_DISP,
		META_SEI_CLL,
		META_HDR10PLUS,
		META_BUF_TAG,
		META_SUBFRAME_OUTPUT,
	};

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	d_vpr_h("%s()\n", __func__);

	capability = inst->capabilities;
	payload[0] = HFI_MODE_METADATA;
	for (i = 0; i < ARRAY_SIZE(metadata_list); i++) {
		if (capability->cap[metadata_list[i]].value) {
			payload[count + 1] =
				capability->cap[metadata_list[i]].hfi_id;
			count++;
		}
	};

	if (!count)
		return 0;

	rc = venus_hfi_session_command(inst,
			HFI_CMD_SUBSCRIBE_MODE,
			port,
			HFI_PAYLOAD_U32_ARRAY,
			&payload[0],
			(count + 1) * sizeof(u32));

	return rc;
}

static int msm_vdec_set_delivery_mode_metadata(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	struct msm_vidc_core *core;
	u32 payload[32] = {0};
	u32 i, count = 0;
	struct msm_vidc_inst_capability *capability;
	u32 metadata_list[] = {
		META_BUF_TAG,
	};

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	d_vpr_h("%s()\n", __func__);

	capability = inst->capabilities;
	payload[0] = HFI_MODE_METADATA;
	for (i = 0; i < ARRAY_SIZE(metadata_list); i++) {
		if (capability->cap[metadata_list[i]].value) {
			payload[count + 1] =
				capability->cap[metadata_list[i]].hfi_id;
			count++;
		}
	};

	if (!count)
		return 0;

	rc = venus_hfi_session_command(inst,
			HFI_CMD_DELIVERY_MODE,
			port,
			HFI_PAYLOAD_U32_ARRAY,
			&payload[0],
			(count + 1) * sizeof(u32));

	return rc;
}

static int msm_vdec_session_resume(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	d_vpr_h("%s()\n", __func__);

	rc = venus_hfi_session_command(inst,
			HFI_CMD_RESUME,
			port,
			HFI_PAYLOAD_NONE,
			NULL,
			0);

	return rc;
}

static int msm_vdec_update_properties(struct msm_vidc_inst *inst)
{
	struct msm_vidc_subscription_params subsc_params;
	struct msm_vidc_core *core;
	u32 width, height;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	subsc_params = inst->subcr_params[INPUT_PORT];
	width = (subsc_params.bitstream_resolution &
		HFI_BITMASK_BITSTREAM_WIDTH) >> 16;
	height = subsc_params.bitstream_resolution &
		HFI_BITMASK_BITSTREAM_HEIGHT;

	inst->fmts[INPUT_PORT].fmt.pix_mp.width = width;
	inst->fmts[INPUT_PORT].fmt.pix_mp.height = height;

	inst->fmts[OUTPUT_PORT].fmt.pix_mp.width = VENUS_Y_STRIDE(
		v4l2_colorformat_to_media(
		inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat, __func__),
		width);
	inst->fmts[OUTPUT_PORT].fmt.pix_mp.height = VENUS_Y_SCANLINES(
		v4l2_colorformat_to_media(
		inst->fmts[OUTPUT_PORT].fmt.pix_mp.pixelformat,	__func__),
		height);
	inst->fmts[OUTPUT_PORT].fmt.pix_mp.plane_fmt[0].bytesperline =
		inst->fmts[OUTPUT_PORT].fmt.pix_mp.width;
	inst->fmts[OUTPUT_PORT].fmt.pix_mp.plane_fmt[0].sizeimage =
		call_session_op(core, buffer_size, inst, MSM_VIDC_BUF_OUTPUT);
	//inst->buffers.output.size = inst->fmts[OUTPUT_PORT].fmt.pix_mp.plane_fmt[0].sizeimage;

	inst->fmts[OUTPUT_PORT].fmt.pix_mp.colorspace =
		(subsc_params.color_info & 0xFF0000) >> 16;
	inst->fmts[OUTPUT_PORT].fmt.pix_mp.xfer_func =
		(subsc_params.color_info & 0xFF00) >> 8;
	inst->fmts[OUTPUT_PORT].fmt.pix_mp.ycbcr_enc =
		subsc_params.color_info & 0xFF;

	inst->buffers.output.min_count = subsc_params.fw_min_count;

	inst->crop.top = subsc_params.crop_offsets & 0xFFFF;
	inst->crop.left = (subsc_params.crop_offsets >> 16) & 0xFFFF;
	inst->crop.height = inst->fmts[INPUT_PORT].fmt.pix_mp.height -
		((subsc_params.crop_offsets >> 32) & 0xFFFF);
	inst->crop.width = inst->fmts[INPUT_PORT].fmt.pix_mp.width -
		((subsc_params.crop_offsets >> 48) & 0xFFFF);

	inst->capabilities->cap[PROFILE].value = subsc_params.profile;
	inst->capabilities->cap[LEVEL].value = subsc_params.level;
	inst->capabilities->cap[HEVC_TIER].value = subsc_params.tier;
	inst->capabilities->cap[POC].value = subsc_params.pic_order_cnt;
	inst->capabilities->cap[BIT_DEPTH].value = subsc_params.bit_depth;
	inst->capabilities->cap[CODED_FRAMES].value = subsc_params.coded_frames;

	return 0;
}

int msm_vdec_input_port_settings_change(struct msm_vidc_inst *inst)
{
	u32 rc = 0;
	struct v4l2_event event = {0};

	if (!inst->vb2q[INPUT_PORT].streaming) {
		s_vpr_e(inst->sid, "%s: input port not streaming\n",
			__func__);
		return 0;
	}

	rc = msm_vdec_update_properties(inst);
	if (rc)
		return rc;

	event.type = V4L2_EVENT_SOURCE_CHANGE;
	event.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION;
	v4l2_event_queue_fh(&inst->event_handler, &event);

	rc = msm_vdec_get_input_internal_buffers(inst);
	if (rc)
		return rc;

	rc = msm_vdec_release_input_internal_buffers(inst);
	if (rc)
		return rc;

	rc = msm_vdec_create_input_internal_buffers(inst);
	if (rc)
		return rc;

	rc = msm_vdec_queue_input_internal_buffers(inst);
	if (rc)
		return rc;

	rc = msm_vdec_session_resume(inst, INPUT_PORT);
	if (rc)
		return rc;

	return rc;
}

int msm_vdec_output_port_settings_change(struct msm_vidc_inst *inst)
{
	//todo
	return 0;
}

int msm_vdec_streamoff_input(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_session_streamoff(inst, INPUT_PORT);
	if (rc)
		return rc;

	return 0;
}

int msm_vdec_streamon_input(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (is_input_meta_enabled(inst) &&
		!inst->vb2q[INPUT_META_PORT].streaming) {
		s_vpr_e(inst->sid,
			"%s: Meta port must be streamed on before data port\n",
			__func__);
		return -EINVAL;
	}

	//rc = msm_vidc_check_session_supported(inst);
	if (rc)
		goto error;
	//rc = msm_vidc_check_scaling_supported(inst);
	if (rc)
		goto error;

	rc = msm_vdec_set_input_properties(inst);
	if (rc)
		goto error;

	/* Decide bse vpp delay after work mode */
	//msm_vidc_set_bse_vpp_delay(inst);

	rc = msm_vdec_get_input_internal_buffers(inst);
	if (rc)
		goto error;
	/* check for memory after all buffers calculation */
	//rc = msm_vidc_check_memory_supported(inst);
	if (rc)
		goto error;

	//msm_vidc_update_dcvs(inst);
	//msm_vidc_update_batching(inst);
	//msm_vidc_scale_power(inst);

	rc = msm_vdec_create_input_internal_buffers(inst);
	if (rc)
		goto error;

	rc = msm_vdec_queue_input_internal_buffers(inst);
	if (rc)
		goto error;

	if (!inst->ipsc_properties_set) {
		rc = msm_vdec_subscribe_input_port_settings_change(
			inst, INPUT_PORT);
		if (rc)
			return rc;
		inst->ipsc_properties_set = true;
	}

	rc = msm_vdec_subscribe_property(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vdec_set_delivery_mode_metadata(inst, INPUT_PORT);
	if (rc)
		return rc;

	rc = msm_vidc_session_streamon(inst, INPUT_PORT);
	if (rc)
		goto error;

	return 0;

error:
	s_vpr_e(inst->sid, "%s: failed\n", __func__);
	msm_vdec_streamoff_input(inst);
	return rc;
}

int msm_vdec_streamoff_output(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_session_streamoff(inst, OUTPUT_PORT);
	if (rc)
		return rc;

	return 0;
}

static int msm_vdec_subscribe_output_port_settings_change(struct msm_vidc_inst *inst,
	enum msm_vidc_port_type port)
{
	int rc = 0;
	u32 payload[32] = {0};
	u32 prop_type, payload_size, payload_type;
	u32 i;
	struct msm_vidc_subscription_params subsc_params;

	d_vpr_h("%s()\n", __func__);

	payload[0] = HFI_MODE_PORT_SETTINGS_CHANGE;
	for (i = 0; i < ARRAY_SIZE(msm_vdec_subscribe_for_port_settings_change);
	     i++)
		payload[i + 1] = msm_vdec_subscribe_for_port_settings_change[i];

	rc = venus_hfi_session_command(inst,
			HFI_CMD_SUBSCRIBE_MODE,
			port,
			HFI_PAYLOAD_U32_ARRAY,
			&payload[0],
			(ARRAY_SIZE(msm_vdec_subscribe_for_port_settings_change) + 1) *
			sizeof(u32));

	subsc_params = inst->subcr_params[port];
	for (i = 0; i < ARRAY_SIZE(msm_vdec_subscribe_for_port_settings_change);
		i++) {
		payload[0] = 0;
		payload[1] = 0;
		payload_size = 0;
		payload_type = 0;
		prop_type = msm_vdec_subscribe_for_port_settings_change[i];
		switch (prop_type) {
		case HFI_PROP_BITSTREAM_RESOLUTION:
			payload[0] = subsc_params.bitstream_resolution;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_CROP_OFFSETS:
			payload[0] = subsc_params.crop_offsets >> 32;
			payload[1] = subsc_params.crop_offsets;
			payload_size = sizeof(u64);
			payload_type = HFI_PAYLOAD_64_PACKED;
			break;
		case HFI_PROP_LUMA_CHROMA_BIT_DEPTH:
			payload[0] = subsc_params.bit_depth;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_CODED_FRAMES:
			payload[0] = subsc_params.coded_frames;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_BUFFER_FW_MIN_OUTPUT_COUNT:
			payload[0] = subsc_params.fw_min_count;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_PIC_ORDER_CNT_TYPE:
			payload[0] = subsc_params.pic_order_cnt;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_SIGNAL_COLOR_INFO:
			payload[0] = subsc_params.color_info;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_PROFILE:
			payload[0] = subsc_params.profile;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_LEVEL:
			payload[0] = subsc_params.level;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		case HFI_PROP_TIER:
			payload[0] = subsc_params.tier;
			payload_size = sizeof(u32);
			payload_type = HFI_PAYLOAD_U32;
			break;
		default:
			d_vpr_e("%s: unknown property %#x\n", __func__,
				prop_type);
			prop_type = 0;
			rc = -EINVAL;
			break;
		}
		if (prop_type) {
			rc = venus_hfi_session_property(inst,
					prop_type,
					HFI_HOST_FLAGS_NONE,
					get_hfi_port(inst, port),
					payload_type,
					&payload,
					payload_size);
			if (rc)
				return rc;
		}
	}

	return rc;
}

int msm_vdec_streamon_output(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (is_output_meta_enabled(inst) &&
		!inst->vb2q[OUTPUT_META_PORT].streaming) {
		s_vpr_e(inst->sid,
			"%s: Meta port must be streamed on before data port\n",
			__func__);
		return -EINVAL;
	}

	rc = msm_vdec_set_output_properties(inst);
	if (rc)
		goto error;

	if (!inst->opsc_properties_set) {
		memcpy(&inst->subcr_params[OUTPUT_PORT],
			   &inst->subcr_params[INPUT_PORT],
			   sizeof(inst->subcr_params[INPUT_PORT]));
		rc = msm_vdec_subscribe_output_port_settings_change(inst, OUTPUT_PORT);
		if (rc)
			goto error;
		inst->opsc_properties_set = true;
	}

	rc = msm_vdec_subscribe_metadata(inst, OUTPUT_PORT);
	if (rc)
		goto error;

	rc = msm_vdec_get_output_internal_buffers(inst);
	if (rc)
		goto error;

	rc = msm_vdec_release_output_internal_buffers(inst);
	if (rc)
		goto error;

	rc = msm_vdec_create_output_internal_buffers(inst);
	if (rc)
		goto error;

	rc = msm_vidc_session_streamon(inst, OUTPUT_PORT);
	if (rc)
		goto error;

	rc = msm_vdec_queue_output_internal_buffers(inst);
	if (rc)
		goto error;

	return 0;

error:
	s_vpr_e(inst->sid, "%s: failed\n", __func__);
	msm_vdec_streamoff_output(inst);
	return rc;
}

static int msm_vdec_qbuf_batch(struct msm_vidc_inst *inst,
	struct vb2_buffer *vb2)
{
	int rc = 0;

	if (!inst || !vb2) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	d_vpr_h("%s()\n", __func__);

	return rc;
}

int msm_vdec_qbuf(struct msm_vidc_inst *inst, struct vb2_buffer *vb2)
{
	int rc = 0;

	if (inst->decode_batch.enable)
		rc = msm_vdec_qbuf_batch(inst, vb2);
	else
		rc = msm_vidc_queue_buffer(inst, vb2);

	return rc;
}

int msm_vdec_process_cmd(struct msm_vidc_inst *inst, u32 cmd)
{
	int rc = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (cmd == V4L2_DEC_CMD_STOP) {
		if (!msm_vidc_allow_stop(inst))
			return -EBUSY;
		rc = venus_hfi_session_command(inst,
				HFI_CMD_DRAIN,
				INPUT_PORT,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
		if (rc)
			return rc;
		rc = msm_vidc_state_change_stop(inst);
		if (rc)
			return rc;
	} else if (cmd == V4L2_DEC_CMD_START) {
		if (!msm_vidc_allow_start(inst))
			return -EBUSY;
		rc = msm_vidc_state_change_start(inst);
		if (rc)
			return rc;
		rc = venus_hfi_session_command(inst,
				HFI_CMD_RESUME,
				OUTPUT_PORT,
				HFI_PAYLOAD_NONE,
				NULL,
				0);
		if (rc)
			return rc;
		vb2_clear_last_buffer_dequeued(&inst->vb2q[OUTPUT_META_PORT]);
		vb2_clear_last_buffer_dequeued(&inst->vb2q[OUTPUT_PORT]);
	} else {
		d_vpr_e("%s: unknown cmd %d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

int msm_vdec_s_fmt(struct msm_vidc_inst *inst, struct v4l2_format *f)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct v4l2_format *fmt;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	if (f->type == INPUT_MPLANE) {
		if (inst->fmts[INPUT_PORT].fmt.pix_mp.pixelformat !=
			f->fmt.pix_mp.pixelformat) {
			s_vpr_h(inst->sid,
				"%s: codec changed from %#x to %#x\n", __func__,
				inst->fmts[INPUT_PORT].fmt.pix_mp.pixelformat,
				f->fmt.pix_mp.pixelformat);
			rc = msm_vdec_codec_change(inst, f->fmt.pix_mp.pixelformat);
			if (rc)
				goto err_invalid_fmt;
		}
		fmt = &inst->fmts[INPUT_PORT];
		fmt->type = INPUT_MPLANE;
		fmt->fmt.pix_mp.width = ALIGN(f->fmt.pix_mp.width, 16);
		fmt->fmt.pix_mp.height = ALIGN(f->fmt.pix_mp.height, 16);
		fmt->fmt.pix_mp.pixelformat = f->fmt.pix_mp.pixelformat;
		fmt->fmt.pix_mp.num_planes = 1;
		fmt->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
		fmt->fmt.pix_mp.plane_fmt[0].sizeimage = call_session_op(core,
			buffer_size, inst, MSM_VIDC_BUF_INPUT);
		inst->buffers.input.min_count = call_session_op(core,
			min_count, inst, MSM_VIDC_BUF_INPUT);
		inst->buffers.input.extra_count = call_session_op(core,
			extra_count, inst, MSM_VIDC_BUF_INPUT);
		if (inst->buffers.input.actual_count <
			inst->buffers.input.min_count +
			inst->buffers.input.extra_count) {
			inst->buffers.input.actual_count =
				inst->buffers.input.min_count +
				inst->buffers.input.extra_count;
		}
		inst->buffers.input.size =
			fmt->fmt.pix_mp.plane_fmt[0].sizeimage;

		/* update crop dimensions */
		inst->crop.left = inst->crop.top = 0;
		inst->crop.width = f->fmt.pix_mp.width;
		inst->crop.height = f->fmt.pix_mp.height;

		//rc = msm_vidc_check_session_supported(inst);
		if (rc)
			goto err_invalid_fmt;
		//update_log_ctxt(inst->sid, inst->session_type,
		//	mplane->pixelformat);
		s_vpr_h(inst->sid,
			"%s: input: codec %#x width %d height %d size %d min_count %d extra_count %d\n",
			__func__, f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width,
			f->fmt.pix_mp.height,
			fmt->fmt.pix_mp.plane_fmt[0].sizeimage,
			inst->buffers.input.min_count,
			inst->buffers.input.extra_count);

		//msm_vidc_update_dcvs(inst);
		//msm_vidc_update_batching(inst);

	} else if (f->type == INPUT_META_PLANE) {
		fmt = &inst->fmts[INPUT_META_PORT];
		fmt->type = INPUT_META_PLANE;
		fmt->fmt.meta.dataformat = V4L2_META_FMT_VIDC;
		if (is_input_meta_enabled(inst)) {
			fmt->fmt.meta.buffersize = call_session_op(core,
				buffer_size, inst, MSM_VIDC_BUF_INPUT_META);
			inst->buffers.input_meta.min_count =
					inst->buffers.input.min_count;
			inst->buffers.input_meta.extra_count =
					inst->buffers.input.extra_count;
			inst->buffers.input_meta.actual_count =
					inst->buffers.input.actual_count;
			inst->buffers.input_meta.size = fmt->fmt.meta.buffersize;
		} else {
			fmt->fmt.meta.buffersize = 0;
			inst->buffers.input_meta.min_count = 0;
			inst->buffers.input_meta.extra_count = 0;
			inst->buffers.input_meta.actual_count = 0;
			inst->buffers.input_meta.size = 0;
		}
		s_vpr_h(inst->sid,
			"%s: input meta: size %d min_count %d extra_count %d\n",
			__func__, fmt->fmt.meta.buffersize,
			inst->buffers.input_meta.min_count,
			inst->buffers.input_meta.extra_count);
	} else if (f->type == OUTPUT_MPLANE) {
		fmt = &inst->fmts[OUTPUT_PORT];
		fmt->type = OUTPUT_MPLANE;
		if (inst->vb2q[INPUT_PORT].streaming) {
			f->fmt.pix_mp.height = fmt->fmt.pix_mp.height;
			f->fmt.pix_mp.width = fmt->fmt.pix_mp.width;
		}
		fmt->fmt.pix_mp.pixelformat = f->fmt.pix_mp.pixelformat;
		fmt->fmt.pix_mp.width = VENUS_Y_STRIDE(
			v4l2_colorformat_to_media(
			fmt->fmt.pix_mp.pixelformat, __func__),
			f->fmt.pix_mp.width);
		fmt->fmt.pix_mp.height = VENUS_Y_SCANLINES(
			v4l2_colorformat_to_media(
			fmt->fmt.pix_mp.pixelformat, __func__),
			f->fmt.pix_mp.height);
		fmt->fmt.pix_mp.num_planes = 1;
		fmt->fmt.pix_mp.plane_fmt[0].bytesperline =
			fmt->fmt.pix_mp.width;
		fmt->fmt.pix_mp.plane_fmt[0].sizeimage = call_session_op(core,
			buffer_size, inst, MSM_VIDC_BUF_OUTPUT);

		if (!inst->vb2q[INPUT_PORT].streaming)
			inst->buffers.output.min_count = call_session_op(core,
				min_count, inst, MSM_VIDC_BUF_OUTPUT);
		inst->buffers.output.extra_count = call_session_op(core,
			extra_count, inst, MSM_VIDC_BUF_OUTPUT);
		if (inst->buffers.output.actual_count <
			inst->buffers.output.min_count +
			inst->buffers.output.extra_count) {
			inst->buffers.output.actual_count =
				inst->buffers.output.min_count +
				inst->buffers.output.extra_count;
		}
		inst->buffers.output.size =
			fmt->fmt.pix_mp.plane_fmt[0].sizeimage;
		inst->capabilities->cap[PIX_FMTS].value =
			v4l2_colorformat_to_driver(f->fmt.pix_mp.pixelformat, __func__);
		//rc = msm_vidc_check_session_supported(inst);
		if (rc)
			goto err_invalid_fmt;
		s_vpr_h(inst->sid,
			"%s: output: format %#x width %d height %d size %d min_count %d extra_count %d\n",
			__func__, fmt->fmt.pix_mp.pixelformat, fmt->fmt.pix_mp.width,
			fmt->fmt.pix_mp.height,
			fmt->fmt.pix_mp.plane_fmt[0].sizeimage,
			inst->buffers.output.min_count,
			inst->buffers.output.extra_count);
	} else if (f->type == OUTPUT_META_PLANE) {
		fmt = &inst->fmts[OUTPUT_META_PORT];
		fmt->type = OUTPUT_META_PLANE;
		fmt->fmt.meta.dataformat = V4L2_META_FMT_VIDC;
		if (is_output_meta_enabled(inst)) {
			fmt->fmt.meta.buffersize = call_session_op(core,
				buffer_size, inst, MSM_VIDC_BUF_OUTPUT_META);
			inst->buffers.output_meta.min_count =
					inst->buffers.output.min_count;
			inst->buffers.output_meta.extra_count =
					inst->buffers.output.extra_count;
			inst->buffers.output_meta.actual_count =
					inst->buffers.output.actual_count;
			inst->buffers.output_meta.size = fmt->fmt.meta.buffersize;
		} else {
			fmt->fmt.meta.buffersize = 0;
			inst->buffers.output_meta.min_count = 0;
			inst->buffers.output_meta.extra_count = 0;
			inst->buffers.output_meta.actual_count = 0;
			inst->buffers.output_meta.size = 0;
		}
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

int msm_vdec_g_fmt(struct msm_vidc_inst *inst, struct v4l2_format *f)
{
	int rc = 0;
	int port;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	port = v4l2_type_to_driver_port(inst, f->type, __func__);
	if (port < 0)
		return -EINVAL;

	memcpy(f, &inst->fmts[port], sizeof(struct v4l2_format));

	return rc;
}

int msm_vdec_s_selection(struct msm_vidc_inst* inst, struct v4l2_selection* s)
{
	if (!inst || !s) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	s_vpr_e(inst->sid, "%s: unsupported\n", __func__);
	return -EINVAL;
}

int msm_vdec_g_selection(struct msm_vidc_inst* inst, struct v4l2_selection* s)
{
	if (!inst || !s) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE:
	default:
		s->r.left = inst->crop.left;
		s->r.top = inst->crop.top;
		s->r.width = inst->crop.width;
		s->r.height = inst->crop.height;
		break;
	}
	s_vpr_h(inst->sid, "%s: type %d target %d, r [%d, %d, %d, %d]\n",
		__func__, s->type, s->target, s->r.top, s->r.left,
		s->r.width, s->r.height);
	return 0;
}

int msm_vdec_s_param(struct msm_vidc_inst *inst,
		struct v4l2_streamparm *s_parm)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability = NULL;
	struct v4l2_fract *timeperframe = NULL;
	u32 q16_rate, max_rate, default_rate;
	u64 us_per_frame = 0, input_rate = 0;
	bool is_frame_rate = false;

	if (!inst || !s_parm) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	if (s_parm->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		timeperframe = &s_parm->parm.output.timeperframe;
		max_rate = capability->cap[FRAME_RATE].max;
		default_rate = capability->cap[FRAME_RATE].value;
		is_frame_rate = true;
	} else {
		timeperframe = &s_parm->parm.capture.timeperframe;
		max_rate = capability->cap[OPERATING_RATE].value;
		default_rate = capability->cap[OPERATING_RATE].value;
	}

	if (!timeperframe->denominator || !timeperframe->numerator) {
		s_vpr_e(inst->sid,
			"%s: invalid rate for type %u\n",
			__func__, s_parm->type);
		input_rate = default_rate >> 16;
		goto set_default;
	}

	us_per_frame = timeperframe->numerator * (u64)USEC_PER_SEC;
	do_div(us_per_frame, timeperframe->denominator);

	if (!us_per_frame) {
		s_vpr_e(inst->sid, "%s: us_per_frame is zero\n",
			__func__);
		rc = -EINVAL;
		goto exit;
	}

	input_rate = (u64)USEC_PER_SEC;
	do_div(input_rate, us_per_frame);

	/* Check max allowed rate */
	if (input_rate > max_rate) {
		s_vpr_e(inst->sid,
			"%s: Unsupported rate %u, max_fps %u, type: %u\n",
			__func__, input_rate, max_rate, s_parm->type);
		rc = -ENOTSUPP;
		goto exit;
	}

set_default:
	q16_rate = (u32)input_rate << 16;
	s_vpr_h(inst->sid, "%s: type %u value %#x\n",
		__func__, s_parm->type, q16_rate);

	if (is_frame_rate) {
		capability->cap[FRAME_RATE].value = q16_rate;
	} else {
		capability->cap[OPERATING_RATE].value = q16_rate;
	}

exit:
	return rc;
}

int msm_vdec_g_param(struct msm_vidc_inst *inst,
		struct v4l2_streamparm *s_parm)
{
	struct msm_vidc_inst_capability *capability = NULL;
	struct v4l2_fract *timeperframe = NULL;

	if (!inst || !s_parm) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	if (s_parm->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		timeperframe = &s_parm->parm.output.timeperframe;
		timeperframe->numerator = 1;
		timeperframe->denominator =
			capability->cap[FRAME_RATE].value >> 16;
	} else {
		timeperframe = &s_parm->parm.capture.timeperframe;
		timeperframe->numerator = 1;
		timeperframe->denominator =
			capability->cap[OPERATING_RATE].value >> 16;
	}

	s_vpr_h(inst->sid, "%s: type %u, num %u denom %u\n",
		__func__, s_parm->type, timeperframe->numerator,
		timeperframe->denominator);
	return 0;
}

static int msm_vdec_check_colorformat_supported(struct msm_vidc_inst* inst,
		enum msm_vidc_colorformat_type colorformat)
{
	bool supported = true;

	/* do not reject coloformats before streamon */
	if (!inst->vb2q[INPUT_PORT].streaming)
		return true;

	/*
	 * bit_depth 8 bit supports 8 bit colorformats only
	 * bit_depth 10 bit supports 10 bit colorformats only
	 * interlace supports ubwc colorformats only
	 */
	if (inst->capabilities->cap[BIT_DEPTH].value == BIT_DEPTH_8 &&
		!is_8bit_colorformat(colorformat))
		supported = false;
	if (inst->capabilities->cap[BIT_DEPTH].value == BIT_DEPTH_10 &&
		!is_10bit_colorformat(colorformat))
		supported = false;
	if (inst->capabilities->cap[CODED_FRAMES].value ==
		CODED_FRAMES_ADAPTIVE_FIELDS &&
		!is_ubwc_colorformat(colorformat))
		supported = false;

	return supported;
}

int msm_vdec_enum_fmt(struct msm_vidc_inst *inst, struct v4l2_fmtdesc *f)
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

	if (f->type == INPUT_MPLANE) {
		u32 codecs = core->capabilities[DEC_CODECS].value;

		while (codecs) {
			if (i > 31)
				break;
			if (codecs & BIT(i)) {
				array[idx] = codecs & BIT(i);
				idx++;
			}
			i++;
		}
		f->pixelformat = v4l2_codec_from_driver(array[f->index],
				__func__);
		if (!f->pixelformat)
			return -EINVAL;
		f->flags = V4L2_FMT_FLAG_COMPRESSED;
		strlcpy(f->description, "codec", sizeof(f->description));
	} else if (f->type == OUTPUT_MPLANE) {
		u32 formats = inst->capabilities->cap[PIX_FMTS].step_or_mask;

		while (formats) {
			if (i > 31)
				break;
			if (formats & BIT(i)) {
				if (msm_vdec_check_colorformat_supported(inst,
						formats & BIT(i))) {
					array[idx] = formats & BIT(i);
					idx++;
				}
			}
			i++;
		}
		f->pixelformat = v4l2_colorformat_from_driver(array[f->index],
				__func__);
		if (!f->pixelformat)
			return -EINVAL;
		strlcpy(f->description, "colorformat", sizeof(f->description));
	} else if (f->type == INPUT_META_PLANE || f->type == OUTPUT_META_PLANE) {
		if (!f->index) {
			f->pixelformat = V4L2_META_FMT_VIDC;
			strlcpy(f->description, "metadata", sizeof(f->description));
		} else {
			return -EINVAL;
		}
	}
	memset(f->reserved, 0, sizeof(f->reserved));

	s_vpr_h(inst->sid, "%s: index %d, %s : %#x, flags %#x, driver colorfmt %#x\n",
		__func__, f->index, f->description, f->pixelformat, f->flags,
		v4l2_colorformat_to_driver(f->pixelformat, __func__));
	return rc;
}

int msm_vdec_inst_init(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_core *core;
	struct v4l2_format *f;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	INIT_DELAYED_WORK(&inst->decode_batch.work, msm_vidc_batch_handler);

	f = &inst->fmts[INPUT_PORT];
	f->type = INPUT_MPLANE;
	f->fmt.pix_mp.width = DEFAULT_WIDTH;
	f->fmt.pix_mp.height = DEFAULT_HEIGHT;
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_H264;
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = 0;
	f->fmt.pix_mp.plane_fmt[0].sizeimage = call_session_op(core,
		buffer_size, inst, MSM_VIDC_BUF_INPUT);
	inst->buffers.input.min_count = call_session_op(core,
		min_count, inst, MSM_VIDC_BUF_INPUT);
	inst->buffers.input.extra_count = call_session_op(core,
		extra_count, inst, MSM_VIDC_BUF_INPUT);
	inst->buffers.input.actual_count =
			inst->buffers.input.min_count +
			inst->buffers.input.extra_count;
	inst->buffers.input.size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	inst->crop.left = inst->crop.top = 0;
	inst->crop.width = f->fmt.pix_mp.width;
	inst->crop.height = f->fmt.pix_mp.height;

	f = &inst->fmts[INPUT_META_PORT];
	f->type = INPUT_META_PLANE;
	f->fmt.meta.dataformat = V4L2_META_FMT_VIDC;
	f->fmt.meta.buffersize = 0;
	inst->buffers.input_meta.min_count = 0;
	inst->buffers.input_meta.extra_count = 0;
	inst->buffers.input_meta.actual_count = 0;
	inst->buffers.input_meta.size = 0;

	f = &inst->fmts[OUTPUT_PORT];
	f->type = OUTPUT_MPLANE;
	f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_VIDC_NV12C;
	f->fmt.pix_mp.width = VENUS_Y_STRIDE(
		v4l2_colorformat_to_media(f->fmt.pix_mp.pixelformat, __func__),
		DEFAULT_WIDTH);
	f->fmt.pix_mp.height = VENUS_Y_SCANLINES(
		v4l2_colorformat_to_media(f->fmt.pix_mp.pixelformat, __func__),
		DEFAULT_HEIGHT);
	f->fmt.pix_mp.num_planes = 1;
	f->fmt.pix_mp.plane_fmt[0].bytesperline = f->fmt.pix_mp.width;
	f->fmt.pix_mp.plane_fmt[0].sizeimage = call_session_op(core,
		buffer_size, inst, MSM_VIDC_BUF_OUTPUT);
	inst->buffers.output.min_count = call_session_op(core,
		min_count, inst, MSM_VIDC_BUF_OUTPUT);
	inst->buffers.output.extra_count = call_session_op(core,
		extra_count, inst, MSM_VIDC_BUF_OUTPUT);
	inst->buffers.output.actual_count =
			inst->buffers.output.min_count +
			inst->buffers.output.extra_count;
	inst->buffers.output.size = f->fmt.pix_mp.plane_fmt[0].sizeimage;

	f = &inst->fmts[OUTPUT_META_PORT];
	f->type = OUTPUT_META_PLANE;
	f->fmt.meta.dataformat = V4L2_META_FMT_VIDC;
	f->fmt.meta.buffersize = 0;
	inst->buffers.output_meta.min_count = 0;
	inst->buffers.output_meta.extra_count = 0;
	inst->buffers.output_meta.actual_count = 0;
	inst->buffers.output_meta.size = 0;

	rc = msm_vdec_codec_change(inst,
			inst->fmts[INPUT_PORT].fmt.pix_mp.pixelformat);

	return rc;
}

int msm_vdec_inst_deinit(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	rc = msm_vidc_ctrl_deinit(inst);

	return rc;
}
