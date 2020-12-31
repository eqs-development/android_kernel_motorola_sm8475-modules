// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include "msm_vidc_control.h"
#include "msm_vidc_debug.h"
#include "hfi_packet.h"
#include "hfi_property.h"
#include "venus_hfi.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_driver.h"

static bool is_priv_ctrl(u32 id)
{
	if (IS_PRIV_CTRL(id))
		return true;

	/*
	 * Treat below standard controls as private because
	 * we have added custom values to the controls
	 */

	return false;
}

static const char *const mpeg_video_rate_control[] = {
	"VBR",
	"CBR",
	"CBR VFR",
	"MBR",
	"MBR VFR",
	"CQ",
	NULL,
};

static const char *const mpeg_video_stream_format[] = {
	"NAL Format Start Codes",
	"NAL Format One NAL Per Buffer",
	"NAL Format One Byte Length",
	"NAL Format Two Byte Length",
	"NAL Format Four Byte Length",
	NULL,
};

static const char *const mpeg_video_blur_types[] = {
	"Blur None",
	"Blur External",
	"Blur Adaptive",
	NULL,
};

static const char *const roi_map_type[] = {
	"None",
	"2-bit",
	"2-bit",
	NULL,
};

static u32 msm_vidc_get_port_info(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id)
{
	struct msm_vidc_inst_capability *capability = inst->capabilities;

	if (capability->cap[cap_id].flags & CAP_FLAG_INPUT_PORT &&
		capability->cap[cap_id].flags & CAP_FLAG_OUTPUT_PORT) {
		s_vpr_e(inst->sid,
			"%s: both ports enabled. Default port set: BITSTREAM\n",
			__func__);
		return HFI_PORT_BITSTREAM;
	}
	if (capability->cap[cap_id].flags & CAP_FLAG_INPUT_PORT)
		return get_hfi_port(inst, INPUT_PORT);
	else if (capability->cap[cap_id].flags & CAP_FLAG_OUTPUT_PORT)
		return get_hfi_port(inst, OUTPUT_PORT);
	else
		return HFI_PORT_NONE;
}

static const char * const * msm_vidc_get_qmenu_type(
		struct msm_vidc_inst *inst, u32 control_id)
{
	switch (control_id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		return mpeg_video_rate_control;
	case V4L2_CID_MPEG_VIDEO_HEVC_SIZE_OF_LENGTH_FIELD:
		return mpeg_video_stream_format;
	case V4L2_CID_MPEG_VIDC_VIDEO_BLUR_TYPES:
		return mpeg_video_blur_types;
	default:
		s_vpr_e(inst->sid, "%s: No available qmenu for ctrl %#x\n",
			__func__, control_id);
		return NULL;
	}
}

static const char *msm_vidc_get_priv_ctrl_name(u32 sid, u32 control_id)
{
	switch (control_id) {
	case V4L2_CID_MPEG_VIDC_SECURE:
		return "Secure Mode";
	case V4L2_CID_MPEG_VIDC_HEIC:
		return "HEIC";
	case V4L2_CID_MPEG_VIDC_LOWLATENCY_REQUEST:
		return "Low Latency Mode";
	case V4L2_CID_MPEG_VIDC_CODEC_CONFIG:
		return "Codec Config";
	case V4L2_CID_MPEG_VIDC_B_FRAME_MIN_QP:
		return "B frame Min QP";
	case V4L2_CID_MPEG_VIDC_B_FRAME_MAX_QP:
		return "B frame Max QP";
	case V4L2_CID_MPEG_VIDC_LTRCOUNT:
		return "LTR count";
	case V4L2_CID_MPEG_VIDC_USELTRFRAME:
		return "Use LTR Frame";
	case V4L2_CID_MPEG_VIDC_MARKLTRFRAME:
		return "Mark LTR Frame";
	case V4L2_CID_MPEG_VIDC_BASELAYER_PRIORITY:
		return "Baselayer Priority";
	case V4L2_CID_MPEG_VIDC_INTRA_REFRESH_PERIOD:
		return "Intra Refresh Period";
	case V4L2_CID_MPEG_VIDC_AU_DELIMITER:
		return "AU Delimiter";
	case V4L2_CID_MPEG_VIDC_TIME_DELTA_BASED_RC:
		return "Time Delta Based RC";
	case V4L2_CID_MPEG_VIDC_CONTENT_ADAPTIVE_CODING:
		return "Content Adaptive Coding";
	case V4L2_CID_MPEG_VIDC_QUALITY_BITRATE_BOOST:
		return "Quality Bitrate Boost";
	case V4L2_CID_MPEG_VIDC_VIDEO_BLUR_TYPES:
		return "Blur Types";
	case V4L2_CID_MPEG_VIDC_VIDEO_BLUR_RESOLUTION:
		return "Blur Resolution";
	case V4L2_CID_MPEG_VIDC_VIDEO_VPE_CSC_CUSTOM_MATRIX:
		return "CSC Custom Matrix";
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY:
		return "H264 Display Delay";
	case V4L2_CID_MPEG_MFC51_VIDEO_DECODER_H264_DISPLAY_DELAY_ENABLE:
		return "H264 Display Delay Enable";
	case V4L2_CID_MPEG_VIDC_METADATA_LTR_MARK_USE_DETAILS:
		return "LTR Mark Use Details Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_SEQ_HEADER_NAL:
		return "Seq Header NAL Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_DPB_LUMA_CHROMA_MISR:
		return "DPB Luma-Chroma MISR Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_OPB_LUMA_CHROMA_MISR:
		return "OPB Luma-Chroma MISR Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_INTERLACE:
		return "Interlace Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_CONCEALED_MB_COUNT:
		return "Concealed MB Count Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_HISTOGRAM_INFO:
		return "Historgram Info Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_SEI_MASTERING_DISPLAY_COLOUR:
		return "SEI Mastering Display Color Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_SEI_CONTENT_LIGHT_LEVEL:
		return "SEI Content Lighting Level Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_HDR10PLUS:
		return "HDR10PLUS Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_EVA_STATS:
		return "EVA Stats Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_BUFFER_TAG:
		return "Buffer Tag Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_SUBFRAME_OUTPUT:
		return "Subframe Output Metadata";
	case V4L2_CID_MPEG_VIDC_METADATA_ROI_INFO:
		return "ROI Info Metadata";
	default:
		s_vpr_e(sid, "%s: ctrl name not available for ctrl id %#x\n",
			__func__, control_id);
		return NULL;
	}
}

static int msm_vidc_packetize_control(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id, u32 payload_type,
	void *hfi_val, u32 payload_size, const char *func)
{
	int rc = 0;

	s_vpr_l(inst->sid,
		"%s: hfi_id: %#x, value: %#x\n", func,
		inst->capabilities->cap[cap_id].hfi_id,
		*(s64 *)hfi_val);

	rc = venus_hfi_session_property(inst,
		inst->capabilities->cap[cap_id].hfi_id,
		HFI_HOST_FLAGS_NONE,
		msm_vidc_get_port_info(inst, cap_id),
		payload_type,
		hfi_val,
		sizeof(payload_size));
	if (rc)
		s_vpr_e(inst->sid,
			"%s: failed to set cap_id: %d to fw\n",
			__func__, cap_id);

	return rc;
}

static enum msm_vidc_inst_capability_type msm_vidc_get_cap_id(
	struct msm_vidc_inst *inst, u32 id)
{
	enum msm_vidc_inst_capability_type i = INST_CAP_NONE + 1;
	struct msm_vidc_inst_capability *capability;
	enum msm_vidc_inst_capability_type cap_id = INST_CAP_NONE;

	capability = inst->capabilities;
	do {
		if (capability->cap[i].v4l2_id == id) {
			cap_id = capability->cap[i].cap;
			break;
		}
		i++;
	} while (i < INST_CAP_MAX);

	return cap_id;
}

static int msm_vidc_add_capid_to_list(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id,
	enum msm_vidc_ctrl_list_type type)
{
	struct msm_vidc_inst_cap_entry *entry = NULL, *curr_node = NULL;

	/* skip adding if cap_id already present in list */
	if (type & FW_LIST) {
		list_for_each_entry(curr_node, &inst->firmware.list, list) {
			if (curr_node->cap_id == cap_id) {
				s_vpr_e(inst->sid,
					"%s: cap %d cannot be the child of two parents\n",
					__func__, cap_id);
				return 0;
			}
		}
	}

	entry = kzalloc(sizeof(*entry), GFP_ATOMIC);
	if (!entry) {
		s_vpr_e(inst->sid, "%s: alloc failed\n", __func__);
		return -ENOMEM;
	}
	entry->cap_id = cap_id;
	if (type & CHILD_LIST)
		list_add_tail(&entry->list, &inst->children.list);
	if (type & FW_LIST)
		list_add_tail(&entry->list, &inst->firmware.list);

	return 0;
}

static int msm_vidc_add_children(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	int i = 0;
	struct msm_vidc_inst_capability *capability = inst->capabilities;

	while (i < MAX_CAP_CHILDREN &&
		capability->cap[cap_id].children[i]) {
		rc = msm_vidc_add_capid_to_list(inst,
			capability->cap[cap_id].children[i],
			CHILD_LIST);
		if (rc)
			return rc;
		i++;
	}
	return rc;
}

static int msm_vidc_adjust_property(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability;

	capability = inst->capabilities;

	/*
	 * skip for uninitialized cap properties.
	 * Eg: Skip Tramform 8x8 cap that is uninitialized for HEVC codec
	 */
	if (!capability->cap[cap_id].cap)
		return 0;

	if (capability->cap[cap_id].adjust) {
		rc = capability->cap[cap_id].adjust(inst, NULL);
		if (rc)
			goto exit;
	}

	/* add children cap_id's to chidren list */
	rc = msm_vidc_add_children(inst, cap_id);
	if (rc)
		goto exit;

	/* add cap_id to firmware list  */
	rc = msm_vidc_add_capid_to_list(inst, cap_id, FW_LIST);
	if (rc)
		goto exit;

	return 0;

exit:
	return rc;
}

static int msm_vidc_adjust_dynamic_property(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id, struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability;
	s32 prev_value;

	capability = inst->capabilities;

	/*
	 * ctrl is NULL for children adjustment calls
	 * When a dynamic control having children is adjusted, check if dynamic
	 * adjustment is allowed for its children.
	 */
	if (!(capability->cap[cap_id].flags & CAP_FLAG_DYNAMIC_ALLOWED)) {
		s_vpr_e(inst->sid,
			"%s: dynamic setting of cap_id %d is not allowed\n",
			__func__, cap_id);
		msm_vidc_change_inst_state(inst, MSM_VIDC_ERROR, __func__);
		return -EINVAL;
	}

	/*
	 * if ctrl is NULL, it is children of some parent, and hence,
	 * must have an adjust function defined
	 */
	if (!ctrl && !capability->cap[cap_id].adjust) {
		s_vpr_e(inst->sid,
			"%s: child cap %d must have ajdust function\n",
			__func__, capability->cap[cap_id].cap);
		return -EINVAL;
	}

	prev_value = capability->cap[cap_id].value;

	if (capability->cap[cap_id].adjust) {
		rc = capability->cap[cap_id].adjust(inst, ctrl);
		if (rc)
			goto exit;
	} else if (ctrl) {
		capability->cap[cap_id].value = ctrl->val;
	}

	/* add children if cap value modified */
	if (capability->cap[cap_id].value != prev_value) {
		rc = msm_vidc_add_children(inst, cap_id);
		if (rc)
			goto exit;
	}

	/* add cap_id to firmware list always */
	rc = msm_vidc_add_capid_to_list(inst, cap_id, FW_LIST);
	if (rc)
		goto exit;

	return 0;

exit:
	return rc;
}

int msm_vidc_ctrl_deinit(struct msm_vidc_inst *inst)
{
	if (!inst) {
		d_vpr_e("%s: invalid parameters\n", __func__);
		return -EINVAL;
	}
	s_vpr_h(inst->sid, "%s(): num ctrls %d\n", __func__, inst->num_ctrls);
	v4l2_ctrl_handler_free(&inst->ctrl_handler);
	kfree(inst->ctrls);

	return 0;
}

int msm_vidc_ctrl_init(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability;
	struct msm_vidc_core *core;
	int idx = 0;
	struct v4l2_ctrl_config ctrl_cfg = {0};
	int num_ctrls = 0, ctrl_idx = 0;

	if (!inst || !inst->core || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;
	capability = inst->capabilities;

	if (!core->v4l2_ctrl_ops) {
		s_vpr_e(inst->sid, "%s: no control ops\n", __func__);
		return -EINVAL;
	}

	for (idx = 0; idx < INST_CAP_MAX; idx++) {
		if (capability->cap[idx].v4l2_id)
			num_ctrls++;
	}
	if (!num_ctrls) {
		s_vpr_e(inst->sid, "%s: no ctrls available in cap database\n",
			__func__);
		return -EINVAL;
	}
	inst->ctrls = kcalloc(num_ctrls,
		sizeof(struct v4l2_ctrl *), GFP_KERNEL);
	if (!inst->ctrls) {
		s_vpr_e(inst->sid, "%s: failed to allocate ctrl\n", __func__);
		return -ENOMEM;
	}

	rc = v4l2_ctrl_handler_init(&inst->ctrl_handler, num_ctrls);
	if (rc) {
		s_vpr_e(inst->sid, "control handler init failed, %d\n",
				inst->ctrl_handler.error);
		return rc;
	}

	for (idx = 0; idx < INST_CAP_MAX; idx++) {
		struct v4l2_ctrl *ctrl;

		if (!capability->cap[idx].v4l2_id)
			continue;

		if (ctrl_idx >= num_ctrls) {
			s_vpr_e(inst->sid,
				"%s: invalid ctrl %#x, max allowed %d\n",
				__func__, capability->cap[idx].v4l2_id,
				num_ctrls);
			return -EINVAL;
		}
		s_vpr_h(inst->sid,
			"%s: cap idx %d, value %d min %d max %d step_or_mask %#x flags %#x v4l2_id %#x hfi_id %#x\n",
			__func__, idx,
			capability->cap[idx].value,
			capability->cap[idx].min,
			capability->cap[idx].max,
			capability->cap[idx].step_or_mask,
			capability->cap[idx].flags,
			capability->cap[idx].v4l2_id,
			capability->cap[idx].hfi_id);

		memset(&ctrl_cfg, 0, sizeof(struct v4l2_ctrl_config));

		if (is_priv_ctrl(capability->cap[idx].v4l2_id)) {
			/* add private control */
			ctrl_cfg.def = capability->cap[idx].value;
			ctrl_cfg.flags = 0;
			ctrl_cfg.id = capability->cap[idx].v4l2_id;
			ctrl_cfg.max = capability->cap[idx].max;
			ctrl_cfg.min = capability->cap[idx].min;
			ctrl_cfg.ops = core->v4l2_ctrl_ops;
			ctrl_cfg.type = (capability->cap[idx].flags &
					CAP_FLAG_MENU) ?
					V4L2_CTRL_TYPE_MENU :
					V4L2_CTRL_TYPE_INTEGER;
			if (ctrl_cfg.type == V4L2_CTRL_TYPE_MENU) {
				ctrl_cfg.menu_skip_mask =
					~(capability->cap[idx].step_or_mask);
				ctrl_cfg.qmenu = msm_vidc_get_qmenu_type(inst,
					capability->cap[idx].v4l2_id);
			} else {
				ctrl_cfg.step =
					capability->cap[idx].step_or_mask;
			}
			ctrl_cfg.name = msm_vidc_get_priv_ctrl_name(inst->sid,
					capability->cap[idx].v4l2_id);
			if (!ctrl_cfg.name) {
				s_vpr_e(inst->sid, "%s: %#x ctrl name is null\n",
					__func__, ctrl_cfg.id);
				return -EINVAL;
			}
			ctrl = v4l2_ctrl_new_custom(&inst->ctrl_handler,
					&ctrl_cfg, NULL);
		} else {
			if (capability->cap[idx].flags & CAP_FLAG_MENU) {
				ctrl = v4l2_ctrl_new_std_menu(
					&inst->ctrl_handler,
					core->v4l2_ctrl_ops,
					capability->cap[idx].v4l2_id,
					capability->cap[idx].max,
					~(capability->cap[idx].step_or_mask),
					capability->cap[idx].value);
			} else {
				ctrl = v4l2_ctrl_new_std(&inst->ctrl_handler,
					core->v4l2_ctrl_ops,
					capability->cap[idx].v4l2_id,
					capability->cap[idx].min,
					capability->cap[idx].max,
					capability->cap[idx].step_or_mask,
					capability->cap[idx].value);
			}
		}
		if (!ctrl) {
			s_vpr_e(inst->sid, "%s: invalid ctrl %#x\n", __func__,
				capability->cap[idx].v4l2_id);
			return -EINVAL;
		}

		rc = inst->ctrl_handler.error;
		if (rc) {
			s_vpr_e(inst->sid,
				"error adding ctrl (%#x) to ctrl handle, %d\n",
				capability->cap[idx].v4l2_id,
				inst->ctrl_handler.error);
			return rc;
		}

		/*
		 * TODO(AS)
		 * ctrl->flags |= capability->cap[idx].flags;
		 */
		ctrl->flags |= V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
		inst->ctrls[ctrl_idx] = ctrl;
		ctrl_idx++;
	}
	inst->num_ctrls = num_ctrls;
	s_vpr_h(inst->sid, "%s(): num ctrls %d\n", __func__, inst->num_ctrls);

	return rc;
}

int msm_v4l2_op_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	struct msm_vidc_inst *inst;
	enum msm_vidc_inst_capability_type cap_id;
	struct msm_vidc_inst_cap_entry *curr_node = NULL, *tmp_node = NULL;
	struct msm_vidc_inst_capability *capability;

	if (!ctrl) {
		d_vpr_e("%s: invalid ctrl parameter\n", __func__);
		return -EINVAL;
	}

	inst = container_of(ctrl->handler,
		struct msm_vidc_inst, ctrl_handler);

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid parameters for inst\n", __func__);
		return -EINVAL;
	}

	if (inst->state == MSM_VIDC_ERROR) {
		s_vpr_e(inst->sid, "%s: set ctrl not allowed in error state\n");
		/* (error name TBD); */
		return -EINVAL;
	}

	capability = inst->capabilities;

	s_vpr_h(inst->sid, "%s: state %d, name %s, id 0x%x value %d\n",
		__func__, inst->state, ctrl->name, ctrl->id, ctrl->val);

	cap_id = msm_vidc_get_cap_id(inst, ctrl->id);
	if (cap_id == INST_CAP_NONE) {
		s_vpr_e(inst->sid, "%s: could not find cap_id for ctrl %s\n",
			__func__, ctrl->name);
		return -EINVAL;
	}

	/* Static setting */
	if (!inst->vb2q[OUTPUT_PORT].streaming) {
		capability->cap[cap_id].value = ctrl->val;
		return 0;
	}

	/* check if dynamic adjustment is allowed */
	if (inst->vb2q[OUTPUT_PORT].streaming &&
		!(capability->cap[cap_id].flags & CAP_FLAG_DYNAMIC_ALLOWED)) {
		s_vpr_e(inst->sid,
			"%s: dynamic setting of cap_id %d is not allowed\n",
			__func__, cap_id);
		return -EBUSY;
	}

	rc = msm_vidc_adjust_dynamic_property(inst, cap_id, ctrl);
	if (rc)
		goto exit;

	/* adjust all children if any */
	list_for_each_entry_safe(curr_node, tmp_node,
			&inst->children.list, list) {
		rc = msm_vidc_adjust_dynamic_property(
				inst, curr_node->cap_id, NULL);
		if (rc)
			goto exit;
		list_del(&curr_node->list);
		kfree(curr_node);
	}

	/* dynamic controls with request will be set along with qbuf */
	if (inst->request)
		return 0;

	/* Dynamic set control ASAP */
	rc = msm_vidc_set_v4l2_properties(inst);
	if (rc) {
		s_vpr_e(inst->sid, "%s: setting %s failed\n",
			__func__, ctrl->name);
		goto exit;
	}

exit:
	return rc;
}

int msm_vidc_adjust_entropy_mode(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability;
	s32 adjusted_value;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *) instance;
	s32 profile = -1;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	capability = inst->capabilities;

	/* ctrl is always NULL in streamon case */
	if (ctrl)
		adjusted_value = ctrl->val;
	else
		adjusted_value = capability->cap[ENTROPY_MODE].value;

	if (inst->codec != MSM_VIDC_H264) {
		s_vpr_e(inst->sid,
			"%s: incorrect entry in database. fix the database\n",
			__func__);
		return 0;
	}

	profile = capability->cap[PROFILE].value;

	if (profile == V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE ||
		profile == V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)
		adjusted_value = V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC;

	if (capability->cap[ENTROPY_MODE].value != adjusted_value) {
		s_vpr_h(inst->sid, "%s: updated from %#x to adjusted %#x\n", __func__,
			capability->cap[ENTROPY_MODE].value, adjusted_value);
		capability->cap[ENTROPY_MODE].value = adjusted_value;
	}

	return rc;
}

int msm_vidc_adjust_profile(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	int i = 0;
	struct msm_vidc_inst_capability *capability;
	s32 adjusted_value;
	enum msm_vidc_inst_capability_type parent_id;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *) instance;
	s32 pix_fmt = -1;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	capability = inst->capabilities;

	/* ctrl is always NULL in streamon case */
	if (ctrl)
		adjusted_value = ctrl->val;
	else
		adjusted_value = capability->cap[PROFILE].value;

	/* TODO(AS): Create a utility for this while loop and add
	 * detailed comments within for utility functionality
	 */
	while (i < MAX_CAP_PARENTS &&
		capability->cap[PROFILE].parents[i]) {
		parent_id = capability->cap[PROFILE].parents[i];
		if (parent_id == PIX_FMTS) {
			pix_fmt = capability->cap[PIX_FMTS].value;
		}
		else
			s_vpr_e(inst->sid,
				"%s: invalid parent %d\n",
				__func__, parent_id);
		i++;
	}

	/* PIX_FMTS dependency is common across all chipsets.
	 * Hence, PIX_FMTS must be specified as Parent for HEVC profile.
	 * Otherwise it would be a database error that should be fixed.
	 */
	if (pix_fmt == -1) {
		s_vpr_e(inst->sid,
			"%s: missing parent: %d, please correct database\n",
			__func__, PIX_FMTS);
		return -EINVAL;
	}

	/* 10 bit profile for 10 bit color format */
	if (pix_fmt == MSM_VIDC_FMT_TP10C ||
		pix_fmt == MSM_VIDC_FMT_P010) {
		adjusted_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10;
	} else {
		/* 8 bit profile for 8 bit color format */
		if (adjusted_value == V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10)
			adjusted_value = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN;
	}

	if (capability->cap[PROFILE].value != adjusted_value) {
		s_vpr_h(inst->sid,
			"%s: updated from %#x to adjusted %#x\n", __func__,
			capability->cap[PROFILE].value, adjusted_value);
		capability->cap[PROFILE].value = adjusted_value;
	}

	return rc;
}

int msm_vidc_adjust_ltr_count(void *instance, struct v4l2_ctrl *ctrl)
{
	int rc = 0;
	int i = 0;
	struct msm_vidc_inst_capability *capability;
	s32 adjusted_value;
	enum msm_vidc_inst_capability_type parent_id;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *) instance;
	s32 rc_type = -1;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	if (ctrl)
		adjusted_value = ctrl->val;
	else
		adjusted_value = capability->cap[LTR_COUNT].value;

	/* check parents and adjust cabac session value */
	while (i < MAX_CAP_PARENTS &&
		capability->cap[LTR_COUNT].parents[i]) {
		parent_id = capability->cap[LTR_COUNT].parents[i];
		i++;
	}

	if (!(rc_type == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR
		/* TODO(AS): remove comment after below rc modes are upstreamed
		|| rc_type == RATE_CONTROL_OFF ||
		|| rc_tpe == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR_VFR
		*/))
		adjusted_value = 0;

	if (capability->cap[LTR_COUNT].value != adjusted_value) {
		s_vpr_h(inst->sid, "%s: adjusted from %#x to %#x\n", __func__,
			capability->cap[LTR_COUNT].value, adjusted_value);
		capability->cap[LTR_COUNT].value = adjusted_value;
	}

	return rc;
}

/*
 * Loop over instance capabilities with CAP_FLAG_ROOT
 * and call adjust function, where
 * - adjust current capability value
 * - update tail of instance children list with capability children
 * - update instance firmware list with current capability id
 * Now, loop over child list and call its adjust function
 */
int msm_vidc_adjust_v4l2_properties(struct msm_vidc_inst *inst)
{
	int rc = 0;
	int i;
	struct msm_vidc_inst_cap_entry *curr_node = NULL, *tmp_node = NULL;
	struct msm_vidc_inst_capability *capability;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	for (i = 0; i < INST_CAP_MAX; i++) {
		if (capability->cap[i].flags & CAP_FLAG_ROOT) {
			rc = msm_vidc_adjust_property(inst,
					capability->cap[i].cap);
			if (rc)
				goto exit;
		}
	}

	/*
	 * children of all root controls are already
	 * added to inst->children list at this point
	 */
	list_for_each_entry_safe(curr_node, tmp_node,
			&inst->children.list, list) {
		/*
		 * call adjust for each child. Each child adjust
		 * will also update child list at the tail with
		 * its own children list.
		 * Also, if current control id value is updated,
		 * its entry should be added to fw list.
		 */
		rc = msm_vidc_adjust_property(inst, curr_node->cap_id);
		if (rc)
			goto exit;
		list_del(&curr_node->list);
		kfree(curr_node);
	}

exit:
	return rc;
}

int msm_vidc_set_bitrate_mode(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *) instance;
	int lossless, frame_rc, bitrate_mode, frame_skip;
	u32 hfi_value;
	struct msm_vidc_inst_capability *capability;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	bitrate_mode = capability->cap[cap_id].value;
	lossless = capability->cap[LOSSLESS].value;
	frame_rc = capability->cap[FRAME_RC_ENABLE].value;
	frame_skip = capability->cap[FRAME_SKIP_MODE].value;

	if (lossless) {
		hfi_value = HFI_RC_LOSSLESS;
		return rc;
	}

	if (!frame_rc) {
		hfi_value = HFI_RC_OFF;
		return rc;
	}

	if (bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR) {
		hfi_value = HFI_RC_VBR_CFR;
	} else if (bitrate_mode == V4L2_MPEG_VIDEO_BITRATE_MODE_CBR) {
		if (frame_skip)
			hfi_value = HFI_RC_CBR_VFR;
		else
			hfi_value = HFI_RC_CBR_CFR;
	}/* TODO: CQ mode
	else if (bitrate_mode == CQ) {
		hfi_value = HFI_RC_CQ;
	}
	*/

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
		&hfi_value, sizeof(u32), __func__);

	return rc;
}

int msm_vidc_set_header_mode(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *) instance;
	int header_mode, prepend_sps_pps, hdr_metadata;
	u32 hfi_value = 0;
	struct msm_vidc_inst_capability *capability;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	header_mode = capability->cap[cap_id].value;
	prepend_sps_pps = capability->cap[PREPEND_SPSPPS_TO_IDR].value;
	hdr_metadata = capability->cap[META_SEQ_HDR_NAL].value;

	if (header_mode == V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE)
		hfi_value |= HFI_SEQ_HEADER_SEPERATE_FRAME;
	else if (header_mode == V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME)
		hfi_value |= HFI_SEQ_HEADER_JOINED_WITH_1ST_FRAME;

	if (prepend_sps_pps) {
		hfi_value |= HFI_SEQ_HEADER_PREFIX_WITH_SYNC_FRAME;
	}

	if (hdr_metadata) {
		hfi_value |= HFI_SEQ_HEADER_METADATA;
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
		&hfi_value, sizeof(u32), __func__);

	return rc;
}

int msm_vidc_set_q16(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value = 0;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	hfi_value = inst->capabilities->cap[cap_id].value;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_Q16,
		&hfi_value, sizeof(u32), __func__);

	return rc;
}

int msm_vidc_set_u32(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	if (inst->capabilities->cap[cap_id].flags & CAP_FLAG_MENU) {
		rc = msm_vidc_v4l2_menu_to_hfi(inst, cap_id, &hfi_value);
		if (rc)
			return -EINVAL;
	} else {
		hfi_value = inst->capabilities->cap[cap_id].value;
	}

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32,
		&hfi_value, sizeof(u32), __func__);

	return rc;
}

int msm_vidc_set_u32_enum(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	u32 hfi_value;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = msm_vidc_v4l2_to_hfi_enum(inst, cap_id, &hfi_value);
	if (rc)
		return -EINVAL;

	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_U32_ENUM,
		&hfi_value, sizeof(u32), __func__);

	return rc;
}

int msm_vidc_set_s32(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;
	s32 hfi_value = 0;

	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	hfi_value = inst->capabilities->cap[cap_id].value;
	rc = msm_vidc_packetize_control(inst, cap_id, HFI_PAYLOAD_S32,
		&hfi_value, sizeof(u32), __func__);

	return rc;
}

/* Please ignore this function for now. TO DO*/
int msm_vidc_set_array(void *instance,
	enum msm_vidc_inst_capability_type cap_id)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability;
	struct msm_vidc_core *core;

	struct msm_vidc_inst *inst = (struct msm_vidc_inst *)instance;

	if (!inst || !inst->core || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	capability = inst->capabilities;
	core = (struct msm_vidc_core *)inst->core;

	switch (cap_id) {
	/*
	 * Needed if any control needs to be packed into a structure
	 * and sent for packetization.
	 * payload types may be:
	 * STRUCTURE, BLOB, STRING, PACKED, ARRAY,
	 *
	case BITRATE_MODE:
		s_vpr_h(inst->sid, "%s: %d\n", __func__, hfi_value);
		hfi_create_packet(inst->packet, inst->packet_size,
			offset,
			capability->cap[cap_id].hfi_id,
			HFI_HOST_FLAGS_NONE, HFI_PAYLOAD_ENUM,
			HFI_PORT_NONE, core->packet_id++,
			&capability->cap[PROFILE].value, sizeof(u32));
		break;
	}
	*/
	default:
		s_vpr_e(inst->sid,
			"%s: Unknown cap id %d, cannot set to fw\n",
			__func__, cap_id);
		rc = -EINVAL;
		break;
	}

	return rc;
}

int msm_vidc_set_v4l2_properties(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_inst_capability *capability;
	struct msm_vidc_inst_cap_entry *curr_node = NULL, *tmp_node = NULL;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	capability = inst->capabilities;

	list_for_each_entry_safe(curr_node, tmp_node,
			&inst->firmware.list, list) {

		/*  cap_id's like PIX_FMT etc may not have set functions */
		if (!capability->cap[curr_node->cap_id].set)
			continue;

		rc = capability->cap[curr_node->cap_id].set(inst,
			curr_node->cap_id);
		if (rc)
			goto exit;

		list_del(&curr_node->list);
		kfree(curr_node);
	}

exit:
	return rc;
}

int msm_vidc_v4l2_menu_to_hfi(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id, u32 *value)
{
	struct msm_vidc_inst_capability *capability = inst->capabilities;

	switch (capability->cap[cap_id].v4l2_id) {
	case V4L2_CID_MPEG_VIDEO_H264_ENTROPY_MODE:
		switch (capability->cap[cap_id].value) {
		case V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CABAC:
			*value = 1;
			break;
		case V4L2_MPEG_VIDEO_H264_ENTROPY_MODE_CAVLC:
			*value = 0;
			break;
		default:
			*value = 1;
			goto set_default;
		}
		return 0;
	default:
		s_vpr_e(inst->sid,
			"%s: mapping not specified for ctrl_id: %#x\n",
			__func__, capability->cap[cap_id].v4l2_id);
		return -EINVAL;
	}

set_default:
	s_vpr_e(inst->sid,
		"%s: invalid value %d for ctrl id: %#x. Set default: %u\n",
		__func__, capability->cap[cap_id].value,
		capability->cap[cap_id].v4l2_id, *value);
	return 0;
}

int msm_vidc_v4l2_to_hfi_enum(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_capability_type cap_id, u32 *value)
{
	struct msm_vidc_inst_capability *capability = inst->capabilities;

	switch (capability->cap[cap_id].v4l2_id) {
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
	case V4L2_CID_MPEG_VIDEO_HEVC_TIER:
	case V4L2_CID_MPEG_VIDC_VIDEO_BLUR_TYPES:
		*value = capability->cap[cap_id].value;
		return 0;
	case V4L2_CID_MPEG_VIDEO_HEVC_HIER_CODING_TYPE:
		switch (capability->cap[cap_id].value) {
		case V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_B:
			*value = HFI_HIER_B;
			break;
		case V4L2_MPEG_VIDEO_HEVC_HIERARCHICAL_CODING_P:
			//TODO (AS): check if this is right mapping
			*value = HFI_HIER_P_SLIDING_WINDOW;
			break;
		default:
			*value = HFI_HIER_P_SLIDING_WINDOW;
			goto set_default;
		}
		return 0;
	default:
		s_vpr_e(inst->sid,
			"%s: mapping not specified for ctrl_id: %#x\n",
			__func__, capability->cap[cap_id].v4l2_id);
		return -EINVAL;
	}

set_default:
	s_vpr_e(inst->sid,
		"%s: invalid value %d for ctrl id: %#x. Set default: %u\n",
		__func__, capability->cap[cap_id].value,
		capability->cap[cap_id].v4l2_id, *value);
	return 0;
}
