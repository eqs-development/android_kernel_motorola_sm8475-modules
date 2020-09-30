// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/iommu.h>
#include <linux/workqueue.h>
#include <media/msm_vidc_utils.h>
#include <media/msm_media_info.h>

#include "msm_vidc_driver.h"
#include "msm_vidc_platform.h"
#include "msm_vidc_internal.h"
#include "msm_vidc_memory.h"
#include "msm_vidc_debug.h"
#include "venus_hfi.h"
#include "msm_vidc.h"

#define COUNT_BITS(a, out) ({       \
	while ((a) >= 1) {          \
		(out) += (a) & (1); \
		(a) >>= (1);        \
	}                           \
})

void print_vidc_buffer(u32 tag, const char *str, struct msm_vidc_inst *inst,
		struct msm_vidc_buffer *vbuf)
{
	struct msm_vidc_buffer *mbuf;

	if (!(tag & msm_vidc_debug) || !inst || !vbuf)
		return;

	mbuf = get_meta_buffer(inst, vbuf);
	if (!mbuf)
		dprintk(tag, inst->sid,
			"%s: %s: idx %2d fd %3d off %d daddr %#x size %d filled %d flags %#x ts %lld attr %#x\n",
			str, vbuf->type == MSM_VIDC_BUF_INPUT ? "INPUT" : "OUTPUT",
			vbuf->index, vbuf->fd, vbuf->data_offset,
			vbuf->device_addr, vbuf->buffer_size, vbuf->data_size,
			vbuf->flags, vbuf->timestamp, vbuf->attr);
	else
		dprintk(tag, inst->sid,
			"%s: %s: idx %2d fd %3d off %d daddr %#x size %d filled %d flags %#x ts %lld attr %#x meta: fd %3d daddr %#x size %d\n",
			str, vbuf->type == MSM_VIDC_BUF_INPUT ? "INPUT" : "OUTPUT",
			vbuf->index, vbuf->fd, vbuf->data_offset,
			vbuf->device_addr, vbuf->buffer_size, vbuf->data_size,
			vbuf->flags, vbuf->timestamp, vbuf->attr,
			mbuf->fd, mbuf->device_addr, mbuf->buffer_size);
}

void print_vb2_buffer(const char *str, struct msm_vidc_inst *inst,
		struct vb2_buffer *vb2)
{
	if (!inst || !vb2)
		return;

	s_vpr_e(inst->sid,
		"%s: %s: idx %2d fd %d off %d size %d filled %d\n",
		str, vb2->type == INPUT_PLANE ? "INPUT" : "OUTPUT",
		vb2->index, vb2->planes[0].m.fd,
		vb2->planes[0].data_offset, vb2->planes[0].length,
		vb2->planes[0].bytesused);
}

enum msm_vidc_buffer_type v4l2_type_to_driver(u32 type)
{
	enum msm_vidc_buffer_type buffer_type = 0;

	switch (type) {
	case INPUT_PLANE:
		buffer_type = MSM_VIDC_BUF_INPUT;
		break;
	case OUTPUT_PLANE:
		buffer_type = MSM_VIDC_BUF_OUTPUT;
		break;
	case INPUT_META_PLANE:
		buffer_type = MSM_VIDC_BUF_INPUT_META;
		break;
	case OUTPUT_META_PLANE:
		buffer_type = MSM_VIDC_BUF_OUTPUT_META;
		break;
	default:
		d_vpr_e("%s: vidc buffer type not found for %#x\n",
			__func__, type);
		break;
	}
	return buffer_type;
}

u32 v4l2_type_from_driver(enum msm_vidc_buffer_type buffer_type)
{
	u32 type = 0;

	switch (buffer_type) {
	case MSM_VIDC_BUF_INPUT:
		type = INPUT_PLANE;
		break;
	case MSM_VIDC_BUF_OUTPUT:
		type = OUTPUT_PLANE;
		break;
	case MSM_VIDC_BUF_INPUT_META:
		type = INPUT_META_PLANE;
		break;
	case MSM_VIDC_BUF_OUTPUT_META:
		type = OUTPUT_META_PLANE;
		break;
	default:
		d_vpr_e("%s: v4l2 type not found for %#x\n",
			__func__, buffer_type);
		break;
	}
	return buffer_type;
}

enum msm_vidc_codec_type v4l2_codec_to_driver(u32 v4l2_codec)
{
	enum msm_vidc_codec_type codec = 0;

	switch (v4l2_codec) {
	case V4L2_PIX_FMT_H264:
		codec = MSM_VIDC_H264;
		break;
	case V4L2_PIX_FMT_HEVC:
		codec = MSM_VIDC_HEVC;
		break;
	case V4L2_PIX_FMT_VP9:
		codec = MSM_VIDC_VP9;
		break;
	case V4L2_PIX_FMT_MPEG2:
		codec = MSM_VIDC_MPEG2;
		break;
	default:
		d_vpr_e("%s: vidc codec not found for %#x\n", __func__, v4l2_codec);
		break;
	}
	return codec;
}

u32 v4l2_codec_from_driver(enum msm_vidc_codec_type codec)
{
	u32 v4l2_codec = 0;

	switch (codec) {
	case MSM_VIDC_H264:
		v4l2_codec = V4L2_PIX_FMT_H264;
		break;
	case MSM_VIDC_HEVC:
		v4l2_codec = V4L2_PIX_FMT_HEVC;
		break;
	case MSM_VIDC_VP9:
		v4l2_codec = V4L2_PIX_FMT_VP9;
		break;
	case MSM_VIDC_MPEG2:
		v4l2_codec = V4L2_PIX_FMT_MPEG2;
		break;
	default:
		d_vpr_e("%s: v4l2 codec not found for %#x\n", __func__, codec);
		break;
	}
	return v4l2_codec;
}

enum msm_vidc_colorformat_type v4l2_colorformat_to_driver(u32 v4l2_colorformat)
{
	enum msm_vidc_colorformat_type colorformat = 0;

	switch (v4l2_colorformat) {
	case V4L2_PIX_FMT_NV12:
		colorformat = MSM_VIDC_FMT_NV12;
		break;
	case V4L2_PIX_FMT_NV21:
		colorformat = MSM_VIDC_FMT_NV21;
		break;
	case V4L2_PIX_FMT_NV12_UBWC:
		colorformat = MSM_VIDC_FMT_NV12_UBWC;
		break;
	case V4L2_PIX_FMT_NV12_TP10_UBWC:
		colorformat = MSM_VIDC_FMT_NV12_TP10_UBWC;
		break;
	case V4L2_PIX_FMT_RGBA8888_UBWC:
		colorformat = MSM_VIDC_FMT_RGBA8888_UBWC;
		break;
	case V4L2_PIX_FMT_NV12_P010_UBWC:
		colorformat = MSM_VIDC_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
		break;
	case V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS:
		colorformat = MSM_VIDC_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
		break;
	default:
		d_vpr_e("%s: vidc format not found for %#x\n", __func__, v4l2_colorformat);
		break;
	}
	return colorformat;
}

u32 v4l2_colorformat_from_driver(enum msm_vidc_colorformat_type colorformat)
{
	u32 v4l2_colorformat = 0;

	switch (colorformat) {
	case MSM_VIDC_FMT_NV12:
		v4l2_colorformat = V4L2_PIX_FMT_NV12;
		break;
	case MSM_VIDC_FMT_NV21:
		v4l2_colorformat = V4L2_PIX_FMT_NV21;
		break;
	case MSM_VIDC_FMT_NV12_UBWC:
		v4l2_colorformat = V4L2_PIX_FMT_NV12_UBWC;
		break;
	case MSM_VIDC_FMT_NV12_TP10_UBWC:
		v4l2_colorformat = V4L2_PIX_FMT_NV12_TP10_UBWC;
		break;
	case MSM_VIDC_FMT_RGBA8888_UBWC:
		v4l2_colorformat = V4L2_PIX_FMT_RGBA8888_UBWC;
		break;
	case MSM_VIDC_FMT_NV12_P010_UBWC:
		v4l2_colorformat = V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
		break;
	case MSM_VIDC_FMT_SDE_Y_CBCR_H2V2_P010_VENUS:
		v4l2_colorformat = V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS;
		break;
	default:
		d_vpr_e("%s: v4l2 format not found for %#x\n", __func__, colorformat);
		break;
	}
	return v4l2_colorformat;
}

u32 v4l2_colorformat_to_media(u32 v4l2_fmt)
{
	switch (v4l2_fmt) {
	case V4L2_PIX_FMT_NV12:
		return COLOR_FMT_NV12;
	case V4L2_PIX_FMT_NV21:
		return COLOR_FMT_NV21;
	case V4L2_PIX_FMT_NV12_512:
		return COLOR_FMT_NV12_512;
	case V4L2_PIX_FMT_SDE_Y_CBCR_H2V2_P010_VENUS:
		return COLOR_FMT_P010;
	case V4L2_PIX_FMT_NV12_UBWC:
		return COLOR_FMT_NV12_UBWC;
	case V4L2_PIX_FMT_NV12_TP10_UBWC:
		return COLOR_FMT_NV12_BPP10_UBWC;
	case V4L2_PIX_FMT_RGBA8888_UBWC:
		return COLOR_FMT_RGBA8888_UBWC;
	default:
		d_vpr_e(
			"Invalid v4l2 color fmt FMT : %x, Set default(NV12)",
			v4l2_fmt);
		return COLOR_FMT_NV12;
	}
}

int v4l2_type_to_driver_port(struct msm_vidc_inst *inst, u32 type,
	const char *func)
{
	int port;

	if (type == INPUT_PLANE) {
		port = INPUT_PORT;
	} else if (type == INPUT_META_PLANE) {
		port = INPUT_META_PORT;
	} else if (type == OUTPUT_PLANE) {
		port = OUTPUT_PORT;
	} else if (type == OUTPUT_META_PLANE) {
		port = OUTPUT_META_PORT;
	} else {
		s_vpr_e(inst->sid, "%s: invalid type %d\n", func, type);
		port = -EINVAL;
	}

	return port;
}

u32 msm_vidc_get_buffer_region(struct msm_vidc_inst *inst,
	enum msm_vidc_buffer_type buffer_type, const char *func)
{
	u32 region = MSM_VIDC_NON_SECURE;

	if (!is_secure_session(inst))
		return region;

	switch (buffer_type) {
	case MSM_VIDC_BUF_INPUT:
		if (is_encode_session(inst))
			region = MSM_VIDC_SECURE_PIXEL;
		else
			region = MSM_VIDC_SECURE_BITSTREAM;
		break;
	case MSM_VIDC_BUF_OUTPUT:
		if (is_encode_session(inst))
			region = MSM_VIDC_SECURE_BITSTREAM;
		else
			region = MSM_VIDC_SECURE_PIXEL;
		break;
	case MSM_VIDC_BUF_INPUT_META:
	case MSM_VIDC_BUF_OUTPUT_META:
		region = MSM_VIDC_NON_SECURE;
		break;
	case MSM_VIDC_BUF_SCRATCH:
		region = MSM_VIDC_SECURE_BITSTREAM;
		break;
	case MSM_VIDC_BUF_SCRATCH_1:
		region = MSM_VIDC_SECURE_NONPIXEL;
		break;
	case MSM_VIDC_BUF_SCRATCH_2:
		region = MSM_VIDC_SECURE_PIXEL;
		break;
	case MSM_VIDC_BUF_PERSIST:
		if (is_encode_session(inst))
			region = MSM_VIDC_SECURE_NONPIXEL;
		else
			region = MSM_VIDC_SECURE_BITSTREAM;
		break;
	case MSM_VIDC_BUF_PERSIST_1:
		region = MSM_VIDC_SECURE_NONPIXEL;
		break;
	default:
		s_vpr_e(inst->sid, "%s: invalid buffer type %d\n",
			func, buffer_type);
	}
	return region;
}

struct msm_vidc_buffers *msm_vidc_get_buffers(
	struct msm_vidc_inst *inst, enum msm_vidc_buffer_type buffer_type,
	const char *func)
{
	switch (buffer_type) {
	case MSM_VIDC_BUF_INPUT:
		return &inst->buffers.input;
	case MSM_VIDC_BUF_INPUT_META:
		return &inst->buffers.input_meta;
	case MSM_VIDC_BUF_OUTPUT:
		return &inst->buffers.output;
	case MSM_VIDC_BUF_OUTPUT_META:
		return &inst->buffers.output_meta;
	case MSM_VIDC_BUF_SCRATCH:
		return &inst->buffers.scratch;
	case MSM_VIDC_BUF_SCRATCH_1:
		return &inst->buffers.scratch_1;
	case MSM_VIDC_BUF_SCRATCH_2:
		return &inst->buffers.scratch_2;
	case MSM_VIDC_BUF_PERSIST:
		return &inst->buffers.persist;
	case MSM_VIDC_BUF_PERSIST_1:
		return &inst->buffers.persist_1;
	default:
		s_vpr_e(inst->sid, "%s: invalid buffer type %d\n",
			func, buffer_type);
		return NULL;
	}
}

struct msm_vidc_mappings *msm_vidc_get_mappings(
	struct msm_vidc_inst *inst, enum msm_vidc_buffer_type buffer_type,
	const char *func)
{
	switch (buffer_type) {
	case MSM_VIDC_BUF_INPUT:
		return &inst->mappings.input;
	case MSM_VIDC_BUF_INPUT_META:
		return &inst->mappings.input_meta;
	case MSM_VIDC_BUF_OUTPUT:
		return &inst->mappings.output;
	case MSM_VIDC_BUF_OUTPUT_META:
		return &inst->mappings.output_meta;
	case MSM_VIDC_BUF_SCRATCH:
		return &inst->mappings.scratch;
	case MSM_VIDC_BUF_SCRATCH_1:
		return &inst->mappings.scratch_1;
	case MSM_VIDC_BUF_SCRATCH_2:
		return &inst->mappings.scratch_2;
	case MSM_VIDC_BUF_PERSIST:
		return &inst->mappings.persist;
	case MSM_VIDC_BUF_PERSIST_1:
		return &inst->mappings.persist_1;
	default:
		s_vpr_e(inst->sid, "%s: invalid buffer type %d\n",
			func, buffer_type);
		return NULL;
	}
}

struct msm_vidc_allocations *msm_vidc_get_allocations(
	struct msm_vidc_inst *inst, enum msm_vidc_buffer_type buffer_type,
	const char *func)
{
	switch (buffer_type) {
	case MSM_VIDC_BUF_SCRATCH:
		return &inst->allocations.scratch;
	case MSM_VIDC_BUF_SCRATCH_1:
		return &inst->allocations.scratch_1;
	case MSM_VIDC_BUF_SCRATCH_2:
		return &inst->allocations.scratch_2;
	case MSM_VIDC_BUF_PERSIST:
		return &inst->allocations.persist;
	case MSM_VIDC_BUF_PERSIST_1:
		return &inst->allocations.persist_1;
	default:
		s_vpr_e(inst->sid, "%s: invalid buffer type %d\n",
			func, buffer_type);
		return NULL;
	}
}

int msm_vidc_change_inst_state(struct msm_vidc_inst *inst,
	enum msm_vidc_inst_state request_state, const char *func)
{
	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	if (!request_state) {
		s_vpr_e(inst->sid, "%s: invalid request state\n", func);
		return -EINVAL;
	}

	if (inst->state == MSM_VIDC_ERROR) {
		s_vpr_h(inst->sid,
			"%s: inst is in bad state, can not change state to %d\n",
			func, request_state);
		return 0;
	}

	s_vpr_h(inst->sid, "%s: state changed from %d to %d\n",
		   func, inst->state, request_state);
	inst->state = request_state;
	return 0;
}

int msm_vidc_get_control(struct msm_vidc_inst *inst, struct v4l2_ctrl *ctrl)
{
	int rc = 0;

	if (!inst || !ctrl) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->val = inst->buffers.output.min_count +
			inst->buffers.output.extra_count;
		s_vpr_h(inst->sid, "g_min: output buffers %d\n", ctrl->val);
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_OUTPUT:
		ctrl->val = inst->buffers.input.min_count +
			inst->buffers.input.extra_count;
		s_vpr_h(inst->sid, "g_min: input buffers %d\n", ctrl->val);
		break;
	default:
		break;
	}

	return rc;
}

static int vb2_buffer_to_driver(struct vb2_buffer *vb2, struct msm_vidc_buffer *buf)
{
	int rc = 0;

	if (!vb2 || !buf) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	buf->valid = true;
	buf->type = v4l2_type_to_driver(vb2->type);
	if (!buf->type)
		return -EINVAL;
	buf->index = vb2->index;
	buf->fd = vb2->planes[0].m.fd;
	buf->data_offset = vb2->planes[0].data_offset;
	buf->data_size = vb2->planes[0].bytesused;
	buf->buffer_size = vb2->planes[0].length;
	buf->timestamp = vb2->timestamp;

	return rc;
}

int msm_vidc_unmap_driver_buf(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buf)
{
	int rc = 0;
	struct msm_vidc_mappings *mappings;
	struct msm_vidc_map *map = NULL;
	bool found = false;

	if (!inst || !buf) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	mappings = msm_vidc_get_mappings(inst, buf->type, __func__);
	if (!mappings)
		return -EINVAL;

	/* sanity check to see if it was not removed */
	list_for_each_entry(map, &mappings->list, list) {
		if (map->dmabuf == buf->dmabuf) {
			found = true;
			break;
		}
	}
	if (!found) {
		print_vidc_buffer(VIDC_ERR, "no buf in mappings", inst, buf);
		return -EINVAL;
	}

	rc = msm_vidc_memory_unmap(inst->core, map);
	if (rc) {
		print_vidc_buffer(VIDC_ERR, "unmap failed", inst, buf);
		return -EINVAL;
	}

	/* finally delete if refcount is zero */
	if (!map->refcount) {
		list_del(&map->list);
		kfree(map);
	}

	return 0;
}

int msm_vidc_put_driver_buf(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buf)
{
	int rc = 0;

	if (!inst || !buf) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	/* do not unmap / delete read only buffer */
	if (buf->attr & MSM_VIDC_ATTR_READ_ONLY)
		return 0;

	rc = msm_vidc_unmap_driver_buf(inst, buf);
	if (rc)
		return rc;

	/* delete the buffer from buffers->list */
	list_del(&buf->list);
	kfree(buf);

	return 0;
}

int msm_vidc_map_driver_buf(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buf)
{
	int rc = 0;
	struct msm_vidc_mappings *mappings;
	struct msm_vidc_map *map = NULL;
	bool found = false;

	if (!inst || !buf) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	mappings = msm_vidc_get_mappings(inst, buf->type, __func__);
	if (!mappings)
		return -EINVAL;

	/* check if it is an existing one */
	list_for_each_entry(map, &mappings->list, list) {
		if (map->dmabuf == buf->dmabuf) {
			found = true;
			break;
		}
	}
	if (found) {
		/* skip mapping for RO buffer */
		if (!(buf->attr & MSM_VIDC_ATTR_READ_ONLY)) {
			rc = msm_vidc_memory_map(inst->core, map);
			if (rc)
				return -ENOMEM;
			buf->device_addr = map->device_addr;
		}
		return 0;
	}
	map = kzalloc(sizeof(struct msm_vidc_map), GFP_KERNEL);
	if (!map) {
		s_vpr_e(inst->sid, "%s: alloc failed\n", __func__);
		return -ENOMEM;
	}
	INIT_LIST_HEAD(&map->list);
	map->type = buf->type;
	map->dmabuf = buf->dmabuf;
	map->region = msm_vidc_get_buffer_region(inst, buf->type, __func__);
	rc = msm_vidc_memory_map(inst->core, map);
	if (rc) {
		kfree(map);
		return -ENOMEM;
	}
	buf->device_addr = map->device_addr;
	list_add_tail(&map->list, &mappings->list);

	return 0;
}

struct msm_vidc_buffer *msm_vidc_get_driver_buf(struct msm_vidc_inst *inst,
	struct vb2_buffer *vb2)
{
	int rc = 0;
	struct msm_vidc_buffer *buf = NULL;
	struct msm_vidc_buffers *buffers;
	struct dma_buf *dmabuf;
	enum msm_vidc_buffer_type buf_type;
	bool found = false;

	if (!inst || !vb2) {
		d_vpr_e("%s: invalid params\n", __func__);
		return NULL;
	}

	buf_type = v4l2_type_to_driver(vb2->type);
	if (!buf_type)
		return NULL;

	buffers = msm_vidc_get_buffers(inst, buf_type, __func__);
	if (!buffers)
		return NULL;

	dmabuf = msm_vidc_memory_get_dmabuf(vb2->planes[0].m.fd);
	if (!dmabuf)
		return NULL;
	msm_vidc_memory_put_dmabuf(dmabuf);

	/* check if it is an existing buffer */
	list_for_each_entry(buf, &buffers->list, list) {
		if (buf->dmabuf == dmabuf &&
		    buf->data_offset == vb2->planes[0].data_offset) {
			found = true;
			break;
		}
	}
	if (found) {
		/* only YUV buffers are allowed to repeat */
		if ((is_decode_session(inst) && vb2->type != OUTPUT_PLANE) ||
		    (is_encode_session(inst) && vb2->type != INPUT_PLANE)) {
			print_vidc_buffer(VIDC_ERR,
				"existing buffer", inst, buf);
			goto error;
		}
		/* for decoder, YUV with RO flag are allowed to repeat */
		if (is_decode_session(inst) &&
		    !(buf->attr & MSM_VIDC_ATTR_READ_ONLY)) {
			print_vidc_buffer(VIDC_ERR,
				"existing buffer without RO flag", inst, buf);
			goto error;
		}
		/* for encoder, treat the repeated buffer as new buffer */
		if (is_encode_session(inst) && vb2->type == INPUT_PLANE)
			found = false;
	} else {
		buf = kzalloc(sizeof(struct msm_vidc_buffer), GFP_KERNEL);
		if (!buf) {
			s_vpr_e(inst->sid, "%s: alloc failed\n", __func__);
			goto error;
		}
		buf->dmabuf = dmabuf;
	}
	rc = vb2_buffer_to_driver(vb2, buf);
	if (rc)
		goto error;

	if (!found)
		list_add_tail(&buf->list, &buffers->list);

	rc = msm_vidc_map_driver_buf(inst, buf);
	if (rc)
		goto error;

	return buf;

error:
	if (!found)
		kfree(buf);
	return NULL;
}

struct msm_vidc_buffer *get_meta_buffer(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buf)
{
	struct msm_vidc_buffer *mbuf;
	struct msm_vidc_buffers *meta;
	bool found = false;

	if (!inst || !buf) {
		d_vpr_e("%s: invalid params\n", __func__);
		return NULL;
	}

	if (buf->type == MSM_VIDC_BUF_INPUT) {
		meta = &inst->buffers.input_meta;
	} else if (buf->type == MSM_VIDC_BUF_OUTPUT) {
		meta = &inst->buffers.output_meta;
	} else {
		s_vpr_e(inst->sid, "%s: invalid buffer type %d\n",
			__func__, buf->type);
		return NULL;
	}
	list_for_each_entry(mbuf, &meta->list, list) {
		if (!mbuf->valid)
			continue;
		if (mbuf->type == buf->type &&
		    mbuf->index == buf->index) {
			found = true;
			break;
		}
	}
	if (!found)
		return NULL;

	return mbuf;
}

int msm_vidc_queue_buffer(struct msm_vidc_inst *inst, struct vb2_buffer *vb2)
{
	int rc = 0;
	struct msm_vidc_buffer *buf;
	int port;

	if (!inst || !vb2) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	buf = msm_vidc_get_driver_buf(inst, vb2);
	if (!buf)
		return -EINVAL;

	/* meta buffer will be queued along with actual buffer */
	if (buf->type == MSM_VIDC_BUF_INPUT_META ||
	    buf->type == MSM_VIDC_BUF_OUTPUT_META)
		return 0;

	/* skip queuing if streamon not completed */
	port = v4l2_type_to_driver_port(inst, vb2->type, __func__);
	if (port < 0)
		return -EINVAL;
	if (!inst->vb2q[port].streaming) {
		buf->attr |= MSM_VIDC_ATTR_DEFERRED;
		print_vidc_buffer(VIDC_HIGH, "qbuf deferred", inst, buf);
		return 0;
	}

	print_vidc_buffer(VIDC_HIGH, "qbuf", inst, buf);
	rc = venus_hfi_queue_buffer(inst, buf, get_meta_buffer(inst, buf));
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_create_internal_buffers(struct msm_vidc_inst *inst,
		enum msm_vidc_buffer_type buffer_type)
{
	int rc = 0;
	struct msm_vidc_buffers *buffers;
	struct msm_vidc_allocations *allocations;
	struct msm_vidc_mappings *mappings;
	int i;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	if (!is_internal_buffer(buffer_type)) {
		s_vpr_e(inst->sid, "%s: buffer type %#x is not internal\n",
			__func__, buffer_type);
		return 0;
	}

	buffers = msm_vidc_get_buffers(inst, buffer_type, __func__);
	if (!buffers)
		return -EINVAL;
	allocations = msm_vidc_get_allocations(inst, buffer_type, __func__);
	if (!allocations)
		return -EINVAL;
	mappings = msm_vidc_get_mappings(inst, buffer_type, __func__);
	if (!mappings)
		return -EINVAL;

	for (i = 0; i < buffers->min_count; i++) {
		struct msm_vidc_buffer *buffer;
		struct msm_vidc_alloc *alloc;
		struct msm_vidc_map *map;

		if (!buffers->size) {
			s_vpr_e(inst->sid, "%s: invalid buffer %#x\n",
				__func__, buffer_type);
			return -EINVAL;
		}
		buffer = kzalloc(sizeof(struct msm_vidc_buffer), GFP_KERNEL);
		if (!buffer) {
			s_vpr_e(inst->sid, "%s: buf alloc failed\n", __func__);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&buffer->list);
		buffer->valid = true;
		buffer->type = buffer_type;
		buffer->index = i;
		buffer->buffer_size = buffers->size;
		list_add_tail(&buffer->list, &buffers->list);

		alloc = kzalloc(sizeof(struct msm_vidc_alloc), GFP_KERNEL);
		if (!alloc) {
			s_vpr_e(inst->sid, "%s: alloc failed\n", __func__);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&alloc->list);
		alloc->type = buffer_type;
		alloc->region = msm_vidc_get_buffer_region(inst,
					buffer_type, __func__);
		alloc->size = buffer->buffer_size;
		rc = msm_vidc_memory_alloc(inst->core, alloc);
		if (rc)
			return -ENOMEM;
		list_add_tail(&alloc->list, &allocations->list);

		map = kzalloc(sizeof(struct msm_vidc_map), GFP_KERNEL);
		if (!map) {
			s_vpr_e(inst->sid, "%s: map alloc failed\n", __func__);
			return -ENOMEM;
		}
		INIT_LIST_HEAD(&map->list);
		map->type = alloc->type;
		map->region = alloc->region;
		map->dmabuf = alloc->dmabuf;
		rc = msm_vidc_memory_map(inst->core, map);
		if (rc)
			return -ENOMEM;
		list_add_tail(&map->list, &mappings->list);

		s_vpr_e(inst->sid, "%s: created buffer_type %#x, size %d\n",
			__func__, buffer_type, buffers->size);
	}

	return 0;
}

int msm_vidc_queue_internal_buffers(struct msm_vidc_inst *inst,
		enum msm_vidc_buffer_type buffer_type)
{
	int rc = 0;
	struct msm_vidc_buffers *buffers;
	struct msm_vidc_buffer *buffer, *dummy;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	if (!is_internal_buffer(buffer_type)) {
		s_vpr_e(inst->sid, "%s: buffer type %#x is not internal\n",
			__func__, buffer_type);
		return 0;
	}

	buffers = msm_vidc_get_buffers(inst, buffer_type, __func__);
	if (!buffers)
		return -EINVAL;

	list_for_each_entry_safe(buffer, dummy, &buffers->list, list) {
		/* do not queue pending release buffers */
		if (buffer->flags & MSM_VIDC_ATTR_PENDING_RELEASE)
			continue;
		/* do not queue already queued buffers */
		if (buffer->attr & MSM_VIDC_ATTR_QUEUED)
			continue;
		rc = venus_hfi_queue_buffer(inst, buffer, NULL);
		if (rc)
			return rc;
		/* mark queued */
		buffer->attr |= MSM_VIDC_ATTR_QUEUED;

		s_vpr_h(inst->sid, "%s: queued buffer_type %#x, size %d\n",
			__func__, buffer_type, buffers->size);
	}

	return 0;
}

int msm_vidc_release_internal_buffers(struct msm_vidc_inst *inst,
		enum msm_vidc_buffer_type buffer_type)
{
	int rc = 0;
	struct msm_vidc_buffers *buffers;
	struct msm_vidc_buffer *buffer, *dummy;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	if (!is_internal_buffer(buffer_type)) {
		s_vpr_e(inst->sid, "%s: buffer type %#x is not internal\n",
			__func__, buffer_type);
		return 0;
	}

	buffers = msm_vidc_get_buffers(inst, buffer_type, __func__);
	if (!buffers)
		return -EINVAL;

	list_for_each_entry_safe(buffer, dummy, &buffers->list, list) {
		/* do not release already pending release buffers */
		if (buffer->attr & MSM_VIDC_ATTR_PENDING_RELEASE)
			continue;
		/* release only queued buffers */
		if (!(buffer->attr & MSM_VIDC_ATTR_QUEUED))
			continue;
		rc = venus_hfi_release_buffer(inst, buffer);
		if (rc)
			return rc;
		/* mark pending release */
		buffer->attr |= MSM_VIDC_ATTR_PENDING_RELEASE;

		s_vpr_e(inst->sid, "%s: released buffer_type %#x, size %d\n",
			__func__, buffer_type, buffers->size);
	}

	return 0;
}

int msm_vidc_setup_event_queue(struct msm_vidc_inst *inst)
{
	int rc = 0;
	int index;
	struct msm_vidc_core *core;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	// TODO: check decode is index = 0 and encode is index 1
	if (is_decode_session(inst))
		index = 0;
	else if (is_encode_session(inst))
		index = 1;
	else
		return -EINVAL;

	v4l2_fh_init(&inst->event_handler, &core->vdev[index].vdev);
	v4l2_fh_add(&inst->event_handler);

	return rc;
}

static int vb2q_init(struct msm_vidc_inst *inst,
	struct vb2_queue *q, enum v4l2_buf_type type)
{
	struct msm_vidc_core *core;

	if (!inst || !q || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	q->type = type;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->ops = core->vb2_ops;
	q->mem_ops = core->vb2_mem_ops;
	q->drv_priv = inst;
	q->allow_zero_bytesused = 1;
	q->copy_timestamp = 1;
	return vb2_queue_init(q);
}

int msm_vidc_vb2_queue_init(struct msm_vidc_inst *inst)
{
	int rc = 0;

	d_vpr_h("%s()\n", __func__);
	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = vb2q_init(inst, &inst->vb2q[INPUT_PORT], INPUT_PLANE);
	if (rc)
		return rc;

	rc = vb2q_init(inst, &inst->vb2q[OUTPUT_PORT], OUTPUT_PLANE);
	if (rc)
		return rc;

	rc = vb2q_init(inst, &inst->vb2q[INPUT_META_PORT], INPUT_META_PLANE);
	if (rc)
		return rc;

	rc = vb2q_init(inst, &inst->vb2q[OUTPUT_META_PORT], OUTPUT_META_PLANE);
	if (rc)
		return rc;

	return rc;
}

int msm_vidc_add_session(struct msm_vidc_inst *inst)
{
	int rc = 0;
	struct msm_vidc_inst *i;
	struct msm_vidc_core *core;
	u32 count = 0;

	if (!inst || !inst->core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	mutex_lock(&core->lock);
	list_for_each_entry(i, &core->instances, list)
		count++;

	if (count < 0xffffff /*TODO: MAX_SUPPORTED_INSTANCES*/) {
		list_add_tail(&inst->list, &core->instances);
	} else {
		d_vpr_e("%s: total sessions %d exceeded max limit %d\n",
			__func__, count, MAX_SUPPORTED_INSTANCES);
		rc = -EINVAL;
	}
	mutex_unlock(&core->lock);

	/* assign session_id */
	inst->session_id = count + 1;
	inst->sid = inst->session_id;

	return rc;
}

int msm_vidc_session_open(struct msm_vidc_inst *inst)
{
	int rc = 0;

	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	rc = venus_hfi_session_open(inst);
	if (rc)
		return rc;

	inst->session_created = true;
	return 0;
}

int msm_vidc_get_inst_capability(struct msm_vidc_inst *inst)
{
	int rc = 0;
	int i;
	struct msm_vidc_core *core;

	d_vpr_h("%s()\n", __func__);
	if (!inst || !inst->core || !inst->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}
	core = inst->core;

	for (i = 0; i < core->codecs_count; i++) {
		if (core->inst_caps[i].domain == inst->domain &&
			core->inst_caps[i].codec == inst->codec) {
			s_vpr_h(inst->sid,
				"%s: copied capabilities with %#x codec\n",
				__func__, inst->codec);
			memcpy(inst->capabilities, &core->inst_caps[i],
				sizeof(struct msm_vidc_inst_capability));
		}
	}
	if (!inst->capabilities) {
		s_vpr_e(inst->sid, "%s: capabilities not found\n", __func__);
		return -EINVAL;
	}

	return rc;
}

static int msm_vidc_init_core_caps(struct msm_vidc_core *core)
{
	int rc = 0;
	int i, num_platform_caps;
	struct msm_platform_core_capability *platform_data;

	if (!core || !core->platform) {
		d_vpr_e("%s: invalid params\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	platform_data = core->platform->data.core_data;
	if (!platform_data) {
		d_vpr_e("%s: platform core data is NULL\n",
				__func__);
			rc = -EINVAL;
			goto exit;
	}

	if (!core->capabilities) {
		core->capabilities = kcalloc(1,
			(sizeof(struct msm_vidc_core_capability) *
			CORE_CAP_MAX), GFP_KERNEL);
		if (!core->capabilities) {
			d_vpr_e("%s: failed to allocate core capabilities\n",
				__func__);
			rc = -ENOMEM;
			goto exit;
		}
	} else {
		d_vpr_e("%s: capabilities memory is expected to be freed\n",
			__func__);
	}

	num_platform_caps = core->platform->data.core_data_size;

	/* loop over platform caps */
	for (i = 0; i < num_platform_caps; i++) {
		core->capabilities[platform_data[i].type].type = platform_data[i].type;
		core->capabilities[platform_data[i].type].value = platform_data[i].value;
	}

exit:
	return rc;
}

static void update_inst_capability(struct msm_platform_inst_capability *in,
		struct msm_vidc_inst_capability *capability)
{
	if (!in || !capability) {
		d_vpr_e("%s: invalid params %pK %pK\n",
			__func__, in, capability);
		return;
	}
	if (in->cap < INST_CAP_MAX) {
		capability->cap[in->cap].cap = in->cap;
		capability->cap[in->cap].min = in->min;
		capability->cap[in->cap].max = in->max;
		capability->cap[in->cap].step_or_mask = in->step_or_mask;
		capability->cap[in->cap].value = in->value;
		capability->cap[in->cap].flags = in->flags;
		capability->cap[in->cap].v4l2_id = in->v4l2_id;
		capability->cap[in->cap].hfi_id = in->hfi_id;
		memcpy(capability->cap[in->cap].parents, in->parents,
			sizeof(capability->cap[in->cap].parents));
		memcpy(capability->cap[in->cap].children, in->children,
			sizeof(capability->cap[in->cap].children));
		capability->cap[in->cap].adjust = in->adjust;
		capability->cap[in->cap].set = in->set;
	} else {
		d_vpr_e("%s: invalid cap %d\n",
			__func__, in->cap);
	}
}

static int msm_vidc_init_instance_caps(struct msm_vidc_core *core)
{
	int rc = 0;
	u8 enc_valid_codecs, dec_valid_codecs;
	u8 count_bits, enc_codec_count;
	u8 codecs_count = 0;
	int i, j, check_bit, num_platform_caps;
	struct msm_platform_inst_capability *platform_data = NULL;

	if (!core || !core->platform || !core->capabilities) {
		d_vpr_e("%s: invalid params\n", __func__);
		rc = -EINVAL;
		goto exit;
	}

	platform_data = core->platform->data.instance_data;
	if (!platform_data) {
		d_vpr_e("%s: platform instance data is NULL\n",
				__func__);
			rc = -EINVAL;
			goto exit;
	}

	enc_valid_codecs = core->capabilities[ENC_CODECS].value;
	count_bits = enc_valid_codecs;
	COUNT_BITS(count_bits, codecs_count);
	enc_codec_count = codecs_count;

	dec_valid_codecs = core->capabilities[DEC_CODECS].value;
	count_bits = dec_valid_codecs;
	COUNT_BITS(count_bits, codecs_count);

	core->codecs_count = codecs_count;

	if (!core->inst_caps) {
		core->inst_caps = kcalloc(codecs_count,
			sizeof(struct msm_vidc_inst_capability),
			GFP_KERNEL);
		if (!core->inst_caps) {
			d_vpr_e("%s: failed to allocate core capabilities\n",
				__func__);
			rc = -ENOMEM;
			goto exit;
		}
	} else {
		d_vpr_e("%s: capabilities memory is expected to be freed\n",
			__func__);
	}

	check_bit = 0;
	/* determine codecs for enc domain */
	for (i = 0; i < enc_codec_count; i++) {
		while (check_bit < (sizeof(enc_valid_codecs) * 8)) {
			if (enc_valid_codecs & BIT(check_bit)) {
				core->inst_caps[i].domain = MSM_VIDC_ENCODER;
				core->inst_caps[i].codec = enc_valid_codecs &
						BIT(check_bit);
				check_bit++;
				break;
			}
			check_bit++;
		}
	}

	/* reset checkbit to check from 0th bit of decoder codecs set bits*/
	check_bit = 0;
	/* determine codecs for dec domain */
	for (; i < codecs_count; i++) {
		while (check_bit < (sizeof(dec_valid_codecs) * 8)) {
			if (dec_valid_codecs & BIT(check_bit)) {
				core->inst_caps[i].domain = MSM_VIDC_DECODER;
				core->inst_caps[i].codec = dec_valid_codecs &
						BIT(check_bit);
				check_bit++;
				break;
			}
			check_bit++;
		}
	}

	num_platform_caps = core->platform->data.instance_data_size;

	d_vpr_h("%s: num caps %d\n", __func__, num_platform_caps);
	/* loop over each platform capability */
	for (i = 0; i < num_platform_caps; i++) {
		/* select matching core codec and update it */
		for (j = 0; j < codecs_count; j++) {
			if ((platform_data[i].domain &
				core->inst_caps[j].domain) &&
				(platform_data[i].codec &
				core->inst_caps[j].codec)) {
				/* update core capability */
				update_inst_capability(&platform_data[i],
					&core->inst_caps[j]);
			}
		}
	}
exit:
	return rc;
}

int msm_vidc_core_init(struct msm_vidc_core *core)
{
	int rc = 0;

	d_vpr_h("%s()\n", __func__);
	if (!core || !core->platform) {
		d_vpr_e("%s: invalid params\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&core->lock);
	if (core->state == MSM_VIDC_CORE_ERROR) {
		d_vpr_e("%s: core invalid state\n", __func__);
		rc = -EINVAL;
		goto unlock;
	}
	if (core->state == MSM_VIDC_CORE_INIT) {
		rc = 0;
		goto unlock;
	}

	rc = msm_vidc_init_core_caps(core);
	if (rc)
		goto unlock;
	rc = msm_vidc_init_instance_caps(core);
	if (rc)
		goto unlock;

	core->state = MSM_VIDC_CORE_INIT;
	init_completion(&core->init_done);
	core->smmu_fault_handled = false;
	core->ssr.trigger = false;

	rc = venus_hfi_core_init(core);
	if (rc) {
		d_vpr_e("%s: core init failed\n", __func__);
		core->state = MSM_VIDC_CORE_DEINIT;
		goto unlock;
	}

	mutex_unlock(&core->lock);
	/*TODO: acquire lock or not */
	d_vpr_h("%s(): waiting for sys init done, %d ms\n", __func__,
		core->platform->data.core_data[HW_RESPONSE_TIMEOUT].value);
	rc = wait_for_completion_timeout(&core->init_done, msecs_to_jiffies(
			core->platform->data.core_data[HW_RESPONSE_TIMEOUT].value));
	if (!rc) {
		d_vpr_e("%s: system init timed out\n", __func__);
		//msm_comm_kill_session(inst);
		//rc = -EIO;
	} else {
		d_vpr_h("%s: system init wait completed\n", __func__);
		rc = 0;
	}
	mutex_lock(&core->lock);

unlock:
	mutex_unlock(&core->lock);
	return rc;
}

int msm_vidc_smmu_fault_handler(struct iommu_domain *domain,
		struct device *dev, unsigned long iova, int flags, void *data)
{
	return -EINVAL;
}

int msm_vidc_trigger_ssr(struct msm_vidc_core *core,
		enum msm_vidc_ssr_trigger_type type)
{
	return 0;
}

void msm_vidc_ssr_handler(struct work_struct *work)
{
}

void msm_vidc_pm_work_handler(struct work_struct *work)
{
}

void msm_vidc_fw_unload_handler(struct work_struct *work)
{
}

void msm_vidc_batch_handler(struct work_struct *work)
{
}

struct msm_vidc_inst *get_inst(struct msm_vidc_core *core,
		u32 session_id)
{
	struct msm_vidc_inst *inst = NULL;
	bool matches = false;

	if (!core) {
		d_vpr_e("%s: invalid params\n", __func__);
		return NULL;
	}

	mutex_lock(&core->lock);
	list_for_each_entry(inst, &core->instances, list) {
		if (inst->session_id == session_id) {
			matches = true;
			break;
		}
	}
	inst = (matches && kref_get_unless_zero(&inst->kref)) ? inst : NULL;
	mutex_unlock(&core->lock);
	return inst;
}

static void put_inst_helper(struct kref *kref)
{
	struct msm_vidc_inst *inst = container_of(kref,
			struct msm_vidc_inst, kref);

	msm_vidc_close(inst);
}

void put_inst(struct msm_vidc_inst *inst)
{
	if (!inst) {
		d_vpr_e("%s: invalid params\n", __func__);
		return;
	}

	kref_put(&inst->kref, put_inst_helper);
}

void core_lock(struct msm_vidc_core *core, const char *function)
{
	mutex_lock(&core->lock);
}

void core_unlock(struct msm_vidc_core *core, const char *function)
{
	mutex_unlock(&core->lock);
}

void inst_lock(struct msm_vidc_inst *inst, const char *function)
{
	mutex_lock(&inst->lock);
}

void inst_unlock(struct msm_vidc_inst *inst, const char *function)
{
	mutex_unlock(&inst->lock);
}
