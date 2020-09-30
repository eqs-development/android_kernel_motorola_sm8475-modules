/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _HFI_PACKET_H_
#define _HFI_PACKET_H_

#include "msm_vidc_internal.h"
#include "msm_vidc_inst.h"
#include "msm_vidc_core.h"
#include "hfi_command.h"
#include "hfi_property.h"
#include "hfi_definition.h"

u32 get_hfi_port(struct msm_vidc_inst *inst,
	enum msm_vidc_buffer_type buffer_type);
u32 get_hfi_buffer_type(enum msm_vidc_domain_type domain,
	enum msm_vidc_buffer_type buffer_type);
u32 get_hfi_codec(struct msm_vidc_inst *inst);
int get_hfi_buffer(struct msm_vidc_inst *inst,
	struct msm_vidc_buffer *buffer, struct hfi_buffer *buf);
int hfi_create_header(u8 *packet, u32 packet_size,
	u32 session_id, u32 header_id);
int hfi_create_packet(u8 *packet, u32 packet_size,
	u32 pkt_type, u32 pkt_flags, u32 payload_type, u32 port,
	u32 packet_id, void *payload, u32 payload_size);
int hfi_create_buffer(u8 *packet, u32 packet_size, u32 *offset,
	enum msm_vidc_domain_type domain, struct msm_vidc_buffer *data);

int hfi_packet_sys_init(struct msm_vidc_core *core,
	u8 *pkt, u32 pkt_size);
int hfi_packet_image_version(struct msm_vidc_core *core,
	u8 *pkt, u32 pkt_size);
int hfi_packet_sys_pc_prep(struct msm_vidc_core *core,
	u8 *pkt, u32 pkt_size);
int hfi_packet_sys_debug_config(struct msm_vidc_core *core,
	u8 *pkt, u32 pkt_size, u32 debug_config);
int hfi_packet_session_command(struct msm_vidc_inst *inst,
	u32 pkt_type, u32 flags, u32 port, u32 session_id,
	u32 payload_type, void *payload, u32 payload_size);
int hfi_packet_session_property(struct msm_vidc_inst *inst,
	u32 pkt_type, u32 flags, u32 port,
	u32 payload_type, void *payload, u32 payload_size);
#endif // _HFI_PACKET_H_
