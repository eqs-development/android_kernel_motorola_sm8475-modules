/*
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt)	"msm-dsi-display:[%s] " fmt, __func__

#include <linux/err.h>
#include <linux/string.h>

#include "dsi_display.h"
#include "dsi_panel.h"
#include "dsi_display_mot_ext.h"

int mot_atoi(const char *src)
{
      int s = 0;
      bool isMinus = false;

      if (!src)
          return 0;

      while(*src == ' ')
      {
          src++;
      }

      if(*src == '+' || *src == '-')
      {
          if(*src == '-')
          {
              isMinus = true;
          }
          src++;
      }
      else if(*src < '0' || *src > '9')
      {
          s = 2147483647;
          return s;
      }

      while(*src != '\0' && *src >= '0' && *src <= '9')
      {
          s = s * 10 + *src - '0';
          src++;
      }
      return s * (isMinus ? -1 : 1);
}

void mot_update_hbmoff(struct dsi_panel *panel,
                        struct msm_param_info *param_info)
{
	struct panel_param_val_map *param_map;
	struct panel_param_val_map *param_map_state;
	struct dsi_cmd_desc *cmds;
	u32 count;
	u8 *payload;
	param_map = panel->param_cmds[param_info->param_idx].val_map;
	param_map_state = &param_map[param_info->value];
	cmds = param_map_state->cmds->cmds;
	count = param_map_state->cmds->count;
	cmds++;
	payload = (u8 *)cmds->msg.tx_buf;
	if(panel->dc_on)
		payload[1] = 0x00;
	else
		payload[1] = 0xD0;
	pr_info("mot_update_hbmoff payload[0] = 0x%x payload[1] = 0x%x",payload[0],payload[1]);
	cmds->msg.tx_buf = payload;
}

void mot_swtich_base(struct dsi_display *display, u32 refresh_rate)
{
	if(refresh_rate == 90){
		display->panel->refresh_rate_base = RRGSFlag_90HzBased;
		display->panel->switch_rate_base = false;
		pr_info("Enter refresh_rate_group = RRGSFlag_90HzBased\n");
	}else if(refresh_rate ==30 || refresh_rate ==10 || refresh_rate == 1){
		display->panel->switch_rate_base = true;
		pr_info("30hz/10hz/1hz base no change.\n");
	}
	else{
		display->panel->refresh_rate_base = RRGSFlag_120HzBased;
		display->panel->switch_rate_base = false;
		pr_info("Enter refresh_rate_group = RRGSFlag_120HzBased\n");
	}

}
void update_dc_cmd_nt37705(struct dsi_panel *panel,
                        struct panel_param_val_map *param_map_state)
{
	u32 i;
	u32 count;
	u8 *payload;
	struct dsi_cmd_desc *cmds;

	if (!param_map_state) {
		DSI_ERR("Invalid Params\n");
		return;
	}
	count = param_map_state->cmds->count;
	cmds = param_map_state->cmds->cmds;

	for ( i =0; i < count; i++,cmds++) {
		payload = (u8 *)cmds->msg.tx_buf;
		if (payload[0] == 0X6D) {
			if(panel->cur_mode->timing.refresh_rate == 30 && panel->refresh_rate_base == RRGSFlag_90HzBased) payload[1] = 0x05;
			else if(panel->cur_mode->timing.refresh_rate == 30)payload[1] = 0x01;
			else if(panel->cur_mode->timing.refresh_rate == 24)payload[1] = 0x02;
			else if(panel->cur_mode->timing.refresh_rate == 10)payload[1] = 0x03;
			else if(panel->cur_mode->timing.refresh_rate == 1)payload[1] = 0x04;
			else payload[1] = 0x00;
			pr_debug("%s: payload[0] = 0x%x payload[1] = 0x%x", __func__, payload[0],payload[1]);
		}
	}
}
