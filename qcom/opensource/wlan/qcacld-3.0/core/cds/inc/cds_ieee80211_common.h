/*
 * Copyright (c) 2011,2014-2019 The Linux Foundation. All rights reserved.
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

#ifndef EXTERNAL_USE_ONLY
#include "osdep.h"
#endif /* EXTERNAL_USE_ONLY */
#include "cds_ieee80211_common_i.h"
#include "cdp_txrx_mob_def.h"

#ifndef CDS_COMMON_IEEE80211_H_
#define CDS_COMMON_IEEE80211_H_

/*
 * generic definitions for IEEE 802.11 frames
 */
struct ieee80211_frame {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	union {
		struct {
			uint8_t i_addr1[IEEE80211_ADDR_LEN];
			uint8_t i_addr2[IEEE80211_ADDR_LEN];
			uint8_t i_addr3[IEEE80211_ADDR_LEN];
		};
		uint8_t i_addr_all[3 * IEEE80211_ADDR_LEN];
	};
	uint8_t i_seq[2];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __packed;

struct ieee80211_qosframe {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_qos[2];
	/* possibly followed by addr4[IEEE80211_ADDR_LEN]; */
	/* see below */
} __packed;

struct ieee80211_qoscntl {
	uint8_t i_qos[2];
};

struct ieee80211_frame_addr4 {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_addr4[IEEE80211_ADDR_LEN];
} __packed;

struct ieee80211_qosframe_addr4 {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_addr4[IEEE80211_ADDR_LEN];
	uint8_t i_qos[2];
} __packed;

/* HTC frame for TxBF*/
/* for TxBF RC */
struct ieee80211_frame_min_one {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
} __packed;                     /* For TxBF RC */

struct ieee80211_qosframe_htc_addr4 {
	uint8_t i_fc[2];
	uint8_t i_dur[2];
	uint8_t i_addr1[IEEE80211_ADDR_LEN];
	uint8_t i_addr2[IEEE80211_ADDR_LEN];
	uint8_t i_addr3[IEEE80211_ADDR_LEN];
	uint8_t i_seq[2];
	uint8_t i_addr4[IEEE80211_ADDR_LEN];
	uint8_t i_qos[2];
	uint8_t i_htc[4];
} __packed;

struct ieee80211_htc {
	uint8_t i_htc[4];
};

#define IEEE80211_FC0_VERSION_MASK          0x03
#define IEEE80211_FC0_VERSION_SHIFT         0
#define IEEE80211_FC0_VERSION_0             0x00
#define IEEE80211_FC0_TYPE_MASK             0x0c
#define IEEE80211_FC0_TYPE_SHIFT            2
#define IEEE80211_FC0_TYPE_MGT              0x00
#define IEEE80211_FC0_TYPE_CTL              0x04
#define IEEE80211_FC0_TYPE_DATA             0x08

#define IEEE80211_FC0_SUBTYPE_MASK          0xf0
#define IEEE80211_FC0_SUBTYPE_SHIFT         4
/* for TYPE_MGT */
#define IEEE80211_FC0_SUBTYPE_ASSOC_REQ     0x00
#define IEEE80211_FC0_SUBTYPE_ASSOC_RESP    0x10
#define IEEE80211_FC0_SUBTYPE_REASSOC_REQ   0x20
#define IEEE80211_FC0_SUBTYPE_REASSOC_RESP  0x30
#define IEEE80211_FC0_SUBTYPE_PROBE_REQ     0x40
#define IEEE80211_FC0_SUBTYPE_PROBE_RESP    0x50
#define IEEE80211_FC0_SUBTYPE_BEACON        0x80
#define IEEE80211_FC0_SUBTYPE_ATIM          0x90
#define IEEE80211_FC0_SUBTYPE_DISASSOC      0xa0
#define IEEE80211_FC0_SUBTYPE_AUTH          0xb0
#define IEEE80211_FC0_SUBTYPE_DEAUTH        0xc0
#define IEEE80211_FC0_SUBTYPE_ACTION        0xd0
#define IEEE80211_FCO_SUBTYPE_ACTION_NO_ACK 0xe0
/* for TYPE_CTL */
#define IEEE80211_FCO_SUBTYPE_Control_Wrapper   0x70    /* For TxBF RC */
#define IEEE80211_FC0_SUBTYPE_BAR           0x80
#define IEEE80211_FC0_SUBTYPE_PS_POLL       0xa0
#define IEEE80211_FC0_SUBTYPE_RTS           0xb0
#define IEEE80211_FC0_SUBTYPE_CTS           0xc0
#define IEEE80211_FC0_SUBTYPE_ACK           0xd0
#define IEEE80211_FC0_SUBTYPE_CF_END        0xe0
#define IEEE80211_FC0_SUBTYPE_CF_END_ACK    0xf0
/* for TYPE_DATA (bit combination) */
#define IEEE80211_FC0_SUBTYPE_DATA          0x00
#define IEEE80211_FC0_SUBTYPE_CF_ACK        0x10
#define IEEE80211_FC0_SUBTYPE_CF_POLL       0x20
#define IEEE80211_FC0_SUBTYPE_CF_ACPL       0x30
#define IEEE80211_FC0_SUBTYPE_NODATA        0x40
#define IEEE80211_FC0_SUBTYPE_CFACK         0x50
#define IEEE80211_FC0_SUBTYPE_CFPOLL        0x60
#define IEEE80211_FC0_SUBTYPE_CF_ACK_CF_ACK 0x70
#define IEEE80211_FC0_SUBTYPE_QOS           0x80
#define IEEE80211_FC0_SUBTYPE_QOS_NULL      0xc0

#define IEEE80211_FC1_DIR_MASK              0x03
#define IEEE80211_FC1_DIR_NODS              0x00        /* STA->STA */
#define IEEE80211_FC1_DIR_TODS              0x01        /* STA->AP  */
#define IEEE80211_FC1_DIR_FROMDS            0x02        /* AP ->STA */
#define IEEE80211_FC1_DIR_DSTODS            0x03        /* AP ->AP  */

#define IEEE80211_FC1_MORE_FRAG             0x04
#define IEEE80211_FC1_RETRY                 0x08
#define IEEE80211_FC1_PWR_MGT               0x10
#define IEEE80211_FC1_MORE_DATA             0x20
#define IEEE80211_FC1_ORDER                 0x80

#define IEEE80211_SEQ_FRAG_MASK             0x000f
#define IEEE80211_SEQ_FRAG_SHIFT            0
#define IEEE80211_SEQ_SEQ_MASK              0xfff0
#define IEEE80211_SEQ_SEQ_SHIFT             4
#define IEEE80211_SEQ_MAX                   4096

#define IEEE80211_QOS_AMSDU                 0x80
#define IEEE80211_QOS_ACKPOLICY_S           5
#define IEEE80211_QOS_TID                   0x0f

#define IEEE80211_IS_DATA(_frame)      (((_frame)->i_fc[0] & IEEE80211_FC0_TYPE_MASK) == IEEE80211_FC0_TYPE_DATA)

#define IEEE8023_MAX_LEN 0x600  /* 1536 - larger is Ethernet II */

/* does frame have QoS sequence control data */
#define IEEE80211_QOS_HAS_SEQ(wh) \
	(((wh)->i_fc[0] & \
	  (IEEE80211_FC0_TYPE_MASK | IEEE80211_FC0_SUBTYPE_QOS)) == \
	 (IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS))

#define IEEE80211_HTCAP_MAXRXAMPDU_FACTOR   13

/*
 * Management information element payloads.
 */
enum {
	IEEE80211_ELEMID_SSID = 0,
	IEEE80211_ELEMID_RATES = 1,
	IEEE80211_ELEMID_FHPARMS = 2,
	IEEE80211_ELEMID_DSPARMS = 3,
	IEEE80211_ELEMID_CFPARMS = 4,
	IEEE80211_ELEMID_TIM = 5,
	IEEE80211_ELEMID_IBSSPARMS = 6,
	IEEE80211_ELEMID_COUNTRY = 7,
	IEEE80211_ELEMID_REQINFO = 10,
	IEEE80211_ELEMID_QBSS_LOAD = 11,
	IEEE80211_ELEMID_TCLAS = 14,
	IEEE80211_ELEMID_CHALLENGE = 16,
	/* 17-31 reserved for challenge text extension */
	IEEE80211_ELEMID_PWRCNSTR = 32,
	IEEE80211_ELEMID_PWRCAP = 33,
	IEEE80211_ELEMID_TPCREQ = 34,
	IEEE80211_ELEMID_TPCREP = 35,
	IEEE80211_ELEMID_SUPPCHAN = 36,
	IEEE80211_ELEMID_CHANSWITCHANN = 37,
	IEEE80211_ELEMID_MEASREQ = 38,
	IEEE80211_ELEMID_MEASREP = 39,
	IEEE80211_ELEMID_QUIET = 40,
	IEEE80211_ELEMID_IBSSDFS = 41,
	IEEE80211_ELEMID_ERP = 42,
	IEEE80211_ELEMID_TCLAS_PROCESS = 44,
	IEEE80211_ELEMID_HTCAP_ANA = 45,
	IEEE80211_ELEMID_RESERVED_47 = 47,
	IEEE80211_ELEMID_RSN = 48,
	IEEE80211_ELEMID_XRATES = 50,
	IEEE80211_ELEMID_HTCAP = 51,
	IEEE80211_ELEMID_HTINFO = 52,
	IEEE80211_ELEMID_MOBILITY_DOMAIN = 54,
	IEEE80211_ELEMID_FT = 55,
	IEEE80211_ELEMID_TIMEOUT_INTERVAL = 56,
	IEEE80211_ELEMID_EXTCHANSWITCHANN = 60,
	IEEE80211_ELEMID_HTINFO_ANA = 61,
	IEEE80211_ELEMID_SECCHANOFFSET = 62,
	IEEE80211_ELEMID_WAPI = 68,     /*IE for WAPI */
	IEEE80211_ELEMID_TIME_ADVERTISEMENT = 69,
	IEEE80211_ELEMID_RRM = 70,      /* Radio resource measurement */
	IEEE80211_ELEMID_2040_COEXT = 72,
	IEEE80211_ELEMID_2040_INTOL = 73,
	IEEE80211_ELEMID_OBSS_SCAN = 74,
	IEEE80211_ELEMID_MMIE = 76,     /* 802.11w Management MIC IE */
	IEEE80211_ELEMID_FMS_DESCRIPTOR = 86,   /* 802.11v FMS descriptor IE */
	IEEE80211_ELEMID_FMS_REQUEST = 87,      /* 802.11v FMS request IE */
	IEEE80211_ELEMID_FMS_RESPONSE = 88,     /* 802.11v FMS response IE */
	IEEE80211_ELEMID_BSSMAX_IDLE_PERIOD = 90,       /* BSS MAX IDLE PERIOD */
	IEEE80211_ELEMID_TFS_REQUEST = 91,
	IEEE80211_ELEMID_TFS_RESPONSE = 92,
	IEEE80211_ELEMID_TIM_BCAST_REQUEST = 94,
	IEEE80211_ELEMID_TIM_BCAST_RESPONSE = 95,
	IEEE80211_ELEMID_INTERWORKING = 107,
	IEEE80211_ELEMID_XCAPS = 127,
	IEEE80211_ELEMID_RESERVED_133 = 133,
	IEEE80211_ELEMID_TPC = 150,
	IEEE80211_ELEMID_CCKM = 156,
	IEEE80211_ELEMID_VHTCAP = 191,  /* VHT Capabilities */
	IEEE80211_ELEMID_VHTOP = 192,   /* VHT Operation */
	IEEE80211_ELEMID_EXT_BSS_LOAD = 193,    /* Extended BSS Load */
	IEEE80211_ELEMID_WIDE_BAND_CHAN_SWITCH = 194,   /* Wide Band Channel Switch */
	IEEE80211_ELEMID_VHT_TX_PWR_ENVLP = 195,        /* VHT Transmit Power Envelope */
	IEEE80211_ELEMID_CHAN_SWITCH_WRAP = 196,        /* Channel Switch Wrapper */
	IEEE80211_ELEMID_AID = 197,     /* AID */
	IEEE80211_ELEMID_QUIET_CHANNEL = 198,   /* Quiet Channel */
	IEEE80211_ELEMID_OP_MODE_NOTIFY = 199,  /* Operating Mode Notification */
	IEEE80211_ELEMID_VENDOR = 221,  /* vendor private */
};

struct ieee80211_channelswitch_ie {
	uint8_t ie;             /* IEEE80211_ELEMID_CHANSWITCHANN */
	uint8_t len;
	uint8_t switchmode;
	uint8_t newchannel;
	uint8_t tbttcount;
} __packed;

struct ieee80211_extendedchannelswitch_ie {
	uint8_t ie;             /* IEEE80211_ELEMID_EXTCHANSWITCHANN */
	uint8_t len;
	uint8_t switchmode;
	uint8_t newClass;
	uint8_t newchannel;
	uint8_t tbttcount;
} __packed;

/* WME stream classes */
#define WME_AC_BE                          0    /* best effort */
#define WME_AC_BK                          1    /* background */
#define WME_AC_VI                          2    /* video */
#define WME_AC_VO                          3    /* voice */
#define WME_AC_NUM                         4

/*
 * Reason codes
 *
 * Unlisted codes are reserved
 */
enum {
	IEEE80211_REASON_UNSPECIFIED = 1,
	IEEE80211_REASON_AUTH_EXPIRE = 2,
	IEEE80211_REASON_AUTH_LEAVE = 3,
	IEEE80211_REASON_ASSOC_EXPIRE = 4,
	IEEE80211_REASON_ASSOC_TOOMANY = 5,
	IEEE80211_REASON_NOT_AUTHED = 6,
	IEEE80211_REASON_NOT_ASSOCED = 7,
	IEEE80211_REASON_ASSOC_LEAVE = 8,
	IEEE80211_REASON_ASSOC_NOT_AUTHED = 9,

	IEEE80211_REASON_RSN_REQUIRED = 11,
	IEEE80211_REASON_RSN_INCONSISTENT = 12,
	IEEE80211_REASON_IE_INVALID = 13,
	IEEE80211_REASON_MIC_FAILURE = 14,

	IEEE80211_REASON_QOS = 32,
	IEEE80211_REASON_QOS_BANDWITDH = 33,
	IEEE80211_REASON_QOS_CH_CONDITIONS = 34,
	IEEE80211_REASON_QOS_TXOP = 35,
	IEEE80211_REASON_QOS_LEAVE = 36,
	IEEE80211_REASON_QOS_DECLINED = 37,
	IEEE80211_REASON_QOS_SETUP_REQUIRED = 38,
	IEEE80211_REASON_QOS_TIMEOUT = 39,
	IEEE80211_REASON_QOS_CIPHER = 45,

	IEEE80211_STATUS_SUCCESS = 0,
	IEEE80211_STATUS_UNSPECIFIED = 1,
	IEEE80211_STATUS_CAPINFO = 10,
	IEEE80211_STATUS_NOT_ASSOCED = 11,
	IEEE80211_STATUS_OTHER = 12,
	IEEE80211_STATUS_ALG = 13,
	IEEE80211_STATUS_SEQUENCE = 14,
	IEEE80211_STATUS_CHALLENGE = 15,
	IEEE80211_STATUS_TIMEOUT = 16,
	IEEE80211_STATUS_TOOMANY = 17,
	IEEE80211_STATUS_BASIC_RATE = 18,
	IEEE80211_STATUS_SP_REQUIRED = 19,
	IEEE80211_STATUS_PBCC_REQUIRED = 20,
	IEEE80211_STATUS_CA_REQUIRED = 21,
	IEEE80211_STATUS_TOO_MANY_STATIONS = 22,
	IEEE80211_STATUS_RATES = 23,
	IEEE80211_STATUS_SHORTSLOT_REQUIRED = 25,
	IEEE80211_STATUS_DSSSOFDM_REQUIRED = 26,
	IEEE80211_STATUS_NO_HT = 27,
	IEEE80211_STATUS_REJECT_TEMP = 30,
	IEEE80211_STATUS_MFP_VIOLATION = 31,
	IEEE80211_STATUS_REFUSED = 37,
	IEEE80211_STATUS_INVALID_PARAM = 38,

	IEEE80211_STATUS_DLS_NOT_ALLOWED = 48,
};

#define IEEE80211_WEP_IVLEN         3   /* 24bit */
#define IEEE80211_WEP_KIDLEN        1   /* 1 octet */
#define IEEE80211_WEP_CRCLEN        4   /* CRC-32 */

/*
 * 802.11i defines an extended IV for use with non-WEP ciphers.
 * When the EXTIV bit is set in the key id byte an additional
 * 4 bytes immediately follow the IV for TKIP.  For CCMP the
 * EXTIV bit is likewise set but the 8 bytes represent the
 * CCMP header rather than IV+extended-IV.
 */
#define IEEE80211_WEP_EXTIV      0x20
#define IEEE80211_WEP_EXTIVLEN      4   /* extended IV length */
#define IEEE80211_WEP_MICLEN        8   /* trailing MIC */

#define IEEE80211_CCMP_HEADERLEN    8
#define IEEE80211_CCMP_MICLEN       8
#define WLAN_IEEE80211_GCMP_HEADERLEN    8
#define WLAN_IEEE80211_GCMP_MICLEN      16

/*
 * 802.11w defines a MMIE chunk to be attached at the end of
 * any outgoing broadcast or multicast robust management frame.
 * MMIE field is total 18 bytes in size. Following the diagram of MMIE
 *
 *        <------------ 18 Bytes MMIE ----------------------->
 *        +--------+---------+---------+-----------+---------+
 *        |Element | Length  | Key id  |   IPN     |  MIC    |
 *        |  id    |         |         |           |         |
 *        +--------+---------+---------+-----------+---------+
 * bytes      1         1         2         6            8
 *
 */
#define IEEE80211_MMIE_LEN          18
#define IEEE80211_MMIE_IPNLEN       6
#define IEEE80211_MMIE_MICLEN       8

/*
 * 802.11ac Wide Bandwidth Channel Switch Element
 */

struct ieee80211_ie_wide_bw_switch {
	uint8_t elem_id;
	uint8_t elem_len;
	uint8_t new_ch_width;   /* New channel width */
	uint8_t new_ch_freq_seg1;       /* Channel Center frequency 1 */
	uint8_t new_ch_freq_seg2;       /* Channel Center frequency 2 */
} __packed;

#endif /* CDS_COMMON_IEEE80211_H_ */
