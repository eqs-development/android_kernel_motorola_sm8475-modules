/*
 * Copyright (c) 2011,2017 The Linux Foundation. All rights reserved.
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

#ifndef _TARGET_IF_SPECTRAL_H_
#define _TARGET_IF_SPECTRAL_H_

#include <wlan_objmgr_cmn.h>
#include <wlan_objmgr_psoc_obj.h>
#include <wlan_objmgr_pdev_obj.h>
#include <qdf_lock.h>
#include <wlan_spectral_public_structs.h>
#include <reg_services_public_struct.h>

extern int spectral_debug_level;

#ifdef WIN32
#pragma pack(push, target_if_spectral, 1)
#define __ATTRIB_PACK
#else
#ifndef __ATTRIB_PACK
#define __ATTRIB_PACK __attribute__ ((packed))
#endif
#endif

#define spectral_log(level, args...) \
QDF_PRINT_INFO(QDF_PRINT_IDX_SHARED, QDF_MODULE_ID_SPECTRAL, level, ## args)

#define spectral_logfl(level, format, args...) \
		spectral_log(level, FL(format), ## args)

#define spectral_fatal(format, args...) \
	spectral_logfl(QDF_TRACE_LEVEL_FATAL, format, ## args)
#define spectral_err(format, args...) \
	spectral_logfl(QDF_TRACE_LEVEL_ERROR, format, ## args)
#define spectral_warn(format, args...) \
	spectral_logfl(QDF_TRACE_LEVEL_WARN, format, ## args)
#define spectral_info(format, args...) \
	spectral_logfl(QDF_TRACE_LEVEL_INFO, format, ## args)
#define spectral_debug(format, args...) \
	spectral_logfl(QDF_TRACE_LEVEL_DEBUG, format, ## args)

#define STATUS_PASS       1
#define STATUS_FAIL       0
#define line()   \
	qdf_print("----------------------------------------------------\n")
#define SPECTRAL_TODO(str) \
	qdf_print(KERN_INFO "SPECTRAL : %s (%s : %d)\n", \
		  (str), __func__, __LINE__)
#define spectral_ops_not_registered(str) \
	qdf_print(KERN_INFO "SPECTRAL : %s not registered\n", (str))
#define not_yet_implemented() \
	qdf_print("SPECTRAL : %s : %d Not yet implemented\n", \
		  __func__, __LINE__)

#define SPECTRAL_HT20_NUM_BINS               56
#define SPECTRAL_HT20_FFT_LEN                56
#define SPECTRAL_HT20_DC_INDEX               (SPECTRAL_HT20_FFT_LEN / 2)
#define SPECTRAL_HT20_DATA_LEN               60
#define SPECTRAL_HT20_TOTAL_DATA_LEN         (SPECTRAL_HT20_DATA_LEN + 3)
#define SPECTRAL_HT40_TOTAL_NUM_BINS         128
#define SPECTRAL_HT40_DATA_LEN               135
#define SPECTRAL_HT40_TOTAL_DATA_LEN         (SPECTRAL_HT40_DATA_LEN + 3)
#define SPECTRAL_HT40_FFT_LEN                128
#define SPECTRAL_HT40_DC_INDEX               (SPECTRAL_HT40_FFT_LEN / 2)

/* Used for the SWAR to obtain approximate combined rssi
 * in secondary 80Mhz segment
 */
#define OFFSET_CH_WIDTH_20	65
#define OFFSET_CH_WIDTH_40	62
#define OFFSET_CH_WIDTH_80	56
#define OFFSET_CH_WIDTH_160	50

#ifdef BIG_ENDIAN_HOST
#define SPECTRAL_MSG_COPY_CHAR_ARRAY(destp, srcp, len)  do { \
	int j; \
	u_int32_t *src, *dest; \
	src = (u_int32_t *)(srcp); \
	dest = (u_int32_t *)(destp); \
	for (j = 0; j < roundup((len), sizeof(u_int32_t)) / 4; j++) { \
	*(dest + j) = qdf_le32_to_cpu(*(src + j)); \
	} \
	} while (0)
#else
#define SPECTRAL_MSG_COPY_CHAR_ARRAY(destp, srcp, len) \
	OS_MEMCPY((destp), (srcp), (len));
#endif

/* 5 categories x (lower + upper) bands */
#define MAX_INTERF                   10
#define ATH_HOST_MAX_ANTENNA         3
/* Mask for time stamp from descriptor */
#define SPECTRAL_TSMASK              0xFFFFFFFF
#define SPECTRAL_SIGNATURE           0xdeadbeef
#define MAX_SPECTRAL_PAYLOAD         1500
#ifndef NETLINK_ATHEROS
#define NETLINK_ATHEROS              (NETLINK_GENERIC + 1)
#endif

/* START of spectral GEN II HW specific details */
#define SPECTRAL_PHYERR_SIGNATURE_GEN2           0xbb
#define TLV_TAG_SPECTRAL_SUMMARY_REPORT_GEN2     0xF9
#define TLV_TAG_ADC_REPORT_GEN2                  0xFA
#define TLV_TAG_SEARCH_FFT_REPORT_GEN2           0xFB

/**
 * struct spectral_search_fft_info_gen2 - spectral search fft report for gen2
 * @relpwr_db:       Total bin power in db
 * @num_str_bins_ib: Number of strong bins
 * @base_pwr:        Base power
 * @total_gain_info: Total gain
 * @fft_chn_idx:     FFT chain on which report is originated
 * @avgpwr_db:       Average power in db
 * @peak_mag:        Peak power seen in the bins
 * @peak_inx:        Index of bin holding peak power
 */
typedef struct spectral_search_fft_info_gen2 {
	uint32_t relpwr_db;
	uint32_t num_str_bins_ib;
	uint32_t base_pwr;
	uint32_t total_gain_info;
	uint32_t fft_chn_idx;
	uint32_t avgpwr_db;
	uint32_t peak_mag;
	int16_t  peak_inx;
} SPECTRAL_SEARCH_FFT_INFO_GEN2;

/* XXX Check if we should be handling the endinness difference in some
 * other way opaque to the host
 */
#ifdef BIG_ENDIAN_HOST

/**
 * struct spectral_phyerr_tlv_gen2 - phyerr tlv info for big endian host
 * @signature: signature
 * @tag:       tag
 * @length:    length
 */
typedef  struct spectral_phyerr_tlv_gen2 {
	u_int8_t  signature;
	u_int8_t  tag;
	u_int16_t length;
} __ATTRIB_PACK SPECTRAL_PHYERR_TLV_GEN2;

#else

/**
 * struct spectral_phyerr_tlv_gen2 - phyerr tlv info for little endian host
 * @length:    length
 * @tag:       tag
 * @signature: signature
 */
typedef  struct spectral_phyerr_tlv_gen2 {
	u_int16_t length;
	u_int8_t  tag;
	u_int8_t  signature;
} __ATTRIB_PACK SPECTRAL_PHYERR_TLV_GEN2;

#endif /* BIG_ENDIAN_HOST */

/**
 * struct spectral_phyerr_hdr_gen2 - phyerr header for gen2 HW
 * @hdr_a: Header[0:31]
 * @hdr_b: Header[32:63]
 */
typedef struct spectral_phyerr_hdr_gen2 {
	u_int32_t hdr_a;
	u_int32_t hdr_b;
} SPECTRAL_PHYERR_HDR_GEN2;

/* Segment ID information for 80+80.
 *
 * If the HW micro-architecture specification extends this DWORD for other
 * purposes, then redefine+rename accordingly. For now, the specification
 * mentions only segment ID (though this doesn't require an entire DWORD)
 * without mention of any generic terminology for the DWORD, or any reservation.
 * We use nomenclature accordingly.
 */
typedef u_int32_t SPECTRAL_SEGID_INFO;

/**
 * struct spectral_phyerr_fft_gen2 - fft info in phyerr event
 * @buf: fft report
 */
typedef struct spectral_phyerr_fft_gen2 {
	u_int8_t buf[0];
} SPECTRAL_PHYERR_FFT_GEN2;
/* END of spectral GEN II HW specific details */

/* START of spectral GEN III HW specific details */

#define get_bitfield(value, size, pos) \
	(((value) >> (pos)) & ((1 << (size)) - 1))
#define unsigned_to_signed(value, width) \
	(((value) >= (1 << ((width) - 1))) ? \
		(value - (1 << (width))) : (value))

#define SPECTRAL_PHYERR_SIGNATURE_GEN3           (0xFA)
#define TLV_TAG_SPECTRAL_SUMMARY_REPORT_GEN3     (0x02)
#define TLV_TAG_SEARCH_FFT_REPORT_GEN3           (0x03)
#define SPECTRAL_PHYERR_TLVSIZE_GEN3             (4)

#define PHYERR_HDR_SIG_POS    \
	(offsetof(struct spectral_phyerr_fft_report_gen3, fft_hdr_sig))
#define PHYERR_HDR_TAG_POS    \
	(offsetof(struct spectral_phyerr_fft_report_gen3, fft_hdr_tag))
#define SPECTRAL_FFT_BINS_POS \
	(offsetof(struct spectral_phyerr_fft_report_gen3, buf))

/**
 * struct phyerr_info - spectral search fft report for gen3
 * @data:       handle to phyerror buffer
 * @datalen:    length of phyerror bufer
 * @p_rfqual:   rf quality matrices
 * @p_chaninfo: pointer to chaninfo
 * @tsf64:      64 bit TSF
 * @acs_stats:  acs stats
 */
struct phyerr_info {
	u_int8_t *data;
	u_int32_t datalen;
	struct target_if_spectral_rfqual_info *p_rfqual;
	struct target_if_spectral_chan_info *p_chaninfo;
	u_int64_t tsf64;
	struct target_if_spectral_acs_stats *acs_stats;
};

/**
 * struct spectral_search_fft_info_gen3 - spectral search fft report for gen3
 * @timestamp:           Timestamp at which fft report was generated
 * @fft_detector_id:     Which radio generated this report
 * @fft_num:             The FFT count number. Set to 0 for short FFT.
 * @fft_radar_check:     NA for spectral
 * @fft_peak_sidx:       Index of bin with maximum power
 * @fft_chn_idx:         Rx chain index
 * @fft_base_pwr_db:     Base power in dB
 * @fft_total_gain_db:   Total gain in dB
 * @fft_num_str_bins_ib: Number of strong bins in the report
 * @fft_peak_mag:        Peak magnitude
 * @fft_avgpwr_db:       Average power in dB
 * @fft_relpwr_db:       Relative power in dB
 */
struct spectral_search_fft_info_gen3 {
	uint32_t timestamp;
	uint32_t fft_detector_id;
	uint32_t fft_num;
	uint32_t fft_radar_check;
	int32_t  fft_peak_sidx;
	uint32_t fft_chn_idx;
	uint32_t fft_base_pwr_db;
	uint32_t fft_total_gain_db;
	uint32_t fft_num_str_bins_ib;
	int32_t  fft_peak_mag;
	uint32_t fft_avgpwr_db;
	uint32_t fft_relpwr_db;
};

/**
 * struct spectral_phyerr_sfftreport_gen3 - fft info in phyerr event
 * @fft_timestamp:  Timestamp at which fft report was generated
 * @fft_hdr_sig:    signature
 * @fft_hdr_tag:    tag
 * @fft_hdr_length: length
 * @hdr_a:          Header[0:31]
 * @hdr_b:          Header[32:63]
 * @hdr_c:          Header[64:95]
 * @resv:           Header[96:127]
 * @buf:            fft bins
 */
struct spectral_phyerr_fft_report_gen3 {
	u_int32_t fft_timestamp;
#ifdef BIG_ENDIAN_HOST
	u_int8_t  fft_hdr_sig;
	u_int8_t  fft_hdr_tag;
	u_int16_t fft_hdr_length;
#else
	u_int16_t fft_hdr_length;
	u_int8_t  fft_hdr_tag;
	u_int8_t  fft_hdr_sig;
#endif /* BIG_ENDIAN_HOST */
	u_int32_t hdr_a;
	u_int32_t hdr_b;
	u_int32_t hdr_c;
	u_int32_t resv;
	u_int8_t buf[0];
} __ATTRIB_PACK;

/* END of spectral GEN III HW specific details */

typedef signed char pwr_dbm;

/**
 * enum spectral_gen - spectral hw generation
 * @SPECTRAL_GEN1 : spectral hw gen 1
 * @SPECTRAL_GEN2 : spectral hw gen 2
 * @SPECTRAL_GEN3 : spectral hw gen 3
 */
enum spectral_gen {
	SPECTRAL_GEN1,
	SPECTRAL_GEN2,
	SPECTRAL_GEN3,
};

#if ATH_PERF_PWR_OFFLOAD
/**
 * enum ol_spectral_info_spec - Enumerations for specifying which spectral
 *                              information (among parameters and states)
 *                              is desired.
 * @OL_SPECTRAL_INFO_SPEC_ACTIVE:  Indicated whether spectral is active
 * @OL_SPECTRAL_INFO_SPEC_ENABLED: Indicated whether spectral is enabled
 * @OL_SPECTRAL_INFO_SPEC_PARAMS:  Config params
 */
enum ol_spectral_info_spec {
	OL_SPECTRAL_INFO_SPEC_ACTIVE,
	OL_SPECTRAL_INFO_SPEC_ENABLED,
	OL_SPECTRAL_INFO_SPEC_PARAMS,
};
#endif /* ATH_PERF_PWR_OFFLOAD */

/* forward declaration */
struct target_if_spectral;

/**
 * struct target_if_spectral_chan_info - Channel information
 * @center_freq1: center frequency 1 in MHz
 * @center_freq2: center frequency 2 in MHz -valid only for
 *		 11ACVHT 80PLUS80 mode
 * @chan_width:   channel width in MHz
 */
struct target_if_spectral_chan_info {
	u_int16_t    center_freq1;
	u_int16_t    center_freq2;
	u_int8_t     chan_width;
};

/**
 * struct target_if_spectral_acs_stats - EACS stats from spectral samples
 * @nfc_ctl_rssi: Control chan rssi
 * @nfc_ext_rssi: Extension chan rssi
 * @ctrl_nf:      Control chan Noise Floor
 * @ext_nf:       Extension chan Noise Floor
 */
struct target_if_spectral_acs_stats {
	int8_t nfc_ctl_rssi;
	int8_t nfc_ext_rssi;
	int8_t ctrl_nf;
	int8_t ext_nf;
};

/**
 * struct target_if_spectral_perchain_rssi_info - per chain rssi info
 * @rssi_pri20: Rssi of primary 20 Mhz
 * @rssi_sec20: Rssi of secondary 20 Mhz
 * @rssi_sec40: Rssi of secondary 40 Mhz
 * @rssi_sec80: Rssi of secondary 80 Mhz
 */
struct target_if_spectral_perchain_rssi_info {
	int8_t    rssi_pri20;
	int8_t    rssi_sec20;
	int8_t    rssi_sec40;
	int8_t    rssi_sec80;
};

/**
 * struct target_if_spectral_rfqual_info - RF measurement information
 * @rssi_comb:    RSSI Information
 * @pc_rssi_info: XXX : For now, we know we are getting information
 *                for only 4 chains at max. For future extensions
 *                use a define
 * @noise_floor:  Noise floor information
 */
struct target_if_spectral_rfqual_info {
	int8_t    rssi_comb;
	struct    target_if_spectral_perchain_rssi_info pc_rssi_info[4];
	int16_t   noise_floor[4];
};

#define GET_TIF_SPECTRAL_OPS(spectral) \
	((struct target_if_spectral_ops *)(&((spectral)->spectral_ops)))

/**
 * struct target_if_spectral_ops - spectral low level ops table
 * @get_tsf64:               Get 64 bit TSF value
 * @get_capability:          Get capability info
 * @set_rxfilter:            Set rx filter
 * @get_rxfilter:            Get rx filter
 * @is_spectral_active:      Check whether icm is active
 * @is_spectral_enabled:     Check whether spectral is enabled
 * @start_spectral_scan:     Start spectral scan
 * @stop_spectral_scan:      Stop spectral scan
 * @get_extension_channel:   Get extension channel
 * @get_ctl_noisefloor:      Get control noise floor
 * @get_ext_noisefloor:      Get extension noise floor
 * @configure_spectral:      Set spectral configurations
 * @get_spectral_config:     Get spectral configurations
 * @get_ent_spectral_mask:   Get spectral mask
 * @get_mac_address:         Get mac address
 * @get_current_channel:     Get current channel
 * @reset_hw:                Reset HW
 * @get_chain_noise_floor:   Get Channel noise floor
 * @spectral_process_phyerr: Process phyerr event
 */
struct target_if_spectral_ops {
	u_int64_t (*get_tsf64)(void *arg);
	u_int32_t (*get_capability)(void *arg, SPECTRAL_CAPABILITY_TYPE type);
	u_int32_t (*set_rxfilter)(void *arg, int rxfilter);
	u_int32_t (*get_rxfilter)(void *arg);
	u_int32_t (*is_spectral_active)(void *arg);
	u_int32_t (*is_spectral_enabled)(void *arg);
	u_int32_t (*start_spectral_scan)(void *arg);
	u_int32_t (*stop_spectral_scan)(void *arg);
	u_int32_t (*get_extension_channel)(void *arg);
	int8_t    (*get_ctl_noisefloor)(void *arg);
	int8_t    (*get_ext_noisefloor)(void *arg);
	u_int32_t (*configure_spectral)(
			void *arg,
			struct spectral_config *params);
	u_int32_t (*get_spectral_config)(
			void *arg,
			struct spectral_config *params);
	u_int32_t (*get_ent_spectral_mask)(void *arg);
	u_int32_t (*get_mac_address)(void *arg, char *addr);
	u_int32_t (*get_current_channel)(void *arg);
	u_int32_t (*reset_hw)(void *arg);
	u_int32_t (*get_chain_noise_floor)(void *arg, int16_t *nf_buf);
	int (*spectral_process_phyerr)(struct target_if_spectral *spectral,
				       u_int8_t *data, u_int32_t datalen,
			struct target_if_spectral_rfqual_info *p_rfqual,
			struct target_if_spectral_chan_info *p_chaninfo,
			u_int64_t tsf64,
			struct target_if_spectral_acs_stats *acs_stats);
};

/**
 * struct target_if_spectral_stats - spectral stats info
 * @num_spectral_detects: Total num. of spectral detects
 * @total_phy_errors:     Total number of phyerrors
 * @owl_phy_errors:       Indicated phyerrors in old gen1 chipsets
 * @pri_phy_errors:       Phyerrors in primary channel
 * @ext_phy_errors:       Phyerrors in secondary channel
 * @dc_phy_errors:        Phyerrors due to dc
 * @early_ext_phy_errors: Early secondary channel phyerrors
 * @bwinfo_errors:        Bandwidth info errors
 * @datalen_discards:     Invalid data length errors, seen in gen1 chipsets
 * @rssi_discards bw:     Indicates reports dropped due to RSSI threshold
 * @last_reset_tstamp:    Last reset time stamp
 */
struct target_if_spectral_stats {
	u_int32_t    num_spectral_detects;
	u_int32_t    total_phy_errors;
	u_int32_t    owl_phy_errors;
	u_int32_t    pri_phy_errors;
	u_int32_t    ext_phy_errors;
	u_int32_t    dc_phy_errors;
	u_int32_t    early_ext_phy_errors;
	u_int32_t    bwinfo_errors;
	u_int32_t    datalen_discards;
	u_int32_t    rssi_discards;
	u_int64_t    last_reset_tstamp;
};

/**
 * struct target_if_spectral_event - spectral event structure
 * @se_ts:        Original 15 bit recv timestamp
 * @se_full_ts:   64-bit full timestamp from interrupt time
 * @se_rssi:      Rssi of spectral event
 * @se_bwinfo:    Rssi of spectral event
 * @se_dur:       Duration of spectral pulse
 * @se_chanindex: Channel of event
 * @se_list:      List of spectral events
 */
struct target_if_spectral_event {
	u_int32_t                       se_ts;
	u_int64_t                       se_full_ts;
	u_int8_t                        se_rssi;
	u_int8_t                        se_bwinfo;
	u_int8_t                        se_dur;
	u_int8_t                        se_chanindex;

	STAILQ_ENTRY(spectral_event)    se_list;
};

/**
 * struct target_if_chain_noise_pwr_info - Noise power info for each channel
 * @rptcount:        Count of reports in pwr array
 * @un_cal_nf:       Uncalibrated noise floor
 * @factory_cal_nf:  Noise floor as calibrated at the factory for module
 * @median_pwr:      Median power (median of pwr array)
 * @pwr:             Power reports
 */
struct target_if_chain_noise_pwr_info {
	int        rptcount;
	pwr_dbm    un_cal_nf;
	pwr_dbm    factory_cal_nf;
	pwr_dbm    median_pwr;
	pwr_dbm    pwr[];
} __ATTRIB_PACK;

/**
 * struct target_if_spectral_chan_stats - Channel information
 * @cycle_count:         Cycle count
 * @channel_load:        Channel load
 * @per:                 Period
 * @noisefloor:          Noise floor
 * @comp_usablity:       Computed usability
 * @maxregpower:         Maximum allowed regulatary power
 * @comp_usablity_sec80: Computed usability of secondary 80 Mhz
 * @maxregpower_sec80:   Max regulatory power in secondary 80 Mhz
 */
struct target_if_spectral_chan_stats {
	int          cycle_count;
	int          channel_load;
	int          per;
	int          noisefloor;
	u_int16_t    comp_usablity;
	int8_t       maxregpower;
	u_int16_t    comp_usablity_sec80;
	int8_t       maxregpower_sec80;
};

#if ATH_PERF_PWR_OFFLOAD
/* Locking operations
 * We have a separate set of definitions for offload to accommodate
 * offload specific changes in the future.
 */
#define OL_SPECTRAL_LOCK_INIT(_lock)            qdf_spinlock_create((_lock))
#define OL_SPECTRAL_LOCK_DESTROY(_lock)         qdf_spinlock_destroy((_lock))
#define OL_SPECTRAL_LOCK(_lock)                 qdf_spin_lock((_lock))
#define OL_SPECTRAL_UNLOCK(_lock)               qdf_spin_unlock((_lock))

/**
 * struct target_if_spectral_cache - Cache used to minimize WMI operations
 *                             in offload architecture
 * @osc_spectral_enabled: Whether Spectral is enabled
 * @osc_spectral_active:  Whether spectral is active
 *                        XXX: Ideally, we should NOT cache this
 *                        since the hardware can self clear the bit,
 *                        the firmware can possibly stop spectral due to
 *                        intermittent off-channel activity, etc
 *                        A WMI read command should be introduced to handle
 *                        this This will be discussed.
 * @osc_params:           Spectral parameters
 * @osc_is_valid:         Whether the cache is valid
 */
struct target_if_spectral_cache {
	u_int8_t                  osc_spectral_enabled;
	u_int8_t                  osc_spectral_active;
	struct spectral_config    osc_params;
	u_int8_t                  osc_is_valid;
};

/**
 * struct target_if_spectral_param_state_info - Structure used to represent and
 *                                        manage spectral information
 *                                        (parameters and states)
 * @osps_lock:  Lock to synchronize accesses to information
 * @osps_cache: Cacheable' information
 */
struct target_if_spectral_param_state_info {
	qdf_spinlock_t               osps_lock;
	struct target_if_spectral_cache    osps_cache;
	/* XXX - Non-cacheable information goes here, in the future */
};
#endif /* ATH_PERF_PWR_OFFLOAD */

/**
 * struct target_if_spectral - main spectral structure
 * @pdev: Pointer to pdev
 * @spectral_ops: Target if internal Spectral low level operations table
 * @capability: Spectral capabilities structure
 * @ath_spectral_lock: Lock used for internal Spectral operations
 * @spectral_curchan_radindex: Current channel spectral index
 * @spectral_extchan_radindex: Extension channel spectral index
 * @spectraldomain: Current Spectral domain
 * @spectral_proc_phyerr:  Flags to process for PHY errors
 * @spectral_defaultparams: Default PHY params per Spectral stat
 * @ath_spectral_stats:  Spectral related stats
 * @events:   Events structure
 * @sc_spectral_ext_chan_ok:  Can spectral be detected on the extension channel?
 * @sc_spectral_combined_rssi_ok:  Can use combined spectral RSSI?
 * @sc_spectral_20_40_mode:  Is AP in 20-40 mode?
 * @sc_spectral_noise_pwr_cal:  Noise power cal required?
 * @sc_spectral_non_edma:  Is the spectral capable device Non-EDMA?
 * @upper_is_control: Upper segment is primary
 * @upper_is_extension: Upper segment is secondary
 * @lower_is_control: Lower segment is primary
 * @lower_is_extension: Lower segment is secondary
 * @sc_spectraltest_ieeechan:  IEEE channel number to return to after a spectral
 * mute test
 * @spectral_numbins: Number of bins
 * @spectral_fft_len: FFT length
 * @spectral_data_len: Total phyerror report length
 * @lb_edge_extrabins: Number of extra bins on left band edge
 * @rb_edge_extrabins: Number of extra bins on right band edge
 * @spectral_max_index_offset: Max FFT index offset (20 MHz mode)
 * @spectral_upper_max_index_offset: Upper max FFT index offset (20/40 MHz mode)
 * @spectral_lower_max_index_offset: Lower max FFT index offset (20/40 MHz mode)
 * @spectral_dc_index: At which index DC is present
 * @send_single_packet: Deprecated
 * @spectral_sent_msg: Indicates whether we send report to upper layers
 * @params: Spectral parameters
 * @last_capture_time: Indicates timestamp of previouse report
 * @num_spectral_data: Number of Spectral samples received in current session
 * @total_spectral_data: Total number of Spectral samples received
 * @max_rssi: Maximum RSSI
 * @detects_control_channel: NA
 * @detects_extension_channel: NA
 * @detects_below_dc: NA
 * @detects_above_dc: NA
 * @sc_scanning: Indicates active wifi scan
 * @sc_spectral_scan: Indicates active specral scan
 * @sc_spectral_full_scan: Deprecated
 * @scan_start_tstamp: Deprecated
 * @last_tstamp: Deprecated
 * @first_tstamp: Deprecated
 * @spectral_samp_count: Deprecated
 * @sc_spectral_samp_count: Deprecated
 * @noise_pwr_reports_reqd: Number of noise power reports required
 * @noise_pwr_reports_recv: Number of noise power reports received
 * @noise_pwr_reports_lock: Lock used for Noise power report processing
 * @noise_pwr_chain_ctl: Noise power report - control channel
 * @noise_pwr_chain_ext: Noise power report - extension channel
 * @chaninfo: Channel statistics
 * @tsf64: Latest TSF Value
 * @ol_info: Offload architecture Spectral parameter cache information
 * @ch_width: Indicates Channel Width 20/40/80/160 MHz with values 0, 1, 2, 3
 * respectively
 * @diag_stats: Diagnostic statistics
 * @is_160_format:  Indicates whether information provided by HW is in altered
 * format for 802.11ac 160/80+80 MHz support (QCA9984 onwards)
 * @is_lb_edge_extrabins_format:  Indicates whether information provided by
 * HW has 4 extra bins, at left band edge, for report mode 2
 * @is_rb_edge_extrabins_format:   Indicates whether information provided
 * by HW has 4 extra bins, at right band edge, for report mode 2
 * @is_sec80_rssi_war_required: Indicates whether the software workaround is
 * required to obtain approximate combined RSSI for secondary 80Mhz segment
 * @simctx: Spectral Simulation context
 * @spectral_gen: Spectral hardware generation
 * @hdr_sig_exp: Expected signature in PHYERR TLV header, for the given hardware
 * generation
 * @tag_sscan_summary_exp: Expected Spectral Scan Summary tag in PHYERR TLV
 * header, for the given hardware generation
 * @tag_sscan_fft_exp: Expected Spectral Scan FFT report tag in PHYERR TLV
 * header, for the given hardware generation
 * @tlvhdr_size: Expected PHYERR TLV header size, for the given hardware
 * generation
 */
struct target_if_spectral {
	struct wlan_objmgr_pdev *pdev_obj;
	struct target_if_spectral_ops                 spectral_ops;
	struct spectral_caps                    capability;
	qdf_spinlock_t                          ath_spectral_lock;
	int16_t                                 spectral_curchan_radindex;
	int16_t                                 spectral_extchan_radindex;
	u_int32_t                               spectraldomain;
	u_int32_t                               spectral_proc_phyerr;
	struct spectral_config                  spectral_defaultparams;
	struct target_if_spectral_stats               ath_spectral_stats;
	struct target_if_spectral_event *events;
	unsigned int                            sc_spectral_ext_chan_ok:1,
						sc_spectral_combined_rssi_ok:1,
						sc_spectral_20_40_mode:1,
						sc_spectral_noise_pwr_cal:1,
						sc_spectral_non_edma:1;
	int                                     upper_is_control;
	int                                     upper_is_extension;
	int                                     lower_is_control;
	int                                     lower_is_extension;
	u_int8_t                                sc_spectraltest_ieeechan;
	struct sock *spectral_sock;
	struct sk_buff *spectral_skb;
	struct nlmsghdr *spectral_nlh;
	u_int32_t                               spectral_pid;

	STAILQ_HEAD(, target_if_spectral_skb_event)    spectral_skbq;
	qdf_spinlock_t                          spectral_skbqlock;
	int                                     spectral_numbins;
	int                                     spectral_fft_len;
	int                                     spectral_data_len;

	/* For 11ac chipsets prior to AR900B version 2.0, a max of 512 bins are
	 * delivered.  However, there can be additional bins reported for
	 * AR900B version 2.0 and QCA9984 as described next:
	 *
	 * AR900B version 2.0: An additional tone is processed on the right
	 * hand side in order to facilitate detection of radar pulses out to
	 * the extreme band-edge of the channel frequency. Since the HW design
	 * processes four tones at a time, this requires one additional Dword
	 * to be added to the search FFT report.
	 *
	 * QCA9984: When spectral_scan_rpt_mode = 2, i.e 2-dword summary +
	 * 1x-oversampled bins (in-band) per FFT, then 8 more bins
	 * (4 more on left side and 4 more on right side)are added.
	 */

	int                                     lb_edge_extrabins;
	int                                     rb_edge_extrabins;
	int                                     spectral_max_index_offset;
	int                                     spectral_upper_max_index_offset;
	int                                     spectral_lower_max_index_offset;
	int                                     spectral_dc_index;
	int                                     send_single_packet;
	int                                     spectral_sent_msg;
	int                                     classify_scan;
	os_timer_t                              classify_timer;
	struct spectral_config                  params;
	struct spectral_classifier_params       classifier_params;
	int                                     last_capture_time;
	int                                     num_spectral_data;
	int                                     total_spectral_data;
	int                                     max_rssi;
	int                                     detects_control_channel;
	int                                     detects_extension_channel;
	int                                     detects_below_dc;
	int                                     detects_above_dc;
	int                                     sc_scanning;
	int                                     sc_spectral_scan;
	int                                     sc_spectral_full_scan;
	u_int64_t                               scan_start_tstamp;
	u_int32_t                               last_tstamp;
	u_int32_t                               first_tstamp;
	u_int32_t                               spectral_samp_count;
	u_int32_t                               sc_spectral_samp_count;
	int                                     noise_pwr_reports_reqd;
	int                                     noise_pwr_reports_recv;
	qdf_spinlock_t                          noise_pwr_reports_lock;
	struct target_if_chain_noise_pwr_info
		*noise_pwr_chain_ctl[ATH_HOST_MAX_ANTENNA];
	struct target_if_chain_noise_pwr_info
		*noise_pwr_chain_ext[ATH_HOST_MAX_ANTENNA];
	u_int64_t                               tsf64;
#if ATH_PERF_PWR_OFFLOAD
	struct target_if_spectral_param_state_info    ol_info;
#endif
	u_int32_t                               ch_width;
	struct spectral_diag_stats              diag_stats;
	bool                                    is_160_format;
	bool                                    is_lb_edge_extrabins_format;
	bool                                    is_rb_edge_extrabins_format;
	bool                                    is_sec80_rssi_war_required;
#if QCA_SUPPORT_SPECTRAL_SIMULATION
	void                                    *simctx;
#endif
	enum spectral_gen                       spectral_gen;
	u_int8_t                                hdr_sig_exp;
	u_int8_t                                tag_sscan_summary_exp;
	u_int8_t                                tag_sscan_fft_exp;
	u_int8_t                                tlvhdr_size;
};

/**
 * struct target_if_spectral_skb_event - Used to broadcast FFT report to
 *                                       applications
 * @sp_skb:            Pointer to skb
 * @sp_nlh:            Pointer to nl message header
 * @spectral_skb_list: Linked list to manipulate the reports
 */
struct target_if_spectral_skb_event {
	struct sk_buff *sp_skb;
	struct nlmsghdr *sp_nlh;

	STAILQ_ENTRY(target_if_spectral_skb_event)    spectral_skb_list;
};

/* TODO:COMMENTS */
struct target_if_samp_msg_params {
	int8_t      rssi;
	int8_t      rssi_sec80;
	int8_t      lower_rssi;
	int8_t      upper_rssi;
	int8_t      chain_ctl_rssi[ATH_HOST_MAX_ANTENNA];
	int8_t      chain_ext_rssi[ATH_HOST_MAX_ANTENNA];
	uint16_t    bwinfo;
	uint16_t    datalen;
	uint16_t    datalen_sec80;
	uint32_t    tstamp;
	uint32_t    last_tstamp;
	uint16_t    max_mag;
	uint16_t    max_mag_sec80;
	uint16_t    max_index;
	uint16_t    max_index_sec80;
	uint8_t     max_exp;
	int         peak;
	int         pwr_count;
	int         pwr_count_sec80;
	int8_t      nb_lower;
	int8_t      nb_upper;
	uint16_t    max_lower_index;
	uint16_t    max_upper_index;
	u_int8_t    *bin_pwr_data;
	u_int8_t    *bin_pwr_data_sec80;
	u_int16_t   freq;
	u_int16_t   vhtop_ch_freq_seg1;
	u_int16_t   vhtop_ch_freq_seg2;
	u_int16_t   freq_loading;
	int16_t     noise_floor;
	int16_t     noise_floor_sec80;
	struct INTERF_SRC_RSP interf_list;
	SPECTRAL_CLASSIFIER_PARAMS classifier_params;
	struct ath_softc *sc;
};

/* NETLINK related declarations */
#ifdef SPECTRAL_USE_NETLINK_SOCKETS
#if (KERNEL_VERSION(2, 6, 31) > LINUX_VERSION_CODE)
	void spectral_nl_data_ready(struct sock *sk, int len);
#else
	void spectral_nl_data_ready(struct sk_buff *skb);
#endif /* VERSION CHECK */
#endif /* SPECTRAL_USE_NETLINK_SOCKETS defined */

void target_if_sptrl_register_tx_ops(struct wlan_lmac_if_tx_ops *tx_ops);
extern struct net init_net;
int target_if_spectral_init_netlink(struct target_if_spectral *spectral);
int target_if_spectral_destroy_netlink(struct target_if_spectral *spectral);
void target_if_spectral_unicast_msg(struct target_if_spectral *spectral);
void target_if_spectral_bcast_msg(struct target_if_spectral *spectral);
void target_if_spectral_prep_skb(struct target_if_spectral *spectral);
void target_if_spectral_skb_dequeue(unsigned long data);
void target_if_spectral_create_samp_msg(
	struct target_if_spectral *spectral,
	struct target_if_samp_msg_params *params);
int spectral_process_phyerr_gen3(
	struct target_if_spectral *spectral,
	u_int8_t *data,
	u_int32_t datalen, struct target_if_spectral_rfqual_info *p_rfqual,
	struct target_if_spectral_chan_info *p_chaninfo,
	u_int64_t tsf64,
	struct target_if_spectral_acs_stats *acs_stats);
int spectral_process_phyerr_gen2(
	struct target_if_spectral *spectral,
	u_int8_t *data,
	u_int32_t datalen, struct target_if_spectral_rfqual_info *p_rfqual,
	struct target_if_spectral_chan_info *p_chaninfo,
	u_int64_t tsf64,
	struct target_if_spectral_acs_stats *acs_stats);
void target_if_spectral_send_intf_found_msg(
	struct wlan_objmgr_pdev *pdev,
	u_int16_t cw_int, u_int32_t dcs_enabled);
void target_if_stop_spectral_scan(struct wlan_objmgr_pdev *pdev);
struct wlan_objmgr_vdev *target_if_spectral_get_vdev(
	struct target_if_spectral *spectral);

int spectral_dump_header_gen2(SPECTRAL_PHYERR_HDR_GEN2 *phdr);
int8_t get_combined_rssi_sec80_segment_gen2(
	struct target_if_spectral *spectral,
	SPECTRAL_SEARCH_FFT_INFO_GEN2 *p_sfft_sec80);
int spectral_dump_tlv_gen2(SPECTRAL_PHYERR_TLV_GEN2 *ptlv, bool is_160_format);
int spectral_dump_phyerr_data_gen2(
	u_int8_t *data,
	u_int32_t datalen,
	bool is_160_format);
int spectral_dump_fft_report_gen3(
	struct spectral_phyerr_fft_report_gen3 *p_fft_report,
	struct spectral_search_fft_info_gen3 *p_sfft);

void target_if_dbg_print_SAMP_msg(SPECTRAL_SAMP_MSG *pmsg);

/* START of spectral GEN III HW specific function declarations */
/* [FIXME] fix the declaration */
int process_search_fft_report_gen3(
	struct spectral_phyerr_fft_report_gen3 *p_fft_report,
	 struct spectral_search_fft_info_gen3 *p_fft_info);
/* END of spectral GEN III HW specific function declarations */

/**
 * target_if_send_phydata() - Send Spectral PHY data over netlink
 * @pdev: Pointer to pdev
 * @sock: Netlink socket to use
 * @nbuf: Network buffer containing PHY data to send
 *
 * Return: 0 on success, negative value on failure
 */
static inline uint32_t target_if_send_phydata(
	struct wlan_objmgr_pdev *pdev,
	struct sock *sock, qdf_nbuf_t nbuf)
{
	struct wlan_objmgr_psoc *psoc = NULL;

	psoc = wlan_pdev_get_psoc(pdev);
	return psoc->soc_cb.rx_ops.sptrl_rx_ops.sptrlro_send_phydata(pdev,
				sock, nbuf);
}

/**
 * get_target_if_spectral_handle_from_pdev() - Get handle to target_if internal
 * Spectral data
 * @pdev: Pointer to pdev
 *
 * Return: Handle to target_if internal Spectral data on success, NULL on
 * failure
 */
static inline
struct target_if_spectral *get_target_if_spectral_handle_from_pdev(
	struct wlan_objmgr_pdev *pdev)
{
	struct target_if_spectral *spectral = NULL;
	struct wlan_objmgr_psoc *psoc = NULL;

	psoc = wlan_pdev_get_psoc(pdev);

	spectral = (struct target_if_spectral *)
		psoc->soc_cb.rx_ops.sptrl_rx_ops.sptrlro_get_target_handle(
		pdev);
	return spectral;
}

/**
 * target_if_spectral_set_rxchainmask() - Set Spectral Rx chainmask
 * @pdev: Pointer to pdev
 * @spectral_rx_chainmask: Spectral Rx chainmask
 *
 * Return: None
 */
static inline
void target_if_spectral_set_rxchainmask(struct wlan_objmgr_pdev *pdev,
					u_int8_t spectral_rx_chainmask)
{
	struct target_if_spectral *spectral = NULL;

	spectral = get_target_if_spectral_handle_from_pdev(pdev);
	spectral->params.ss_chn_mask = spectral_rx_chainmask;
}

/**
 * target_if_spectral_process_phyerr() - Process Spectral PHY error
 * @pdev: Pointer to pdev
 * @data: PHY error data received from FW
 * @datalen: Length of data
 * @p_rfqual: Pointer to RF Quality information
 * @p_chaninfo: Pointer to channel information
 * @tsf: TSF time instance at which the Spectral sample was received
 * @acs_stats: ACS stats
 *
 * Process Spectral PHY error by extracting necessary information from the data
 * sent by FW, and send the extracted information to application layer.
 *
 * Return: None
 */
static inline
void target_if_spectral_process_phyerr(
	struct wlan_objmgr_pdev *pdev,
	u_int8_t *data, u_int32_t datalen,
	struct target_if_spectral_rfqual_info *p_rfqual,
	struct target_if_spectral_chan_info *p_chaninfo,
	u_int64_t tsf64,
	struct target_if_spectral_acs_stats *acs_stats)
{
	struct target_if_spectral *spectral = NULL;
	struct target_if_spectral_ops *p_sops = NULL;

	spectral = get_target_if_spectral_handle_from_pdev(pdev);
	p_sops = GET_TIF_SPECTRAL_OPS(spectral);
	p_sops->spectral_process_phyerr(spectral, data, datalen,
					p_rfqual, p_chaninfo,
					tsf64, acs_stats);
}

#ifdef WIN32
#pragma pack(pop, target_if_spectral)
#endif
#ifdef __ATTRIB_PACK
#undef __ATTRIB_PACK
#endif

#endif /* _TARGET_IF_SPECTRAL_H_ */
