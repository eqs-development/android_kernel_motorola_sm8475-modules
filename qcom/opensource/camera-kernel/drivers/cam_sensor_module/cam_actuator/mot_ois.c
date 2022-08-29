#include <linux/module.h>
#include <linux/firmware.h>
#include <linux/component.h>
#include <linux/kernel.h>
#include <uapi/linux/media.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/gpio.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <cam_subdev.h>
#include <cam_sensor_cmn_header.h>
#include <linux/regulator/consumer.h>
#include <linux/platform_device.h>
#include "cam_debug_util.h"
#include "cam_cci_dev.h"
#include "mot_actuator.h"
#include "mot_actuator_policy.h"
#include "cam_sensor_io.h"
#include "cam_req_mgr_dev.h"
#include "linux/pm_wakeup.h"

#define MAX_OIS_NUM 3
#define DEVICE_NAME_LEN 32
#define INVALID_OIS_INDEX (-1)

#define BU63169_CALI_DATA_SIZE (40)
#define BU63169_AF_DRIFT_SIZE (62)
#define BU63169_AF_DRIFT_SEGMENTS 9
#define BU63169_AF_DAC_STEP 512

typedef enum {
	MOT_OIS_FIRST,
	MOT_OIS_BU63169 = MOT_OIS_FIRST,
	MOT_OIS_DW9781,
	MOT_OIS_DW9784,
	MOT_ACTUATOR_NUM,
} mot_ois_type;

typedef enum {
	MOT_DEVICE_EQS,
	MOT_DEVICE_NUM,
} mot_dev_type;

typedef enum {
	REGULATOR_IOVDD,
	REGULATOR_VCMVDD,
	REGULATOR_OISVDD,
	REGULATOR_NUM,
} mot_ois_regulator_type;

typedef struct
{
    int32_t afDac;
    int32_t oisHallX;
    int32_t oisHallY;
} mot_ois_af_drift_segment;

typedef struct
{
    int32_t segmentCount;
    mot_ois_af_drift_segment AfDriftSegments[BU63169_AF_DRIFT_SEGMENTS];
} mot_ois_af_drift_cali_data;

typedef struct {
	struct device                  *dev;
	const char                     *dev_name;
	uint32_t                        use_shared_clk;
	uint32_t                        num_clk;
	const char                     *clk_name[CAM_SOC_MAX_CLK];
	struct clk                     *clk[CAM_SOC_MAX_CLK];
	int32_t                         clk_rate[CAM_MAX_VOTE][CAM_SOC_MAX_CLK];
	uint32_t                        clk_id[CAM_SOC_MAX_CLK];
	uint32_t                        shared_clk_mask;
	int32_t                         prev_clk_level;
	int32_t                         src_clk_idx;
	unsigned long                   applied_src_clk_rate;
	bool                            clk_level_valid[CAM_MAX_VOTE];
	uint32_t                        lowest_clk_level;
	int32_t                         scl_clk_count;
	int32_t                         scl_clk_idx[CAM_SOC_MAX_CLK];
	const char                     *optional_clk_name[CAM_SOC_MAX_OPT_CLK];
	struct clk                     *optional_clk[CAM_SOC_MAX_OPT_CLK];
	int32_t                         optional_clk_rate[CAM_SOC_MAX_OPT_CLK];
	uint32_t                        optional_clk_id[CAM_SOC_MAX_OPT_CLK];
	uint32_t                        optional_shared_clk_mask;
	bool                            clk_control_enable;
} mot_ois_clk_info;

typedef int32_t (*mot_ois_ctrl_hdl)(struct device *device, uint32_t index);
typedef struct {
	mot_ois_type ois_type;
	uint16_t cci_addr; //7bit
	uint16_t cci_dev;
	uint16_t cci_master;
	char *regulator_list[REGULATOR_NUM];
	uint32_t regulator_min_volt_uv[REGULATOR_NUM];
	uint32_t regulator_max_volt_uv[REGULATOR_NUM];
	const char *ois_name;
	//For OIS calibration stored in EEPROM
	uint16_t eeprom_cci_addr; //7bit
	uint32_t cali_offset;
	uint32_t cali_size;
	uint32_t af_drift_offset;
	uint32_t af_drift_size;
	uint16_t af_safe_dac;
	bool is_af_drift_supported;
	mot_ois_ctrl_hdl start_func;
	mot_ois_ctrl_hdl stop_func;
} mot_ois_hw_info;

typedef struct {
	mot_dev_type dev_type;
	uint16_t ois_num;
	const char *dev_name;
	mot_ois_hw_info ois_info[MAX_OIS_NUM];
} mot_dev_ois_info;

typedef struct {
	bool is_af_drift_valid;
	bool is_cali_data_valid;
	uint16_t x_offset;
	uint16_t y_offset;
	mot_ois_af_drift_cali_data af_drift_data;
	uint8_t cali_data[BU63169_CALI_DATA_SIZE];
} mot_bu63169_prv_data;

typedef union {
	mot_bu63169_prv_data bu63169_data;
} mot_ois_priv_data;

typedef struct {
	struct cam_sensor_cci_client client;
	struct camera_io_master io_master;
	struct cam_sensor_cci_client eeprom_client;
	struct camera_io_master eeprom_io_master;
	struct regulator * regulators[REGULATOR_NUM];
	mot_ois_priv_data ois_prv_data;
	mot_ois_clk_info clk_info;
	const char *ois_name;
	mot_ois_type ois_type;
} mot_ois_runtime_type;

static int32_t mot_ois_bu63169_start(struct device *device, uint32_t index);
static int32_t mot_ois_bu63169_stop(struct device *device, uint32_t index);

static int32_t mot_device_index = 0;
static const mot_dev_ois_info mot_ois_dev_list[] = {
	//EQS
	{
		.dev_type = MOT_DEVICE_EQS,
		.ois_num = 1,
		.dev_name = "eqs",
		.ois_info = {
			{
				.ois_name = "mot_bu63169",
				.ois_type = MOT_OIS_BU63169,
				.cci_addr = 0x0E,
				.cci_dev = 0x0,
				.cci_master = 0x0,
				.regulator_list = { "ldo5", "ldo7", "pm8350c_l3"},
				.regulator_min_volt_uv = {1800000, 3100000,  3100000},
				.regulator_max_volt_uv = {1800000, 3104000,  3104000},
				.eeprom_cci_addr = 0x50,
				.cali_offset = 12160,
				.cali_size = BU63169_CALI_DATA_SIZE,
				.af_drift_offset = 12200,
				.af_drift_size = BU63169_AF_DRIFT_SIZE,
				.af_safe_dac = 0x08F0,//Please conver to 12Bit DAC
				.is_af_drift_supported = true,
				.start_func = mot_ois_bu63169_start,
				.stop_func = mot_ois_bu63169_stop,
			},
		},
	},
};

/*----------------------For EQS BU63169------------------------------*/
static struct cam_sensor_i2c_reg_array mot_eqs_ois_fw_init_serial[]= {
	{0x8262, 0xFF02, 1},
	{0x8263, 0x9F05, 1},
	{0x8264, 0x6040, 0},
	{0x8260, 0x1130, 0},
	{0x8265, 0x8000, 0},
	{0x8261, 0x0280, 0},
	{0x8261, 0x0380, 0},
	{0x8261, 0x0988, 0},
};

static struct cam_sensor_i2c_reg_setting mot_eqs_ois_fw_init_setting = {
	.reg_setting = mot_eqs_ois_fw_init_serial,
	.size = ARRAY_SIZE(mot_eqs_ois_fw_init_serial),
	.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
};

static struct cam_sensor_i2c_reg_array mot_ois_init_serial1[] = {
	{0x8205, 0x0c00, 0},
	{0x8205, 0x0d00, 0},
};

static struct cam_sensor_i2c_reg_array mot_ois_init_serial2[] = {
	{0x8c, 0x01, 0},
};

static struct cam_sensor_i2c_reg_array mot_ois_af_drift_serial[] = {
	{0x841E, 0x0000, 0},
	{0x849E, 0x0000, 0},
};

static struct cam_sensor_i2c_reg_setting mot_eqs_ois_init_setting[] = {
	{
		.reg_setting = mot_ois_init_serial1,
		.size = ARRAY_SIZE(mot_ois_init_serial1),
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	},
	{
		.reg_setting = mot_ois_init_serial2,
		.size = ARRAY_SIZE(mot_ois_init_serial2),
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
	},
};

static struct cam_sensor_i2c_reg_array mot_ois_lock_center_serial[] = {
	{0x847f, 0x0c0c, 0},
};

static struct cam_sensor_i2c_reg_setting mot_eqs_ois_lock_center_setting[] = {
	{
		.reg_setting = mot_ois_lock_center_serial,
		.size = ARRAY_SIZE(mot_ois_lock_center_serial),
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	}
};

static struct cam_sensor_i2c_reg_setting mot_eqs_ois_af_drift_setting[] = {
	{
		.reg_setting = mot_ois_af_drift_serial,
		.size = ARRAY_SIZE(mot_ois_af_drift_serial),
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	}
};

static mot_ois_runtime_type mot_ois_runtime[MAX_OIS_NUM];
static bool ois_runtime_inited = 0;

static int mot_ois_get_dt_clk_info(mot_ois_clk_info *soc_info);

static int mot_ois_runtime_init(struct device *dev)
{
	int ois_idx;
	int regIdx;

	if (mot_device_index >= MOT_DEVICE_NUM) {
		CAM_DBG(CAM_OIS, "Device index overflow (idx: %d, max: %d", mot_device_index, MOT_DEVICE_NUM);
		return -1;
	}

	for (ois_idx=0; ois_idx < mot_ois_dev_list[mot_device_index].ois_num; ois_idx++) {
		//Copy OIS type/name
		mot_ois_runtime[ois_idx].ois_type = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].ois_type;
		mot_ois_runtime[ois_idx].ois_name = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].ois_name;

		//Init CCI for OIS
		mot_ois_runtime[ois_idx].io_master.master_type = CCI_MASTER;
		mot_ois_runtime[ois_idx].io_master.cci_client = &mot_ois_runtime[ois_idx].client;
		mot_ois_runtime[ois_idx].client.sid = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].cci_addr;
		mot_ois_runtime[ois_idx].client.cci_device = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].cci_dev;
		mot_ois_runtime[ois_idx].client.cci_i2c_master = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].cci_master;
		mot_ois_runtime[ois_idx].client.i2c_freq_mode = I2C_FAST_PLUS_MODE;
		CAM_DBG(CAM_OIS, "NO. %d OIS sid:%x, cci_device:%d, cci_i2c_master:%d",
			ois_idx, mot_ois_runtime[ois_idx].client.sid, mot_ois_runtime[ois_idx].client.cci_device,
			mot_ois_runtime[ois_idx].client.cci_i2c_master);

		//Init CCI for EEPROM
		mot_ois_runtime[ois_idx].eeprom_io_master.master_type = CCI_MASTER;
		mot_ois_runtime[ois_idx].eeprom_io_master.cci_client = &mot_ois_runtime[ois_idx].eeprom_client;
		mot_ois_runtime[ois_idx].eeprom_client.sid = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].eeprom_cci_addr;
		mot_ois_runtime[ois_idx].eeprom_client.cci_device = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].cci_dev;
		mot_ois_runtime[ois_idx].eeprom_client.cci_i2c_master = mot_ois_dev_list[mot_device_index].ois_info[ois_idx].cci_master;
		CAM_DBG(CAM_OIS, "NO. %d EEPROM sid:%x, cci_device:%d, cci_i2c_master:%d",
			ois_idx, mot_ois_runtime[ois_idx].eeprom_client.sid, mot_ois_runtime[ois_idx].eeprom_client.cci_device,
			mot_ois_runtime[ois_idx].eeprom_client.cci_i2c_master);

		//Init regulator
		for (regIdx=0; regIdx<REGULATOR_NUM; regIdx++) {
			CAM_DBG(CAM_OIS, "Name:%s", mot_ois_dev_list[mot_device_index].ois_info[ois_idx].regulator_list[regIdx]);
			mot_ois_runtime[ois_idx].regulators[regIdx] = regulator_get(dev, mot_ois_dev_list[mot_device_index].ois_info[ois_idx].regulator_list[regIdx]);
			if (mot_ois_runtime[ois_idx].regulators[regIdx]  != NULL) {
				CAM_DBG(CAM_OIS, "REGULATOR GET SUCCESS:%p", mot_ois_runtime[ois_idx].regulators[regIdx]);
				regulator_set_voltage(mot_ois_runtime[ois_idx].regulators[regIdx],
					mot_ois_dev_list[mot_device_index].ois_info[ois_idx].regulator_min_volt_uv[regIdx],
					mot_ois_dev_list[mot_device_index].ois_info[ois_idx].regulator_max_volt_uv[regIdx]);
			} else {
				CAM_ERR(CAM_OIS, "OIS: %d, REGULATOR %d get failed!!!", ois_idx, regIdx);
				return -1;
			}
		}

		//Init mclk
		mot_ois_runtime[ois_idx].clk_info.dev_name = mot_ois_dev_list[mot_device_index].dev_name;
		mot_ois_runtime[ois_idx].clk_info.dev = dev;
		if (mot_ois_get_dt_clk_info(&mot_ois_runtime[ois_idx].clk_info) < 0) {
			CAM_ERR(CAM_OIS, "get clk info failed. (idx: %d, max: %d", mot_device_index, MOT_DEVICE_NUM);
			return -1;
		} else {
			int i;
			mot_ois_clk_info *pSocInfo = &mot_ois_runtime[ois_idx].clk_info;
			/* Initialize default parameters */
			for (i = 0; i < pSocInfo->num_clk; i++) {
				pSocInfo->clk[i] = devm_clk_get(pSocInfo->dev,
							pSocInfo->clk_name[i]);
				CAM_DBG(CAM_OIS, "clk%d: %s, ptr:%p", i, pSocInfo->clk_name[i], pSocInfo->clk[i] );
				if (!pSocInfo->clk[i]) {
					CAM_ERR(CAM_OIS, "get failed for %s",
						 pSocInfo->clk_name[i]);
					return -1;
				}
			}
		}
	}

	ois_runtime_inited = 1;
	return 0;
}

void mot_ois_runtime_uinit(void)
{
	int ois_idx;
	int regIdx;

	if (!ois_runtime_inited) {
		CAM_ERR(CAM_OIS, "OIS has already been uninitialsed");
		return;
	}

	if (mot_device_index >= MOT_DEVICE_NUM) {
		CAM_ERR(CAM_OIS, "Device index overflow (idx: %d, max: %d", mot_device_index, MOT_DEVICE_NUM);
		return;
	}

	for (ois_idx=0; ois_idx < mot_ois_dev_list[mot_device_index].ois_num; ois_idx++) {
		//Uninit regulator
		for (regIdx=0; regIdx<REGULATOR_NUM; regIdx++) {
			CAM_DBG(CAM_OIS, "Name:%s", mot_ois_dev_list[mot_device_index].ois_info[ois_idx].regulator_list[regIdx]);
			regulator_put(mot_ois_runtime[ois_idx].regulators[regIdx]);
		}

		//Uninit mclk
		{
			int i;
			mot_ois_clk_info *pSocInfo = &mot_ois_runtime[ois_idx].clk_info;
			for (i = 0; i < pSocInfo->num_clk; i++) {
				CAM_DBG(CAM_OIS, "clk%d: %s, ptr:%p", i, pSocInfo->clk_name[i], pSocInfo->clk[i] );
				devm_clk_put(pSocInfo->dev, pSocInfo->clk[i]);
			}
		}
	}

	ois_runtime_inited = 0;
	return;
}

static int32_t mot_get_ois_runtime_index(mot_ois_type ois_type, const char *ois_name)
{
	int i;
	for (i=0; i<mot_ois_dev_list[mot_device_index].ois_num; i++) {
		if (mot_ois_runtime[i].ois_type == ois_type && !strcmp(ois_name, mot_ois_runtime[i].ois_name)) {
			return i;
		}
	}
	CAM_DBG(CAM_OIS, "OIS inst not found!");
	return 0;
}

static int32_t mot_ois_power_on(struct device *dev, uint32_t index)
{
	int i;
	int ret = 0;

	for (i = 0; i < REGULATOR_NUM; i++) {
		if (mot_ois_runtime[index].regulators[i] != NULL) {
			ret = regulator_enable(mot_ois_runtime[index].regulators[i]);
			if(ret){
				CAM_ERR(CAM_OIS, "power on regulators[%d] failed, ret:%d", i,ret);
			}
		}
	}

	for (i=0; i<mot_ois_runtime[index].clk_info.num_clk; i++) {
		if (mot_ois_runtime[index].clk_info.clk[i]) {
			ret = clk_prepare_enable(mot_ois_runtime[index].clk_info.clk[i]);
			if (ret) {
				CAM_ERR(CAM_UTIL, "enable failed for %s: rc(%d)", mot_ois_runtime[index].clk_info.clk_name[i], ret);
				return ret;
			}
		} else {
			CAM_ERR(CAM_UTIL, "clk is NULL (%d)", i);
		}
	}

	return ret;
}

static int32_t mot_ois_power_off(struct device *dev, uint32_t index)
{
	int i;
	int ret = 0;

	for (i = 0; i < REGULATOR_NUM; i++) {
		if (mot_ois_runtime[index].regulators[i] != NULL) {
			ret = regulator_disable(mot_ois_runtime[index].regulators[i]);
			CAM_DBG(CAM_OIS, "power off ret %d", ret);
		}
	}

	CAM_DBG(CAM_OIS, "Disabling cam clocks.");
	for (i=0; i<mot_ois_runtime[index].clk_info.num_clk; i++) {
		clk_disable_unprepare(mot_ois_runtime[index].clk_info.clk[i]);
	}
	return ret;
}

int32_t mot_ois_init_cci(uint32_t index)
{
	int32_t ret = 0,rc = 0;

	CAM_DBG(CAM_OIS, "init cci.");
	ret = camera_io_init(&mot_ois_runtime[index].io_master);
	if (ret != 0) {
		rc = camera_io_release(&mot_ois_runtime[index].io_master);
		CAM_ERR(CAM_OIS, "init cci failed!!! ret=%d,release rc=%d. try again!", ret, rc);
		/*delay 100ms and try again*/
		usleep_range(100000, 100100);
		ret = camera_io_init(&mot_ois_runtime[index].io_master);
		if (ret != 0) {
			rc = camera_io_release(&mot_ois_runtime[index].io_master);
			CAM_ERR(CAM_OIS, "try again init cci failed!!! ret=%d, release rc=%d", ret, rc);
		}
	}

	ret = camera_io_init(&mot_ois_runtime[index].eeprom_io_master);
	if (ret != 0) {
		rc = camera_io_release(&mot_ois_runtime[index].eeprom_io_master);
		CAM_ERR(CAM_OIS, "init cci failed!!! ret=%d,release rc=%d. try again!", ret, rc);
		/*delay 100ms and try again*/
		usleep_range(100000, 100100);
		ret = camera_io_init(&mot_ois_runtime[index].eeprom_io_master);
		if (ret != 0) {
			rc = camera_io_release(&mot_ois_runtime[index].eeprom_io_master);
			CAM_ERR(CAM_OIS, "try again init cci failed!!! ret=%d, release rc=%d", ret, rc);
		}
	}

	return ret;
}

int32_t mot_ois_release_cci(uint32_t index)
{
	int32_t ret = 0;

	CAM_DBG(CAM_OIS, "release cci.");
	ret = camera_io_release(&mot_ois_runtime[index].io_master);
	if (ret != 0) {
		CAM_ERR(CAM_OIS, "release cci failed, ret=%d!!!", ret);
	}

	ret = camera_io_release(&mot_ois_runtime[index].eeprom_io_master);
	if (ret != 0) {
		CAM_ERR(CAM_OIS, "release cci failed, ret=%d!!!", ret);
	}
	return ret;
}

static int32_t mot_ois_apply_settings(struct camera_io_master *io_master_info,
	struct cam_sensor_i2c_reg_setting *write_setting, uint32_t item_num)
{
	int32_t ret = 0;
	int32_t i;

	for (i=0; i<item_num; i++) {
		ret = camera_io_dev_write(io_master_info, &write_setting[i]);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "apply settting failed, ret=%d!!!", ret);
			break;
		}
	}
	return ret;
}

static int32_t mot_ois_read_data(struct camera_io_master * io_master_info, uint32_t addr, uint8_t * data,
                                enum camera_sensor_i2c_type addr_type, enum camera_sensor_i2c_type data_type, int32_t num_bytes)
{
	return camera_io_dev_read_seq(io_master_info, addr, data, addr_type, data_type, num_bytes);
}

static int32_t mot_ois_bu63169_apply_calibration(struct camera_io_master * io_master_info, uint8_t *pCaliData, uint32_t size)
{
	#define CALI_REG_NUM (19)
	struct cam_sensor_i2c_reg_setting cali_setting;
	struct cam_sensor_i2c_reg_array cali_reg_array[CALI_REG_NUM];
	int i;
	const uint16_t cali_reg_addr[] = {
		0x8230,
		0x8231,
		0x8232,
		0x841E,
		0x849E,
		0x8239,
		0x823B,
		0x8406,
		0x8486,
		0x8446,
		0x84C6,
		0x840F,
		0x848F,
		0x846A,
		0x846B,
		0x846A,
		0x846B,
		0x8470,
		0x8472,
	};

	if (size != BU63169_CALI_DATA_SIZE) {
		CAM_ERR(CAM_OIS, "OIS calibration size error(%d != %d)", size, BU63169_CALI_DATA_SIZE);
		return -1;
	}

	/*Let factory test sort the CRC mismatch modules, then we can skip CRC here.
	  (Better check CRC here, since bad calibration data will make OIS disfunction) */
	for (i=0; i<CALI_REG_NUM; i++) {
		cali_reg_array[i].reg_addr = cali_reg_addr[i];
		cali_reg_array[i].reg_data = (pCaliData[i*2] |(pCaliData[i*2+1] << 8) );
		cali_reg_array[i].delay = 1;
	}
	cali_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	cali_setting.data_type =CAMERA_SENSOR_I2C_TYPE_WORD ;
	cali_setting.size = CALI_REG_NUM;
	cali_setting.reg_setting = cali_reg_array;

	return mot_ois_apply_settings(io_master_info, &cali_setting, 1);
}

#define FW_REG_ADDR (0x80)
#define COEF_REG_ADDR (0x88)
static int32_t mot_ois_bu63169_fw_download(struct camera_io_master * io_master_info, const struct firmware *prog, uint16_t op_reg)
{
	#define CHUNCK_SIZE (256)
	struct cam_sensor_i2c_reg_setting cali_setting = {0};
	struct cam_sensor_i2c_reg_array cali_reg_array[CHUNCK_SIZE] = {0};
	struct cam_sensor_i2c_reg_array *pCaliRegArray;
	uint32_t wr_bytes = 0;
	uint32_t remain_bytes = 0;
	uint8_t *pFwData = (uint8_t *)prog->data;
	int32_t ret = 0;

	if (prog->size <= 0 || prog->data == NULL) {
		CAM_ERR(CAM_OIS, "FW is not valid( buf:%p, size:%d)", prog->data, prog->size);
		return -1;
	}

	remain_bytes = prog->size;

	CAM_DBG(CAM_OIS, "FW download start.");
	while (remain_bytes) {
		pCaliRegArray = cali_reg_array;
		wr_bytes = remain_bytes>=CHUNCK_SIZE ? CHUNCK_SIZE:remain_bytes;
		remain_bytes -= wr_bytes;
		cali_setting.addr_type = 1;
		cali_setting.data_type = 1;
		cali_setting.reg_setting = cali_reg_array;
		cali_setting.size = wr_bytes;
		while (wr_bytes--) {
			pCaliRegArray->reg_addr = op_reg;
			pCaliRegArray->reg_data = *pFwData++;
			pCaliRegArray->delay = 0;
			pCaliRegArray->data_mask = 0;
			pCaliRegArray++;
		}
		ret = camera_io_dev_write_continuous(io_master_info, &cali_setting, CAM_SENSOR_I2C_WRITE_BURST);
		if (ret < 0) {
			CAM_ERR(CAM_OIS, "FW Download error. ret(%d)", ret);
			return -1;
		}
	}
	CAM_DBG(CAM_OIS, "FW download done.");
	return 0;
}

static int16_t _read_16bit_msb_first(uint8_t *pbuf)
{
	return (((uint16_t)pbuf[0] << 8) | ((uint16_t)pbuf[1]));
}

static int32_t mot_ois_bu63169_write_af_drift(uint32_t index)
{
	if (mot_ois_runtime[index].ois_prv_data.bu63169_data.is_af_drift_valid != true) {
		CAM_ERR(CAM_OIS, "AF drift is invalid.");
		return -1;
	}
	// OIS AF drift register writing
	mot_eqs_ois_af_drift_setting[0].reg_setting[0].reg_data = mot_ois_runtime[index].ois_prv_data.bu63169_data.x_offset;
	mot_eqs_ois_af_drift_setting[0].reg_setting[1].reg_data= mot_ois_runtime[index].ois_prv_data.bu63169_data.y_offset;
	if (mot_ois_apply_settings(&mot_ois_runtime[index].io_master, mot_eqs_ois_af_drift_setting, ARRAY_SIZE(mot_eqs_ois_af_drift_setting)) < 0) {
		CAM_ERR(CAM_OIS, "AF drift write failed.");
		return -1;
	}
	CAM_DBG(CAM_OIS, "AF drift write success.");
	return 0;
}

static int32_t mot_ois_bu63169_apply_af_drift(uint32_t index)
{
	if (mot_ois_runtime[index].ois_prv_data.bu63169_data.is_af_drift_valid == true) {
		CAM_DBG(CAM_OIS, "OIS AF drift data is ready cached.");
		return mot_ois_bu63169_write_af_drift(index);
	} else {
		uint8_t bu63169_af_drift[BU63169_AF_DRIFT_SIZE];
		int32_t i;
		int32_t xOffset = 0, yOffset = 0;
		int32_t segIdx;
		int32_t safeDac = mot_ois_dev_list[mot_device_index].ois_info[index].af_safe_dac;

		if (mot_ois_read_data(&mot_ois_runtime[index].eeprom_io_master, mot_ois_dev_list[mot_device_index].ois_info[index].af_drift_offset, bu63169_af_drift,
		    CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE, mot_ois_dev_list[mot_device_index].ois_info[index].af_drift_size) < 0) {
			CAM_ERR(CAM_OIS, "Failed to read OIS AF drift data.");
			mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.segmentCount = 0;
			return -1;
		}
		CAM_DBG(CAM_OIS, "OIS AF drift data is ready.");
		/*Better check EEPROM data here.*/
		for (i=0; i<BU63169_AF_DRIFT_SEGMENTS; i++) {
			mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[i].afDac = (i ? (BU63169_AF_DAC_STEP * i) - 1 : 0);
			mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[i].oisHallX = _read_16bit_msb_first(&bu63169_af_drift[i*4+8]);
			mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[i].oisHallY =_read_16bit_msb_first(&bu63169_af_drift[i*4+8+2]) ;
			CAM_DBG(CAM_OIS, "AF Drift[%d]: DAC: %d, xOffset:%d, yOffset:%d",
			        i,
			        mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[i].afDac,
			        mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[i].oisHallX,
			        mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[i].oisHallY);
		}
		mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.segmentCount = BU63169_AF_DRIFT_SEGMENTS;
		if (safeDac <= mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[0].afDac) {
			xOffset = mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[0].oisHallX;
			yOffset = mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[0].oisHallY;
			CAM_DBG(CAM_OIS, "DAC touch lower bound, x,y offset: 0x%04x, 0x%04x", xOffset, yOffset);
		} else if (safeDac >= mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[BU63169_AF_DRIFT_SEGMENTS-1].afDac) {
			xOffset = mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[BU63169_AF_DRIFT_SEGMENTS-1].oisHallX;
			yOffset = mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[BU63169_AF_DRIFT_SEGMENTS-1].oisHallY;
			CAM_DBG(CAM_OIS, "DAC touch higher bound, x,y offset: 0x%04x, 0x%04x", xOffset, yOffset);
		} else {
			for (segIdx=0; segIdx<BU63169_AF_DRIFT_SEGMENTS; segIdx++) {
				if (safeDac < mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx].afDac) {
					xOffset = (mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx].oisHallX - mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].oisHallX) *
					          (safeDac - mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].afDac) / 512 + mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].oisHallX;
					yOffset = (mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx].oisHallY - mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].oisHallY)*
					          (safeDac - mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].afDac) / 512 + mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].oisHallY;
					CAM_DBG(CAM_OIS,"curAfDac: \t%d\t segIdx: \t%d\t  xOffset: \t%d\t yOffset: \t%d\t x_top: \t%d\t x_bot: \t%d\t y_top: \t%d\t y_bot: \t%d\t count: \t%d",
					                safeDac, segIdx, xOffset, yOffset,
					                mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx].oisHallX,
					                mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].oisHallX,
					                mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx].oisHallX,
					                mot_ois_runtime[index].ois_prv_data.bu63169_data.af_drift_data.AfDriftSegments[segIdx-1].oisHallX,
					                BU63169_AF_DRIFT_SEGMENTS);
					break;
				}
			}
		}
		mot_ois_runtime[index].ois_prv_data.bu63169_data.x_offset = xOffset;
		mot_ois_runtime[index].ois_prv_data.bu63169_data.y_offset = yOffset;
		mot_ois_runtime[index].ois_prv_data.bu63169_data.is_af_drift_valid = true;

		return mot_ois_bu63169_write_af_drift(index);
	}
}

static int32_t mot_ois_bu63169_start(struct device *device, uint32_t index)
{
	int32_t rc = 0;
	const struct firmware *fw_prog = NULL;
	const struct firmware *fw_coef = NULL;
	char fw_name_prog[DEVICE_NAME_LEN];
	char fw_name_coef[DEVICE_NAME_LEN];

	snprintf(fw_name_prog, DEVICE_NAME_LEN, "%s.prog", mot_ois_dev_list[mot_device_index].ois_info[index].ois_name);
	rc = request_firmware(&fw_prog, fw_name_prog, device);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	snprintf(fw_name_coef, DEVICE_NAME_LEN, "%s.coeff", mot_ois_dev_list[mot_device_index].ois_info[index].ois_name);
	rc = request_firmware(&fw_coef, fw_name_coef, device);
	if (rc) {
		release_firmware(fw_prog);
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coef);
		return rc;
	}

	/*
	  Key flow:
	    Power ON -> oisfwinitSettings -> Prog firmware download -> Coef firmware download ->
	    oisinitSettings -> Calibration download -> OIS mode setting(Lock center)
	*/
	//Power ON
	CAM_DBG(CAM_OIS, "OIS protection power on OIS...");
	if (mot_ois_power_on(device, index) < 0) {
		CAM_ERR(CAM_OIS, "Power ON OIS failed.");
		goto error_exit;
	}

	//CCI init
	CAM_DBG(CAM_OIS, "OIS protection init CCI...");
	if (mot_ois_init_cci(index) < 0) {
		CAM_ERR(CAM_OIS, "Init OIS failed.");
		goto error_exit;
	}
	usleep_range(5000,6000);

	//oisfwinitSettings
	CAM_DBG(CAM_OIS, "OIS protection preparing for fw downloading...");
	if (mot_ois_apply_settings(&mot_ois_runtime[index].io_master, &mot_eqs_ois_fw_init_setting, 1) < 0) {
		CAM_ERR(CAM_OIS, "preparing for FW download failed.");
		goto error_exit;
	}
	usleep_range(2000,3000);

	//Prog firmware download
	CAM_DBG(CAM_OIS, "OIS protection downloading prog data...");
	if (mot_ois_bu63169_fw_download(&mot_ois_runtime[index].io_master, fw_prog, FW_REG_ADDR) < 0) {
		CAM_ERR(CAM_OIS, "FW download failed.");
		goto error_exit;
	}
	usleep_range(1000,2000);

	//Coef firmware download
	CAM_DBG(CAM_OIS, "OIS protection downloading coef data...");
	if (mot_ois_bu63169_fw_download(&mot_ois_runtime[index].io_master, fw_coef, COEF_REG_ADDR) < 0) {
		CAM_ERR(CAM_OIS, "FW download failed.");
		goto error_exit;
	}

	//oisinitSettings -> Calibration download
	CAM_DBG(CAM_OIS, "OIS protection preparing for download calibration data...");
	if (mot_ois_apply_settings(&mot_ois_runtime[index].io_master, mot_eqs_ois_init_setting, ARRAY_SIZE(mot_eqs_ois_init_setting)) < 0) {
		CAM_ERR(CAM_OIS, "FW download failed.");
		goto error_exit;
	}

	// Calibration download (read calibration from EEPROM -> Download calibration)
	CAM_DBG(CAM_OIS, "OIS protection downloading calibration data...");
	if (mot_ois_runtime[index].ois_prv_data.bu63169_data.is_cali_data_valid == false) {
		if (!mot_ois_read_data(&mot_ois_runtime[index].eeprom_io_master, mot_ois_dev_list[mot_device_index].ois_info[index].cali_offset,
		    mot_ois_runtime[index].ois_prv_data.bu63169_data.cali_data, CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE,
		    mot_ois_dev_list[mot_device_index].ois_info[index].cali_size)) {
			CAM_DBG(CAM_OIS, "OIS calibration data ready.");
			mot_ois_runtime[index].ois_prv_data.bu63169_data.is_cali_data_valid = true;
		} else {
			CAM_ERR(CAM_OIS, "Failed to read OIS calibration data.");
			goto error_exit;
		}
	}
	if (mot_ois_runtime[index].ois_prv_data.bu63169_data.is_cali_data_valid == true) {
		if (mot_ois_bu63169_apply_calibration(&mot_ois_runtime[index].io_master, mot_ois_runtime[index].ois_prv_data.bu63169_data.cali_data,
		    mot_ois_dev_list[mot_device_index].ois_info[index].cali_size) < 0) {
			CAM_ERR(CAM_OIS, "FW download failed.");
			goto error_exit;
		}
		CAM_DBG(CAM_OIS, "OIS calibration is ready.");
	}

	// OIS mode setting(Lock center)
	if (mot_ois_apply_settings(&mot_ois_runtime[index].io_master, mot_eqs_ois_lock_center_setting, ARRAY_SIZE(mot_eqs_ois_lock_center_setting)) < 0) {
		CAM_ERR(CAM_OIS, "FW download failed.");
		goto error_exit;
	}
	CAM_DBG(CAM_OIS, "OIS protection is ready.");

	if (mot_ois_dev_list[mot_device_index].ois_info[index].is_af_drift_supported == true) {
		if (mot_ois_bu63169_apply_af_drift(index) < 0) {
			CAM_ERR(CAM_OIS, "FW download failed.");
			goto error_exit;
		}
		CAM_DBG(CAM_OIS, "AF drift is ready.");
	}

	release_firmware(fw_prog);
	release_firmware(fw_coef);

	return 0;
error_exit:
	release_firmware(fw_prog);
	release_firmware(fw_coef);
	return -1;
}

static int32_t mot_ois_bu63169_stop(struct device *dev, uint32_t index)
{
	//CCI release
	CAM_DBG(CAM_OIS, "OIS protection release CCI...");
	mot_ois_release_cci(index);
	return mot_ois_power_off(dev, index);
}

int32_t mot_ois_start_protection(struct device *device)
{
	int i;
	int32_t index;
	int32_t ret = 0;

	if (mot_device_index == INVALID_OIS_INDEX) {
		CAM_ERR(CAM_OIS, "OIS index is invalid.");
		return -1;
	}

	if (!device) {
		CAM_ERR(CAM_OIS, "device ptr is NULL.");
		return -1;
	}

	if (!ois_runtime_inited) {
		mot_ois_runtime_init(device);
	}

	for (i=0; i<mot_ois_dev_list[mot_device_index].ois_num; i++) {
		index = mot_get_ois_runtime_index(mot_ois_dev_list[mot_device_index].ois_info[i].ois_type, mot_ois_dev_list[mot_device_index].ois_info[i].ois_name);
		if (mot_ois_dev_list[mot_device_index].ois_info[i].start_func) {
			ret |= mot_ois_dev_list[mot_device_index].ois_info[i].start_func(device, index);
			if (ret < 0) {
				CAM_ERR(CAM_OIS, "OIS start failed!(%d)", ret);
			}
		}
	}
	return ret;
}
EXPORT_SYMBOL(mot_ois_start_protection);

int32_t mot_ois_stop_protection(struct device *device)
{
	int i;
	int32_t index;
	int32_t ret = 0;

	if (mot_device_index == INVALID_OIS_INDEX) {
		CAM_ERR(CAM_OIS, "OIS index is invalid.");
		return -1;
	}

	if (!device) {
		CAM_ERR(CAM_OIS, "device ptr is NULL.");
		return -1;
	}

	if (!ois_runtime_inited) {
		mot_ois_runtime_init(device);
	}

	for (i=0; i<mot_ois_dev_list[mot_device_index].ois_num; i++) {
		index = mot_get_ois_runtime_index(mot_ois_dev_list[mot_device_index].ois_info[i].ois_type, mot_ois_dev_list[mot_device_index].ois_info[i].ois_name);
		if (mot_ois_dev_list[mot_device_index].ois_info[i].stop_func) {
			ret |= mot_ois_dev_list[mot_device_index].ois_info[i].stop_func(device, index);
			if (ret < 0) {
				CAM_ERR(CAM_OIS, "OIS start failed!(%d)", ret);
			}
		}
	}
	return ret;
}
EXPORT_SYMBOL(mot_ois_stop_protection);

int32_t mot_ois_handle_shut_down(struct device *device)
{
	if (mot_device_index == INVALID_OIS_INDEX) {
		CAM_ERR(CAM_OIS, "OIS index is invalid.");
		return -1;
	}
	mot_ois_runtime_uinit();
	return 0;
}
EXPORT_SYMBOL(mot_ois_handle_shut_down);

int32_t mot_ois_select_device_by_name(char *dev_name)
{
	int32_t i;
	int32_t dev_num = ARRAY_SIZE(mot_ois_dev_list);
	if (!dev_name) {
		CAM_ERR(CAM_OIS, "NULL DEVICE:%s", dev_name);
		return -1;
	}
	for (i=0; i<dev_num; i++) {
		if (!strcmp(dev_name, mot_ois_dev_list[i].dev_name)) {
			mot_device_index = i;
			CAM_DBG(CAM_OIS, "Found device:%s, index:%d", dev_name, mot_device_index);
			break;
		}
	}
	if (i >= dev_num) {
		mot_device_index = INVALID_OIS_INDEX;
		CAM_ERR(CAM_OIS, "UNKNOWN DEVICE:%s", dev_name);
	}
	return 0;
}
EXPORT_SYMBOL(mot_ois_select_device_by_name);

/**
 * mot_ois_get_dt_clk_info()
 *
 * @brief:              Parse the DT and populate the Clock properties
 *
 * @soc_info:           device soc struct to be populated
 * @src_clk_str         name of src clock that has rate control
 *
 * @return:             success or failure
 */
static int mot_ois_get_dt_clk_info(mot_ois_clk_info *soc_info)
{
	#define CAM_TO_MASK(bitn)          (1 << (int)(bitn))
	#define CAM_SET_BIT(mask, bit)     ((mask) |= CAM_TO_MASK(bit))
	struct device_node *of_node = NULL;
	int count;
	int num_clk_rates, num_clk_levels;
	int i, j, rc;
	int32_t num_clk_level_strings;
	const char *src_clk_str = NULL;
	const char *scl_clk_str = NULL;
	const char *clk_control_debugfs = NULL;
	const char *clk_cntl_lvl_string = NULL;
	enum cam_vote_level level;
	int shared_clk_cnt;
	struct of_phandle_args clk_args = {0};

	if (!soc_info || !soc_info->dev)
		return -EINVAL;

	of_node = soc_info->dev->of_node;

	if (!of_property_read_bool(of_node, "use-shared-clk")) {
		CAM_DBG(CAM_UTIL, "No shared clk parameter defined");
		soc_info->use_shared_clk = false;
	} else {
		soc_info->use_shared_clk = true;
	}

	count = of_property_count_strings(of_node, "clock-names");

	CAM_DBG(CAM_UTIL, "E: dev_name = %s count = %d",
		soc_info->dev_name, count);
	if (count > CAM_SOC_MAX_CLK) {
		CAM_ERR(CAM_UTIL, "invalid count of clocks, count=%d", count);
		rc = -EINVAL;
		return rc;
	}
	if (count <= 0) {
		CAM_DBG(CAM_UTIL, "No clock-names found");
		count = 0;
		soc_info->num_clk = count;
		return 0;
	}
	soc_info->num_clk = count;

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(of_node, "clock-names",
				i, &(soc_info->clk_name[i]));
		CAM_DBG(CAM_UTIL, "clock-names[%d] = %s",
			i, soc_info->clk_name[i]);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"i= %d count= %d reading clock-names failed",
				i, count);
			return rc;
		}
	}

	num_clk_rates = of_property_count_u32_elems(of_node, "clock-rates");
	if (num_clk_rates <= 0) {
		CAM_ERR(CAM_UTIL, "reading clock-rates count failed");
		return -EINVAL;
	}

	if ((num_clk_rates % soc_info->num_clk) != 0) {
		CAM_ERR(CAM_UTIL,
			"mismatch clk/rates, No of clocks=%d, No of rates=%d",
			soc_info->num_clk, num_clk_rates);
		return -EINVAL;
	}

	num_clk_levels = (num_clk_rates / soc_info->num_clk);

	num_clk_level_strings = of_property_count_strings(of_node,
		"clock-cntl-level");
	if (num_clk_level_strings != num_clk_levels) {
		CAM_ERR(CAM_UTIL,
			"Mismatch No of levels=%d, No of level string=%d",
			num_clk_levels, num_clk_level_strings);
		return -EINVAL;
	}

	soc_info->lowest_clk_level = CAM_TURBO_VOTE;

	for (i = 0; i < num_clk_levels; i++) {
		rc = of_property_read_string_index(of_node,
			"clock-cntl-level", i, &clk_cntl_lvl_string);
		if (rc) {
			CAM_ERR(CAM_UTIL,
				"Error reading clock-cntl-level, rc=%d", rc);
			return rc;
		}

		rc = cam_soc_util_get_level_from_string(clk_cntl_lvl_string,
			&level);
		if (rc)
			return rc;

		CAM_DBG(CAM_UTIL,
			"[%d] : %s %d", i, clk_cntl_lvl_string, level);
		soc_info->clk_level_valid[level] = true;
		for (j = 0; j < soc_info->num_clk; j++) {
			rc = of_property_read_u32_index(of_node, "clock-rates",
				((i * soc_info->num_clk) + j),
				&soc_info->clk_rate[level][j]);
			if (rc) {
				CAM_ERR(CAM_UTIL,
					"Error reading clock-rates, rc=%d",
					rc);
				return rc;
			}

			soc_info->clk_rate[level][j] =
				(soc_info->clk_rate[level][j] == 0) ?
				(int32_t)NO_SET_RATE :
				soc_info->clk_rate[level][j];

			CAM_DBG(CAM_UTIL, "soc_info->clk_rate[%d][%d] = %d",
				level, j,
				soc_info->clk_rate[level][j]);
		}

		if ((level > CAM_MINSVS_VOTE) &&
			(level < soc_info->lowest_clk_level))
			soc_info->lowest_clk_level = level;
	}

	soc_info->src_clk_idx = -1;
	rc = of_property_read_string_index(of_node, "src-clock-name", 0,
		&src_clk_str);
	if (rc || !src_clk_str) {
		CAM_DBG(CAM_UTIL, "No src_clk_str found");
		rc = 0;
		goto end;
	}

	for (i = 0; i < soc_info->num_clk; i++) {
		if (strcmp(soc_info->clk_name[i], src_clk_str) == 0) {
			soc_info->src_clk_idx = i;
			CAM_DBG(CAM_UTIL, "src clock = %s, index = %d",
				src_clk_str, i);
		}

		rc = of_parse_phandle_with_args(of_node, "clocks",
			"#clock-cells", i, &clk_args);
		if (rc) {
			CAM_ERR(CAM_CPAS,
				"failed to clock info rc=%d", rc);
			rc = -EINVAL;
			goto end;
		}

		soc_info->clk_id[i] = clk_args.args[0];
		of_node_put(clk_args.np);

		CAM_DBG(CAM_UTIL, "Dev %s clk %s id %d",
			soc_info->dev_name, soc_info->clk_name[i],
			soc_info->clk_id[i]);
	}

	CAM_DBG(CAM_UTIL, "Dev %s src_clk_idx %d, lowest_clk_level %d",
		soc_info->dev_name, soc_info->src_clk_idx,
		soc_info->lowest_clk_level);

	soc_info->shared_clk_mask = 0;
	shared_clk_cnt = of_property_count_u32_elems(of_node, "shared-clks");
	if (shared_clk_cnt <= 0) {
		CAM_DBG(CAM_UTIL, "Dev %s, no shared clks", soc_info->dev_name);
	} else if (shared_clk_cnt != count) {
		CAM_ERR(CAM_UTIL, "Dev %s, incorrect shared clock count %d %d",
			soc_info->dev_name, shared_clk_cnt, count);
		rc = -EINVAL;
		goto end;
	} else {
		uint32_t shared_clk_val;

		for (i = 0; i < shared_clk_cnt; i++) {
			rc = of_property_read_u32_index(of_node,
				"shared-clks", i, &shared_clk_val);
			if (rc || (shared_clk_val > 1)) {
				CAM_ERR(CAM_UTIL,
					"Incorrect shared clk info at %d, val=%d, count=%d",
					i, shared_clk_val, shared_clk_cnt);
				rc = -EINVAL;
				goto end;
			}

			if (shared_clk_val)
				CAM_SET_BIT(soc_info->shared_clk_mask, i);
		}

		CAM_DBG(CAM_UTIL, "Dev %s shared clk mask 0x%x",
			soc_info->dev_name, soc_info->shared_clk_mask);
	}

	/* scalable clk info parsing */
	soc_info->scl_clk_count = 0;
	soc_info->scl_clk_count = of_property_count_strings(of_node,
		"scl-clk-names");
	if ((soc_info->scl_clk_count <= 0) ||
		(soc_info->scl_clk_count > CAM_SOC_MAX_CLK)) {
		if (soc_info->scl_clk_count == -EINVAL) {
			CAM_DBG(CAM_UTIL, "scl_clk_name prop not avialable");
		} else if ((soc_info->scl_clk_count == -ENODATA) ||
			(soc_info->scl_clk_count > CAM_SOC_MAX_CLK)) {
			CAM_ERR(CAM_UTIL, "Invalid scl_clk_count: %d",
				soc_info->scl_clk_count);
			return -EINVAL;
		}
		CAM_DBG(CAM_UTIL, "Invalid scl_clk count: %d",
			soc_info->scl_clk_count);
		soc_info->scl_clk_count = -1;
	} else {
		CAM_DBG(CAM_UTIL, "No of scalable clocks: %d",
			soc_info->scl_clk_count);
		for (i = 0; i < soc_info->scl_clk_count; i++) {
			rc = of_property_read_string_index(of_node,
				"scl-clk-names", i,
				(const char **)&scl_clk_str);
			if (rc || !scl_clk_str) {
				CAM_WARN(CAM_UTIL, "scl_clk_str is NULL");
				soc_info->scl_clk_idx[i] = -1;
				continue;
			}
			for (j = 0; j < soc_info->num_clk; j++) {
				if (strnstr(scl_clk_str, soc_info->clk_name[j],
					strlen(scl_clk_str))) {
					soc_info->scl_clk_idx[i] = j;
					CAM_DBG(CAM_UTIL,
						"scl clock = %s, index = %d",
						scl_clk_str, j);
					break;
				}
			}
		}
	}

	rc = of_property_read_string_index(of_node,
		"clock-control-debugfs", 0, &clk_control_debugfs);
	if (rc || !clk_control_debugfs) {
		CAM_DBG(CAM_UTIL, "No clock_control_debugfs property found");
		rc = 0;
		goto end;
	}

	if (strcmp("true", clk_control_debugfs) == 0)
		soc_info->clk_control_enable = true;

	CAM_DBG(CAM_UTIL, "X: dev_name = %s count = %d",
		soc_info->dev_name, count);
end:
	return rc;
}
