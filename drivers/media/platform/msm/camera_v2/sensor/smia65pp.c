/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * SMIA++ module wrapped as an MSM sensor (Nokia X2 smia65pp pattern).
 * Ident uses the mainline smiapp register map (smiapp-reg-defs.h), not a
 * hardcoded Nokia X2 chip-id table. CCI is not an i2c_adapter, so the
 * 3.10 i2c smiapp driver cannot bind these nodes; we run its identify
 * sequence over MSM CCI after boot.
 */
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_dt_util.h"
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/err.h>

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

/* smiapp-reg-defs.h expects these from smiapp-regs.h (do not link smiapp.o). */
#define SMIA_REG_FLAG_FLOAT		(1 << 24)
#define SMIA_REG_8BIT			1
#define SMIA_REG_16BIT			2
#define SMIA_REG_32BIT			4
#include "../../../../i2c/smiapp/smiapp-reg-defs.h"

#define SMIAPP_ADDR(r)			((u16)(r))
#define SMIAPP_LEN(r)			((u8)((r) >> 16))

#define SMIA65PP_SENSOR_NAME "smia65pp"
#define SMIA65PP_MAX_CAM		3
#define SMIA65PP_MCLK_HZ		9600000
#define SMIA65PP_IDENT_DELAY_SEC	8
/* ACPI CAMS D0 waits 0x19 ms after MCLK pin mux before the sensor is used. */
#define SMIA65PP_ACPI_MCLK_US		25000
/* smiapp.h: 2400 extclk cycles + 1ms, plus the extra 10ms the driver notes. */
#define SMIA65PP_XSHUTDOWN_US		(1000 + \
	(2400 * 1000 + SMIA65PP_MCLK_HZ / 1000 - 1) / (SMIA65PP_MCLK_HZ / 1000) \
	+ 10000)

DEFINE_MSM_MUTEX(smia65pp_mut);

static struct msm_sensor_ctrl_t smia65pp_s_ctrl;
static struct msm_sensor_ctrl_t *smia65pp_devs[SMIA65PP_MAX_CAM];
static int smia65pp_ndev;
static DEFINE_MUTEX(smia65pp_list_lock);
static void smia65pp_ident_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(smia65pp_ident_work, smia65pp_ident_work_fn);
/* LVS1 / cam_vio: enable for ident, never disable (power_down reboots talkman). */
static struct regulator *smia65pp_vio_hold;

static int ident_delay_sec = SMIA65PP_IDENT_DELAY_SEC;
module_param(ident_delay_sec, int, 0644);

static struct msm_sensor_power_setting smia65pp_power_setting[] = {
	/* Do not put CAM_VIO here. cam_vio is PM8994 LVS1 (shared 1.8V). */
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = SMIA65PP_MCLK_HZ,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 1,
	},
};

static struct v4l2_subdev_info smia65pp_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SGRBG10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id smia65pp_i2c_id[] = {
	{SMIA65PP_SENSOR_NAME, (kernel_ulong_t)&smia65pp_s_ctrl},
	{ }
};

static int32_t msm_smia65pp_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &smia65pp_s_ctrl);
}

static struct i2c_driver smia65pp_i2c_driver = {
	.id_table = smia65pp_i2c_id,
	.probe  = msm_smia65pp_i2c_probe,
	.driver = {
		.name = SMIA65PP_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client smia65pp_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id smia65pp_dt_match[] = {
	{.compatible = "qcom,smia65pp"},
	{}
};

MODULE_DEVICE_TABLE(of, smia65pp_dt_match);

static int32_t smia65pp_parse_dt(struct platform_device *pdev,
				 struct msm_sensor_ctrl_t *s_ctrl)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_sensor_board_info *sd;
	const char *name = "smia65pp";
	uint32_t cell_id = 0, master = 1, i2c_reg = 0;
	uint32_t slave[3] = {0, 0, 0};
	int32_t rc;

	s_ctrl->pdev = pdev;
	s_ctrl->of_node = of_node;
	s_ctrl->sensor_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	s_ctrl->sensordata = kzalloc(sizeof(*s_ctrl->sensordata), GFP_KERNEL);
	if (!s_ctrl->sensordata)
		return -ENOMEM;
	sd = s_ctrl->sensordata;

	of_property_read_u32(of_node, "cell-index", &cell_id);
	s_ctrl->id = cell_id;

	of_property_read_string(of_node, "qcom,sensor-name", &name);
	sd->sensor_name = name;

	of_property_read_u32(of_node, "qcom,cci-master", &master);
	s_ctrl->cci_i2c_master = master;

	/* msm-cci.txt: reg is the I2C slave address. slave-id is optional. */
	of_property_read_u32_array(of_node, "qcom,slave-id", slave, 3);
	if (!of_property_read_u32(of_node, "reg", &i2c_reg) && i2c_reg)
		slave[0] = i2c_reg;

	sd->slave_info = kzalloc(sizeof(*sd->slave_info), GFP_KERNEL);
	if (!sd->slave_info)
		return -ENOMEM;
	sd->slave_info->sensor_slave_addr = slave[0];
	sd->slave_info->sensor_id_reg_addr = slave[1];
	sd->slave_info->sensor_id = slave[2];

	rc = msm_sensor_get_sub_module_index(of_node, &sd->sensor_info);
	if (rc < 0) {
		pr_err("talkman_smia: sub_module_index rc=%d\n", rc);
		return rc;
	}

	of_property_read_u32(of_node, "qcom,mount-angle",
			     &sd->sensor_info->sensor_mount_angle);
	sd->sensor_info->is_mount_angle_valid = 1;
	of_property_read_u32(of_node, "qcom,sensor-position",
			     &sd->sensor_info->position);
	of_property_read_u32(of_node, "qcom,sensor-mode",
			     &sd->sensor_info->modes_supported);

	pr_err("talkman_smia: parsed %s cell=%u cci_master=%u sid=0x%x expect=0x%04x csiphy=%d csid=%d mount=%u\n",
	       name, cell_id, master, slave[0], slave[2],
	       sd->sensor_info->subdev_id[SUB_MODULE_CSIPHY],
	       sd->sensor_info->subdev_id[SUB_MODULE_CSID],
	       sd->sensor_info->sensor_mount_angle);
	return 0;
}

static void smia65pp_fill_cci(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_camera_cci_client *cci_client;

	if (!s_ctrl->sensor_i2c_client ||
	    !s_ctrl->sensor_i2c_client->cci_client)
		return;
	cci_client = s_ctrl->sensor_i2c_client->cci_client;
	/* Do not replace a probe-time pointer with NULL (%pK also hides it). */
	if (msm_cci_get_subdev())
		cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	if (s_ctrl->sensordata && s_ctrl->sensordata->slave_info)
		cci_client->sid =
			s_ctrl->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	if (s_ctrl->sensordata && s_ctrl->pdev)
		s_ctrl->sensordata->power_info.dev = &s_ctrl->pdev->dev;
	pr_err("talkman_smia: cci sid=0x%x master=%u subdev=%pK\n",
	       cci_client->sid, cci_client->cci_i2c_master,
	       cci_client->cci_subdev);
}

static int32_t smia65pp_alloc_s_ctrl(struct msm_sensor_ctrl_t **out)
{
	struct msm_sensor_ctrl_t *s_ctrl;
	struct msm_camera_i2c_client *i2c;
	struct mutex *mut;

	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	i2c = kzalloc(sizeof(*i2c), GFP_KERNEL);
	mut = kzalloc(sizeof(*mut), GFP_KERNEL);
	if (!s_ctrl || !i2c || !mut) {
		kfree(s_ctrl);
		kfree(i2c);
		kfree(mut);
		return -ENOMEM;
	}
	*s_ctrl = smia65pp_s_ctrl;
	*i2c = smia65pp_sensor_i2c_client;
	mutex_init(mut);
	s_ctrl->sensor_i2c_client = i2c;
	s_ctrl->msm_sensor_mutex = mut;
	s_ctrl->pdev = NULL;
	s_ctrl->sensordata = NULL;
	s_ctrl->of_node = NULL;
	*out = s_ctrl;
	return 0;
}

static int32_t smia65pp_read(struct msm_sensor_ctrl_t *s_ctrl, uint16_t reg,
			     uint16_t *val, uint32_t dt)
{
	*val = 0;
	if (!s_ctrl || !s_ctrl->sensor_i2c_client ||
	    !s_ctrl->sensor_i2c_client->i2c_func_tbl ||
	    !s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read)
		return -ENODEV;
	return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client, reg, val, dt);
}

static int32_t smia65pp_write(struct msm_sensor_ctrl_t *s_ctrl, uint16_t reg,
			      uint16_t val, uint32_t dt)
{
	if (!s_ctrl || !s_ctrl->sensor_i2c_client ||
	    !s_ctrl->sensor_i2c_client->i2c_func_tbl ||
	    !s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write)
		return -ENODEV;
	return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
		s_ctrl->sensor_i2c_client, reg, val, dt);
}

static int smia65pp_smiapp_read(struct msm_sensor_ctrl_t *s_ctrl, u32 packed,
				u32 *out)
{
	uint16_t val = 0;
	u16 addr = SMIAPP_ADDR(packed);
	u8 len = SMIAPP_LEN(packed);
	uint32_t dt;
	int rc;

	*out = 0;
	if (len == SMIA_REG_8BIT)
		dt = MSM_CAMERA_I2C_BYTE_DATA;
	else if (len == SMIA_REG_16BIT)
		dt = MSM_CAMERA_I2C_WORD_DATA;
	else
		return -EINVAL;
	rc = smia65pp_read(s_ctrl, addr, &val, dt);
	if (rc < 0)
		return rc;
	*out = val;
	return 0;
}

static int smia65pp_smiapp_write(struct msm_sensor_ctrl_t *s_ctrl, u32 packed,
				 u16 val)
{
	u16 addr = SMIAPP_ADDR(packed);
	u8 len = SMIAPP_LEN(packed);
	uint32_t dt;

	if (len == SMIA_REG_8BIT)
		dt = MSM_CAMERA_I2C_BYTE_DATA;
	else if (len == SMIA_REG_16BIT)
		dt = MSM_CAMERA_I2C_WORD_DATA;
	else
		return -EINVAL;
	return smia65pp_write(s_ctrl, addr, val, dt);
}

/* Mainline smiapp_identify_module() register list, over CCI. */
static void smia65pp_smiapp_identify(struct msm_sensor_ctrl_t *s_ctrl)
{
	const char *name = s_ctrl->sensordata ?
		s_ctrl->sensordata->sensor_name : "?";
	u32 manufacturer_id = 0, model_id = 0;
	u32 rev_major = 0, rev_minor = 0;
	u32 year = 0, month = 0, day = 0;
	u32 sensor_mfr = 0, sensor_model = 0;
	u32 sensor_rev = 0, sensor_fw = 0;
	u32 smia = 0, smiapp = 0;
	u32 xout = 0, yout = 0, fll = 0, llp = 0;
	int rc = 0;

	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MANUFACTURER_ID,
				   &manufacturer_id);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_MODEL_ID, &model_id);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_REVISION_NUMBER_MAJOR,
				   &rev_major);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_REVISION_NUMBER_MINOR,
				   &rev_minor);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MODULE_DATE_YEAR,
				   &year);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MODULE_DATE_MONTH,
				   &month);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MODULE_DATE_DAY, &day);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SENSOR_MANUFACTURER_ID,
				   &sensor_mfr);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_SENSOR_MODEL_ID,
				   &sensor_model);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SENSOR_REVISION_NUMBER,
				   &sensor_rev);
	rc |= smia65pp_smiapp_read(s_ctrl,
				   SMIAPP_REG_U8_SENSOR_FIRMWARE_VERSION,
				   &sensor_fw);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SMIA_VERSION, &smia);
	rc |= smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_SMIAPP_VERSION,
				   &smiapp);
	smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_X_OUTPUT_SIZE, &xout);
	smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_Y_OUTPUT_SIZE, &yout);
	smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_FRAME_LENGTH_LINES, &fll);
	smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_LINE_LENGTH_PCK, &llp);

	if (rc) {
		pr_err("smiapp: %s sensor detection failed rc=%d\n", name, rc);
		return;
	}

	if (!manufacturer_id && !model_id) {
		manufacturer_id = sensor_mfr;
		model_id = sensor_model;
		rev_major = sensor_rev;
	}

	pr_err("smiapp: %s module 0x%02x-0x%04x\n",
	       name, manufacturer_id, model_id);
	pr_err("smiapp: %s module revision 0x%02x-0x%02x date %02u-%02u-%02u\n",
	       name, rev_major, rev_minor, year, month, day);
	pr_err("smiapp: %s sensor 0x%02x-0x%04x\n",
	       name, sensor_mfr, sensor_model);
	pr_err("smiapp: %s sensor revision 0x%02x firmware version 0x%02x\n",
	       name, sensor_rev, sensor_fw);
	pr_err("smiapp: %s smia version %u smiapp version %u ident %02x%04x%02x\n",
	       name, smia, smiapp, manufacturer_id, model_id, rev_major);
	pr_err("smiapp: %s window %ux%u llp=%u fll=%u\n",
	       name, xout, yout, llp, fll);
}

static int smia65pp_cci_util(struct msm_sensor_ctrl_t *s_ctrl, uint16_t cmd)
{
	if (!s_ctrl->sensor_i2c_client ||
	    !s_ctrl->sensor_i2c_client->i2c_func_tbl ||
	    !s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util)
		return -ENODEV;
	return s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
		s_ctrl->sensor_i2c_client, cmd);
}

/*
 * Acquire a GPIO already muxed by pinctrl-0. -EBUSY: still drive it.
 * Returns 1 if gpio_free() is required, 0 if not, negative on hard fail.
 */
static int smia65pp_xpin_acquire(int gpio, const char *label, int init_high)
{
	unsigned long flags = init_high ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
	int rc;

	if (!gpio_is_valid(gpio))
		return -EINVAL;
	rc = gpio_request_one(gpio, flags, label);
	if (!rc)
		return 1;
	pr_err("smiapp: gpio %d (%s) request rc=%d, forcing output %d\n",
	       gpio, label, rc, init_high);
	gpio_direction_output(gpio, init_high);
	return 0;
}

static void smia65pp_vio_enable_keep(struct device *dev, const char *name)
{
	struct regulator *vio;
	int rc;

	vio = regulator_get(dev, "cam_vio");
	if (IS_ERR(vio)) {
		pr_err("smiapp: %s cam_vio get rc=%ld\n", name, PTR_ERR(vio));
		return;
	}
	rc = regulator_enable(vio);
	pr_err("smiapp: %s cam_vio enable rc=%d (LVS1, never disable)\n",
	       name, rc);
	if (rc) {
		regulator_put(vio);
		return;
	}
	if (!smia65pp_vio_hold)
		smia65pp_vio_hold = vio;
	else
		regulator_put(vio);
}

static void smia65pp_ident_one(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct device *dev;
	struct device_node *np;
	struct regulator *vana = NULL, *vdig = NULL;
	struct clk *src = NULL, *mclk = NULL;
	const char *name;
	u32 rst_idx = 1, stby_idx = 2, stby_rel = 1;
	int rst = -EINVAL, stby = -EINVAL;
	int rst_owned = 0, stby_owned = 0;
	int rc;

	if (!s_ctrl || !s_ctrl->pdev)
		return;
	dev = &s_ctrl->pdev->dev;
	np = dev->of_node;
	name = s_ctrl->sensordata ? s_ctrl->sensordata->sensor_name : "?";

	pr_err("smiapp: ident start %s (%s)\n", name, dev_name(dev));

	smia65pp_fill_cci(s_ctrl);

	of_property_read_u32(np, "qcom,gpio-reset", &rst_idx);
	of_property_read_u32(np, "qcom,gpio-standby", &stby_idx);
	of_property_read_u32(np, "qcom,standby-release", &stby_rel);
	rst = of_get_gpio(np, rst_idx);
	stby = of_get_gpio(np, stby_idx);
	pr_err("smiapp: %s xshutdown gpio=%d (idx %u) standby gpio=%d (idx %u) release=%u\n",
	       name, rst, rst_idx, stby, stby_idx, stby_rel);

	vdig = regulator_get(dev, "cam_vdig");
	if (IS_ERR(vdig)) {
		pr_err("smiapp: %s cam_vdig get rc=%ld\n", name, PTR_ERR(vdig));
		vdig = NULL;
	}
	vana = regulator_get(dev, "cam_vana");
	if (IS_ERR(vana)) {
		pr_err("smiapp: %s cam_vana get rc=%ld\n", name, PTR_ERR(vana));
		vana = NULL;
	}
	src = clk_get(dev, "cam_src_clk");
	if (IS_ERR(src)) {
		pr_err("smiapp: %s cam_src_clk get rc=%ld\n", name, PTR_ERR(src));
		src = NULL;
	}
	mclk = clk_get(dev, "cam_clk");
	if (IS_ERR(mclk)) {
		pr_err("smiapp: %s cam_clk get rc=%ld\n", name, PTR_ERR(mclk));
		mclk = NULL;
	}

	/*
	 * 3.10 smiapp_power_on: VANA, xclk, xshutdown=1, software reset.
	 * ACPI CAMS D0 also votes LVS1 (VIO). Enable it and never disable.
	 */
	if (vana) {
		rc = regulator_enable(vana);
		pr_err("smiapp: %s cam_vana enable rc=%d\n", name, rc);
	}
	if (vdig) {
		rc = regulator_enable(vdig);
		pr_err("smiapp: %s cam_vdig enable rc=%d\n", name, rc);
	}
	smia65pp_vio_enable_keep(dev, name);
	usleep_range(1000, 2000);

	/* Board standby pin is not in smiapp; pinctrl leaves it floating. */
	rc = smia65pp_xpin_acquire(stby, "smiapp-stby", stby_rel ? 1 : 0);
	if (rc > 0)
		stby_owned = 1;
	rc = smia65pp_xpin_acquire(rst, "smiapp-xshutdown", 0);
	if (rc > 0)
		rst_owned = 1;
	else if (rc < 0)
		pr_err("smiapp: %s xshutdown missing, ident will NACK\n", name);

	if (src) {
		rc = clk_set_rate(src, SMIA65PP_MCLK_HZ);
		if (rc)
			pr_err("smiapp: %s mclk rate %u rc=%d\n",
			       name, SMIA65PP_MCLK_HZ, rc);
		rc = clk_prepare_enable(src);
		pr_err("smiapp: %s cam_src_clk enable rc=%d rate=%lu\n",
		       name, rc, clk_get_rate(src));
	}
	if (mclk) {
		rc = clk_prepare_enable(mclk);
		pr_err("smiapp: %s cam_clk enable rc=%d\n", name, rc);
	}
	usleep_range(SMIA65PP_ACPI_MCLK_US, SMIA65PP_ACPI_MCLK_US + 1000);

	if (gpio_is_valid(rst)) {
		gpio_set_value_cansleep(rst, 1);
		pr_err("smiapp: %s xshutdown=1, wait %u us\n",
		       name, SMIA65PP_XSHUTDOWN_US);
		usleep_range(SMIA65PP_XSHUTDOWN_US, SMIA65PP_XSHUTDOWN_US + 1000);
	}

	if (smia65pp_cci_util(s_ctrl, MSM_CCI_INIT) < 0)
		pr_err("smiapp: %s CCI init failed\n", name);

	rc = smia65pp_smiapp_write(s_ctrl, SMIAPP_REG_U8_SOFTWARE_RESET, 1);
	pr_err("smiapp: %s software reset rc=%d\n", name, rc);
	usleep_range(SMIA65PP_XSHUTDOWN_US, SMIA65PP_XSHUTDOWN_US + 1000);

	smia65pp_smiapp_identify(s_ctrl);

	smia65pp_cci_util(s_ctrl, MSM_CCI_RELEASE);

	/* smiapp_power_off: xshutdown=0, clock off, VANA off. Never LVS1. */
	if (gpio_is_valid(rst))
		gpio_set_value_cansleep(rst, 0);
	if (mclk)
		clk_disable_unprepare(mclk);
	if (src)
		clk_disable_unprepare(src);
	if (rst_owned)
		gpio_free(rst);
	if (stby_owned)
		gpio_free(stby);
	if (vana) {
		regulator_disable(vana);
		regulator_put(vana);
	}
	if (vdig) {
		regulator_disable(vdig);
		regulator_put(vdig);
	}
	if (src)
		clk_put(src);
	if (mclk)
		clk_put(mclk);
	pr_err("smiapp: ident done %s\n", name);
}

static void smia65pp_ident_work_fn(struct work_struct *work)
{
	int i;

	pr_err("smiapp: delayed ident for %d camera node(s)\n", smia65pp_ndev);
	mutex_lock(&smia65pp_list_lock);
	for (i = 0; i < smia65pp_ndev; i++)
		smia65pp_ident_one(smia65pp_devs[i]);
	mutex_unlock(&smia65pp_list_lock);
}

static int32_t smia65pp_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	struct msm_sensor_ctrl_t *s_ctrl;

	pr_err("talkman_smia: smia-msm#12 pr-clean %s\n",
	       dev_name(&pdev->dev));

	rc = smia65pp_alloc_s_ctrl(&s_ctrl);
	if (rc < 0) {
		pr_err("talkman_smia: alloc s_ctrl failed\n");
		return 0;
	}
	platform_set_drvdata(pdev, s_ctrl);

	rc = smia65pp_parse_dt(pdev, s_ctrl);
	if (rc < 0)
		pr_err("talkman_smia: parse_dt rc=%d (still bind)\n", rc);

	if (s_ctrl->sensordata && s_ctrl->sensordata->sensor_info) {
		rc = msm_sensor_init_default_params(s_ctrl);
		pr_err("talkman_smia: init_default_params rc=%d\n", rc);
		if (!rc)
			smia65pp_fill_cci(s_ctrl);
	}

	mutex_lock(&smia65pp_list_lock);
	if (smia65pp_ndev < SMIA65PP_MAX_CAM)
		smia65pp_devs[smia65pp_ndev++] = s_ctrl;
	mutex_unlock(&smia65pp_list_lock);
	schedule_delayed_work(&smia65pp_ident_work,
			      msecs_to_jiffies(ident_delay_sec * 1000));

	return 0;
}

static struct platform_driver smia65pp_platform_driver = {
	.probe = smia65pp_platform_probe,
	.driver = {
		.name = "qcom,smia65pp",
		.owner = THIS_MODULE,
		.of_match_table = smia65pp_dt_match,
	},
};

static int __init smia65pp_init_module(void)
{
	pr_err("talkman_smia: smia-msm#12 3cam smiapp ident + LVS1 hold (no VIDEO_SMIAPP, no v4l2 at CCI probe)\n");
	return platform_driver_register(&smia65pp_platform_driver);
}

static void __exit smia65pp_exit_module(void)
{
	cancel_delayed_work_sync(&smia65pp_ident_work);
	platform_driver_unregister(&smia65pp_platform_driver);
}

int32_t smia65pp_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	u32 manufacturer_id = 0, model_id = 0, rev_major = 0;
	int rc;

	rc = smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U16_MODEL_ID, &model_id);
	if (rc < 0) {
		pr_err("smia65pp_match_id: module id read failed rc=%d\n", rc);
		return rc;
	}
	smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_MANUFACTURER_ID,
			     &manufacturer_id);
	smia65pp_smiapp_read(s_ctrl, SMIAPP_REG_U8_REVISION_NUMBER_MAJOR,
			     &rev_major);
	pr_err("smia65pp_match_id: manufacturer=0x%02x model=0x%04x rev=0x%02x\n",
	       manufacturer_id, model_id, rev_major);
	smia65pp_smiapp_identify(s_ctrl);
	return 0;
}

static struct msm_sensor_fn_t smia65pp_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = smia65pp_match_id,
};

static struct msm_sensor_ctrl_t smia65pp_s_ctrl = {
	.sensor_i2c_client = &smia65pp_sensor_i2c_client,
	.power_setting_array.power_setting = smia65pp_power_setting,
	.power_setting_array.size = ARRAY_SIZE(smia65pp_power_setting),
	.msm_sensor_mutex = &smia65pp_mut,
	.sensor_v4l2_subdev_info = smia65pp_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(smia65pp_subdev_info),
	.func_tbl = &smia65pp_sensor_func_tbl,
};

module_init(smia65pp_init_module);
module_exit(smia65pp_exit_module);
MODULE_DESCRIPTION("smia65pp");
MODULE_LICENSE("GPL v2");
