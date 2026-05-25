/*
 * Copyright (c) 2026 LalaPadGen2 contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * TI DRV2605L Haptic Motor Controller driver.
 *
 * Drives an LRA or ERM via the built-in waveform library. Effects are
 * triggered by writing the effect ID to WAVEFORM_SEQUENCER_1 and setting
 * the GO bit. Init configures feedback control for LRA (or leaves it at
 * the default for ERM) and selects library 6 (LRA) by default.
 */

#define DT_DRV_COMPAT ti_drv2605

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/drivers/haptic/drv2605.h>

LOG_MODULE_REGISTER(drv2605, CONFIG_LOG_DEFAULT_LEVEL);

/* Register map (subset). See DRV2605L datasheet for full description. */
#define DRV2605_REG_STATUS          0x00
#define DRV2605_REG_MODE            0x01
#define DRV2605_REG_RT_PB_INPUT     0x02
#define DRV2605_REG_LIBRARY         0x03
#define DRV2605_REG_WAVEFORM_SEQ_1  0x04
#define DRV2605_REG_WAVEFORM_SEQ_2  0x05
#define DRV2605_REG_WAVEFORM_SEQ_3  0x06
#define DRV2605_REG_WAVEFORM_SEQ_4  0x07
#define DRV2605_REG_WAVEFORM_SEQ_5  0x08
#define DRV2605_REG_WAVEFORM_SEQ_6  0x09
#define DRV2605_REG_WAVEFORM_SEQ_7  0x0A
#define DRV2605_REG_WAVEFORM_SEQ_8  0x0B
#define DRV2605_REG_GO              0x0C
#define DRV2605_REG_RATED_V         0x16
#define DRV2605_REG_OVERDRIVE_CLAMP 0x17
#define DRV2605_REG_AUTOCAL_COMP    0x18
#define DRV2605_REG_AUTOCAL_BEMF    0x19
#define DRV2605_REG_FEEDBACK        0x1A
#define DRV2605_REG_CONTROL1        0x1B
#define DRV2605_REG_CONTROL2        0x1C
#define DRV2605_REG_CONTROL3        0x1D
#define DRV2605_REG_CONTROL4        0x1E

/* MODE register bits */
#define DRV2605_MODE_INTERNAL_TRIG  0x00
#define DRV2605_MODE_AUTO_CAL       0x07
#define DRV2605_MODE_STANDBY        0x40
#define DRV2605_MODE_RESET          0x80

/* FEEDBACK register bits */
#define DRV2605_FEEDBACK_LRA        0x80
#define DRV2605_FEEDBACK_BRAKE_4X   0x10

/* GO register */
#define DRV2605_GO_TRIGGER          0x01

/* Library selection (register 0x03) */
#define DRV2605_LIBRARY_EMPTY       0x00
#define DRV2605_LIBRARY_ERM_A       0x01
#define DRV2605_LIBRARY_ERM_B       0x02
#define DRV2605_LIBRARY_ERM_C       0x03
#define DRV2605_LIBRARY_ERM_D       0x04
#define DRV2605_LIBRARY_ERM_E       0x05
#define DRV2605_LIBRARY_LRA         0x06

#define DRV2605_SEQ_SLOTS           8

#define DRV2605_RESET_DELAY_MS      10
#define DRV2605_AUTOCAL_TIMEOUT_MS  500

struct drv2605_config {
	struct i2c_dt_spec i2c;
	uint16_t rated_voltage_mv;
	uint16_t overdrive_voltage_mv;
	uint8_t library;
	bool is_lra;
};

struct drv2605_data {
	struct k_mutex lock;
	bool initialized;
};

static int drv2605_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct drv2605_config *cfg = dev->config;

	return i2c_reg_write_byte_dt(&cfg->i2c, reg, val);
}

static int drv2605_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct drv2605_config *cfg = dev->config;

	return i2c_reg_read_byte_dt(&cfg->i2c, reg, val);
}

/*
 * Compute the RATED_VOLTAGE register value for an LRA driven in closed-loop
 * mode. See DRV2605L datasheet section 8.5.2.1, equation:
 *     RATED_VOLTAGE = V_rms * 255 / 5.3V * sqrt(1 - (4*SAMPLE_TIME + 300us)/PERIOD)
 *
 * We use the simplified approximation that the manufacturer documents in the
 * application note (sufficient for typical 175 Hz LRAs):
 *     reg = V_rms_mv * 255 / 5300
 */
static uint8_t drv2605_rated_v_reg(uint16_t rated_mv)
{
	uint32_t val = ((uint32_t)rated_mv * 255U) / 5300U;

	return (uint8_t)MIN(val, 255U);
}

/*
 * Compute OD_CLAMP register value (overdrive clamp). Simplified:
 *     reg = V_peak_mv * 255 / 5300
 */
static uint8_t drv2605_overdrive_reg(uint16_t overdrive_mv)
{
	uint32_t val = ((uint32_t)overdrive_mv * 255U) / 5300U;

	return (uint8_t)MIN(val, 255U);
}

int drv2605_play_effect(const struct device *dev, uint8_t effect_id)
{
	struct drv2605_data *data;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}
	data = dev->data;

	if (!data->initialized) {
		return -ENODEV;
	}
	if (effect_id == 0) {
		return 0;
	}
	if (effect_id > 123) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = drv2605_write_reg(dev, DRV2605_REG_WAVEFORM_SEQ_1, effect_id);
	if (ret < 0) {
		goto out;
	}
	ret = drv2605_write_reg(dev, DRV2605_REG_WAVEFORM_SEQ_2, 0x00);
	if (ret < 0) {
		goto out;
	}
	ret = drv2605_write_reg(dev, DRV2605_REG_GO, DRV2605_GO_TRIGGER);

out:
	k_mutex_unlock(&data->lock);
	if (ret < 0) {
		LOG_WRN("play_effect(%u) failed: %d", effect_id, ret);
	}
	return ret;
}

int drv2605_play_sequence(const struct device *dev,
			  const uint8_t *effects, size_t count)
{
	struct drv2605_data *data;
	static const uint8_t slot_regs[DRV2605_SEQ_SLOTS] = {
		DRV2605_REG_WAVEFORM_SEQ_1, DRV2605_REG_WAVEFORM_SEQ_2,
		DRV2605_REG_WAVEFORM_SEQ_3, DRV2605_REG_WAVEFORM_SEQ_4,
		DRV2605_REG_WAVEFORM_SEQ_5, DRV2605_REG_WAVEFORM_SEQ_6,
		DRV2605_REG_WAVEFORM_SEQ_7, DRV2605_REG_WAVEFORM_SEQ_8,
	};
	int ret = 0;
	size_t programmed;

	if (dev == NULL || effects == NULL) {
		return -EINVAL;
	}
	data = dev->data;
	if (!data->initialized) {
		return -ENODEV;
	}
	if (count == 0) {
		return 0;
	}
	count = MIN(count, (size_t)DRV2605_SEQ_SLOTS);

	k_mutex_lock(&data->lock, K_FOREVER);

	for (programmed = 0; programmed < count; programmed++) {
		ret = drv2605_write_reg(dev, slot_regs[programmed],
					effects[programmed]);
		if (ret < 0) {
			goto out;
		}
		if (effects[programmed] == 0) {
			programmed++;
			break;
		}
	}
	/* Mark end-of-sequence in the next slot if room left. */
	if (programmed < DRV2605_SEQ_SLOTS) {
		ret = drv2605_write_reg(dev, slot_regs[programmed], 0x00);
		if (ret < 0) {
			goto out;
		}
	}

	ret = drv2605_write_reg(dev, DRV2605_REG_GO, DRV2605_GO_TRIGGER);

out:
	k_mutex_unlock(&data->lock);
	if (ret < 0) {
		LOG_WRN("play_sequence failed: %d", ret);
	}
	return ret;
}

int drv2605_stop(const struct device *dev)
{
	struct drv2605_data *data;
	int ret;

	if (dev == NULL) {
		return -EINVAL;
	}
	data = dev->data;
	if (!data->initialized) {
		return -ENODEV;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = drv2605_write_reg(dev, DRV2605_REG_GO, 0x00);
	k_mutex_unlock(&data->lock);
	return ret;
}

bool drv2605_is_playing(const struct device *dev)
{
	uint8_t go = 0;

	if (dev == NULL) {
		return false;
	}
	(void)drv2605_read_reg(dev, DRV2605_REG_GO, &go);
	return (go & DRV2605_GO_TRIGGER) != 0;
}

#if IS_ENABLED(CONFIG_DRV2605_AUTO_CALIBRATION)
static int drv2605_run_autocal(const struct device *dev)
{
	uint8_t status;
	int ret;
	int64_t deadline;

	ret = drv2605_write_reg(dev, DRV2605_REG_MODE, DRV2605_MODE_AUTO_CAL);
	if (ret < 0) {
		return ret;
	}
	ret = drv2605_write_reg(dev, DRV2605_REG_GO, DRV2605_GO_TRIGGER);
	if (ret < 0) {
		return ret;
	}

	deadline = k_uptime_get() + DRV2605_AUTOCAL_TIMEOUT_MS;
	do {
		k_msleep(10);
		ret = drv2605_read_reg(dev, DRV2605_REG_GO, &status);
		if (ret < 0) {
			return ret;
		}
		if ((status & DRV2605_GO_TRIGGER) == 0) {
			break;
		}
	} while (k_uptime_get() < deadline);

	if ((status & DRV2605_GO_TRIGGER) != 0) {
		LOG_WRN("auto-calibration timed out");
		return -ETIMEDOUT;
	}

	ret = drv2605_read_reg(dev, DRV2605_REG_STATUS, &status);
	if (ret < 0) {
		return ret;
	}
	if (status & 0x08) {
		LOG_WRN("auto-calibration reported failure (status=0x%02x)", status);
		return -EIO;
	}
	LOG_INF("auto-calibration ok (status=0x%02x)", status);
	return 0;
}
#endif

static int drv2605_init(const struct device *dev)
{
	const struct drv2605_config *cfg = dev->config;
	struct drv2605_data *data = dev->data;
	uint8_t feedback;
	int ret;

	k_mutex_init(&data->lock);

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	/* Soft reset. The device clears the reset bit when ready. */
	ret = drv2605_write_reg(dev, DRV2605_REG_MODE, DRV2605_MODE_RESET);
	if (ret < 0) {
		LOG_ERR("reset write failed: %d", ret);
		return ret;
	}
	k_msleep(DRV2605_RESET_DELAY_MS);

	/* Exit standby into internal-trigger mode. */
	ret = drv2605_write_reg(dev, DRV2605_REG_MODE, DRV2605_MODE_INTERNAL_TRIG);
	if (ret < 0) {
		return ret;
	}

	/* Configure feedback control. */
	feedback = cfg->is_lra ? (DRV2605_FEEDBACK_LRA | DRV2605_FEEDBACK_BRAKE_4X)
			       : DRV2605_FEEDBACK_BRAKE_4X;
	ret = drv2605_write_reg(dev, DRV2605_REG_FEEDBACK, feedback);
	if (ret < 0) {
		return ret;
	}

	/* Program rated and overdrive clamp voltages. */
	ret = drv2605_write_reg(dev, DRV2605_REG_RATED_V,
				drv2605_rated_v_reg(cfg->rated_voltage_mv));
	if (ret < 0) {
		return ret;
	}
	ret = drv2605_write_reg(dev, DRV2605_REG_OVERDRIVE_CLAMP,
				drv2605_overdrive_reg(cfg->overdrive_voltage_mv));
	if (ret < 0) {
		return ret;
	}

	/* Select waveform library. */
	ret = drv2605_write_reg(dev, DRV2605_REG_LIBRARY, cfg->library);
	if (ret < 0) {
		return ret;
	}

#if IS_ENABLED(CONFIG_DRV2605_AUTO_CALIBRATION)
	(void)drv2605_run_autocal(dev);
	/* Return to internal-trigger mode after calibration regardless of result. */
	ret = drv2605_write_reg(dev, DRV2605_REG_MODE, DRV2605_MODE_INTERNAL_TRIG);
	if (ret < 0) {
		return ret;
	}
#endif

	data->initialized = true;
	LOG_INF("DRV2605L ready (%s, %u mV rated, %u mV overdrive, lib=%u)",
		cfg->is_lra ? "LRA" : "ERM",
		cfg->rated_voltage_mv, cfg->overdrive_voltage_mv, cfg->library);
	return 0;
}

#define DRV2605_DEFINE(inst)                                                    \
	static struct drv2605_data drv2605_data_##inst;                         \
	static const struct drv2605_config drv2605_config_##inst = {            \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                              \
		.rated_voltage_mv =                                             \
			DT_INST_PROP_OR(inst, rated_voltage_mv, 2000),          \
		.overdrive_voltage_mv =                                         \
			DT_INST_PROP_OR(inst, overdrive_voltage_mv, 2400),      \
		.library = DT_INST_PROP_OR(inst, library, 6),                   \
		.is_lra = (DT_INST_ENUM_IDX_OR(inst, actuator_type, 0) == 0),   \
	};                                                                       \
	DEVICE_DT_INST_DEFINE(inst, drv2605_init, NULL,                         \
			      &drv2605_data_##inst, &drv2605_config_##inst,     \
			      POST_KERNEL,                                       \
			      CONFIG_DRV2605_INIT_PRIORITY,                      \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(DRV2605_DEFINE)
