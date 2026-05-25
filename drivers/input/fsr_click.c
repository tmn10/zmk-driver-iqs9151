/*
 * Copyright (c) 2026 LalaPadGen2 contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * FSR (Force Sensitive Resistor) click detector via ADS1015 I2C ADC.
 *
 * Polls an ADS1015 channel connected to an FSR voltage divider. Emits
 * INPUT_BTN_0 press/release events into the ZMK input subsystem.
 *
 * Power-on calibration: waits 500 ms after init, samples the ADC 10 times,
 * and uses the minimum as the resting baseline. Detection fires when the
 * reading exceeds baseline + threshold_delta.
 *
 * Haptic gating: if a DRV2605L phandle is provided and
 * CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE is enabled, the driver checks the
 * DRV2605L GO register each poll. While GO is set (vibrating), and for
 * haptic-gate-ms afterward, FSR readings are suppressed to prevent
 * vibration-induced false clicks.
 */

#define DT_DRV_COMPAT zmk_fsr_click

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#if IS_ENABLED(CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE)
#include <zmk/drivers/haptic/drv2605.h>
#endif
#include <zmk/drivers/input/fsr_click.h>

LOG_MODULE_REGISTER(fsr_click, CONFIG_INPUT_FSR_CLICK_LOG_LEVEL);

#define FSR_CALIBRATION_DELAY_MS 500
#define FSR_CALIBRATION_SAMPLES  10
#define FSR_CALIBRATION_RETRY_MS 1000

struct fsr_click_config {
    struct adc_dt_spec adc;
#if IS_ENABLED(CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE)
    const struct device *haptic_dev;
    uint32_t haptic_gate_ms;
#endif
    uint32_t threshold_delta;
    uint32_t poll_interval_ms;
};

struct fsr_click_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
    struct adc_sequence adc_seq;
    int16_t adc_buf;
    int16_t baseline;
    bool calibrated;
    bool pressed;
#if IS_ENABLED(CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE)
    int64_t haptic_gate_until_ms;
#endif
};

bool fsr_click_is_pressed(const struct device *dev)
{
    if (dev == NULL) {
        return false;
    }
    const struct fsr_click_data *data = dev->data;
    return data->calibrated && data->pressed;
}

static void fsr_click_poll_work_cb(struct k_work *work)
{
    struct k_work_delayable *dwork =
        CONTAINER_OF(work, struct k_work_delayable, work);
    struct fsr_click_data *data =
        CONTAINER_OF(dwork, struct fsr_click_data, poll_work);
    const struct device *dev = data->dev;
    const struct fsr_click_config *cfg = dev->config;

    /* First invocation: run calibration (500 ms after boot). */
    if (!data->calibrated) {
        int16_t min_val = INT16_MAX;

        for (int i = 0; i < FSR_CALIBRATION_SAMPLES; i++) {
            if (adc_read(cfg->adc.dev, &data->adc_seq) < 0) {
                LOG_WRN("calibration ADC read failed, retrying in %d ms",
                        FSR_CALIBRATION_RETRY_MS);
                k_work_schedule(dwork, K_MSEC(FSR_CALIBRATION_RETRY_MS));
                return;
            }
            if (data->adc_buf < min_val) {
                min_val = data->adc_buf;
            }
            k_msleep(5);
        }
        data->baseline = min_val;
        data->calibrated = true;
        LOG_INF("FSR calibrated: baseline=%d, threshold=%d+%u",
                data->baseline, data->baseline, cfg->threshold_delta);
        k_work_schedule(dwork, K_MSEC(cfg->poll_interval_ms));
        return;
    }

#if IS_ENABLED(CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE)
    if (cfg->haptic_dev != NULL) {
        if (drv2605_is_playing(cfg->haptic_dev)) {
            data->haptic_gate_until_ms = k_uptime_get() + cfg->haptic_gate_ms;
        }
        if (k_uptime_get() < data->haptic_gate_until_ms) {
            k_work_schedule(dwork, K_MSEC(cfg->poll_interval_ms));
            return;
        }
    }
#endif

    if (adc_read(cfg->adc.dev, &data->adc_seq) < 0) {
        k_work_schedule(dwork, K_MSEC(cfg->poll_interval_ms));
        return;
    }

    bool pressed_now =
        (data->adc_buf > data->baseline + (int16_t)cfg->threshold_delta);

    if (pressed_now != data->pressed) {
        data->pressed = pressed_now;
        input_report_key(dev, INPUT_BTN_0, pressed_now ? 1 : 0, true,
                         K_NO_WAIT);
        LOG_DBG("FSR %s (raw=%d baseline=%d)",
                pressed_now ? "PRESS" : "RELEASE",
                data->adc_buf, data->baseline);
    }

    k_work_schedule(dwork, K_MSEC(cfg->poll_interval_ms));
}

static int fsr_click_init(const struct device *dev)
{
    struct fsr_click_data *data = dev->data;
    const struct fsr_click_config *cfg = dev->config;

    data->dev = dev;
    data->calibrated = false;
    data->pressed = false;
#if IS_ENABLED(CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE)
    data->haptic_gate_until_ms = 0;
#endif

    if (!adc_is_ready_dt(&cfg->adc)) {
        LOG_ERR("ADS1015 ADC not ready");
        return -ENODEV;
    }

    int ret = adc_channel_setup_dt(&cfg->adc);
    if (ret < 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }

    adc_sequence_init_dt(&cfg->adc, &data->adc_seq);
    data->adc_seq.buffer = &data->adc_buf;
    data->adc_seq.buffer_size = sizeof(data->adc_buf);

    k_work_init_delayable(&data->poll_work, fsr_click_poll_work_cb);
    k_work_schedule(&data->poll_work, K_MSEC(FSR_CALIBRATION_DELAY_MS));
    return 0;
}

#if IS_ENABLED(CONFIG_INPUT_FSR_CLICK_HAPTIC_GATE)
#define FSR_HAPTIC_DEV(inst)                                                    \
    .haptic_dev = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, haptic_dev),         \
        (DEVICE_DT_GET(DT_INST_PHANDLE(inst, haptic_dev))), (NULL)),            \
    .haptic_gate_ms = DT_INST_PROP_OR(inst, haptic_gate_ms, 200),
#else
#define FSR_HAPTIC_DEV(inst)
#endif

#define FSR_CLICK_DEFINE(inst)                                                  \
    static struct fsr_click_data fsr_click_data_##inst;                         \
    static const struct fsr_click_config fsr_click_config_##inst = {            \
        .adc = ADC_DT_SPEC_INST_GET(inst),                                      \
        FSR_HAPTIC_DEV(inst)                                                    \
        .threshold_delta = DT_INST_PROP_OR(inst, threshold_delta, 50),          \
        .poll_interval_ms = DT_INST_PROP_OR(inst, poll_interval_ms, 5),         \
    };                                                                           \
    DEVICE_DT_INST_DEFINE(inst, fsr_click_init, NULL,                           \
                          &fsr_click_data_##inst, &fsr_click_config_##inst,     \
                          POST_KERNEL, CONFIG_INPUT_FSR_CLICK_INIT_PRIORITY,    \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(FSR_CLICK_DEFINE)
