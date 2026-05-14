/*
 * Copyright (c) 2026 LalaPadGen2 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_DRIVERS_HAPTIC_DRV2605_H_
#define ZMK_DRIVERS_HAPTIC_DRV2605_H_

#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Play a single effect from the DRV2605L internal waveform library.
 *
 * @param dev       DRV2605L device pointer (from DEVICE_DT_GET).
 * @param effect_id Effect identifier 1..123. 0 is treated as no-op (returns 0).
 *
 * @return 0 on success, negative errno on failure.
 */
int drv2605_play_effect(const struct device *dev, uint8_t effect_id);

/**
 * Play a sequence of up to 8 effects back-to-back.
 *
 * @param dev     DRV2605L device pointer.
 * @param effects Array of effect IDs (1..123). Entries with value 0 terminate
 *                the sequence early.
 * @param count   Number of entries in @p effects (clamped to 8).
 *
 * @return 0 on success, negative errno on failure.
 */
int drv2605_play_sequence(const struct device *dev,
                          const uint8_t *effects, size_t count);

/**
 * Stop any ongoing waveform playback.
 *
 * @param dev DRV2605L device pointer.
 *
 * @return 0 on success, negative errno on failure.
 */
int drv2605_stop(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* ZMK_DRIVERS_HAPTIC_DRV2605_H_ */
