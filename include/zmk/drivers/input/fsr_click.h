#pragma once

#include <zephyr/device.h>
#include <stdbool.h>

/**
 * Returns true if the FSR click detector currently reports a pressed state.
 * Returns false if dev is NULL or the driver has not completed calibration.
 */
bool fsr_click_is_pressed(const struct device *dev);
