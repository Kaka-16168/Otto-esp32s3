#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pca9685_init(void);
esp_err_t pca9685_set_angle(uint8_t channel, uint8_t angle);
void pca9685_all_off(void);

#ifdef __cplusplus
}
#endif
