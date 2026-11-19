#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// =======================================================
// 【核心定义】这些宏之前丢了，导致 .c 文件找不到
// =======================================================
#define PCA9685_I2C_SDA_PIN   8
#define PCA9685_I2C_SCL_PIN   9
#define PCA9685_I2C_FREQ_HZ   400000  // 提升到 400kHz 解决动作卡顿
#define PCA9685_ADDR          0x40

// 舵机脉宽标定（500us -> 102, 2500us -> 512）
#define SERVO_0_DEG_COUNT     102
#define SERVO_180_DEG_COUNT   512

esp_err_t pca9685_init(void);
esp_err_t pca9685_set_angle(uint8_t channel, uint8_t angle);
void pca9685_all_off(void);

#ifdef __cplusplus
}
#endif
