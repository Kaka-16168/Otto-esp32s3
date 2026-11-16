#include "otto_pca_adapter.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// 逻辑舵机索引 (0=左腿, 1=右腿, 2=左脚, 3=右脚, 4=左手, 5=右手)
static const uint8_t s_channel_map[6] = {0, 1, 2, 3, 4, 5};

void otto_pca_init(void) {
    pca9685_init();
}

void otto_pca_set_servo_angle(uint8_t servo_id, uint8_t angle) {
    if (servo_id > 5) return;
    // 短暂让出CPU给Wi-Fi驱动，避免I2C长时间独占导致协议栈超时复位
    taskYIELD();
    pca9685_set_angle(s_channel_map[servo_id], angle);
}
