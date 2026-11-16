#include "otto_pca_adapter.h"
#include "esp_log.h"

static const char *TAG = "OTTO_PCA";

// =========================================================
// 【核心修复】物理通道映射表
// 索引对应 Otto 的逻辑舵机: 0=左腿, 1=右腿, 2=左脚, 3=右脚, 4=左手, 5=右手
// 值对应 PCA9685 的实际物理通道 (0~15)
// =========================================================
static const uint8_t s_channel_map[6] = {
    0,  // 逻辑左腿 -> 物理通道 0
    1,  // 逻辑右腿 -> 物理通道 1
    2,  // 逻辑左脚 -> 物理通道 2
    3,  // 逻辑右脚 -> 物理通道 3
    4,  // 逻辑左手 -> 物理通道 4
    5   // 逻辑右手 -> 物理通道 5
};

void otto_pca_init(void) {
    pca9685_init();
}

void otto_pca_set_servo_angle(uint8_t servo_id, uint8_t angle) {
    if (servo_id > 5) return;
    
    // 通过映射表获取实际物理通道
    uint8_t physical_channel = s_channel_map[servo_id];
    
    // 调试日志：如果动作不对，可以打开这行看映射是否正确
    // ESP_LOGI(TAG, "Logic Servo %d -> Physical CH %d, Angle: %d", servo_id, physical_channel, angle);
    
    pca9685_set_angle(physical_channel, angle);
}
