#include "otto_pca_adapter.h"

void otto_pca_init(void) {
    pca9685_init();
}

void otto_pca_set_servo_angle(uint8_t servo_id, uint8_t angle) {
    if (servo_id > 5) return;
    pca9685_set_angle(servo_id, angle);
}
