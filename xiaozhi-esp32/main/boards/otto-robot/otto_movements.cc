#include "otto_movements.h"

#include <algorithm>
#include <cmath>

#include "freertos/idf_additions.h"
#include "oscillator.h"

// =========================================================
// 引入 PCA9685 的底层适配层
// =========================================================
extern "C" {
#include "otto_pca_adapter.h"
}

static const char* TAG = "OttoMovements";

#define HAND_HOME_POSITION 45
#define PI 3.14159265f

namespace {
// 【核心优化 1】使用正弦平滑函数代替 EaseOutCubic，让动作起步和停止都非常温柔
float EaseInOutSine(float t) {
    return -(cosf(PI * t) - 1.0f) / 2.0f;
}
}  // namespace

Otto::Otto() {
    is_otto_resting_ = false;
    has_hands_ = false;
    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_pins_[i] = -1;
        servo_trim_[i] = 0;
    }
}

Otto::~Otto() {
    DetachServos();
}

unsigned long IRAM_ATTR millis() {
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

void Otto::Init(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand,
                int right_hand) {
    // 强制通道映射
    servo_pins_[LEFT_LEG] = 0;
    servo_pins_[RIGHT_LEG] = 1;
    servo_pins_[LEFT_FOOT] = 2;
    servo_pins_[RIGHT_FOOT] = 3;
    servo_pins_[LEFT_HAND] = 4;
    servo_pins_[RIGHT_HAND] = 5;

    has_hands_ = true; 
    AttachServos();
    is_otto_resting_ = false;
}

///////////////////////////////////////////////////////////////////
//-- ATTACH & DETACH FUNCTIONS ----------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::AttachServos() {}

void Otto::DetachServos() {
    pca9685_all_off();
}

///////////////////////////////////////////////////////////////////
//-- OSCILLATORS TRIMS ------------------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::SetTrims(int left_leg, int right_leg, int left_foot, int right_foot, int left_hand,
                    int right_hand) {
    servo_trim_[LEFT_LEG] = left_leg;
    servo_trim_[RIGHT_LEG] = right_leg;
    servo_trim_[LEFT_FOOT] = left_foot;
    servo_trim_[RIGHT_FOOT] = right_foot;

    if (has_hands_) {
        servo_trim_[LEFT_HAND] = left_hand;
        servo_trim_[RIGHT_HAND] = right_hand;
    }

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetTrim(servo_trim_[i]);
        }
    }
}

///////////////////////////////////////////////////////////////////
//-- BASIC MOTION FUNCTIONS -------------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::MoveServos(int time, int servo_target[]) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    if (time <= 10) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].SetPosition(servo_target[i]);
                pca9685_set_angle(i, servo_target[i]);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(time));
        return;
    }

    int start[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        start[i] = servo_[i].GetPosition();
    }

    // 【核心优化 2】拉长步进时间，从 10ms 改为 15ms，动作更平缓
    int steps = std::max(1, time / 15);
    for (int step = 1; step <= steps; step++) {
        float t = static_cast<float>(step) / static_cast<float>(steps);
        float eased_t = EaseInOutSine(t); // 使用正弦缓动
        
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                float interpolated = start[i] + (servo_target[i] - start[i]) * eased_t;
                int pos = static_cast<int>(std::round(interpolated));
                servo_[i].SetPosition(pos);
                pca9685_set_angle(i, pos);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15)); // 对应的延迟也改为 15ms
    }

    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetPosition(servo_target[i]);
            pca9685_set_angle(i, servo_target[i]);
        }
    }
}

void Otto::MoveSingle(int position, int servo_number) {
    if (position > 180) position = 90;
    if (position < 0) position = 90;

    if (GetRestState() == true) {
        SetRestState(false);
    }

    if (servo_number >= 0 && servo_number < SERVO_COUNT && servo_pins_[servo_number] != -1) {
        servo_[servo_number].SetPosition(position);
        pca9685_set_angle(servo_number, position);
    }
}

void Otto::OscillateServos(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                           double phase_diff[SERVO_COUNT], float cycle = 1) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) {
            servo_[i].SetO(offset[i]);
            servo_[i].SetA(amplitude[i]);
            servo_[i].SetT(period);
            servo_[i].SetPh(phase_diff[i]);
        }
    }

    double ref = millis();
    double end_time = period * cycle + ref;

    while (millis() < end_time) {
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (servo_pins_[i] != -1) {
                servo_[i].Refresh();
                pca9685_set_angle(i, servo_[i].GetPosition());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Otto::Execute(int amplitude[SERVO_COUNT], int offset[SERVO_COUNT], int period,
                   double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    int cycles = (int)steps;

    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void Otto::Execute2(int amplitude[SERVO_COUNT], int center_angle[SERVO_COUNT], int period,
                    double phase_diff[SERVO_COUNT], float steps = 1.0) {
    if (GetRestState() == true) {
        SetRestState(false);
    }

    int offset[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        offset[i] = center_angle[i] - 90;
    }

    int cycles = (int)steps;

    if (cycles >= 1)
        for (int i = 0; i < cycles; i++)
            OscillateServos(amplitude, offset, period, phase_diff);

    OscillateServos(amplitude, offset, period, phase_diff, (float)steps - cycles);
    vTaskDelay(pdMS_TO_TICKS(10));
}

///////////////////////////////////////////////////////////////////
//-- HOME = Otto at rest position -------------------------------//
///////////////////////////////////////////////////////////////////
void Otto::Home(bool hands_down) {
    if (is_otto_resting_ == false) {  
        int homes[SERVO_COUNT];
        int max_delta = 0;
        for (int i = 0; i < SERVO_COUNT; i++) {
            if (i == LEFT_HAND || i == RIGHT_HAND) {
                if (hands_down) {
                    if (i == LEFT_HAND) {
                        homes[i] = HAND_HOME_POSITION;
                    } else {                                  
                        homes[i] = 180 - HAND_HOME_POSITION;  
                    }
                } else {
                    homes[i] = servo_[i].GetPosition();
                }
            } else {
                homes[i] = 90;
            }

            if (servo_pins_[i] != -1) {
                max_delta = std::max(max_delta, std::abs(homes[i] - servo_[i].GetPosition()));
            }
        }

        // 归位时间稍微拉长，显得更从容
        int home_time = std::clamp(800 + max_delta * 10, 800, 2000);
        MoveServos(home_time, homes);
        is_otto_resting_ = true;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
}

bool Otto::GetRestState() {
    return is_otto_resting_;
}

void Otto::SetRestState(bool state) {
    is_otto_resting_ = state;
}

///////////////////////////////////////////////////////////////////
//-- PREDETERMINED MOTION SEQUENCES -----------------------------//
///////////////////////////////////////////////////////////////////

void Otto::Jump(float steps, int period) {
    int up[SERVO_COUNT] = {90, 90, 150, 30, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    MoveServos(period, up);
    int down[SERVO_COUNT] = {90, 90, 90, 90, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    MoveServos(period, down);
}

void Otto::Walk(float steps, int period, int dir, int amount) {
    // 【核心优化 3】降低基础振幅，从 30 降到 20，走起来更稳、更温柔
    int A[SERVO_COUNT] = {20, 20, 20, 20, 0, 0};
    int O[SERVO_COUNT] = {0, 0, 5, -5, HAND_HOME_POSITION - 90, HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(dir * -90), DEG2RAD(dir * -90), 0, 0};

    if (amount > 0 && has_hands_) {
        // 手臂摆幅也跟着减小
        A[LEFT_HAND] = amount / 2; 
        A[RIGHT_HAND] = amount / 2;
        phase_diff[LEFT_HAND] = phase_diff[RIGHT_LEG];  
        phase_diff[RIGHT_HAND] = phase_diff[LEFT_LEG];  
    } else {
        A[LEFT_HAND] = 0;
        A[RIGHT_HAND] = 0;
    }
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Turn(float steps, int period, int dir, int amount) {
    // 【核心优化】转弯振幅也降低
    int A[SERVO_COUNT] = {20, 20, 20, 20, 0, 0};
    int O[SERVO_COUNT] = {0, 0, 5, -5, HAND_HOME_POSITION - 90, HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(-90), DEG2RAD(-90), 0, 0};

    if (dir == LEFT) {
        A[0] = 20;  
        A[1] = 0;   
    } else {
        A[0] = 0;
        A[1] = 20;
    }

    if (amount > 0 && has_hands_) {
        A[LEFT_HAND] = amount / 2;
        A[RIGHT_HAND] = amount / 2;
        phase_diff[LEFT_HAND] = phase_diff[LEFT_LEG];    
        phase_diff[RIGHT_HAND] = phase_diff[RIGHT_LEG];  
    } else {
        A[LEFT_HAND] = 0;
        A[RIGHT_HAND] = 0;
    }
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Bend(int steps, int period, int dir) {
    int bend1[SERVO_COUNT] = {90, 90, 62, 35, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    int bend2[SERVO_COUNT] = {90, 90, 62, 105, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    int homes[SERVO_COUNT] = {90, 90, 90, 90, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};

    if (dir == -1) {
        bend1[2] = 180 - 35;
        bend1[3] = 180 - 60;  
        bend2[2] = 180 - 105;
        bend2[3] = 180 - 60;
    }

    int T2 = 1000; // 拉长弯曲时间
    for (int i = 0; i < steps; i++) {
        MoveServos(T2 / 2, bend1);
        MoveServos(T2 / 2, bend2);
        vTaskDelay(pdMS_TO_TICKS(period * 0.8));
        MoveServos(600, homes);
    }
}

void Otto::ShakeLeg(int steps, int period, int dir) {
    int numberLegMoves = 2;
    int shake_leg1[SERVO_COUNT] = {90, 90, 58, 35, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    int shake_leg2[SERVO_COUNT] = {90, 90, 58, 120, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    int shake_leg3[SERVO_COUNT] = {90, 90, 58, 60, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    int homes[SERVO_COUNT] = {90, 90, 90, 90, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};

    if (dir == LEFT) {
        shake_leg1[2] = 180 - 35;
        shake_leg1[3] = 180 - 58;
        shake_leg2[2] = 180 - 120;
        shake_leg2[3] = 180 - 58;
        shake_leg3[2] = 180 - 60;
        shake_leg3[3] = 180 - 58;
    }

    int T2 = 1200;
    period = period - T2;
    period = std::max(period, 200 * numberLegMoves);

    for (int j = 0; j < steps; j++) {
        MoveServos(T2 / 2, shake_leg1);
        MoveServos(T2 / 2, shake_leg2);
        for (int i = 0; i < numberLegMoves; i++) {
            MoveServos(period / (2 * numberLegMoves), shake_leg3);
            MoveServos(period / (2 * numberLegMoves), shake_leg2);
        }
        MoveServos(500, homes); 
    }
    vTaskDelay(pdMS_TO_TICKS(period));
}

void Otto::Sit() {
    int target[SERVO_COUNT] = {120, 60, 0, 180, 45, 135};
    MoveServos(800, target); // 坐下放慢
}

void Otto::UpDown(float steps, int period, int height) {
    int A[SERVO_COUNT] = {0, 0, height, height, 0, 0};
    int O[SERVO_COUNT] = {0, 0, height, -height, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(-90), DEG2RAD(90), 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Swing(float steps, int period, int height) {
    int A[SERVO_COUNT] = {0, 0, height, height, 0, 0};
    int O[SERVO_COUNT] = {
        0, 0, height / 2, -height / 2, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(0), DEG2RAD(0), 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::TiptoeSwing(float steps, int period, int height) {
    int A[SERVO_COUNT] = {0, 0, height, height, 0, 0};
    int O[SERVO_COUNT] = {0, 0, height, -height, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Jitter(float steps, int period, int height) {
    height = std::min(20, height);
    int A[SERVO_COUNT] = {height, height, 0, 0, 0, 0};
    int O[SERVO_COUNT] = {0, 0, 0, 0, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {DEG2RAD(-90), DEG2RAD(90), 0, 0, 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::AscendingTurn(float steps, int period, int height) {
    height = std::min(10, height);
    int A[SERVO_COUNT] = {height, height, height, height, 0, 0};
    int O[SERVO_COUNT] = {
        0, 0, height + 4, -height + 4, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {DEG2RAD(-90), DEG2RAD(90), DEG2RAD(-90), DEG2RAD(90), 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Moonwalker(float steps, int period, int height, int dir) {
    int A[SERVO_COUNT] = {0, 0, height, height, 0, 0};
    int O[SERVO_COUNT] = {
        0, 0, height / 2 + 2, -height / 2 - 2, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    int phi = -dir * 90;
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(phi), DEG2RAD(-60 * dir + phi), 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Crusaito(float steps, int period, int height, int dir) {
    int A[SERVO_COUNT] = {20, 20, height, height, 0, 0};
    int O[SERVO_COUNT] = {
        0, 0, height / 2 + 4, -height / 2 - 4, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {90, 90, DEG2RAD(0), DEG2RAD(-60 * dir), 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::Flapping(float steps, int period, int height, int dir) {
    int A[SERVO_COUNT] = {10, 10, height, height, 0, 0};
    int O[SERVO_COUNT] = {
        0, 0, height - 10, -height + 10, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    double phase_diff[SERVO_COUNT] = {
        DEG2RAD(0), DEG2RAD(180), DEG2RAD(-90 * dir), DEG2RAD(90 * dir), 0, 0};
    Execute(A, O, period, phase_diff, steps);
}

void Otto::WhirlwindLeg(float steps, int period, int amplitude) {
    int target[SERVO_COUNT] = {90, 90, 180, 90, 45, 20};
    MoveServos(150, target);
    target[RIGHT_FOOT] = 160;
    MoveServos(600, target);
    vTaskDelay(pdMS_TO_TICKS(1000));

    int C[SERVO_COUNT] = {90, 90, 180, 160, 45, 20};
    int A[SERVO_COUNT] = {amplitude, 0, 0, 0, amplitude, 0};
    double phase_diff[SERVO_COUNT] = {DEG2RAD(20), 0, 0, 0, DEG2RAD(20), 0};
    Execute2(A, C, period, phase_diff, steps);
}

void Otto::HandsUp(int period, int dir) {
    if (!has_hands_) return;
    int target[SERVO_COUNT] = {90, 90, 90, 90, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    if (dir == 0) {
        target[LEFT_HAND] = 150; // 不举那么高
        target[RIGHT_HAND] = 30;
    } else if (dir == LEFT) {
        target[LEFT_HAND] = 150;
        target[RIGHT_HAND] = servo_[RIGHT_HAND].GetPosition();
    } else if (dir == RIGHT) {
        target[RIGHT_HAND] = 30;
        target[LEFT_HAND] = servo_[LEFT_HAND].GetPosition();
    }
    MoveServos(period, target);
}

void Otto::HandsDown(int period, int dir) {
    if (!has_hands_) return;
    int target[SERVO_COUNT] = {90, 90, 90, 90, HAND_HOME_POSITION, 180 - HAND_HOME_POSITION};
    if (dir == LEFT) {
        target[RIGHT_HAND] = servo_[RIGHT_HAND].GetPosition();
    } else if (dir == RIGHT) {
        target[LEFT_HAND] = servo_[LEFT_HAND].GetPosition();
    }
    MoveServos(period, target);
}

void Otto::HandWave(int dir) {
    if (!has_hands_) return;
    if (dir == LEFT) {
        int center_angle[SERVO_COUNT] = {90, 90, 90, 90, 140, 120};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 15, 0}; // 挥手幅度减小
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), 0};
        Execute2(A, center_angle, 400, phase_diff, 4); // 速度放慢
    } else if (dir == RIGHT) {
        int center_angle[SERVO_COUNT] = {90, 90, 90, 90, 60, 40};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 0, 15};
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, 0, DEG2RAD(90)};
        Execute2(A, center_angle, 400, phase_diff, 4);
    } else {
        int center_angle[SERVO_COUNT] = {90, 90, 90, 90, 140, 40};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 15, 15};
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), DEG2RAD(90)};
        Execute2(A, center_angle, 400, phase_diff, 4);
    }
}

void Otto::Windmill(float steps, int period, int amplitude) {
    if (!has_hands_) return;
    int center_angle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
    int A[SERVO_COUNT] = {0, 0, 0, 0, amplitude / 2, amplitude / 2}; // 风车幅度减半
    double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), DEG2RAD(90)};
    Execute2(A, center_angle, period, phase_diff, steps);
}

void Otto::Takeoff(float steps, int period, int amplitude) {
    if (!has_hands_) return;
    Home(true);
    int center_angle[SERVO_COUNT] = {90, 90, 90, 90, 90, 90};
    int A[SERVO_COUNT] = {0, 0, 0, 0, amplitude / 2, amplitude / 2};
    double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), DEG2RAD(-90)};
    Execute2(A, center_angle, period, phase_diff, steps);
}

void Otto::Fitness(float steps, int period, int amplitude) {
    if (!has_hands_) return;
    int target[SERVO_COUNT] = {90, 90, 90, 0, 160, 135};
    MoveServos(150, target);
    target[LEFT_FOOT] = 20;
    MoveServos(500, target);
    vTaskDelay(pdMS_TO_TICKS(2000));
    int C[SERVO_COUNT] = {90, 90, 20, 90, 160, 135};
    int A[SERVO_COUNT] = {0, 0, 0, 0, 0, amplitude / 2};
    double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
    Execute2(A, C, period, phase_diff, steps);
}

void Otto::Greeting(int dir, float steps) {
    if (!has_hands_) return;
    if (dir == LEFT) {
        int target[SERVO_COUNT] = {90, 90, 150, 150, 45, 135};
        MoveServos(500, target);
        int C[SERVO_COUNT] = {90, 90, 150, 150, 150, 135};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 15, 0};
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
        Execute2(A, C, 400, phase_diff, steps);
    } else if (dir == RIGHT) {
        int target[SERVO_COUNT] = {90, 90, 30, 30, 45, 135};
        MoveServos(500, target);
        int C[SERVO_COUNT] = {90, 90, 30, 30, 45, 30};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 0, 15};
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
        Execute2(A, C, 400, phase_diff, steps);
    }
}

void Otto::Shy(int dir, float steps) {
    if (!has_hands_) return;
    if (dir == LEFT) {
        int target[SERVO_COUNT] = {90, 90, 150, 150, 45, 135};
        MoveServos(500, target);
        int C[SERVO_COUNT] = {90, 90, 150, 150, 45, 135};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 15, 15};
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), DEG2RAD(-90)};
        Execute2(A, C, 400, phase_diff, steps);
    } else if (dir == RIGHT) {
        int target[SERVO_COUNT] = {90, 90, 30, 30, 45, 135};
        MoveServos(500, target);
        int C[SERVO_COUNT] = {90, 90, 30, 30, 45, 135};
        int A[SERVO_COUNT] = {0, 0, 0, 0, 0, 15};
        double phase_diff[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), DEG2RAD(-90)};
        Execute2(A, C, 400, phase_diff, steps);
    }
}

void Otto::RadioCalisthenics() {
    if (!has_hands_) return;
    const int period = 1500; // 延长体操周期
    const float steps = 6.0; 

    int C1[SERVO_COUNT] = {90, 90, 90, 90, 130, 60};
    int A1[SERVO_COUNT] = {0, 0, 0, 0, 30, 30};
    double phase_diff1[SERVO_COUNT] = {0, 0, 0, 0, DEG2RAD(90), DEG2RAD(-90)};
    Execute2(A1, C1, period, phase_diff1, steps);

    int C2[SERVO_COUNT] = {90, 90, 110, 70, 90, 90};
    int A2[SERVO_COUNT] = {0, 0, 15, 15, 0, 0};
    double phase_diff2[SERVO_COUNT] = {0, 0, DEG2RAD(90), DEG2RAD(-90), 0, 0};
    Execute2(A2, C2, period, phase_diff2, steps);
    
    int C3[SERVO_COUNT] = {90, 90, 120, 120, 90, 90};
    int A3[SERVO_COUNT] = {0, 0, 0, 0, 15, 0};
    double phase_diff3[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
    Execute2(A3, C3, period, phase_diff3, steps);

    int C4[SERVO_COUNT] = {90, 90, 60, 60, 90, 90};
    int A4[SERVO_COUNT] = {0, 0, 0, 0, 0, 15};
    double phase_diff4[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};
    Execute2(A4, C4, period, phase_diff4, steps);
}

void Otto::MagicCircle() {
    if (!has_hands_) return;
    int A[SERVO_COUNT] = {20, 20, 20, 20, 30, 30}; // 减小幅度
    int O[SERVO_COUNT] = {0, 0, 5, -5, 0, 0};
    double phase_diff[SERVO_COUNT] = {0, 0, DEG2RAD(-90), DEG2RAD(-90), DEG2RAD(-90) , DEG2RAD(90)};
    Execute(A, O, 1000, phase_diff, 30); // 周期从 700 拉长到 1000
}

void Otto::Showcase() {
    if (GetRestState() == true) SetRestState(false);
    Walk(2, 1500, FORWARD, 30); vTaskDelay(pdMS_TO_TICKS(500));
    if (has_hands_) { HandWave(LEFT); vTaskDelay(pdMS_TO_TICKS(500)); }
    if (has_hands_) { RadioCalisthenics(); vTaskDelay(pdMS_TO_TICKS(500)); }
    Moonwalker(2, 1200, 20, LEFT); vTaskDelay(pdMS_TO_TICKS(500));
    Swing(2, 1500, 20); vTaskDelay(pdMS_TO_TICKS(500));
    if (has_hands_) { Takeoff(3, 400, 30); vTaskDelay(pdMS_TO_TICKS(500)); }
    if (has_hands_) { Fitness(3, 1200, 15); vTaskDelay(pdMS_TO_TICKS(500)); }
    Walk(2, 1500, BACKWARD, 30);
}

void Otto::EnableServoLimit(int diff_limit) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) servo_[i].SetLimiter(diff_limit);
    }
}

void Otto::DisableServoLimit() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (servo_pins_[i] != -1) servo_[i].DisableLimiter();
    }
}
