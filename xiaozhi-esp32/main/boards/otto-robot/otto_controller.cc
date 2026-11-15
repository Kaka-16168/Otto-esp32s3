/*
    Otto机器人控制器 - MCP协议版本
*/

#include <cJSON.h>
#include <esp_log.h>

#include <cstdlib> 
#include <cstring>

#include "application.h"
#include "board.h"
#include "config.h"
#include "mcp_server.h"
#include "otto_movements.h"
#include "power_manager.h"
#include "sdkconfig.h"
#include "settings.h"
#include <wifi_manager.h>

#define TAG "OttoController"

class OttoController {
private:
    Otto otto_;
    TaskHandle_t action_task_handle_ = nullptr;
    QueueHandle_t action_queue_;
    bool has_hands_ = false;
    bool is_action_in_progress_ = false;

    struct OttoActionParams {
        int action_type;
        int steps;
        int speed;
        int direction;
        int amount;
        char servo_sequence_json[512];
    };

    enum ActionType {
        ACTION_WALK = 1,
        ACTION_TURN = 2,
        ACTION_JUMP = 3,
        ACTION_SWING = 4,
        ACTION_MOONWALK = 5,
        ACTION_BEND = 6,
        ACTION_SHAKE_LEG = 7,
        ACTION_SIT = 25,
        ACTION_RADIO_CALISTHENICS = 26,
        ACTION_MAGIC_CIRCLE = 27,
        ACTION_UPDOWN = 8,
        ACTION_TIPTOE_SWING = 9,
        ACTION_JITTER = 10,
        ACTION_ASCENDING_TURN = 11,
        ACTION_CRUSAITO = 12,
        ACTION_FLAPPING = 13,
        ACTION_HANDS_UP = 14,
        ACTION_HANDS_DOWN = 15,
        ACTION_HAND_WAVE = 16,
        ACTION_WINDMILL = 20,
        ACTION_TAKEOFF = 21,
        ACTION_FITNESS = 22,
        ACTION_GREETING = 23,
        ACTION_SHY = 24,
        ACTION_SHOWCASE = 28,
        ACTION_HOME = 17,
        ACTION_SERVO_SEQUENCE = 18,
        ACTION_WHIRLWIND_LEG = 19
    };

    static void ActionTask(void* arg) {
        OttoController* controller = static_cast<OttoController*>(arg);
        OttoActionParams params;
        controller->otto_.AttachServos();

        while (true) {
            if (xQueueReceive(controller->action_queue_, &params, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "执行动作: %d", params.action_type);
                PowerManager::PauseBatteryUpdate();
                controller->is_action_in_progress_ = true;
                
                if (params.action_type == ACTION_SERVO_SEQUENCE) {
                    cJSON* json = cJSON_Parse(params.servo_sequence_json);
                    if (json != nullptr) {
                        cJSON* actions = cJSON_GetObjectItem(json, "a");
                        if (cJSON_IsArray(actions)) {
                            int array_size = cJSON_GetArraySize(actions);
                            int sequence_delay = 0;
                            cJSON* delay_item = cJSON_GetObjectItem(json, "d");
                            if (cJSON_IsNumber(delay_item)) {
                                sequence_delay = delay_item->valueint;
                                if (sequence_delay < 0) sequence_delay = 0;
                            }
                            
                            int current_positions[SERVO_COUNT];
                            for (int j = 0; j < SERVO_COUNT; j++) current_positions[j] = 90;
                            current_positions[LEFT_HAND] = 45;
                            current_positions[RIGHT_HAND] = 135;
                            
                            for (int i = 0; i < array_size; i++) {
                                cJSON* action_item = cJSON_GetArrayItem(actions, i);
                                if (cJSON_IsObject(action_item)) {
                                    cJSON* osc_item = cJSON_GetObjectItem(action_item, "osc");
                                    if (cJSON_IsObject(osc_item)) {
                                        int amplitude[SERVO_COUNT] = {0};
                                        int center_angle[SERVO_COUNT] = {0};
                                        double phase_diff[SERVO_COUNT] = {0};
                                        int period = 300;
                                        float steps = 8.0;
                                        const char* servo_names[] = {"ll", "rl", "lf", "rf", "lh", "rh"};
                                        
                                        for (int j = 0; j < SERVO_COUNT; j++) amplitude[j] = 0;
                                        cJSON* amp_item = cJSON_GetObjectItem(osc_item, "a");
                                        if (cJSON_IsObject(amp_item)) {
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* amp_value = cJSON_GetObjectItem(amp_item, servo_names[j]);
                                                if (cJSON_IsNumber(amp_value)) {
                                                    int amp = amp_value->valueint;
                                                    if (amp >= 10 && amp <= 90) amplitude[j] = amp;
                                                }
                                            }
                                        }
                                        
                                        for (int j = 0; j < SERVO_COUNT; j++) center_angle[j] = 90;
                                        cJSON* center_item = cJSON_GetObjectItem(osc_item, "o");
                                        if (cJSON_IsObject(center_item)) {
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* center_value = cJSON_GetObjectItem(center_item, servo_names[j]);
                                                if (cJSON_IsNumber(center_value)) {
                                                    int center = center_value->valueint;
                                                    if (center >= 0 && center <= 180) center_angle[j] = center;
                                                }
                                            }
                                        }
                                        
                                        const int LARGE_AMPLITUDE_THRESHOLD = 40;
                                        if (amplitude[LEFT_LEG] >= LARGE_AMPLITUDE_THRESHOLD && amplitude[RIGHT_LEG] >= LARGE_AMPLITUDE_THRESHOLD) {
                                            amplitude[RIGHT_LEG] = 0;
                                        }
                                        if (amplitude[LEFT_FOOT] >= LARGE_AMPLITUDE_THRESHOLD && amplitude[RIGHT_FOOT] >= LARGE_AMPLITUDE_THRESHOLD) {
                                            amplitude[RIGHT_FOOT] = 0;
                                        }
                                        
                                        cJSON* phase_item = cJSON_GetObjectItem(osc_item, "ph");
                                        if (cJSON_IsObject(phase_item)) {
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* phase_value = cJSON_GetObjectItem(phase_item, servo_names[j]);
                                                if (cJSON_IsNumber(phase_value)) {
                                                    phase_diff[j] = phase_value->valuedouble * 3.141592653589793 / 180.0;
                                                }
                                            }
                                        }
                                        
                                        cJSON* period_item = cJSON_GetObjectItem(osc_item, "p");
                                        if (cJSON_IsNumber(period_item)) {
                                            period = period_item->valueint;
                                            if (period < 100) period = 100;
                                            if (period > 3000) period = 3000;
                                        }
                                        
                                        cJSON* steps_item = cJSON_GetObjectItem(osc_item, "c");
                                        if (cJSON_IsNumber(steps_item)) {
                                            steps = (float)steps_item->valuedouble;
                                            if (steps < 0.1) steps = 0.1;
                                            if (steps > 20.0) steps = 20.0;
                                        }
                                        
                                        controller->otto_.Execute2(amplitude, center_angle, period, phase_diff, steps);
                                        for (int j = 0; j < SERVO_COUNT; j++) current_positions[j] = center_angle[j];
                                    } else {
                                        int servo_target[SERVO_COUNT];
                                        for (int j = 0; j < SERVO_COUNT; j++) servo_target[j] = current_positions[j];
                                        
                                        cJSON* servos_item = cJSON_GetObjectItem(action_item, "s");
                                        if (cJSON_IsObject(servos_item)) {
                                            const char* servo_names[] = {"ll", "rl", "lf", "rf", "lh", "rh"};
                                            for (int j = 0; j < SERVO_COUNT; j++) {
                                                cJSON* servo_value = cJSON_GetObjectItem(servos_item, servo_names[j]);
                                                if (cJSON_IsNumber(servo_value)) {
                                                    int position = servo_value->valueint;
                                                    if (position >= 0 && position <= 180) servo_target[j] = position;
                                                }
                                            }
                                        }
                                        
                                        int speed = 1000;
                                        cJSON* speed_item = cJSON_GetObjectItem(action_item, "v");
                                        if (cJSON_IsNumber(speed_item)) {
                                            speed = speed_item->valueint;
                                            if (speed < 100) speed = 100;
                                            if (speed > 3000) speed = 3000;
                                        }
                                        
                                        controller->otto_.MoveServos(speed, servo_target);
                                        for (int j = 0; j < SERVO_COUNT; j++) current_positions[j] = servo_target[j];
                                    }
                                    
                                    int delay_after = 0;
                                    cJSON* delay_item = cJSON_GetObjectItem(action_item, "d");
                                    if (cJSON_IsNumber(delay_item)) {
                                        delay_after = delay_item->valueint;
                                        if (delay_after < 0) delay_after = 0;
                                    }
                                    
                                    if (delay_after > 0 && i < array_size - 1) {
                                        vTaskDelay(pdMS_TO_TICKS(delay_after));
                                    }
                                }
                            }
                            
                            if (sequence_delay > 0) {
                                UBaseType_t queue_count = uxQueueMessagesWaiting(controller->action_queue_);
                                if (queue_count > 0) vTaskDelay(pdMS_TO_TICKS(sequence_delay));
                            }
                            cJSON_Delete(json);
                        }
                    }
                } else {
                    switch (params.action_type) {
                        case ACTION_WALK: controller->otto_.Walk(params.steps, params.speed, params.direction, params.amount); break;
                        case ACTION_TURN: controller->otto_.Turn(params.steps, params.speed, params.direction, params.amount); break;
                        case ACTION_JUMP: controller->otto_.Jump(params.steps, params.speed); break;
                        case ACTION_SWING: controller->otto_.Swing(params.steps, params.speed, params.amount); break;
                        case ACTION_MOONWALK: controller->otto_.Moonwalker(params.steps, params.speed, params.amount, params.direction); break;
                        case ACTION_BEND: controller->otto_.Bend(params.steps, params.speed, params.direction); break;
                        case ACTION_SHAKE_LEG: controller->otto_.ShakeLeg(params.steps, params.speed, params.direction); break;
                        case ACTION_SIT: controller->otto_.Sit(); break;
                        case ACTION_RADIO_CALISTHENICS: if (controller->has_hands_) controller->otto_.RadioCalisthenics(); break;
                        case ACTION_MAGIC_CIRCLE: if (controller->has_hands_) controller->otto_.MagicCircle(); break;
                        case ACTION_SHOWCASE: controller->otto_.Showcase(); break;
                        case ACTION_UPDOWN: controller->otto_.UpDown(params.steps, params.speed, params.amount); break;
                        case ACTION_TIPTOE_SWING: controller->otto_.TiptoeSwing(params.steps, params.speed, params.amount); break;
                        case ACTION_JITTER: controller->otto_.Jitter(params.steps, params.speed, params.amount); break;
                        case ACTION_ASCENDING_TURN: controller->otto_.AscendingTurn(params.steps, params.speed, params.amount); break;
                        case ACTION_CRUSAITO: controller->otto_.Crusaito(params.steps, params.speed, params.amount, params.direction); break;
                        case ACTION_FLAPPING: controller->otto_.Flapping(params.steps, params.speed, params.amount, params.direction); break;
                        case ACTION_WHIRLWIND_LEG: controller->otto_.WhirlwindLeg(params.steps, params.speed, params.amount); break;
                        case ACTION_HANDS_UP: if (controller->has_hands_) controller->otto_.HandsUp(params.speed, params.direction); break;
                        case ACTION_HANDS_DOWN: if (controller->has_hands_) controller->otto_.HandsDown(params.speed, params.direction); break;
                        case ACTION_HAND_WAVE: if (controller->has_hands_) controller->otto_.HandWave(params.direction); break;
                        case ACTION_WINDMILL: if (controller->has_hands_) controller->otto_.Windmill(params.steps, params.speed, params.amount); break;
                        case ACTION_TAKEOFF: if (controller->has_hands_) controller->otto_.Takeoff(params.steps, params.speed, params.amount); break;
                        case ACTION_FITNESS: if (controller->has_hands_) controller->otto_.Fitness(params.steps, params.speed, params.amount); break;
                        case ACTION_GREETING: if (controller->has_hands_) controller->otto_.Greeting(params.direction, params.steps); break;
                        case ACTION_SHY: if (controller->has_hands_) controller->otto_.Shy(params.direction, params.steps); break;
                        case ACTION_HOME: controller->otto_.Home(true); break;
                    }
                    if(params.action_type != ACTION_SIT){
                        if (params.action_type != ACTION_HOME && params.action_type != ACTION_SERVO_SEQUENCE) {
                            UBaseType_t pending_actions = uxQueueMessagesWaiting(controller->action_queue_);
                            if (pending_actions == 0) {
                                controller->otto_.Home(params.action_type != ACTION_HANDS_UP);
                            }
                        }
                    }
                }
                controller->is_action_in_progress_ = false;
                PowerManager::ResumeBatteryUpdate();
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    void StartActionTaskIfNeeded() {
        if (action_task_handle_ == nullptr) {
            xTaskCreate(ActionTask, "otto_action", 1024 * 3, this, configMAX_PRIORITIES - 1, &action_task_handle_);
        }
    }

    void QueueAction(int action_type, int steps, int speed, int direction, int amount) {
        if ((action_type >= ACTION_HANDS_UP && action_type <= ACTION_HAND_WAVE) || 
            (action_type == ACTION_WINDMILL) || (action_type == ACTION_TAKEOFF) || 
            (action_type == ACTION_FITNESS) || (action_type == ACTION_GREETING) ||
            (action_type == ACTION_SHY) || (action_type == ACTION_RADIO_CALISTHENICS) ||
            (action_type == ACTION_MAGIC_CIRCLE)) {
            if (!has_hands_) {
                ESP_LOGW(TAG, "尝试执行手部动作，但机器人没有配置手部舵机");
                return;
            }
        }
        OttoActionParams params = {action_type, steps, speed, direction, amount, ""};
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void QueueServoSequence(const char* servo_sequence_json) {
        if (servo_sequence_json == nullptr) return;
        int input_len = strlen(servo_sequence_json);
        const int buffer_size = 512;
        if (input_len >= buffer_size || input_len == 0) return;
        OttoActionParams params = {ACTION_SERVO_SEQUENCE, 0, 0, 0, 0, ""};
        strncpy(params.servo_sequence_json, servo_sequence_json, sizeof(params.servo_sequence_json) - 1);
        params.servo_sequence_json[sizeof(params.servo_sequence_json) - 1] = '\0';
        xQueueSend(action_queue_, &params, portMAX_DELAY);
        StartActionTaskIfNeeded();
    }

    void LoadTrimsFromNVS() {
        Settings settings("otto_trims", false);
        int left_leg = settings.GetInt("left_leg", 0);
        int right_leg = settings.GetInt("right_leg", 0);
        int left_foot = settings.GetInt("left_foot", 0);
        int right_foot = settings.GetInt("right_foot", 0);
        int left_hand = settings.GetInt("left_hand", 0);
        int right_hand = settings.GetInt("right_hand", 0);
        otto_.SetTrims(left_leg, right_leg, left_foot, right_foot, left_hand, right_hand);
    }

public:
    OttoController(const HardwareConfig& hw_config) {
        otto_.Init(
            hw_config.left_leg_pin, hw_config.right_leg_pin, hw_config.left_foot_pin, 
            hw_config.right_foot_pin, hw_config.left_hand_pin, hw_config.right_hand_pin
        );
        has_hands_ = (hw_config.left_hand_pin != GPIO_NUM_NC && hw_config.right_hand_pin != GPIO_NUM_NC);
        LoadTrimsFromNVS();
        action_queue_ = xQueueCreate(10, sizeof(OttoActionParams));
        QueueAction(ACTION_HOME, 1, 1000, 1, 0);
        RegisterMcpTools();
    }

    void RegisterMcpTools() {
        auto& mcp_server = McpServer::GetInstance();
        ESP_LOGI(TAG, "开始注册MCP工具...");

        // 【核心修复】补全所有动作的描述，并优化 AI 触发词
        mcp_server.AddTool("self.otto.action",
                           "执行机器人动作。基础动作：walk(走路/前进/后退，需direction)、turn(转弯/左转/右转，需direction)、jump(跳跃)、"
                           "swing(摇摆)、moonwalk(太空步)、bend(弯腰/侧弯)、shake_leg(抖腿)、updown(上下运动)、tiptoe_swing(踮脚摇摆)、"
                           "jitter(发抖/原地抖动)、ascending_turn(跳跃并转身)、crusaito(交叉步)、flapping(扑腾/拍打翅膀)、"
                           "whirlwind_leg(旋风腿)；固定动作：sit(坐下)、showcase(终极展示/全部技能展示)、home(复位/回中)；"
                           "手部动作：hands_up(举手)、hands_down(放手)、hand_wave(挥手/打招呼)、windmill(大风车/双臂画圈)、"
                           "takeoff(起飞/双臂高频摆动)、fitness(健身)、greeting(敬礼/打招呼)、shy(害羞捂脸)、"
                           "radio_calisthenics(广播体操/跳舞)、magic_circle(爱的魔力转圈圈)。",
                           PropertyList({
                               Property("action", kPropertyTypeString, "sit"),
                               Property("steps", kPropertyTypeInteger, 3, 1, 100),
                               Property("speed", kPropertyTypeInteger, 700, 100, 3000),
                               Property("direction", kPropertyTypeInteger, 1, -1, 1),
                               Property("amount", kPropertyTypeInteger, 30, 0, 170),
                               Property("arm_swing", kPropertyTypeInteger, 50, 0, 170)
                           }),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string action = properties["action"].value<std::string>();
                               int steps = properties["steps"].value<int>();
                               int speed = properties["speed"].value<int>();
                               int direction = properties["direction"].value<int>();
                               int amount = properties["amount"].value<int>();
                               int arm_swing = properties["arm_swing"].value<int>();

                               if (action == "walk") { QueueAction(ACTION_WALK, steps, speed, direction, arm_swing); return true; }
                               else if (action == "turn") { QueueAction(ACTION_TURN, steps, speed, direction, arm_swing); return true; }
                               else if (action == "jump") { QueueAction(ACTION_JUMP, steps, speed, 0, 0); return true; }
                               else if (action == "swing") { QueueAction(ACTION_SWING, steps, speed, 0, amount); return true; }
                               else if (action == "moonwalk") { QueueAction(ACTION_MOONWALK, steps, speed, direction, amount); return true; }
                               else if (action == "bend") { QueueAction(ACTION_BEND, steps, speed, direction, 0); return true; }
                               else if (action == "shake_leg") { QueueAction(ACTION_SHAKE_LEG, steps, speed, direction, 0); return true; }
                               else if (action == "updown") { QueueAction(ACTION_UPDOWN, steps, speed, 0, amount); return true; }
                               else if (action == "whirlwind_leg") { QueueAction(ACTION_WHIRLWIND_LEG, steps, speed, 0, amount); return true; }
                               
                               // 【补充漏掉的基础动作路由】
                               else if (action == "tiptoe_swing") { QueueAction(ACTION_TIPTOE_SWING, steps, speed, 0, amount); return true; }
                               else if (action == "jitter") { QueueAction(ACTION_JITTER, steps, speed, 0, amount); return true; }
                               else if (action == "ascending_turn") { QueueAction(ACTION_ASCENDING_TURN, steps, speed, 0, amount); return true; }
                               else if (action == "crusaito") { QueueAction(ACTION_CRUSAITO, steps, speed, direction, amount); return true; }
                               else if (action == "flapping") { QueueAction(ACTION_FLAPPING, steps, speed, direction, amount); return true; }

                               else if (action == "sit") { QueueAction(ACTION_SIT, 1, 0, 0, 0); return true; }
                               else if (action == "showcase") { QueueAction(ACTION_SHOWCASE, 1, 0, 0, 0); return true; }
                               else if (action == "home") { QueueAction(ACTION_HOME, 1, 1000, 1, 0); return true; }
                               
                               else if (action == "hands_up") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_HANDS_UP, 1, speed, direction, 0); return true; }
                               else if (action == "hands_down") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_HANDS_DOWN, 1, speed, direction, 0); return true; }
                               else if (action == "hand_wave") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_HAND_WAVE, 1, 0, 0, direction); return true; }
                               else if (action == "windmill") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_WINDMILL, steps, speed, 0, amount); return true; }
                               else if (action == "takeoff") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_TAKEOFF, steps, speed, 0, amount); return true; }
                               else if (action == "fitness") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_FITNESS, steps, speed, 0, amount); return true; }
                               else if (action == "greeting") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_GREETING, steps, 0, direction, 0); return true; }
                               else if (action == "shy") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_SHY, steps, 0, direction, 0); return true; }
                               else if (action == "radio_calisthenics") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_RADIO_CALISTHENICS, 1, 0, 0, 0); return true; }
                               else if (action == "magic_circle") { if (!has_hands_) return "错误：无手部舵机"; QueueAction(ACTION_MAGIC_CIRCLE, 1, 0, 0, 0); return true; }
                               else { return "错误：无效的动作名称。"; }
                           });

        // 舵机序列工具保持原样
        mcp_server.AddTool(
            "self.otto.servo_sequences",
            "AI自定义动作编程（即兴动作）。支持分段发送序列：超过5个序列建议AI可以连续多次调用此工具，每次发送一个短序列，系统会自动排队按顺序执行。支持普通移动和振荡器两种模式。"
            "机器人结构：双手可上下摆动，双腿可内收外展，双脚可上下翻转。"
            "舵机说明：ll(左腿)：0度=完全外展，90度=中立，180度=完全内收；rl(右腿)：0度=完全内收，90度=中立，180度=完全外展；lf(左脚)：0度=完全向上，90度=水平，180度=完全向下；rf(右脚)：0度=完全向下，90度=水平，180度=完全向上；lh(左手)：0度=完全向下，90度=水平，180度=完全向上；rh(右手)：0度=完全向上，90度=水平，180度=完全向下；"
            "sequence: 单个序列对象，包含'a'动作数组，顶层可选参数：'d'(序列执行完成后延迟毫秒数)。"
            "每个动作对象包含：普通模式：'s'舵机位置对象(键名：ll/rl/lf/rf/lh/rh，值：0-180度)，'v'移动速度100-3000毫秒(默认1000)，'d'动作后延迟毫秒数(默认0)；振荡模式：'osc'振荡器对象，包含'a'振幅对象(默认20度)，'o'中心角度对象(默认90度)，'ph'相位差对象(度)，'p'周期100-3000毫秒(默认500)，'c'周期数0.1-20.0(默认5.0)；"
            "普通模式示例：{\"sequence\":\"{\\\"a\\\":[{\\\"s\\\":{\\\"ll\\\":100},\\\"v\\\":1000}],\\\"d\\\":500}\"}",
            PropertyList({Property("sequence", kPropertyTypeString, "{\"a\":[{\"s\":{\"ll\":90,\"rl\":90},\"v\":1000}]}")}),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string sequence = properties["sequence"].value<std::string>();
                QueueServoSequence(sequence.c_str());
                return true;
            });

        mcp_server.AddTool("self.otto.stop", "立即停止所有动作并复位", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               if (action_task_handle_ != nullptr) {
                                   vTaskDelete(action_task_handle_);
                                   action_task_handle_ = nullptr;
                               }
                               is_action_in_progress_ = false;
                               PowerManager::ResumeBatteryUpdate();
                               xQueueReset(action_queue_);
                               QueueAction(ACTION_HOME, 1, 1000, 1, 0);
                               return true;
                           });

        mcp_server.AddTool("self.otto.set_trim", "校准单个舵机位置。servo_type: left_leg/right_leg/left_foot/right_foot/left_hand/right_hand; trim_value: -50到50", PropertyList({Property("servo_type", kPropertyTypeString, "left_leg"), Property("trim_value", kPropertyTypeInteger, 0, -50, 50)}),
                           [this](const PropertyList& properties) -> ReturnValue {
                               std::string servo_type = properties["servo_type"].value<std::string>();
                               int trim_value = properties["trim_value"].value<int>();
                               Settings settings("otto_trims", true);
                               int left_leg = settings.GetInt("left_leg", 0);
                               int right_leg = settings.GetInt("right_leg", 0);
                               int left_foot = settings.GetInt("left_foot", 0);
                               int right_foot = settings.GetInt("right_foot", 0);
                               int left_hand = settings.GetInt("left_hand", 0);
                               int right_hand = settings.GetInt("right_hand", 0);
                               if (servo_type == "left_leg") { left_leg = trim_value; settings.SetInt("left_leg", left_leg); }
                               else if (servo_type == "right_leg") { right_leg = trim_value; settings.SetInt("right_leg", right_leg); }
                               else if (servo_type == "left_foot") { left_foot = trim_value; settings.SetInt("left_foot", left_foot); }
                               else if (servo_type == "right_foot") { right_foot = trim_value; settings.SetInt("right_foot", right_foot); }
                               else if (servo_type == "left_hand") { left_hand = trim_value; settings.SetInt("left_hand", left_hand); }
                               else if (servo_type == "right_hand") { right_hand = trim_value; settings.SetInt("right_hand", right_hand); }
                               otto_.SetTrims(left_leg, right_leg, left_foot, right_foot, left_hand, right_hand);
                               QueueAction(ACTION_JUMP, 1, 500, 0, 0);
                               return "舵机微调设置已保存";
                           });

        mcp_server.AddTool("self.otto.get_trims", "获取当前的舵机微调设置", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               Settings settings("otto_trims", false);
                               std::string result = "{\"left_leg\":" + std::to_string(settings.GetInt("left_leg", 0)) + ",\"right_leg\":" + std::to_string(settings.GetInt("right_leg", 0)) + ",\"left_foot\":" + std::to_string(settings.GetInt("left_foot", 0)) + ",\"right_foot\":" + std::to_string(settings.GetInt("right_foot", 0)) + ",\"left_hand\":" + std::to_string(settings.GetInt("left_hand", 0)) + ",\"right_hand\":" + std::to_string(settings.GetInt("right_hand", 0)) + "}";
                               return result;
                           });

        mcp_server.AddTool("self.otto.get_status", "获取机器人状态，返回 moving 或 idle", PropertyList(),
                           [this](const PropertyList& properties) -> ReturnValue {
                               return is_action_in_progress_ ? "moving" : "idle";
                           });

        mcp_server.AddTool("self.battery.get_level", "获取机器人电池电量和充电状态", PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               auto& board = Board::GetInstance();
                               int level = 0;
                               bool charging = false;
                               bool discharging = false;
                               board.GetBatteryLevel(level, charging, discharging);
                               return "{\"level\":" + std::to_string(level) + ",\"charging\":" + (charging ? "true" : "false") + "}";
                           });
                           
        mcp_server.AddTool("self.otto.get_ip", "获取机器人WiFi IP地址", PropertyList(),
                           [](const PropertyList& properties) -> ReturnValue {
                               auto& wifi = WifiManager::GetInstance();
                               std::string ip = wifi.GetIpAddress();
                               if (ip.empty()) return "{\"ip\":\"\",\"connected\":false}";
                               return "{\"ip\":\"" + ip + "\",\"connected\":true}";
                           });                           

        ESP_LOGI(TAG, "MCP工具注册完成");
    }

    ~OttoController() {
        if (action_task_handle_ != nullptr) {
            vTaskDelete(action_task_handle_);
            action_task_handle_ = nullptr;
        }
        vQueueDelete(action_queue_);
    }
};

static OttoController* g_otto_controller = nullptr;

void InitializeOttoController(const HardwareConfig& hw_config) {
    if (g_otto_controller == nullptr) {
        g_otto_controller = new OttoController(hw_config);
        ESP_LOGI(TAG, "Otto控制器已初始化并注册MCP工具");
    }
}
