#include <driver/i2c_master.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <new>
#include "cJSON.h"

#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "display/lcd_display.h"
#include "esp32_camera.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "otto_emoji_display.h" // 动态表情
#include "power_manager.h"
#include "system_reset.h"
#include "websocket_control_server.h"
#include "wifi_board.h"

extern "C" {
#include "otto_pca_adapter.h"
}

#define TAG "OttoRobot"
extern void InitializeOttoController(const HardwareConfig& hw_config);

class OttoRobot : public WifiBoard {
private:
    LcdDisplay* display_;
    PowerManager* power_manager_;
    Button boot_button_;
    WebSocketControlServer* ws_control_server_;
    HardwareConfig hw_config_;
    AudioCodec* audio_codec_;
    
    // 闹钟定时器句柄
    esp_timer_handle_t alarm_timer_ = nullptr;

    void InitializePowerManager() {
        power_manager_ = new PowerManager(hw_config_.power_charge_detect_pin,
                                          hw_config_.power_adc_unit, hw_config_.power_adc_channel);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = hw_config_.display_mosi_pin;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = hw_config_.display_clk_pin;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = hw_config_.display_cs_pin;
        io_config.dc_gpio_num = hw_config_.display_dc_pin;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = hw_config_.display_rst_pin;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        // 恢复动态表情
        display_ = new OttoEmojiDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                        DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                        DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    // 核心防崩：延时独立任务初始化舵机
    static void OttoInitTask(void* arg) {
        OttoRobot* self = static_cast<OttoRobot*>(arg);
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        otto_pca_init(); 
        ESP_LOGI(TAG, ">>> 舵机恢复至 90 度 <<<");
        for (int i = 0; i < 6; i++) {
            otto_pca_set_servo_angle(i, 90);
            vTaskDelay(pdMS_TO_TICKS(50)); 
        }
        ::InitializeOttoController(self->hw_config_); 
        vTaskDelete(NULL);
    }

    // 闹钟触发的招手任务
    static void AlarmWaveTask(void* arg) {
        ESP_LOGW(TAG, "⏰ 主人，时间到了！执行招手动作...");
        // 左手(4)和右手(5)交替摆动
        for (int i = 0; i < 3; i++) {
            otto_pca_set_servo_angle(4, 160); // 左手抬起
            otto_pca_set_servo_angle(5, 20);  // 右手抬起
            vTaskDelay(pdMS_TO_TICKS(300));
            otto_pca_set_servo_angle(4, 45);  // 左手放下
            otto_pca_set_servo_angle(5, 135); // 右手放下
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        // 恢复中位
        otto_pca_set_servo_angle(4, 90);
        otto_pca_set_servo_angle(5, 90);
        vTaskDelete(NULL);
    }

    // 注册 MCP 工具（闹钟与关机）
    void InitializeMcpTools() {
        auto& mcp_server = McpServer::GetInstance();

        // 1. 闹钟工具
        mcp_server.AddTool(
            "self.set_alarm",
            "Set an alarm for a specific number of seconds. When the alarm rings, the robot will wave.",
            PropertyList({
                Property("seconds", kPropertyTypeInteger, 10, 1, 86400)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int seconds = properties["seconds"].value<int>();
                if (seconds <= 0) seconds = 10; 

                if (alarm_timer_ != nullptr) {
                    esp_timer_stop(alarm_timer_);
                    esp_timer_delete(alarm_timer_);
                    alarm_timer_ = nullptr;
                }

                esp_timer_create_args_t timer_args = {
                    .callback = [](void* arg) {
                        // 闹钟回调中不能阻塞，必须启动独立任务
                        xTaskCreate(AlarmWaveTask, "alarm_wave", 2048, nullptr, 3, nullptr);
                        
                        // 尝试触发语音提示（播放一段系统内置音效）
                        auto& app = Application::GetInstance();
                        app.Schedule([&app]() {
                            // 你可以在这里添加让系统主动 TTS 的逻辑，但为了防崩，这里只发声
                            // app.PlaySound(Lang::Sounds::OGG_LOW_BATTERY); 
                        });
                    },
                    .arg = nullptr, 
                    .dispatch_method = ESP_TIMER_TASK, 
                    .name = "otto_alarm"
                };

                if (esp_timer_create(&timer_args, &alarm_timer_) == ESP_OK) {
                    esp_timer_start_once(alarm_timer_, seconds * 1000000ULL);
                    ESP_LOGI(TAG, "⏰ 闹钟已设置，%d 秒后响铃并招手", seconds);
                    return true;
                }
                return false;
            }
        );

        // 2. 关机工具
        mcp_server.AddTool(
            "self.shutdown",
            "Shutdown the robot. Turns off screen and servos, then enters deep sleep. (关机)",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                ESP_LOGW(TAG, "🔌 收到关机指令，准备进入深度睡眠...");
                pca9685_all_off();
                
                GetBacklight()->SetBrightness(0);
                gpio_set_level(hw_config_.display_backlight_pin, 0);
                gpio_hold_en(hw_config_.display_backlight_pin);

                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_deep_sleep_start();
                return true;
            }
        );
    }

public:
    OttoRobot() : boot_button_(BOOT_BUTTON_GPIO), audio_codec_(nullptr) {
        hw_config_ = NON_CAMERA_VERSION_CONFIG;
        
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializePowerManager();
        
        if (hw_config_.audio_use_simplex) {
            audio_codec_ = new NoAudioCodecSimplex(
                hw_config_.audio_input_sample_rate, hw_config_.audio_output_sample_rate,
                hw_config_.audio_i2s_spk_gpio_bclk, hw_config_.audio_i2s_spk_gpio_lrck,
                hw_config_.audio_i2s_spk_gpio_dout, hw_config_.audio_i2s_mic_gpio_sck,
                hw_config_.audio_i2s_mic_gpio_ws, hw_config_.audio_i2s_mic_gpio_din);
        } else {
            audio_codec_ = new NoAudioCodecDuplex(
                hw_config_.audio_input_sample_rate, hw_config_.audio_output_sample_rate,
                hw_config_.audio_i2s_gpio_bclk, hw_config_.audio_i2s_gpio_ws,
                hw_config_.audio_i2s_gpio_dout, hw_config_.audio_i2s_gpio_din);
        }

        // 在 OttoInitTask 创建处，修改为：
        xTaskCreatePinnedToCore(OttoInitTask, "otto_init", 4096, this, 6, NULL, 0);
        InitializeMcpTools();

        ws_control_server_ = nullptr;
        GetBacklight()->RestoreBrightness();
    }

    virtual AudioCodec* GetAudioCodec() override { return audio_codec_; }
    virtual Display* GetDisplay() override { return display_; }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight* backlight = nullptr;
        if (backlight == nullptr) {
            backlight = new PwmBacklight(hw_config_.display_backlight_pin, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        }
        return backlight;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        level = 100; charging = true; discharging = false; return true;
    }

    virtual Camera* GetCamera() override { return nullptr; }

    virtual std::string GetDeviceStatusJson() override {
        std::string baseJson = WifiBoard::GetDeviceStatusJson();
        cJSON* root = cJSON_Parse(baseJson.c_str());
        if (root != nullptr) {
            cJSON_DeleteItemFromObject(root, "chip_name");
            cJSON_AddStringToObject(root, "chip_name", "ESP32-S3-N16R8");
            cJSON_DeleteItemFromObject(root, "screen_type");
            cJSON_AddStringToObject(root, "screen_type", "ST7789");
            cJSON_DeleteItemFromObject(root, "emoji_collection"); 
            
            char* newJson = cJSON_PrintUnformatted(root);
            std::string result(newJson);
            cJSON_free(newJson);
            cJSON_Delete(root);
            return result;
        }
        return baseJson;
    }
};
DECLARE_BOARD(OttoRobot);
