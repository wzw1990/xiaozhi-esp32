#include "wifi_board.h"
#include "audio_codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "iot/thing_manager.h"
#include "led/single_led.h"
#include "esp32_camera.h"
#include "ds18b20.h"
#include "ds18b20_types.h"
#include "onewire_bus_impl_rmt.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
 
#define TAG "WenBoard"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);


class DS18B20Controller {
private:
    float tempC = 0.0; // 当前温度，单位摄氏度
    ds18b20_device_handle_t ds18b20_ = nullptr;
    gpio_num_t gpio_num_;

    void DetectDS18B20() {
        // install 1-wire bus
        onewire_bus_handle_t bus = NULL;
        onewire_bus_config_t bus_config = {
            .bus_gpio_num = gpio_num_,
        };
        onewire_bus_rmt_config_t rmt_config = {
            .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
        };
        ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

        onewire_device_iter_handle_t iter = NULL;
        onewire_device_t next_onewire_device;
        esp_err_t search_result = ESP_OK;

        // create 1-wire device iterator, which is used for device search
        ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
        ESP_LOGI(TAG, "Device iterator created, start searching...");
        do {
            search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
            if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
                ds18b20_config_t ds_cfg = {};
                // check if the device is a DS18B20, if so, return the ds18b20 handle
                if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20_) == ESP_OK) {
                    ESP_LOGI(TAG, "Found a DS18B20, address: %016llX", next_onewire_device.address);
                    break;
                } else {
                    ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
                }
            }
        } while (search_result != ESP_ERR_NOT_FOUND);
        ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    }


public:
    DS18B20Controller(gpio_num_t gpio_num) : gpio_num_(gpio_num) {
        DetectDS18B20();
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.sensor.get_temperature", "Get the current ambient temperature in Celsius", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            if (ds18b20_) {
                ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20_));
                ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20_, &tempC));
                ESP_LOGI(TAG, "temperature read from DS18B20: %.2fC", tempC);
                return "{\"tempC\":\"" + std::to_string(tempC) + "\"}";
            } else {
                return "{\"tempC\": \"unavailable\"}";
            }
        });
    }
};

class WenBoard : public WifiBoard {
private:
 
    Button boot_button_;
    LcdDisplay* display_;
     Esp32Camera* camera_;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));        
        esp_lcd_panel_reset(panel);
 

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_16_4,
                                        .icon_font = &font_awesome_16_4,
#if CONFIG_USE_WECHAT_MESSAGE_STYLE
                                        .emoji_font = font_emoji_32_init(),
#else
                                        .emoji_font = DISPLAY_HEIGHT >= 240 ? font_emoji_64_init() : font_emoji_32_init(),
#endif
                                    });
    }

    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;  
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
#if CONFIG_IOT_PROTOCOL_MCP
        static DS18B20Controller ds18b20(DS18B20_GPIO);
#endif
    }

public:
    WenBoard() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeIot();
        // InitializeCamera();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

DECLARE_BOARD(WenBoard);
