#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "sdkconfig.h"
#include "esp_timer.h"
#include "modbusslave.hh"
#include <vector>
#include <rgbled_rmt.hh>
#include <crgb.hh>
#include <driver/gpio.h>
#include <common.hh>
#include <array>


constexpr size_t LED_CNT{44};
constexpr size_t INPUTS_CNT{13};
constexpr uint64_t INPUTS_BITS{IO(39)|IO(41)|IO(42)|IO(40)|IO(2)|IO(1)|IO(13)|IO(15)|IO(16)|IO(14)|IO(17)|IO(18)|IO(0)};
constexpr std::array<uint8_t,INPUTS_CNT> INPUTS{39,41,42,40,2,1,13,15,16,14,17,18,0};
static const char *TAG = "MAIN";
static std::vector<bool> coilData(LED_CNT);
static std::vector<bool> inputsData(INPUTS_CNT);
static std::vector<uint16_t> registerData(LED_CNT);

modbus::M<10000> *modbusSlave;
RGBLED::M<LED_CNT, RGBLED::Timing::SK6805> *leds;

void tinyusb_cdc_rx_callback(int itf_i, cdcacm_event_t *event)
{
    tinyusb_cdcacm_itf_t itf = static_cast<tinyusb_cdcacm_itf_t>(itf_i);
    /* initialization */
    size_t rx_size = 0;
    size_t rx_size_max;
    size_t tx_size = 64;
    uint8_t tx_buf[256+1];
    uint8_t* rx_buf;
    modbusSlave->ReceiveBytesPhase1(&rx_buf, &rx_size_max);
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, rx_size_max, &rx_size);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Got a %d-Message with %d bytes", rx_buf[1], rx_size);
        modbusSlave->ReceiveBytesPhase2(rx_size, tx_buf, tx_size);
        if(tx_size!=0){
            tinyusb_cdcacm_write_queue(itf, tx_buf, tx_size);
            tinyusb_cdcacm_write_flush(itf, 0);
        }
    } else {
        ESP_LOGE(TAG, "Read error");
    }
}
static CRGB INDEX2COLOR[LED_CNT]={
    //Fußgänger links oben
    CRGB::Red, 
    CRGB::Yellow,
    CRGB::Green,
    //Verkehr oben
    CRGB::Red,
    CRGB::Yellow,
    CRGB::Green,
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    //fußgänger rechts oben
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    CRGB::Red,
    CRGB::Green,
    //Verkehr rechts
    CRGB::Red,
    CRGB::Yellow,
    CRGB::Green,
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    //Fußgänger rechts unten
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    CRGB::Red,
    CRGB::Green,
    //Verkehr unten
    CRGB::Red,
    CRGB::Yellow,
    CRGB::Green,
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    //Fußgänger links unten
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    CRGB::Red,
    CRGB::Green,
    //Verkehr links
    CRGB::Red,
    CRGB::Yellow,
    CRGB::Green,
    CRGB::Green,
    CRGB::Yellow,
    CRGB::Red,
    //Fußgänger links oben
    CRGB::Green,
    CRGB::Red,
};

void modbusCallback(uint8_t fc, uint16_t start, size_t len){
    ESP_LOGD(TAG, "Modbus Registers changed! fc:%d, start:%d len:%d. [%d %d %d %d]", fc, start, len, static_cast<int>(coilData[0]), static_cast<int>(coilData[1]),static_cast<int>(coilData[2]),static_cast<int>(coilData[3]));
    if(fc==15 || fc==5){
        for(int i=start;i<start+len;i++){
            leds->SetPixel(i, coilData.at(i)?INDEX2COLOR[i]:CRGB::Black);
        }
        leds->Refresh();
    }
    else if(fc==16 || fc==6){
        for(int i=start;i<start+len;i++){
            leds->SetPixel(i, CRGB::FromRGB565(registerData.at(i)));
        }
        leds->Refresh();
    }
}

extern "C" void app_main(void)
{
    //Query Inputs:      modpoll -r 1 -c 9 -t 1 COM8
    //Set Coil Outputs:  modpoll -r 1 -c 1 -t 0 COM8 1 bzw 0
    //Set LED Color      modpoll -r 1 -c 1 -t 4 COM8 65535
    modbusSlave = new modbus::M<10000>(1, modbusCallback, &coilData, &inputsData, nullptr, &registerData);
    //leds = new RGBLED::M<1, RGBLED::Timing::WS2812>();//Für Bastelplatine
    //ESP_ERROR_CHECK(leds->Init(RMT_CHANNEL_0, GPIO_NUM_48));Für Bastelplatine

    leds = new RGBLED::M<LED_CNT, RGBLED::Timing::SK6805>();
    ESP_ERROR_CHECK(leds->Init(RMT_CHANNEL_0, GPIO_NUM_38));
    const int lower=0;
    const int upper=44;


    for(int i=lower; i<upper;i++){
        ESP_ERROR_CHECK(leds->SetPixel(i, CRGB::Red));
    }
    ESP_LOGI(TAG, "White");
    leds->Refresh();
    vTaskDelay(pdMS_TO_TICKS(200));
    
    for(int i=lower; i<upper;i++){
        ESP_ERROR_CHECK(leds->SetPixel(i, CRGB::Yellow));
    }
    ESP_LOGI(TAG, "Red");
    leds->Refresh();
    vTaskDelay(pdMS_TO_TICKS(200));

    for(int i=lower; i<upper;i++){
        ESP_ERROR_CHECK(leds->SetPixel(i, CRGB::Green));
    }
    ESP_LOGI(TAG, "Green");
    leds->Refresh();
    vTaskDelay(pdMS_TO_TICKS(200));

    for(int i=lower; i<upper;i++){
        ESP_ERROR_CHECK(leds->SetPixel(i, CRGB::Black));
    }
    ESP_LOGI(TAG, "Black");
    leds->Refresh();
    vTaskDelay(pdMS_TO_TICKS(200));
        

    tinyusb_config_t tusb_cfg = {};
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    tinyusb_config_cdcacm_t amc_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = INPUTS_BITS;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Initialization DONE, Device Ready!");
    while(true){
        for (size_t i = 0; i < INPUTS.size(); i++) {
            inputsData.at(i) = gpio_get_level(static_cast<gpio_num_t>(INPUTS[i]))==0;
        }
        //ESP_LOGI(TAG, "0:%d 1:%d", (int)inputsData.at(0), (int)inputsData.at(1));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
