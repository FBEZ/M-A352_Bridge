#include <stdio.h>
#include <string.h>
#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "M_A352.h"

#define CONFIG_UART_NUM UART_NUM_2
#define CONFIG_TX_IO 16//5
#define CONFIG_RX_IO 4//18
#define CONFIG_RST_IO 2//23
#define CONFIG_EXT_IO 1//21
#define CONFIG_DRDY_IO 13//22

void app_main(void)
{

    M_A352_t* ma352 = M_A352__create(CONFIG_TX_IO, CONFIG_RX_IO,CONFIG_UART_NUM,CONFIG_RST_IO,CONFIG_DRDY_IO,CONFIG_EXT_IO);
    
    ESP_ERROR_CHECK(M_A352__begin(ma352));
    ESP_ERROR_CHECK(M_A352__setSamplingPins(ma352,false,false,true,true));
    ESP_ERROR_CHECK(M_A352__setBurstConfiguration(ma352,true,true,true,true,true,true,false));
    ESP_ERROR_CHECK(M_A352__setSamplingRate(ma352, SAMPLING_RATE_1000));


    ESP_ERROR_CHECK(M_A352__gotoToSamplingMode(ma352));
    
    measurement_set_t data_array[16];
    const uint8_t DATA_POINT_LENGTH = 16;
    uint8_t k = 0;

    for(k=0;k<DATA_POINT_LENGTH;k++){
        data_array[k] = M_A352__readMeasurementSet(ma352, false);
    }
    for(k=0;k<DATA_POINT_LENGTH;k++){
        M_A352__printMeasurmentSet(data_array[k]);
    }
    for(k=0;k<DATA_POINT_LENGTH;k++){
       printf("%d: %d",k,data_array[k].counter);
    }
    
    
    while(1){
        vTaskDelay(1);
    }


}
