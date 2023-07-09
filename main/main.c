#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "M_A352.h"

#define CONFIG_UART_NUM UART_NUM_2
#define CONFIG_TX_IO 5
#define CONFIG_RX_IO 18
#define CONFIG_RST_IO 23
#define CONFIG_EXT_IO 21
#define CONIFG_DRDY_IO 22


void app_main(void)
{

    M_A352_t* ma352 = M_A352__create(CONFIG_TX_IO, CONFIG_RX_IO,CONFIG_UART_NUM,CONFIG_RST_IO,-1);
    ESP_ERROR_CHECK(M_A352__begin(ma352));
    printf("Init complete\n");
    
    uint16_t version =0;
    M_A352__getFirmwareVersion(ma352, &version);
    printf("firmware version: %d\n", version);

    M_A352__gotoToSamplingMode(ma352);
    
    const uint8_t burst_length = 128;
    uint16_t sample_values[burst_length];
    M_A352__readBurst(ma352,sample_values,burst_length);

    uint8_t k = 0;
    for(k=0;k<burst_length;k++){
        printf("%d : %02X\n", k%23, sample_values[k]);
    }


    while(1){
        vTaskDelay(1);
    }

 
    // while (1) {
    //     printf("About to write\n");
    //     uart_write_bytes(CONFIG_UART_NUM, (const uint8_t*)test_str, 2);
    //     delayMicroseconds(delay);
    //     uart_write_bytes(CONFIG_UART_NUM, (const uint8_t*)test_str, 2);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    printf("Ciao mondo\n");

}
