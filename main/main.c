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
#define CONFIG_GPIO_PIN GPIO_NUM_21

void app_main(void)
{

    M_A352_t* handle = M_A352__create(CONFIG_TX_IO, CONFIG_RX_IO,CONFIG_UART_NUM,-1,-1);
    printf("handle tx gpio %d\n",handle->tx_pin);
    M_A352__begin(handle);

    gpio_set_direction(CONFIG_GPIO_PIN, GPIO_MODE_OUTPUT);
    uint32_t delay = 197;
    while(1){
        gpio_set_level(CONFIG_GPIO_PIN,0);
        delayMicroseconds(delay);
        gpio_set_level(CONFIG_GPIO_PIN,1);
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
