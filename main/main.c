#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "M-A352.h"

#define CONFIG_UART_NUM UART_NUM_2
#define CONFIG_TX_IO 5
#define CONFIG_RX_IO 18

void app_main(void)
{

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_UART_NUM, CONFIG_TX_IO, CONFIG_RX_IO, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0));

    uint8_t test_str[] = {0x01, 0x02};
    uint32_t delay = 174;
    while (1) {
        printf("About to write\n");
        uart_write_bytes(CONFIG_UART_NUM, (const uint8_t*)test_str, 2);
        delayMicroseconds(delay);
        uart_write_bytes(CONFIG_UART_NUM, (const uint8_t*)test_str, 2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    printf("Ciao mondo\n");

}
