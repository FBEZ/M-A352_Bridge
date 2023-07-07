#include <stdio.h>
//#include "esp_err.h"
#include <esp_timer.h>
#include "M_A352.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define M_A352_BAUDRATE 115200


M_A352_t* M_A352__create(uint8_t tx_pin,uint8_t rx_pin ,uint8_t uart_num, uint8_t rst_pin, uint8_t drdy_pin){
    M_A352_t* ret = (M_A352_t*) malloc(sizeof(M_A352_t));
    ret->tx_pin = tx_pin;
    ret->rx_pin = rx_pin;
    ret->uart_num = uart_num;
    ret->rst_pin = rst_pin;
    ret->drdy_pin = drdy_pin;
    return ret;
}

esp_err_t M_A352__begin(M_A352_t* ma352){
    uart_config_t uart_config = {
        .baud_rate = M_A352_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(ma352->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ma352->uart_num, ma352->tx_pin, ma352->rx_pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(ma352->uart_num, 1024, 1024, 0, NULL, 0));

    uint8_t test_str[] = {0x01, 0x02, 0x55};
    while (1) {
        printf("About to write\n");
        uart_write_bytes(ma352->uart_num, (const uint8_t*)test_str, 3);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


esp_err_t delayMicroseconds(uint32_t us)
{
    uint64_t m = (uint64_t)esp_timer_get_time();
    if(us){
        uint64_t e = (m + us);
        if(m > e){ //overflow
            while((uint64_t)esp_timer_get_time() > e){
                __asm__ __volatile__ ("nop");
            }
        }
        while((uint64_t)esp_timer_get_time() < e){
            __asm__ __volatile__ ("nop");
        }
    }

    return ESP_OK;
}