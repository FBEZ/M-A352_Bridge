#include <stdio.h>
//#include "esp_err.h"
#include <esp_timer.h>
#include "M_A352.h"
#include "M_A352_definitions.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define M_A352_BAUDRATE 460800

char* TAG = "M-A352";
esp_err_t M_A352__HWReset(M_A352_t* ma352);

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
    
    // Configure RST pin
    gpio_set_direction(ma352->rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(ma352->rst_pin, 1);

    M_A352__HWReset(ma352);
    ESP_LOGI(TAG,"Sensor reset");

    // uint8_t test_str[] = {0x01, 0x02, 0x55};
    // while (1) {
    //     printf("About to write\n");
    //     uart_write_bytes(ma352->uart_num, (const uint8_t*)test_str, 3);
    //     uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    //     M_A352__HWReset(ma352);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    return ESP_OK;
}


/**
 * @brief Reads a 16 byte register from the M_A352
 * 
 * @param ma352 M_A352 handle
 * @param return_value pointer for the result reception
 * @param window_id Register window id 
 * @param address Register address
 * @param verbose If logging is required
 * @return esp_err_t
 */
esp_err_t M_A352__readRegister16Bytes(M_A352_t* ma352, uint16_t* return_value, uint8_t window_id, uint8_t address, bool verbose){
    
    uint8_t retByte[4];
    uint8_t xmtVal[3];

    // Send the window command & win ID
    xmtVal[0] = ADDR_WIN_CTRL|0x80; // msb is set 1b for register write
    xmtVal[1] = window_id;
    xmtVal[2] = DELIMITER;
    uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
    uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    // Delay between commands
    EpsonStall();

    // Send the read register command & address
    xmtVal[0] = address&0x7f; // msb is set 0b for register read
    xmtVal[1] = 0x00;  // Dummy byte
    xmtVal[2] = DELIMITER;
    uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
    uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    // Delay between commands
    EpsonStall();
    int len_read = uart_read_bytes(ma352->uart_num, retByte, 4, 40 / portTICK_PERIOD_MS);

    // Check that a respose has arrived
    if(len_read<4){
        return ESP_ERR_TIMEOUT;
    }
    // Delay between 16 bit transfers
    EpsonStall();

    // Check first byte in the response should be the register address
    // Check last byte in the response should be DELIMITER
    if ((retByte[0] != (address&0x7f)) || (retByte[3] != DELIMITER)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *return_value = (uint16_t)(retByte[1]<<8) | (retByte[2]);

        // TODO: remove and handle as debug
        if (verbose) {
            printf("REG[0x");
            printf("%x",(address&0x7F));
            printf(" W(");
            printf("%d",window_id);
            printf(")\n");
            printf("] > 0x");
            printf("%x\n",(unsigned int)return_value);
        }

    
    return ESP_OK;
}


/**
 * @brief Resets the sensor to start from a known state
 * 
 * @return esp_err_t 
 */
esp_err_t M_A352__HWReset(M_A352_t* ma352){
    gpio_set_level(ma352->rst_pin,0);
    EpsonResetAssertDelay();
    gpio_set_level(ma352->rst_pin, 1);
    // Wait for the sensor re-initialization
    EpsonPowerOnDelay();
    
    // Check NOT_READY bit = 0
    uint16_t return_value= 0;
    ESP_ERROR_CHECK(M_A352__readRegister16Bytes(ma352, &return_value, CMD_WINDOW1, ADDR_GLOB_CMD_LO, true));
    if ((return_value & VAL_NOT_READY)== 0){
        //_burstCnt_calculated = 0;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_RESPONSE;


}

esp_err_t M_A352__getProductID(M_A352_t* ma352, char* product_id){
    uint8_t i;
    uint16_t return_value = 0;
    //read model name from registers, stored as ascii values
    for (i = 0; i < 8; i = i + 2) {
        ESP_ERROR_CHECK(M_A352__readRegister16Bytes(ma352, &return_value, CMD_WINDOW1, ADDR_PROD_ID1 + i, false));
        product_id[i] = (char) (return_value & 0xFF);
        product_id[i + 1] = (char) (return_value>>8);
    }
    product_id[i] = 0x00;  // add NULL terminator to make it a string
    return ESP_OK;
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