#include <stdio.h>
#include <string.h>
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
    
    // TODO: check UART_AUTO status

    // Configure RST pin
    gpio_set_direction(ma352->rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(ma352->rst_pin, 1);

    M_A352__HWReset(ma352);
    ESP_LOGI(TAG,"Sensor reset");

    char* model_name = malloc(9);
    M_A352__getProductID(ma352,model_name);

    //printf("Model name: %s\n", model_name);
    if(strcmp(model_name, "A352AD10")==0){
        return ESP_OK;
    }
    else{
        return ESP_ERR_INVALID_RESPONSE;
    }
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
 * @brief Write an 8 bit register
 * 
 * @param ma352 M_A352 handle pointer
 * @param window_id register window id
 * @param address register address
 * @param value value to be written
 * @param verbose if true, outputs the register value (to be removed)
 * @return esp_err_t 
 */
esp_err_t M_A352__writeRegister8Byte(M_A352_t* ma352, uint8_t window_id, uint8_t address, uint8_t value, bool verbose) {

    uint8_t xmtVal[3];

    // Send the window command & win ID
    xmtVal[0] = ADDR_WIN_CTRL|0x80;  // msb is set 1b for register write
    xmtVal[1] = window_id;
    xmtVal[2] = DELIMITER;
    uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
    uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    // Delay between commands
    EpsonStall();

    // Send the write register command & address
    xmtVal[0] = address|0x80;  // msb is set 1b for register write
    xmtVal[1] = value;
    xmtVal[2] = DELIMITER;
    uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
    uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    // Delay between commands
    EpsonStall();

    // TODO handle with a debug flag
    if (verbose) {
            printf("REG[0x");
            printf("%x",(address&0x7F));
            printf(" W(");
            printf("%d",window_id);
            printf(")");
            printf("] > 0x");
            printf("%04X\n",(unsigned int)(value));
        }

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
            printf(")");
            printf("] > 0x");
            printf("%04X\n",(unsigned int)(*return_value));
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
    ESP_ERROR_CHECK(M_A352__readRegister16Bytes(ma352, &return_value, CMD_WINDOW1, ADDR_GLOB_CMD_LO, false));
    if ((return_value & VAL_NOT_READY)== 0){
        //_burstCnt_calculated = 0;
        return ESP_OK;
    }
    return ESP_ERR_INVALID_RESPONSE;


}

esp_err_t M_A352__getProductID(M_A352_t* ma352, char* product_id){
    uint8_t i =0;
    uint16_t return_value = 0;
    esp_err_t ret_err=ESP_OK;
    //read model name from registers, stored as ascii values
    for (i=0;i<8;i=i+2){
        ret_err = M_A352__readRegister16Bytes(ma352, &return_value, CMD_WINDOW1, ADDR_PROD_ID1 + i, false);
        product_id[i] = (char) (return_value & 0xFF);
        product_id[i + 1] = (char) (return_value>>8);
    }
    product_id[i] = 0x00;  // add NULL terminator to make it a string
    return ret_err;
}


esp_err_t M_A352__gotoToSamplingMode(M_A352_t* ma352) {
    M_A352__writeRegister8Byte(ma352, CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING,false);
    delayMicroseconds(2000); // // TO-DO - If this delay is removed, it
                                // does not reliably enter sampling mode
    return ESP_OK;
}


esp_err_t M_A352__gotoToConfigMode(M_A352_t* ma352) {
    M_A352__writeRegister8Byte(ma352, CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING,false);
    return ESP_OK;
}

esp_err_t M_A352__getFirmwareVersion(M_A352_t* ma352, uint16_t* return_value) {
    return M_A352__readRegister16Bytes(ma352, return_value ,CMD_WINDOW1, ADDR_VERSION, false);
}

esp_err_t M_A352__readNSequentialRegisters (M_A352_t* ma352, uint16_t* arrayOut, uint8_t address, uint8_t read_length, uint32_t retries_max_count) {

    uint8_t retByte[128];  // Burst length should never exceed 128 bytes
    uint8_t readByteLength = (read_length * 2) + 2;  // (16-bit length * 2) + header byte + delimiter byte
    uint8_t j = 0;
    uint32_t retryCount = 0;

    // If DRDY is used then assume UART manual mode, and send a burst command every sample
    if (ma352->drdy_pin != -1) {
        uint8_t xmtVal[3];
        // Setup read burst command
        xmtVal[0] = address;     // This should be the BURST COMMAND (0x20 for V340 or 0x80 for others)
        xmtVal[1] = 0x00;
        xmtVal[2] = DELIMITER;
        uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
        uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);

        // delay after 1st command
        EpsonBurstStall();
    }

    // Wait for buffer to fill to 1 complete burst or return false on exceeding max retries
    int len_read = uart_read_bytes(ma352->uart_num, retByte, readByteLength, retries_max_count / portTICK_PERIOD_MS);
    
    if(len_read<readByteLength){
        return ESP_ERR_TIMEOUT;
    }
    // do {
    //   retryCount++;
    //   delayMicroseconds(100);
    //   if (retryCount > retries_max_count) {
    //     return ESP_ERR_TIMEOUT;
    //   }
    // } while (SerialEpson.available() < readByteLength);

    // Read UART and store into byte array
    //int bytesRead = SerialEpson.readBytes(retByte, readByteLength);

    // Return false if # of bytes is incorrect
    // if (bytesRead != readByteLength) {
    //     SerialConsole.print("Incorrect bytes read:");
    //     SerialConsole.println(bytesRead, DEC);
    //     findDelimiter();     // read sensor Rx buffer until finds DELIMITER
    //     return false;
    // }
    // Return false if first byte & last byte is incorrect
    if ((retByte[0] != (address)) || (retByte[readByteLength - 1] != DELIMITER)) {
        printf("Unexpected read response:");
        for(int i=0; i<readByteLength; i++) {
            printf("%02X",retByte[i]);
            printf(", ");
        }
        printf("\n");
       //findDelimiter();     // read sensor Rx buffer until finds DELIMITER
        return ESP_ERR_TIMEOUT;
    }

    // Parse UART read byte array to 16-bit output array
    for (int i = 1; i < (readByteLength-1); i = i+2) {
        arrayOut[j] = retByte[i]<<8 | retByte[i+1];
        j++;
    }
    return ESP_OK;
}


esp_err_t M_A352__readBurst(M_A352_t* ma352, uint16_t* return_array, uint8_t burst_length) {

    const uint32_t retries = 10000;
    
    // if burst count flag is nonzero, overwrite len parameter
    //TODO!!!!!!!!!!!!!!!!!
    //if (_burstCnt_calculated)
    //    len = _burstCnt_calculated>>1;

    // if DRDY is used then wait for DRDY = HIGH
    //if (ma352->drdy_pin != -1)
    //    while (!waitDataReady(true, 100000));

    // Read burst sensor data
    esp_err_t retval = M_A352__readNSequentialRegisters(ma352, return_array, CMD_BURST, burst_length, retries);

    // if DRDY is used then wait for DRDY = LOW
    // if (_drdy != -1)
    //     while (!waitDataReady(false, 1000));

    return retval;
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