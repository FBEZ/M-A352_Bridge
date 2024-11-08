#include <stdio.h>
#include <string.h>
//#include "esp_err.h"
#include <esp_timer.h>
#include "M_A352.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"



char* TAG = "M-A352";
esp_err_t M_A352__HWReset(M_A352_t* ma352);
esp_err_t M_A352__setBurstFlags(M_A352_t* ma352);
uint16_t M_A352__setMode(M_A352_t* ma352);
bool M_A352__waitDataReady(M_A352_t* ma352, bool asserted);
void M_A352__goToWindow(M_A352_t* ma352, uint8_t window_id);
uint16_t M_A352__readFilterSetting(M_A352_t* ma352);

typedef struct{
      // 0 = Off, 1 = On
      bool nd_ea;
      bool tempc;
      bool acclx;
      bool accly;
      bool acclz;
      bool inclx;
      bool incly;
      bool inclz;
      bool count;
      bool chksm;
    }  burst_flags_t;

typedef struct{
      // 0 = Off, 1 = On
    bool external_trigger_enable;
    bool external_trigger_polarity;
    bool data_ready_enable;
    bool data_ready_polarity;
} sampling_pins_config_t;

struct M_A352_t{
 uint8_t tx_pin;
 uint8_t rx_pin;
 uint8_t uart_num;
 int8_t rst_pin;
 int8_t drdy_pin;
 int8_t ext_pin;
 uint8_t burst_length;
 burst_flags_t flags;
 sampling_pins_config_t sampling_pins_conf;
 filter_setting_t filter_setting;
 sampling_rate_t sampling_rate;
 mode_t status_mode;
 bool initialised;  // Device not initialized yet
 bool uart_auto;  // UART_AUTO is disabled
 uint8_t window_id;
} ;


M_A352_t* M_A352__create(uint8_t tx_pin,uint8_t rx_pin ,uint8_t uart_num, int8_t rst_pin, int8_t drdy_pin, int8_t ext_pin){
    M_A352_t* ret = (M_A352_t*) malloc(sizeof(M_A352_t));
    ret->tx_pin = tx_pin;
    ret->rx_pin = rx_pin;
    ret->uart_num = uart_num;
    ret->rst_pin = rst_pin;
    ret->drdy_pin = drdy_pin;
    ret->ext_pin = ext_pin;
    ret->status_mode = UNDEFINED;
    ret->initialised = false;
    return ret;
}

esp_err_t M_A352__begin(M_A352_t* ma352){
    uart_config_t uart_config = {
        .baud_rate = DEFAULT_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    printf("tx: %d\trx: %d",ma352->tx_pin,ma352->rx_pin);
    ESP_ERROR_CHECK(uart_param_config(ma352->uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ma352->uart_num, ma352->tx_pin, ma352->rx_pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(ma352->uart_num, 1024, 1024, 0, NULL, 0));
    
    // TODO: check UART_AUTO status

    // Configure RST pin
    gpio_set_direction(ma352->rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(ma352->rst_pin, 1);

    // Configure DRDY if used
    if(ma352->drdy_pin!=-1){
        gpio_set_direction(ma352->drdy_pin, GPIO_MODE_INPUT);
        
    }
    // Configure External trigger if used
    if(ma352->ext_pin!=-1){
        gpio_set_direction(ma352->ext_pin, GPIO_MODE_INPUT);
    }


    M_A352__HWReset(ma352);
    ESP_LOGI(TAG,"Sensor reset"); 

    M_A352__goToWindow(ma352,0); // set a define starting widow (and internal status)
    M_A352__gotoToConfigMode(ma352); // be sure to be in confuration mode
    M_A352__setBurstFlags(ma352);
    M_A352__setSamplingRate(ma352,DEFAULT_SAMPLING_FREQUENCY); //set default value and set internal variable
    //filter_setting_t filter = {.tap = DEFAULT_FILTER_TAP, .fc = DEFAULT_FILTER_FC };
    M_A352__setFilterSetting(ma352,(filter_setting_t){.tap = DEFAULT_FILTER_TAP, .fc = DEFAULT_FILTER_FC });
    char* model_name = malloc(9);
    M_A352__getProductID(ma352,model_name);
    

    //printf("Model name: %s\n", model_name);
    if(strcmp(model_name, "A352AD10")==0){
        ma352->initialised = true;
        return ESP_OK;
    }
    else{
        return ESP_ERR_INVALID_RESPONSE;
    }
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

    if(ma352->status_mode==SAMPLING_MODE && (address != ADDR_MODE_CTRL_LO || address!=ADDR_GLOB_CMD_LO)){
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t xmtVal[3];
    if(window_id != ma352->window_id){
        M_A352__goToWindow(ma352,window_id);
        // //Send the window command & win ID
        // xmtVal[0] = ADDR_WIN_CTRL|0x80;  // msb is set 1b for register write
        // xmtVal[1] = window_id;
        // xmtVal[2] = DELIMITER;
        // uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
        // uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
        // //Delay between commands
        EpsonStall();
        //printf("Changed Window Write! (addr: %04X)\n", address);
    }

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

    if(window_id!=ma352->window_id){
        M_A352__goToWindow(ma352,window_id);
        // // Send the window command & win ID
        // xmtVal[0] = ADDR_WIN_CTRL|0x80; // msb is set 1b for register write
        // xmtVal[1] = window_id;
        // xmtVal[2] = DELIMITER;
        // uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
        // uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
        // // Delay between commands
        EpsonStall();
        //printf("Changed Window Read! (addr: %04X)\n", address);
    }

    // Send the read register command & address
    xmtVal[0] = address&0x7f; // msb is set 0b for register read
    xmtVal[1] = 0x00;  // Dummy byte
    xmtVal[2] = DELIMITER;
    uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
    uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    // Delay between commands
    EpsonStall();
    int len_read = uart_read_bytes(ma352->uart_num, retByte, 4, MAX_RETRIES_NUM);

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
 * @brief Move to window and change internal state
 * 
 * @return esp_err_t 
 */
void M_A352__goToWindow(M_A352_t* ma352, uint8_t window_id){
    uint8_t xmtVal[3];
    // Send the window command & win ID
    xmtVal[0] = ADDR_WIN_CTRL|0x80;  // msb is set 1b for register write
    xmtVal[1] = window_id;
    xmtVal[2] = DELIMITER;
    uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
    uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
    
    ma352->window_id = window_id;
    // Delay between commands
    EpsonStall();
    //printf("Changed Window, now: %d\n", window_id);
    
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
    M_A352__goToWindow(ma352,1); //added because this function is called at start up and here the read register is called
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
    ma352->status_mode = SAMPLING_MODE;
    delayMicroseconds(2000); // // TO-DO - If this delay is removed, it
                                // does not reliably enter sampling mode
    return ESP_OK;
}


esp_err_t M_A352__gotoToConfigMode(M_A352_t* ma352) {
    M_A352__writeRegister8Byte(ma352, CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING,false);
    ma352->status_mode = CONFIGURATION_MODE;
    return ESP_OK;
}

mode_t M_A352__getStatusMode(M_A352_t* ma352){
    return ma352->status_mode;
}

sampling_rate_t M_A352__getSMPL_CTRL(M_A352_t* ma352){
    uint16_t register_value = 0;
    sampling_rate_t ret = SAMPLING_RATE_UNDEFINED;
    M_A352__readRegister16Bytes(ma352,&register_value,CMD_WINDOW1,ADDR_SMPL_CTRL_LO,false);
    ret = register_value;
    // switch(ret){
    //     case 0x02:
    //         ret = SAMPLING_RATE_1000;
    //         break;
    //     case 0x03:
    //         ret = SAMPLING_RATE_500;
    //         break;
    //     case 0x04:
    //         ret = SAMPLING_RATE_200;
    //         break;
    //     case 0x05:
    //         ret = SAMPLING_RATE_100;
    //         break;
    //     case 0x06:
    //         ret = SAMPLING_RATE_50;
    //         break;
    // }
    return ret;
}

esp_err_t M_A352__getFirmwareVersion(M_A352_t* ma352, uint16_t* return_value) {
    return M_A352__readRegister16Bytes(ma352, return_value ,CMD_WINDOW1, ADDR_VERSION, false);
}

esp_err_t M_A352__getBurstConfig(M_A352_t* ma352, uint16_t* burst_config, uint16_t* sig_out){
    esp_err_t err_burst = M_A352__readRegister16Bytes(ma352, burst_config ,CMD_WINDOW1, ADDR_BURST_CTRL_LO, false);
    esp_err_t err_sig_ctrl = M_A352__readRegister16Bytes(ma352, sig_out ,CMD_WINDOW1, ADDR_SIG_CTRL_LO, false);
    if(err_burst == ESP_OK && err_sig_ctrl == ESP_OK){
        return ESP_OK;
    }else{
        return ESP_ERR_TIMEOUT;
    }    
}

/**
 * @brief Set burst flags and burst_length
 * 
 * @param ma352 
 * @return esp_err_t 
 */
esp_err_t M_A352__setBurstFlags(M_A352_t* ma352){
    uint16_t sig_ctrl = 0;
    uint16_t burst_ctrl = 0;
    esp_err_t ret = M_A352__readRegister16Bytes(ma352,&sig_ctrl, CMD_WINDOW1, ADDR_SIG_CTRL_LO,false);
    M_A352__readRegister16Bytes(ma352,&burst_ctrl, CMD_WINDOW1, ADDR_BURST_CTRL_LO,false);

          // burst_ctrl check
      ma352->flags.nd_ea = (burst_ctrl&0x8000) ? 1 : 0;
      ma352->flags.tempc = (burst_ctrl&0x4000) ? 1 : 0;
      ma352->flags.acclx = (burst_ctrl&0x400) ? 1 : 0;
      ma352->flags.accly = (burst_ctrl&0x200) ? 1 : 0;
      ma352->flags.acclz = (burst_ctrl&0x100) ? 1 : 0;
      ma352->flags.count = (burst_ctrl&0x2) ? 1 : 0;
      ma352->flags.chksm = (burst_ctrl&0x1) ? 1 : 0;
      // sig_ctrl check
      ma352->flags.inclx = (sig_ctrl&0x80) ? 1 : 0;
      ma352->flags.incly = (sig_ctrl&0x40) ? 1 : 0;
      ma352->flags.inclz = (sig_ctrl&0x20) ? 1 : 0;

      ma352->burst_length = 0;
      if (ma352->flags.nd_ea) ma352->burst_length += 2;
      if (ma352->flags.tempc) ma352->burst_length += 4;
      if (ma352->flags.acclx) ma352->burst_length += 4;
      if (ma352->flags.accly) ma352->burst_length += 4;
      if (ma352->flags.acclz) ma352->burst_length += 4;
      if (ma352->flags.count) ma352->burst_length += 2;
      if (ma352->flags.chksm) ma352->burst_length += 2;

      return ret;
}


esp_err_t M_A352__readNSequentialRegisters (M_A352_t* ma352, uint16_t* arrayOut, uint8_t address, uint8_t read_length) {

    uint8_t retByte[128];  // Burst length should never exceed 128 bytes
    //uint8_t readByteLength = (read_length * 2) + 2;  // (16-bit length * 2) + header byte + delimiter byte
    uint8_t readByteLength = ma352->burst_length+2;
    if(ma352->window_id!=0){ // the burst command is on window 0 
        M_A352__goToWindow(ma352,0);
        printf("Moved to Window 0 for bust\n");
    }
    // If DRDY is used then assume UART manual mode, and send a burst command
    if (ma352->drdy_pin != -1) {
        //M_A352__writeRegister8Byte(ma352,CMD_WINDOW0,CMD_BURST,0x0,false);
        uint8_t xmtVal[3];
        // Setup read burst command
        xmtVal[0] = address;     // This should be the BURST COMMAND (0x20 for V340 or 0x80 for others)
        xmtVal[1] = 0x00;
        xmtVal[2] = DELIMITER;
        uart_write_bytes(ma352->uart_num, (const uint8_t*)xmtVal, 3);
        uart_wait_tx_done(ma352->uart_num, portMAX_DELAY);
        //delay after 1st command
        EpsonBurstStall();
    }
    //printf("Prima di len_read\n");
    // Wait for buffer to fill to 1 complete burst or return false on exceeding max retries
    int len_read = uart_read_bytes(ma352->uart_num, retByte, readByteLength, 1000);
    
   //printf("Prima test lungezza: len_read %d\t readByteLength: %d\n",len_read, readByteLength);
    if(len_read<readByteLength){
        return ESP_ERR_TIMEOUT;
    }
    //printf("Passato test lungezza\n");
    if ((retByte[0] != (address)) || (retByte[readByteLength - 1] != DELIMITER)) {
        printf("Unexpected read response:");
        for(int i=0; i<readByteLength; i++) {
            printf("%02X",retByte[i]);
            printf(", ");
        }
       // printf("\n");
        return ESP_ERR_TIMEOUT;
    }

    // Parse UART read byte array to 16-bit output array
    uint8_t j = 0;
    for (int i = 1; i < (readByteLength-1); i = i+2) {
        arrayOut[j] = retByte[i]<<8 | retByte[i+1];
        j++;
    }
    return ESP_OK;
}


esp_err_t M_A352__readBurst(M_A352_t* ma352, uint16_t* return_array) {

    if (ma352->drdy_pin != -1)
        while (!M_A352__waitDataReady(ma352,true)); //waiting for asserted dataready 
    //printf("Asserted!\n");
    // Read burst sensor data
    esp_err_t retval = M_A352__readNSequentialRegisters(ma352, return_array, CMD_BURST, ma352->burst_length);
    
    //if DRDY is used then wait for DRDY = LOW
    // if (ma352->drdy_pin != -1)
    //    while (!M_A352__waitDataReady(ma352,false));
    // printf("Passata secondo data ready\n");
    return retval;
}

esp_err_t M_A352__readTriggeredBurst(M_A352_t* ma352, uint16_t* return_array) {

    if(ma352->ext_pin == -1 || ma352->drdy_pin == -1)
        return ESP_ERR_INVALID_STATE; // you can't call it without set ext_pin and drdy_pin
    //printf("ext_pin to:%d", ma352->sampling_pins_conf.external_trigger_polarity);
    gpio_set_level(ma352->ext_pin, ma352->sampling_pins_conf.external_trigger_polarity);
    gpio_set_level(ma352->ext_pin, !ma352->sampling_pins_conf.external_trigger_polarity);
    //printf("ext_pin to:%d", !ma352->sampling_pins_conf.external_trigger_polarity);
    while (!M_A352__waitDataReady(ma352,true)); //waiting for asserted dataready 
    //printf("DRDY Asserted!\n");
    
    // Read burst sensor data
    esp_err_t retval = M_A352__readNSequentialRegisters(ma352, return_array, CMD_BURST, ma352->burst_length);
    
    return retval;
}

esp_err_t M_A352__setBurstConfiguration(M_A352_t* ma352, bool flag_out, bool temp_out, bool acceleration_x_out, bool acceleration_y_out, bool acceleration_z_out, bool count_out, bool checksum_out){
    uint8_t burst_ctrl_low = checksum_out*0x01+
                             count_out   *0x02;

    uint8_t burst_ctrl_high = acceleration_z_out*0x01+
                              acceleration_y_out*0x02+
                              acceleration_x_out*0x04+
                              temp_out*          0x40+
                              flag_out*0x80;

    esp_err_t ret = M_A352__writeRegister8Byte(ma352,CMD_WINDOW1, ADDR_BURST_CTRL_LO,burst_ctrl_low,false);
    M_A352__writeRegister8Byte(ma352,CMD_WINDOW1, ADDR_BURST_CTRL_HI,burst_ctrl_high,false);
    M_A352__setBurstFlags(ma352);
    return ret;
}

uint16_t M_A352__getBurstLength(M_A352_t* ma352){
    return ma352->burst_length;
}



measurement_set_t M_A352__readMeasurementSet(M_A352_t* ma352, bool triggered){
    
    uint16_t data [M_A352__getBurstLength(ma352)];
    if(triggered){
        M_A352__readTriggeredBurst(ma352,data);
    }else{
        M_A352__readBurst(ma352, data);
    }
    measurement_set_t ret;

    int idx = 0;

    if (ma352->flags.nd_ea) {idx += 1;} //the nd flags are ignored

    if (ma352->flags.tempc) { // if enabled temperature
        int32_t temp = (data[idx]<<16) | (data[idx+1]<<0);
        ret.temperature = ((float)temp*EPSON_TEMP_SF) + 34.987f;
        idx += 2;
    }else{
        ret.temperature = -1.0;
    }

    if(ma352->flags.acclx) {
        int32_t x = (data[idx]<<16) | (data[idx+1]<<0);
        if(ma352->flags.inclx){
          ret.tilt_x = (EPSON_TILT_SF * (float)x); //< tilt
          ret.acceleration_x = -1.0;
        }
        else{
          ret.acceleration_x = (EPSON_ACCL_SF * (float)x); //< acceleration
          ret.tilt_x = -1.0;
        }
        idx += 2;
    }else{
        ret.acceleration_x = -1.0;
        ret.tilt_x = -1.0;
    }

    if(ma352->flags.accly) {
       int32_t y = (data[idx]<<16) | (data[idx+1]<<0);
       if(ma352->flags.incly){
         ret.tilt_y = (EPSON_TILT_SF * (float)y); //< tilt
         ret.acceleration_y = -1.0;
       }
       else{
         ret.acceleration_y = (EPSON_ACCL_SF * (float)y); //< acceleration
         ret.tilt_y = -1.0;
       }
       idx += 2;
     }else{
        ret.acceleration_y = -1.0;
        ret.tilt_y = -1.0;
    }

      if(ma352->flags.acclz) {
        int32_t z = (data[idx]<<16) | (data[idx+1]<<0);
        if(ma352->flags.inclz){
          ret.tilt_z = (EPSON_TILT_SF * (float)z); //< tilt
          ret.acceleration_z = -1.0;
        }
        else{
          ret.acceleration_z = (EPSON_ACCL_SF * (float)z); //< acceleration
          ret.tilt_z = -1.0;
        }
        idx += 2;
      }else{
        ret.acceleration_z = -1.0;
        ret.tilt_z = -1.0;
    }

      if(ma352->flags.count) {
        ret.counter = data[idx];
        idx += 1;
      }else{
        ret.counter = 0;
      }
    //checksum not used
    //   if(ma352->flags.chksm) {
    //     printf("%04X\t",data[idx]);
    //   }

    return ret;

}

//debug only
void M_A352__printMeasurmentSet(measurement_set_t meas){
    printf("Temperature   : %f\n", meas.temperature);
    printf("Acceleration X: %f\n", meas.acceleration_x);
    printf("Acceleration Y: %f\n", meas.acceleration_y);
    printf("Acceleration Z: %f\n", meas.acceleration_z);
    printf("Tilt X        : %f\n", meas.tilt_x);
    printf("Tilt Y        : %f\n", meas.tilt_y);
    printf("Tilt Z        : %f\n", meas.tilt_z);
    printf("Counter       : %d\n", meas.counter);
    printf("\n");
}

esp_err_t M_A352__getCount(M_A352_t* ma352, uint16_t* count_receiver){
    return M_A352__readRegister16Bytes(ma352, count_receiver ,CMD_WINDOW0, ADDR_COUNT, false);
}

esp_err_t M_A352__setSamplingRate(M_A352_t* ma352, sampling_rate_t sampling_rate){
    uint8_t s_rate = sampling_rate >> 8; // the sampling_rate_t is 0x0400, the last byte is always 0
    printf("s_rate %04X\n", s_rate);
    esp_err_t err = M_A352__writeRegister8Byte(ma352, CMD_WINDOW1, ADDR_SMPL_CTRL_HI, s_rate, true);
    
    sampling_rate_t check = M_A352__getSMPL_CTRL(ma352);
    printf("check: %04X\t sampling_rate: %04X\n", check, sampling_rate);
    if(check == sampling_rate && err == ESP_OK){
        ma352->sampling_rate = sampling_rate;
        return ESP_OK;
    }else{
        return ESP_ERR_INVALID_RESPONSE;
    }
    
}

sampling_rate_t M_A352__getSamplingRate(M_A352_t* ma352){
    return ma352->sampling_rate;
}

esp_err_t M_A352__getMeasurement(M_A352_t* ma352, u_int32_t* meas_receiver, measurement_t meas){
    uint16_t temp_low = 0;
    uint16_t temp_high = 0;
    esp_err_t err_low = M_A352__readRegister16Bytes(ma352, &temp_low ,CMD_WINDOW0, meas, false);
    esp_err_t err_high = M_A352__readRegister16Bytes(ma352, &temp_high ,CMD_WINDOW0, meas-2, false);
    //printf("temp low: %04X\n", temp_low);
    //printf("temp high: %04X\n", temp_high);
    *meas_receiver = (temp_high<<8) | (temp_low);
    //printf("temp tot: %ldn\n", *temp_receiver);
    if(err_low==ESP_OK && err_high == ESP_OK){
        return ESP_OK;
    }else{
        return err_low;
    }
}


float M_A352__getTemperature(M_A352_t* ma352){
    uint32_t temp_reg = 0;
    M_A352__getMeasurement(ma352, &temp_reg, TEMP);
    return TEMP_CONV((int32_t)temp_reg);
}

float M_A352__getAccelerationX(M_A352_t* ma352){
    uint32_t temp_reg = 0;
    M_A352__getMeasurement(ma352, &temp_reg, ADDR_XACCL_LOW);
    return (float)ACC_CONV((int32_t) temp_reg);
}
float M_A352__getAccelerationY(M_A352_t* ma352){
    uint32_t temp_reg = 0;
    M_A352__getMeasurement(ma352, &temp_reg, ADDR_YACCL_LOW);
    return (float)ACC_CONV((int32_t) temp_reg);
}
float M_A352__getAccelerationZ(M_A352_t* ma352){
    uint32_t temp_reg = 0;
    M_A352__getMeasurement(ma352, &temp_reg, ADDR_ZACCL_LOW);
    return (float)ACC_CONV((int32_t) temp_reg);
}


void M_A352__printSensorHeader(M_A352_t* ma352){

      printf("\nSample#\t");
      if (ma352->flags.nd_ea) {
        printf("ND_EA\t");
      }

      if (ma352->flags.tempc) {
        printf("TempC\t\t");
      }

      if(ma352->flags.acclx) {
        if(ma352->flags.inclx) printf("Tilt X\t");
        else                printf("Accl X\t");
      }

      if(ma352->flags.accly) {
        if(ma352->flags.incly) printf("Tilt Y\t");
        else                printf("Accl Y\t");
      }

      if(ma352->flags.acclz) {
        if(ma352->flags.inclz) printf("Tilt Z\t");
        else                printf("Accl Z\t");
      }

      if(ma352->flags.count) {
        printf("Count\t");
      }

      if(ma352->flags.chksm) {
        printf("Checksum");
      }

      printf("\n");
}

uint16_t M_A352__getMSC_CTRL(M_A352_t* ma352){
    uint16_t return_value = 0;
    M_A352__readRegister16Bytes(ma352,&return_value,CMD_WINDOW1,ADDR_MSC_CTRL_LO, false);
    return return_value;
}

uint16_t M_A352__getUART_CTRL(M_A352_t* ma352){
    uint16_t return_value = 0;
    M_A352__readRegister16Bytes(ma352,&return_value,CMD_WINDOW1,ADDR_UART_CTRL_LO, false);
    return return_value;
}

esp_err_t M_A352__setSamplingPins(M_A352_t* ma352, bool external_trigger_enable,bool external_trigger_polarity,bool data_ready_enable, bool data_ready_polarity){
    uint8_t value = 0;
    value += (data_ready_polarity       ? MSC_CTRL_DRDY_POL:0);
    value += (data_ready_enable         ? MSC_CTRL_DRDY_SEL:0);
    value += (external_trigger_enable   ? MSC_CTRL_EXT_SEL :0);
    value += (external_trigger_polarity ? 0:MSC_CTRL_EXT_POL); // EXT_POL = 1 is negative logic, hence it's swapped
    
    // Setting internal variable
    ma352->sampling_pins_conf.external_trigger_enable=external_trigger_enable;
    ma352->sampling_pins_conf.external_trigger_polarity = external_trigger_polarity;
    ma352->sampling_pins_conf.data_ready_enable = data_ready_enable;
    ma352->sampling_pins_conf.data_ready_polarity = data_ready_polarity;

    return M_A352__writeRegister8Byte(ma352,CMD_WINDOW1,ADDR_MSC_CTRL_LO,value,false);
}

//TODO!!!
uint16_t M_A352__setMode(M_A352_t* ma352){
    uint16_t return_value=0;
    M_A352__readRegister16Bytes(ma352,&return_value,CMD_WINDOW0,ADDR_MODE_CTRL_LO,false);
    return return_value;
}



 void M_A352__printSensorData(M_A352_t* ma352, uint16_t* data, uint32_t sampleCount) {

      printf("%ld\t",sampleCount);

      // stores the accelerometer data array index when parsing out data fields
      int idx = 0;

      // parsing of data fields applying conversion factor if applicable
      if (ma352->flags.nd_ea) {
        //process ND flag data
        unsigned short ndflags = data[idx];
        idx += 1;
        printf("%02X\t",ndflags);
      }

      if (ma352->flags.tempc) {
        //process temperature data
        int32_t temp = (data[idx]<<16) | (data[idx+1]<<0);
        float temperature = ((float)temp*EPSON_TEMP_SF) + 34.987f;
        idx += 2;
        printf("%6f\t",temperature);
      }

      if(ma352->flags.acclx) {
        //process x axis data
        float accel_x;
        int32_t x = (data[idx]<<16) | (data[idx+1]<<0);
        if(ma352->flags.inclx)
          accel_x = (EPSON_TILT_SF * (float)x); //< tilt
        else
          accel_x = (EPSON_ACCL_SF * (float)x); //< acceleration
        printf("%6f\t",accel_x);
        idx += 2;
      }

      if(ma352->flags.accly) {
        //process y axis data
        float accel_y;
        int32_t y = (data[idx]<<16) | (data[idx+1]<<0);
        if(ma352->flags.incly)
          accel_y = (EPSON_TILT_SF * (float)y); //< tilt
        else
          accel_y = (EPSON_ACCL_SF * (float)y); //< acceleration
        printf("%6f\t",accel_y);
        idx += 2;
      }

      if(ma352->flags.acclz) {
        //process z axis data
        float accel_z;
        int32_t z = (data[idx]<<16) | (data[idx+1]<<0);
        if(ma352->flags.inclz)
          accel_z = (EPSON_TILT_SF * (float)z); //< tilt
        else
          accel_z = (EPSON_ACCL_SF * (float)z); //< acceleration
        printf("%6f\t",accel_z);
        idx += 2;
      }

      if(ma352->flags.count) {
        //process count out data
        printf("%d\t",data[idx]);
        idx += 1;
      }

      if(ma352->flags.chksm) {
        // process checksum data
        printf("%04X\t",data[idx]);
      }
      printf("\n");
}

/**
 * @brief Waiting for data ready to be asserted (positive or negative depending on the configuration)
 * 
 * @param ma352 
 * @return esp_err_t 
 */
bool M_A352__waitDataReady(M_A352_t* ma352, bool activated) {
    
    uint32_t retryCount = 0;
    bool condition_value = (ma352->sampling_pins_conf.data_ready_polarity==activated); //asserted is true when positive polarity and false if negative polarity
    //printf("Polarita: %d\t Input: %d \t condition_value: %d\n",ma352->sampling_pins_conf.data_ready_polarity, activated,condition_value );
    // Loop continuously to check the status of DataReady until Low or timeout
    do {
        retryCount++;
        delayMicroseconds(10);     // 10 usec
    } while ((gpio_get_level(ma352->drdy_pin)!= condition_value) & (retryCount < MAX_RETRIES_NUM));
     //printf("***************\n");
    // return true on success, or fail for a timeout
    if (retryCount < MAX_RETRIES_NUM)
        return true; // success
    else
        return false; // fail
}


esp_err_t M_A352__setFilterSetting(M_A352_t* ma352, filter_setting_t filter_setting){
    printf("FILTER: fc:%d\ttap:%d\n",filter_setting.fc, filter_setting.tap);
    uint8_t cmd = 0;
    if(filter_setting.tap == 64){

        if(filter_setting.fc == 83){
            cmd = CMD_FIRTAP64FC83;
        }else if(filter_setting.fc==220){
            cmd = CMD_FIRTAP64FC220;
        }else{
            return ESP_ERR_INVALID_ARG;
        }

    }else if(filter_setting.tap == 128){

        if(filter_setting.fc == 110){
            cmd = CMD_FIRTAP128FC110;
        }else if(filter_setting.fc==350){
            cmd = CMD_FIRTAP128FC350;
        }else if(filter_setting.fc==36){
            cmd = CMD_FIRTAP128FC36;
        }else{
            return ESP_ERR_INVALID_ARG;
        }

    }else if(filter_setting.tap==512){

        if(filter_setting.fc == 9){
            cmd = CMD_FIRTAP512FC9;
        }else if(filter_setting.fc==16){
            cmd = CMD_FIRTAP512FC16;
        }else if(filter_setting.fc==60){
            cmd = CMD_FIRTAP512FC60;
        }else if(filter_setting.fc==210){
            cmd = CMD_FIRTAP512FC210;
        }else if(filter_setting.fc==460){
            cmd = CMD_FIRTAP512FC460;
        }else{
            return ESP_ERR_INVALID_ARG;
        }

    }else{
        return ESP_ERR_INVALID_ARG;
    }

    M_A352__writeRegister8Byte(ma352,CMD_WINDOW1, ADDR_FILTER_CTRL_LO,cmd,false);

    if((M_A352__readFilterSetting(ma352)& 0x0F)==cmd){ // only lower bits are useful
        ma352->filter_setting.fc = filter_setting.fc;
        ma352->filter_setting.tap = filter_setting.tap;
        return ESP_OK;
    }else{
        return ESP_ERR_INVALID_RESPONSE;
    }
}

filter_setting_t M_A352__getFilterSetting(M_A352_t* ma352){
    return ma352->filter_setting;
}

uint16_t M_A352__readFilterSetting(M_A352_t* ma352){
    uint16_t ret = 0;
    M_A352__readRegister16Bytes(ma352,&ret,CMD_WINDOW1,ADDR_FILTER_CTRL_LO,false);
    return ret;
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