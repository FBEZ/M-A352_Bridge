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
#define CONFIG_TX_IO 5
#define CONFIG_RX_IO 18
#define CONFIG_RST_IO 23
#define CONFIG_EXT_IO 21
#define CONFIG_DRDY_IO 22

void app_main(void)
{

    M_A352_t* ma352 = M_A352__create(CONFIG_TX_IO, CONFIG_RX_IO,CONFIG_UART_NUM,CONFIG_RST_IO,CONFIG_DRDY_IO,-1);
    ESP_ERROR_CHECK(M_A352__begin(ma352));
    printf("Init complete\n");
    
    //uint16_t version =0;
    //M_A352__getFirmwareVersion(ma352, &version);
    //printf("firmware version: %d\n", version);

    //printf("MSC_CTRL: %04X\n", M_A352__getMSC_CTRL(ma352));
    if(M_A352__setSamplingPins(ma352,false,false,true,true)!=ESP_OK){
        printf("ERROR");
    }
   // printf("MSC_CTRL: %04X\n", M_A352__getMSC_CTRL(ma352));
    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__setBurstConfiguration(ma352,true,true,true,true,true,true,false));
    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__setSamplingRate(ma352, SAMPLING_RATE_1000));
    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__gotoToSamplingMode(ma352));
    
    M_A352__printSensorHeader(ma352);


    //uint16_t sig_out =0;
    //uint16_t burst_ctrl = 0;

    //ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__getBurstConfig(ma352,&burst_ctrl,&sig_out));
    //printf("Burst config: %04X\n", burst_ctrl);
    //printf("Sig OUT: %04X\n", sig_out);

    //printf("Burst length: %d\n", M_A352__getBurstLength(ma352));
    //uint16_t return_array [M_A352__getBurstLength(ma352)];

    //printf("Status Mode: %04lX\n", M_A352__getStatusMode(ma352)); 
    //printf("Temperature: %f\n", M_A352__getTemperature(ma352));
    //printf("Misura: %f\n", M_A352__getAccelerationZ(ma352));

    printf("SMPL: %04X\n", M_A352__getSMPL_CTRL(ma352));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__readBurst(ma352, return_array));

    M_A352__printSensorHeader(ma352);
    //M_A352__printSensorData(ma352,return_array,1);

    measurement_set_t data_array[16];
    const uint8_t DATA_POINT_LENGTH = 16;
    uint8_t k = 0;
    for(k=0;k<DATA_POINT_LENGTH;k++){
        data_array[k] = M_A352__readMeasurementSet(ma352);
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

    printf("Ciao mondo\n");

}
