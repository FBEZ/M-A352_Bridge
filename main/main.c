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
    
    uint16_t version =0;
    M_A352__getFirmwareVersion(ma352, &version);
    printf("firmware version: %d\n", version);

    printf("MSC_CTRL: %04X\n", M_A352__getMSC_CTRL(ma352));
    if(M_A352__setSamplingPins(ma352,false,false,true,true)!=ESP_OK){
        printf("ERROR");
    }
    printf("MSC_CTRL: %04X\n", M_A352__getMSC_CTRL(ma352));

    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__gotoToSamplingMode(ma352));
    
    uint16_t count = 0;

    u_int32_t acc_receiver = 0;
    M_A352__getCount(ma352, &count);
    printf("Pre Count: %04X --  %d\n", count,count);
    printf("Temperature getTemp: %f\n", M_A352__getTemperature(ma352));
    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__getMeasurement(ma352, &acc_receiver, TEMP));
    printf("Temperature hex: %08lX\n", acc_receiver);

    printf("UART_CTRL: %04X\n",M_A352__getUART_CTRL(ma352));
    // 
    // M_A352__getMeasurement(ma352, &acc_receiver, ACC_X);
    // printf("Acc X : %f\n", ACC_CONV(acc_receiver));
    // M_A352__getMeasurement(ma352, &acc_receiver, ACC_Y);
    // printf("Acc Y : %f\n", ACC_CONV(acc_receiver));
    // M_A352__getMeasurement(ma352, &acc_receiver, ACC_Z);
    // printf("Acc Z : %f\n", ACC_CONV(acc_receiver));


    uint16_t sig_out =0;
    uint16_t burst_ctrl = 0;

    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__getBurstConfig(ma352,&burst_ctrl,&sig_out));
    M_A352__getCount(ma352, &count);
    printf("Post Count: %04X --  %d\n", count,count);
    printf("BURST_CTRL: %02X\n",burst_ctrl );
    printf("SIG_OUT: %02X\n",sig_out );

    M_A352__printSensorHeader(ma352);
    printf("Burst length%d\n", M_A352__getBurstLength(ma352));
    uint16_t return_array [M_A352__getBurstLength(ma352)];
    ESP_ERROR_CHECK_WITHOUT_ABORT(M_A352__readBurst(ma352, return_array));

    // int k =0;
    // for(k=0;k<M_A352__getBurstLength(ma352);k++){
    //     printf("%d: %04X\n",k,return_array[k]);
    // }
    M_A352__printSensorHeader(ma352);
    M_A352__printSensorData(ma352,return_array,1);
    while(1){
        vTaskDelay(1);
    }

    printf("Ciao mondo\n");

}
