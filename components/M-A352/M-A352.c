#include <stdio.h>
//#include "esp_err.h"
#include <esp_timer.h>
#include "M-A352.h"


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