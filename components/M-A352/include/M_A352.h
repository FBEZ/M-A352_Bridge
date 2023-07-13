#ifndef M_A352_H
#define M_A352_H

#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "M_A352_definitions.h"
#include <sdkconfig.h>


#define MSC_CTRL_EXT_SEL  0x40
#define MSC_CTRL_EXT_POL  0x20
#define MSC_CTRL_DRDY_SEL 0x04
#define MSC_CTRL_DRDY_POL 0x02

#define MAX_RETRIES_NUM 10000

#define TEMP_CONV(X) (-0.0037918*(double)(X)+34.987)
#define ACC_CONV(X)  (EPSON_ACCL_SF*((float)(X)))

typedef enum{
    TEMP   = ADDR_TEMP_LOW,
    ACC_X  = ADDR_XACCL_LOW,
    ACC_Y  = ADDR_YACCL_LOW,
    ACC_Z  = ADDR_ZACCL_LOW,
    TILT_X = ADDR_XTILT_LOW,
    TILT_Y = ADDR_YTILT_LOW,
    TILT_Z = ADDR_ZTILT_LOW
}measurement_t;

typedef enum{
    UNDEFINED,
    CONFIGURATION_MODE,
    SAMPLING_MODE
}status_mode_t;

typedef struct M_A352_t M_A352_t;


/**
 * @brief Creates an M-A352 object and defines the pins where it's attached to.
 * 
 * @param tx_pin UART TX pin   
 * @param rx_pin UART RX pin
 * @param uart_num UART number (1 or 2 in ESP32)
 * @param rst_pin Reset pin location. -1 if it's not used
 * @param drdy_pin Data ready pin location. -1 if it's not used
 * @param ext_pin External trigger pin location. -1 if it's not used
 * @return M_A352_t* 
 */
M_A352_t* M_A352__create(uint8_t tx_pin,uint8_t rx_pin ,uint8_t uart_num, int8_t rst_pin, int8_t drdy_pin, int8_t ext_pin);

/**
 * @brief Starts the UART connection with the previously created sensor
 * 
 * @param ma352 the structure with the connection pins
 * @return esp_err_t 
 */
esp_err_t M_A352__begin(M_A352_t* ma352);

/**
 * @brief Return product ID as stored in the sensor - used as check
 * 
 * @param ma352 ma352 handle
 * @param product_id pointer where to return the string (size: 9)
 * @return esp_err_t 
 */
esp_err_t M_A352__getProductID(M_A352_t* ma352, char* product_id);

/**
 * @brief Puts the M-A352 into sampling mode
 * 
 * @param ma352 M_A352 handle pointer
 * @return esp_err_t 
 */
esp_err_t M_A352__gotoToSamplingMode(M_A352_t* ma352);

/**
 * @brief Puts the M-A352 into configuration mode
 * 
 * @param ma352 M_A352 handle pointer
 * @return esp_err_t 
 */

esp_err_t M_A352__gotoToConfigMode(M_A352_t* ma352);

/**
 * @brief Return the firmware version: MUST be in configuration mode 
 * 
 * @param ma352 M_A352 handle pointer
 * @param return_value firmware version result
 * @return esp_err_t 
 */
esp_err_t M_A352__getFirmwareVersion(M_A352_t* ma352, uint16_t* return_value);


/**
 * @brief Reads the burst configuration
 * 
 * @param ma352 
 * @param burst_config the receiving parameter for the burst register 
 * @param sig_out the SIG_OUT register 
 * @return esp_err_t 
 */
esp_err_t M_A352__getBurstConfig(M_A352_t* ma352, uint16_t* burst_config, uint16_t* sig_out);

/**
 * @brief Get temperature
 * 
 * @param ma352 handle pointer
 * @param temp_receiver 32bit temperature receiver
 * @return esp_err_t 
 */
float M_A352__getTemperature(M_A352_t* ma352);

/**
 * @brief Returns acceleration of x-axis (G)
 * 
 * @param ma352 pointer handle
 * @return float 
 */
float M_A352__getAccelerationX(M_A352_t* ma352);

/**
 * @brief Returns acceleration of y-axis (G)
 * 
 * @param ma352 pointer handle
 * @return float 
 */
float M_A352__getAccelerationY(M_A352_t* ma352);
/**
 * @brief Returns acceleration of z-axis (G)
 * 
 * @param ma352 pointer handle
 * @return float 
 */
float M_A352__getAccelerationZ(M_A352_t* ma352);

/**
 * @brief Get a single measurement (temperature, one of the accelerations or tilt)
 * 
 * @param ma352 pointer handle
 * @param meas_receiver measurement receiving variable 
 * @param meas the measurement required
 * @return esp_err_t 
 */
esp_err_t M_A352__getMeasurement(M_A352_t* ma352, u_int32_t* meas_receiver, measurement_t meas);

/**
 * @brief Read the count register
 * 
 * @param ma352 handle pointer
 * @param count_receiver receving parameter for the count
 * @return esp_err_t 
 */
esp_err_t M_A352__getCount(M_A352_t* ma352, uint16_t* count_receiver);


/**
 * @brief Reads a burst 
 * 
 * @param ma352 M_A352 handle pointer
 * @param return_array 
 * @param burst_length
 * @return esp_err_t 
 */
esp_err_t M_A352__readBurst(M_A352_t* ma352, uint16_t* return_array);

/**
 * @brief Returns the data header depending on the enabled burst flags
 * 
 * @param ma352 pointer handle
 */
void M_A352__printSensorHeader(M_A352_t* ma352);

/**
 * @brief Formats and print the burst data
 * 
 * @param ma352 pointer handle
 * @param data the data from a bust
 * @param sampleCount sample number
 */
void M_A352__printSensorData(M_A352_t* ma352, uint16_t* data, uint32_t sampleCount);


/**
 * @brief Get MSC_CTRL address values
 * 
 * @param ma352 
 * @return uint16_t 
 */
uint16_t M_A352__getMSC_CTRL(M_A352_t* ma352);

/**
 * @brief Set the sampling pin enable and polarity
 * 
 * @param ma352 
 * @param external_trigger_enable 
 * @param external_trigger_polarity 
 * @param data_ready_enable 
 * @param data_ready_polarity 
 * @return esp_err_t 
 */
esp_err_t M_A352__setSamplingPins(M_A352_t* ma352, bool external_trigger_enable,bool external_trigger_polarity,bool data_ready_enable, bool data_ready_polarity);

/**
 * @brief Returns burst length, useful to prepare the return array
 * 
 * @param ma352 
 * @return uint16_t 
 */
uint16_t M_A352__getBurstLength(M_A352_t* ma352);

/**
 * @brief Read the uart ctrl register
 * 
 * @param ma352 
 * @return uint16_t 
 */
uint16_t M_A352__getUART_CTRL(M_A352_t* ma352);

/**
 * @brief Utility function for delay because the standard one wasn't uploaded correctly
 * 
 * @param us 
 * @return esp_err_t 
 */
esp_err_t delayMicroseconds(uint32_t us);

#endif /*M_A352_H*/