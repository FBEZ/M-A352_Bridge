#ifndef M_A352_H
#define M_A352_H

#include "esp_err.h"
#include "driver/uart.h"
#include "esp_log.h"


typedef struct{
 uint8_t tx_pin;
 uint8_t rx_pin;
 uint8_t uart_num;
 uint8_t rst_pin;
 uint8_t drdy_pin;
 bool initialised;  // Device not initialized yet
 bool uart_auto;  // UART_AUTO is disabled
} M_A352_t;

/**
 * @brief Creates an M-A352 object and defines the pins where it's attached to.
 * 
 * @param tx_pin UART TX pin   
 * @param rx_pin UART RX pin
 * @param uart_num UART number (1 or 2 in ESP32)
 * @param rst_pin Reset pin location. -1 if it's not used
 * @param drdy_pin Data ready pin location. -1 if it's not used
 * @return M_A352_t* 
 */
M_A352_t* M_A352__create(uint8_t tx_pin,uint8_t rx_pin ,uint8_t uart_num, uint8_t rst_pin, uint8_t drdy_pin);
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
 * @brief Reads a burst 
 * 
 * @param ma352 M_A352 handle pointer
 * @param return_array 
 * @param burst_length
 * @return esp_err_t 
 */
esp_err_t M_A352__readBurst(M_A352_t* ma352, uint16_t* return_array, uint8_t burst_length);

/**
 * @brief Utility function for delay because the standard one wasn't uploaded correctly
 * 
 * @param us 
 * @return esp_err_t 
 */
esp_err_t delayMicroseconds(uint32_t us);

#endif /*M_A352_H*/