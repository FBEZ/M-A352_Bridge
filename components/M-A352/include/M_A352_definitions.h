/**************************************************************************/
/*!
    @file     accel_epson_M-A352.h
    @license  BSD (see license.txt)

    Epson M-A352AD Accelerometer specific definitions

    @section  HISTORY

    v1.0 - First release
    v1.1 - Remove legacy, add new device models, refactor

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2019-2022 Seiko Epson Corporation.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
    OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#pragma once
#ifndef M_A352_DEF_H_
#define M_A352_DEF_H_

#include <stdio.h>
#include "M_A352_config.h"

#ifdef EPSON_SELF_TEST_DELAY
#undef EPSON_SELF_TEST_DELAY
#endif
#define EPSON_SELF_TEST_DELAY       (200000)    //< 200 ms

#define EPSON_ACCL_SF          (0.00000006f)    //   G/LSB
#define EPSON_TEMP_SF          (-0.0037918f)    //   C/LSB
#define EPSON_TILT_SF         (0.000000002f)    // rad/LSB
#define EPSON_MODEL_STR           "A352AD10"
#define EPSON_UNIT_TYPE      "Accelerometer"


/*                                      -- Commands --
    - ADDR_ address byte of transfer to select the register to access
    - CMD_  data byte of transfer to write to the register selected

    - All accesses are 16 bit transfers
    - For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or odd) + 8-bit write data + Delimiter Byte
                                  - No response
        - For UART read accesses - 8-bit address with msb=0b(even only) + 8-bit dummy data + Delimiter Byte
                                 - Response is transferred immediately
                                 - Return value consists of Register Read Address + 16-bit read data (high byte + low byte) + Delimiter Byte

    - NOTE: Register Address Maps that depend on the WINDOW_ID (page) */


// WINDOW_ID 0
#define ADDR_BURST              0x00    // BURST (W0)
#define ADDR_MODE_CTRL_LO       0x02    // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI       0x03    // MODE_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT          0x04    // DIAG_STAT (W0)
#define ADDR_FLAG               0x06    // FLAG(ND/EA) (W0)
#define ADDR_COUNT              0x0A    // COUNT (W0)
#define ADDR_TEMP_HIGH          0x0E    // TEMPC HIGH (W0)
#define ADDR_TEMP_LOW           0x10    // TEMPC LOW  (W0)
#define ADDR_XACCL_HIGH         0x30    // XACCL HIGH (W0)
#define ADDR_XACCL_LOW          0x32    // XACCL LOW  (W0)
#define ADDR_YACCL_HIGH         0x34    // YACCL HIGH (W0)
#define ADDR_YACCL_LOW          0x36    // YACCL LOW  (W0)
#define ADDR_ZACCL_HIGH         0x38    // ZACCL HIGH (W0)
#define ADDR_ZACCL_LOW          0x3A    // ZACCL LOW  (W0)
#define ADDR_XTILT_HIGH         0x3C    // XTILT HIGH (W0)
#define ADDR_XTILT_LOW          0x3E    // XTILT LOW  (W0)
#define ADDR_YTILT_HIGH         0x40    // YTILT HIGH (W0)
#define ADDR_YTILT_LOW          0x42    // YTILT LOW  (W0)
#define ADDR_ZTILT_HIGH         0x44    // ZTILT HIGH (W0)
#define ADDR_ZTILT_LOW          0x46    // ZTILT LOW  (W0)

// WINDOW_ID 1
#define ADDR_SIG_CTRL_LO        0x00    // SIG_CTRL Byte0 (W1)
#define ADDR_SIG_CTRL_HI        0x01    // SIG_CTRL Byte1 (W1)
#define ADDR_MSC_CTRL_LO        0x02    // MSC_CTRL Byte0 (W1)
#define ADDR_MSC_CTRL_HI        0x03    // MSC_CTRL Byte1 (W1)
#define ADDR_SMPL_CTRL_LO       0x04    // SMPL_CTRL Byte0 (W1)
#define ADDR_SMPL_CTRL_HI       0x05    // SMPL_CTRL Byte1 (W1)
#define ADDR_FILTER_CTRL_LO     0x06    // FILTER_CTRL (W1)
#define ADDR_UART_CTRL_LO       0x08    // UART_CTRL Byte0 (W1)
#define ADDR_UART_CTRL_HI       0x09    // UART_CTRL Byte1 (W1)
#define ADDR_GLOB_CMD_LO        0x0A    // GLOB_CMD Byte0 (W1)
#define ADDR_BURST_CTRL_LO      0x0C    // BURST_CTRL1 Byte0 (W1)
#define ADDR_BURST_CTRL_HI      0x0D    // BURST_CTRL1 Byte1 (W1)
#define ADDR_FIR_UCMD           0x16    // FIR_UCMD (W1)
#define ADDR_FIR_UDATA          0x18    // FIR_UDATA (W1)
#define ADDR_FIR_UADDR_LO       0x1A    // FIR_UADDR Byte0 (W1)
#define ADDR_FIR_UADDR_HI       0x1B    // FIR_UADDR Byte1 (W1)
#define ADDR_LONGFILT_CTRL      0x1C    // LONGFILT_CTRL (W1)
#define ADDR_LONGFILT_TAP       0x1E    // LONGFILT_TAP (W1)
#define ADDR_XOFFSET_HIGH_L     0x2C    // XOFFSET_HIGH_L (W1)
#define ADDR_XOFFSET_HIGH_H     0x2D    // XOFFSET_HIGH_H (W1)
#define ADDR_XOFFSET_LOW_L      0x2E    // XOFFSET_LOW_L (W1)
#define ADDR_XOFFSET_LOW_H      0x2F    // XOFFSET_LOW_H (W1)
#define ADDR_YOFFSET_HIGH_L     0x30    // YOFFSET_HIGH_L (W1)
#define ADDR_YOFFSET_HIGH_H     0x31    // YOFFSET_HIGH_H (W1)
#define ADDR_YOFFSET_LOW_L      0x32    // YOFFSET_LOW_L (W1)
#define ADDR_YOFFSET_LOW_H      0x33    // YOFFSET_LOW_H (W1)
#define ADDR_ZOFFSET_HIGH_L     0x34    // ZOFFSET_HIGH_L (W1)
#define ADDR_ZOFFSET_HIGH_H     0x35    // ZOFFSET_HIGH_H (W1)
#define ADDR_ZOFFSET_LOW_L      0x36    // ZOFFSET_LOW_L (W1)
#define ADDR_ZOFFSET_LOW_H      0x37    // ZOFFSET_LOW_H (W1)
#define ADDR_XALARM_LO          0x46    // XALARM_LO (W1)
#define ADDR_XALARM_UP          0x47    // XALARM_UP (W1)
#define ADDR_YALARM_LO          0x48    // YALARM_LO (W1)
#define ADDR_YALARM_UP          0x49    // YALARM_UP (W1)
#define ADDR_ZALARM_LO          0x4A    // ZALARM_LO (W1)
#define ADDR_ZALARM_UP          0x4B    // ZALARM_UP (W1)

#define ADDR_PROD_ID1           0x6A    // PROD_ID1(W1)
#define ADDR_PROD_ID2           0x6C    // PROD_ID2(W1)
#define ADDR_PROD_ID3           0x6E    // PROD_ID3(W1)
#define ADDR_PROD_ID4           0x70    // PROD_ID4(W1)
#define ADDR_VERSION            0x72    // VERSION(W1)
#define ADDR_SERIAL_NUM1        0x74    // SERIAL_NUM1(W1)
#define ADDR_SERIAL_NUM2        0x76    // SERIAL_NUM2(W1)
#define ADDR_SERIAL_NUM3        0x78    // SERIAL_NUM3(W1)
#define ADDR_SERIAL_NUM4        0x7A    // SERIAL_NUM4(W1)
#define ADDR_WIN_CTRL           0x7E    // WIN_CTRL(W0 or W1)

#define CMD_BURST               0x80    // Write value to Issue Burst Read
#define CMD_WINDOW0             0x00    // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1             0x01    // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING      0x01    // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING        0x02    // Write value for MODE_CMD_HI to stop sampling
#define CMD_GOTO_SLEEP_MODE     0x03    // Write value for MODE_CMD_HI to go to the Sleep Mode
#define CMD_ZSENSTEST           0x40    // Write value for MSC_CTRL_HI to issue Z Accelerometer Test
#define CMD_YSENSTEST           0x20    // Write value for MSC_CTRL_HI to issue Y Accelerometer Test
#define CMD_XSENSTEST           0x10    // Write value for MSC_CTRL_HI to issue X Accelerometer Test
#define CMD_FLASHTEST           0x08    // Write value for MSC_CTRL_HI to issue Flash Test
#define CMD_SELFTEST            0x07    // Does ACCTEST, TEMPTEST and VDDTEST for selftest
#define CMD_ACCTEST             0x04    // Write value for MSC_CTRL_HI to issue Accelerometer Test
#define CMD_TEMPTEST            0x02    // Write value for MSC_CTRL_HI to issue Temp Sensor Test
#define CMD_VDDTEST             0x01    // Write value for MSC_CTRL_HI to issue Voltage Test

#define CMD_SOFTRESET           0x80    // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHBKUP           0x08    // Write value for GLOB_CMD_LO to issue Flash Backup
#define CMD_FLASHRST            0x04    // Write value for GLOB_CMD_LO to issue Flash Reset

#define CMD_UART_AUTO_EN        0x01     // Write value for UART_CTRL_LO to enable UART_AUTO mode and set AUTO_START = disabled
#define CMD_UART_AUTO_DIS       0x00     // Write value for UART_CTRL_LO to disable UART_AUTO mode and set AUTO_START = disabled

// Write value for SIG_CTRL_LO
#define OPT_TEMP_STAB           (TEMP_SHOCK<<2)       /* bias stabilization against thermal shock */
#define OPT_MESMOD_SEL          (REDUCED_NOISE<<4)    /* measurement cond: 0=standard noise floor, 1=reduced noise floor */
#define OPT_OUT_SEL_Z           (TILTZ<<5)    /* 0=acceleration, 1=tilt */
#define OPT_OUT_SEL_Y           (TILTY<<6)    /* 0=acceleration, 1=tilt */
#define OPT_OUT_SEL_X           (TILTX<<7)    /* 0=acceleration, 1=tilt */
#define CMD_SIG_CTR_LO_FLAGS    (OPT_OUT_SEL_X | OPT_OUT_SEL_Y | OPT_OUT_SEL_Z | OPT_MESMOD_SEL | OPT_TEMP_STAB)

// Write value for SIG_CTRL_HI to Enables new data (ND) flags for accel_out X,Y,Z are enabled if accel_out is enabled
#define ND_ENABLE_ZACCL         (ENABLE_ACCLZ<<1)
#define ND_ENABLE_YACCL         (ENABLE_ACCLY<<2)
#define ND_ENABLE_XACCL         (ENABLE_ACCLX<<3)
#define ND_ENABLE_TEMP          (ENABLE_TEMP<<7)
#define CMD_EN_NDFLAGS          (ND_ENABLE_TEMP | ND_ENABLE_XACCL | ND_ENABLE_YACCL | ND_ENABLE_ZACCL)

// MSC_CTRL
#define EXT_SEL                 (ENABLE_EXT<<6)    /* EXT function: 0=External trigger disabled, 1=enabled */
#define EXT_POL                 (POL_ACT_LOW<<5)   /* EXT polarity: 0=rising edge, 1=falling edge */
#define ENABLE_DRDY             (1<<2)    /* DRDY function: 0=DRDY disabled, 1=enabled */
#define OPT_DRDY_POL            (1<<1)    /* DRDY polarity: 0=Active low, 1=Active high */
#define CMD_CNTR_DRDY           (EXT_SEL | EXT_POL | ENABLE_DRDY | OPT_DRDY_POL)

// BURST_CTRL_HI (default: 0x47)
#define ENABLE_FLAG_OUT         (ENABLE_FLAG<<7)    /* 0=disabled, 1=enabled 0*/
#define ENABLE_TEMP_OUT         (ENABLE_TEMP<<6)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCX_OUT         (ENABLE_ACCLX<<2)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCY_OUT         (ENABLE_ACCLY<<1)    /* 0=disabled, 1=enabled */
#define ENABLE_ACCZ_OUT         (ENABLE_ACCLZ<<0)    /* 0=disabled, 1=enabled */
// Write value for BURST_CTRL1_HI to enable FLAG, TempC, ACC X,Y,Z registers in burst mode
#define CMD_EN_BRSTDATA_HI      (ENABLE_FLAG_OUT |  ENABLE_TEMP_OUT |  ENABLE_ACCX_OUT | ENABLE_ACCY_OUT | ENABLE_ACCZ_OUT)

// BURST_CTRL_LO (default: 0x02)
#define ENABLE_COUNT_OUT        (ENABLE_COUNT<<1)    /* 0=disabled, 1=enabled */
#define ENABLE_CHKSM_OUT        (ENABLE_CHKSM<<0)    /* 0=disabled, 1=enabled */
// Write value for BURST_CTRL_LO to enable CHKSM, and COUNT bytes in burst mode
#define CMD_EN_BRSTDATA_LO      (ENABLE_COUNT_OUT | ENABLE_CHKSM_OUT)


// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE1000            0x02
#define CMD_RATE500             0x03
#define CMD_RATE200             0x04
#define CMD_RATE100             0x05
#define CMD_RATE50              0x06

// Write values for FILTER_CTRL_LO to set Filter
// The Filter setting should be set according to the sampling rate. Refer
// to the Accelerometer DataSheet to determine a valid filter setting.
#define CMD_FIRTAP64FC83        0x01
#define CMD_FIRTAP64FC220       0x02
#define CMD_FIRTAP128FC36       0x03
#define CMD_FIRTAP128FC110      0x04
#define CMD_FIRTAP128FC350      0x05
#define CMD_FIRTAP512FC9        0x06
#define CMD_FIRTAP512FC16       0x07
#define CMD_FIRTAP512FC60       0x08
#define CMD_FIRTAP512FC210      0x09
#define CMD_FIRTAP512FC460      0x0A
#define CMD_USERFIRTAP4         0x0B //< user filter starts here
#define CMD_USERFIRTAP64        0x0C
#define CMD_USERFIRTAP128       0x0D
#define CMD_USERFIRTAP512       0x0E

// Write values for ADDR_FIR_UCMD
// These values are used to read/write FIR user coefficient values
#define CMD_USERFIR_READ        0x01
#define CMD_USERFIR_WRITE       0x02

// MODE STAT
#define VAL_SAMPLING_MODE       0x00

// Return Values
#define VAL_CONFIG_MASK         0x0F00
#define VAL_CONFIG_MODE         0x0400
#define VAL_SLEEP_MODE          0x0800
#define VAL_NOT_READY           0x0400
#define VAL_FILTER_STAT_BIT     0x0020
#define VAL_SELF_TEST_BIT       0x0700
#define VAL_FLASH_STATUS_BIT    0x0800
#define VAL_DIAG_FLASHBU_ERROR  0x0001
#define VAL_DIAG_ST_ERR_ALL     0x0002
#define VAL_DIAG_FLASH_ERR      0x0004
#define VAL_DIAG_STAT_MASK      0x7302



//const uint8_t DELIMITER =           0x0D;          // EOL marker
#define DELIMITER                   0x0D

// Epson specific timing delays
#define EPSON_STALL_DELAY           25             // Microseconds, minimum delay to wait between commands
#define BURST_STALL                 70             // Microseconds, minumum delay after initial BURST read command
#define EPSON_SWRESET_DELAY         800000         // Microseconds, minimum delay after software reset
#define EPSON_POWER_ON_DELAY        800000         // Microseconds, max delay for poweron startup completion
#define EPSON_FLASH_TEST_DELAY      5000           // Microseconds, max delay for flash-test completion
#define EPSON_FILTER_DELAY          1000           // Microseconds, max delay for filter setup completion
#define EPSON_NRESET_LOW_DELAY      100000         // Microseconds, min delay for nRESET assertion

#define EPSON_DRDYCHECK             1000
//extern const uint32_t EPSON_DRDYCHECK =    1000;          // Define max # of times to check


#define EpsonStall()                delayMicroseconds(EPSON_STALL_DELAY)
#define EpsonBurstStall()           delayMicroseconds(BURST_STALL)
#define EpsonPowerOnDelay()         delayMicroseconds(EPSON_POWER_ON_DELAY)
#define EpsonSelfTestDelay()        delayMicroseconds(EPSON_SELF_TEST_DELAY)
#define EpsonFlashTestDelay()       delayMicroseconds(EPSON_FLASH_TEST_DELAY)
#define EpsonSwResetDelay()         delayMicroseconds(EPSON_SWRESET_DELAY)
#define EpsonFilterDelay()          delayMicroseconds(EPSON_FILTER_DELAY)
#define EpsonResetAssertDelay()     delayMicroseconds(EPSON_NRESET_LOW_DELAY)


#endif /* M_A352_DEF_H_ */




