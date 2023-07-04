
#define REG_DUMP(fnRead, dbgv) {            \
  printf("\r\nRegister Dump:\r\n"); \
  printf("Window 0:");              \
                                            \
  fnRead(0x00, 0x02, dbgv);                 \
  fnRead(0x00, 0x04, dbgv);                 \
  fnRead(0x00, 0x06, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x0A, dbgv);                 \
  fnRead(0x00, 0x0E, dbgv);                 \
  fnRead(0x00, 0x10, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x30, dbgv);                 \
  fnRead(0x00, 0x32, dbgv);                 \
  fnRead(0x00, 0x34, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x36, dbgv);                 \
  fnRead(0x00, 0x38, dbgv);                 \
  fnRead(0x00, 0x3A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x3C, dbgv);                 \
  fnRead(0x00, 0x3E, dbgv);                 \
  fnRead(0x00, 0x40, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x00, 0x42, dbgv);                 \
  fnRead(0x00, 0x44, dbgv);                 \
  fnRead(0x00, 0x46, dbgv);                 \
  SerialConsole.println();                  \
                                            \
  SerialConsole.println("Window 1:\r\n");   \
  fnRead(0x01, 0x00, dbgv);                 \
  fnRead(0x01, 0x02, dbgv);                 \
  fnRead(0x01, 0x04, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x06, dbgv);                 \
  fnRead(0x01, 0x08, dbgv);                 \
  fnRead(0x01, 0x0A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x0C, dbgv);                 \
  fnRead(0x01, 0x16, dbgv);                 \
  fnRead(0x01, 0x18, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x1A, dbgv);                 \
  fnRead(0x01, 0x1C, dbgv);                 \
  fnRead(0x01, 0x1E, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x2C, dbgv);                 \
  fnRead(0x01, 0x2E, dbgv);                 \
  fnRead(0x01, 0x30, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x32, dbgv);                 \
  fnRead(0x01, 0x34, dbgv);                 \
  fnRead(0x01, 0x36, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x46, dbgv);                 \
  fnRead(0x01, 0x48, dbgv);                 \
  fnRead(0x01, 0x4A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x6A, dbgv);                 \
  fnRead(0x01, 0x6C, dbgv);                 \
  fnRead(0x01, 0x6E, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x70, dbgv);                 \
  fnRead(0x01, 0x72, dbgv);                 \
  fnRead(0x01, 0x74, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x76, dbgv);                 \
  fnRead(0x01, 0x78, dbgv);                 \
  fnRead(0x01, 0x7A, dbgv);                 \
  SerialConsole.println();                  \
  fnRead(0x01, 0x7E, dbgv);                 \
  SerialConsole.println();                  \
}


class EPSON_DEV:public UART_EPSON_COM {

 protected:
    // Stores burst flags after decodeBurstCtrl() method
    struct _burstFlag {
      // 0 = Off, 1 = On
      uint8_t nd_ea;
      uint8_t tempc;
      uint8_t acclx;
      uint8_t accly;
      uint8_t acclz;
      uint8_t inclx;
      uint8_t incly;
      uint8_t inclz;
      uint8_t count;
      uint8_t chksm;
    } _burstFlag = {0};

 public:
    EPSON_DEV(int8_t nrst, int8_t drdy):UART_EPSON_COM(nrst, drdy){};   // UART non-AUTO Mode (with DRDY)

    /**************************************************************************/
    /*!
        @brief  Decodes the DOUT_RATE register value to output rate in Hz

        @returns    float of output rate (Hz)
    */
    /**************************************************************************/
    float decodeDoutRate(void) {

      uint8_t dout_rate = (regRead16(CMD_WINDOW1, ADDR_SMPL_CTRL_LO)&0x0F00)>>8;
      switch (dout_rate) {
        case 2: return 1000.0; break;
        case 3: return 500.0; break;
        case 4: return 200.0; break;
        case 5: return 100.0; break;
        case 6: return 50.0; break;
        default:
    #ifdef DEBUG
        SerialConsole.print("Invalid DOUT_RATE");
    #endif //DEBUG
        return -1; break;
      }
    }


    /**************************************************************************/
    /*!
        @brief  Decodes the FILTER_SEL register value to return filter in
                as decoded string value

        @param [in]  filterString
                     Pointer to string to store decoded filter
                     setting
    */
    /**************************************************************************/
    void decodeFilterSel(char* filterString) {

      uint8_t filter_sel = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO)&0x0F;
      switch (filter_sel) {
        case 1: sprintf(filterString, "KAISER64FC83"); break;
        case 2: sprintf(filterString, "KAISER64FC220"); break;
        case 3: sprintf(filterString, "KAISER128FC36"); break;
        case 4: sprintf(filterString, "KAISER128FC110"); break;
        case 5: sprintf(filterString, "KAISER128FC350"); break;
        case 6: sprintf(filterString, "KAISER512FC9"); break;
        case 7: sprintf(filterString, "KAISER512FC16"); break;
        case 8: sprintf(filterString, "KAISER512FC60"); break;
        case 9: sprintf(filterString, "KAISER512FC210"); break;
        case 10: sprintf(filterString, "KAISER512FC460"); break;
        case 11: sprintf(filterString, "UDF4"); break;
        case 12: sprintf(filterString, "UDF64"); break;
        case 13: sprintf(filterString, "UDF128"); break;
        case 14: sprintf(filterString, "UDF512"); break;
        default:
          sprintf(filterString, "INVALID");
          break;
      }
    }


    /**************************************************************************/
    /*!
        @brief  Decodes the BURST_CTRL register and stores
                status in _burstFlag struct and returns burst length in bytes

        @returns    sensor burst length in bytes(excluding header & delimiter byte)
    */
    /**************************************************************************/
    uint8_t decodeBurstCtrl(void) {

      uint8_t burst_len = 0;

      uint8_t sig_ctrl = regRead16(CMD_WINDOW1, ADDR_SIG_CTRL_LO)&0xE0;
      uint16_t burst_ctrl = regRead16(CMD_WINDOW1, ADDR_BURST_CTRL_LO)&0xC703;

      // burst_ctrl check
      _burstFlag.nd_ea = (burst_ctrl&0x8000) ? 1 : 0;
      _burstFlag.tempc = (burst_ctrl&0x4000) ? 1 : 0;
      _burstFlag.acclx = (burst_ctrl&0x400) ? 1 : 0;
      _burstFlag.accly = (burst_ctrl&0x200) ? 1 : 0;
      _burstFlag.acclz = (burst_ctrl&0x100) ? 1 : 0;
      _burstFlag.count = (burst_ctrl&0x2) ? 1 : 0;
      _burstFlag.chksm = (burst_ctrl&0x1) ? 1 : 0;
      // sig_ctrl check
      _burstFlag.inclx = (sig_ctrl&0x80) ? 1 : 0;
      _burstFlag.incly = (sig_ctrl&0x40) ? 1 : 0;
      _burstFlag.inclz = (sig_ctrl&0x20) ? 1 : 0;

      // Calc burst_len
      if (_burstFlag.nd_ea) burst_len += 2;
      if (_burstFlag.tempc) burst_len += 4;
      if (_burstFlag.acclx) burst_len += 4;
      if (_burstFlag.accly) burst_len += 4;
      if (_burstFlag.acclz) burst_len += 4;
      if (_burstFlag.count) burst_len += 2;
      if (_burstFlag.chksm) burst_len += 2;
      return burst_len;
    }


    /**************************************************************************/
    /*!
        @brief      Enters Config Mode to check and output
                    sensor configuration in table format to console
    */
    /**************************************************************************/
    void sensorConfigDump(void) {

      if (sensorStop() == true) {

        // Print to formated table
        SerialConsole.println("\n*****************************************************************");
        char prod_id[]="XXXXXXXX";
        getProdID(prod_id);
        char serial_id[]="XXXXXXXX";
        getSerialNumber(serial_id);

        char linebuf[80];
        sprintf(linebuf, "PROD_ID: %s\tSERIAL_ID: %s\tVERSION: %x", prod_id, serial_id, getVersion());
        SerialConsole.println(linebuf);

        char buf[16];
        decodeFilterSel(buf);
        sprintf(linebuf, "DOUT_RATE: %0.3f\tFILTER_SEL: %s", decodeDoutRate(), buf);
        SerialConsole.println(linebuf);
        decodeBurstCtrl();
        if (_burstFlag.nd_ea) SerialConsole.print("ND_EA: ON\t");
        else SerialConsole.print("ND_EA: OFF\t");

        if (_burstFlag.tempc) SerialConsole.print("TempC: 32\t");
        else SerialConsole.print("TempC: OFF\t");

        SerialConsole.println();
        if (_burstFlag.acclx) {
          if (_burstFlag.inclx) SerialConsole.print("IncX: 32\t");
          else SerialConsole.print("AccX: 32\t");
        }
        else SerialConsole.print("AccX: OFF\t");

        if (_burstFlag.accly) {
          if (_burstFlag.incly) SerialConsole.print("IncY: 32\t");
          else SerialConsole.print("AccY: 32\t");
        }
        else SerialConsole.print("AccY: OFF\t");

        if (_burstFlag.acclz) {
          if (_burstFlag.inclz) SerialConsole.print("IncZ: 32\t");
          else SerialConsole.print("AccZ: 32\t");
        }
        else SerialConsole.print("AccZ: OFF\t");

        SerialConsole.println();
        if (_burstFlag.count) SerialConsole.print("Count: ON\t");
        else SerialConsole.print("Count: OFF\t");

        if (_burstFlag.chksm) SerialConsole.print("Chksm: ON");
        else SerialConsole.print("Chksm: OFF");

        SerialConsole.println("\n*****************************************************************");
      }
      else {
        SerialConsole.println("Error entering CONFIG mode.");
      }
    }


    /**************************************************************************/
    /*!
        @brief  Prints sensor's scale factors
    */
    /**************************************************************************/
    void sensorScaleFactorsPrint(void){
      SerialConsole.println("*****************************************************************");
      SerialConsole.print("Accl SF: "); SerialConsole.print(float(EPSON_ACCL_SF),9); SerialConsole.println(" G/bit");
      SerialConsole.print("Tilt SF: "); SerialConsole.print(float(EPSON_TILT_SF),9); SerialConsole.println(" rad/bit");
      SerialConsole.println("*****************************************************************");
    }

    /**************************************************************************/
    /*!
        @brief  Prints header row of output data
    */
    /**************************************************************************/
    void sensorHeaderPrint(void){

      SerialConsole.println();
      SerialConsole.print("Sample#\t");
      if (_burstFlag.nd_ea) {
        SerialConsole.print("ND_EA\t");
      }

      if (_burstFlag.tempc) {
        SerialConsole.print("TempC\t\t");
      }

      if(_burstFlag.acclx) {
        if(_burstFlag.inclx) SerialConsole.print("Tilt X\t");
        else                SerialConsole.print("Accl X\t");
      }

      if(_burstFlag.accly) {
        if(_burstFlag.incly) SerialConsole.print("Tilt Y\t");
        else                SerialConsole.print("Accl Y\t");
      }

      if(_burstFlag.acclz) {
        if(_burstFlag.inclz) SerialConsole.print("Tilt Z\t");
        else                SerialConsole.print("Accl Z\t");
      }

      if(_burstFlag.count) {
        SerialConsole.print("Count\t");
      }

      if(_burstFlag.chksm) {
        SerialConsole.print("Checksum");
      }

      SerialConsole.println();
    }

    /**************************************************************************/
    /*!
        @brief  Convert and output sensor data to console

        @param [in]  data
                     Pointer to 16-bit Array
        @param [in]  sampleCount
                     Current Sample Count
    */
    void sensorDataPrint(uint16_t* data, uint32_t sampleCount) {

    SerialConsole.print(sampleCount); SerialConsole.print("\t");

    #ifdef DEBUG
      for(int i = 0; i < _burstCnt_calculated; i++) {
        SerialConsole.print(data[i], HEX);
        SerialConsole.print("\t");
      }
      SerialConsole.println();
    #endif

      // stores the accelerometer data array index when parsing out data fields
      int idx = 0;

      // parsing of data fields applying conversion factor if applicable
      if (_burstFlag.nd_ea) {
        //process ND flag data
        unsigned short ndflags = data[idx];
        idx += 1;
        SerialConsole.print(ndflags, HEX); SerialConsole.print("\t");
      }

      if (_burstFlag.tempc) {
        //process temperature data
        int32_t temp = (data[idx]<<16) | (data[idx+1]<<0);
        float temperature = ((float)temp*EPSON_TEMP_SF) + 34.987f;
        idx += 2;
        SerialConsole.print(temperature, 6); SerialConsole.print("\t");
      }

      if(_burstFlag.acclx) {
        //process x axis data
        float accel_x;
        int32_t x = (data[idx]<<16) | (data[idx+1]<<0);
        if(_burstFlag.inclx)
          accel_x = (EPSON_TILT_SF * (float)x); //< tilt
        else
          accel_x = (EPSON_ACCL_SF * (float)x); //< acceleration
        SerialConsole.print(accel_x, 6); SerialConsole.print("\t");
        idx += 2;
      }

      if(_burstFlag.accly) {
        //process y axis data
        float accel_y;
        int32_t y = (data[idx]<<16) | (data[idx+1]<<0);
        if(_burstFlag.incly)
          accel_y = (EPSON_TILT_SF * (float)y); //< tilt
        else
          accel_y = (EPSON_ACCL_SF * (float)y); //< acceleration
        SerialConsole.print(accel_y, 6); SerialConsole.print("\t");
        idx += 2;
      }

      if(_burstFlag.acclz) {
        //process z axis data
        float accel_z;
        int32_t z = (data[idx]<<16) | (data[idx+1]<<0);
        if(_burstFlag.inclz)
          accel_z = (EPSON_TILT_SF * (float)z); //< tilt
        else
          accel_z = (EPSON_ACCL_SF * (float)z); //< acceleration
        SerialConsole.print(accel_z, 6); SerialConsole.print("\t");
        idx += 2;
      }

      if(_burstFlag.count) {
        //process count out data
        SerialConsole.print(data[idx], DEC); SerialConsole.print("\t");
        idx += 1;
      }

      if(_burstFlag.chksm) {
        // process checksum data
        SerialConsole.print(data[idx], HEX); SerialConsole.print("\t");
      }

      SerialConsole.println();
    }

    /**************************************************************************/
    /*!
       @brief  Assumes currently in Configuration Mode to set counter, ND_Flags,
               Burst Settings, output rate & filter settings.
               For valid settings, refer to device datasheet

        @param [in]  outputRate
                     refer to possible #defines CMD_RATExxx earlier in this file
        @param [in]  filterSetting
                     refer to possible #defines CMD_FIRTAPxxxFCxxx
                     earlier in this file

        @returns     false if fail / true if success
    */
    /**************************************************************************/
    boolean sensorInit(uint8_t outputRate, uint8_t filterSetting) {

      if (filterSetting >= CMD_USERFIRTAP4)
      {
        SerialConsole.println("User filter coefficients not implemented!");
        return false;
      }

      regWrite8(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filterSetting);

      // Delay for filter config
      EpsonFilterDelay();

      // Check that the FILTER_BUSY bit returns 0
      unsigned short rxData;
      unsigned short retryCount = 3000;
      do
      {
        rxData = regRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO);
        retryCount--;
      } while((rxData & 0x0020) == 0x0020 && (retryCount != 0));

      if (retryCount == 0)
      {
        SerialConsole.println("...Error: Filter busy bit did not return to 0b.");
        return false;
      }
      regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_LO, CMD_CNTR_DRDY);
      regWrite8(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, outputRate);
      regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, CMD_UART_AUTO_DIS);

      regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_LO, CMD_EN_BRSTDATA_LO);
      regWrite8(CMD_WINDOW1, ADDR_BURST_CTRL_HI, CMD_EN_BRSTDATA_HI);

      regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_LO, CMD_SIG_CTR_LO_FLAGS);
      regWrite8(CMD_WINDOW1, ADDR_SIG_CTRL_HI, CMD_EN_NDFLAGS);

      _burstCnt_calculated = decodeBurstCtrl();

      #ifdef DEBUG
      SerialConsole.println(_burstCnt_calculated, DEC);
      #endif

      // If DRDY is not used then enable UART_AUTO mode, otherwise keep in UART Manual Mode
      if ( getDRDY() == -1 ) {
          // When DRDY is not used, enable UART AUTO mode
          regWrite8(CMD_WINDOW1, ADDR_UART_CTRL_LO, CMD_UART_AUTO_EN ); // UART_CTRL Low Byte enables/disables UART_AUTO mode
          setUARTAuto(true);
      }

      return true;
    }

    /**************************************************************************/
    /*!
        @brief  Output register values to console
    */
    /**************************************************************************/
    void registerDump(void){

      if (sensorStop()) {
        REG_DUMP(regRead16, true);
      }
      else  {
        SerialConsole.println("Warning: Not entering Config Mode");
      }
    }

};

