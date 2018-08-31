
 /* Copyright (c) 2018 Richie Mickan. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. *
 * 
 * Based on the Code from Neil Kolban: https://github.com/nkolban/esp32-snippets/blob/master/hardware/infra_red/receiver/rmt_receiver.c
 * Based on the Code from pcbreflux: https://github.com/pcbreflux/espressif/blob/master/esp32/arduino/sketchbook/ESP32_IR_Remote/ir_demo/ir_demo.ino
 * Based on the Code from Xplorer001: https://github.com/ExploreEmbedded/ESP32_RMT
 * Based on the Code from Darryl Scott: https://
 */

/* This library uses the RMT hardware in the ESP32 to send/receive LTTO infrared data.
 * It uses a single channel of the RMT, therefore up to 8 instances can be used in a single sketch.
 * Hopefully it is useful.
 */


#ifndef ESP32_IR_LTTO_H_
#define ESP32_IR_LTTO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp32-hal.h"
#include "esp_intr.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "freertos/semphr.h"
#include "soc/rmt_struct.h"

#ifdef __cplusplus
}
#endif

struct LttoMessage
{
    char type;
    unsigned int data;
} ;


class ESP32_IR {
  public:
    ESP32_IR();
    bool ESP32_IRrxPIN (int _rxPin, int _port);
    bool ESP32_IRtxPIN (int _txPin, int _port);
    void initReceive();
    void initTransmit();
    void stopIR();
    int  readIR(unsigned int *data, int maxBuf);
//    void sendIR(unsigned int data[], int IRlength);
    void sendIR(rmt_item32_t data[], int IRlength);
    
    

  private:
    int gpioNum;
    int rmtPort;
    void decodeRAW(rmt_item32_t *rawDataIn, int numItems, unsigned int* irDataOut);
    bool decodeLTTO(rmt_item32_t *rawDataIn, int numItems, unsigned int *irDataOut);
    void getDataIR(rmt_item32_t item, unsigned int *datato, int index);
    void buildItem(rmt_item32_t &item,int high_us,int low_us);
    bool checkData(rmt_item32_t *rawDataIn, int _index, int _durationToCheck, unsigned int _timing);
};

#endif /* ESP32_IR_LTTO_H_ */
