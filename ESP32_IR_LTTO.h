
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
 * Based on the Code from Darryl Scott: https://github.com/Darryl-Scott/ESP32-RMT-Library-IR-code-RAW
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

#define ARRAY_SIZE  150

struct LttoMessage
{
    char            type;           //message type (Tag, Beacon, Enhanced beacon, Paclet, Data, Checksum)
    unsigned int    data;           //eg. 78dec, but info inside is often individual bits
    
    unsigned int    teamNum;        //what team a tagger belongs to
    unsigned int    playerNum;      //the taggers player number in the team
    unsigned int    megaTag;        //what strength of Megatag (0-3 are valid).
    bool            isTaggedbeacon; //this beacon was sent as player was just tagged by....
} ;

struct hostGameData
{
    int             playerNum;      //1-24 for hosted games, 0 for non-hosted
    int             gameType;       //
    
};

class ESP32_IR {
  public:
    ESP32_IR();
    bool    ESP32_IRrxPIN (int _rxPin, int _channel);  //valid channels are 0-7 incl.
    bool    ESP32_IRtxPIN (int _txPin, int _channel);  //valid channels are 0-7 incl.
    void    initReceive();
    void    initTransmit();
    void    stopIR();
    int     readIR(unsigned int *data, int maxBuf);
    void    sendIR(rmt_item32_t data[], int IRlength, bool waitTilDone = false);
    void    sendLttoIR(char _type, int _data);
    void    sendLttoIR(String _fullDataString);
    
    int     getLttoMessageTeamNum();
    int     getLttoMessagePlayerNum();
    int     getLttoMessageMegatag();
    
    void        sendIR(char type, uint8_t message);
    bool        sendLTAG(byte tagPower);
    bool        sendTag(byte teamID, byte playerID, byte tagPower);
    bool        sendBeacon(bool tagReceived, byte teamID, byte tagPower);
    bool        sendZoneBeacon(byte zoneType, byte teamID);
    bool        sendLTARbeacon(bool tagReceived, bool shieldsActive,
                               byte tagsRemaining, byte unKnown, byte teamID);
//    int         hostPlayerToGame(void (*callBack)(), uint8_t _playerNumber, uint8_t _gameType, uint8_t _gameID,
//                                 uint8_t _gameLength, uint8_t _health, uint8_t _reloads,
//                                 uint8_t _shields, uint8_t _megaTags, uint8_t _flags1,
//                                 uint8_t _flags2, uint8_t _flags3 = -1);
    int         hostPlayerToGame(uint8_t _teamNumber, uint8_t _playerNumber, uint8_t _gameType,
                                 uint8_t _gameID,     uint8_t _gameLength,   uint8_t _health,
                                 uint8_t _reloads,    uint8_t _shields,      uint8_t _megaTags,
                                 uint8_t _flags1,     uint8_t _flags2,       int8_t _flags3 = -1);
    void        assignPlayer(uint8_t _gameID, uint8_t _taggerID, uint8_t _teamNumber, uint8_t _playerNumber);
    
    char        readMessageType();
    uint16_t    readRawDataPacket();
    //void        writeCancelHosting();
    //void        writeHostingInterval(int _interval);
    //int         readHostingInterval();
    
    bool        available();
    void        clearMessageOverwrittenCount();
    byte        readMessageOverwrittenCount();
    
    byte        readTeamID();
    byte        readPlayerID();
    byte        readShotStrength();
    char        readBeaconType();
    bool        readTagReceivedBeacon();
    byte        readPacketByte();
    byte        readByteCount();
    String      readPacketName();
    String      readDataType();
    long int    readDataByte();
    uint8_t     readCheckSumRxByte();
    bool        readCheckSumOK();

  private:
    rmt_item32_t    irDataArray[ARRAY_SIZE];
    int             irDataRxArray[50]
    int             arrayIndex;
    int             gpioNum;
    int             rmtPort;
    uint16_t        calculatedCheckSum;
    //bool            cancelHosting;
    //uint16_t        hostingInterval;
    
    
    void    decodeRAW(rmt_item32_t *rawDataIn, int numItems, unsigned int* irDataOut);
   
    void    getDataIR(rmt_item32_t item, unsigned int *datato, int index);
    void    buildItem(rmt_item32_t &item,int high_us,int low_us);
    
    bool    decodeLTTO(rmt_item32_t *rawDataIn, int numItems, unsigned int *irDataOut);
    bool    checkData(rmt_item32_t *rawDataIn, int _index, int _itemToCheck, unsigned int _expectedDuration);
    //void encodeLTTO(rmt_item32_t *irDataArrayLocal, char _type, int _data);
    void    encodeLTTO(char _type, uint16_t _data = 0);
    int     convertTeamAndPlayer(uint8_t _teamNumber, uint8_t _playerNumber);
    void    clearIRdataArray();
    
    int     convertDecToBCD(int _dec);
    int     convertBCDtoDec(int _bcd);
    
    LttoMessage lttoMessage;
};

#endif /* ESP32_IR_LTTO_H_ */
