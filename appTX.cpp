/*
 * Copyright (c) 2024 Particle Industries, Inc.  All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "Particle.h"
#include "LoRaWAN.h"
#include <spark_wiring_error.h>
#include <spark_wiring_logging.h>
#include "dct_hal.h"

//allows for threading + automatic BLE reconnectionn //cellular connection
SYSTEM_THREAD(ENABLED); 
SYSTEM_MODE(AUTOMATIC);
//

SerialLogHandler logHandler(LOG_LEVEL_ALL);

LoRaWAN lora(LORA_TYPE_SERIAL1);

Thread *LoRaThread;

const char* serviceUuid = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char* BLEUuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

int interruptBLE = D3;
int interruptCELL = D21;
int interruptLORA = D2;

int CELL_LED = D4;
int BLE_LED= D6;
int LORA_LED = D5;



volatile bool bleStat = false;
volatile bool cellStat = false;
volatile bool loraStat = false;
    
void cellfunction(void){
    digitalWrite(CELL_LED,LOW);}
    
    
    
void blefunction(void){
    digitalWrite(BLE_LED,LOW);}


void lorafunction(void){
    digitalWrite(LORA_LED,LOW);
}
void LoRaThreadFunction();

Timer cellLEDTimer(2500,cellfunction);
Timer bleLEDTimer(2500,blefunction);
Timer loraLEDTimer(2500,lorafunction);

const unsigned long debounceDelay = 500;
volatile unsigned long lastBleInterrupt = 0;
volatile unsigned long lastCellInterrupt = 0;
volatile unsigned long lastLoraInterrupt = 0;



BleCharacteristic myChar("char", BleCharacteristicProperty::READ| BleCharacteristicProperty::NOTIFY, BLEUuid, serviceUuid);


BleAdvertisingData data;

void sendBLE(void){
   bleStat = !bleStat;
}
    
void sendCell(void){
   cellStat = !cellStat;
}

void sendLora(void){
   loraStat = !loraStat;
}





// joinEui (8 bytes) and appKey (16 bytes) combined
const auto CTRL_REQUEST_KEYS_RESP_DATA_SIZE = 24;

// DCT offset for reserved2 which is currently unused
// XXX: Change this if the DCT layout changes
const auto DCT_RESERVED2_OFFSET = 8172;

const auto AUX_3V3_POWER_CONTROL_IO = D7;

int writeKeysToDCT(void* data, size_t size) {
    Log.info("Writing lorawan keys to DCT...");
    return dct_write_app_data(data, DCT_RESERVED2_OFFSET, size);
}

int readKeysFromDCT(void* joinEui, void* appKey) {
    Log.info("Reading lorawan keys from DCT...");

    char buf[CTRL_REQUEST_KEYS_RESP_DATA_SIZE];

    int res = dct_read_app_data_copy(DCT_RESERVED2_OFFSET, buf, CTRL_REQUEST_KEYS_RESP_DATA_SIZE);

    Log.info("Read back %d bytes of data:", CTRL_REQUEST_KEYS_RESP_DATA_SIZE);

    for (size_t i = 0; i < CTRL_REQUEST_KEYS_RESP_DATA_SIZE; i++) {
        // fill first 8 bytes with joinEui
        if (i < 8) {
            ((uint8_t*)joinEui)[i] = buf[i];
        }
        // fill next 16 bytes with appKey
        else {
            ((uint8_t*)appKey)[i-8] = buf[i];
        }
        Log.printf(LOG_LEVEL_TRACE, "%02X", buf[i]);
    }
    Log.print(LOG_LEVEL_TRACE, "\r\n");
    return res;
}

int handleRequest(ctrl_request* req) {
    auto size = req->request_size;
    auto data = req->request_data;

    if (size != CTRL_REQUEST_KEYS_RESP_DATA_SIZE) {
        Log.error("Invalid keys data size received: %d", size);
        return Error::BAD_DATA;
    }

    Log.info("Received %d bytes of data:", size);
    for (size_t i = 0; i < size; i++) {
        Log.printf("%02X", data[i]);
    }
    Log.print(LOG_LEVEL_TRACE, "\r\n");

    int r = writeKeysToDCT(data, size);

    system_ctrl_set_result(req, r, nullptr /* handler */, nullptr /* data */, nullptr /* reserved */);

    return r;
}

// Custom control request handler
void ctrl_request_custom_handler(ctrl_request* req) {
    handleRequest(req);
}

void setup(){
    BLE.on();
    //all pull-up buttons
    pinMode(interruptBLE, INPUT_PULLUP);
    pinMode(interruptCELL, INPUT_PULLUP);
    pinMode(interruptLORA, INPUT_PULLUP);
    attachInterrupt(interruptBLE, sendBLE, FALLING);
    attachInterrupt(interruptCELL, sendCell, FALLING);
    attachInterrupt(interruptLORA, sendLora, FALLING);
    
    //all transmit LEDs
    pinMode(CELL_LED, OUTPUT);
    pinMode(BLE_LED, OUTPUT);
    pinMode(LORA_LED, OUTPUT);
    digitalWrite(CELL_LED, LOW);
    digitalWrite(BLE_LED, LOW);
    digitalWrite(LORA_LED, LOW);
    
    //BLE setup
    BLE.addCharacteristic(myChar);
    data.appendLocalName("BAZOOKA");
    data.appendServiceUUID(serviceUuid);
    myChar.setValue("I LOVE");
    BLE.advertise(&data);
    Particle.publish("Transmit_BLE", "I LOVE MY JOB!!!");

    LoRaThread = new Thread("Workerthread",LoRaThreadFunction);
}


void loop()
{
    //if BLE button pressed
    if(bleStat){
    bleStat = false;
    unsigned long currentTime = millis();
    if (currentTime-lastInterrupt>debounceDelay){
        Log.info("BLE Button Press");
        myChar.setValue("I LOVE MY JOB!!!!");
        digitalWrite(BLE_LED,HIGH);
        bleLEDTimer.start();
     lastInterrupt=currentTime;
}
}

    if(cellStat){
    cellStat = false;
     unsigned long currentTime = millis();
    if (currentTime-lastInterrupt>debounceDelay){
        Log.info("Cell Button Press");
        Particle.publish("Transmit_Cellular","Relay On");
        digitalWrite(CELL_LED,HIGH);
        cellLEDTimer.start();
     lastInterrupt=currentTime;
}
}

if(loraStat){
    loraStat=false;
    unsigned long currentTime = millis();
    if (currentTime-lastLoraInterrupt>debounceDelay){
        Log.info("LORA button press");
    static uint32_t fcnt = 0;
    if (lora.getNwJoinStatus() == NW_JOIN_SUCCESS) {
            Log.info("TXing: %lu", fcnt);
            Variant v;
            v["btn"] = "on";
            v["count"] = fcnt;
            lora.publish(123 /* code */, v);
            digitalWrite(LORA_LED,HIGH);
            if(Particle.connected()){
                Particle.publish("Transmit_LORAWAN","LORA DOWNLINK SENT");
            }
            loraLEDTimer.start();
            fcnt++;
    }
    // lora.process();
    // Particle.process();
    lastLoraInterrupt=currentTime;
}
    }
}

void LoRaThreadFunction(){
    while(true){
        if(!joinedLora){
    pinMode(AUX_3V3_POWER_CONTROL_IO, OUTPUT);
    digitalWrite(AUX_3V3_POWER_CONTROL_IO, HIGH);
    RGB.control(true);
    RGB.color(0,255,0);
    uint8_t devEui[8] = {0x94, 0x94, 0x4a, 0x00, 0x00, 0x07, 0x15, 0x30};
    uint8_t joinEui[8] = {0};
    uint8_t appKey[16] = {0};
    readKeysFromDCT(joinEui, appKey);

    Log.info("BEGIN --------------------");
    auto conf = LoRaWANConfig()
            .devEui(devEui)
            .joinEui(joinEui)
            .appKey(appKey);
    int begin = lora.begin(std::move(conf));
    lora.process();

    if (begin != 0) {
        Log.info("Init failed.");
        RGB.color(255,0,0);

        // Attempt to recover the module by reflashing it
        lora.updateFirmware(true);
    } else {
        // Update the firmware if a new one is available
        lora.updateFirmware();
            Log.info("JOIN ---------------------");
            lora.join();
        }
        lora.process();

    if (lora.getNwJoinStatus() == NW_JOIN_SUCCESS) {
        RGB.color(0,255,255);
        Log.info("PUBLISH ------------------");
        joinedLora= true;
        digitalWrite(AUX_3V3_POWER_CONTROL_IO, HIGH);
    }
}
else{
    delay(1000);
}
}}