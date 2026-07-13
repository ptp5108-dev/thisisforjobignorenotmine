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

SYSTEM_THREAD(ENABLED); 
SYSTEM_MODE(AUTOMATIC);

SerialLogHandler logHandler(LOG_LEVEL_ALL);

LoRaWAN lora(LORA_TYPE_SERIAL1);
int LED_CELL = D4;
int LED_BLE= D6;
int LED_LORA = D5;

Thread *newThread;
Thread *newerThread;



String CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
char macStr[18] = "94944A0717B5"; 
BlePeerDevice peer;
BleCharacteristic characteristic;

volatile bool joinedLora = false;
const unsigned long debounceDelay = 500;
volatile unsigned long lastInterrupt = 0;

void switchOn(int LED, String msg){
    digitalWrite(LED,HIGH);
    transmitHandler(msg);}
    
void cellfunction(void){
    digitalWrite(LED_CELL,LOW);}
    
    
    
void blefunction(void){
    digitalWrite(LED_BLE,LOW);}
    
    
Timer cellLEDTimer(2500,cellfunction);
Timer bleLEDTimer(2500,blefunction);
    
    
    
void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
                String receivedString = String((const char*)data, len);
                Log.info(receivedString);
                transmitHandler(receivedString);
                digitalWrite(LED_BLE, HIGH);
                bleLEDTimer.start();
                Log.info("Peer %s connected + message read!", peer.address().toString().c_str());
}

void recieveHandler(const char *event, const char *data);
void transmitHandler(const char *message);
void LoRaThread();
void BLEThread();



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
    Particle.subscribe("Transmit_Cellular", receiveHandler);
    newThread = new Thread("Workerthread",BLEThread);
    newerThread = new Thread("Workerthread",LoRaThread);
    pinMode(LED_CELL, OUTPUT);
    pinMode(LED_BLE, OUTPUT);
    pinMode(LED_LORA, OUTPUT);
    digitalWrite(LED_CELL, LOW);
    digitalWrite(LED_BLE, LOW);
    digitalWrite(LED_LORA, LOW);
}

void loop()
{
    delay(1000);
    }

void BLEThread(){
    while(true){
    if (!peer.connected()){
        peer = BLE.connect(macStr);
        delay(3000);
        
    }
    
    if(peer.connected() && !characteristic){
        Log.info("connected yay");
        while(!characteristic){
            peer.getCharacteristicByUUID(characteristic, CHARACTERISTIC_UUID);
        }
        Log.info("got characteristic, subbing now!");
        characteristic.onDataReceived(onDataReceived, NULL);
    }
    delay(1000);
}}

void LoRaThread(){
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
            // .defaultDevEui()
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
static uint32_t s = millis() - 20000;
    static uint32_t fcnt = 0;
    if (lora.getNwJoinStatus() == NW_JOIN_SUCCESS) {
        if (millis() - s > 20000) {
            Log.info("TXing: %lu", fcnt);
            Variant v;
            v["foo"] = "bar";
            v["count"] = fcnt;
            lora.publish(123 /* code */, v);
            s = millis();
            fcnt++;
        }
    }

    lora.process();
    Particle.process();
}
}}



void receiveHandler(const char *event, const char *data) {
        switchOn(LED_CELL,data);
        cellLEDTimer.start();
}

void transmitHandler(const char *message) {
  String eventName = "Transmit_Recieved";
  if (Particle.connected()) {
    Particle.publish(eventName, message);
    Log.info("published %s", message);
  } else {
    Log.info("not cloud connected %s", message);
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
