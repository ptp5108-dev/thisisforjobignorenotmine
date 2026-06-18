/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#define LMIC_SPI_FREQ 1000000

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

int lastState=0;
int BLEPin =25;
int LORAPin =35;
bool BLEIsOn=true;
bool LORAIsOn=true;
class MyServerCallbacks : public BLEServerCallbacks{
  void onDisconnect(BLEServer *pServer){
    Serial.println("Client Disconnected");
    esp_restart();
  }
};
BLECharacteristic *pCharacteristic;

hw_timer_t *debounceTimer=NULL;
const uint32_t TIMER_FREQ_HZ=1000000;
const uint64_t DEBOUNCE_TIME_US=50000;

// volatile bool buttonPendingBLE = false;
volatile bool stableButtonStateBLE = HIGH;


void IRAM_ATTR onTimer(){
    stableButtonStateBLE = digitalRead(BLEPin);

    if(stableButtonStateBLE==LOW){
        // buttonPendingBLE=true;
        Serial.println('hi');
    }

    gpio_intr_enable((gpio_num_t)BLEPin);
}


void IRAM_ATTR handleButtonPressBLE(){
    gpio_intr_disable((gpio_num_t)BLEPin);
    timerRestart(debounceTimer);
    timerStart(debounceTimer);
}
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]= {0x2C, 0x4F, 0xA2, 0x47, 0xA5, 0x76, 0x31, 0x84 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x9F, 0xFA, 0x44, 0x31, 0xFC, 0xE1, 0x7E, 0xB3};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x86, 0xE7, 0xE9, 0xDA, 0x3E, 0x6B, 0x62, 0xBA, 0x6A, 0x01, 0xFB, 0x15, 0xF8, 0x46, 0x53, 0x2D };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,                         // Hardwired Chip Select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,                         // Hardwired Reset Pin
    .dio = {26,34, LMIC_UNUSED_PIN},  // DIO0 = 26, DIO1 = 34
};
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        mydata[0] = LORAIsOn ? 1 : 0;

        // Send 1 byte of data
        LMIC_setTxData2(1, mydata, 1, 0);
        // Serial.println(F("Packet queued"));
        // Prepare upstream data transmission at the next possible time.
        // LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void loraWAN_setup(){
    Serial.println(F("Starting"));
    pinMode(18, OUTPUT);
    digitalWrite(18, HIGH);
    pinMode(23, OUTPUT);
    digitalWrite(23, HIGH);
    SPI.begin(5, 19, 27, 18);
    // #ifdef VCC_ENABLE
    // // For Pinoccio Scout boards
    // pinMode(VCC_ENABLE, OUTPUT);
    // digitalWrite(VCC_ENABLE, HIGH);
    // delay(1000);
    // #endif

    // LMIC init
    os_init_ex(&lmic_pins);
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    Serial.println("boomlakerd");
    LMIC_setLinkCheckMode(0);
    
    LMIC_setDrTxpow(DR_SF10, 14); // Set clean DataRate Spreading Factor 7 at safe 14dBm
    
    // Loosen timing synchronization matrix boundaries by 50%
    LMIC_setClockError(MAX_CLOCK_ERROR * 50 / 100); 
    
    // Enforce US915 Channel Sub-Band 2 (Channels 8-15)
    LMIC_selectSubBand(1);
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}



void BLE_setup() {
  pinMode(BLEPin,INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(BLEPin), handleButtonPress, FALLING);
  Serial.println("Starting BLE work!");
  BLEDevice::init("Secret Govt Materials");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ
                                       );
  pCharacteristic->setValue("ON");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}







void setup() {
  delay(3000);
Serial.begin(115200);
 BLE_setup();
 attachInterrupt(digitalPinToInterrupt(BLEPin),handleButtonPressBLE,FALLING);
 debounceTimer=timerBegin(TIMER_FREQ_HZ);
 timerAttachInterrupt(debounceTimer,&onTimer);
 timerAlarm(debounceTimer,DEBOUNCE_TIME_US,false,0);
 loraWAN_setup();
}


void loop() {
    os_runloop_once();
}



// #include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>
// // See the following for generating UUIDs:
// // https://www.uuidgenerator.net/
// #define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// #define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// int lastState=0;
// int buttonPin =25;
// bool ledIsOn=false;
// class MyServerCallbacks : public BLEServerCallbacks{
//   void onDisconnect(BLEServer *pServer){
//     Serial.println("Client Disconnected");
//     esp_restart();
//   }
// };
// BLECharacteristic *pCharacteristic;



// }
// myServerCallbacks 
// void loop() {
//   // put your main code here, to run repeatedly:
//   int buttonVal = digitalRead(buttonPin);
//   if (buttonVal==LOW&&lastState==HIGH&&ledIsOn==false){
//     while(digitalRead(buttonPin)==LOW){
//       Serial.println("button pushed down");
//     }
//     ledIsOn=true;
//     Serial.println("button puhshed");
//     Serial.println(lastState);
//     pCharacteristic->setValue("ON");
//     delay(500);
//   }
//   else if (buttonVal==LOW&&lastState==HIGH&&ledIsOn==true){
//     while(digitalRead(buttonPin)==LOW){
//       Serial.println("button pushed down");
//     }
//     ledIsOn=false;
//     Serial.println("button puhshed");
//     Serial.println(lastState);
//     pCharacteristic->setValue("OFF");
//     delay(500);
//   }
//   lastState=buttonVal;
// }
