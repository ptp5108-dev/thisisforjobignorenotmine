#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
int lastState=0;
int buttonPin =25;
bool ledIsOn=false;
class MyServerCallbacks : public BLEServerCallbacks{
  void onDisconnect(BLEServer *pServer){
    Serial.println("Client Disconnected");
    esp_restart();
  }
};
BLECharacteristic *pCharacteristic;

// myServerCallbacks 
void setup() {
  Serial.begin(115200);
  pinMode(buttonPin,INPUT_PULLUP);
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

void loop() {
  // put your main code here, to run repeatedly:
  int buttonVal = digitalRead(buttonPin);
  if (buttonVal==LOW&&lastState==HIGH&&ledIsOn==false){
    while(digitalRead(buttonPin)==LOW){
      Serial.println("button pushed down");
    }
    ledIsOn=true;
    Serial.println("button puhshed");
    Serial.println(lastState);
    pCharacteristic->setValue("ON");
    delay(500);
  }
  else if (buttonVal==LOW&&lastState==HIGH&&ledIsOn==true){
    while(digitalRead(buttonPin)==LOW){
      Serial.println("button pushed down");
    }
    ledIsOn=false;
    Serial.println("button puhshed");
    Serial.println(lastState);
    pCharacteristic->setValue("OFF");
    delay(500);
  }
  lastState=buttonVal;
}