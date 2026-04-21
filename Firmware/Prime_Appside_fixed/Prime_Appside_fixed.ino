/*
 * OVERCAR PRIME - APP SIDE (ESP32) V2.8.4
 * Korrektur: Batterie-Daten werden absolut 1:1 durchgereicht.
 * Optimiert: Höhere Puffer-Stabilität für lange Pakete (0x48, 0x30).
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID        "be15beef-6186-407e-8381-0bd89c4d8df4"
#define CHAR_UUID_WRITE     "be15bee1-6186-407e-8381-0bd89c4d8df4"
#define CHAR_UUID_READ      "be15bee0-6186-407e-8381-0bd89c4d8df4"

BLECharacteristic *pNotifyChar;
BLEAdvertising *pAdvertising;
bool deviceConnected = false;
bool pcRecognized = false;
uint32_t lastIdent = 0;

uint8_t spoofModelID = 0x13; 
bool spoofVersionEnabled = false;
uint8_t spMaj = 0x5A, spMin = 0x2E, spBld = 0x01;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        uint8_t* pData = pChar->getData();
        size_t len = pChar->getLength();
        Serial.print("RX_APP=");
        for(int i = 0; i < len; i++) {
            if (pData[i] < 0x10) Serial.print("0");
            Serial.print(pData[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
};

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pS) { deviceConnected = true; Serial.println("MSG=Handy verbunden"); }
    void onDisconnect(BLEServer* pS) { deviceConnected = false; if(pAdvertising) pAdvertising->start(); Serial.println("MSG=Handy getrennt"); }
};

void updateAdvertising() {
    if(pAdvertising) pAdvertising->stop();
    
    uint8_t adv[] = {
        0x02, 0x01, 0x06, 
        0x11, 0x07, 0xF4, 0x8D, 0x4D, 0x9C, 0xD8, 0x0B, 0x81, 0x83, 0x7E, 0x40, 0x86, 0x61, 0xEF, 0xBE, 0x15, 0xBE, 
        0x09, 0xFF, 0xBE, 0xEF, 0x00, spoofModelID, 0x0E, 0xA3, 0x2B, 0x9F
    };
    
    uint8_t vMaj = spoofVersionEnabled ? spMaj : 0x5A;
    uint8_t vMin = spoofVersionEnabled ? spMin : 0x2E;
    uint8_t vBld = spoofVersionEnabled ? spBld : 0x01;

    uint8_t scn[] = {
        0x0F, 0x09, 
        0x50, vMaj, vMin, vBld, 
        0x20, 0x20, 0x20, 0x20, 
        0x44, 0x72, 0x69, 0x76, 0x65, 0x00, 
        0x02, 0x0A, 0x00
    };
    
    BLEAdvertisementData oAdvData;
    oAdvData.addData(String((char*)adv, 31));
    
    BLEAdvertisementData oScanResponse;
    oScanResponse.addData(String((char*)scn, 19));
    
    pAdvertising->setAdvertisementData(oAdvData);
    pAdvertising->setScanResponseData(oScanResponse);
    pAdvertising->start();
}

void setup() {
    Serial.begin(500000);
    // Erhöhe seriellen Puffer für lange Datenpakete
    Serial.setRxBufferSize(2048); 
    
    BLEDevice::init("OverCar_App_Node");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    BLECharacteristic *pW = pService->createCharacteristic(CHAR_UUID_WRITE, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    pW->setCallbacks(new MyCallbacks());
    
    pNotifyChar = pService->createCharacteristic(CHAR_UUID_READ, BLECharacteristic::PROPERTY_NOTIFY);
    pNotifyChar->addDescriptor(new BLE2902());
    
    pService->start();
    pAdvertising = BLEDevice::getAdvertising();
    updateAdvertising();
}

void loop() {
    if (!pcRecognized && millis() - lastIdent > 1000) {
        Serial.println("IDENT:OVERCAR_PRIME_APP");
        lastIdent = millis();
    }

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd == "ACK_PRIME") {
            pcRecognized = true;
        }
        else if (cmd == "IDENT?") {
            Serial.println("IDENT:OVERCAR_PRIME_APP");
        }
        else if (cmd.startsWith("ID=")) {
            spoofModelID = strtol(cmd.substring(3).c_str(), NULL, 16);
            updateAdvertising();
        }
        else if (cmd.startsWith("V_SPOOF=")) {
            String data = cmd.substring(8);
            spoofVersionEnabled = (data.charAt(0) == '1');
            spMaj = strtol(data.substring(2, 4).c_str(), NULL, 16);
            spMin = strtol(data.substring(5, 7).c_str(), NULL, 16);
            updateAdvertising();
        }
        else if (cmd.startsWith("TX_APP=")) {
            String hex = cmd.substring(7);
            int hexLen = hex.length();
            uint8_t buf[40]; // Größerer Puffer für lange Nachrichten (0x48 etc.)
            int bufIdx = 0;

            // Schnelle Konvertierung von "XX XX XX " zu Byte-Array
            for (int i = 0; i + 1 < hexLen; i += 3) {
                if (bufIdx >= 40) break;
                char high = hex[i];
                char low = hex[i+1];
                
                uint8_t hVal = (high >= 'a') ? (high - 'a' + 10) : (high >= 'A') ? (high - 'A' + 10) : (high - '0');
                uint8_t lVal = (low >= 'a') ? (low - 'a' + 10) : (low >= 'A') ? (low - 'A' + 10) : (low - '0');
                
                buf[bufIdx++] = (hVal << 4) | lVal;
            }
            
            if (deviceConnected && pNotifyChar && bufIdx > 0) {
                // Nur Version Spoof (falls aktiv), Batterie 1:1 durchgereicht
                if (bufIdx >= 5 && buf[1] == 0x12 && spoofVersionEnabled) {
                    buf[2] = spMaj; buf[3] = spMin;
                }
                
                pNotifyChar->setValue(buf, bufIdx);
                pNotifyChar->notify();
            }
        }
    }

    static uint32_t lastTele = 0;
    if (pcRecognized && millis() - lastTele > 500) {
        uint8_t tele[6] = {0xAA, 0x01, spoofModelID, (uint8_t)(deviceConnected ? 0x80 : 0x00), 100, 0x00};
        tele[5] = tele[2] ^ tele[3]; Serial.write(tele, 6); lastTele = millis();
    }
}