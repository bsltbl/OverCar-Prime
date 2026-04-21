/*
 * OVERCAR PRIME - CAR SIDE (ESP32) V2.8
 */

#include <BLEDevice.h>
#include <BLEUtils.h>

static BLEUUID serviceUUID("be15beef-6186-407e-8381-0bd89c4d8df4");
static BLEUUID charWriteUUID("be15bee1-6186-407e-8381-0bd89c4d8df4");
static BLEUUID charReadUUID("be15bee0-6186-407e-8381-0bd89c4d8df4");

String targetAddress = ""; 
bool connectionPending = false;
bool connected = false;
bool pcRecognized = false;
uint32_t lastIdent = 0;

BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharWrite = nullptr;

static void carNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
    if (len > 0) {
        Serial.print("RX_CAR=");
        for(int i = 0; i < len; i++) {
            if (pData[i] < 0x10) Serial.print("0");
            Serial.print(pData[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}

bool connectToAuto() {
    Serial.println("MSG=Suche Auto: " + targetAddress);
    BLEScan* pScan = BLEDevice::getScan();
    pScan->setActiveScan(true);
    BLEScanResults* results = pScan->start(3, false); 
    BLEAdvertisedDevice* target = nullptr;

    for (int i = 0; i < results->getCount(); i++) {
        BLEAdvertisedDevice device = results->getDevice(i);
        if (device.getAddress().toString() == targetAddress.c_str()) {
            target = new BLEAdvertisedDevice(device);
            break;
        }
    }

    if (!target) {
        Serial.println("MSG=Auto nicht gefunden.");
        pScan->clearResults();
        return false;
    }

    if (pClient == nullptr) pClient = BLEDevice::createClient();
    if (!pClient->connect(target)) {
        Serial.println("MSG=Verbindung fehlgeschlagen.");
        delete target; pScan->clearResults();
        return false;
    }

    BLERemoteService* rs = pClient->getService(serviceUUID);
    if (rs) {
        pRemoteCharWrite = rs->getCharacteristic(charWriteUUID);
        auto rc = rs->getCharacteristic(charReadUUID);
        if (rc) rc->registerForNotify(carNotifyCallback);
        connected = true;
        Serial.println("MSG=Verbindung zum Auto OK");
    }
    delete target; pScan->clearResults();
    return connected;
}

void setup() {
    Serial.begin(500000);
    BLEDevice::init("OverCar_Car_Node");
}

void loop() {
    if (!pcRecognized && millis() - lastIdent > 1000) {
        Serial.println("IDENT:OVERCAR_PRIME_CAR");
        lastIdent = millis();
    }

    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "ACK_PRIME") pcRecognized = true;
        else if (cmd == "IDENT?") Serial.println("IDENT:OVERCAR_PRIME_CAR");
        else if (cmd == "SCAN") {
            BLEScan* s = BLEDevice::getScan();
            BLEScanResults* res = s->start(3, false);
            for(int i=0; i<res->getCount(); i++) {
                BLEAdvertisedDevice d = res->getDevice(i);
                if (d.haveServiceUUID() && d.isAdvertisingService(serviceUUID)) {
                    uint8_t modelId = 0x00;
                    uint8_t vMaj = 0x00, vMin = 0x00;
                    
                    if (d.haveManufacturerData()) {
                        String mData = d.getManufacturerData();
                        if (mData.length() >= 4 && (uint8_t)mData[0] == 0xBE && (uint8_t)mData[1] == 0xEF) {
                            modelId = (uint8_t)mData[3];
                        }
                    }
                    
                    if (d.haveName()) {
                        String name = d.getName();
                        if (name.length() >= 3) {
                            vMaj = (uint8_t)name[1];
                            vMin = (uint8_t)name[2];
                        }
                    }
                    Serial.printf("DEV=%s|%02X|%02X|%02X\n", d.getAddress().toString().c_str(), modelId, vMaj, vMin);
                }
            }
            s->clearResults();
            Serial.println("SCAN_DONE");
        }
        else if (cmd.startsWith("CONN=")) {
            targetAddress = cmd.substring(5);
            connectionPending = true;
        }
        else if (cmd.startsWith("TX_CAR=")) {
            String hex = cmd.substring(7); hex.replace(" ", "");
            int len = hex.length() / 2;
            uint8_t buf[len];
            for (int i = 0; i < len; i++) buf[i] = strtol(hex.substring(i*2, i*2+2).c_str(), NULL, 16);
            if (connected && pRemoteCharWrite) pRemoteCharWrite->writeValue(buf, len, false);
        }
    }

    if (connectionPending && !connected) {
        connectToAuto();
        connectionPending = false;
    }
}