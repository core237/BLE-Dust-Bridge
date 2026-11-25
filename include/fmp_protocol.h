#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

void resetRxCobsBuffer();
void onFe02Notify(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify);
bool sendFmpRawPayloadTo(uint8_t routeByte, const uint8_t* protoBytes, size_t protoLen);
bool sendFmpRawPayload(const uint8_t* protoBytes, size_t protoLen);
bool sendFmpCtControl(uint8_t powerState, uint8_t suctionPct = 0, uint8_t origin = 1);
bool sendFmpIdentify(bool enable = true);
void sendFmpHandshake();
void startTelemetryPoll(bool fullRefresh = true);
void serviceTelemetryPoll(unsigned long now);
void stopExtractorMotor(bool forceSend = false);
