#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

void setCoexScanner();
int findGattSlotByMac(const String& macStr);
bool isGattMacConnectedOrBusy(const String& macStr);
bool isGattPending(const String& macStr);
void enqueueGattConnect(const String& macStr, uint8_t addrType, uint8_t bType, uint8_t bSpd, bool forceImmediate = false, bool wakePreTrig = false);
void disconnectGattButtonMac(const String& macStr);
void serviceGattButton(unsigned long now);

void drainScanEvents(unsigned long now);
void startBleConnect(const String& macStr, uint8_t forcedType = 255);
void finishBleConnect(unsigned long now);
void initBleSubsystem();

void noteWebUiActivity();
