#pragma once
#include <Arduino.h>
#include "app_state.h"

bool isAnyButtonGestureBusy();
void resetMonitoredButtonGesture(MonitoredDevice& m, bool clearLatched = true);
void resetAllMonitoredButtonGestures(bool clearLatched = true);
void syncButtonDemandsFromMonitored();
void syncToolBattDemandFromMonitored(unsigned long now);
void clearAllButtonToggles();
void clearOtherHoseButtonStates(const String& activeMac);

bool isAnyPrimaryDemandActive();
MonitoredDevice* getWinningStandardDevice(bool* isWebUiWon = nullptr);
uint8_t getActiveLiveSpeedOverride();
void setActiveLiveSpeedOverride(uint8_t spd);
void adjustLiveSpeed(int delta);
void evaluateMotorDemand();

void executeToggleTapAction(const String& mac, uint8_t presetSpd, uint8_t taps, unsigned long now);
void onButtonRelease(const String& mac, unsigned long now);
void onButtonPacket(const String& mac, uint8_t presetSpd, unsigned long now, bool isGatt = false, int16_t seqByte = -1);
void serviceButtonGestures(unsigned long now);
