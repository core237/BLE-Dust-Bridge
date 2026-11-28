#include "app_state.h"
#include "motor_control.h"
#include "fmp_protocol.h"
#include "mqtt_manager.h"

bool isAnyButtonGestureBusy() {
    for (const auto& m : monitoredList) {
        if (m.btnType > 0 && m.btnPhase != BTN_IDLE) return true;
    }
    return false;
}

void resetMonitoredButtonGesture(MonitoredDevice& m, bool clearLatched) {
    m.btnPhase = BTN_IDLE;
    m.btnTapCount = 0;
    m.primaryHoldActivated = false;
    m.wakePreTriggered = false;
    if (clearLatched) {
        m.btnLastSeqByte = -1;
    }
    m.momActive = false;
    m.liveSpeedOverride = 0;
    if (clearLatched) {
        m.toggleActive = false;
        m.toggleActivatedMs = 0;
        m.toolActive = false;
        m.toolLastSeen = 0;
    }
}

void resetAllMonitoredButtonGestures(bool clearLatched) {
    for (auto& m : monitoredList) {
        if (m.btnType > 0) resetMonitoredButtonGesture(m, clearLatched);
    }
}

void syncButtonDemandsFromMonitored() {
    bool anyPrimaryMom = false;
    String bestPrimaryMomMac = "";
    uint8_t bestPrimaryMomSpd = 100;
    unsigned long newestPrimaryMomTs = 0;

    bool anyPrimaryToggle = false;
    String bestPrimaryMac = "";
    uint8_t bestPrimarySpd = 100;
    unsigned long newestPrimaryTs = 0;

    bool anyToolToggle = false;
    String bestToolMac = "";
    uint8_t bestToolSpd = 20;
    unsigned long newestToolTs = 0;

    for (const auto& m : monitoredList) {
        if (m.btnType == 0) continue;
        bool isPrimaryGatt = ((m.isPrimary || !m.canAdv) && m.isGatt);
        if (m.momActive && isPrimaryGatt) {
            if (!anyPrimaryMom || m.btnPressStartMs >= newestPrimaryMomTs) {
                anyPrimaryMom = true; newestPrimaryMomTs = m.btnPressStartMs; bestPrimaryMomMac = m.address; bestPrimaryMomSpd = m.momSpeed;
            }
        }
        if (m.toggleActive) {
            if (isPrimaryGatt) {
                if (!anyPrimaryToggle || m.toggleActivatedMs >= newestPrimaryTs) {
                    anyPrimaryToggle = true; newestPrimaryTs = m.toggleActivatedMs; bestPrimaryMac = m.address; bestPrimarySpd = m.speed;
                }
            } else {
                if (!anyToolToggle || m.speed > bestToolSpd || (m.speed == bestToolSpd && m.toggleActivatedMs >= newestToolTs)) {
                    anyToolToggle = true; newestToolTs = m.toggleActivatedMs; bestToolMac = m.address; bestToolSpd = m.speed;
                }
            }
        }
    }

    primaryMomentaryActive = anyPrimaryMom;
    activePrimaryMomMac = bestPrimaryMomMac;
    if (anyPrimaryMom) primaryMomTargetSpeed = bestPrimaryMomSpd;

    primaryToggleDemandActive = anyPrimaryToggle;
    activePrimaryToggleMac = bestPrimaryMac;
    if (anyPrimaryToggle) primaryToggleTargetSpeed = bestPrimarySpd;

    btnToggleDemandActive = anyToolToggle;
    activeBtnToggleMac = bestToolMac;
    if (anyToolToggle) btnToggleTargetSpeed = bestToolSpd;
}

void syncToolBattDemandFromMonitored(unsigned long now) {
    bool anyTool = false;
    String bestMac = "";
    uint8_t bestSpd = 20;
    unsigned long bestTs = 0;

    for (const auto& m : monitoredList) {
        if (m.btnType == 0 && m.toolActive && (m.toolLastSeen == 0 || elapsedMs(now, m.toolLastSeen) < 12000UL)) {
            if (!anyTool || m.speed > bestSpd || (m.speed == bestSpd && m.toolLastSeen >= bestTs)) {
                anyTool = true;
                bestMac = m.address;
                bestSpd = m.speed;
                bestTs = m.toolLastSeen;
            }
        }
    }

    if (anyTool) {
        if (!toolBattDemandActive || !activeToolBattMac.equalsIgnoreCase(bestMac) || toolBattTargetSpeed != bestSpd) {
            demandDirty = true;
        }
        toolBattDemandActive = true;
        activeToolBattMac = bestMac;
        toolBattTargetSpeed = bestSpd;
    } else if (toolBattDemandActive) {
        toolBattDemandActive = false;
        activeToolBattMac = "";
        demandDirty = true;
    }
}

void clearAllButtonToggles() {
    resetAllMonitoredButtonGestures(true);
    syncButtonDemandsFromMonitored();
}

void clearOtherPrimaryButtonStates(const String& activeMac) {
    for (auto& other : monitoredList) {
        if (other.btnType > 0 && other.isPrimary && !other.address.equalsIgnoreCase(activeMac)) {
            other.toggleActive = false;
            other.toggleActivatedMs = 0;
            other.momActive = false;
            other.liveSpeedOverride = 0;
        }
    }
}

bool isAnyPrimaryDemandActive() {
    return primaryMomentaryActive || primaryToggleDemandActive || webUiDemandActive || btnToggleDemandActive || toolBattDemandActive;
}

MonitoredDevice* getWinningStandardDevice(bool* isWebUiWon) {
    if (isWebUiWon) *isWebUiWon = false;
    uint8_t bestSpd = 0;
    MonitoredDevice* bestDev = nullptr;
    bool webWon = false;

    if (toolBattDemandActive) {
        MonitoredDevice* s = findMonitoredByMac(activeToolBattMac);
        uint8_t eff = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : toolBattTargetSpeed;
        if (eff >= bestSpd) { bestSpd = eff; bestDev = s; webWon = false; }
    }
    if (btnToggleDemandActive) {
        MonitoredDevice* s = findMonitoredByMac(activeBtnToggleMac);
        uint8_t eff = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : btnToggleTargetSpeed;
        if (eff >= bestSpd) { bestSpd = eff; bestDev = s; webWon = false; }
    }
    if (webUiDemandActive) {
        uint8_t eff = (webUiSpeedOverride > 0) ? webUiSpeedOverride : userManualSpeed;
        if (eff >= bestSpd) { bestSpd = eff; bestDev = nullptr; webWon = true; }
    }
    if (isWebUiWon) *isWebUiWon = webWon;
    return bestDev;
}

uint8_t getActiveLiveSpeedOverride() {
    if (primaryMomentaryActive) { MonitoredDevice* s = findMonitoredByMac(activePrimaryMomMac); return s ? s->liveSpeedOverride : 0; }
    if (primaryToggleDemandActive) { MonitoredDevice* s = findMonitoredByMac(activePrimaryToggleMac); return s ? s->liveSpeedOverride : 0; }
    bool webWon = false;
    MonitoredDevice* stdDev = getWinningStandardDevice(&webWon);
    if (webWon) return webUiSpeedOverride;
    return stdDev ? stdDev->liveSpeedOverride : 0;
}

void setActiveLiveSpeedOverride(uint8_t spd) {
    uint8_t q = quantizeSpeed(spd);
    if (primaryMomentaryActive) { MonitoredDevice* s = findMonitoredByMac(activePrimaryMomMac); if (s) s->liveSpeedOverride = q; }
    else if (primaryToggleDemandActive) { MonitoredDevice* s = findMonitoredByMac(activePrimaryToggleMac); if (s) s->liveSpeedOverride = q; }
    else {
        bool webWon = false;
        MonitoredDevice* stdDev = getWinningStandardDevice(&webWon);
        if (webWon) webUiSpeedOverride = q;
        else if (stdDev) stdDev->liveSpeedOverride = q;
    }
}

void adjustLiveSpeed(int delta) {
    uint8_t base = getActiveLiveSpeedOverride();
    if (base == 0) base = currentSuctionPct;
    int next = (int)base + delta;
    uint8_t q = quantizeSpeed((uint8_t)(next < 20 ? 20 : (next > 100 ? 100 : next)));
    setActiveLiveSpeedOverride(q);
    LOGF("[LIVE SPEED] %s -> %u%%\n", delta > 0 ? "+20%" : "-20%", q);
    demandDirty = true;
}

void evaluateMotorDemand() {
    bool wantOn = false;
    uint8_t targetSpeed = 20;

    if (primaryMomentaryActive) {
        wantOn = true;
        activePresetSpeed = primaryMomTargetSpeed;
        MonitoredDevice* s = findMonitoredByMac(activePrimaryMomMac);
        targetSpeed = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : primaryMomTargetSpeed;
    } else if (primaryToggleDemandActive) {
        wantOn = true;
        activePresetSpeed = primaryToggleTargetSpeed;
        MonitoredDevice* s = findMonitoredByMac(activePrimaryToggleMac);
        targetSpeed = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : primaryToggleTargetSpeed;
    } else if (webUiDemandActive || btnToggleDemandActive || toolBattDemandActive) {
        wantOn = true;
        uint8_t bestSpd = 0;
        uint8_t bestPreset = 20;
        if (toolBattDemandActive) {
            MonitoredDevice* s = findMonitoredByMac(activeToolBattMac);
            uint8_t eff = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : toolBattTargetSpeed;
            if (eff >= bestSpd) { bestSpd = eff; bestPreset = toolBattTargetSpeed; }
        }
        if (btnToggleDemandActive) {
            MonitoredDevice* s = findMonitoredByMac(activeBtnToggleMac);
            uint8_t eff = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : btnToggleTargetSpeed;
            if (eff >= bestSpd) { bestSpd = eff; bestPreset = btnToggleTargetSpeed; }
        }
        if (webUiDemandActive) {
            uint8_t eff = (webUiSpeedOverride > 0) ? webUiSpeedOverride : userManualSpeed;
            if (eff >= bestSpd) { bestSpd = eff; bestPreset = userManualSpeed; }
        }
        activePresetSpeed = bestPreset;
        targetSpeed = bestSpd;
    } else if (hoseClearActive) {
        wantOn = true;
        targetSpeed = 20;
    }

    if (!wantOn) {
        bool fStop = forceStopNow;
        forceStopNow = false;
        stopExtractorMotor(fStop);
        return;
    }

    forceStopNow = false;
    targetSpeed = quantizeSpeed(targetSpeed);
    if (connState == STATE_CONNECTED) {
        if (currentPowerState != 1 || currentSuctionPct != targetSpeed) {
            if (sendFmpCtControl(1, targetSpeed, 1)) {
                currentPowerState = 1;
                currentSuctionPct = targetSpeed;
                publishExtractorTelemetry(millis());
                LOGF("[EXTRACT] Motor ON -> %u%%\n", targetSpeed);
            }
        }
    }
}

static uint8_t getCurrentStandardDemandSpeed() {
    uint8_t maxSpd = 0;
    if (webUiDemandActive) {
        uint8_t s = (webUiSpeedOverride > 0) ? webUiSpeedOverride : userManualSpeed;
        if (s > maxSpd) maxSpd = s;
    }
    if (btnToggleDemandActive) {
        MonitoredDevice* m = findMonitoredByMac(activeBtnToggleMac);
        uint8_t s = (m && m->liveSpeedOverride > 0) ? m->liveSpeedOverride : btnToggleTargetSpeed;
        if (s > maxSpd) maxSpd = s;
    }
    if (toolBattDemandActive) {
        MonitoredDevice* m = findMonitoredByMac(activeToolBattMac);
        uint8_t s = (m && m->liveSpeedOverride > 0) ? m->liveSpeedOverride : toolBattTargetSpeed;
        if (s > maxSpd) maxSpd = s;
    }
    return maxSpd;
}

static bool isBlockedByActiveRun(const String& mac, uint8_t reqSpd) {
    if (primaryMomentaryActive || primaryToggleDemandActive) return true;
    uint8_t activeStdSpd = getCurrentStandardDemandSpeed();
    if (activeStdSpd > 0) {
        return (reqSpd < activeStdSpd);
    }
    for (const auto& other : monitoredList) {
        if (other.address.equalsIgnoreCase(mac)) continue;
        if (other.btnType > 0 && (other.toggleActive || other.momActive)) return true;
    }
    return false;
}

void executeToggleTapAction(const String& mac, uint8_t presetSpd, uint8_t taps, unsigned long now) {
    MonitoredDevice* targetMon = findMonitoredByMac(mac);
    if (!targetMon) return;

    bool isPrimaryGatt = ((targetMon->isPrimary || !targetMon->canAdv) && targetMon->isGatt);
    const char* roleTag = isPrimaryGatt ? "PRIMARY TOGGLE" : "BTN TOGGLE";

    if (taps == 1) {
        if (!targetMon->toggleActive) {
            if (!isPrimaryGatt && isBlockedByActiveRun(mac, presetSpd)) {
                LOGF("[%s] Ignored press -> higher/equal speed run or Primary switch is already active (%s)\n", roleTag, mac.c_str());
                return;
            }
            if (!isPrimaryGatt && hoseClearActive) {
                LOGF("[%s] 1x Tap -> OFF (cleared evac run) (%s)\n", roleTag, mac.c_str());
                hoseClearActive = false;
                targetMon->liveSpeedOverride = 0;
                forceStopNow = true;
                demandDirty = true;
                return;
            }
            LOGF("[%s] 1x Tap -> %s at %u%% (%s)\n",
                 roleTag,
                 isPrimaryGatt ? "Takeover ON" : "ON",
                 presetSpd, mac.c_str());
            if (isPrimaryGatt) {
                clearOtherPrimaryButtonStates(mac);
                webUiDemandActive = false;
                webUiSpeedOverride = 0;
            }
            targetMon->toggleActive = true;
            targetMon->toggleActivatedMs = now;
            targetMon->liveSpeedOverride = 0;
            syncButtonDemandsFromMonitored();
            hoseClearActive = false;
            demandDirty = true;
        } else {
            LOGF("[%s] 1x Tap -> %s (%s)\n",
                 roleTag,
                 isPrimaryGatt ? "Release OFF" : "OFF",
                 mac.c_str());
            targetMon->toggleActive = false;
            targetMon->toggleActivatedMs = 0;
            targetMon->liveSpeedOverride = 0;
            syncButtonDemandsFromMonitored();
            if (!isAnyPrimaryDemandActive()) {
                if (evacSeconds > 0) { LOGF("[%s] Entering %us debris evacuation at 20%%\n", roleTag, evacSeconds); hoseClearActive = true; hoseClearTimer = now; }
                else forceStopNow = true;
            }
            demandDirty = true;
        }
    } else if (taps == 2 && isPrimaryGatt) {
        if (isAnyPrimaryDemandActive() && !hoseClearActive && !primaryMomentaryActive) {
            LOGF("[%s] 2x Tap -> Speed +20%% (%s)\n", roleTag, mac.c_str());
            adjustLiveSpeed(+20);
        } else {
            LOGF("[%s] Ignored 2x Tap -> extractor is idle or in momentary mode (%s)\n", roleTag, mac.c_str());
        }
    } else if (taps >= 3 && isPrimaryGatt) {
        if (isAnyPrimaryDemandActive() && !hoseClearActive && !primaryMomentaryActive) {
            LOGF("[%s] 3x Tap -> Speed -20%% (%s)\n", roleTag, mac.c_str());
            adjustLiveSpeed(-20);
        } else {
            LOGF("[%s] Ignored 3x Tap -> extractor is idle or in momentary mode (%s)\n", roleTag, mac.c_str());
        }
    }
}

void onButtonRelease(const String& mac, unsigned long now) {
    MonitoredDevice* m = findMonitoredByMac(mac);
    if (!m || m->btnPhase == BTN_IDLE) return;

    bool isPrimaryBtn = ((m->isPrimary || !m->canAdv) && m->isGatt);
    if (isPrimaryBtn && m->primaryHoldActivated) {
        m->primaryHoldActivated = false;
        m->momActive = false;
        m->liveSpeedOverride = 0;
        m->btnPhase = BTN_IDLE;
        m->btnTapCount = 0;
        syncButtonDemandsFromMonitored();
        if (!isAnyPrimaryDemandActive()) {
            LOGF("[PRIMARY AUTO] Hold release -> OFF (%s)\n", mac.c_str());
            if (evacSeconds > 0) {
                LOGF("[PRIMARY AUTO] Entering %us debris evacuation at 20%%\n", evacSeconds);
                hoseClearActive = true;
                hoseClearTimer = now;
            } else {
                forceStopNow = true;
            }
        } else {
            LOGF("[PRIMARY AUTO] Hold release -> returning to underlying run (%s)\n", mac.c_str());
        }
        demandDirty = true;
        return;
    }

    if (m->isGatt) {
        if (m->btnTapCount == 0) m->btnTapCount = 1;
        m->btnPhase = BTN_WAIT_TAP;
        m->btnReleaseMs = now;
    } else {
        m->btnPhase = BTN_IDLE;
        m->btnTapCount = 0;
        m->primaryHoldActivated = false;
    }
}

void onButtonPacket(const String& mac, uint8_t presetSpd, unsigned long now, bool isGatt, int16_t seqByte) {
    MonitoredDevice* m = findMonitoredByMac(mac);
    if (!m || pairedMac.length() == 0) return;

    if (!isGatt && seqByte >= 0) {
        if (m->btnLastSeqByte >= 0 && seqByte != m->btnLastSeqByte && m->btnPhase == BTN_HELD) {
            onButtonRelease(mac, now);
        }
        m->btnLastSeqByte = seqByte;
    }

    bool isPrimaryBtn = ((m->isPrimary || !m->canAdv) && m->isGatt);

    if (m->btnPhase == BTN_IDLE) {
        m->btnPhase = BTN_HELD;
        m->btnPressStartMs = now;
        m->btnLastPacketMs = now;
        m->btnTapCount = 1;
        m->primaryHoldActivated = false;

        if (!isGatt) {
            executeToggleTapAction(mac, presetSpd, 1, now);
        } else if (isPrimaryBtn) {
            bool underlyingRunning = (webUiDemandActive || btnToggleDemandActive || toolBattDemandActive ||
                                      primaryToggleDemandActive || (primaryMomentaryActive && !activePrimaryMomMac.equalsIgnoreCase(mac)));
            if (!underlyingRunning && !m->toggleActive) {
                m->momActive = true;
                m->momSpeed = presetSpd;
                m->liveSpeedOverride = 0;
                hoseClearActive = false;
                syncButtonDemandsFromMonitored();
                demandDirty = true;
            }
        }
        return;
    }

    if (m->btnPhase == BTN_WAIT_TAP) {
        m->btnPhase = BTN_HELD;
        m->btnPressStartMs = now;
        m->btnLastPacketMs = now;
        m->btnTapCount++;
        return;
    }

    if (m->btnPhase == BTN_HELD) {
        m->btnLastPacketMs = now;
        if (isPrimaryBtn && m->btnTapCount <= 1 && !m->primaryHoldActivated && elapsedMs(now, m->btnPressStartMs) >= BTN_MOMENTARY_TAP_MAX_MS) {
            m->primaryHoldActivated = true;
            m->btnTapCount = 0;
            if (m->toggleActive) {
                m->toggleActive = false;
                m->toggleActivatedMs = 0;
            }
            clearOtherPrimaryButtonStates(mac);
            m->momActive = true;
            m->momSpeed = presetSpd;
            m->liveSpeedOverride = 0;
            hoseClearActive = false;
            syncButtonDemandsFromMonitored();
            demandDirty = true;
            LOGF("[PRIMARY AUTO] Hold confirmed -> Momentary at %u%% (%s)\n", presetSpd, mac.c_str());
        }
    }
}

void serviceButtonGestures(unsigned long now) {
    for (auto& m : monitoredList) {
        if (m.btnType == 0 || m.btnPhase == BTN_IDLE) continue;

        if (m.btnPhase == BTN_WAIT_TAP) {
            if (elapsedMs(now, m.btnReleaseMs) >= BTN_GATT_TAP_WINDOW_MS) {
                if ((m.isPrimary || !m.canAdv) && m.isGatt) {
                    uint8_t taps = m.btnTapCount;
                    m.btnTapCount = 0;
                    m.btnPhase = BTN_IDLE;
                    if (taps == 1) {
                        if (!m.toggleActive) {
                            clearOtherPrimaryButtonStates(m.address);
                            m.momActive = false;
                            m.toggleActive = true;
                            m.toggleActivatedMs = now;
                            m.liveSpeedOverride = 0;
                            syncButtonDemandsFromMonitored();
                            hoseClearActive = false;
                            demandDirty = true;
                            LOGF("[PRIMARY AUTO] 1x Tap -> Primary Override ON at %u%% (%s)\n", m.speed, m.address.c_str());
                        } else {
                            m.momActive = false;
                            m.toggleActive = false;
                            m.toggleActivatedMs = 0;
                            m.liveSpeedOverride = 0;
                            syncButtonDemandsFromMonitored();
                            if (!isAnyPrimaryDemandActive()) {
                                if (evacSeconds > 0) { LOGF("[PRIMARY AUTO] Entering %us debris evacuation at 20%%\n", evacSeconds); hoseClearActive = true; hoseClearTimer = now; }
                                else forceStopNow = true;
                                LOGF("[PRIMARY AUTO] 1x Tap -> OFF (%s)\n", m.address.c_str());
                            } else {
                                LOGF("[PRIMARY AUTO] 1x Tap -> Released Primary Override, returning to underlying run (%s)\n", m.address.c_str());
                            }
                            demandDirty = true;
                        }
                    } else if (taps >= 2) {
                        if (m.momActive) {
                            clearOtherPrimaryButtonStates(m.address);
                            m.momActive = false;
                            m.toggleActive = true;
                            m.toggleActivatedMs = now;
                            m.liveSpeedOverride = 0;
                            syncButtonDemandsFromMonitored();
                            hoseClearActive = false;
                            demandDirty = true;
                        }
                        if (isAnyPrimaryDemandActive() && !hoseClearActive && !primaryMomentaryActive) {
                            if (taps == 2) {
                                LOGF("[PRIMARY AUTO] 2x Tap -> Speed +20%% (%s)\n", m.address.c_str());
                                adjustLiveSpeed(+20);
                            } else {
                                LOGF("[PRIMARY AUTO] 3x Tap -> Speed -20%% (%s)\n", m.address.c_str());
                                adjustLiveSpeed(-20);
                            }
                        } else {
                            LOGF("[PRIMARY AUTO] Ignored %ux Tap -> extractor is idle or in momentary mode (%s)\n", taps, m.address.c_str());
                        }
                    }
                    continue;
                }
                if (m.btnTapCount > 0) executeToggleTapAction(m.address, m.speed, m.btnTapCount, now);
                m.btnTapCount = 0;
                m.btnPhase = BTN_IDLE;
            }
            continue;
        }
        if (m.btnPhase == BTN_HELD && !m.isGatt) {
            if (elapsedMs(now, m.btnLastPacketMs) > ADV_TOGGLE_TAP_RELEASE_MS) onButtonRelease(m.address, now);
        }
    }
}
