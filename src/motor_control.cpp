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
    m.hoseHoldActivated = false;
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
    bool anyHoseMom = false;
    String bestHoseMomMac = "";
    uint8_t bestHoseMomSpd = 100;
    unsigned long newestHoseMomTs = 0;

    bool anyHoseToggle = false;
    String bestHoseMac = "";
    uint8_t bestHoseSpd = 100;
    unsigned long newestHoseTs = 0;

    bool anyToolToggle = false;
    String bestToolMac = "";
    uint8_t bestToolSpd = 20;
    unsigned long newestToolTs = 0;

    for (const auto& m : monitoredList) {
        if (m.btnType == 0) continue;
        bool isHoseGatt = (m.isHose && m.isGatt);
        if (m.momActive && isHoseGatt) {
            if (!anyHoseMom || m.btnPressStartMs >= newestHoseMomTs) {
                anyHoseMom = true; newestHoseMomTs = m.btnPressStartMs; bestHoseMomMac = m.address; bestHoseMomSpd = m.momSpeed;
            }
        }
        if (m.toggleActive) {
            if (isHoseGatt) {
                if (!anyHoseToggle || m.toggleActivatedMs >= newestHoseTs) {
                    anyHoseToggle = true; newestHoseTs = m.toggleActivatedMs; bestHoseMac = m.address; bestHoseSpd = m.speed;
                }
            } else {
                if (!anyToolToggle || m.speed > bestToolSpd || (m.speed == bestToolSpd && m.toggleActivatedMs >= newestToolTs)) {
                    anyToolToggle = true; newestToolTs = m.toggleActivatedMs; bestToolMac = m.address; bestToolSpd = m.speed;
                }
            }
        }
    }

    hoseMomentaryActive = anyHoseMom;
    activeHoseMomMac = bestHoseMomMac;
    if (anyHoseMom) hoseMomTargetSpeed = bestHoseMomSpd;

    hoseToggleDemandActive = anyHoseToggle;
    activeHoseToggleMac = bestHoseMac;
    if (anyHoseToggle) hoseToggleTargetSpeed = bestHoseSpd;

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

void clearOtherHoseButtonStates(const String& activeMac) {
    for (auto& other : monitoredList) {
        if (other.btnType > 0 && other.isHose && !other.address.equalsIgnoreCase(activeMac)) {
            other.toggleActive = false;
            other.toggleActivatedMs = 0;
            other.momActive = false;
            other.liveSpeedOverride = 0;
        }
    }
}

bool isAnyPrimaryDemandActive() {
    return hoseMomentaryActive || hoseToggleDemandActive || webUiDemandActive || btnToggleDemandActive || toolBattDemandActive;
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
    if (hoseMomentaryActive) { MonitoredDevice* s = findMonitoredByMac(activeHoseMomMac); return s ? s->liveSpeedOverride : 0; }
    if (hoseToggleDemandActive) { MonitoredDevice* s = findMonitoredByMac(activeHoseToggleMac); return s ? s->liveSpeedOverride : 0; }
    bool webWon = false;
    MonitoredDevice* stdDev = getWinningStandardDevice(&webWon);
    if (webWon) return webUiSpeedOverride;
    return stdDev ? stdDev->liveSpeedOverride : 0;
}

void setActiveLiveSpeedOverride(uint8_t spd) {
    uint8_t q = quantizeSpeed(spd);
    if (hoseMomentaryActive) { MonitoredDevice* s = findMonitoredByMac(activeHoseMomMac); if (s) s->liveSpeedOverride = q; }
    else if (hoseToggleDemandActive) { MonitoredDevice* s = findMonitoredByMac(activeHoseToggleMac); if (s) s->liveSpeedOverride = q; }
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

    if (hoseMomentaryActive) {
        wantOn = true;
        activePresetSpeed = hoseMomTargetSpeed;
        MonitoredDevice* s = findMonitoredByMac(activeHoseMomMac);
        targetSpeed = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : hoseMomTargetSpeed;
    } else if (hoseToggleDemandActive) {
        wantOn = true;
        activePresetSpeed = hoseToggleTargetSpeed;
        MonitoredDevice* s = findMonitoredByMac(activeHoseToggleMac);
        targetSpeed = (s && s->liveSpeedOverride > 0) ? s->liveSpeedOverride : hoseToggleTargetSpeed;
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
    if (hoseMomentaryActive || hoseToggleDemandActive) return true;
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

    bool isHoseGatt = (targetMon->isHose && targetMon->isGatt);
    const char* roleTag = isHoseGatt ? "PRIMARY TOGGLE" : "BTN TOGGLE";

    if (taps == 1) {
        if (!targetMon->toggleActive) {
            if (!isHoseGatt && isBlockedByActiveRun(mac, presetSpd)) {
                LOGF("[%s] Ignored press -> higher/equal speed run or Primary switch is already active (%s)\n", roleTag, mac.c_str());
                return;
            }
            if (!isHoseGatt && hoseClearActive) {
                LOGF("[%s] 1x Tap -> OFF (cleared evac run) (%s)\n", roleTag, mac.c_str());
                hoseClearActive = false;
                targetMon->liveSpeedOverride = 0;
                forceStopNow = true;
                demandDirty = true;
                return;
            }
            LOGF("[%s] 1x Tap -> %s at %u%% (%s)\n",
                 roleTag,
                 isHoseGatt ? "Takeover ON" : "ON",
                 presetSpd, mac.c_str());
            if (isHoseGatt) {
                clearOtherHoseButtonStates(mac);
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
                 isHoseGatt ? "Release OFF" : "OFF",
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
    } else if (taps == 2 && isHoseGatt) {
        if (isAnyPrimaryDemandActive() && !hoseClearActive && !hoseMomentaryActive) {
            LOGF("[%s] 2x Tap -> Speed +20%% (%s)\n", roleTag, mac.c_str());
            adjustLiveSpeed(+20);
        } else {
            LOGF("[%s] Ignored 2x Tap -> extractor is idle or in momentary mode (%s)\n", roleTag, mac.c_str());
        }
    } else if (taps >= 3 && isHoseGatt) {
        if (isAnyPrimaryDemandActive() && !hoseClearActive && !hoseMomentaryActive) {
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

    bool isHoseBtn = (m->isHose && m->isGatt);
    if (isHoseBtn && m->hoseHoldActivated) {
        m->hoseHoldActivated = false;
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
        m->hoseHoldActivated = false;
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

    bool isHoseBtn = (m->isHose && m->isGatt);

    if (m->btnPhase == BTN_IDLE) {
        m->btnPhase = BTN_HELD;
        m->btnPressStartMs = now;
        m->btnLastPacketMs = now;
        m->btnTapCount = 1;
        m->hoseHoldActivated = false;

        if (!isGatt) {
            executeToggleTapAction(mac, presetSpd, 1, now);
        } else if (isHoseBtn) {
            bool underlyingRunning = (webUiDemandActive || btnToggleDemandActive || toolBattDemandActive ||
                                      hoseToggleDemandActive || (hoseMomentaryActive && !activeHoseMomMac.equalsIgnoreCase(mac)));
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
        if (isHoseBtn && m->momActive) {
            m->momActive = false;
            m->liveSpeedOverride = 0;
            syncButtonDemandsFromMonitored();
            if (!isAnyPrimaryDemandActive() && !hoseClearActive) {
                forceStopNow = true;
                demandDirty = true;
            }
        }
        return;
    }

    if (m->btnPhase == BTN_HELD) {
        m->btnLastPacketMs = now;
        if (isHoseBtn && !m->hoseHoldActivated && elapsedMs(now, m->btnPressStartMs) >= BTN_MOMENTARY_TAP_MAX_MS) {
            m->hoseHoldActivated = true;
            m->btnTapCount = 0;
            if (m->toggleActive) {
                m->toggleActive = false;
                m->toggleActivatedMs = 0;
            }
            clearOtherHoseButtonStates(mac);
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
                if (m.isHose && m.isGatt) {
                    uint8_t taps = m.btnTapCount;
                    m.btnTapCount = 0;
                    m.btnPhase = BTN_IDLE;
                    if (taps == 1) {
                        if (!m.toggleActive) {
                            clearOtherHoseButtonStates(m.address);
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
                            m.momActive = false;
                            m.liveSpeedOverride = 0;
                            syncButtonDemandsFromMonitored();
                            if (!isAnyPrimaryDemandActive() && !hoseClearActive) {
                                forceStopNow = true;
                                demandDirty = true;
                            }
                        }
                        if (isAnyPrimaryDemandActive() && !hoseClearActive && !hoseMomentaryActive) {
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
