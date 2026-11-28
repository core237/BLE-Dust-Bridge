#include <Update.h>
#include <esp_coexist.h>
#include "app_state.h"
#include "ble_manager.h"
#include "fmp_protocol.h"
#include "motor_control.h"
#include "mqtt_manager.h"
#include "storage.h"

// Track last Web UI /api/status poll so Wi-Fi gets balanced airtime only while a client is active
static volatile unsigned long lastWebUiActivityMs = 0;
void noteWebUiActivity() {
    lastWebUiActivityMs = millis();
}

static ScanEvent scanQueue[SCAN_QUEUE_CAP];
static volatile size_t scanQueueHead = 0;
static volatile size_t scanQueueTail = 0;
static portMUX_TYPE scanQueueMux = portMUX_INITIALIZER_UNLOCKED;

static void pushScanEvent(const ScanEvent& ev) {
    portENTER_CRITICAL(&scanQueueMux);
    size_t nextHead = (scanQueueHead + 1) % SCAN_QUEUE_CAP;
    if (nextHead == scanQueueTail) scanQueueTail = (scanQueueTail + 1) % SCAN_QUEUE_CAP;
    scanQueue[scanQueueHead] = ev;
    scanQueueHead = nextHead;
    portEXIT_CRITICAL(&scanQueueMux);
}

static bool popScanEvent(ScanEvent& out) {
    bool has = false;
    portENTER_CRITICAL(&scanQueueMux);
    if (scanQueueTail != scanQueueHead) {
        out = scanQueue[scanQueueTail];
        scanQueueTail = (scanQueueTail + 1) % SCAN_QUEUE_CAP;
        has = true;
    }
    portEXIT_CRITICAL(&scanQueueMux);
    return has;
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* client) override { (void)client; connState = STATE_CONNECTING; extLinkConnectedLog.store(true); }
    void onDisconnect(NimBLEClient* client, int reason) override {
        (void)client; (void)reason; connState = STATE_DISCONNECTED; currentPowerState = 3;
        pairedRssi = 0;
        pWriteChar = nullptr; pNotifyChar = nullptr; resetRxCobsBuffer();
        initialTelemDone = false;
        postConnectActionPending = false;
        telemPollRequested = false; telemAwaitingReply = false; extLinkDisconnectedFlag.store(true);
    }
};

static uint32_t globalGattConnGen = 1;

struct GattConnTaskParams {
    int slotIdx;
    uint32_t connGen;
    NimBLEClient* client;
    char mac[18];
    uint8_t addrType;
};
static std::atomic<bool> gattBtnTaskDone{false};
static std::atomic<bool> gattBtnTaskOk{false};
static int gattBtnTaskSlotIdx = -1;
static uint32_t gattBtnTaskConnGen = 0;

struct GattPendingReq { bool active; String mac; uint8_t addrType; uint8_t btnType; uint8_t speed; bool wakePreTrig; };
static GattPendingReq gattPending[MAX_GATT_PENDING];

struct GattBackoffEntry { String mac; unsigned long untilMs; };
static std::vector<GattBackoffEntry> gattBackoffs;
static unsigned long gattBtnLastTryMs = 0;

static void setGattBackoff(const String& macStr, unsigned long untilMs) {
    for (auto& b : gattBackoffs) if (b.mac.equalsIgnoreCase(macStr)) { b.untilMs = untilMs; return; }
    gattBackoffs.push_back({macStr, untilMs});
}
static void clearGattBackoff(const String& macStr) {
    for (auto it = gattBackoffs.begin(); it != gattBackoffs.end(); ) { if (it->mac.equalsIgnoreCase(macStr)) it = gattBackoffs.erase(it); else ++it; }
}
static bool isGattBackedOff(const String& macStr, unsigned long now) {
    for (auto it = gattBackoffs.begin(); it != gattBackoffs.end(); ) {
        if (now >= it->untilMs) it = gattBackoffs.erase(it);
        else if (it->mac.equalsIgnoreCase(macStr)) return true;
        else ++it;
    }
    return false;
}
static int findGattSlotByClient(NimBLEClient* pC) { for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) if (gattSlots[i].client == pC) return i; return -1; }
int findGattSlotByMac(const String& macStr) {
    int found = -1;
    portENTER_CRITICAL(&gattSlotMux);
    for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) {
        if (gattSlots[i].inUse && strcasecmp(gattSlots[i].mac, macStr.c_str()) == 0) {
            found = i;
            break;
        }
    }
    portEXIT_CRITICAL(&gattSlotMux);
    return found;
}
bool isGattMacConnectedOrBusy(const String& macStr) { int idx = findGattSlotByMac(macStr); return (idx >= 0) && (gattSlots[idx].connected || gattSlots[idx].connecting); }
bool isGattPending(const String& macStr) {
    for (int i = 0; i < MAX_GATT_PENDING; i++) {
        if (gattPending[i].active && gattPending[i].mac.equalsIgnoreCase(macStr)) return true;
    }
    return false;
}

void enqueueGattConnect(const String& macStr, uint8_t addrType, uint8_t bType, uint8_t bSpd, bool forceImmediate, bool wakePreTrig) {
    unsigned long now = millis();
    if (forceImmediate) clearGattBackoff(macStr);
    else if (isGattBackedOff(macStr, now)) return;
    if (isGattMacConnectedOrBusy(macStr)) return;
    for (int i = 0; i < MAX_GATT_PENDING; i++) {
        if (gattPending[i].active && gattPending[i].mac.equalsIgnoreCase(macStr)) {
            gattPending[i].addrType = addrType; gattPending[i].btnType = bType; gattPending[i].speed = bSpd;
            if (wakePreTrig) gattPending[i].wakePreTrig = true;
            return;
        }
    }
    for (int i = 0; i < MAX_GATT_PENDING; i++) if (!gattPending[i].active) { gattPending[i] = {true, macStr, addrType, bType, bSpd, wakePreTrig}; return; }
}

static void handleGattSlotNotify(int idx, NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length) {
    (void)pChr;
    if (idx < 0 || idx >= MAX_GATT_BTN_SLOTS || !pData || length == 0) return;
    GattBtnSlot& s = gattSlots[idx];
    unsigned long now = millis();
    char hexTmp[48];
    size_t pos = 0;
    for (size_t i = 0; i < length && pos + 4 < sizeof(hexTmp); i++) pos += snprintf(hexTmp + pos, sizeof(hexTmp) - pos, "%02X ", pData[i]);
    if (pos > 0 && hexTmp[pos - 1] == ' ') hexTmp[pos - 1] = '\0'; else hexTmp[pos] = '\0';
    bool pressed = false;
    for (size_t i = 0; i < length; i++) if (pData[i] != 0x00) { pressed = true; break; }
    if (length == 1 && pData[0] == '0') pressed = false;

    portENTER_CRITICAL(&gattSlotMux);
    s.lastActivityMs = now;
    s.gotAnyNotify = true;
    memcpy(s.lastHex, hexTmp, sizeof(s.lastHex));
    s.rxLogPressed = pressed;
    s.rxLogPending = true;
    if (pressed) {
        bool isNewPhysicalPress = (!s.pressed && (s.lastPressEdgeMs == 0 || elapsedMs(now, s.lastPressEdgeMs) >= 55UL));
        s.pressed = true;
        if (isNewPhysicalPress) {
            s.lastPressEdgeMs = now;
            if (s.pendingPressEdges < 8) s.pendingPressEdges = s.pendingPressEdges + 1;
        }
    } else {
        if (s.pressed) {
            s.pressed = false;
            s.releaseNow = true;
        }
    }
    portEXIT_CRITICAL(&gattSlotMux);
}

static void gattSlot0NotifyCB(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify) { (void)isNotify; handleGattSlotNotify(0, pChr, pData, length); }
static void gattSlot1NotifyCB(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify) { (void)isNotify; handleGattSlotNotify(1, pChr, pData, length); }

class GattBtnClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pC) override {
        int idx = findGattSlotByClient(pC);
        if (idx >= 0 && pC) {
            std::string peerStr = pC->getPeerAddress().toString();
            portENTER_CRITICAL(&gattSlotMux);
            if (gattSlots[idx].inUse && gattSlots[idx].mac[0] != '\0' &&
                strcasecmp(gattSlots[idx].mac, peerStr.c_str()) == 0) {
                gattSlots[idx].connected = true;
            }
            portEXIT_CRITICAL(&gattSlotMux);
        }
    }
    void onDisconnect(NimBLEClient* pC, int reason) override {
        (void)reason;
        int idx = findGattSlotByClient(pC);
        if (idx >= 0 && pC) {
            GattBtnSlot& s = gattSlots[idx];
            std::string peerStr = pC->getPeerAddress().toString();
            portENTER_CRITICAL(&gattSlotMux);
            if (s.inUse && s.mac[0] != '\0' && strcasecmp(s.mac, peerStr.c_str()) == 0) {
                s.connected = false;
                s.connecting = false;
                if (s.pressed) { s.pressed = false; s.releaseNow = true; }
                s.sleepDiscLogPending = true;
            }
            portEXIT_CRITICAL(&gattSlotMux);
        }
    }
};
static GattBtnClientCallbacks gattBtnClientCB;

static void gattBtnConnectWorkerTask(void* param) {
    GattConnTaskParams* p = static_cast<GattConnTaskParams*>(param);
    bool ok = false;
    if (p && p->client) {
        if (p->client->isConnected()) {
            p->client->disconnect();
            for (int w = 0; w < 20 && p->client->isConnected(); w++) {
                delay(10);
            }
        }
        NimBLEAddress targetAddr(p->mac, p->addrType);
        ok = p->client->connect(targetAddr, true);
    }
    delete p;
    gattBtnTaskOk.store(ok);
    gattBtnTaskRunning.store(false);
    gattBtnTaskDone.store(true);
    vTaskDelete(nullptr);
}

static void startGattButtonConnect(const String& macStr, uint8_t addrType, uint8_t bType, uint8_t bSpd, bool wakePreTrig = false) {
    (void)bType; (void)bSpd;
    if (gattBtnTaskRunning.load() || extConnTaskRunning.load()) return;
    gattBtnLastTryMs = millis();
    int existing = findGattSlotByMac(macStr);
    if (existing >= 0) {
        GattBtnSlot& s = gattSlots[existing];
        if (s.connecting) return;
        if (s.connected && s.client && s.client->isConnected()) return;
    }
    int slotIdx = existing;
    if (slotIdx < 0) for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) if (!gattSlots[i].connected && !gattSlots[i].connecting) { slotIdx = i; break; }
    if (slotIdx < 0) {
        unsigned long oldestMs = ULONG_MAX;
        for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) if (!gattSlots[i].connecting && !gattSlots[i].pressed && gattSlots[i].lastActivityMs < oldestMs) { oldestMs = gattSlots[i].lastActivityMs; slotIdx = i; }
        if (slotIdx >= 0) {
            char evictedMac[18] = {0};
            portENTER_CRITICAL(&gattSlotMux);
            if (gattSlots[slotIdx].inUse && gattSlots[slotIdx].mac[0] != '\0' &&
                strcasecmp(gattSlots[slotIdx].mac, macStr.c_str()) != 0) {
                strncpy(evictedMac, gattSlots[slotIdx].mac, sizeof(evictedMac) - 1);
            }
            portEXIT_CRITICAL(&gattSlotMux);
            if (evictedMac[0] != '\0') {
                MonitoredDevice* evictedMon = findMonitoredByMac(String(evictedMac));
                if (evictedMon) {
                    bool wasActive = evictedMon->toggleActive || evictedMon->momActive;
                    resetMonitoredButtonGesture(*evictedMon, true);
                    syncButtonDemandsFromMonitored();
                    if (wasActive && !isAnyPrimaryDemandActive() && !hoseClearActive) {
                        forceStopNow = true;
                        demandDirty = true;
                    }
                    LOGF("[GATT BTN] Evicted %s from Slot %d -> toggled OFF.\n", evictedMac, slotIdx);
                }
            }
        }
    }
    if (slotIdx < 0) return;

    addrType = inferRandomAddrType(macStr, addrType);

    NimBLEDevice::getScan()->stop();
    esp_coex_preference_set(ESP_COEX_PREFER_BT);

    GattBtnSlot& s = gattSlots[slotIdx];
    uint32_t gen = ++globalGattConnGen;
    portENTER_CRITICAL(&gattSlotMux);
    s.inUse = true; s.connecting = true; s.connected = false; s.connGen = gen;
    s.connectedAtMs = 0;
    strncpy(s.mac, macStr.c_str(), sizeof(s.mac) - 1);
    s.mac[sizeof(s.mac) - 1] = '\0';
    s.pressed = false; s.pendingPressEdges = 0; s.gotAnyNotify = false; s.releaseNow = false;
    s.wakePreTriggered = wakePreTrig;
    s.lastPressEdgeMs = 0; s.rxLogPending = false; s.sleepDiscLogPending = false;
    portEXIT_CRITICAL(&gattSlotMux);

    if (s.client && s.client->isConnected()) {
        s.client->disconnect();
    }

    if (!s.client) {
        s.client = NimBLEDevice::createClient();
        s.client->setClientCallbacks(&gattBtnClientCB, false);
        // Restored from Presep baseline: 4-event peripheral latency frees ~80% of radio slots for scanner
        s.client->setConnectionParams(24, 40, 4, 400);
        s.client->setConnectTimeout(1800);
    }

    LOGF("[GATT BTN] [Slot %d] Connecting to %s (addrType=%u)...\n", slotIdx, macStr.c_str(), addrType);

    GattConnTaskParams* taskParams = new GattConnTaskParams();
    taskParams->slotIdx = slotIdx;
    taskParams->connGen = gen;
    taskParams->client = s.client;
    strncpy(taskParams->mac, macStr.c_str(), sizeof(taskParams->mac) - 1);
    taskParams->mac[sizeof(taskParams->mac) - 1] = '\0';
    taskParams->addrType = addrType;

    gattBtnTaskSlotIdx = slotIdx;
    gattBtnTaskConnGen = gen;
    gattBtnTaskOk.store(false);
    gattBtnTaskDone.store(false);
    gattBtnTaskRunning.store(true);
    if (xTaskCreatePinnedToCore(gattBtnConnectWorkerTask, "gattBtnConn", 6144, taskParams, 1, nullptr, 1) != pdPASS) {
        delete taskParams;
        gattBtnTaskRunning.store(false);
        portENTER_CRITICAL(&gattSlotMux);
        s.connecting = false;
        s.connected = false;
        portEXIT_CRITICAL(&gattSlotMux);
        setCoexScanner();
    }
}

static void finishGattButtonConnect(unsigned long now) {
    int slotIdx = gattBtnTaskSlotIdx;
    uint32_t expectedGen = gattBtnTaskConnGen;
    bool ok = gattBtnTaskOk.load();
    if (slotIdx < 0 || slotIdx >= MAX_GATT_BTN_SLOTS) {
        setCoexScanner();
        return;
    }

    GattBtnSlot& s = gattSlots[slotIdx];
    char macBuf[18] = {0};
    portENTER_CRITICAL(&gattSlotMux);
    bool slotInUse = s.inUse && (s.connGen == expectedGen);
    strncpy(macBuf, s.mac, sizeof(macBuf) - 1);
    macBuf[sizeof(macBuf) - 1] = '\0';
    portEXIT_CRITICAL(&gattSlotMux);
    String macStr(macBuf);

    if (!slotInUse) {
        if (s.client && s.client->isConnected() && !s.inUse) s.client->disconnect();
        setCoexScanner();
        return;
    }

    if (!ok) {
        LOGF("[GATT BTN] [Slot %d] Connect failed for %s\n", slotIdx, macStr.c_str());
        bool wasWakePre = s.wakePreTriggered;
        portENTER_CRITICAL(&gattSlotMux);
        s.connecting = false; s.connected = false; s.wakePreTriggered = false;
        portEXIT_CRITICAL(&gattSlotMux);
        if (wasWakePre) {
            MonitoredDevice* monPtr = findMonitoredByMac(macStr);
            if (monPtr && monPtr->wakePreTriggered) {
                monPtr->wakePreTriggered = false;
                if (monPtr->btnPhase == BTN_HELD) {
                    unsigned long relTs = monPtr->isPrimary ? (monPtr->btnPressStartMs + 100UL) : now;
                    onButtonRelease(macStr, relTs);
                }
            }
        }
        gattBtnLastTryMs = now;
        setGattBackoff(macStr, now + 1200UL);
        setCoexScanner();
        return;
    }

    NimBLERemoteCharacteristic::notify_callback cb = (slotIdx == 0) ? gattSlot0NotifyCB : gattSlot1NotifyCB;
    int subCount = 0;
    NimBLERemoteService* pFfe0 = s.client->getService(NimBLEUUID((uint16_t)0xFFE0));
    if (pFfe0) {
        const auto& chars = pFfe0->getCharacteristics(true);
        for (auto* c : chars) if (c && (c->canNotify() || c->canIndicate()) && c->subscribe(c->canNotify(), cb)) { subCount++; LOGF("[GATT BTN] [Slot %d] Subscribed 0xFFE0 chr=%s\n", slotIdx, c->getUUID().toString().c_str()); }
    }
    if (subCount == 0) {
        NimBLERemoteService* pHid = s.client->getService(NimBLEUUID((uint16_t)0x1812));
        if (pHid) {
            const auto& chars = pHid->getCharacteristics(true);
            for (auto* c : chars) if (c && (c->canNotify() || c->canIndicate()) && c->subscribe(c->canNotify(), cb)) { subCount++; LOGF("[GATT BTN] [Slot %d] Subscribed HID chr=%s\n", slotIdx, c->getUUID().toString().c_str()); }
        }
    }
    if (subCount == 0) {
        const auto& svcs = s.client->getServices(true);
        for (auto* svc : svcs) {
            if (!svc) continue;
            NimBLEUUID u = svc->getUUID();
            if (u == NimBLEUUID((uint16_t)0x1800) || u == NimBLEUUID((uint16_t)0x1801) ||
                u == NimBLEUUID((uint16_t)0x180A) || u == NimBLEUUID((uint16_t)0x180F)) continue;
            const auto& chars = svc->getCharacteristics(true);
            for (auto* c : chars) {
                if (c && (c->canNotify() || c->canIndicate()) && c->subscribe(c->canNotify(), cb)) {
                    subCount++;
                    LOGF("[GATT BTN] [Slot %d] Subscribed svc=%s chr=%s\n", slotIdx, u.toString().c_str(), c->getUUID().toString().c_str());
                }
            }
        }
    }

    if (subCount == 0 && s.client->secureConnection()) {
        const auto& svcs = s.client->getServices(true);
        for (auto* svc : svcs) {
            if (!svc) continue;
            NimBLEUUID u = svc->getUUID();
            if (u == NimBLEUUID((uint16_t)0x1800) || u == NimBLEUUID((uint16_t)0x1801) ||
                u == NimBLEUUID((uint16_t)0x180A) || u == NimBLEUUID((uint16_t)0x180F)) continue;
            const auto& chars = svc->getCharacteristics(true);
            for (auto* c : chars) {
                if (c && (c->canNotify() || c->canIndicate()) && c->subscribe(c->canNotify(), cb)) {
                    subCount++;
                    LOGF("[GATT BTN] [Slot %d] Subscribed after bond svc=%s chr=%s\n", slotIdx, u.toString().c_str(), c->getUUID().toString().c_str());
                }
            }
        }
    }

    if (subCount == 0) {
        LOGF("[GATT BTN] [Slot %d] No notify/indicate characteristic found on %s; disconnecting.\n", slotIdx, macStr.c_str());
        for (auto& m : monitoredList) {
            if (m.address.equalsIgnoreCase(macStr) && m.canAdv) {
                m.isPrimary = false;
                m.isGatt = false;
                m.canGatt = false;
                m.gattReady = true;
                m.activeRaw = "ADV:STREAM";
                saveMonitoredToNvs();
                LOGF("[PRIMARY ROLE] %s has no GATT button service -> reverted to Passive switch.\n", macStr.c_str());
                break;
            }
        }
        s.client->disconnect();
        bool wasWakePre = s.wakePreTriggered;
        portENTER_CRITICAL(&gattSlotMux);
        s.connecting = false; s.connected = false; s.wakePreTriggered = false;
        portEXIT_CRITICAL(&gattSlotMux);
        if (wasWakePre) {
            MonitoredDevice* monPtr = findMonitoredByMac(macStr);
            if (monPtr && monPtr->wakePreTriggered) {
                monPtr->wakePreTriggered = false;
                if (monPtr->btnPhase == BTN_HELD) {
                    unsigned long relTs = monPtr->isPrimary ? (monPtr->btnPressStartMs + 100UL) : now;
                    onButtonRelease(macStr, relTs);
                }
            }
        }
        gattBtnLastTryMs = now;
        setGattBackoff(macStr, now + 1200UL);
        setCoexScanner();
        return;
    }

    clearGattBackoff(macStr);
    bool handedOffImmediately = false;
    bool wakeTapAlreadyReleased = false;
    portENTER_CRITICAL(&gattSlotMux);
    s.connected = true; s.connecting = false; s.connectedAtMs = now; s.lastRssiMs = now; s.lastActivityMs = now;
    if (s.wakePreTriggered) {
        if (s.pendingPressEdges > 0) {
            s.pendingPressEdges = s.pendingPressEdges - 1;
            handedOffImmediately = true;
        } else {
            wakeTapAlreadyReleased = true;
        }
        s.wakePreTriggered = false;
    }
    bool alreadyPressed = (s.pendingPressEdges > 0) || s.pressed;
    portEXIT_CRITICAL(&gattSlotMux);
    mqttDevicesDirty = true;
    for (auto& b : bleList) {
        if (b.address.equalsIgnoreCase(macStr)) {
            b.lastSeen = now;
            break;
        }
    }
    bool saveNvsNeeded = false;
    for (auto& m : monitoredList) {
        if (m.address.equalsIgnoreCase(macStr)) {
            bool wasInitialPair = !m.gattReady;
            m.isGatt = true;
            m.canGatt = true;
            m.gattReady = true;
            if (wasInitialPair) {
                saveNvsNeeded = true;
                m.waitReleaseAfterPair = alreadyPressed;
                m.pairHoldGuardUntilMs = now + 1200UL;
                if (alreadyPressed) {
                    LOGF("[GATT BTN] %s paired while held -> release button before arming.\n", macStr.c_str());
                } else {
                    LOGF("[GATT BTN] %s paired -> armed & ready.\n", macStr.c_str());
                }
            } else if (m.wakePreTriggered) {
                m.wakePreTriggered = false;
                if (handedOffImmediately) {
                    LOGF("[GATT BTN] %s wake pre-trigger handed off to live GATT link.\n", macStr.c_str());
                } else if (wakeTapAlreadyReleased) {
                    if (m.btnPhase == BTN_HELD) {
                        unsigned long relTs = m.isPrimary ? (m.btnPressStartMs + 100UL) : now;
                        onButtonRelease(macStr, relTs);
                    }
                    LOGF("[GATT BTN] %s quick wake tap completed on GATT connect.\n", macStr.c_str());
                }
            }
            break;
        }
    }
    if (saveNvsNeeded) {
        saveMonitoredToNvs();
    }
    setCoexScanner();
    LOGF("[GATT BTN] %s configured & ready.\n", macStr.c_str());
}

void disconnectGattButtonMac(const String& macStr) {
    for (int i = 0; i < MAX_GATT_PENDING; i++) if (gattPending[i].active && gattPending[i].mac.equalsIgnoreCase(macStr)) gattPending[i].active = false;
    clearGattBackoff(macStr);
    for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) {
        bool match = false;
        portENTER_CRITICAL(&gattSlotMux);
        match = (gattSlots[i].inUse && strcasecmp(gattSlots[i].mac, macStr.c_str()) == 0);
        portEXIT_CRITICAL(&gattSlotMux);
        if (match) {
            bool workerActiveOnSlot = (gattBtnTaskRunning.load() && gattBtnTaskSlotIdx == i);
            if (!workerActiveOnSlot && gattSlots[i].client && gattSlots[i].client->isConnected()) {
                gattSlots[i].client->disconnect();
            }
            portENTER_CRITICAL(&gattSlotMux);
            gattSlots[i].connGen = ++globalGattConnGen;
            gattSlots[i].connected = false;
            gattSlots[i].connecting = false;
            gattSlots[i].pressed = false;
            gattSlots[i].pendingPressEdges = 0;
            gattSlots[i].releaseNow = false;
            gattSlots[i].wakePreTriggered = false;
            gattSlots[i].lastPressEdgeMs = 0;
            gattSlots[i].inUse = false;
            gattSlots[i].mac[0] = '\0';
            portEXIT_CRITICAL(&gattSlotMux);
        }
    }
}

void serviceGattButton(unsigned long now) {
    if (gattBtnTaskDone.exchange(false)) {
        finishGattButtonConnect(now);
    }
    if (!requestConnect && !wifiScanInProgress && connState != STATE_CONNECTING && !extConnTaskRunning.load() && !gattBtnTaskRunning.load() && elapsedMs(now, gattBtnLastTryMs) >= 150UL) {
        for (int i = 0; i < MAX_GATT_PENDING; i++) {
            if (gattPending[i].active) {
                GattPendingReq req = gattPending[i]; gattPending[i].active = false;
                startGattButtonConnect(req.mac, req.addrType, req.btnType, req.speed, req.wakePreTrig);
                break;
            }
        }
    }
    for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) {
        GattBtnSlot& s = gattSlots[i];
        char hexCopy[48];
        char slotMacBuf[18] = {0};
        portENTER_CRITICAL(&gattSlotMux);
        strncpy(slotMacBuf, s.mac, sizeof(slotMacBuf) - 1);
        slotMacBuf[sizeof(slotMacBuf) - 1] = '\0';
        bool sleepDisc = s.sleepDiscLogPending;
        s.sleepDiscLogPending = false;
        bool rxLog = s.rxLogPending;
        s.rxLogPending = false;
        bool rxPressed = s.rxLogPressed;
        memcpy(hexCopy, s.lastHex, sizeof(hexCopy));
        uint8_t edges = s.pendingPressEdges;
        s.pendingPressEdges = 0;
        bool relNow = s.releaseNow;
        s.releaseNow = false;
        bool isPressedNow = s.pressed;
        bool isSlotConnected = (s.connected && !s.connecting && s.connectedAtMs > 0);
        unsigned long slotConnAtMs = s.connectedAtMs;
        bool slotGotAnyNotify = s.gotAnyNotify;
        portEXIT_CRITICAL(&gattSlotMux);
        String slotMac(slotMacBuf);
        if (sleepDisc) {
            mqttDevicesDirty = true;
        }

        MonitoredDevice* monPtr = findMonitoredByMac(slotMac);
        if (monPtr && !monPtr->canAdv && isSlotConnected && !slotGotAnyNotify && elapsedMs(now, slotConnAtMs) >= 600UL) {
            if (!monPtr->periodicBeacon) {
                monPtr->periodicBeacon = true;
                if (!needSaveMonitored) needSaveMonitoredAtMs = now;
                needSaveMonitored = true;
            }
        }
        if (isSlotConnected && elapsedMs(now, s.lastRssiMs) >= 5000UL) {
            s.lastRssiMs = now;
            for (auto& b : bleList) {
                if (b.address.equalsIgnoreCase(slotMac)) {
                    b.lastSeen = now;
                    break;
                }
            }
        }
        bool cfg = monPtr ? isMonitoredConfiguring(*monPtr, now) : false;
        if (rxLog) {
            if (cfg) LOGF("[GATT BTN %s] Ignored RX while Configuring: hex=[%s]\n", slotMac.c_str(), hexCopy);
            else LOGF("[GATT BTN %s] hex=[%s] -> %s\n", slotMac.c_str(), hexCopy, rxPressed ? "PRESSED (1)" : "RELEASED (0)");
        }
        if (cfg || !monPtr) {
            portENTER_CRITICAL(&gattSlotMux);
            s.pressed = false;
            s.pendingPressEdges = 0;
            s.releaseNow = false;
            portEXIT_CRITICAL(&gattSlotMux);
            continue;
        }

        if (rxLog && rxPressed && monPtr->periodicBeacon && isSlotConnected && elapsedMs(now, slotConnAtMs) < 600UL) {
            monPtr->periodicBeacon = false;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }

        if (monPtr->pairHoldGuardUntilMs > 0) {
            if (isPressedNow || edges > 0) {
                monPtr->waitReleaseAfterPair = true;
                monPtr->pairHoldGuardUntilMs = 0;
                LOGF("[GATT BTN] %s paired while held -> release button before arming.\n", slotMac.c_str());
            } else if (now >= monPtr->pairHoldGuardUntilMs) {
                monPtr->pairHoldGuardUntilMs = 0;
            }
        }

        if (monPtr->waitReleaseAfterPair) {
            if (relNow || (!isPressedNow && edges == 0)) {
                monPtr->waitReleaseAfterPair = false;
                monPtr->pairHoldGuardUntilMs = now + 1200UL;
                LOGF("[GATT BTN] %s released -> armed & ready.\n", slotMac.c_str());
            }
            continue;
        }

        uint8_t liveSpeed = monPtr->speed;
        if (s.wakePreTriggered && edges > 0) {
            edges--;
            s.wakePreTriggered = false;
            monPtr->wakePreTriggered = false;
            LOGF("[GATT BTN] %s wake pre-trigger handed off to live GATT link.\n", slotMac.c_str());
        }
        while (edges > 0) {
            edges--;
            if (pairedMac.length() > 0) {
                onButtonPacket(slotMac, liveSpeed, now, true);
            }
        }
        if (relNow) {
            s.wakePreTriggered = false;
            monPtr->wakePreTriggered = false;
            onButtonRelease(slotMac, now);
        } else if (isSlotConnected && s.wakePreTriggered && !isPressedNow && edges == 0 &&
                   elapsedMs(now, slotConnAtMs) >= 120UL) {
            s.wakePreTriggered = false;
            monPtr->wakePreTriggered = false;
            if (monPtr->btnPhase == BTN_HELD) {
                unsigned long relTs = monPtr->isPrimary ? (monPtr->btnPressStartMs + 100UL) : now;
                onButtonRelease(slotMac, relTs);
            }
            LOGF("[GATT BTN] %s quick wake tap completed before GATT link settled.\n", slotMac.c_str());
        }
        if (isSlotConnected && isPressedNow && pairedMac.length() > 0) {
            onButtonPacket(slotMac, liveSpeed, now, true);
        }
    }
}

void setCoexScanner() {
    if (extConnTaskRunning.load() || gattBtnTaskRunning.load() || wifiScanInProgress || Update.isRunning()) return;
    unsigned long now = millis();
    bool webActive = (lastWebUiActivityMs > 0 && elapsedMs(now, lastWebUiActivityMs) < 3000UL);
    esp_coex_prefer_t desiredCoex = webActive ? ESP_COEX_PREFER_BALANCE : ESP_COEX_PREFER_BT;
    static esp_coex_prefer_t lastCoexPref = (esp_coex_prefer_t)-1;
    if (desiredCoex != lastCoexPref) {
        esp_coex_preference_set(desiredCoex);
        lastCoexPref = desiredCoex;
    }

    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    NimBLEScan* pScan = NimBLEDevice::getScan();
    static bool scannerConfigured = false;
    if (scannerConfigured && pScan->isScanning()) {
        return;
    }
    if (pScan->isScanning()) {
        pScan->stop();
    }
    pScan->setDuplicateFilter(false);
    pScan->setActiveScan(false);
    // Restored from Presep baseline: 160/80 (100ms cycle / 50ms listen)
    pScan->setInterval(160);
    pScan->setWindow(80);
    pScan->setMaxResults(0);
    scannerConfigured = true;
    pScan->start(0, false);
}

class ScanCallbacks : public NimBLEScanCallbacks {
    static bool strStartsWithCI(const char* str, const char* prefix) {
        if (!str || !prefix) return false;
        while (*prefix) {
            char a = *str++;
            char b = *prefix++;
            if (a >= 'a' && a <= 'z') a -= 32;
            if (b >= 'a' && b <= 'z') b -= 32;
            if (a != b) return false;
        }
        return true;
    }

    static bool strContainsCI(const char* str, const char* needle) {
        if (!str || !needle || !*needle) return false;
        for (const char* p = str; *p; p++) {
            if (strStartsWithCI(p, needle)) return true;
        }
        return false;
    }

    void onResult(const NimBLEAdvertisedDevice* dev) override {
        unsigned long now = millis();
        const uint8_t* macVal = dev->getAddress().getVal();
        uint8_t aType = dev->getAddressType();
        int rssi = dev->getRSSI();
        uint8_t advType = dev->getAdvType();
        bool isScanRsp = (advType == 4);

        ScanEvent ev;
        memset(&ev, 0, sizeof(ev));
        ev.watts = -1;
        snprintf(ev.mac, sizeof(ev.mac), "%02x:%02x:%02x:%02x:%02x:%02x",
                 macVal[5], macVal[4], macVal[3], macVal[2], macVal[1], macVal[0]);
        ev.addrType = aType;
        ev.rssi = (int8_t)(rssi < -127 ? -127 : (rssi > 0 ? 0 : rssi));
        ev.advType = advType;
        ev.ts = now;

        bool isPairedHit = false;
        portENTER_CRITICAL(&pairedMux);
        if (pairedMacBuf[0] != '\0' && strcasecmp(ev.mac, pairedMacBuf) == 0) isPairedHit = true;
        portEXIT_CRITICAL(&pairedMux);

        if (isPairedHit) {
            pairedLastSeenMs = now; pairedRssi = rssi; pairedAddrType = aType;
            if (connState == STATE_DISCONNECTED && !requestConnect && !extConnTaskRunning.load()) pairedAdvSeenFlag.store(true);
        }

        bool isFestoolSvc = dev->isAdvertisingService(DUST_SVC_UUID);
        bool isFestoolMfg = false, isShaperMfg = false, isShellyPlug = false, isBurstBtnAdv = false, isGattBtnAdv = false;
        bool hasBtnSvc = false, hasFe3aSvc = false, hasFe39SvcData = false, hasFlags05 = false;
        bool isConnectable = (advType == 0 || advType == 1);
        bool hasFlags06 = false;
        if (advType == 1) {
            isGattBtnAdv = true;
        }
        if (dev->isAdvertisingService(NimBLEUUID((uint16_t)0x1812))) {
            isGattBtnAdv = true;
            isConnectable = true;
            hasBtnSvc = true;
        }
        if (dev->isAdvertisingService(NimBLEUUID((uint16_t)0xFFE0))) {
            isConnectable = true;
            hasBtnSvc = true;
        }
        if (dev->haveAppearance()) {
            uint16_t app = dev->getAppearance();
            if (app == 0x0180) isBurstBtnAdv = true;
            else if (app == 0x03C1 || app == 0x03C0 || app == 0x03C2) isGattBtnAdv = true;
        }

        int parsedSoc = -1, parsedState = -1;
        int16_t parsedBtnSeq = -1;
        uint32_t parsedMatNum = 0;
        int32_t parsedWatts = -1;

        const std::vector<uint8_t>& payVec = dev->getPayload();
        const uint8_t* payPtr = payVec.data();
        size_t payLen = payVec.size();
        if (payPtr && payLen > 0) {
            if (!isScanRsp) {
                size_t pos = 0;
                for (size_t i = 0; i < payLen && pos + 4 < sizeof(ev.raw); i++) {
                    pos += snprintf(ev.raw + pos, sizeof(ev.raw) - pos, "%02X ", payPtr[i]);
                }
                if (pos > 0 && ev.raw[pos - 1] == ' ') ev.raw[pos - 1] = '\0';
            }
            size_t idx = 0;
            size_t namePos = 0;
            while (idx + 1 < payLen) {
                uint8_t fieldLen = payPtr[idx];
                if (fieldLen == 0 || idx + 1 + fieldLen > payLen) break;
                uint8_t adType = payPtr[idx + 1];
                const uint8_t* fData = payPtr + idx + 2;
                size_t fDataLen = (size_t)(fieldLen - 1);

                if (adType == 0x01 && fDataLen >= 1) {
                    uint8_t flags = fData[0];
                    if (flags == 0x05) { isBurstBtnAdv = true; hasFlags05 = true; }
                    else if (flags == 0x06) hasFlags06 = true;
                } else if ((adType == 0x02 || adType == 0x03) && fDataLen >= 2) {
                    for (size_t k = 0; k + 1 < fDataLen; k += 2) {
                        uint16_t u16 = (uint16_t)fData[k] | ((uint16_t)fData[k + 1] << 8);
                        if (u16 == 0x1812) { isGattBtnAdv = true; isConnectable = true; hasBtnSvc = true; }
                        else if (u16 == 0xFFE0) { isConnectable = true; hasBtnSvc = true; }
                        else if (u16 == 0xFE39) isFestoolSvc = true;
                        else if (u16 == 0xFE3A) { isFestoolSvc = true; hasFe3aSvc = true; isBurstBtnAdv = true; isConnectable = true; }
                    }
                } else if (adType == 0x19 && fDataLen >= 2) {
                    uint16_t app = (uint16_t)fData[0] | ((uint16_t)fData[1] << 8);
                    if (app == 0x0180) isBurstBtnAdv = true;
                    else if (app == 0x03C1 || app == 0x03C0 || app == 0x03C2) isGattBtnAdv = true;
                } else if ((adType == 0x09 || adType == 0x08) && fDataLen > 0) {
                    if (adType == 0x09 || namePos == 0) {
                        namePos = 0;
                        for (size_t k = 0; k < fDataLen && namePos + 1 < sizeof(ev.name); k++) {
                            char ch = (char)fData[k];
                            if (ch >= 32 && ch < 127) ev.name[namePos++] = ch;
                        }
                        ev.name[namePos] = '\0';
                    }
                } else if (adType == 0x16 && fDataLen >= 2) {
                    uint16_t svcUuid16 = (uint16_t)fData[0] | ((uint16_t)fData[1] << 8);
                    if (svcUuid16 == 0xFE39) {
                        isFestoolSvc = true;
                        hasFe39SvcData = true;
                        const uint8_t* sData = fData + 2;
                        size_t sLen = fDataLen - 2;
                        if (sLen >= 5) {
                            uint32_t tnr = (uint32_t)sData[2] | ((uint32_t)sData[3] << 8) | ((uint32_t)sData[4] << 16);
                            if (tnr >= 100000 && tnr <= 16777215) parsedMatNum = tnr;
                        }
                        if (sLen >= 8) {
                            uint8_t fType = sData[0];
                            bool isRemoteTnr = (parsedMatNum == 202097 || parsedMatNum == 204842 || parsedMatNum == 10029918 || parsedMatNum == 10030046);
                            if (isRemoteTnr) {
                                parsedState = (sData[7] != 0) ? 1 : 0;
                            } else if (fType == 0x01 || fType == 0x02 || fType == 0x11 || fType == 0x12) {
                                uint8_t rawSoc = sData[6];
                                if (rawSoc <= 100) parsedSoc = (int)rawSoc;
                                parsedState = ((sData[7] & 0x01) != 0) ? 1 : 0;
                            }
                        }
                    } else if (svcUuid16 == 0xFE3A) {
                        isFestoolSvc = true;
                        hasFe3aSvc = true;
                        isBurstBtnAdv = true;
                        isConnectable = true;
                        parsedState = 1;
                    } else if (svcUuid16 == 0xFFE0) {
                        hasBtnSvc = true;
                        isConnectable = true;
                    } else if (svcUuid16 == 0xFCD2) {
                        for (size_t k = 3; k + 1 < fDataLen; k++) {
                            if (fData[k] == 0x3A) {
                                isBurstBtnAdv = true;
                                parsedBtnSeq = (uint8_t)fData[k + 1];
                                break;
                            }
                        }
                    }
                } else if (adType == 0xFF && fDataLen >= 2) {
                    if ((fData[0] == 0x4F || fData[0] == 0x1F) && fData[1] == 0x04) {
                        isFestoolMfg = true;
                        if (fData[0] == 0x4F && fDataLen >= 6 && parsedMatNum == 0) {
                            uint32_t mfgTnr = (uint32_t)fData[3] | ((uint32_t)fData[4] << 8) | ((uint32_t)fData[5] << 16);
                            if (mfgTnr >= 100000 && mfgTnr <= 16777215) parsedMatNum = mfgTnr;
                        }
                    } else if (fDataLen >= 5 && fData[0] == 0xA9 && fData[1] == 0x0B && fData[2] == 'S' && fData[3] == 'C') {
                        isShellyPlug = true;
                        uint8_t flagsByte = fData[4];
                        bool relayOn = ((flagsByte & 0x01) != 0);
                        if (fDataLen >= 7) {
                            uint16_t wVal = (uint16_t)fData[5] | ((uint16_t)fData[6] << 8);
                            parsedWatts = relayOn ? (int32_t)wVal : 0;
                            parsedState = (relayOn && wVal >= 70) ? 1 : 0;
                        } else {
                            parsedState = relayOn ? 1 : 0;
                        }
                    } else if (fData[0] == 0x5E && fData[1] == 0x0E) {
                        isShaperMfg = true;
                    } else if (!isFestoolMfg && !isShaperMfg && !isShellyPlug && fDataLen >= 3) {
                        uint16_t mfgId = (uint16_t)fData[0] | ((uint16_t)fData[1] << 8);
                        if (mfgId == 0x000D && fDataLen >= 4 && fDataLen <= 6) {
                            isBurstBtnAdv = true;
                        }
                        parsedBtnSeq = (uint8_t)fData[fDataLen - 1];
                    }
                }
                idx += 1 + fieldLen;
            }
        }

        const char* rawNameC = ev.name;
        if (strContainsCI(rawNameC, "SHUTTER") || strContainsCI(rawNameC, "ITAG") || strContainsCI(rawNameC, "PTT-Z01")) isGattBtnAdv = true;

        bool nameIsRemote = (parsedMatNum == 202097 || parsedMatNum == 204842 || parsedMatNum == 10029918 || parsedMatNum == 10030046 ||
                             strContainsCI(rawNameC, "CT-F") || strContainsCI(rawNameC, "CTF"));
        bool nameIsExtractor = !nameIsRemote && (
                               parsedMatNum == 574830 || parsedMatNum == 574832 || parsedMatNum == 574833 ||
                               parsedMatNum == 576936 || parsedMatNum == 576944 || parsedMatNum == 577065 ||
                               parsedMatNum == 575267 || parsedMatNum == 575279 || parsedMatNum == 574947 ||
                               parsedMatNum == 574955 || parsedMatNum == 574963 ||
                               strStartsWithCI(rawNameC, "CT") || strContainsCI(rawNameC, "CTL") ||
                               strContainsCI(rawNameC, "CTM") || strContainsCI(rawNameC, "CTH") ||
                               strContainsCI(rawNameC, "CLEANTEC") || strContainsCI(rawNameC, "MIDI") ||
                               strContainsCI(rawNameC, "MINI"));
        bool isBenchPilot = isShaperMfg || strStartsWithCI(rawNameC, "SCC");
        bool nameIsPlug = !isBenchPilot && (isShellyPlug || strStartsWithCI(rawNameC, "SHELLY-DUST"));
        bool nameIsSwitch = !isBenchPilot && (strStartsWithCI(rawNameC, "PTT") || strContainsCI(rawNameC, "SHUTTER") ||
                            strContainsCI(rawNameC, "ITAG") || strContainsCI(rawNameC, "CLICKER") ||
                            strContainsCI(rawNameC, "KEYFOB"));

        bool isFestoolRemote = !isBenchPilot && (hasFe3aSvc || nameIsRemote || (isFestoolSvc && hasFlags05 && !hasFe39SvcData && !nameIsExtractor));
        if (isFestoolRemote) {
            isBurstBtnAdv = true;
            isConnectable = false;
            if (!hasFe3aSvc && parsedState == -1) parsedState = -2;
        }

        if (isBenchPilot && !isPairedHit) ev.devCat = CAT_OTHER;
        else if (isPairedHit || ((isFestoolSvc || isFestoolMfg) && !isFestoolRemote && (nameIsExtractor || (isConnectable && parsedSoc < 0)))) ev.devCat = CAT_EXTRACTOR;
        else if (nameIsPlug) ev.devCat = CAT_TOOL_PLUG;
        else if (isFestoolRemote || isGattBtnAdv || isBurstBtnAdv || hasBtnSvc || nameIsSwitch) {
            ev.devCat = CAT_BUTTON;
            // Switches are only GATT-capable if they advertise 0xFFE0/0x1812, HID appearance, or directed wake (advType==1)
            isConnectable = (isGattBtnAdv || hasBtnSvc || advType == 1);
        }
        else if ((isFestoolSvc || isFestoolMfg) && parsedSoc >= 0) ev.devCat = CAT_BATTERY;
        else if (isFestoolSvc || isFestoolMfg) ev.devCat = CAT_FESTOOL;
        else ev.devCat = CAT_OTHER;

        ev.isKnown = (ev.devCat != CAT_OTHER);
        ev.soc = (int8_t)parsedSoc;
        ev.state = (int8_t)parsedState;
        ev.matNum = parsedMatNum;
        ev.watts = parsedWatts;
        ev.isGattBtnAdv = isGattBtnAdv;
        ev.isBurstBtnAdv = isBurstBtnAdv;
        ev.isShellyPlug = isShellyPlug;
        ev.isFestoolRemote = isFestoolRemote;
        ev.isConnectable = isConnectable;
        ev.hasFlags06 = hasFlags06;
        ev.btnSeq = parsedBtnSeq;

        bool isMonHit = false;
        portENTER_CRITICAL(&monMacMux);
        for (uint8_t mi = 0; mi < monMacCacheCount; mi++) {
            if (strcasecmp(ev.mac, monMacCache[mi]) == 0) { isMonHit = true; break; }
        }
        portEXIT_CRITICAL(&monMacMux);
        if (!ev.isKnown && !isPairedHit && !isMonHit && !(isScanRsp && nameIsExtractor)) {
            return;
        }

        pushScanEvent(ev);
    }
};

static void processSingleScanEvent(const ScanEvent& ev, unsigned long now) {
    String addr = ev.mac;
    uint8_t aType = ev.addrType;
    String name = ev.name;
    int rssi = ev.rssi;
    uint8_t advType = ev.advType;
    bool isScanRsp = (advType == 4);
    int parsedSoc = ev.soc;
    int parsedState = ev.state;
    uint32_t parsedMatNum = ev.matNum;
    int32_t parsedWatts = ev.watts;
    bool isKnown = ev.isKnown;
    bool isGattBtnAdv = ev.isGattBtnAdv;
    bool isBurstBtnAdv = ev.isBurstBtnAdv;
    bool isShellyPlug = ev.isShellyPlug;
    String parsedRaw = ev.raw;
    String festoolMatch = (parsedMatNum > 0) ? lookupFestoolModelByTnr(parsedMatNum) : String("");
    String devType = devCatToTypeStr(ev.devCat);

    BleEntry* blePtr = nullptr;
    for (auto& b : bleList) {
        if (b.address.equalsIgnoreCase(addr)) { blePtr = &b; break; }
    }
    if (blePtr) {
        if (ev.isConnectable) blePtr->canGatt = true;
        if (isBurstBtnAdv) {
            blePtr->canAdv = true;
            if (ev.devCat == CAT_BUTTON && !ev.isConnectable) blePtr->canGatt = false;
        }
        if (ev.devCat == CAT_EXTRACTOR && !isScanRsp) {
            if (ev.isBurstBtnAdv) blePtr->pairingMode = true;
            else if (ev.hasFlags06) blePtr->pairingMode = false;
        }
        if (!isScanRsp) {
            if (blePtr->lastPrimaryAdvMs > 0) {
                unsigned long dt = elapsedMs(ev.ts, blePtr->lastPrimaryAdvMs);
                if (dt > 0 && dt < 250UL) {
                    if (blePtr->fastWakeHits < 10) blePtr->fastWakeHits++;
                    if (dt < 130UL) {
                        if (blePtr->fastCadenceHits < 10) blePtr->fastCadenceHits++;
                        blePtr->canAdv = true;
                    }
                } else if (dt >= 350UL) {
                    blePtr->fastWakeHits = 0;
                    if (!blePtr->canAdv && (isGattBtnAdv || ev.hasFlags06 || ev.devCat == CAT_BUTTON)) {
                        if (blePtr->slowCadenceHits < 10) blePtr->slowCadenceHits++;
                    }
                }
            }
            blePtr->lastPrimaryAdvMs = ev.ts;
        }
    }

    bool knownCanAdv = isBurstBtnAdv || (blePtr && blePtr->canAdv);
    bool knownGattOnly = !knownCanAdv && (isGattBtnAdv || (blePtr && blePtr->slowCadenceHits >= 1));

    if (knownCanAdv) {
        parsedRaw = "ADV:STREAM";
    } else if (knownGattOnly && (parsedRaw.length() == 0 || !parsedRaw.startsWith("GATT"))) {
        parsedRaw = "GATT:FFE0/HID";
    }

    String friendlyName = "Unknown";
    if (festoolMatch.length() > 0) friendlyName = festoolMatch;
    else if (pairedMac.length() > 0 && addr.equalsIgnoreCase(pairedMac)) friendlyName = extFriendlyName;
    else if (name.length() > 0) friendlyName = name;
    else if (isShellyPlug) friendlyName = "Shelly Tool Plug";
    else if (ev.isFestoolRemote) friendlyName = "CT-F I Remote";
    else if (ev.devCat == CAT_BATTERY) friendlyName = "Festool Battery";
    else if (ev.devCat == CAT_EXTRACTOR) friendlyName = "Festool Extractor";
    else if (ev.devCat == CAT_FESTOOL) friendlyName = "Festool";
    else if (ev.devCat == CAT_BUTTON && isGattBtnAdv) friendlyName = "HID Switch";
    MonitoredDevice* mon = findMonitoredByMac(addr);
    int monSpeed = -1;
    uint8_t monBtnType = 0;
    bool monIsGatt = false;
    bool monConfiguring = false;
    if (mon) {
        monSpeed = mon->speed;
        monBtnType = mon->btnType;
        if ((aType == 0 || aType == 1) && mon->addrType != aType) {
            mon->addrType = aType;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (name.length() > 0 && (mon->advName.length() == 0 || name.length() > mon->advName.length())) {
            mon->advName = name;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (parsedMatNum != 0 && mon->matNum != parsedMatNum) {
            mon->matNum = parsedMatNum;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (ev.devCat != CAT_OTHER && mon->devCat != ev.devCat) {
            mon->devCat = ev.devCat;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (mon->btnType > 0 && ev.isConnectable && !mon->canGatt) {
            mon->canGatt = true;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (mon->btnType > 0 && knownCanAdv && !mon->canAdv) {
            mon->canAdv = true;
            mon->periodicBeacon = false;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (mon->btnType > 0 && mon->canAdv && mon->periodicBeacon) {
            mon->periodicBeacon = false;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (mon->btnType > 0 && !mon->canAdv && blePtr && blePtr->slowCadenceHits >= 2 && !mon->periodicBeacon) {
            mon->periodicBeacon = true;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
        if (mon->btnType > 0 && !mon->isGatt && !mon->canAdv && knownGattOnly) {
            mon->isGatt = true;
            mon->canGatt = true;
            mon->isPrimary = true;
            mon->activeRaw = "GATT:FFE0/HID";
            if (!isGattMacConnectedOrBusy(addr)) {
                mon->gattReady = false;
            }
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
            LOGF("[AUTO GATT] Exception triggered for %s -> promoted to GATT switch\n", addr.c_str());
        }
        monIsGatt = mon->isGatt;
        monConfiguring = isMonitoredConfiguring(*mon, now);
        if (!mon->customNamed && name.length() > 0 &&
            (mon->name.length() == 0 || name.length() > mon->name.length() || mon->name == "Device" || mon->name == "Button")) {
            mon->name = name;
            if (!needSaveMonitored) needSaveMonitoredAtMs = now;
            needSaveMonitored = true;
        }
    }

    if (mon) {
        if (monBtnType >= 1 && monBtnType <= 5) {
            devType = "Button";
            if (monIsGatt) {
                int gIdx = findGattSlotByMac(addr);
                bool busyOrConn = isGattMacConnectedOrBusy(addr);
                bool alreadyActiveBtn = (mon->btnPhase != BTN_IDLE);
                parsedState = ((gIdx >= 0 && gattSlots[gIdx].pressed) || alreadyActiveBtn) ? 1 : 0;
                if (!busyOrConn && !isScanRsp) {
                    // Multi-Primary rotation: pre-trigger & rotate into the 2 GATT slots on directed wake (advType == 1)
                    // OR burst wake from dual-mode Primary switches (mon->canAdv && !mon->periodicBeacon)
                    bool isDirectedWake = (advType == 1);
                    bool allowWakePreTrig = isDirectedWake || (mon->canAdv && !mon->periodicBeacon);
                    bool isBootedAndConnected = (connState == STATE_CONNECTED && connectedAtMs > 0 && elapsedMs(ev.ts, connectedAtMs) >= 500UL);
                    bool canPreTrig = (isBootedAndConnected &&
                                       allowWakePreTrig &&
                                       mon->gattReady &&
                                       !monConfiguring &&
                                       !mon->waitReleaseAfterPair &&
                                       mon->pairHoldGuardUntilMs == 0 &&
                                       !alreadyActiveBtn &&
                                       !isGattBackedOff(addr, ev.ts) &&
                                       pairedMac.length() > 0);
                    if (canPreTrig) {
                        mon->wakePreTriggered = true;
                        parsedState = 1;
                        LOGF("[GATT BTN] Wake packet pre-trigger -> %s at %u%%\n", addr.c_str(), (unsigned)monSpeed);
                        onButtonPacket(addr, (uint8_t)monSpeed, ev.ts, true);
                    }
                    if (allowWakePreTrig || !mon->gattReady) {
                        enqueueGattConnect(addr, aType, monBtnType, (uint8_t)monSpeed, canPreTrig, canPreTrig);
                    }
                }
            } else if (!monConfiguring && !isScanRsp && ev.ts >= 2500UL) {
                if (parsedState == -2) {
                    // CT-F I remotes are strictly ON/OFF via MAN button; ignore Bluetooth pairing button packets
                } else if (parsedState == 0) {
                    if (mon->btnPhase == BTN_HELD) onButtonRelease(addr, ev.ts);
                } else {
                    parsedState = 1;
                    onButtonPacket(addr, (uint8_t)monSpeed, ev.ts, false, ev.btnSeq);
                }
            }
        } else if (monBtnType == 0 && (parsedWatts >= 0 || parsedState != -1)) {
            int effState = parsedState;
            if (parsedWatts >= 0) {
                if (mon->lastWatts < 0 || abs(parsedWatts - mon->lastWatts) >= 5) {
                    mqttDevicesDirty = true;
                }
                mon->lastWatts = parsedWatts;
                uint16_t floorW = (mon->wattsThr >= 5) ? mon->wattsThr : 70;
                constexpr uint8_t OFF_CONFIRM_PKTS = 3;
                if ((uint32_t)parsedWatts >= floorW) {
                    mon->offConfirmHits = 0;
                    effState = 1;
                } else if (mon->toolActive) {
                    if (mon->offConfirmHits < 20) mon->offConfirmHits++;
                    if (mon->offConfirmHits >= OFF_CONFIRM_PKTS) {
                        effState = 0;
                        mon->offConfirmHits = 0;
                    } else {
                        effState = 1;
                    }
                } else {
                    mon->offConfirmHits = 0;
                    effState = 0;
                }
                parsedState = effState;
            }
            if (effState == 1) {
                bool wasOff = !mon->toolActive;
                mon->toolActive = true;
                mon->toolLastSeen = ev.ts;
                if (wasOff) {
                    mon->liveSpeedOverride = 0;
                    mqttDevicesDirty = true;
                    if (parsedWatts >= 0) LOGF("[TOOL PLUG] Load ON (%ldW >= %uW) -> %u%% (%s)\n", (long)parsedWatts, (unsigned)mon->wattsThr, (unsigned)monSpeed, addr.c_str());
                    else LOGF("[AUTO TOOL] Load ON -> %u%% (%s)\n", (unsigned)monSpeed, addr.c_str());
                    hoseClearActive = false;
                }
                syncToolBattDemandFromMonitored(ev.ts);
            } else if (effState == 0 && mon->toolActive) {
                mon->toolActive = false;
                mon->liveSpeedOverride = 0;
                mqttDevicesDirty = true;
                if (parsedWatts >= 0) LOGF("[TOOL PLUG] Load OFF (%ldW < %uW) -> %s\n", (long)parsedWatts, (unsigned)mon->wattsThr, addr.c_str());
                else LOGF("[AUTO TOOL] Load OFF -> %s\n", addr.c_str());
                syncToolBattDemandFromMonitored(ev.ts);
                if (!toolBattDemandActive && !isAnyPrimaryDemandActive()) {
                    if (evacSeconds > 0) {
                        LOGF("[AUTO TOOL] Entering %us debris evacuation at 20%%\n", evacSeconds);
                        hoseClearActive = true;
                        hoseClearTimer = ev.ts;
                    } else {
                        forceStopNow = true;
                    }
                    demandDirty = true;
                }
            }
        }
    }

    if (blePtr) {
        if (name.length() > 0) blePtr->advName = name;
        if (friendlyName != "Unknown" && (blePtr->name == "Unknown" || blePtr->name == "Festool" || blePtr->name == "HID Switch" || festoolMatch.length() > 0 || name.length() > 0)) {
            if (!(blePtr->name.startsWith("CT-F I") && friendlyName == "CT-F I Remote")) {
                blePtr->name = friendlyName;
            }
        }
        if (aType == 0 || aType == 1) blePtr->addrType = aType;
        blePtr->rssi = rssi;
        if (isKnown) blePtr->isKnownDevice = true;
        if (devType != "Other") blePtr->type = devType;
        blePtr->lastSeen = ev.ts;
        if (parsedSoc >= 0) {
            if (blePtr->soc != parsedSoc) mqttDevicesDirty = true;
            blePtr->soc = parsedSoc;
        }
        if (parsedState != -1) {
            if (blePtr->state != parsedState) mqttDevicesDirty = true;
            blePtr->state = parsedState;
        }
        if (parsedMatNum > 0) blePtr->matNum = parsedMatNum;
        if (parsedWatts >= 0) blePtr->watts = parsedWatts;
        if (blePtr->canAdv) {
            blePtr->rawData = "ADV:STREAM";
        } else if (parsedRaw.length() > 0) {
            blePtr->rawData = parsedRaw;
        }
    } else if (isKnown || monSpeed != -1 || (pairedMac.length() > 0 && addr.equalsIgnoreCase(pairedMac))) {
        if (bleList.size() >= MAX_BLE_LIST_ENTRIES) {
            pruneStaleBleEntries(now);
            if (bleList.size() >= MAX_BLE_LIST_ENTRIES) {
                for (auto it = bleList.begin(); it != bleList.end(); ++it) {
                    bool isPinned = (pairedMac.length() > 0 && it->address.equalsIgnoreCase(pairedMac));
                    if (!isPinned && findMonitoredByMac(it->address) == nullptr) {
                        bleList.erase(it);
                        break;
                    }
                }
            }
        }
        bool initPairingMode = (ev.devCat == CAT_EXTRACTOR && !isScanRsp && ev.isBurstBtnAdv);
        BleEntry newEntry = {friendlyName, name, addr, aType, rssi, isKnown, devType, ev.ts, parsedSoc, parsedState, parsedMatNum, parsedWatts, parsedRaw, isBurstBtnAdv, ev.isConnectable, initPairingMode, isScanRsp ? 0UL : ev.ts, 0, 0, 0};
        bleList.push_back(newEntry);
        mqttDevicesDirty = true;
    }
}

void drainScanEvents(unsigned long now) {
    ScanEvent ev; int processed = 0;
    while (processed < 64 && popScanEvent(ev)) { processSingleScanEvent(ev, now); processed++; }
}

struct ExtConnTaskParams {
    NimBLEClient* client;
    char mac[18];
    uint8_t addrType;
};
static String extConnTaskMac = "";
static uint8_t extConnTaskAddrType = 0;
static uint8_t extConnTaskForcedType = 255;

static void extConnectWorkerTask(void* param) {
    ExtConnTaskParams* p = static_cast<ExtConnTaskParams*>(param);
    bool ok = false;
    if (p && p->client) {
        NimBLEAddress addr(p->mac, p->addrType);
        if (p->client->connect(addr)) {
            ok = p->client->secureConnection();
            if (!ok && p->client->isConnected()) {
                p->client->disconnect();
            }
        }
    }
    delete p;
    extConnTaskSuccess.store(ok);
    extConnTaskRunning.store(false);
    extConnTaskDone.store(true);
    vTaskDelete(nullptr);
}

void startBleConnect(const String& macStr, uint8_t forcedType) {
    if (extConnTaskRunning.load() || gattBtnTaskRunning.load() || macStr.length() == 0) return;
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
    connState = STATE_CONNECTING;
    NimBLEDevice::getScan()->stop();
    esp_coex_preference_set(ESP_COEX_PREFER_BT);
    LOGF("[BLE] >>> Connecting to Extractor (GATT App): %s ...\n", macStr.c_str());

    uint8_t addrType = pairedAddrType;
    if (forcedType != 255) addrType = forcedType;
    else for (const auto& item : bleList) if (item.address.equalsIgnoreCase(macStr)) { addrType = item.addrType; break; }
    addrType = inferRandomAddrType(macStr, addrType);

    if (!pClient) { pClient = NimBLEDevice::createClient(); pClient->setClientCallbacks(new ClientCallbacks(), false); }
    pClient->setConnectTimeout(2200);

    ExtConnTaskParams* taskParams = new ExtConnTaskParams();
    taskParams->client = pClient;
    strncpy(taskParams->mac, macStr.c_str(), sizeof(taskParams->mac) - 1);
    taskParams->mac[sizeof(taskParams->mac) - 1] = '\0';
    taskParams->addrType = addrType;

    extConnTaskMac = macStr;
    extConnTaskAddrType = addrType;
    extConnTaskForcedType = forcedType;
    extConnTaskSuccess.store(false);
    extConnTaskDone.store(false);
    extConnTaskRunning.store(true);
    if (xTaskCreatePinnedToCore(extConnectWorkerTask, "extBleConn", 6144, taskParams, 1, nullptr, 1) != pdPASS) {
        delete taskParams;
        extConnTaskRunning.store(false);
        connState = STATE_DISCONNECTED;
        setCoexScanner();
    }
}

void finishBleConnect(unsigned long now) {
    bool ok = extConnTaskSuccess.load();
    String macStr = extConnTaskMac;
    uint8_t addrType = extConnTaskAddrType;
    uint8_t forcedType = extConnTaskForcedType;

    if (extUnpairPending.exchange(false) || (forcedType != 255 && pairedMac.length() == 0)) {
        if (pClient && pClient->isConnected()) pClient->disconnect();
        connState = STATE_DISCONNECTED;
        setCoexScanner();
        return;
    }

    if (!ok || !pClient || !pClient->isConnected()) {
        LOGLN("[BLE] Connect or security handshake failed. Resuming scanner.");
        if (pClient && pClient->isConnected()) pClient->disconnect();
        lastExtConnectFailMs = now;
        connState = STATE_DISCONNECTED; setCoexScanner();
        for (auto& m : monitoredList) {
            if (m.btnType > 0 && m.btnPhase == BTN_HELD) {
                resetMonitoredButtonGesture(m, false);
            }
        }
        syncButtonDemandsFromMonitored();
        return;
    }

    LOGLN("[BLE] Security complete -> discovering GATT...");
    pWriteChar = nullptr; pNotifyChar = nullptr;

    NimBLERemoteService* pSvc = pClient->getService(DUST_SVC_UUID);
    if (pSvc) {
        pNotifyChar = pSvc->getCharacteristic(CHAR_NOTIFY_UUID);
        pWriteChar  = pSvc->getCharacteristic(CHAR_WRITE_UUID);
        if (pNotifyChar && pNotifyChar->canNotify()) { pNotifyChar->subscribe(true, onFe02Notify); LOGLN("[BLE] Subscribed to 3734fe02 notifications"); }
        if (pWriteChar) LOGLN("[BLE] Bound 3734fe03 FMP V2 command characteristic");
    }

    if (!pWriteChar || !pNotifyChar || !pClient->isConnected()) {
        LOGLN("[BLE] Missing FMP V2 GATT characteristics or link dropped -> disconnecting.");
        if (pClient && pClient->isConnected()) pClient->disconnect();
        lastExtConnectFailMs = now;
        connState = STATE_DISCONNECTED; setCoexScanner();
        return;
    }

    lastExtConnectFailMs = 0;
    connState = STATE_CONNECTED;
    LOGLN("[BLE] GATT connection complete & FMP V2 ready!");

    setPairedMacState(macStr, addrType);
    prefs.putString("paired_mac", pairedMac);
    prefs.putUChar("paired_type", pairedAddrType);
    publishExtractorTelemetry(now);

    bool removedStaleMon = false;
    for (auto it = monitoredList.begin(); it != monitoredList.end(); ) {
        if (it->address.equalsIgnoreCase(macStr)) {
            disconnectGattButtonMac(macStr);
            it = monitoredList.erase(it);
            removedStaleMon = true;
        } else {
            ++it;
        }
    }
    if (removedStaleMon) {
        saveMonitoredToNvs();
        LOGF("[BLE] Removed paired extractor %s from Monitored Devices.\n", macStr.c_str());
    }

    setCoexScanner();
    for (auto& m : monitoredList) {
        if (m.btnType > 0 && m.btnPhase == BTN_HELD) {
            resetMonitoredButtonGesture(m, false);
        }
    }
    syncButtonDemandsFromMonitored();
    connectedAtMs = now; lastBleActivityMs = connectedAtMs;
    postConnectActionPending = true;
    postConnectActionAtMs = now + 60UL;
    initialTelemDone = false; telemPollRequested = false; telemAwaitingReply = false;
}

void initBleSubsystem() {
    bleList.reserve(MAX_BLE_LIST_ENTRIES);
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(new ScanCallbacks(), true);
    scannerStartedMs = millis();
    setCoexScanner();
    LOGLN("[BLE] Continuous scanner started");
}
