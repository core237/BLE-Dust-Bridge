#include <Arduino.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NimBLEDevice.h>
#include <esp_coexist.h>

#include "config.h"
#include "app_state.h"
#include "storage.h"
#include "fmp_protocol.h"
#include "motor_control.h"
#include "ble_manager.h"
#include "mqtt_manager.h"
#include "serial_improv.h"
#include "web_server.h"

SET_LOOP_TASK_STACK_SIZE(16384);
extern "C" bool btInUse() { return true; }
extern "C" bool bleInUse() { return true; }

void setup() {
    logMutex = xSemaphoreCreateMutex();
    ledMutex = xSemaphoreCreateMutex();
    ledBootMode = true;
    setAtomLed(96, 0, 0);
    Serial.setRxBufferSize(1024);
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);
    while (!Serial && millis() < 600UL) {
        delay(10);
    }
    LOGLN("[BOOT] M5Stack AtomS3 Lite starting...");

    bootAuthSecret = generateRandomHex(16);

    uint64_t chipMac = ESP.getEfuseMac();
    char cidBuf[24];
    snprintf(cidBuf, sizeof(cidBuf), "BLDB_%06X", (uint32_t)(chipMac & 0xFFFFFFULL));
    mqttClientId = String(cidBuf);

    prefs.begin("bldb", false);
    String savedPaired = prefs.isKey("paired_mac") ? prefs.getString("paired_mac", "") : "";
    uint8_t rawSavedType = prefs.isKey("paired_type") ? prefs.getUChar("paired_type", 255) : 255;
    uint8_t savedType = inferRandomAddrType(savedPaired, rawSavedType);
    setPairedMacState(savedPaired, savedType);
    if (pairedMac.length() == 0) {
        LOGLN("[BOOT] No extractor paired -> continuing startup (LED flashes green when ready to configure).");
    }

    extFriendlyName = prefs.isKey("ext_name") ? prefs.getString("ext_name", "Festool Extractor") : "Festool Extractor";
    evacSeconds = prefs.getUChar("evac_sec", 4);
    loadMonitoredFromNvs();
    loadSettingsFromNvs();

    pinMode(BTN_ATOM_GPIO, INPUT_PULLUP);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    espClient.setConnectionTimeout(100);
#endif
    espClient.setTimeout(1);
    mqttClient.setBufferSize(2048);

    // Start SoftAP ('dustbridge_setup') immediately in AP_STA mode while connecting STA in parallel
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.setHostname(hostname);
    WiFi.setAutoReconnect(false);
    WiFi.softAP("dustbridge_setup", nullptr, 1);
    delay(100);
    LOGLN("[WIFI] Setup AP 'dustbridge_setup' ready immediately! IP: 192.168.4.1");

    if (!wifiList.empty() && wifiList[0].ssid.length() > 0) {
        LOGF("[WIFI] Connecting in background to: %s...\n", wifiList[0].ssid.c_str());
        WiFi.begin(wifiList[0].ssid.c_str(), wifiList[0].pass.c_str());
        bgWifiIdx = 1;
    }

    ledBootMode = false;
    updateStatusLed(millis());

    NimBLEDevice::init("BLE-Dust-Bridge");
    esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
    NimBLEDevice::setSecurityAuth(true, false, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);

    LOGF("[BLE] App MAC: %s | Paired Extractor: %s\n",
         NimBLEDevice::getAddress().toString().c_str(),
         pairedMac.length() > 0 ? pairedMac.c_str() : "NONE");

    setupWebServer();
    initBleSubsystem();
}

void loop() {
    unsigned long now = millis();
    static unsigned long lastLedTickMs = 0;
    if (elapsedMs(now, lastLedTickMs) >= 100UL) {
        lastLedTickMs = now;
        updateStatusLed(now);
    }
    server.handleClient();
    serviceSerialAndImprov(now);

    if (requestReboot && elapsedMs(now, rebootTimer) >= 600UL) {
        delay(100);
        ESP.restart();
    }
    if (extLinkConnectedLog.exchange(false)) {
        LOGLN("[BLE] Link connected -> securing bond & discovering GATT...");
    }
    if (extLinkDisconnectedFlag.exchange(false)) {
        mqttTelemDirty = true;
        LOGLN("[BLE] Extractor link disconnected.");
        setCoexScanner();
    }
    if (externalShutoffPending) {
        externalShutoffPending = false;
        if (isAnyPrimaryDemandActive() || hoseClearActive) {
            webUiDemandActive = false;
            webUiSpeedOverride = 0;
            clearAllButtonToggles();
            for (auto& m : monitoredList) {
                if (m.btnType == 0) {
                    m.toolActive = false;
                    m.liveSpeedOverride = 0;
                }
            }
            toolBattDemandActive = false;
            activeToolBattMac = "";
            hoseClearActive = false;
            mqttDevicesDirty = true;
            LOGLN("[EXTRACT] External/cutout shutoff event -> cleared all active demands.");
        }
    }
    if (mqttVacStateDirty || mqttVacSpeedDirty || mqttTelemDirty) {
        mqttVacStateDirty = false;
        mqttVacSpeedDirty = false;
        mqttTelemDirty = false;
        publishExtractorTelemetry(now);
    }
    if (mqttDevicesDirty) {
        mqttDevicesDirty = false;
        publishDevicesTelemetry(now);
    }
    if (extConnTaskDone.exchange(false)) {
        finishBleConnect(now);
    }
    if (postConnectActionPending && now >= postConnectActionAtMs) {
        postConnectActionPending = false;
        if (connState == STATE_CONNECTED) {
            if (isAnyPrimaryDemandActive() || hoseClearActive) evaluateMotorDemand();
            else sendFmpHandshake();
        }
    }
    if (connState == STATE_DISCONNECTED && !requestConnect && !extConnTaskRunning.load() && pairedMac.length() > 0 && pairedAdvSeenFlag.exchange(false)) {
        if (lastExtConnectFailMs == 0 || elapsedMs(now, lastExtConnectFailMs) >= 4000UL) {
            targetConnectMac = pairedMac;
            targetConnectType = pairedAddrType;
            requestConnect = true;
        }
    }

    if (requestConnect && !extConnTaskRunning.load() && !gattBtnTaskRunning.load()) {
        requestConnect = false;
        startBleConnect(targetConnectMac, targetConnectType);
    }
    drainScanEvents(now);
    if (needSaveMonitored && elapsedMs(now, needSaveMonitoredAtMs) >= 10000UL) {
        needSaveMonitored = false;
        needSaveMonitoredAtMs = 0;
        saveMonitoredToNvs();
    }

    static unsigned long lastBlePruneMs = 0;
    if (elapsedMs(now, lastBlePruneMs) >= 30000UL) {
        lastBlePruneMs = now;
        pruneStaleBleEntries(now);
    }

    serviceGattButton(now);
    serviceButtonGestures(now);

    static unsigned long lastToolWatchdogMs = 0;
    if (toolBattDemandActive && (elapsedMs(now, lastToolWatchdogMs) >= 250UL)) {
        lastToolWatchdogMs = now;
        for (auto& m : monitoredList) {
            if (m.btnType == 0 && m.toolActive && elapsedMs(now, m.toolLastSeen) >= 12000UL) {
                m.toolActive = false;
                m.liveSpeedOverride = 0;
            }
        }
        syncToolBattDemandFromMonitored(now);
        if (!toolBattDemandActive) {
            if (!isAnyPrimaryDemandActive()) {
                if (evacSeconds > 0) { LOGF("[TOOL WATCHDOG] Signal lost -> %us debris evacuation at 20%%\n", evacSeconds); hoseClearActive = true; hoseClearTimer = now; }
                else { LOGLN("[TOOL WATCHDOG] Signal lost -> Motor OFF"); forceStopNow = true; }
            }
            demandDirty = true;
        }
    }

    if (demandDirty) { demandDirty = false; evaluateMotorDemand(); }

    bool extRecentlySeen = (pairedLastSeenMs > 0 && elapsedMs(now, pairedLastSeenMs) < 30000UL);
    bool bootGrace = (scannerStartedMs > 0 && elapsedMs(now, scannerStartedMs) >= 1500UL && elapsedMs(now, scannerStartedMs) < 5000UL);
    if (pairedMac.length() > 0 && connState == STATE_DISCONNECTED && !requestConnect && !extConnTaskRunning.load() && (extRecentlySeen || bootGrace) &&
        (lastAutoReconnectTryMs == 0 || elapsedMs(now, lastAutoReconnectTryMs) >= 15000UL)) {
        lastAutoReconnectTryMs = now;
        LOGLN("[BLE] Maintaining GATT link -> connecting to paired extractor...");
        targetConnectMac = pairedMac; targetConnectType = pairedAddrType; requestConnect = true;
    }

    bool quietForDiag = !isAnyPrimaryDemandActive() && !hoseClearActive && !demandDirty && !isAnyButtonGestureBusy() &&
                        (elapsedMs(now, connectedAtMs) >= 6500UL) && (elapsedMs(now, lastControlTxMs) >= 3000UL) && (elapsedMs(now, lastBleActivityMs) >= 2500UL);
    if (connState == STATE_CONNECTED && !initialTelemDone && !telemPollRequested && quietForDiag) {
        initialTelemDone = true;
        updateStatusLed(now);
        LOGLN("[FMP TELEM] Link settled -> starting gentle background diagnostics...");
        startTelemetryPoll(true);
    }
    serviceTelemetryPoll(now);
    if (identifyActive && (elapsedMs(now, identifyStartMs) >= 10000UL)) sendFmpIdentify(false);

    if (mqtt_en && WiFi.status() == WL_CONNECTED) {
        if (!mqttClient.connected() && !isAnyButtonGestureBusy() && (elapsedMs(now, lastMqttReconnect) > 5000UL)) {
            lastMqttReconnect = now;
            bool ok = (mqtt_user.length() > 0)
                ? mqttClient.connect(mqttClientId.c_str(), mqtt_user.c_str(), mqtt_pass.c_str(), "BLE_DB/bridge/status", 0, true, "offline")
                : mqttClient.connect(mqttClientId.c_str(), nullptr, nullptr, "BLE_DB/bridge/status", 0, true, "offline");
            if (ok) {
                publishAllMqttTelemetry(now);
            }
        }
        if (mqttClient.connected()) {
            mqttClient.loop();
            if (elapsedMs(now, lastMqttHeartbeatMs) >= 10000UL) {
                publishAllMqttTelemetry(now);
            }
        }
    }

    static bool bgStaPromoted = false;
    static bool bgWifiExhausted = false;
    static unsigned long lastBgWifiTryMs = 0;
    static unsigned long staConnectedAtMs = 0;
    uint8_t apClients = WiFi.softAPgetStationNum();

    if (WiFi.status() == WL_CONNECTED) {
        bgWifiExhausted = false;
        if (staConnectedAtMs == 0) staConnectedAtMs = now;
        // Wait 6.5s so any phone handoff modal finishes before closing SoftAP
        if (!bgStaPromoted && !wifiScanInProgress && WiFi.getMode() == WIFI_AP_STA && (now - staConnectedAtMs > 6500UL)) {
            bgStaPromoted = true;
            installerWifiScreenActive = false;
            wifiConfirmedSolidUntilMs = now + 1500UL;
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_STA);
            LOGF("[WIFI] STA connected! IP: %s (closed setup AP)\n", WiFi.localIP().toString().c_str());
            if (MDNS.begin(hostname)) LOGLN("[MDNS] Started: http://dustbridge.local");
            if (mqtt_en && mqtt_ip.length() > 0) mqttClient.setServer(mqtt_ip.c_str(), mqtt_port);
        }
    } else {
        staConnectedAtMs = 0;
        if (WiFi.getMode() == WIFI_AP_STA || WiFi.getMode() == WIFI_AP) {
            bgStaPromoted = false;
        }
        size_t maxBgAttempts = wifiList.size() * 2;
        bool blePriorityActive = isAnyButtonGestureBusy() || (currentPowerState == 1) || isAnyPrimaryDemandActive() || hoseClearActive;
        if (!bgWifiExhausted && maxBgAttempts > 0 && bgWifiIdx < maxBgAttempts) {
            if (apClients == 0 && !blePriorityActive && !wifiScanInProgress && !improvProvPending &&
                !extConnTaskRunning.load() && !gattBtnTaskRunning.load() && connState != STATE_CONNECTING &&
                (now >= 8000UL) && (lastBgWifiTryMs == 0 || elapsedMs(now, lastBgWifiTryMs) >= 10000UL)) {
                lastBgWifiTryMs = now;
                const SavedWiFi& w = wifiList[bgWifiIdx % wifiList.size()];
                bgWifiIdx++;
                if (w.ssid.length() > 0) {
                    WiFi.disconnect(false);
                    LOGF("[WIFI] Discovery pass %u/2: %s...\n", (unsigned)((bgWifiIdx - 1) / wifiList.size() + 1), w.ssid.c_str());
                    WiFi.begin(w.ssid.c_str(), w.pass.c_str());
                }
            }
        } else if (!bgWifiExhausted && !wifiScanInProgress && (now >= 8000UL) &&
                   (lastBgWifiTryMs == 0 || elapsedMs(now, lastBgWifiTryMs) >= 10000UL)) {
            bgWifiExhausted = true;
            WiFi.disconnect(false);
            if (WiFi.getMode() != WIFI_AP) {
                WiFi.mode(WIFI_AP);
            }
            esp_coex_preference_set(ESP_COEX_PREFER_BT);
            LOGLN("[WIFI] No saved networks found after 2 passes -> hotspot only (BLE priority locked).");
        }
    }

    bool btnAtomLow = (digitalRead(BTN_ATOM_GPIO) == LOW);
    static bool prevAtomLow = false;
    static unsigned long atomPressStartMs = 0;
    if (btnAtomLow) {
        if (!prevAtomLow) atomPressStartMs = now;
        else if (atomPressStartMs > 0 && elapsedMs(now, atomPressStartMs) > 10000UL) { prefs.clear(); delay(500); ESP.restart(); }
    } else {
        atomPressStartMs = 0;
    }
    prevAtomLow = btnAtomLow;

    if (hoseClearActive && (elapsedMs(now, hoseClearTimer) >= (unsigned long)evacSeconds * 1000UL)) {
        LOGLN("[DEBRIS EVAC] Evacuation complete -> Motor OFF");
        hoseClearActive = false; forceStopNow = true; evaluateMotorDemand();
    }
    delay(2);
}
