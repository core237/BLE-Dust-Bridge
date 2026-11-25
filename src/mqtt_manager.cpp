#include <WiFi.h>
#include <ArduinoJson.h>
#include "app_state.h"
#include "mqtt_manager.h"
#include "ble_manager.h"

void mqttPublish(const String& topic, const String& payload, bool retained) {
    if (mqtt_en && mqttClient.connected()) {
        mqttClient.publish(topic.c_str(), payload.c_str(), retained);
    }
}

static String macToTopicId(const String& mac) {
    String id = "";
    id.reserve(12);
    for (size_t i = 0; i < mac.length(); i++) {
        char c = mac[i];
        if (c == ':') continue;
        if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
        id += c;
    }
    return id;
}

static uint8_t rssiToBars(int rssi) {
    if (rssi >= 0) return 0;
    if (rssi >= -60) return 4;
    if (rssi >= -72) return 3;
    if (rssi >= -84) return 2;
    return 1;
}

void publishBridgeTelemetry(unsigned long now) {
    if (!mqtt_en || !mqttClient.connected()) return;
    String ipStr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    String ssidStr = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : String("dustbridge_setup");
    int wifiRssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    unsigned long upSec = now / 1000UL;

    mqttPublish("BLE_DB/bridge/status", "online", true);

    StaticJsonDocument<320> doc;
    doc["status"] = "online";
    doc["ip"] = ipStr;
    doc["hostname"] = hostname;
    doc["wifi_ssid"] = ssidStr;
    doc["wifi_rssi"] = wifiRssi;
    doc["uptime_sec"] = upSec;
    doc["free_heap"] = ESP.getFreeHeap();
    String out;
    serializeJson(doc, out);
    mqttPublish("BLE_DB/bridge/telemetry", out, true);
}

void publishExtractorTelemetry(unsigned long now) {
    if (!mqtt_en || !mqttClient.connected()) return;
    ExtractorTelemetry tSnap;
    portENTER_CRITICAL(&telemMux);
    tSnap = extTelem;
    portEXIT_CRITICAL(&telemMux);

    const char* connStr = (connState == STATE_CONNECTED) ? "CONNECTED" :
                          ((connState == STATE_CONNECTING) ? "CONNECTING" : "DISCONNECTED");
    const char* stateStr = (currentPowerState == 1) ? "ON" : "OFF";
    uint8_t liveOvr = 0;
    if (webUiDemandActive) liveOvr = webUiSpeedOverride;
    else if (hoseMomentaryActive) { MonitoredDevice* s = findMonitoredByMac(activeHoseMomMac); if (s) liveOvr = s->liveSpeedOverride; }
    else if (hoseToggleDemandActive) { MonitoredDevice* s = findMonitoredByMac(activeHoseToggleMac); if (s) liveOvr = s->liveSpeedOverride; }
    else if (btnToggleDemandActive) { MonitoredDevice* s = findMonitoredByMac(activeBtnToggleMac); if (s) liveOvr = s->liveSpeedOverride; }
    else if (toolBattDemandActive) { MonitoredDevice* s = findMonitoredByMac(activeToolBattMac); if (s) liveOvr = s->liveSpeedOverride; }

    unsigned long evacRemSec = 0;
    if (hoseClearActive && evacSeconds > 0) {
        unsigned long totalMs = (unsigned long)evacSeconds * 1000UL;
        unsigned long el = elapsedMs(now, hoseClearTimer);
        if (el < totalMs) evacRemSec = (totalMs - el + 999UL) / 1000UL;
    }

    uint32_t filterRemSec = (tSnap.filterRunSec < 180000UL) ? (180000UL - tSnap.filterRunSec) : 0UL;
    float motorHours = tSnap.motorRunSec / 3600.0f;
    float filterRemHours = filterRemSec / 3600.0f;
    uint8_t filterRemPct = (uint8_t)((filterRemSec * 100UL) / 180000UL);
    String fwCombined = "V " + String(tSnap.mainSwMajor) + "/" + String(tSnap.bleSwMajor) +
                        " (" + String(tSnap.mainFw) + " / " + String(tSnap.bleFw) + ")";

    mqttPublish("BLE_DB/extractor/connection", connStr, true);
    mqttPublish("BLE_DB/extractor/state", stateStr, true);
    mqttPublish("BLE_DB/extractor/speed", String(currentSuctionPct), true);

    StaticJsonDocument<768> doc;
    doc["connection"] = connStr;
    doc["paired"] = (pairedMac.length() > 0);
    doc["paired_mac"] = pairedMac;
    doc["name"] = extFriendlyName;
    doc["rssi"] = pairedRssi;
    doc["signal_bars"] = (connState == STATE_CONNECTED && pairedRssi < 0) ? rssiToBars(pairedRssi) : 0;
    doc["state"] = stateStr;
    doc["speed"] = currentSuctionPct;
    doc["preset_speed"] = activePresetSpeed;
    doc["override_speed"] = liveOvr;
    doc["evac_active"] = hoseClearActive;
    doc["evac_sec"] = evacSeconds;
    doc["evac_rem_sec"] = evacRemSec;
    doc["motor_run_sec"] = tSnap.motorRunSec;
    doc["motor_hours"] = motorHours;
    doc["cycles"] = tSnap.operatingCycles;
    doc["filter_run_sec"] = tSnap.filterRunSec;
    doc["filter_remaining_sec"] = filterRemSec;
    doc["filter_remaining_hours"] = filterRemHours;
    doc["filter_remaining_pct"] = filterRemPct;
    doc["water_level_protection"] = tSnap.waterLevelActive ? "Active" : "Inactive";
    doc["temp1_f"] = tSnap.temp1F;
    doc["temp2_f"] = tSnap.temp2F;
    doc["faults"] = tSnap.faultText;
    doc["model"] = tSnap.modelName;
    doc["firmware"] = fwCombined;

    String out;
    serializeJson(doc, out);
    mqttPublish("BLE_DB/extractor/telemetry", out, true);
}

void publishDevicesTelemetry(unsigned long now) {
    if (!mqtt_en || !mqttClient.connected()) return;

    for (const auto& m : monitoredList) {
        String id = macToTopicId(m.address);
        if (id.length() == 0) continue;

        const BleEntry* liveBle = nullptr;
        for (const auto& b : bleList) {
            if (b.address.equalsIgnoreCase(m.address)) { liveBle = &b; break; }
        }

        int gSlot = m.isGatt ? findGattSlotByMac(m.address) : -1;
        bool gattLinked = (gSlot >= 0);
        bool hasFreshRssi = (liveBle && liveBle->rssi < 0 && (liveBle->lastSeen == 0 || elapsedMs(now, liveBle->lastSeen) <= 300000UL));
        int effRssi = hasFreshRssi ? liveBle->rssi : 0;
        uint8_t bars = hasFreshRssi ? rssiToBars(effRssi) : 0;
        unsigned long seenAgeSec = (liveBle && liveBle->lastSeen > 0) ? (elapsedMs(now, liveBle->lastSeen) / 1000UL) : 999999UL;

        uint32_t effMat = (m.matNum > 0) ? m.matNum : (liveBle ? liveBle->matNum : 0);
        String effModel = lookupFestoolModelByTnr(effMat);
        String effType = (m.btnType > 0) ? "Button" : devCatToTypeStr(m.devCat);
        if (effType == "Other" && liveBle && liveBle->type.length() > 0 && liveBle->type != "Other") {
            effType = liveBle->type;
        }
        int effSoc = liveBle ? liveBle->soc : -1;
        int32_t effWatts = (liveBle && liveBle->watts >= 0) ? liveBle->watts : m.lastWatts;
        bool isActive = (m.btnType > 0)
            ? (m.toggleActive || m.momActive || m.btnPhase == BTN_HELD)
            : m.toolActive;

        StaticJsonDocument<512> doc;
        doc["mac"] = m.address;
        doc["name"] = m.name;
        if (m.advName.length() > 0) doc["adv_name"] = m.advName;
        if (effModel.length() > 0) doc["model"] = effModel;
        if (effMat > 0) doc["mat_num"] = effMat;
        doc["type"] = effType;
        doc["active"] = isActive;
        doc["speed"] = m.speed;
        doc["live_speed_override"] = m.liveSpeedOverride;
        doc["rssi"] = effRssi;
        doc["signal_bars"] = bars;
        doc["last_seen_age_sec"] = seenAgeSec;
        if (effSoc >= 0) doc["battery_pct"] = effSoc;
        if (effType == "Tool Plug") {
            doc["watts"] = hasFreshRssi && effWatts >= 0 ? effWatts : -1;
            doc["plug_status"] = hasFreshRssi ? (isActive ? "ON" : "IDLE") : "OFF";
            doc["watts_threshold"] = m.wattsThr;
        }
        if (m.btnType > 0) {
            doc["connection_type"] = m.isGatt ? "GATT" : "ADV";
            doc["gatt_connected"] = gattLinked;
            doc["hose_mode"] = (m.isGatt && m.isHose);
            doc["toggle_active"] = m.toggleActive;
            doc["momentary_active"] = m.momActive;
        }

        String out;
        serializeJson(doc, out);
        mqttPublish("BLE_DB/devices/" + id + "/telemetry", out, true);
    }

    for (const auto& b : bleList) {
        if (!b.isKnownDevice || elapsedMs(now, b.lastSeen) > 60000UL) continue;
        if (pairedMac.length() > 0 && b.address.equalsIgnoreCase(pairedMac)) continue;
        if (findMonitoredByMac(b.address) != nullptr) continue;
        if (b.type == "Extractor" || b.type == "Festool") continue;

        String id = macToTopicId(b.address);
        if (id.length() == 0) continue;

        StaticJsonDocument<320> doc;
        doc["mac"] = b.address;
        doc["name"] = b.name;
        doc["type"] = b.type;
        doc["rssi"] = b.rssi;
        doc["signal_bars"] = rssiToBars(b.rssi);
        doc["last_seen_age_sec"] = elapsedMs(now, b.lastSeen) / 1000UL;
        if (b.matNum > 0) doc["mat_num"] = b.matNum;
        if (b.soc >= 0) doc["battery_pct"] = b.soc;
        if (b.watts >= 0) doc["watts"] = b.watts;
        if (b.state >= 0) doc["active"] = (b.state == 1);

        String out;
        serializeJson(doc, out);
        mqttPublish("BLE_DB/discovered/" + id + "/telemetry", out, false);
    }
}

void publishAllMqttTelemetry(unsigned long now) {
    if (!mqtt_en || !mqttClient.connected()) return;
    lastMqttHeartbeatMs = now;
    publishBridgeTelemetry(now);
    publishExtractorTelemetry(now);
    publishDevicesTelemetry(now);
}
