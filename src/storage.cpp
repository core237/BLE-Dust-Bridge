#include <ArduinoJson.h>
#include "app_state.h"
#include "storage.h"

void saveMonitoredToNvs() {
    needSaveMonitored = false;
    needSaveMonitoredAtMs = 0;
    DynamicJsonDocument doc(4096);
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& m : monitoredList) {
        JsonObject o = arr.createNestedObject();
        o["mac"] = m.address;
        if (m.addrType == 0 || m.addrType == 1) o["at"] = m.addrType;
        o["nm"] = m.name;
        o["advnm"] = m.advName;
        o["spd"] = m.speed;
        o["bt"] = m.btnType;
        o["raw"] = m.activeRaw;
        o["cn"] = m.customNamed;
        o["gatt"] = m.isGatt;
        o["grdy"] = m.gattReady;
        o["primary"] = (m.isPrimary && m.isGatt);
        o["adv"] = m.canAdv;
        o["cg"] = m.canGatt;
        o["mat"] = m.matNum;
        o["cat"] = m.devCat;
        o["wthr"] = m.wattsThr;
        if (m.periodicBeacon && !m.canAdv) o["bcn"] = true;
    }
    if (doc.overflowed()) {
        LOGLN("[NVS] Warning: monitored_list JSON overflowed 4096B buffer; skipping save.");
        return;
    }
    String jsonStr;
    serializeJson(doc, jsonStr);
    prefs.putString("mon_json", jsonStr);
    refreshMonMacCache();
    mqttDevicesDirty = true;
}

void loadMonitoredFromNvs() {
    String jsonStr = prefs.isKey("mon_json") ? prefs.getString("mon_json", "[]") : "[]";
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, jsonStr) != DeserializationError::Ok || !doc.is<JsonArray>()) {
        refreshMonMacCache();
        return;
    }
    monitoredList.clear();
    for (JsonObject o : doc.as<JsonArray>()) {
        String mac = o["mac"] | "";
        if (mac.length() == 0) continue;
        if (pairedMac.length() > 0 && mac.equalsIgnoreCase(pairedMac)) continue;
        String nm = o["nm"] | "Device";
        String advNm = o["advnm"] | "";
        uint8_t spd = o["spd"] | 20;
        uint8_t bt = o["bt"] | 0;
        String raw = o["raw"] | "";
        bool cn = o["cn"] | false;
        bool gatt = o["gatt"] | false;
        bool gReady = o.containsKey("grdy") ? (bool)(o["grdy"] | false) : (!gatt);
        if (!gatt) gReady = true;
        bool cAdv = o["adv"] | (!gatt);
        bool rawPrimary = o.containsKey("primary") ? (bool)(o["primary"] | false) : (bool)(o["hose"] | false);
        bool isPrimary = gatt && (!cAdv || rawPrimary);
        bool cGatt = o["cg"] | gatt;
        uint32_t mat = o["mat"] | 0;
        uint8_t cat = o["cat"] | 0;
        uint16_t wThr = o["wthr"] | 60;
        uint8_t aType = o.containsKey("at") ? (uint8_t)(o["at"] | 255) : inferRandomAddrType(mac, 255);
        bool pBcn = cAdv ? false : (o["bcn"] | false);
        if (bt > 0) {
            cat = CAT_BUTTON;
            String upNm = nm; upNm.toUpperCase();
            String upAdv = advNm; upAdv.toUpperCase();
            bool isCtfRemote = (raw == "ADV:FESTOOL_REMOTE" || mat == 202097 || mat == 204842 ||
                                mat == 10029918 || mat == 10030046 ||
                                upNm.indexOf("CT-F") >= 0 || upNm.indexOf("CTF") >= 0 ||
                                upAdv.indexOf("CT-F") >= 0 || upAdv.indexOf("CTF") >= 0);
            if (isCtfRemote) {
                raw = "ADV:STREAM";
                cAdv = true;
                cGatt = false;
                gatt = false;
                gReady = true;
                isPrimary = false;
            }
        }
        monitoredList.push_back({mac, nm, advNm, spd, bt, raw, cn, false, 0, gatt, gReady, false, 0, isPrimary, cAdv, mat, cat, wThr, -1, cGatt, aType, pBcn});
    }
    refreshMonMacCache();
}

void saveWifiListToNvs() {
    StaticJsonDocument<1536> doc;
    JsonArray arr = doc.to<JsonArray>();
    for (const auto& w : wifiList) {
        if (w.ssid.length() == 0) continue;
        JsonObject o = arr.createNestedObject();
        o["ssid"] = w.ssid;
        o["pass"] = w.pass;
    }
    String jsonStr;
    serializeJson(doc, jsonStr);
    prefs.putString("wifi_json", jsonStr);
}

void loadSettingsFromNvs() {
    String wifiJson = prefs.isKey("wifi_json") ? prefs.getString("wifi_json", "[]") : "[]";
    StaticJsonDocument<1536> doc;
    if (deserializeJson(doc, wifiJson) == DeserializationError::Ok && doc.is<JsonArray>()) {
        wifiList.clear();
        for (JsonObject o : doc.as<JsonArray>()) {
            String ssid = o["ssid"] | "";
            String pass = o["pass"] | "";
            if (ssid.length() > 0) wifiList.push_back({ssid, pass});
        }
    }

    mqtt_en = prefs.getBool("mqtt_en", false);
    mqtt_ip = prefs.isKey("mqtt_ip") ? prefs.getString("mqtt_ip", "") : "";
    mqtt_port = prefs.getInt("mqtt_port", 1883);
    if (mqtt_port <= 0 || mqtt_port > 65535) mqtt_port = 1883;
    mqtt_user = prefs.isKey("mqtt_user") ? prefs.getString("mqtt_user", "") : "";
    mqtt_pass = prefs.isKey("mqtt_pass") ? prefs.getString("mqtt_pass", "") : "";

    bool authInit = prefs.getBool("auth_init", false);
    if (!authInit) {
        auth_en = (DEFAULT_AUTH_EN != 0);
        auth_user = DEFAULT_AUTH_USER;
        auth_salt = generateRandomHex(16);
        auth_hash = sha256Hex(auth_salt + ":" + auth_user + ":" + String(DEFAULT_AUTH_PASS));
        prefs.putBool("auth_init", true);
        prefs.putBool("auth_en", auth_en);
        prefs.putString("auth_user", auth_user);
        prefs.putString("auth_salt", auth_salt);
        prefs.putString("auth_hash", auth_hash);
    } else {
        auth_en = prefs.getBool("auth_en", (DEFAULT_AUTH_EN != 0));
        auth_user = prefs.isKey("auth_user") ? prefs.getString("auth_user", DEFAULT_AUTH_USER) : DEFAULT_AUTH_USER;
        auth_salt = prefs.isKey("auth_salt") ? prefs.getString("auth_salt", "") : "";
        auth_hash = prefs.isKey("auth_hash") ? prefs.getString("auth_hash", "") : "";
        if (auth_salt.length() != 32 || auth_hash.length() != 64) {
            auth_salt = generateRandomHex(16);
            auth_hash = sha256Hex(auth_salt + ":" + auth_user + ":" + String(DEFAULT_AUTH_PASS));
            prefs.putString("auth_salt", auth_salt);
            prefs.putString("auth_hash", auth_hash);
        }
    }
}
