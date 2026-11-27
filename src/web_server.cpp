#include <WiFi.h>
#include <esp_coexist.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <algorithm>
#include "app_state.h"
#include "web_server.h"
#include "web_pages.h"
#include "ble_manager.h"
#include "fmp_protocol.h"
#include "motor_control.h"
#include "storage.h"

static bool verifyAndConsumeSignedNonce(const String& nonce, unsigned long nowMs) {
    int dot1 = nonce.indexOf('.');
    if (dot1 != 16) return false;
    int dot2 = nonce.indexOf('.', dot1 + 1);
    if (dot2 <= dot1 + 1) return false;

    String rHex = nonce.substring(0, dot1);
    String tsStr = nonce.substring(dot1 + 1, dot2);
    String sig = nonce.substring(dot2 + 1);
    if (sig.length() != 32 || tsStr.length() > 10) return false;

    for (size_t i = 0; i < tsStr.length(); i++) {
        if (tsStr[i] < '0' || tsStr[i] > '9') return false;
    }

    unsigned long tsSec = strtoul(tsStr.c_str(), nullptr, 10);
    unsigned long nowSec = nowMs / 1000UL;
    if (nowSec < tsSec || (nowSec - tsSec) > 30UL) return false;

    String expectedSig = sha256Hex(bootAuthSecret + ":" + rHex + ":" + tsStr).substring(0, 32);
    if (!constantTimeHexEquals(sig, expectedSig)) return false;

    for (size_t i = 0; i < MAX_USED_NONCES; i++) {
        if (usedLoginNonces[i][0] != '\0' && constantTimeHexEquals(nonce, String(usedLoginNonces[i]))) {
            return false;
        }
    }

    strncpy(usedLoginNonces[usedLoginNonceNext], nonce.c_str(), sizeof(usedLoginNonces[0]) - 1);
    usedLoginNonces[usedLoginNonceNext][sizeof(usedLoginNonces[0]) - 1] = '\0';
    usedLoginNonceNext = (usedLoginNonceNext + 1) % MAX_USED_NONCES;
    return true;
}

void clearAllAuthSessions() {
    for (size_t i = 0; i < MAX_AUTH_SESSIONS; i++) {
        authSessions[i].active = false;
        authSessions[i].token[0] = '\0';
    }
}

static String extractSessionCookie() {
    if (!server.hasHeader("Cookie")) return "";
    String cookieHdr = server.header("Cookie");
    int idx = cookieHdr.indexOf("bldb_sid=");
    if (idx < 0) return "";
    int valStart = idx + 9;
    int valEnd = cookieHdr.indexOf(';', valStart);
    String tok = (valEnd >= 0) ? cookieHdr.substring(valStart, valEnd) : cookieHdr.substring(valStart);
    tok.trim();
    return tok;
}

static bool isSessionAuthenticated() {
    if (!auth_en) return true;
    String tok = extractSessionCookie();
    if (tok.length() != 32) return false;
    unsigned long now = millis();
    for (size_t i = 0; i < MAX_AUTH_SESSIONS; i++) {
        if (!authSessions[i].active) continue;
        if (elapsedMs(now, authSessions[i].lastSeenMs) > AUTH_SESSION_TTL_MS) {
            authSessions[i].active = false;
            continue;
        }
        if (constantTimeHexEquals(tok, String(authSessions[i].token))) {
            authSessions[i].lastSeenMs = now;
            return true;
        }
    }
    return false;
}

static bool checkAuth(bool isApi = true) {
    if (isSessionAuthenticated()) return true;
    if (isApi) {
        server.send(401, "application/json", "{\"error\":\"unauthorized\"}");
    } else {
        server.sendHeader("Location", "/login", true);
        server.send(302, "text/plain", "");
    }
    return false;
}

static void buildStatusJson(String& out) {
    static DynamicJsonDocument doc(8192);
    doc.clear();
    unsigned long now = millis();
    unsigned long evacRemMs = 0;
    if (hoseClearActive && evacSeconds > 0) {
        unsigned long totalMs = (unsigned long)evacSeconds * 1000UL;
        unsigned long elapsed = elapsedMs(now, hoseClearTimer);
        if (elapsed < totalMs) evacRemMs = totalMs - elapsed;
    }

    uint8_t gattSlotsUsed = 0;
    bool slotConnSnap[MAX_GATT_BTN_SLOTS] = {false};
    bool slotBusySnap[MAX_GATT_BTN_SLOTS] = {false};
    bool slotPressSnap[MAX_GATT_BTN_SLOTS] = {false};
    portENTER_CRITICAL(&gattSlotMux);
    for (int i = 0; i < MAX_GATT_BTN_SLOTS; i++) {
        slotConnSnap[i] = gattSlots[i].connected;
        slotBusySnap[i] = gattSlots[i].connected || gattSlots[i].connecting;
        slotPressSnap[i] = gattSlots[i].pressed;
        if (slotBusySnap[i]) gattSlotsUsed++;
    }
    portEXIT_CRITICAL(&gattSlotMux);
    uint8_t gattSwitchesCfg = 0;
    for (const auto& m : monitoredList) {
        if (m.btnType > 0 && m.isGatt) gattSwitchesCfg++;
    }

    doc["state"] = (connState == STATE_CONNECTED ? "CONNECTED" : (connState == STATE_CONNECTING ? "CONNECTING" : "DISCONNECTED"));
    doc["fw_version"] = "1.1.0";
    doc["auth_en"] = auth_en;
    doc["power"] = currentPowerState; doc["speed"] = currentSuctionPct;
    doc["preset_speed"] = activePresetSpeed; doc["override_speed"] = getActiveLiveSpeedOverride();
    doc["paired_mac"] = pairedMac; doc["paired_rssi"] = pairedRssi;
    doc["ext_name"] = extFriendlyName; doc["evac_sec"] = evacSeconds; doc["evac_rem_ms"] = evacRemMs;
    doc["gatt_slots_used"] = gattSlotsUsed;
    doc["gatt_slots_max"] = MAX_GATT_BTN_SLOTS;
    doc["gatt_switches_cfg"] = gattSwitchesCfg;

    ExtractorTelemetry tSnap;
    portENTER_CRITICAL(&telemMux); tSnap = extTelem; portEXIT_CRITICAL(&telemMux);
    JsonObject tObj = doc.createNestedObject("telem");
    tObj["motor_sec"] = tSnap.motorRunSec; tObj["cycles"] = tSnap.operatingCycles; tObj["filter_sec"] = tSnap.filterRunSec;
    tObj["water_active"] = tSnap.waterLevelActive; tObj["temp1_f"] = tSnap.temp1F; tObj["temp2_f"] = tSnap.temp2F;
    tObj["faults"] = tSnap.faultText; tObj["model"] = tSnap.modelName;
    tObj["sw_ver"] = "V " + String(tSnap.mainSwMajor) + "/" + String(tSnap.bleSwMajor);
    tObj["main_fw"] = tSnap.mainFw; tObj["ble_fw"] = tSnap.bleFw;

    JsonArray monArr = doc.createNestedArray("monitored");
    for (auto& m : monitoredList) {
        JsonObject o = monArr.createNestedObject();
        int gSlot = m.isGatt ? findGattSlotByMac(m.address) : -1;
        bool gattLinked = (gSlot >= 0) && slotBusySnap[gSlot];
        bool btnLive = false;
        if (m.btnType > 0 && !m.waitReleaseAfterPair) {
            if (m.isGatt) {
                bool gattPressed = (gSlot >= 0 && slotConnSnap[gSlot] && slotPressSnap[gSlot]);
                bool gestureHeld = (m.btnPhase == BTN_HELD);
                btnLive = gattPressed || gestureHeld;
            } else {
                btnLive = (m.btnPhase == BTN_HELD) || (m.toolActive && (m.toolLastSeen > 0 && elapsedMs(now, m.toolLastSeen) < ADV_TOGGLE_TAP_RELEASE_MS));
            }
        }
        const BleEntry* liveBle = nullptr;
        for (const auto& b : bleList) {
            if (b.address.equalsIgnoreCase(m.address)) { liveBle = &b; break; }
        }
        uint32_t effMat = (m.matNum > 0) ? m.matNum : (liveBle ? liveBle->matNum : 0);
        String effAdvName = m.advName;
        if (effAdvName.length() == 0 && liveBle && liveBle->advName.length() > 0) effAdvName = liveBle->advName;
        String effModel = lookupFestoolModelByTnr(effMat);
        String effDevType = (m.btnType > 0) ? "Button" : devCatToTypeStr(m.devCat);
        if (effDevType == "Other" && liveBle && liveBle->type.length() > 0 && liveBle->type != "Other") {
            effDevType = liveBle->type;
        }
        if (effDevType == "Other") {
            String lowRaw = m.activeRaw; lowRaw.toLowerCase();
            String lowAdv = effAdvName; lowAdv.toLowerCase();
            if (effMat > 0) effDevType = "Battery";
            else if (lowAdv.startsWith("shelly") || lowAdv.startsWith("scc") || lowRaw.indexOf("a9 0b 53 43") != -1) effDevType = "Tool Plug";
            else effDevType = "Auto Tool";
        }
        o["mac"] = m.address; o["name"] = m.name; o["adv_name"] = effAdvName; o["model_name"] = effModel;
        o["dev_type"] = effDevType; o["mat_num"] = effMat;
        bool hasFreshRssi = (liveBle && liveBle->rssi < 0 && (liveBle->lastSeen == 0 || elapsedMs(now, liveBle->lastSeen) <= 300000UL));
        o["rssi"] = hasFreshRssi ? liveBle->rssi : 0;
        o["rssi_age"] = (liveBle && liveBle->lastSeen > 0) ? elapsedMs(now, liveBle->lastSeen) : 999999UL;
        o["soc"] = liveBle ? liveBle->soc : -1;
        o["watts_thr"] = m.wattsThr;
        o["watts"] = (liveBle && liveBle->watts >= 0) ? liveBle->watts : m.lastWatts;
        o["speed"] = m.speed; o["btnType"] = m.btnType;
        o["is_gatt"] = m.isGatt; o["can_adv"] = m.canAdv; o["can_gatt"] = m.canGatt;
        o["is_primary"] = (m.isGatt && (!m.canAdv || m.isPrimary)); o["gatt_linked"] = gattLinked; o["btn_live"] = btnLive;
        o["cfg"] = isMonitoredConfiguring(m, now);
        o["await_rel"] = m.waitReleaseAfterPair;
    }

    pruneStaleBleEntries(now);
    auto appendBleDeviceObj = [&](JsonArray& arr, const BleEntry& it, bool isMon) {
        JsonObject o = arr.createNestedObject();
        o["name"] = it.name; o["adv_name"] = it.advName; o["address"] = it.address; o["addr_type"] = it.addrType; o["rssi"] = it.rssi;
        o["isKnownDevice"] = it.isKnownDevice; o["type"] = it.type; o["soc"] = it.soc; o["state"] = it.state;
        o["can_adv"] = it.canAdv; o["can_gatt"] = it.canGatt;
        o["age"] = elapsedMs(now, it.lastSeen);
        if (it.matNum > 0) {
            o["mat_num"] = it.matNum;
            String mdl = lookupFestoolModelByTnr(it.matNum);
            if (mdl.length() > 0) o["model_name"] = mdl;
        }
        if (it.watts >= 0) o["watts"] = it.watts;
        if (isMon || it.canAdv || it.rawData.startsWith("GATT")) o["raw"] = it.rawData;
    };

    JsonArray devArr = doc.createNestedArray("devices");
    std::vector<size_t> unpinnedIndices;
    unpinnedIndices.reserve(bleList.size());
    for (size_t idx = 0; idx < bleList.size(); idx++) {
        const auto& it = bleList[idx];
        bool isPinned = (pairedMac.length() > 0 && it.address.equalsIgnoreCase(pairedMac));
        bool isMon = (findMonitoredByMac(it.address) != nullptr);
        if (isPinned || isMon) {
            appendBleDeviceObj(devArr, it, isMon);
        } else {
            unpinnedIndices.push_back(idx);
        }
    }
    static const size_t MAX_DISCOVERED_JSON = 12;
    if (unpinnedIndices.size() > MAX_DISCOVERED_JSON) {
        std::sort(unpinnedIndices.begin(), unpinnedIndices.end(), [&](size_t a, size_t b) {
            return bleList[a].lastSeen > bleList[b].lastSeen;
        });
        unpinnedIndices.resize(MAX_DISCOVERED_JSON);
        std::sort(unpinnedIndices.begin(), unpinnedIndices.end());
    }
    for (size_t idx : unpinnedIndices) {
        appendBleDeviceObj(devArr, bleList[idx], false);
    }
    if (doc.overflowed()) {
        LOGLN("[API] Warning: /api/status JSON buffer overflowed 8192B capacity.");
    }
    out.clear();
    out.reserve(8192);
    serializeJson(doc, out);
}

static void sendJsonStatus() {
    static String out;
    buildStatusJson(out);
    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", out);
}

void setupWebServer() {
    const char* headerKeys[] = {"Cookie"};
    server.collectHeaders(headerKeys, 1);

    server.on("/favicon.ico", HTTP_GET, []() {
        server.send(204, "image/x-icon", "");
    });

    server.on("/login", HTTP_GET, []() {
        if (!auth_en || isSessionAuthenticated()) {
            server.sendHeader("Location", "/", true);
            server.send(302, "text/plain", "");
            return;
        }
        server.send_P(200, "text/html", LOGIN_HTML);
    });

    server.on("/api/auth/challenge", HTTP_GET, []() {
        unsigned long now = millis();
        String reqUser = server.hasArg("user") ? server.arg("user") : "";
        reqUser.trim();

        String rHex = generateRandomHex(8);
        String tsStr = String(now / 1000UL);
        String sig = sha256Hex(bootAuthSecret + ":" + rHex + ":" + tsStr).substring(0, 32);
        String nonce = rHex + "." + tsStr + "." + sig;

        String saltOut = (reqUser == auth_user && auth_salt.length() == 32)
            ? auth_salt
            : sha256Hex(auth_salt + ":decoy:" + reqUser).substring(0, 32);

        StaticJsonDocument<256> doc;
        doc["nonce"] = nonce;
        doc["salt"] = saltOut;
        String out;
        serializeJson(doc, out);
        server.send(200, "application/json", out);
    });

    server.on("/api/auth/login", HTTP_POST, []() {
        unsigned long now = millis();
        if (lastFailedLoginMs > 0 && elapsedMs(now, lastFailedLoginMs) < 1000UL) {
            server.send(429, "application/json", "{\"error\":\"Please wait a moment before retrying.\"}");
            return;
        }
        if (!server.hasArg("plain")) {
            server.send(400, "application/json", "{\"error\":\"Invalid request.\"}");
            return;
        }
        StaticJsonDocument<384> doc;
        if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
            server.send(400, "application/json", "{\"error\":\"Invalid JSON.\"}");
            return;
        }
        String u = doc["user"] | "";
        u.trim();
        String nonce = doc["nonce"] | "";
        String proof = doc["proof"] | "";

        if (!verifyAndConsumeSignedNonce(nonce, now)) {
            lastFailedLoginMs = now;
            server.send(401, "application/json", "{\"error\":\"Login challenge expired. Please try again.\"}");
            return;
        }

        String expectedProof = sha256Hex(nonce + ":" + auth_hash);
        if (u == auth_user && constantTimeHexEquals(proof, expectedProof)) {
            String sid = generateRandomHex(16);
            size_t sessIdx = MAX_AUTH_SESSIONS;
            size_t oldestIdx = 0;
            unsigned long oldestTs = ULONG_MAX;
            for (size_t i = 0; i < MAX_AUTH_SESSIONS; i++) {
                if (!authSessions[i].active || elapsedMs(now, authSessions[i].lastSeenMs) > AUTH_SESSION_TTL_MS) {
                    sessIdx = i;
                    break;
                }
                if (authSessions[i].lastSeenMs < oldestTs) {
                    oldestTs = authSessions[i].lastSeenMs;
                    oldestIdx = i;
                }
            }
            if (sessIdx >= MAX_AUTH_SESSIONS) sessIdx = oldestIdx;
            AuthSession& s = authSessions[sessIdx];
            strncpy(s.token, sid.c_str(), sizeof(s.token) - 1);
            s.token[sizeof(s.token) - 1] = '\0';
            s.lastSeenMs = now;
            s.active = true;
            lastFailedLoginMs = 0;

            server.sendHeader("Set-Cookie", "bldb_sid=" + sid + "; Path=/; HttpOnly; SameSite=Strict; Max-Age=86400");
            server.send(200, "application/json", "{\"ok\":true}");
            LOGF("[AUTH] Web UI login succeeded for user '%s'\n", u.c_str());
        } else {
            lastFailedLoginMs = now;
            server.send(401, "application/json", "{\"error\":\"Invalid username or password.\"}");
            LOGF("[AUTH] Failed Web UI login attempt for user '%s'\n", u.c_str());
        }
    });

    server.on("/api/auth/logout", HTTP_POST, []() {
        String tok = extractSessionCookie();
        if (tok.length() == 32) {
            for (size_t i = 0; i < MAX_AUTH_SESSIONS; i++) {
                if (authSessions[i].active && constantTimeHexEquals(tok, String(authSessions[i].token))) {
                    authSessions[i].active = false;
                }
            }
        }
        server.sendHeader("Set-Cookie", "bldb_sid=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0");
        server.send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/", []() { if (!checkAuth(false)) return; server.sendHeader("Content-Encoding", "gzip"); server.send_P(200, "text/html", (const char*)INDEX_HTML_GZ, INDEX_HTML_GZ_LEN); });
    server.on("/api/identify", []() { if (!checkAuth(true)) return; sendFmpIdentify(true); server.send(200, "application/json", "{}"); });
    server.on("/api/telem/refresh", []() { if (!checkAuth(true)) return; startTelemetryPoll(true); sendJsonStatus(); });

    server.on("/api/evac", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("sec")) {
            int s = server.arg("sec").toInt();
            evacSeconds = (uint8_t)(s < 0 ? 0 : (s > 60 ? 60 : s));
            prefs.putUChar("evac_sec", evacSeconds);
            mqttTelemDirty = true;
            LOGF("[EXTRACT] Debris evacuation set to %us\n", evacSeconds);
        }
        sendJsonStatus();
    });

    server.on("/ping", HTTP_GET, []() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/api/status", []() { noteWebUiActivity(); if (!checkAuth(true)) return; sendJsonStatus(); });

    server.on("/api/settings", HTTP_GET, []() {
        if (!checkAuth(true)) return;
        StaticJsonDocument<1536> doc;
        JsonArray wifis = doc.createNestedArray("wifis");
        for (const auto& w : wifiList) {
            JsonObject obj = wifis.createNestedObject();
            obj["ssid"] = w.ssid; obj["pass"] = w.pass.length() > 0 ? "********" : "";
        }
        doc["mqtt_en"] = mqtt_en; doc["mqtt_ip"] = mqtt_ip; doc["mqtt_port"] = mqtt_port;
        doc["mqtt_user"] = mqtt_user; doc["mqtt_pass"] = mqtt_pass.length() > 0 ? "********" : "";
        doc["auth_en"] = auth_en; doc["auth_user"] = auth_user;
        bool staConn = (WiFi.status() == WL_CONNECTED);
        doc["active_ssid"] = staConn ? WiFi.SSID() : "";
        doc["active_rssi"] = staConn ? WiFi.RSSI() : 0;
        String out; serializeJson(doc, out);
        server.send(200, "application/json", out);
    });

    server.on("/api/wifi/reorder", HTTP_POST, []() {
        if (!checkAuth(true)) return;
        if (!server.hasArg("plain")) {
            server.send(400, "text/plain", "Bad Request");
            return;
        }
        StaticJsonDocument<1536> doc;
        if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok || !doc["wifis"].is<JsonArray>()) {
            server.send(400, "text/plain", "Invalid JSON");
            return;
        }
        std::vector<SavedWiFi> prevWifis = wifiList;
        std::vector<SavedWiFi> reordered;
        JsonArray wifis = doc["wifis"].as<JsonArray>();
        for (JsonObject w : wifis) {
            String ssid = w["ssid"].as<String>();
            if (ssid.length() == 0) continue;
            String pass = "";
            for (const auto& oldW : prevWifis) {
                if (oldW.ssid.equalsIgnoreCase(ssid)) {
                    pass = oldW.pass;
                    break;
                }
            }
            reordered.push_back({ssid, pass});
        }
        if (!reordered.empty()) {
            wifiList = reordered;
            bgWifiIdx = 0;
            saveWifiListToNvs();
            String orderLog = "";
            for (size_t i = 0; i < wifiList.size(); i++) {
                if (i > 0) orderLog += " -> ";
                orderLog += wifiList[i].ssid;
            }
            LOGF("[WIFI] Priority order saved: %s\n", orderLog.c_str());
        }
        server.send(200, "application/json", "{\"ok\":true}");
    });

    server.on("/api/settings/save", HTTP_POST, []() {
        if (!checkAuth(true)) return;
        if (!server.hasArg("plain")) {
            server.send(400, "application/json", "{\"error\":\"Missing request payload.\"}");
            return;
        }
        StaticJsonDocument<1536> doc;
        if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok || !doc.is<JsonObject>()) {
            server.send(400, "application/json", "{\"error\":\"Invalid JSON payload.\"}");
            return;
        }

        bool nextAuthEn = doc["auth_en"] | auth_en;
        String nextAuthUser = doc.containsKey("auth_user") ? doc["auth_user"].as<String>() : auth_user;
        nextAuthUser.trim();
        String nextAuthPass = doc.containsKey("auth_pass") ? doc["auth_pass"].as<String>() : "";

        if (nextAuthEn) {
            if (nextAuthUser.length() == 0) {
                server.send(400, "application/json", "{\"error\":\"Username cannot be empty.\"}");
                return;
            }
            if (nextAuthUser != auth_user && nextAuthPass.length() == 0) {
                server.send(400, "application/json", "{\"error\":\"Enter a password when changing username.\"}");
                return;
            }
            if (nextAuthPass.length() == 0 && (auth_salt.length() != 32 || auth_hash.length() != 64)) {
                server.send(400, "application/json", "{\"error\":\"Password is required to enable login.\"}");
                return;
            }
        }

        if (doc["wifis"].is<JsonArray>()) {
            std::vector<SavedWiFi> prevWifis = wifiList;
            wifiList.clear();

            JsonArray wifis = doc["wifis"].as<JsonArray>();
            for (JsonObject w : wifis) {
                String ssid = w["ssid"].as<String>();
                if (ssid.length() == 0) continue;
                String pass = w["pass"].as<String>();
                if (pass == "********") {
                    pass = "";
                    for (const auto& oldW : prevWifis) if (oldW.ssid.equalsIgnoreCase(ssid)) { pass = oldW.pass; break; }
                }
                wifiList.push_back({ssid, pass});
            }
            saveWifiListToNvs();
        }

        mqtt_en = doc["mqtt_en"] | mqtt_en;
        if (doc.containsKey("mqtt_ip")) mqtt_ip = doc["mqtt_ip"].as<String>();
        if (doc.containsKey("mqtt_port")) {
            int p = doc["mqtt_port"] | 1883;
            if (p <= 0 || p > 65535) p = 1883;
            mqtt_port = p;
        }
        if (doc.containsKey("mqtt_user")) mqtt_user = doc["mqtt_user"].as<String>();
        if (doc.containsKey("mqtt_pass")) {
            String inMqttPass = doc["mqtt_pass"].as<String>();
            if (inMqttPass != "********") mqtt_pass = inMqttPass;
        }
        prefs.putBool("mqtt_en", mqtt_en); prefs.putString("mqtt_ip", mqtt_ip); prefs.putInt("mqtt_port", mqtt_port);
        prefs.putString("mqtt_user", mqtt_user); prefs.putString("mqtt_pass", mqtt_pass);

        auth_en = nextAuthEn;
        if (nextAuthUser.length() > 0) auth_user = nextAuthUser;
        if (nextAuthPass.length() > 0) {
            auth_salt = generateRandomHex(16);
            auth_hash = sha256Hex(auth_salt + ":" + auth_user + ":" + nextAuthPass);
            clearAllAuthSessions();
        } else if (!auth_en) {
            clearAllAuthSessions();
        }
        prefs.putBool("auth_init", true);
        prefs.putBool("auth_en", auth_en);
        prefs.putString("auth_user", auth_user);
        prefs.putString("auth_salt", auth_salt);
        prefs.putString("auth_hash", auth_hash);

        String saveResp = "{\"ok\":true,\"ip\":\"" + (WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String("")) + "\"}";
        server.send(200, "application/json", saveResp);
        if (currentPowerState == 1) {
            stopExtractorMotor(true);
        }
        requestReboot = true; rebootTimer = millis() + 900UL;
    });

    server.on("/api/wifi/verify", HTTP_POST, []() {
        if (!checkAuth(true)) return;
        if (!server.hasArg("plain")) {
            server.send(400, "application/json", "{\"error\":\"Missing request payload.\"}");
            return;
        }
        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, server.arg("plain")) != DeserializationError::Ok) {
            server.send(400, "application/json", "{\"error\":\"Invalid JSON.\"}");
            return;
        }
        String testSsid = doc["ssid"] | "";
        testSsid.trim();
        String testPass = doc["pass"] | "";
        if (testSsid.length() == 0) {
            server.send(400, "application/json", "{\"error\":\"SSID is required.\"}");
            return;
        }

        if (extConnTaskRunning.load() || gattBtnTaskRunning.load() || connState == STATE_CONNECTING) {
            server.send(409, "application/json", "{\"error\":\"BLE connection in progress, retry in a moment.\"}");
            return;
        }

        // Snapshot current STA connection so we can restore it if verification fails
        bool wasConnected = (WiFi.status() == WL_CONNECTED);
        String prevSsid = wasConnected ? WiFi.SSID() : "";
        String prevPass = "";
        if (prevSsid.length() > 0) {
            for (const auto& w : wifiList) {
                if (w.ssid.equalsIgnoreCase(prevSsid)) { prevPass = w.pass; break; }
            }
        }

        // If already connected to this exact network and password didn't change ("********"), succeed immediately
        if (wasConnected && prevSsid.equalsIgnoreCase(testSsid) && testPass == "********") {
            DynamicJsonDocument okDoc(256);
            okDoc["ok"] = true;
            okDoc["rssi"] = WiFi.RSSI();
            String out; serializeJson(okDoc, out);
            String _vJson = (out);
        if (_vJson.endsWith("}")) _vJson = _vJson.substring(0, _vJson.length() - 1) + ",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
        server.send(200, "application/json", _vJson);
            return;
        }

        wifiScanInProgress = true;
        NimBLEDevice::getScan()->stop();
        esp_coex_preference_set(ESP_COEX_PREFER_WIFI);

        // Keep AP active if currently in AP/AP_STA mode so local client session stays alive
        wifi_mode_t curMode = WiFi.getMode();
        if (curMode == WIFI_AP) {
            WiFi.mode(WIFI_AP_STA);
        }

        WiFi.disconnect(false);
        delay(80);
        if (testPass.length() > 0) {
            WiFi.begin(testSsid.c_str(), testPass.c_str());
        } else {
            WiFi.begin(testSsid.c_str());
        }

        unsigned long startMs = millis();
        bool verified = false;
        int verifiedRssi = 0;
        while (elapsedMs(millis(), startMs) < 9000UL) {
            wl_status_t st = WiFi.status();
            if (st == WL_CONNECTED) {
                verified = true;
                verifiedRssi = WiFi.RSSI();
                break;
            }
            if (st == WL_CONNECT_FAILED || st == WL_NO_SSID_AVAIL) {
                break;
            }
            delay(100);
        }

        if (!verified) {
            WiFi.disconnect(false);
            delay(50);
            if (prevSsid.length() > 0) {
                if (prevPass.length() > 0) WiFi.begin(prevSsid.c_str(), prevPass.c_str());
                else WiFi.begin(prevSsid.c_str());
            } else if (curMode == WIFI_AP) {
                WiFi.mode(WIFI_AP);
            }
        }

        wifiScanInProgress = false;
        WiFi.setSleep(WIFI_PS_MIN_MODEM);
        esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
        setCoexScanner();

        if (verified) {
            bool updatedExisting = false;
            for (auto& w : wifiList) {
                if (w.ssid.equalsIgnoreCase(testSsid)) {
                    if (testPass != "********") w.pass = testPass;
                    updatedExisting = true;
                    break;
                }
            }
            if (!updatedExisting) {
                if (wifiList.size() >= 5) wifiList.pop_back();
                wifiList.insert(wifiList.begin(), {testSsid, testPass == "********" ? String("") : testPass});
            }
            saveWifiListToNvs();
            LOGF("[WIFI] Verified & saved '%s' to NVS (IP: %s)\n", testSsid.c_str(), WiFi.localIP().toString().c_str());

            DynamicJsonDocument okDoc(256);
            okDoc["ok"] = true;
            okDoc["rssi"] = verifiedRssi;
            okDoc["ip"] = WiFi.localIP().toString();
            String out; serializeJson(okDoc, out);
            server.send(200, "application/json", out);
        } else {
            server.send(401, "application/json", "{\"error\":\"Could not authenticate with \'" + testSsid + "\'. Check password and try again.\"}");
        }
    });

    server.on("/api/wifi/scan", []() {
        if (!checkAuth(true)) return;
        if (extConnTaskRunning.load() || gattBtnTaskRunning.load() || connState == STATE_CONNECTING) {
            server.send(409, "application/json", "{\"error\":\"BLE connection in progress, retry in a moment\"}");
            return;
        }
        wifiScanInProgress = true;
        NimBLEDevice::getScan()->stop();
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.disconnect(false);
        }
        WiFi.setSleep(WIFI_PS_MIN_MODEM);
        delay(20);
        int n = WiFi.scanNetworks(true, true);
        if (n == -1) {
            delay(250);
            n = WiFi.scanNetworks(false, true);
        }
        unsigned long scanStart = millis();
        while (WiFi.scanComplete() == WIFI_SCAN_RUNNING && elapsedMs(millis(), scanStart) < 4000UL) {
            unsigned long tNow = millis();
            if (extConnTaskDone.exchange(false)) finishBleConnect(tNow);
            drainScanEvents(tNow);
            serviceGattButton(tNow);
            serviceButtonGestures(tNow);
            if (hoseClearActive && (elapsedMs(tNow, hoseClearTimer) >= (unsigned long)evacSeconds * 1000UL)) {
                LOGLN("[DEBRIS EVAC] Evacuation complete -> Motor OFF");
                hoseClearActive = false;
                forceStopNow = true;
                demandDirty = true;
            }
            if (demandDirty) { demandDirty = false; evaluateMotorDemand(); }
            delay(15);
        }
        if (n < 0) n = WiFi.scanComplete();
        struct ScanNet { String ssid; int rssi; };
        std::vector<ScanNet> uniqueNets;
        for (int i = 0; i < n; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;
            int rssi = WiFi.RSSI(i);
            bool found = false;
            for (auto& u : uniqueNets) {
                if (u.ssid == ssid) {
                    if (rssi > u.rssi) u.rssi = rssi;
                    found = true;
                    break;
                }
            }
            if (!found) uniqueNets.push_back({ssid, rssi});
        }
        StaticJsonDocument<1536> doc;
        JsonArray arr = doc.to<JsonArray>();
        for (const auto& u : uniqueNets) {
            JsonObject obj = arr.createNestedObject();
            obj["ssid"] = u.ssid;
            obj["rssi"] = u.rssi;
        }
        String out; serializeJson(doc, out);
        server.send(200, "application/json", out);
        WiFi.scanDelete();
        wifiScanInProgress = false;
        WiFi.setSleep(WIFI_PS_MIN_MODEM);
        setCoexScanner();
    });

    server.on("/api/control", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("power") && server.arg("power").toInt() != 1) {
            LOGLN("[WEB UI] Master OFF requested");
            webUiDemandActive = false; webUiSpeedOverride = 0; clearAllButtonToggles();
            toolBattDemandActive = false; activeToolBattMac = "";
            hoseClearActive = false; forceStopNow = true;
            for (auto& m : monitoredList) { m.toolActive = false; m.liveSpeedOverride = 0; }
            evaluateMotorDemand();
            sendJsonStatus();
            return;
        }
        if (pairedMac.length() == 0 || connState != STATE_CONNECTED) { sendJsonStatus(); return; }
        bool hasSpeedArg = server.hasArg("speed");
        if (hasSpeedArg) {
            userManualSpeed = quantizeSpeed((uint8_t)server.arg("speed").toInt());
            bool otherDemandActive = (primaryMomentaryActive || primaryToggleDemandActive || btnToggleDemandActive || toolBattDemandActive);
            if (!webUiDemandActive && otherDemandActive) {
                setActiveLiveSpeedOverride(userManualSpeed);
                evaluateMotorDemand();
                sendJsonStatus();
                return;
            }
            webUiSpeedOverride = 0;
            if (webUiDemandActive || currentPowerState == 1) {
                evaluateMotorDemand();
            }
        }
        if (server.hasArg("power") && server.arg("power").toInt() == 1) {
            LOGF("[WEB UI] Start requested -> %u%%\n", userManualSpeed);
            webUiDemandActive = true; hoseClearActive = false; webUiSpeedOverride = 0;
            evaluateMotorDemand();
        }
        sendJsonStatus();
    });

    server.on("/api/pair", []() {
        if (!checkAuth(true)) return;
        extUnpairPending.store(false);
        lastExtConnectFailMs = 0;
        targetConnectMac = server.arg("mac");
        targetConnectType = 255;
        requestConnect = true;
        sendJsonStatus();
    });

    server.on("/api/forget", []() {
        if (!checkAuth(true)) return;
        requestConnect = false;
        targetConnectMac = "";
        postConnectActionPending = false;
        lastExtConnectFailMs = 0;
        if (extConnTaskRunning.load()) {
            extUnpairPending.store(true);
        } else {
            extUnpairPending.store(false);
            if (pClient && pClient->isConnected()) pClient->disconnect();
        }
        setPairedMacState("", 0);
        pairedLastSeenMs = 0; pairedRssi = 0; lastAutoReconnectTryMs = 0; initialTelemDone = false;
        resetExtractorTelemetry();
        prefs.remove("paired_mac"); prefs.remove("paired_type");
        mqttTelemDirty = true;
        updateStatusLed(millis());
        if (!extConnTaskRunning.load()) setCoexScanner();
        sendJsonStatus();
    });

    server.on("/api/rename_ext", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("name")) {
            extFriendlyName = server.arg("name");
            prefs.putString("ext_name", extFriendlyName);
            mqttTelemDirty = true;
        }
        sendJsonStatus();
    });

    server.on("/api/monitored/rename", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("mac") && server.hasArg("name")) {
            String m = server.arg("mac"), n = server.arg("name");
            for (auto& i : monitoredList) if (i.address.equalsIgnoreCase(m)) { i.name = n; i.customNamed = true; break; }
            saveMonitoredToNvs();
        }
        sendJsonStatus();
    });

    server.on("/api/monitored/update", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("mac")) {
            String m = server.arg("mac");
            for (auto& i : monitoredList) {
                if (i.address.equalsIgnoreCase(m)) {
                    if (server.hasArg("speed")) {
                        uint8_t newSpd = quantizeSpeed((uint8_t)server.arg("speed").toInt());
                        i.speed = newSpd;
                        if (i.momActive) i.momSpeed = newSpd;
                        i.liveSpeedOverride = 0;
                        if (i.toggleActive || i.momActive || i.toolActive) {
                            syncButtonDemandsFromMonitored();
                            syncToolBattDemandFromMonitored(millis());
                            demandDirty = true;
                        }
                    }
                    if (server.hasArg("wattsThr")) {
                        int wt = server.arg("wattsThr").toInt();
                        if (wt < 5) wt = 5;
                        if (wt > 500) wt = 500;
                        i.wattsThr = (uint16_t)wt;
                    }
                    break;
                }
            }
            saveMonitoredToNvs();
        }
        sendJsonStatus();
    });

    server.on("/api/monitored/primary", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("mac")) {
            String m = server.arg("mac");
            bool nextPrimary = server.hasArg("primary") && (server.arg("primary").toInt() == 1);
            uint8_t aType = 255, bType = 1, spd = 20;
            bool shouldConnectGatt = false;
            bool shouldDisconnectGatt = false;
            for (const auto& b : bleList) {
                if (b.address.equalsIgnoreCase(m)) { aType = b.addrType; break; }
            }
            for (auto& i : monitoredList) {
                if (i.address.equalsIgnoreCase(m)) {
                    if (aType == 255 && (i.addrType == 0 || i.addrType == 1)) aType = i.addrType;
                    else if (aType == 0 || aType == 1) i.addrType = aType;
                    if (i.btnType > 0) {
                        if (nextPrimary && !i.canGatt) {
                            LOGF("[PRIMARY ROLE] %s does not support GATT Primary mode; ignoring.\n", m.c_str());
                            break;
                        }
                        if (!nextPrimary && !i.canAdv) {
                            LOGF("[PRIMARY ROLE] %s is GATT-only -> Primary remains locked ON.\n", m.c_str());
                            break;
                        }
                        i.isPrimary = nextPrimary;
                        i.waitReleaseAfterPair = false;
                        i.pairHoldGuardUntilMs = 0;
                        if (nextPrimary) {
                            i.isGatt = true;
                            if (i.canAdv) i.periodicBeacon = false;
                            i.activeRaw = "GATT:FFE0/HID";
                            i.gattReady = i.gattReady || isGattMacConnectedOrBusy(m);
                            shouldConnectGatt = !i.gattReady;
                            bType = i.btnType;
                            spd = i.speed;
                            if (i.gattReady) {
                                LOGF("[PRIMARY ROLE] %s -> Primary Switch ON (already paired & ready)\n", m.c_str());
                            } else {
                                LOGF("[PRIMARY ROLE] %s -> Primary Switch ON (press and hold button to pair GATT)\n", m.c_str());
                            }
                        } else {
                            if (i.canAdv) {
                                i.isGatt = false;
                                i.gattReady = true;
                                i.activeRaw = "ADV:STREAM";
                                shouldDisconnectGatt = true;
                                LOGF("[PRIMARY ROLE] %s -> Primary Switch OFF (returned to passive ADV)\n", m.c_str());
                            } else {
                                i.gattReady = i.gattReady || isGattMacConnectedOrBusy(m);
                                LOGF("[PRIMARY ROLE] %s -> Primary Switch OFF (GATT Tool Switch)\n", m.c_str());
                            }
                        }
                        resetMonitoredButtonGesture(i, true);
                    }
                    break;
                }
            }
            saveMonitoredToNvs();
            if (shouldDisconnectGatt) disconnectGattButtonMac(m);
            else if (shouldConnectGatt) enqueueGattConnect(m, aType, bType, spd, true);
            syncButtonDemandsFromMonitored();
            demandDirty = true;
        }
        sendJsonStatus();
    });

    server.on("/api/monitored/add", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("mac")) {
            String mac = server.arg("mac");
            String name = server.arg("name") ? server.arg("name") : "Device";
            int speed = server.hasArg("speed") ? server.arg("speed").toInt() : 20;
            uint8_t btnType = server.hasArg("btnType") ? server.arg("btnType").toInt() : 0;
            if (btnType > 0) btnType = 1;
            String raw = server.hasArg("raw") ? server.arg("raw") : "";
            int rssi = server.hasArg("rssi") ? server.arg("rssi").toInt() : 0;
            uint8_t addrType = server.hasArg("addrType") ? (uint8_t)server.arg("addrType").toInt() : 255;
            bool argCanAdv = server.hasArg("canAdv") && (server.arg("canAdv").toInt() == 1);
            bool argCanGatt = server.hasArg("canGatt") && (server.arg("canGatt").toInt() == 1);

            unsigned long now = millis();
            String initAdvName = "";
            uint32_t initMatNum = 0;
            uint8_t initDevCat = (btnType > 0) ? CAT_BUTTON : CAT_OTHER;
            bool bleCanAdv = false;
            bool bleCanGatt = false;
            bool bleGattConfirmed = false;
            for (const auto& b : bleList) {
                if (b.address.equalsIgnoreCase(mac)) {
                    addrType = b.addrType;
                    if (b.advName.length() > 0) initAdvName = b.advName;
                    if (b.matNum > 0) initMatNum = b.matNum;
                    bleCanAdv = b.canAdv;
                    bleCanGatt = b.canGatt;
                    bleGattConfirmed = (!b.canAdv && (b.rawData.startsWith("GATT") || b.slowCadenceHits >= 1));
                    if (b.type == "Battery") initDevCat = CAT_BATTERY;
                    else if (b.type == "Tool Plug") initDevCat = CAT_TOOL_PLUG;
                    else if (b.type == "Button") initDevCat = CAT_BUTTON;
                    break;
                }
            }
            String upName = name; upName.toUpperCase();
            String upAdvName = initAdvName; upAdvName.toUpperCase();
            bool isLegacyRemote = (raw == "ADV:FESTOOL_REMOTE" || initMatNum == 202097 || initMatNum == 204842 ||
                                   initMatNum == 10029918 || initMatNum == 10030046 ||
                                   upName.indexOf("CT-F") >= 0 || upName.indexOf("CTF") >= 0 ||
                                   upAdvName.indexOf("CT-F") >= 0 || upAdvName.indexOf("CTF") >= 0);
            if (isLegacyRemote) {
                initDevCat = CAT_BUTTON;
            }
            if (initAdvName.length() == 0 && name != "Device" && name != "Festool Battery" && name != "Shelly Tool Plug" && name != "Auto Tool") {
                initAdvName = name;
            }

            bool canAdvInit = false;
            bool canGattInit = false;
            bool isGattInit = false;
            if (btnType > 0) {
                canAdvInit = bleCanAdv || argCanAdv || isLegacyRemote || raw.startsWith("ADV");
                canGattInit = isLegacyRemote ? false : (bleCanGatt || argCanGatt || raw.startsWith("GATT"));
                if (canAdvInit) {
                    isGattInit = false;
                    raw = "ADV:STREAM";
                } else if (raw.startsWith("GATT") || bleGattConfirmed || (canGattInit && !canAdvInit)) {
                    isGattInit = true;
                    canGattInit = true;
                    raw = "GATT:FFE0/HID";
                }
            }
            bool alreadyConnGatt = isGattInit && isGattMacConnectedOrBusy(mac);

            if (findMonitoredByMac(mac) == nullptr) {
                bool gReady = (!isGattInit) || alreadyConnGatt;
                bool initPrimary = (isGattInit && !canAdvInit);
                uint8_t resolvedAddrType = inferRandomAddrType(mac, addrType);
                monitoredList.push_back({mac, name, initAdvName, (uint8_t)speed, btnType, raw, false, false, 0, isGattInit, gReady, false, 0, initPrimary, canAdvInit, initMatNum, initDevCat, 60, -1, canGattInit, resolvedAddrType});
            }
            saveMonitoredToNvs();
            if (isGattInit && !alreadyConnGatt) enqueueGattConnect(mac, inferRandomAddrType(mac, addrType), btnType, (uint8_t)speed, true);

            bool inBle = false;
            for (auto& b : bleList) {
                if (b.address.equalsIgnoreCase(mac)) {
                    if (rssi < 0) b.rssi = rssi;
                    if (addrType == 0 || addrType == 1) b.addrType = addrType;
                    if (canAdvInit) b.canAdv = true;
                    if (canGattInit) b.canGatt = true;
                    if (raw.length() > 0) b.rawData = raw;
                    inBle = true;
                    break;
                }
            }
            if (!inBle) {
                bleList.push_back({name, initAdvName, mac, addrType, rssi, true, devCatToTypeStr(initDevCat), now, -1, 0, initMatNum, -1, raw, canAdvInit, canGattInit, 0, 0, 0, 0});
            }
        }
        sendJsonStatus();
    });

    server.on("/api/monitored/delete", []() {
        if (!checkAuth(true)) return;
        if (server.hasArg("mac")) {
            String mac = server.arg("mac");
            disconnectGattButtonMac(mac);
            for (auto it = monitoredList.begin(); it != monitoredList.end(); ) {
                if (it->address.equalsIgnoreCase(mac)) it = monitoredList.erase(it);
                else ++it;
            }
            saveMonitoredToNvs();
            syncButtonDemandsFromMonitored();
            syncToolBattDemandFromMonitored(millis());
            if (!isAnyPrimaryDemandActive() && currentPowerState == 1 && !hoseClearActive) {
                if (evacSeconds > 0) { hoseClearActive = true; hoseClearTimer = millis(); }
                else forceStopNow = true;
            }
            demandDirty = true;
        }
        sendJsonStatus();
    });

    server.on("/update", HTTP_GET, []() { if (!checkAuth(false)) return; server.send_P(200, "text/html", OTA_HTML); });
    server.on("/update", HTTP_POST, []() {
        if (!checkAuth(true)) return;
        bool otaOk = !Update.hasError() && Update.isFinished();
        server.sendHeader("Connection", "close");
        if (otaOk) {
            server.send(200, "text/plain", "OK - Rebooting...");
            requestReboot = true;
            rebootTimer = millis();
        } else {
            String errStr = Update.errorString();
            if (errStr.length() == 0) errStr = "Update failed";
            server.send(500, "text/plain", "FAIL: " + errStr);
            setCoexScanner();
        }
    }, []() {
        HTTPUpload& upload = server.upload();
        if (!isSessionAuthenticated()) {
            if (upload.status == UPLOAD_FILE_START) {
                server.sendHeader("Connection", "close");
                server.send(401, "text/plain", "Unauthorized");
                server.client().stop();
            }
            return;
        }
        if (upload.status == UPLOAD_FILE_START) {
            NimBLEDevice::getScan()->stop();
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                LOGF("[OTA] Update.begin failed: %s\n", Update.errorString());
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.isRunning()) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    LOGF("[OTA] Update.write failed: %s\n", Update.errorString());
                    Update.abort();
                }
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.isRunning()) {
                if (!Update.end(true)) {
                    LOGF("[OTA] Update.end failed: %s\n", Update.errorString());
                }
            }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
            LOGLN("[OTA] Upload aborted by client; restoring BLE scanner.");
            if (Update.isRunning()) {
                Update.abort();
            }
            setCoexScanner();
        }
    });

    server.onNotFound([]() {
        wifi_mode_t wMode = WiFi.getMode();
        if (wMode == WIFI_AP || wMode == WIFI_AP_STA) {
            server.sendHeader("Location", "http://192.168.4.1/", true);
            server.send(302, "text/plain", "");
        } else {
            server.send(404, "text/plain", "Not Found");
        }
    });

    server.enableDelay(false);
    server.begin();
}
