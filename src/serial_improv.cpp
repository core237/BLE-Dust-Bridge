#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_coexist.h>
#include <ArduinoJson.h>
#include <algorithm>
#include "app_state.h"
#include "serial_improv.h"
#include "ble_manager.h"
#include "storage.h"
#include "web_server.h"

static uint8_t improvRxBuf[260];
static size_t improvRxLen = 0;
static size_t improvExpectedLen = 0;
static char serialLineBuf[384];
static size_t serialLineLen = 0;
bool improvProvPending = false;
static unsigned long improvProvStartMs = 0;
static String improvProvSsid = "";
static String improvProvPass = "";

static void improvSendPacket(uint8_t type, const uint8_t* data, uint8_t len) {
    uint8_t pkt[264];
    size_t idx = 0;
    pkt[idx++] = 'I'; pkt[idx++] = 'M'; pkt[idx++] = 'P';
    pkt[idx++] = 'R'; pkt[idx++] = 'O'; pkt[idx++] = 'V';
    pkt[idx++] = 0x01;
    pkt[idx++] = type;
    pkt[idx++] = len;
    if (len > 0 && data) {
        memcpy(pkt + idx, data, len);
        idx += len;
    }
    uint8_t csum = 0;
    for (size_t i = 0; i < idx; i++) csum = (uint8_t)(csum + pkt[i]);
    pkt[idx++] = csum;
    pkt[idx++] = 0x0A;

    Serial.setTxTimeoutMs(100);
    if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial.write(pkt, idx);
        Serial.flush();
        xSemaphoreGive(logMutex);
    } else {
        Serial.write(pkt, idx);
        Serial.flush();
    }
    Serial.setTxTimeoutMs(0);
}

static void improvSendState(uint8_t state) {
    improvSendPacket(0x01, &state, 1);
}

static void improvSendError(uint8_t err) {
    improvSendPacket(0x02, &err, 1);
}

static void improvSendRpcResult(uint8_t cmd, const std::vector<String>& strings) {
    uint8_t buf[240];
    size_t pos = 2;
    for (const auto& s : strings) {
        uint8_t sl = (uint8_t)(s.length() > 64 ? 64 : s.length());
        if (pos + 1 + sl >= sizeof(buf)) break;
        buf[pos++] = sl;
        memcpy(buf + pos, s.c_str(), sl);
        pos += sl;
    }
    buf[0] = cmd;
    buf[1] = (uint8_t)(pos - 2);
    improvSendPacket(0x04, buf, (uint8_t)pos);
}

static String getDeviceWebUrl() {
    if (WiFi.status() == WL_CONNECTED) {
        return "http://" + WiFi.localIP().toString() + "/";
    }
    return "http://192.168.4.1/";
}

static int performRadioPriorityWifiScan() {
    wifiScanInProgress = true;
    NimBLEDevice::getScan()->stop();
    esp_coex_preference_set(ESP_COEX_PREFER_WIFI);

    wifi_mode_t curMode = WiFi.getMode();
    if (curMode == WIFI_OFF) {
        WiFi.mode(WIFI_STA);
        delay(120);
    } else if (curMode == WIFI_AP) {
        WiFi.mode(WIFI_AP_STA);
        delay(120);
    }

    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    WiFi.scanDelete();
    delay(80);

    int n = WiFi.scanNetworks(true, true);
    unsigned long startMs = millis();
    while (WiFi.scanComplete() == WIFI_SCAN_RUNNING && elapsedMs(millis(), startMs) < 5000UL) {
        delay(25);
    }
    n = WiFi.scanComplete();
    if (n <= 0) {
        WiFi.scanDelete();
        delay(150);
        n = WiFi.scanNetworks(true, true);
        startMs = millis();
        while (WiFi.scanComplete() == WIFI_SCAN_RUNNING && elapsedMs(millis(), startMs) < 5000UL) {
            delay(25);
        }
        n = WiFi.scanComplete();
    }
    return n;
}

static void restoreCoexAfterWifiOp() {
    wifiScanInProgress = false;
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
    setCoexScanner();
}

static void handleImprovRpcCommand(const uint8_t* data, uint8_t len, unsigned long now) {
    if (len < 1) return;
    uint8_t cmd = data[0];

    if (cmd == 0x02) { // GET_CURRENT_STATE
        if (improvProvPending) {
            improvSendState(0x03);
        } else if (WiFi.status() == WL_CONNECTED) {
            improvSendState(0x04);
            improvSendRpcResult(0x02, { getDeviceWebUrl() });
        } else {
            improvSendState(0x02);
        }
    } else if (cmd == 0x03) { // GET_DEVICE_INFO
        improvSendRpcResult(0x03, { "BLE Dust Bridge", "1.0.0", "ESP32-S3", "BLE-Dust-Bridge" });
    } else if (cmd == 0x04) { // GET_WIFI_NETWORKS
        if (extConnTaskRunning.load() || gattBtnTaskRunning.load() || connState == STATE_CONNECTING) {
            improvSendRpcResult(0x04, {});
            return;
        }

        int n = performRadioPriorityWifiScan();

        struct ScanNet { String ssid; int rssi; bool enc; };
        std::vector<ScanNet> nets;
        for (int i = 0; i < n; i++) {
            String ssid = WiFi.SSID(i);
            if (ssid.length() == 0) continue;
            int rssi = WiFi.RSSI(i);
            bool enc = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
            bool found = false;
            for (auto& u : nets) {
                if (u.ssid == ssid) {
                    if (rssi > u.rssi) { u.rssi = rssi; u.enc = enc; }
                    found = true;
                    break;
                }
            }
            if (!found) nets.push_back({ssid, rssi, enc});
        }
        std::sort(nets.begin(), nets.end(), [](const ScanNet& a, const ScanNet& b) { return a.rssi > b.rssi; });

        for (const auto& u : nets) {
            improvSendRpcResult(0x04, { u.ssid, String(u.rssi), u.enc ? "YES" : "NO" });
            delay(25);
        }
        improvSendRpcResult(0x04, {});

        WiFi.scanDelete();
        restoreCoexAfterWifiOp();
        delay(30);
        LOGF("[WIFI_PROV_DBG] cmd=0x04 mode=%d scan_n=%d unique=%u\n", (int)WiFi.getMode(), n, (unsigned)nets.size());
    } else if (cmd == 0x01) { // SEND_WIFI_SETTINGS
        if (len < 4) {
            improvSendError(0x02);
            return;
        }
        uint8_t ssidLen = data[2];
        if (3 + ssidLen >= len) {
            improvSendError(0x02);
            return;
        }
        String ssid = "";
        for (uint8_t i = 0; i < ssidLen; i++) ssid += (char)data[3 + i];
        uint8_t passLen = data[3 + ssidLen];
        if (4 + ssidLen + passLen > len) {
            improvSendError(0x02);
            return;
        }
        String pass = "";
        for (uint8_t i = 0; i < passLen; i++) pass += (char)data[4 + ssidLen + i];
        ssid.trim();
        if (ssid.length() == 0) {
            improvSendError(0x02);
            return;
        }

        for (auto it = wifiList.begin(); it != wifiList.end(); ) {
            if (it->ssid.equalsIgnoreCase(ssid)) it = wifiList.erase(it);
            else ++it;
        }
        wifiList.insert(wifiList.begin(), {ssid, pass});
        if (wifiList.size() > 5) wifiList.resize(5);
        bgWifiIdx = 0;
        saveWifiListToNvs();

        improvProvSsid = ssid;
        improvProvPass = pass;
        improvProvPending = true;
        improvProvStartMs = now;

        improvSendState(0x03);

        NimBLEDevice::getScan()->stop();
        esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
        WiFi.setSleep(WIFI_PS_MIN_MODEM);
        if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_OFF) {
            WiFi.mode(WIFI_AP_STA);
            delay(120);
        }
        if (WiFi.status() == WL_CONNECTED) {
            WiFi.disconnect(false);
            delay(100);
        }
        WiFi.begin(ssid.c_str(), pass.c_str());
    }
}

static void printSerialStatusBanner() {
    if (improvProvPending) return;
    String ipStr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
    const char* modeStr = (WiFi.status() == WL_CONNECTED) ? "STA" : "AP (dustbridge_setup)";
    LOGF("[BLDB] Console Ready | Wi-Fi: %s (%s) | BLE MAC: %s | Extractor: %s\n",
         modeStr,
         ipStr.c_str(),
         NimBLEDevice::getAddress().toString().c_str(),
         pairedMac.length() > 0 ? pairedMac.c_str() : "UNPAIRED");
}

void serviceSerialAndImprov(unsigned long now) {
    static int lastApStations = -1;
    int curApStations = (int)WiFi.softAPgetStationNum();
    if (curApStations != lastApStations) {
        lastApStations = curApStations;
        LOGF("[AP_CLIENTS] %d\n", curApStations);
    }
    static bool prevSerialConnected = false;
    bool serialConnected = (bool)Serial;
    if (serialConnected && !prevSerialConnected) {
        printSerialStatusBanner();
    }
    prevSerialConnected = serialConnected;

    while (Serial.available() > 0) {
        uint8_t b = (uint8_t)Serial.read();
        if (improvRxLen == 0 && b != 'I') {
            if (b == '\r' || b == '\n') {
                if (serialLineLen > 0) {
                    serialLineBuf[serialLineLen] = '\0';
                    String rawLine(serialLineBuf);
                    String cmd = rawLine;
                    cmd.trim();
                    cmd.toLowerCase();
                    if (cmd == "status" || cmd == "help" || cmd == "info") {
                        printSerialStatusBanner();
                    } else if (cmd == "bldb_led_wifi 1") {
                        installerWifiScreenActive = true;
                        updateStatusLed(now);
                    } else if (cmd == "bldb_led_wifi 0") {
                        installerWifiScreenActive = false;
                        updateStatusLed(now);
                    } else if (cmd.startsWith("bldb_set_auth ")) {
                        String jsonPart = rawLine;
                        jsonPart.trim();
                        jsonPart = jsonPart.substring(14);
                        jsonPart.trim();
                        StaticJsonDocument<384> doc;
                        DeserializationError jErr = deserializeJson(doc, jsonPart);
                        if (jErr == DeserializationError::Ok) {
                            bool en = doc["en"].is<bool>() ? doc["en"].as<bool>() : ((doc["en"] | 0) != 0);
                            if (!en) {
                                auth_en = false;
                                clearAllAuthSessions();
                                prefs.putBool("auth_init", true);
                                prefs.putBool("auth_en", false);
                                LOGLN("[SERIAL_AUTH_OK] Web UI login disabled (None) via Web Installer.");
                            } else {
                                String u = doc["user"] | "";
                                String p = doc["pass"] | "";
                                u.trim();
                                if (u.length() > 0 && p.length() > 0) {
                                    auth_en = true;
                                    auth_user = u;
                                    auth_salt = generateRandomHex(16);
                                    auth_hash = sha256Hex(auth_salt + ":" + auth_user + ":" + p);
                                    clearAllAuthSessions();
                                    prefs.putBool("auth_init", true);
                                    prefs.putBool("auth_en", true);
                                    prefs.putString("auth_user", auth_user);
                                    prefs.putString("auth_salt", auth_salt);
                                    prefs.putString("auth_hash", auth_hash);
                                    LOGF("[SERIAL_AUTH_OK] Web UI login configured for '%s' via Web Installer.\n", auth_user.c_str());
                                } else {
                                    LOGLN("[SERIAL_AUTH_ERR] Missing username or password.");
                                }
                            }
                        } else {
                            LOGF("[SERIAL_AUTH_ERR] JSON parse failed: %s\n", jErr.c_str());
                        }
                    }
                    serialLineLen = 0;
                }
            } else if (b >= 32 && b < 127 && serialLineLen + 1 < sizeof(serialLineBuf)) {
                serialLineBuf[serialLineLen++] = (char)b;
            }
            continue;
        }

        const char magic[6] = {'I', 'M', 'P', 'R', 'O', 'V'};
        if (improvRxLen < 6) {
            if (b == (uint8_t)magic[improvRxLen]) {
                improvRxBuf[improvRxLen++] = b;
            } else {
                improvRxLen = 0;
            }
            continue;
        }

        improvRxBuf[improvRxLen++] = b;
        if (improvRxLen == 9) {
            improvExpectedLen = 10 + (size_t)improvRxBuf[8];
            if (improvExpectedLen > sizeof(improvRxBuf)) {
                improvRxLen = 0;
                improvExpectedLen = 0;
            }
        }
        if (improvExpectedLen > 0 && improvRxLen >= improvExpectedLen) {
            uint8_t csum = 0;
            for (size_t i = 0; i < improvExpectedLen - 1; i++) {
                csum = (uint8_t)(csum + improvRxBuf[i]);
            }
            if (csum == improvRxBuf[improvExpectedLen - 1] && improvRxBuf[6] == 0x01 && improvRxBuf[7] == 0x03) {
                handleImprovRpcCommand(improvRxBuf + 9, improvRxBuf[8], now);
            } else if (csum != improvRxBuf[improvExpectedLen - 1]) {
                improvSendError(0x01);
            }
            improvRxLen = 0;
            improvExpectedLen = 0;
        }
    }

    if (improvProvPending) {
        if (WiFi.status() == WL_CONNECTED) {
            improvProvPending = false;
            installerWifiScreenActive = false;
            wifiConfirmedSolidUntilMs = now + 1500UL;
            updateStatusLed(now);
            restoreCoexAfterWifiOp();
            improvSendState(0x04);
            improvSendRpcResult(0x01, { getDeviceWebUrl() });
            delay(50);
            LOGF("[WIFI_PROV] Wi-Fi connected! URL: %s\n", getDeviceWebUrl().c_str());
        } else if (elapsedMs(now, improvProvStartMs) >= 25000UL) {
            improvProvPending = false;
            restoreCoexAfterWifiOp();
            improvSendError(0x03);
            improvSendState(0x02);
            delay(50);
            LOGLN("[WIFI_PROV] Wi-Fi connection timed out.");
        }
    }
}
