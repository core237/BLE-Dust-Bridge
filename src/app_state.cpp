#include <WiFi.h>
#include <mbedtls/sha256.h>
#include <esp_random.h>
#include "app_state.h"
#include "serial_improv.h"

const char* hostname = "dustbridge";
NimBLEUUID DUST_SVC_UUID("0000fe39-0000-1000-8000-00805f9b34fb");
NimBLEUUID CHAR_WRITE_UUID("3734fe03-c847-11e6-9d9d-cec0c932ce01");
NimBLEUUID CHAR_NOTIFY_UUID("3734fe02-c847-11e6-9d9d-cec0c932ce01");

SemaphoreHandle_t ledMutex = nullptr;
SemaphoreHandle_t logMutex = nullptr;
volatile bool ledBootMode = true;
volatile bool installerWifiScreenActive = false;
unsigned long wifiConfirmedSolidUntilMs = 0;

WebServer server(80);
Preferences prefs;

std::vector<SavedWiFi> wifiList;
size_t bgWifiIdx = 0;

bool mqtt_en = false;
String mqtt_ip = "";
int mqtt_port = 1883;
String mqtt_user = "";
String mqtt_pass = "";
String mqttClientId = "BLDB_000000";

bool auth_en = (DEFAULT_AUTH_EN != 0);
String auth_user = DEFAULT_AUTH_USER;
String auth_salt = "";
String auth_hash = "";
String bootAuthSecret = "";
unsigned long lastFailedLoginMs = 0;
char usedLoginNonces[MAX_USED_NONCES][68] = {};
size_t usedLoginNonceNext = 0;
AuthSession authSessions[MAX_AUTH_SESSIONS] = {};

WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastMqttReconnect = 0;
unsigned long lastMqttHeartbeatMs = 0;
volatile bool mqttTelemDirty = false;
bool mqttDevicesDirty = false;

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pWriteChar  = nullptr;
NimBLERemoteCharacteristic* pNotifyChar = nullptr;

char pairedMacBuf[18] = {0};
portMUX_TYPE pairedMux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<bool> pairedAdvSeenFlag{false};
std::atomic<bool> extLinkConnectedLog{false};
std::atomic<bool> extLinkDisconnectedFlag{false};

volatile BleConnState connState = STATE_DISCONNECTED;
volatile bool wifiScanInProgress = false;

String pairedMac = "";
volatile uint8_t pairedAddrType = 0;
volatile int pairedRssi = 0;
volatile unsigned long pairedLastSeenMs = 0;
unsigned long lastAutoReconnectTryMs = 0;
unsigned long lastExtConnectFailMs = 0;
unsigned long scannerStartedMs = 0;
volatile unsigned long lastBleActivityMs = 0;
String targetConnectMac = "";
uint8_t targetConnectType = 0;
String extFriendlyName = "Festool Extractor";
uint8_t evacSeconds = 4;
volatile bool requestConnect = false;
bool requestReboot = false;
unsigned long rebootTimer = 0;

bool postConnectActionPending = false;
unsigned long postConnectActionAtMs = 0;

std::atomic<bool> extConnTaskRunning{false};
std::atomic<bool> extConnTaskDone{false};
std::atomic<bool> extConnTaskSuccess{false};
std::atomic<bool> extUnpairPending{false};
std::atomic<bool> gattBtnTaskRunning{false};

ExtractorTelemetry extTelem = {
    0, 0, 0, true, 0, 0, "Healthy", "CT MINI MIDI I", 2, 10, "--", "--"
};
portMUX_TYPE telemMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool telemPollRequested = false;
volatile bool telemAwaitingReply = false;
volatile bool initialTelemDone = false;
uint8_t telemPollStep = 0;
unsigned long connectedAtMs = 0;
unsigned long lastControlTxMs = 0;
unsigned long lastCtCtrlTxMs = 0;
unsigned long lastTelemStepMs = 0;

bool identifyActive = false;
unsigned long identifyStartMs = 0;

std::vector<MonitoredDevice> monitoredList;
bool needSaveMonitored = false;
unsigned long needSaveMonitoredAtMs = 0;

char monMacCache[MAX_MON_MAC_CACHE][18];
volatile uint8_t monMacCacheCount = 0;
portMUX_TYPE monMacMux = portMUX_INITIALIZER_UNLOCKED;

std::vector<BleEntry> bleList;
GattBtnSlot gattSlots[MAX_GATT_BTN_SLOTS];
portMUX_TYPE gattSlotMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t currentPowerState = 3;
volatile uint8_t currentSuctionPct = 60;
uint8_t activePresetSpeed = 60;
uint8_t webUiSpeedOverride = 0;

volatile bool mqttVacStateDirty = false;
volatile uint8_t mqttVacStateVal = 3;
volatile bool mqttVacSpeedDirty = false;
volatile uint8_t mqttVacSpeedVal = 60;
volatile bool externalShutoffPending = false;

bool demandDirty = false;
bool forceStopNow = false;
bool hoseClearActive = false;
unsigned long hoseClearTimer = 0;

bool primaryMomentaryActive = false;
String activePrimaryMomMac = "";
uint8_t primaryMomTargetSpeed = 100;

bool primaryToggleDemandActive = false;
String activePrimaryToggleMac = "";
uint8_t primaryToggleTargetSpeed = 100;

bool btnToggleDemandActive = false;
String activeBtnToggleMac = "";
uint8_t btnToggleTargetSpeed = 60;

bool toolBattDemandActive = false;
String activeToolBattMac = "";
uint8_t toolBattTargetSpeed = 60;

bool webUiDemandActive = false;
uint8_t userManualSpeed = 60;

void setAtomLed(uint8_t r, uint8_t g, uint8_t b) {
    static int lastR = -1, lastG = -1, lastB = -1;
    if (ledMutex && xSemaphoreTake(ledMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    if ((int)r != lastR || (int)g != lastG || (int)b != lastB) {
        lastR = r; lastG = g; lastB = b;
        neopixelWrite(RGB_LED_GPIO, r, g, b);
    }
    if (ledMutex) xSemaphoreGive(ledMutex);
}

void updateStatusLed(unsigned long now) {
    if (ledBootMode) return;
    wifi_mode_t wMode = WiFi.getMode();
    bool blinkOn = ((now / 500UL) % 2) == 0;

    // 1. Solid Yellow confirmation window right after Wi-Fi credentials succeed
    if (wifiConfirmedSolidUntilMs > 0) {
        if (now < wifiConfirmedSolidUntilMs) {
            setAtomLed(96, 72, 0);
            return;
        }
        wifiConfirmedSolidUntilMs = 0;
    }

    // 2. Blinking Yellow while on the Web Installer Wi-Fi page or actively provisioning Wi-Fi
    if (installerWifiScreenActive || improvProvPending) {
        setAtomLed(blinkOn ? 96 : 0, blinkOn ? 72 : 0, 0);
        return;
    }

    // 3. Connected to Dust Extractor (normal running ops)
    if (connState == STATE_CONNECTED) {
        if (initialTelemDone) {
            setAtomLed(0, 96, 0);   // Solid Green: Normal running ops
        } else {
            setAtomLed(0, 0, 96);   // Solid Blue: Link settling
        }
        return;
    }

    // 4. Hotspot fallback mode (user skipped Wi-Fi or no STA connection yet): Alternating Yellow / Green
    if (wMode == WIFI_AP || (wMode == WIFI_AP_STA && WiFi.status() != WL_CONNECTED)) {
        if (blinkOn) {
            setAtomLed(96, 72, 0);  // Yellow
        } else {
            setAtomLed(0, 96, 0);   // Green
        }
        return;
    }

    // 5. Connected to LAN Wi-Fi (STA), ready to configure/pair Dust Extractor: Blinking Green
    if (!isExtractorPaired()) {
        setAtomLed(0, blinkOn ? 96 : 0, 0);
        return;
    }

    // 6. Connected to LAN Wi-Fi (STA), searching/reconnecting to paired extractor: Blinking Blue
    setAtomLed(0, 0, blinkOn ? 96 : 0);
}

String generateRandomHex(size_t byteLen) {
    if (byteLen > 32) byteLen = 32;
    uint8_t buf[32];
    esp_fill_random(buf, byteLen);
    char hex[65];
    for (size_t i = 0; i < byteLen; i++) {
        snprintf(hex + (i * 2), 3, "%02x", buf[i]);
    }
    hex[byteLen * 2] = '\0';
    return String(hex);
}

String sha256Hex(const String& input) {
    uint8_t digest[32];
    mbedtls_sha256((const unsigned char*)input.c_str(), input.length(), digest, 0);
    char hex[65];
    for (size_t i = 0; i < 32; i++) {
        snprintf(hex + (i * 2), 3, "%02x", digest[i]);
    }
    hex[64] = '\0';
    return String(hex);
}

bool constantTimeHexEquals(const String& a, const String& b) {
    if (a.length() == 0 || a.length() != b.length()) return false;
    uint8_t diff = 0;
    for (size_t i = 0; i < a.length(); i++) {
        char ca = a[i];
        char cb = b[i];
        if (ca >= 'A' && ca <= 'F') ca = (char)(ca + ('a' - 'A'));
        if (cb >= 'A' && cb <= 'F') cb = (char)(cb + ('a' - 'A'));
        diff |= (uint8_t)(ca ^ cb);
    }
    return diff == 0;
}

void resetExtractorTelemetry() {
    portENTER_CRITICAL(&telemMux);
    extTelem.motorRunSec = 0; extTelem.operatingCycles = 0; extTelem.filterRunSec = 0;
    extTelem.waterLevelActive = true; extTelem.temp1F = 0; extTelem.temp2F = 0;
    strncpy(extTelem.faultText, "Healthy", sizeof(extTelem.faultText));
    strncpy(extTelem.modelName, "CT MINI MIDI I", sizeof(extTelem.modelName));
    extTelem.mainSwMajor = 2; extTelem.bleSwMajor = 10;
    strncpy(extTelem.mainFw, "--", sizeof(extTelem.mainFw));
    strncpy(extTelem.bleFw, "--", sizeof(extTelem.bleFw));
    portEXIT_CRITICAL(&telemMux);
}

uint8_t inferRandomAddrType(const String& macStr, uint8_t currentType) {
    if (currentType == 0 || currentType == 1) return currentType;
    if (macStr.length() >= 2) {
        uint8_t firstByte = (uint8_t)strtoul(macStr.substring(0, 2).c_str(), nullptr, 16);
        if ((firstByte & 0xC0) == 0xC0) return 1;
    }
    return 0;
}

void setPairedMacState(const String& mac, uint8_t addrType) {
    pairedMac = mac; pairedAddrType = addrType;
    portENTER_CRITICAL(&pairedMux);
    strncpy(pairedMacBuf, mac.c_str(), sizeof(pairedMacBuf) - 1);
    pairedMacBuf[sizeof(pairedMacBuf) - 1] = '\0';
    portEXIT_CRITICAL(&pairedMux);
}

bool isExtractorPaired() {
    portENTER_CRITICAL(&pairedMux);
    bool has = (pairedMacBuf[0] != '\0');
    portEXIT_CRITICAL(&pairedMux);
    return has;
}

void refreshMonMacCache() {
    portENTER_CRITICAL(&monMacMux);
    uint8_t cnt = 0;
    for (size_t i = 0; i < monitoredList.size() && cnt < MAX_MON_MAC_CACHE; i++) {
        strncpy(monMacCache[cnt], monitoredList[i].address.c_str(), 17);
        monMacCache[cnt][17] = '\0';
        cnt++;
    }
    monMacCacheCount = cnt;
    portEXIT_CRITICAL(&monMacMux);
}

String lookupFestoolModelByTnr(uint32_t tnr) {
    if (tnr == 0) return "";
    switch (tnr) {
        case 202479: return "BP 18 Li 5.2 ASI (202479)";
        case 202480: return "BP 18 Li 6.2 ASI (202480)";
        case 205034: return "BP 18 Li 4.0 HPC-ASI (205034)";
        case 205035: return "BP 18 Li 5.2 ASI (205035)";
        case 205036: return "BP 18 Li 3.1 Ergo-I (205036)";
        case 576933: return "BP 18 Li 4.0 HPC-ASI (576933)";
        case 577153: return "BP 18 Li 3.0 ERGO-I (577153)";
        case 577240: return "BP 18 Li 5.0 ASI (577240)";
        case 577658: return "BP 18 Li 4.0 HPC-ASI (577658)";
        case 577659: return "BP 18 Li 5.0 ASI (577659)";
        case 577660: return "BP 18 Li 8.0 HP-ASI (577660)";
        case 202097: return "CT-F I Remote (202097)";
        case 204842: return "CT-F I/M-Set (204842)";
        case 10029918: return "CT-F I Remote (10029918)";
        case 10030046: return "CT-F I Remote (10030046)";
        case 574830: return "CT SYS I (574830)";
        case 574832: return "CT MINI I (574832)";
        case 574833: return "CT MIDI I (574833)";
        case 576936: return "CTLC MINI I (576936)";
        case 576944: return "CTLC MIDI I (576944)";
        case 577065: return "CTMC MIDI I (577065)";
        case 575267: return "CTL 26 E (575267)";
        case 575279: return "CTL 36 E (575279)";
        case 574947: return "CT 26 E (574947)";
        case 574955: return "CT 36 E (574955)";
        case 574963: return "CT 48 E (574963)";
        default:     return "Festool " + String(tnr);
    }
}

const char* devCatToTypeStr(uint8_t cat) {
    switch (cat) {
        case CAT_EXTRACTOR: return "Extractor";
        case CAT_BATTERY:   return "Battery";
        case CAT_FESTOOL:   return "Festool";
        case CAT_TOOL_PLUG: return "Tool Plug";
        case CAT_BUTTON:    return "Button";
        default:            return "Other";
    }
}

MonitoredDevice* findMonitoredByMac(const String& mac) {
    for (auto& m : monitoredList) {
        if (m.address.equalsIgnoreCase(mac)) return &m;
    }
    return nullptr;
}

void pruneStaleBleEntries(unsigned long now) {
    for (auto it = bleList.begin(); it != bleList.end(); ) {
        bool isPinned = (pairedMac.length() > 0 && it->address.equalsIgnoreCase(pairedMac));
        bool isMon = (findMonitoredByMac(it->address) != nullptr);
        if (!isPinned && !isMon && elapsedMs(now, it->lastSeen) > 300000UL) {
            it = bleList.erase(it);
        } else {
            ++it;
        }
    }
}

uint8_t quantizeSpeed(uint8_t pct) {
    if (pct <= 20) return 20;
    if (pct <= 40) return 40;
    if (pct <= 60) return 60;
    if (pct <= 80) return 80;
    return 100;
}
