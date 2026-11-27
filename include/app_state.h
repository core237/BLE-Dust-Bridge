#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <PubSubClient.h>
#include <atomic>
#include <vector>
#include "config.h"

// ============================================================================
// Structs & Enums
// ============================================================================
struct SavedWiFi {
    String ssid;
    String pass;
};

struct AuthSession {
    char token[33];
    unsigned long lastSeenMs;
    bool active;
};

enum BleConnState {
    STATE_DISCONNECTED,
    STATE_CONNECTING,
    STATE_CONNECTED
};

struct ExtractorTelemetry {
    uint32_t motorRunSec;
    uint32_t operatingCycles;
    uint32_t filterRunSec;
    bool waterLevelActive;
    int temp1F;
    int temp2F;
    char faultText[32];
    char modelName[32];
    uint8_t mainSwMajor;
    uint8_t bleSwMajor;
    char mainFw[20];
    char bleFw[20];
};

enum BtnNormPhase {
    BTN_IDLE = 0,
    BTN_HELD,
    BTN_WAIT_TAP
};

struct MonitoredDevice {
    String address; String name; String advName; uint8_t speed; uint8_t btnType; String activeRaw;
    bool customNamed; bool toolActive; unsigned long toolLastSeen;
    bool isGatt; bool gattReady;
    bool toggleActive; unsigned long toggleActivatedMs;
    bool isPrimary; bool canAdv; bool canGatt;
    uint32_t matNum; uint8_t devCat;
    uint16_t wattsThr; int32_t lastWatts;
    uint8_t offConfirmHits = 0;
    int8_t lastShellySeq = -1;
    BtnNormPhase btnPhase = BTN_IDLE;
    bool primaryHoldActivated = false;
    int16_t btnLastSeqByte = -1;
    unsigned long btnPressStartMs = 0;
    unsigned long btnLastPacketMs = 0;
    unsigned long btnReleaseMs = 0;
    uint8_t btnTapCount = 0;
    bool momActive = false;
    uint8_t momSpeed = 20;
    uint8_t liveSpeedOverride = 0;
    uint8_t addrType = 255;
    bool waitReleaseAfterPair = false;
    unsigned long pairHoldGuardUntilMs = 0;
    bool wakePreTriggered = false;
    bool periodicBeacon = false;

    MonitoredDevice() = default;
    MonitoredDevice(const String& addr, const String& nm, const String& advNm, uint8_t spd, uint8_t bt,
                    const String& raw, bool cn, bool tAct, unsigned long tSeen, bool gatt, bool gReady,
                    bool togAct, unsigned long togMs, bool hose, bool cAdv,
                    uint32_t mat, uint8_t cat, uint16_t wThr, int32_t lWatts,
                    bool cGatt = false, uint8_t aType = 255, bool pBcn = false)
        : address(addr), name(nm), advName(advNm), speed(spd), btnType(bt), activeRaw(raw),
          customNamed(cn), toolActive(tAct), toolLastSeen(tSeen), isGatt(gatt), gattReady(gReady),
          toggleActive(togAct), toggleActivatedMs(togMs), isPrimary(hose),
          canAdv(cAdv), canGatt(cGatt || gatt), matNum(mat), devCat(cat), wattsThr(wThr), lastWatts(lWatts),
          addrType(aType), periodicBeacon(pBcn) {}
};

struct BleEntry {
    String name; String advName; String address; uint8_t addrType; int rssi; bool isKnownDevice; String type;
    unsigned long lastSeen; int soc; int state; uint32_t matNum; int32_t watts; String rawData;
    bool canAdv;
    bool canGatt;
    unsigned long lastPrimaryAdvMs;
    uint8_t fastCadenceHits;
    uint8_t slowCadenceHits;
    uint8_t fastWakeHits;
};

enum ScanDevCat : uint8_t {
    CAT_OTHER = 0,
    CAT_EXTRACTOR,
    CAT_BATTERY,
    CAT_FESTOOL,
    CAT_TOOL_PLUG,
    CAT_BUTTON
};

struct ScanEvent {
    char mac[18]; char name[36]; char raw[96];
    uint8_t addrType; int8_t rssi; uint8_t advType; int8_t soc; int8_t state;
    uint32_t matNum; int32_t watts;
    bool isKnown; uint8_t devCat; bool isGattBtnAdv; bool isBurstBtnAdv; bool isShellyPlug;
    bool isFestoolRemote; bool isConnectable; bool hasFlags06;
    int16_t btnSeq;
    unsigned long ts;
};

struct GattBtnSlot {
    NimBLEClient* client = nullptr;
    bool inUse = false;
    volatile bool connected = false;
    bool connecting = false;
    volatile uint32_t connGen = 0;
    char mac[18] = {0};
    volatile bool pressed = false;
    volatile uint8_t pendingPressEdges = 0;
    volatile bool gotAnyNotify = false;
    volatile bool releaseNow = false;
    bool wakePreTriggered = false;
    volatile unsigned long lastActivityMs = 0;
    volatile unsigned long lastPressEdgeMs = 0;
    unsigned long connectedAtMs = 0;
    unsigned long lastRssiMs = 0;
    char lastHex[48] = {0};
    volatile bool rxLogPending = false;
    volatile bool rxLogPressed = false;
    volatile bool sleepDiscLogPending = false;
};

// ============================================================================
// Extern Global State
// ============================================================================
extern SemaphoreHandle_t ledMutex;
extern SemaphoreHandle_t logMutex;
extern volatile bool ledBootMode;
extern volatile bool installerWifiScreenActive;
extern unsigned long wifiConfirmedSolidUntilMs;

extern WebServer server;
extern Preferences prefs;

extern std::vector<SavedWiFi> wifiList;
extern size_t bgWifiIdx;

extern bool mqtt_en;
extern String mqtt_ip;
extern int mqtt_port;
extern String mqtt_user;
extern String mqtt_pass;
extern String mqttClientId;

extern bool auth_en;
extern String auth_user;
extern String auth_salt;
extern String auth_hash;
extern String bootAuthSecret;
extern unsigned long lastFailedLoginMs;
extern char usedLoginNonces[MAX_USED_NONCES][68];
extern size_t usedLoginNonceNext;
extern AuthSession authSessions[MAX_AUTH_SESSIONS];

extern WiFiClient espClient;
extern PubSubClient mqttClient;
extern unsigned long lastMqttReconnect;
extern unsigned long lastMqttHeartbeatMs;
extern volatile bool mqttTelemDirty;
extern bool mqttDevicesDirty;

extern NimBLEClient* pClient;
extern NimBLERemoteCharacteristic* pWriteChar;
extern NimBLERemoteCharacteristic* pNotifyChar;

extern char pairedMacBuf[18];
extern portMUX_TYPE pairedMux;
extern std::atomic<bool> pairedAdvSeenFlag;
extern std::atomic<bool> extLinkConnectedLog;
extern std::atomic<bool> extLinkDisconnectedFlag;

extern volatile BleConnState connState;
extern volatile bool wifiScanInProgress;

extern String pairedMac;
extern volatile uint8_t pairedAddrType;
extern volatile int pairedRssi;
extern volatile unsigned long pairedLastSeenMs;
extern unsigned long lastAutoReconnectTryMs;
extern unsigned long lastExtConnectFailMs;
extern unsigned long scannerStartedMs;
extern volatile unsigned long lastBleActivityMs;
extern String targetConnectMac;
extern uint8_t targetConnectType;
extern String extFriendlyName;
extern uint8_t evacSeconds;
extern volatile bool requestConnect;
extern bool requestReboot;
extern unsigned long rebootTimer;

extern bool postConnectActionPending;
extern unsigned long postConnectActionAtMs;

extern std::atomic<bool> extConnTaskRunning;
extern std::atomic<bool> extConnTaskDone;
extern std::atomic<bool> extConnTaskSuccess;
extern std::atomic<bool> extUnpairPending;
extern std::atomic<bool> gattBtnTaskRunning;

extern ExtractorTelemetry extTelem;
extern portMUX_TYPE telemMux;

extern volatile bool telemPollRequested;
extern volatile bool telemAwaitingReply;
extern volatile bool initialTelemDone;
extern uint8_t telemPollStep;
extern unsigned long connectedAtMs;
extern unsigned long lastControlTxMs;
extern unsigned long lastCtCtrlTxMs;
extern unsigned long lastTelemStepMs;

extern bool identifyActive;
extern unsigned long identifyStartMs;

extern std::vector<MonitoredDevice> monitoredList;
extern bool needSaveMonitored;
extern unsigned long needSaveMonitoredAtMs;

extern char monMacCache[MAX_MON_MAC_CACHE][18];
extern volatile uint8_t monMacCacheCount;
extern portMUX_TYPE monMacMux;

extern std::vector<BleEntry> bleList;
extern GattBtnSlot gattSlots[MAX_GATT_BTN_SLOTS];
extern portMUX_TYPE gattSlotMux;

extern volatile uint8_t currentPowerState;
extern volatile uint8_t currentSuctionPct;
extern uint8_t activePresetSpeed;
extern uint8_t webUiSpeedOverride;

extern volatile bool mqttVacStateDirty;
extern volatile uint8_t mqttVacStateVal;
extern volatile bool mqttVacSpeedDirty;
extern volatile uint8_t mqttVacSpeedVal;
extern volatile bool externalShutoffPending;

extern bool demandDirty;
extern bool forceStopNow;
extern bool hoseClearActive;
extern unsigned long hoseClearTimer;

extern bool primaryMomentaryActive;
extern String activePrimaryMomMac;
extern uint8_t primaryMomTargetSpeed;

extern bool primaryToggleDemandActive;
extern String activePrimaryToggleMac;
extern uint8_t primaryToggleTargetSpeed;

extern bool btnToggleDemandActive;
extern String activeBtnToggleMac;
extern uint8_t btnToggleTargetSpeed;

extern bool toolBattDemandActive;
extern String activeToolBattMac;
extern uint8_t toolBattTargetSpeed;

extern bool webUiDemandActive;
extern uint8_t userManualSpeed;

// ============================================================================
// Inline Utilities & Logging Macros
// ============================================================================
inline unsigned long elapsedMs(unsigned long now, unsigned long past) {
    return (now >= past) ? (now - past) : 0UL;
}

inline void logWriteAtomic(const char* buf, int len) {
    if (len <= 0) return;
    size_t n = (len >= 256) ? 255 : (size_t)len;
    if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        Serial.write((const uint8_t*)buf, n);
        xSemaphoreGive(logMutex);
    }
}

#define LOGF(fmt, ...) do { unsigned long _ms = millis(); char _logBuf[256]; int _n = snprintf(_logBuf, sizeof(_logBuf), "[%4lu.%03lu] " fmt, _ms / 1000UL, _ms % 1000UL, ##__VA_ARGS__); logWriteAtomic(_logBuf, _n); } while (0)
#define LOGLN(msg)     do { unsigned long _ms = millis(); char _logBuf[256]; int _n = snprintf(_logBuf, sizeof(_logBuf), "[%4lu.%03lu] %s\n", _ms / 1000UL, _ms % 1000UL, msg); logWriteAtomic(_logBuf, _n); } while (0)

inline bool isMonitoredConfiguring(const MonitoredDevice& m, unsigned long now = 0) {
    (void)now;
    return (m.isGatt && !m.gattReady);
}

// ============================================================================
// Shared Helper Functions
// ============================================================================
void setAtomLed(uint8_t r, uint8_t g, uint8_t b);
void updateStatusLed(unsigned long now);
String generateRandomHex(size_t byteLen);
String sha256Hex(const String& input);
bool constantTimeHexEquals(const String& a, const String& b);
void resetExtractorTelemetry();
uint8_t inferRandomAddrType(const String& macStr, uint8_t currentType);
void setPairedMacState(const String& mac, uint8_t addrType);
bool isExtractorPaired();
void refreshMonMacCache();
String lookupFestoolModelByTnr(uint32_t tnr);
const char* devCatToTypeStr(uint8_t cat);
MonitoredDevice* findMonitoredByMac(const String& mac);
void pruneStaleBleEntries(unsigned long now);
uint8_t quantizeSpeed(uint8_t pct);
