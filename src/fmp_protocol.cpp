#include "app_state.h"
#include "fmp_protocol.h"
#include "motor_control.h"
#include "mqtt_manager.h"

static uint8_t fmpReqId = 1;
static uint8_t rxCobsBuf[256];
static size_t rxCobsLen = 0;

void resetRxCobsBuffer() {
    rxCobsLen = 0;
}

static uint16_t fmpCrc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) crc = (crc & 0x0001) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

static size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst) {
    size_t readIdx = 0, writeIdx = 1, codeIdx = 0;
    uint8_t code = 1;
    while (readIdx < len) {
        if (src[readIdx] == 0x00) {
            dst[codeIdx] = code; code = 1; codeIdx = writeIdx++; readIdx++;
        } else {
            dst[writeIdx++] = src[readIdx++]; code++;
            if (code == 0xFF) { dst[codeIdx] = code; code = 1; codeIdx = writeIdx++; }
        }
    }
    dst[codeIdx] = code; dst[writeIdx++] = 0x00;
    return writeIdx;
}

static size_t cobsDecode(const uint8_t* src, size_t len, uint8_t* dst) {
    size_t readIdx = 0, writeIdx = 0;
    while (readIdx < len) {
        uint8_t code = src[readIdx++];
        if (code == 0) break;
        for (uint8_t i = 1; i < code; i++) {
            if (readIdx >= len) return 0;
            dst[writeIdx++] = src[readIdx++];
        }
        if (code < 0xFF && readIdx < len) dst[writeIdx++] = 0x00;
    }
    return writeIdx;
}

static bool pbReadVarint(const uint8_t* buf, size_t len, size_t& pos, uint32_t& val) {
    val = 0; uint8_t shift = 0;
    while (pos < len) {
        uint8_t b = buf[pos++];
        val |= (uint32_t)(b & 0x7F) << shift;
        if (!(b & 0x80)) return true;
        shift += 7;
        if (shift > 28) return false;
    }
    return false;
}

static bool pbSkipField(const uint8_t* buf, size_t len, size_t& pos, uint8_t wt) {
    if (wt == 0) { uint32_t dummy; return pbReadVarint(buf, len, pos, dummy); }
    if (wt == 1) { if (pos + 8 > len) return false; pos += 8; return true; }
    if (wt == 2) { uint32_t l; if (!pbReadVarint(buf, len, pos, l) || pos + l > len) return false; pos += l; return true; }
    if (wt == 5) { if (pos + 4 > len) return false; pos += 4; return true; }
    return false;
}

static void parseCtControlFields(const uint8_t* buf, size_t start, size_t end, const char* tagLabel) {
    unsigned long now = millis();
    bool isTelem = (strcmp(tagLabel, "FMP TELEM") == 0);
    bool isEvent = (strcmp(tagLabel, "FMP EVENT") == 0);
    bool recentCtrl = (lastCtCtrlTxMs > 0 && elapsedMs(now, lastCtCtrlTxMs) < 800UL);
    size_t p = start;
    while (p < end) {
        uint32_t tag;
        if (!pbReadVarint(buf, end, p, tag)) break;
        uint32_t field = tag >> 3; uint8_t wt = tag & 0x07;
        if (field == 1 && wt == 0) {
            uint32_t pwr;
            if (pbReadVarint(buf, end, p, pwr) && (pwr == 1 || pwr == 2 || pwr == 3)) {
                if (isTelem && recentCtrl) {
                    LOGF("[%s] Ignored stale Vacuum Power = %s (%u)\n", tagLabel, (pwr == 1) ? "ON" : "OFF", (unsigned)pwr);
                } else {
                    currentPowerState = (uint8_t)pwr;
                    mqttVacStateVal = (uint8_t)pwr;
                    mqttVacStateDirty = true;
                    LOGF("[%s] Vacuum Power = %s (%u)\n", tagLabel, (pwr == 1) ? "ON" : "OFF", (unsigned)pwr);
                    if (isEvent && pwr != 1 && !recentCtrl) {
                        externalShutoffPending = true;
                    }
                }
            }
        } else if (field == 4 && wt == 2) {
            uint32_t vLen;
            if (!pbReadVarint(buf, end, p, vLen) || p + vLen > end) break;
            size_t subEnd = p + vLen;
            while (p < subEnd) {
                uint32_t stag;
                if (!pbReadVarint(buf, subEnd, p, stag)) break;
                uint32_t sf = stag >> 3; uint8_t swt = stag & 0x07; uint32_t sval;
                if (swt == 0 && pbReadVarint(buf, subEnd, p, sval)) {
                    if (sf == 1 && sval >= 20 && sval <= 100) {
                        if (isTelem && recentCtrl) LOGF("[%s] Ignored stale Vacuum Suction = %u%%\n", tagLabel, (unsigned)sval);
                        else {
                            currentSuctionPct = (uint8_t)sval;
                            mqttVacSpeedVal = (uint8_t)sval;
                            mqttVacSpeedDirty = true;
                            LOGF("[%s] Vacuum Suction = %u%%\n", tagLabel, (unsigned)sval);
                        }
                    }
                } else if (!pbSkipField(buf, subEnd, p, swt)) break;
            }
            p = subEnd;
        } else if (!pbSkipField(buf, end, p, wt)) break;
    }
}

static void parseTelemetryBlock6A(const uint8_t* buf, size_t len) {
    size_t p = 0;
    while (p < len) {
        uint32_t tag; if (!pbReadVarint(buf, len, p, tag)) break;
        uint32_t f = tag >> 3; uint8_t wt = tag & 0x07;
        if (f == 2 && wt == 2) {
            uint32_t l; if (!pbReadVarint(buf, len, p, l) || p + l > len) break;
            size_t end = p + l;
            while (p < end) {
                uint32_t st; if (!pbReadVarint(buf, end, p, st)) break;
                uint32_t sf = st >> 3; uint8_t swt = st & 0x07;
                if (sf == 1 && swt == 0) { uint32_t cyc; if (pbReadVarint(buf, end, p, cyc) && cyc > 0) { portENTER_CRITICAL(&telemMux); extTelem.operatingCycles = cyc; portEXIT_CRITICAL(&telemMux); mqttTelemDirty = true; } }
                else if (!pbSkipField(buf, end, p, swt)) break;
            }
            p = end;
        } else if (!pbSkipField(buf, len, p, wt)) break;
    }
}

static void parseTelemetryBlock72(const uint8_t* buf, size_t len) {
    size_t p = 0;
    while (p < len) {
        uint32_t tag; if (!pbReadVarint(buf, len, p, tag)) break;
        uint32_t f = tag >> 3; uint8_t wt = tag & 0x07;
        if (wt == 2) {
            uint32_t l; if (!pbReadVarint(buf, len, p, l) || p + l > len) break;
            size_t end = p + l;
            while (p < end) {
                uint32_t st; if (!pbReadVarint(buf, end, p, st)) break;
                uint32_t sf = st >> 3; uint8_t swt = st & 0x07; uint32_t sval;
                if (sf == 1 && swt == 0 && pbReadVarint(buf, end, p, sval)) {
                    portENTER_CRITICAL(&telemMux);
                    if (f == 8) { extTelem.motorRunSec = sval; if (extTelem.filterRunSec == 0) extTelem.filterRunSec = sval; }
                    else if (f == 10 && sval > 0 && sval < 250) extTelem.temp1F = (int)sval;
                    else if (f == 12 && sval > 0 && sval < 250) extTelem.temp2F = (int)sval;
                    portEXIT_CRITICAL(&telemMux);
                    mqttTelemDirty = true;
                } else if (!pbSkipField(buf, end, p, swt)) break;
            }
            p = end;
        } else if (!pbSkipField(buf, len, p, wt)) break;
    }
}

static void parseTelemetryResponse33(const uint8_t* buf, size_t len) {
    size_t p = 0;
    while (p < len) {
        uint32_t tag; if (!pbReadVarint(buf, len, p, tag)) break;
        uint32_t f = tag >> 3; uint8_t wt = tag & 0x07;
        if (wt == 2) {
            uint32_t l; if (!pbReadVarint(buf, len, p, l) || p + l > len) break;
            if (f == 13) parseTelemetryBlock6A(buf + p, l);
            else if (f == 14) parseTelemetryBlock72(buf + p, l);
            p += l;
        } else if (!pbSkipField(buf, len, p, wt)) break;
    }
}

static void parseSettingsResponse12(const uint8_t* buf, size_t len) {
    size_t p = 0;
    while (p < len) {
        uint32_t tag; if (!pbReadVarint(buf, len, p, tag)) break;
        uint32_t f = tag >> 3; uint8_t wt = tag & 0x07;
        if (wt == 2) {
            uint32_t l; if (!pbReadVarint(buf, len, p, l) || p + l > len) break;
            size_t end = p + l;
            if (f == 3) {
                while (p < end) {
                    uint32_t st; if (!pbReadVarint(buf, end, p, st)) break;
                    uint32_t sf = st >> 3; uint8_t swt = st & 0x07; uint32_t sval;
                    if (sf == 1 && swt == 0 && pbReadVarint(buf, end, p, sval)) {
                        portENTER_CRITICAL(&telemMux);
                        extTelem.filterRunSec = sval;
                        if (extTelem.motorRunSec == 0) extTelem.motorRunSec = sval;
                        portEXIT_CRITICAL(&telemMux);
                        mqttTelemDirty = true;
                    } else if (!pbSkipField(buf, end, p, swt)) break;
                }
            }
            p = end;
        } else if (!pbSkipField(buf, len, p, wt)) break;
    }
}

static void parseFirmwareResponse13(const uint8_t* buf, size_t len, uint8_t srcNode) {
    size_t p = 0;
    while (p < len) {
        uint32_t tag; if (!pbReadVarint(buf, len, p, tag)) break;
        uint32_t f = tag >> 3; uint8_t wt = tag & 0x07;
        if (f == 2 && wt == 0) {
            uint32_t swMaj;
            if (pbReadVarint(buf, len, p, swMaj) && swMaj > 0) {
                portENTER_CRITICAL(&telemMux);
                if (srcNode == 0x12) extTelem.mainSwMajor = (uint8_t)swMaj;
                else if (srcNode == 0x14) extTelem.bleSwMajor = (uint8_t)swMaj;
                portEXIT_CRITICAL(&telemMux);
                mqttTelemDirty = true;
            }
        } else if (f == 10 && wt == 2) {
            uint32_t l; if (!pbReadVarint(buf, len, p, l) || p + l > len) break;
            size_t end = p + l;
            uint32_t maj = 0, minV = 0, patch = 0;
            char hashBuf[20] = {0}; size_t hLen = 0;
            while (p < end) {
                uint32_t st; if (!pbReadVarint(buf, end, p, st)) break;
                uint32_t sf = st >> 3; uint8_t swt = st & 0x07;
                if (swt == 0) {
                    uint32_t sv; if (!pbReadVarint(buf, end, p, sv)) break;
                    if (sf == 2) maj = sv; else if (sf == 3) minV = sv; else if (sf == 4) patch = sv;
                } else if (sf == 7 && swt == 2) {
                    uint32_t sl; if (!pbReadVarint(buf, end, p, sl) || p + sl > end) break;
                    hLen = 0;
                    for (uint32_t k = 0; k < sl && hLen + 1 < sizeof(hashBuf); k++) hashBuf[hLen++] = (char)buf[p + k];
                    hashBuf[hLen] = '\0';
                    p += sl;
                } else if (!pbSkipField(buf, end, p, swt)) break;
            }
            portENTER_CRITICAL(&telemMux);
            if (srcNode == 0x12 && maj > 0) snprintf(extTelem.mainFw, sizeof(extTelem.mainFw), "%u.%u.%u", (unsigned)maj, (unsigned)minV, (unsigned)patch);
            else if (srcNode == 0x14 && hLen > 0) strncpy(extTelem.bleFw, hashBuf, sizeof(extTelem.bleFw));
            portEXIT_CRITICAL(&telemMux);
            mqttTelemDirty = true;
            p = end;
        } else if (!pbSkipField(buf, len, p, wt)) break;
    }
}

static void parseDeviceInfoResponse5(const uint8_t* buf, size_t len) {
    size_t p = 0;
    while (p < len) {
        uint32_t tag; if (!pbReadVarint(buf, len, p, tag)) break;
        uint32_t f = tag >> 3; uint8_t wt = tag & 0x07;
        if (f == 25 && wt == 2) {
            uint32_t l; if (!pbReadVarint(buf, len, p, l) || p + l > len) break;
            char mdl[32] = {0}; size_t mLen = 0;
            for (uint32_t k = 0; k < l && mLen + 1 < sizeof(mdl); k++) { if (buf[p + k] >= 32 && buf[p + k] < 127) mdl[mLen++] = (char)buf[p + k]; }
            mdl[mLen] = '\0';
            if (mLen > 0) { portENTER_CRITICAL(&telemMux); strncpy(extTelem.modelName, mdl, sizeof(extTelem.modelName)); portEXIT_CRITICAL(&telemMux); mqttTelemDirty = true; }
            p += l;
        } else if (!pbSkipField(buf, len, p, wt)) break;
    }
}

static void parseIncomingFmpFrame(const uint8_t* dec, size_t len) {
    if (len < 6) return;
    uint8_t routeByte = dec[1];
    size_t payloadEnd = len - 2, p = 2;
    while (p < payloadEnd) {
        uint32_t topTag; if (!pbReadVarint(dec, payloadEnd, p, topTag)) break;
        uint32_t topField = topTag >> 3; uint8_t topWt = topTag & 0x07;
        if (topWt != 2) { if (!pbSkipField(dec, payloadEnd, p, topWt)) break; continue; }
        uint32_t msgLen; if (!pbReadVarint(dec, payloadEnd, p, msgLen) || p + msgLen > payloadEnd) break;
        size_t msgEnd = p + msgLen;
        if (topField == 4) {
            telemAwaitingReply = false;
            while (p < msgEnd) {
                uint32_t rTag; if (!pbReadVarint(dec, msgEnd, p, rTag)) break;
                uint32_t rField = rTag >> 3; uint8_t rWt = rTag & 0x07;
                if (rWt == 2) {
                    uint32_t subLen; if (!pbReadVarint(dec, msgEnd, p, subLen) || p + subLen > msgEnd) break;
                    if (rField == 21) parseCtControlFields(dec, p, p + subLen, "FMP TELEM");
                    else if (rField == 37) { if (subLen >= 2 && dec[p] == 0x08) { portENTER_CRITICAL(&telemMux); extTelem.waterLevelActive = (dec[p + 1] == 2); portEXIT_CRITICAL(&telemMux); mqttTelemDirty = true; } }
                    else if (rField == 33) parseTelemetryResponse33(dec + p, subLen);
                    else if (rField == 12) parseSettingsResponse12(dec + p, subLen);
                    else if (rField == 34) { portENTER_CRITICAL(&telemMux); if (subLen == 0) strncpy(extTelem.faultText, "Healthy", sizeof(extTelem.faultText)); else snprintf(extTelem.faultText, sizeof(extTelem.faultText), "Warning (Code %u)", (unsigned)dec[p]); portEXIT_CRITICAL(&telemMux); mqttTelemDirty = true; }
                    else if (rField == 13) parseFirmwareResponse13(dec + p, subLen, routeByte);
                    else if (rField == 5) parseDeviceInfoResponse5(dec + p, subLen);
                    p += subLen;
                } else if (!pbSkipField(dec, msgEnd, p, rWt)) break;
            }
        } else if (topField == 5) {
            while (p < msgEnd) {
                uint32_t eTag; if (!pbReadVarint(dec, msgEnd, p, eTag)) break;
                uint32_t eField = eTag >> 3; uint8_t eWt = eTag & 0x07;
                if (eWt == 2) {
                    uint32_t subLen; if (!pbReadVarint(dec, msgEnd, p, subLen) || p + subLen > msgEnd) break;
                    if (eField == 7) parseCtControlFields(dec, p, p + subLen, "FMP EVENT");
                    else if (eField == 16 && subLen >= 2) { if (dec[p] == 0x08 && dec[p + 1] == 0x02) { portENTER_CRITICAL(&telemMux); extTelem.waterLevelActive = true; portEXIT_CRITICAL(&telemMux); mqttTelemDirty = true; } }
                    p += subLen;
                } else if (!pbSkipField(dec, msgEnd, p, eWt)) break;
            }
        }
        p = msgEnd;
    }
}

void onFe02Notify(NimBLERemoteCharacteristic* pChr, uint8_t* pData, size_t length, bool isNotify) {
    (void)pChr; (void)isNotify;
    if (!pData || length == 0) return;
    for (size_t i = 0; i < length; i++) {
        uint8_t b = pData[i];
        if (b == 0x00) {
            if (rxCobsLen > 0) {
                uint8_t decoded[256];
                size_t decLen = cobsDecode(rxCobsBuf, rxCobsLen, decoded);
                if (decLen >= 4) {
                    uint16_t calcCrc = fmpCrc16(decoded, decLen - 2);
                    uint16_t pktCrc  = ((uint16_t)decoded[decLen - 2] << 8) | decoded[decLen - 1];
                    if (calcCrc == pktCrc) { lastBleActivityMs = millis(); parseIncomingFmpFrame(decoded, decLen); }
                }
                rxCobsLen = 0;
            }
        } else if (rxCobsLen < sizeof(rxCobsBuf)) rxCobsBuf[rxCobsLen++] = b;
        else rxCobsLen = 0;
    }
}

bool sendFmpRawPayloadTo(uint8_t routeByte, const uint8_t* protoBytes, size_t protoLen) {
    if (!protoBytes || protoLen == 0 || protoLen > 60) return false;
    if (connState != STATE_CONNECTED || !pClient || !pClient->isConnected() || !pWriteChar) return false;
    uint8_t raw[64]; size_t idx = 0;
    raw[idx++] = 0x02; raw[idx++] = routeByte;
    memcpy(raw + idx, protoBytes, protoLen); idx += protoLen;
    uint16_t crc = fmpCrc16(raw, idx);
    raw[idx++] = (crc >> 8) & 0xFF; raw[idx++] = crc & 0xFF;
    uint8_t frame[72]; size_t frameLen = cobsEncode(raw, idx, frame);
    bool useResp = !pWriteChar->canWriteNoResponse();
    for (size_t off = 0; off < frameLen; off += 20) {
        size_t chunkLen = (frameLen - off > 20) ? 20 : (frameLen - off);
        if (!pWriteChar->writeValue(frame + off, chunkLen, useResp)) return false;
        if (off + 20 < frameLen) delay(15);
    }
    lastBleActivityMs = millis();
    return true;
}

bool sendFmpRawPayload(const uint8_t* protoBytes, size_t protoLen) {
    return sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, protoBytes, protoLen);
}

bool sendFmpCtControl(uint8_t powerState, uint8_t suctionPct, uint8_t origin) {
    uint8_t ct[16]; size_t c = 0;
    if (powerState > 0) { ct[c++] = 0x08; ct[c++] = powerState; }
    if (suctionPct >= 20 && suctionPct <= 100) { ct[c++] = 0x22; ct[c++] = 0x02; ct[c++] = 0x08; ct[c++] = suctionPct; }
    if (origin > 0) { ct[c++] = 0x30; ct[c++] = origin; }
    uint8_t reqId = fmpReqId++; if (fmpReqId == 0 || fmpReqId > 127) fmpReqId = 1;
    uint8_t proto[32]; size_t p = 0;
    proto[p++] = 0x1A; proto[p++] = (uint8_t)(5 + c); proto[p++] = 0x08; proto[p++] = reqId;
    proto[p++] = 0xF2; proto[p++] = 0x03; proto[p++] = (uint8_t)c;
    memcpy(proto + p, ct, c); p += c;
    unsigned long txNow = millis(); lastControlTxMs = txNow; lastCtCtrlTxMs = txNow;
    return sendFmpRawPayload(proto, p);
}

bool sendFmpIdentify(bool enable) {
    uint8_t reqId = fmpReqId++; if (fmpReqId == 0 || fmpReqId > 127) fmpReqId = 1;
    lastControlTxMs = millis();
    if (enable) {
        identifyActive = true; identifyStartMs = millis();
        uint8_t proto[] = { 0x1A, 0x09, 0x08, reqId, 0xD2, 0x02, 0x04, 0x08, 0x01, 0x10, 0x0A };
        return sendFmpRawPayload(proto, sizeof(proto));
    } else {
        identifyActive = false;
        uint8_t proto[] = { 0x1A, 0x05, 0x08, reqId, 0xD2, 0x02, 0x00 };
        return sendFmpRawPayload(proto, sizeof(proto));
    }
}

void sendFmpHandshake() {
    uint8_t reqId = fmpReqId++; if (fmpReqId == 0 || fmpReqId > 127) fmpReqId = 1;
    uint8_t proto[] = { 0x1A, 0x05, 0x08, reqId, 0xAA, 0x01, 0x00 };
    sendFmpRawPayload(proto, sizeof(proto));
}

void startTelemetryPoll(bool fullRefresh) {
    if (connState != STATE_CONNECTED) return;
    telemPollRequested = true; telemAwaitingReply = false;
    telemPollStep = fullRefresh ? 1 : 4; lastTelemStepMs = 0;
}

void serviceTelemetryPoll(unsigned long now) {
    if (!telemPollRequested) return;
    if (connState != STATE_CONNECTED) { telemPollRequested = false; telemAwaitingReply = false; return; }
    if (isAnyPrimaryDemandActive() || hoseClearActive || demandDirty || isAnyButtonGestureBusy()) return;
    if (elapsedMs(now, lastControlTxMs) < 1500UL) return;
    if (telemAwaitingReply) { if (elapsedMs(now, lastTelemStepMs) < 1200UL) return; }
    else { if (elapsedMs(now, lastTelemStepMs) < 450UL) return; }
    lastTelemStepMs = now; telemAwaitingReply = true;
    uint8_t reqId = fmpReqId++; if (fmpReqId == 0 || fmpReqId > 127) fmpReqId = 1;
    switch (telemPollStep) {
        case 1: { uint8_t p[] = { 0x1A, 0x04, 0x08, reqId, 0x6A, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, p, sizeof(p)); break; }
        case 2: { uint8_t p[] = { 0x1A, 0x04, 0x08, reqId, 0x6A, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_BLE, p, sizeof(p)); break; }
        case 3: { uint8_t p[] = { 0x1A, 0x04, 0x08, reqId, 0x2A, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_BLE, p, sizeof(p)); break; }
        case 4: { uint8_t p[] = { 0x1A, 0x09, 0x08, reqId, 0x8A, 0x02, 0x04, 0x0A, 0x00, 0x72, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, p, sizeof(p)); break; }
        case 5: { uint8_t p[] = { 0x1A, 0x09, 0x08, reqId, 0x8A, 0x02, 0x04, 0x0A, 0x00, 0x6A, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, p, sizeof(p)); break; }
        case 6: { uint8_t p[] = { 0x1A, 0x04, 0x08, reqId, 0x62, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, p, sizeof(p)); break; }
        case 7: { uint8_t p[] = { 0x1A, 0x05, 0x08, reqId, 0xAA, 0x02, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, p, sizeof(p)); break; }
        case 8: { uint8_t p[] = { 0x1A, 0x05, 0x08, reqId, 0x92, 0x02, 0x00 }; sendFmpRawPayloadTo(FMP_ROUTE_APP_TO_MAIN, p, sizeof(p)); break; }
        default: telemPollRequested = false; telemAwaitingReply = false; return;
    }
    telemPollStep++;
}

void stopExtractorMotor(bool forceSend) {
    webUiSpeedOverride = 0;
    if (connState != STATE_CONNECTED) return;
    if (currentPowerState == 1 || forceSend) {
        if (sendFmpCtControl(3, 0, 1)) {
            currentPowerState = 3;
            publishExtractorTelemetry(millis());
            LOGLN("[EXTRACT] Motor OFF sent.");
        }
    }
}
