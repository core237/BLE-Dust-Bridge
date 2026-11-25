/**
 * BLE Dust Bridge - Generic Fast-Response Shelly Plug Power Beacon (Gen3 / Gen4)
 *
 * Stateless telemetry broadcaster (Company ID 0x0BA9, signature "SC"):
 *  - Byte 0-1: "SC" (0x53, 0x43)
 *  - Byte 2:   (seq << 2) | (relayOn ? 1 : 0)
 *  - Byte 3-4: Live wattage (uint16 little-endian, W)
 *  - Byte 5:   Instantaneous positive ramp delta (uint8, W jump over last 120ms sample, capped at 255)
 *
 * All floor cutoffs and trigger decisions stay in the BLE Dust Bridge Web UI.
 */

var CONFIG = {
  // BLE local name advertised to BLE Dust Bridge (max 17 chars to fit 31B ADV PDU)
  deviceName: "Shelly-Dust",
  // Switch component ID (0 for single-outlet Shelly Plug US / Plus Plug)
  switchId: 0,
  // Local power sampling interval in milliseconds
  pollMs: 120,
  // Minimum wattage change (W) to trigger an immediate BLE burst
  stepDeltaW: 10,
  // Minimum hardware spacing (ms) so Shelly finishes airing across ch 37/38/39
  burstCooldownMs: 250,
  // Steady-state refresh interval (ms) to keep the 3000 ms broadcast alive
  keepAliveMs: 2000
};

var seq = 0;
var tickMs = 0;
var lastSampleWatts = -1;
var lastTxMs = -10000;
var lastTxWatts = -1;
var lastTxRelay = null;
var hasBootTx = false;

function getNowMs() {
  if (typeof Shelly !== "undefined" && typeof Shelly.getUptimeMs === "function") {
    var up = Shelly.getUptimeMs();
    if (typeof up === "number" && up >= 0) {
      return up;
    }
  }
  return tickMs;
}

function transmitBleBeacon(relayOn, watts, rampW, nowMs) {
  try {
    var clampedW = watts < 0 ? 0 : (watts > 65535 ? 65535 : watts);
    var clampedRamp = rampW < 0 ? 0 : (rampW > 255 ? 255 : rampW);
    seq = (seq + 1) & 0x3F;
    var flagsByte = (seq << 2) | (relayOn ? 0x01 : 0x00);
    var wLo = clampedW & 0xFF;
    var wHi = (clampedW >> 8) & 0xFF;

    var mfgPayload = String.fromCharCode(0x53, 0x43, flagsByte, wLo, wHi, clampedRamp);

    BLE.AdvBuilder.reset();
    if (!BLE.AdvBuilder.addShellyManufacturerData(mfgPayload)) {
      print("[BLDB] Error: Failed to add manufacturer data");
      return false;
    }

    if (CONFIG.deviceName && CONFIG.deviceName.length > 0) {
      var safeName = CONFIG.deviceName.length > 17
        ? CONFIG.deviceName.slice(0, 17)
        : CONFIG.deviceName;
      BLE.AdvBuilder.addName(safeName, true);
    }

    var advData = BLE.AdvBuilder.build();
    if (BLE.advertiseOnce(advData)) {
      lastTxMs = nowMs;
      lastTxWatts = clampedW;
      lastTxRelay = relayOn;
      hasBootTx = true;
      return true;
    }
  } catch (err) {
    print("[BLDB] Broadcast exception:", JSON.stringify(err));
  }
  return false;
}

function evaluatePowerState(isTimerTick) {
  if (isTimerTick) {
    tickMs += CONFIG.pollMs;
  }

  var sw = null;
  try {
    sw = Shelly.getComponentStatus("switch", CONFIG.switchId);
  } catch (e) {
    return;
  }
  if (!sw) return;

  var relayOn = !!sw.output;
  var rawPower = (typeof sw.apower === "number") ? sw.apower : 0;
  var watts = relayOn ? Math.max(0, Math.round(rawPower)) : 0;
  var rampW = (lastSampleWatts >= 0 && watts > lastSampleWatts) ? (watts - lastSampleWatts) : 0;
  lastSampleWatts = watts;

  var nowMs = getNowMs();
  var sinceLastTx = nowMs - lastTxMs;
  if (sinceLastTx < 0) {
    sinceLastTx = CONFIG.keepAliveMs;
  }

  var relayChanged = (lastTxRelay === null) || (relayOn !== lastTxRelay);
  var bigPowerStep = (lastTxWatts < 0) || (Math.abs(watts - lastTxWatts) >= CONFIG.stepDeltaW);
  var keepAliveDue = (sinceLastTx >= CONFIG.keepAliveMs);

  if (!hasBootTx || keepAliveDue) {
    transmitBleBeacon(relayOn, watts, rampW, nowMs);
    return;
  }

  if ((relayChanged || bigPowerStep) && sinceLastTx >= CONFIG.burstCooldownMs) {
    transmitBleBeacon(relayOn, watts, rampW, nowMs);
  }
}

Shelly.addStatusHandler(function (event) {
  if (!event || event.component !== ("switch:" + CONFIG.switchId)) return;
  if (event.delta && (typeof event.delta.output !== "undefined" || typeof event.delta.apower !== "undefined")) {
    evaluatePowerState(false);
  }
});

evaluatePowerState(false);
Timer.set(CONFIG.pollMs, true, function () {
  evaluatePowerState(true);
});

print("[BLDB] Generic Shelly power beacon started (poll=" + CONFIG.pollMs + "ms, step=" + CONFIG.stepDeltaW + "W, cooldown=" + CONFIG.burstCooldownMs + "ms)");