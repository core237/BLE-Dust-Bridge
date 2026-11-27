import sys
import time
import serial

PORT = "/dev/cu.SLAB_USBtoUART"
BAUD = 460800
TARGET_MAC = "E8:F6:0A:7D:79:0E"
LOG_FILE = "e8_f6_0a_7d_79_0e_packets.log"

SLIP_START = 0xAB
SLIP_END = 0xBC
SLIP_ESC = 0xAC
SLIP_ESC_START = 0xAA
SLIP_ESC_END = 0xBB
SLIP_ESC_ESC = 0xAD

REQ_SCAN_CONT = 0x07
EVENT_PACKET_DATA = 0x06
PING_REQ = 0x0D

PDU_TYPES = {
    0x00: "ADV_IND (Passive)",
    0x01: "ADV_DIRECT_IND",
    0x02: "ADV_NONCONN (Passive)",
    0x04: "SCAN_RSP (Active)",
    0x06: "ADV_SCAN_IND (Passive)",
}

def slip_encode(raw_body: bytes) -> bytes:
    out = bytearray([SLIP_START])
    for b in raw_body:
        if b == SLIP_START:
            out.extend([SLIP_ESC, SLIP_ESC_START])
        elif b == SLIP_END:
            out.extend([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            out.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def slip_decode(escaped_body: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(escaped_body):
        b = escaped_body[i]
        if b == SLIP_ESC and i + 1 < len(escaped_body):
            nxt = escaped_body[i + 1]
            if nxt == SLIP_ESC_START:
                out.append(SLIP_START)
            elif nxt == SLIP_ESC_END:
                out.append(SLIP_END)
            elif nxt == SLIP_ESC_ESC:
                out.append(SLIP_ESC)
            else:
                out.append(nxt)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)

def decode_ad(adv_data: bytes) -> str:
    parts = []
    pos = 0
    while pos < len(adv_data):
        length = adv_data[pos]
        if length == 0 or pos + 1 + length > len(adv_data):
            break
        ad_type = adv_data[pos + 1]
        content = adv_data[pos + 2 : pos + 1 + length]

        if ad_type == 0x01 and content:
            parts.append(f"Flags=0x{content[0]:02X}")
        elif ad_type in (0x08, 0x09):
            name = content.decode("utf-8", errors="ignore").strip()
            parts.append(f"Name='{name}'")
        elif ad_type in (0x02, 0x03) and len(content) >= 2:
            uuids = [f"0x{int.from_bytes(content[i:i+2], 'little'):04X}" for i in range(0, len(content), 2)]
            parts.append(f"UUID={','.join(uuids)}")
        elif ad_type == 0x16 and len(content) >= 2:
            uuid = int.from_bytes(content[:2], "little")
            parts.append(f"Svc(0x{uuid:04X})={content[2:].hex(' ')}")
        elif ad_type == 0xFF and len(content) >= 2:
            cid = int.from_bytes(content[:2], "little")
            parts.append(f"Mfg(0x{cid:04X})={content[2:].hex(' ')}")
        else:
            parts.append(f"AD(0x{ad_type:02X})={content.hex(' ')}")

        pos += 1 + length
    return " | ".join(parts) if parts else "(Empty)"

def format_delta(dt_sec) -> str:
    if dt_sec is None:
        return "FIRST"
    ms = dt_sec * 1000.0
    return f"+{ms:5.0f}ms" if ms < 1000.0 else f"+{dt_sec:5.2f}s"

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.02)
    except Exception as e:
        print(f"Error opening {PORT}: {e}")
        sys.exit(1)

    cnt = 0
    def send_cmd(pkt_id: int, payload: bytes = b""):
        nonlocal cnt
        hdr = bytes([0x06, len(payload), 0x01, cnt & 0xFF, (cnt >> 8) & 0xFF, pkt_id])
        cnt = (cnt + 1) & 0xFFFF
        ser.write(slip_encode(hdr + payload))

    ser.reset_input_buffer()
    send_cmd(REQ_SCAN_CONT)
    send_cmd(PING_REQ)
    time.sleep(0.15)
    ser.reset_input_buffer()

    log_f = open(LOG_FILE, "w")

    def emit(line: str):
        print(line)
        log_f.write(line + "\n")
        log_f.flush()

    header = (
        f"{'PKT#':<5} | {'TIMESTAMP':<12} | {'DELTA':<8} | {'MAC ADDRESS':<17} | "
        f"{'CH':<2} | {'RSSI':<6} | {'PDU TYPE':<22} | {'RAW PAYLOAD (HEX)':<48} | {'DECODED / DIFF'}"
    )
    emit("=" * len(header))
    emit(f"REAL-TIME BLE PACKET RECORDER | Target: {TARGET_MAC} | Started: {time.strftime('%Y-%m-%d %H:%M:%S')} | File: {LOG_FILE}")
    emit("=" * len(header))
    emit(header)
    emit("-" * len(header))

    last_seen = None
    last_payload = {}
    pkt_count = 0
    last_ping = time.time()
    buf = bytearray()

    try:
        while True:
            now = time.time()
            if now - last_ping > 2.0:
                send_cmd(PING_REQ)
                last_ping = now

            chunk = ser.read(1024)
            if not chunk:
                continue
            buf.extend(chunk)

            while True:
                s_idx = buf.find(bytes([SLIP_START]))
                if s_idx == -1:
                    buf.clear()
                    break
                if s_idx > 0:
                    del buf[:s_idx]

                e_idx = buf.find(bytes([SLIP_END]), 1)
                if e_idx == -1:
                    break

                raw_slip = bytes(buf[1:e_idx])
                del buf[: e_idx + 1]

                frame = slip_decode(raw_slip)
                if len(frame) < 16:
                    continue

                hdr_len = frame[0]
                pkt_id = frame[5]
                if pkt_id != EVENT_PACKET_DATA or len(frame) < hdr_len + 10:
                    continue

                evt_hdr_len = frame[hdr_len]
                flags = frame[hdr_len + 1]
                crc_ok = bool(flags & 0x01)
                channel = frame[hdr_len + 2]
                rssi = -frame[hdr_len + 3]

                if not crc_ok or channel > 39:
                    continue

                ble_start = hdr_len + evt_hdr_len
                if len(frame) < ble_start + 13:
                    continue

                if frame[ble_start : ble_start + 4] != b"\xd6\xbe\x89\x8e":
                    continue

                pdu_type = frame[ble_start + 4] & 0x0F
                pdu_len = frame[ble_start + 5] & 0x3F

                if pdu_type == 0x03 or pdu_len < 6 or len(frame) < ble_start + 7 + pdu_len:
                    continue

                mac = ":".join(f"{b:02X}" for b in reversed(frame[ble_start + 7 : ble_start + 13]))
                if mac != TARGET_MAC:
                    continue

                adv_data = frame[ble_start + 13 : ble_start + 7 + pdu_len]
                pkt_count += 1

                dt = None if last_seen is None else (now - last_seen)
                last_seen = now
                delta_str = format_delta(dt)

                t_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"
                pdu_label = PDU_TYPES.get(pdu_type, f"PDU_0x{pdu_type:02X}")
                raw_hex = adv_data.hex(" ")
                decoded = decode_ad(adv_data)

                prev = last_payload.get(pdu_type)
                diff_tag = "[NEW]" if prev is None else ("[CHANGED!]" if prev != adv_data else "[SAME]")
                last_payload[pdu_type] = adv_data

                row = (
                    f"{pkt_count:<5} | {t_str:<12} | {delta_str:<8} | {mac:<17} | "
                    f"{channel:<2} | {rssi:>3}dBm | {pdu_label:<22} | {raw_hex:<48} | {diff_tag} {decoded}"
                )
                emit(row)

    except KeyboardInterrupt:
        print("\nStopped.")
        log_f.close()
        ser.close()
        print(f"[✓] Recorded {pkt_count} packets to {LOG_FILE}\n")

if __name__ == "__main__":
    main()
