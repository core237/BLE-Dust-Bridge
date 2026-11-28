import sys
import time
import struct
import serial

PORT = "/dev/cu.SLAB_USBtoUART"
BAUD = 460800
TARGET_MAC = "78:C5:E5:6E:D6:24"
LOG_FILE = "78_c5_e5_6e_d6_24_ota.log"

SLIP_START = 0xAB
SLIP_END = 0xBC
SLIP_ESC = 0xAC
SLIP_ESC_START = 0xAA
SLIP_ESC_END = 0xBB
SLIP_ESC_ESC = 0xAD

REQ_FOLLOW = 0x00
EVENT_FOLLOW = 0x01
EVENT_CONNECT = 0x05
EVENT_PACKET_DATA = 0x06
REQ_SCAN_CONT = 0x07
EVENT_DISCONNECT = 0x09
SET_TEMPORARY_KEY = 0x0C
PING_REQ = 0x0D

ATT_OPCODES = {
    0x01: "ATT_ERROR_RSP",
    0x08: "ATT_READ_BY_TYPE_REQ",
    0x09: "ATT_READ_BY_TYPE_RSP",
    0x0A: "ATT_READ_REQ",
    0x0B: "ATT_READ_RSP",
    0x10: "ATT_READ_BY_GRP_TYPE_REQ",
    0x11: "ATT_READ_BY_GRP_TYPE_RSP",
    0x12: "ATT_WRITE_REQ",
    0x13: "ATT_WRITE_RSP",
    0x1B: "ATT_HANDLE_VALUE_NTF",
    0x1D: "ATT_HANDLE_VALUE_IND",
    0x52: "ATT_WRITE_CMD",
}

SMP_OPCODES = {
    0x01: "SMP_PAIRING_REQ",
    0x02: "SMP_PAIRING_RSP",
    0x03: "SMP_PAIRING_CONFIRM",
    0x04: "SMP_PAIRING_RANDOM",
    0x05: "SMP_PAIRING_FAILED",
    0x06: "SMP_ENCRYPTION_INFO (LTK)",
    0x07: "SMP_MASTER_IDENT",
    0x08: "SMP_IDENTITY_INFO (IRK)",
    0x09: "SMP_IDENTITY_ADDR_INFO",
    0x0C: "SMP_PUBLIC_KEY (LESC)",
    0x0D: "SMP_DHKEY_CHECK (LESC)",
}

def slip_encode(raw_body: bytes) -> bytes:
    out = bytearray([SLIP_START])
    for b in raw_body:
        if b == SLIP_START: out.extend([SLIP_ESC, SLIP_ESC_START])
        elif b == SLIP_END: out.extend([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC: out.extend([SLIP_ESC, SLIP_ESC_ESC])
        else: out.append(b)
    out.append(SLIP_END)
    return bytes(out)

def slip_decode(escaped_body: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(escaped_body):
        b = escaped_body[i]
        if b == SLIP_ESC and i + 1 < len(escaped_body):
            nxt = escaped_body[i + 1]
            if nxt == SLIP_ESC_START: out.append(SLIP_START)
            elif nxt == SLIP_ESC_END: out.append(SLIP_END)
            elif nxt == SLIP_ESC_ESC: out.append(SLIP_ESC)
            else: out.append(nxt)
            i += 2
        else:
            out.append(b)
            i += 1
    return bytes(out)

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

    # 0x78 top bits = 01 -> Public Address (addr_type = 0x00)
    mac_msb = bytes([int(x, 16) for x in TARGET_MAC.split(":")])
    addr_type = b"\x00"

    ser.reset_input_buffer()
    send_cmd(REQ_SCAN_CONT)
    time.sleep(0.05)
    send_cmd(SET_TEMPORARY_KEY, b"\x00" * 16)
    send_cmd(REQ_FOLLOW, mac_msb + addr_type)

    log_f = open(LOG_FILE, "w")
    def emit(line: str):
        print(line)
        log_f.write(line + "\n")
        log_f.flush()

    emit("=" * 105)
    emit(f"OTA FULL PACKET & PAIRING SNIFFER | Target: {TARGET_MAC} (Public) | Log: {LOG_FILE}")
    emit("=" * 105)

    buf = bytearray()
    last_ping = time.time()

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

                frame = slip_decode(bytes(buf[1:e_idx]))
                del buf[: e_idx + 1]
                if len(frame) < 6:
                    continue

                hdr_len = frame[0]
                pkt_id = frame[5]
                t_str = time.strftime("%H:%M:%S", time.localtime(now)) + f".{int((now % 1) * 1000):03d}"

                if pkt_id == EVENT_FOLLOW:
                    send_cmd(SET_TEMPORARY_KEY, b"\x00" * 16)
                    emit(f"[{t_str}] [SNIFFER] Locked onto {TARGET_MAC} — initiate connection/pairing now.")
                elif pkt_id == EVENT_CONNECT:
                    send_cmd(SET_TEMPORARY_KEY, b"\x00" * 16)
                    emit(f"\n[{t_str}] === [CONNECTED!] Link established with {TARGET_MAC} (TK=0 armed) ===")
                elif pkt_id == EVENT_DISCONNECT:
                    reason = frame[6] if len(frame) > 6 else 0
                    emit(f"[{t_str}] === [DISCONNECTED] Reason=0x{reason:02X} — re-arming follow ===\n")
                    send_cmd(REQ_SCAN_CONT)
                    send_cmd(REQ_FOLLOW, mac_msb + addr_type)
                    send_cmd(SET_TEMPORARY_KEY, b"\x00" * 16)
                elif pkt_id == EVENT_PACKET_DATA and len(frame) >= hdr_len + 10:
                    evt_hdr_len = frame[hdr_len]
                    flags = frame[hdr_len + 1]
                    if not (flags & 0x01):  # CRC check
                        continue
                    direction = "C->P" if (flags & 0x02) else "P->C"
                    enc = "ENC(MIC_OK)" if (flags & 0x04) else "PLAIN"
                    ch = frame[hdr_len + 2]
                    rssi = -frame[hdr_len + 3]
                    ble_start = hdr_len + evt_hdr_len
                    if len(frame) < ble_start + 6:
                        continue

                    aa = frame[ble_start : ble_start + 4]
                    if aa == b"\xd6\xbe\x89\x8e":  # Advertising channel
                        pdu_type = frame[ble_start + 4] & 0x0F
                        pdu_len = frame[ble_start + 5] & 0x3F
                        if len(frame) >= ble_start + 7 + pdu_len and pdu_len >= 6:
                            mac = ":".join(f"{b:02X}" for b in reversed(frame[ble_start + 7 : ble_start + 13]))
                            if mac == TARGET_MAC:
                                adv = frame[ble_start + 13 : ble_start + 7 + pdu_len]
                                emit(f"[{t_str}] [ADV] CH:{ch:<2} {rssi:>3}dBm PDU=0x{pdu_type:02X} | {adv.hex(' ')}")
                        continue

                    # Data channel
                    llid = frame[ble_start + 4] & 0x03
                    pdu_len = frame[ble_start + 5]
                    if pdu_len == 0:
                        continue
                    cand_7 = frame[ble_start + 7 : ble_start + 7 + pdu_len]
                    cand_6 = frame[ble_start + 6 : ble_start + 6 + pdu_len]
                    payload = cand_7 if len(cand_7) == pdu_len else cand_6

                    if llid == 0x03:
                        emit(f"[{t_str}] [{direction}] CH:{ch:<2} {rssi:>3}dBm {enc:<11} | [LL_CTRL 0x{payload[0]:02X}] {payload[1:].hex(' ')}")
                    elif llid == 0x02 and len(payload) >= 5:
                        l2_len, l2_cid = struct.unpack("<HH", payload[:4])
                        body = payload[4 : 4 + l2_len]
                        op = body[0] if body else 0
                        if l2_cid == 0x0006:
                            emit(f"[{t_str}] [{direction}] CH:{ch:<2} {rssi:>3}dBm {enc:<11} | [SMP] {SMP_OPCODES.get(op, f'0x{op:02X}')}: {body[1:].hex(' ')}")
                        elif l2_cid == 0x0004:
                            if op in (0x12, 0x52, 0x1B, 0x1D) and len(body) >= 3:
                                h = struct.unpack("<H", body[1:3])[0]
                                emit(f"[{t_str}] [{direction}] CH:{ch:<2} {rssi:>3}dBm {enc:<11} | [ATT] {ATT_OPCODES.get(op)} Handle=0x{h:04X} -> {body[3:].hex(' ')}")
                            else:
                                emit(f"[{t_str}] [{direction}] CH:{ch:<2} {rssi:>3}dBm {enc:<11} | [ATT] {ATT_OPCODES.get(op, f'0x{op:02X}')}: {body[1:].hex(' ')}")

    except KeyboardInterrupt:
        print("\nStopped.")
        log_f.close()
        ser.close()
        print(f"[✓] Saved OTA log to {LOG_FILE}")

if __name__ == "__main__":
    main()
