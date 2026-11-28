import asyncio
import time
from bleak import BleakClient, BleakScanner

TARGET_MAC = "78:C5:E5:6E:D6:24"
LOG_FILE = "78_c5_e5_6e_d6_24_gatt.log"

def log(msg: str):
    ts = time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"
    line = f"[{ts}] {msg}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")

async def main():
    open(LOG_FILE, "w").close()
    log(f"Scanning for {TARGET_MAC}... Press the button on the remote to wake it if asleep.")

    target_dev = None
    def on_adv(dev, adv):
        nonlocal target_dev
        # macOS hides raw MACs behind a UUID, so check address, mfg data, or local name
        addr_match = dev.address.upper() == TARGET_MAC
        mfg_str = " ".join(f"{k:04x}:{v.hex()}" for k, v in adv.manufacturer_data.items())
        if addr_match or "78c5e56ed624" in mfg_str.replace(" ", ""):
            target_dev = dev
            log(f"Matched target: Name='{dev.name}' | Addr={dev.address} | RSSI={adv.rssi}dBm | Mfg={mfg_str}")

    scanner = BleakScanner(detection_callback=on_adv)
    await scanner.start()
    for _ in range(150):  # 15s scan window
        if target_dev:
            break
        await asyncio.sleep(0.1)
    await scanner.stop()

    if not target_dev:
        log("Target not matched by MAC/Mfg. Listing visible devices so you can pick it if macOS masked the MAC:")
        devices = await BleakScanner.discover(timeout=5.0, return_adv=True)
        for d, adv in devices.values():
            log(f"  Candidate: Name='{d.name}' | CoreBT_ID={d.address} | RSSI={adv.rssi}dBm | UUIDs={adv.service_uuids}")
        return

    log(f"Connecting & initiating SMP pairing with {target_dev.address}...")
    async with BleakClient(target_dev, pair=True, timeout=25.0) as client:
        log(f"Connected & paired! Enumerating full GATT table...")
        notify_chars = []

        for svc in client.services:
            log(f"\nSERVICE: {svc.uuid} ({svc.description})")
            for c in svc.characteristics:
                props = ",".join(c.properties)
                if "read" in c.properties:
                    try:
                        val = await client.read_gatt_char(c.uuid)
                        asc = "".join(chr(b) if 32 <= b <= 126 else "." for b in val)
                        log(f"  Handle 0x{c.handle:04X} | {c.uuid} | [{props}] -> {val.hex(' ')}  |{asc}|")
                    except Exception as e:
                        log(f"  Handle 0x{c.handle:04X} | {c.uuid} | [{props}] -> ReadErr: {e}")
                else:
                    log(f"  Handle 0x{c.handle:04X} | {c.uuid} | [{props}]")

                if "notify" in c.properties or "indicate" in c.properties:
                    notify_chars.append(c)

        if notify_chars:
            log(f"\nSubscribing to {len(notify_chars)} notify/indicate characteristics. Press/hold button for 15s...")
            for c in notify_chars:
                try:
                    await client.start_notify(
                        c.uuid,
                        lambda sender, data, h=c.handle: log(f"NOTIFY Handle=0x{h:04X} ({sender.uuid}): {data.hex(' ')}")
                    )
                except Exception as e:
                    log(f"  Notify subscribe failed for 0x{c.handle:04X}: {e}")
            await asyncio.sleep(15.0)

    log(f"Done. Saved full log to {LOG_FILE}")

if __name__ == "__main__":
    asyncio.run(main())
