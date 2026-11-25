Import("env")
import glob, os, subprocess

def build_factory_bin(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    fw_bin = os.path.join(build_dir, "firmware.bin")
    boot_bin = os.path.join(build_dir, "bootloader.bin")
    part_bin = os.path.join(build_dir, "partitions.bin")

    esptool_list = glob.glob(os.path.expanduser("~/.platformio/packages/tool-esptoolpy*/esptool.py"))
    boot_app0_list = glob.glob(os.path.expanduser("~/.platformio/packages/framework-arduinoespressif32*/tools/partitions/boot_app0.bin"))
    if not esptool_list or not boot_app0_list:
        print("[FACTORY] Warning: esptool.py or boot_app0.bin not found; skipping factory merge.")
        return

    out_bin = os.path.join(env.subst("$PROJECT_DIR"), "installer", "bldb-factory.bin")
    os.makedirs(os.path.dirname(out_bin), exist_ok=True)

    cmd = [
        env.subst("$PYTHONEXE"), esptool_list[0], "--chip", "esp32s3", "merge_bin",
        "-o", out_bin,
        "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "8MB",
        "0x0", boot_bin,
        "0x8000", part_bin,
        "0xe000", boot_app0_list[0],
        "0x10000", fw_bin
    ]
    subprocess.run(cmd, check=True)
    print(f"[FACTORY] Updated {out_bin} ({os.path.getsize(out_bin)} bytes)")

env.AddPostAction("$BUILD_DIR/firmware.bin", build_factory_bin)
