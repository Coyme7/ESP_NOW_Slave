import os
import shutil
import subprocess
from pathlib import Path

Import("env")

def link_or_copy_file(link_path, target_path):
    if link_path.exists():
        return
    subprocess.run(
        ["cmd", "/c", "mklink", "/H", str(link_path), str(target_path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if not link_path.exists():
        shutil.copy2(target_path, link_path)


def link_directory(link_path, target_path):
    if link_path.exists():
        return
    subprocess.run(
        ["cmd", "/c", "mklink", "/J", str(link_path), str(target_path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )

# Arduino-ESP32 3.x 的 idf_component.yml 会为 esp32s3 拉取语音、RainMaker、
# Zigbee 等可选组件；当前业务固件只需要 Arduino core、WiFi、SPI 和 ESP-NOW。
os.environ["IDF_COMPONENT_MANAGER"] = "0"
env["ENV"]["IDF_COMPONENT_MANAGER"] = "0"

platform = env.PioPlatform()
toolchain_dir = platform.get_package_dir("toolchain-xtensa-esp-elf")
if toolchain_dir:
    toolchain_root = Path(toolchain_dir)
    nested_bin = Path(toolchain_dir) / "xtensa-esp-elf" / "bin"
    nested_root = Path(toolchain_dir) / "xtensa-esp-elf"
    binutils_bin = nested_root / "xtensa-esp-elf" / "bin"
    target_sysroot = nested_root / "xtensa-esp-elf"
    target_lib = nested_root / "xtensa-esp-elf" / "lib"
    compat_lib = nested_root / "lib"
    for entry_name in ("bin", "include", "lib", "libexec", "picolibc", "share"):
        link_path = toolchain_root / entry_name
        target_path = nested_root / entry_name
        if target_path.exists() and not link_path.exists():
            link_directory(link_path, target_path)

    # pioarduino 当前包布局会让顶层 gcc 入口把 sysroot 算到 nested_root。
    # 真实 target runtime 在 nested_root/xtensa-esp-elf/lib，这里只补缺失映射。
    if target_lib.is_dir() and compat_lib.is_dir():
        for target_entry in target_lib.iterdir():
            compat_entry = compat_lib / target_entry.name
            if target_entry.is_dir():
                link_directory(compat_entry, target_entry)
            elif target_entry.is_file():
                link_or_copy_file(compat_entry, target_entry)

    if nested_bin.is_dir():
        path_entries = [str(nested_bin)]
        if binutils_bin.is_dir():
            path_entries.insert(0, str(binutils_bin))
            env["ENV"]["COMPILER_PATH"] = str(binutils_bin)
            os.environ["COMPILER_PATH"] = str(binutils_bin)

        if target_lib.is_dir():
            library_entries = [str(target_lib)]
            esp32s3_lib = target_lib / "esp32s3"
            if esp32s3_lib.is_dir():
                library_entries.insert(0, str(esp32s3_lib))
            library_path = os.pathsep.join(library_entries)
            env["ENV"]["LIBRARY_PATH"] = library_path
            os.environ["LIBRARY_PATH"] = library_path

        if target_sysroot.is_dir():
            sysroot_flag = f"--sysroot={target_sysroot}"
            env.Append(CCFLAGS=[sysroot_flag], LINKFLAGS=[sysroot_flag])

        for path_entry in path_entries:
            env.PrependENVPath("PATH", path_entry)
        os.environ["PATH"] = os.pathsep.join(path_entries) + os.pathsep + os.environ.get("PATH", "")

        tool_prefix = nested_bin / "xtensa-esp32s3-elf"
        tool_vars = {
            "CC": "gcc.exe",
            "CXX": "g++.exe",
            "AS": "gcc.exe",
            "AR": "ar.exe",
            "LD": "ld.exe",
            "NM": "nm.exe",
            "OBJCOPY": "objcopy.exe",
            "OBJDUMP": "objdump.exe",
            "RANLIB": "ranlib.exe",
            "READELF": "readelf.exe",
            "STRIP": "strip.exe",
            "SIZE": "size.exe",
        }
        for env_name, suffix in tool_vars.items():
            tool_path = Path(f"{tool_prefix}-{suffix}")
            if tool_path.is_file():
                tool_path_text = str(tool_path)
                os.environ[env_name] = tool_path_text
                env["ENV"][env_name] = tool_path_text
                env.Replace(**{env_name: tool_path_text})
