#!/usr/bin/env python3
import os
import hashlib
import subprocess
from pathlib import Path
import difflib
import datetime

OLD_DIR = "err_Packages"
NEW_DIR = "Packages"
LOG_FILE = f"compare_result_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"

def log(msg):
    print(msg)  # 输出到终端
    with open(LOG_FILE, "a", encoding="utf-8") as f:
        f.write(msg + "\n")

def file_hash(path, algo="md5"):
    h = hashlib.new(algo)
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()

def run_cmd(cmd):
    try:
        return subprocess.check_output(cmd, stderr=subprocess.STDOUT).decode("utf-8", errors="ignore").splitlines()
    except subprocess.CalledProcessError as e:
        return e.output.decode("utf-8", errors="ignore").splitlines()

def compare_text(old_lines, new_lines, title):
    diff = list(difflib.unified_diff(old_lines, new_lines, lineterm="", fromfile="old/"+title, tofile="new/"+title))
    return diff

def main():
    old_files = {f.name: f for f in Path(OLD_DIR).glob("*.so*")}
    new_files = {f.name: f for f in Path(NEW_DIR).glob("*.so*")}

    common_files = sorted(set(old_files) & set(new_files))

    if not common_files:
        log("没有找到需要比较的共享库文件")
        return

    for fname in common_files:
        log("=" * 80)
        log(f"比较文件: {fname}")
        old_path = old_files[fname]
        new_path = new_files[fname]

        # 文件大小
        old_size = old_path.stat().st_size
        new_size = new_path.stat().st_size
        log(f"[大小] {old_size} vs {new_size} bytes")

        # 哈希
        old_md5 = file_hash(old_path, "md5")
        new_md5 = file_hash(new_path, "md5")
        log(f"[MD5 ] {old_md5} vs {new_md5}")

        old_sha256 = file_hash(old_path, "sha256")
        new_sha256 = file_hash(new_path, "sha256")
        log(f"[SHA256] {old_sha256} vs {new_sha256}")

        # file 信息
        old_file_info = run_cmd(["file", str(old_path)])
        new_file_info = run_cmd(["file", str(new_path)])
        if old_file_info != new_file_info:
            log("[file] 差异：")
            for diff in compare_text(old_file_info, new_file_info, "file"):
                log(diff)

        # ELF 头信息
        old_readelf_h = run_cmd(["readelf", "-h", str(old_path)])
        new_readelf_h = run_cmd(["readelf", "-h", str(new_path)])
        if old_readelf_h != new_readelf_h:
            log("[readelf -h] 差异：")
            for diff in compare_text(old_readelf_h, new_readelf_h, "readelf-h"):
                log(diff)

        # 动态段依赖
        old_needed = run_cmd(["readelf", "-d", str(old_path)])
        new_needed = run_cmd(["readelf", "-d", str(new_path)])
        if old_needed != new_needed:
            log("[readelf -d] 差异：")
            for diff in compare_text(old_needed, new_needed, "readelf-d"):
                log(diff)

        # 符号表
        old_nm = run_cmd(["nm", "-D", "--with-symbol-versions", str(old_path)])
        new_nm = run_cmd(["nm", "-D", "--with-symbol-versions", str(new_path)])
        if old_nm != new_nm:
            log("[nm -D] 差异：")
            for diff in compare_text(old_nm, new_nm, "nm-D"):
                log(diff)

        # objdump 符号表
        old_objdump_T = run_cmd(["objdump", "-T", str(old_path)])
        new_objdump_T = run_cmd(["objdump", "-T", str(new_path)])
        if old_objdump_T != new_objdump_T:
            log("[objdump -T] 差异：")
            for diff in compare_text(old_objdump_T, new_objdump_T, "objdump-T"):
                log(diff)

        # objdump 反汇编（非常详细，可能很大）
        log("[objdump -d] 开始对比反汇编代码（仅显示差异行）...")
        old_dis = run_cmd(["objdump", "-d", str(old_path)])
        new_dis = run_cmd(["objdump", "-d", str(new_path)])
        dis_diff = compare_text(old_dis, new_dis, "objdump-d")
        if dis_diff:
            for diff in dis_diff:
                log(diff)
        else:
            log("无反汇编差异")

    log("=" * 80)
    log("比较完成！")
    log(f"日志已保存到: {LOG_FILE}")

if __name__ == "__main__":
    main()

