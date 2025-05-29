#!/usr/bin/env python3
import subprocess
import sys
import os
import importlib
import traceback
import argparse

REQUIREMENTS_FILE = "requirements.txt"
FIRST_ORDER  = "-o 1"
FIRST_ORDER_LAG = "-o 2"
SECOND_ORDER = "-o 3"
#MAIN_SCRIPT = "main.py" +" "+FIRST_ORDER
MAIN_SCRIPT = "main.py" 
MAX_MAIN_RUNS = 2  # 最多尝试运行 main.py 两次


def log(msg):
    print(f">>> {msg}")

def read_requirements():
    if not os.path.exists(REQUIREMENTS_FILE):
        log(f"{REQUIREMENTS_FILE} 不存在，跳过依赖检查")
        return []
    with open(REQUIREMENTS_FILE, encoding="utf-8") as f:
        lines = f.readlines()
    modules = []
    for line in lines:
        line = line.strip()
        if line and not line.startswith("#"):
            mod = line.split("==")[0].strip()
            if mod:
                modules.append(mod)
    return modules

def install_missing(modules):
    missing = []
    for mod in modules:
        try:
            importlib.import_module(mod)
        except ImportError:
            missing.append(mod)

    if not missing:
        log("✅ 所有 requirements.txt 中的模块已安装")
        return False

    log(f"发现缺失模块：{missing}，正在安装 …")
    subprocess.check_call([sys.executable, "-m", "pip", "install"] + missing, stdout=sys.stdout)
    return True

def ensure_pipreqs():
    try:
        import pipreqs
    except ImportError:
        log("未检测到 pipreqs，正在安装 …")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pipreqs"], stdout=sys.stdout)

def update_requirements():
    ensure_pipreqs()
    log("使用 pipreqs 更新 requirements.txt …")
    subprocess.check_call([sys.executable, "-m", "pipreqs", ".", "--force", "--encoding", "utf-8"], stdout=sys.stdout)
    log("✅ requirements.txt 已更新")

def run_main(type):
    try:
        print([sys.executable, MAIN_SCRIPT] + sys.argv[1:])
        if type == 1:
            subprocess.check_call([sys.executable, MAIN_SCRIPT,FIRST_ORDER] + sys.argv[1:],stdout=sys.stdout)
        if type == 2:
            subprocess.check_call([sys.executable, MAIN_SCRIPT,FIRST_ORDER_LAG] + sys.argv[1:],stdout=sys.stdout)
        if type == 3:
            subprocess.check_call([sys.executable, MAIN_SCRIPT,SECOND_ORDER] + sys.argv[1:],stdout=sys.stdout)
        return True
    except subprocess.CalledProcessError as e:
        log(f"⚠️ main.py 运行失败，退出码：{e.returncode}")
        return False
    except Exception as e:
        log(f"❌ 运行 main.py 异常：\n{traceback.format_exc()}")
        return False

def main():
    # 1. 设置参数解析器
    parser = argparse.ArgumentParser(description="热系统仿真程序")
    parser.add_argument("-o", "--order", type=int, choices=[1, 2, 3], required=True,
                        help="选择系统阶数 (1: 一阶, 2: 一阶滞后 3: 二阶)")
    args = parser.parse_args()
    # Step 1: 检查并安装 requirements.txt 中的模块
    log("🔍 检查 requirements.txt 中的依赖模块 …")
    modules = read_requirements()
    if modules:
        install_missing(modules)

    for attempt in range(1, MAX_MAIN_RUNS + 1):
        log(f"🚀 第 {attempt} 次尝试运行 {MAIN_SCRIPT} …")
        success = run_main(args.order)
        if success:
            break
        log("📦 尝试自动修复缺失模块并更新 requirements.txt …")
        update_requirements()
        new_modules = read_requirements()
        install_missing(new_modules)

    log("✅ 任务完成")

if __name__ == "__main__":
    main()
