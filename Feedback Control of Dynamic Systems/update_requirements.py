#!/usr/bin/env python3
import subprocess
import sys
import shutil

def log(msg):
    print(f">>> {msg}")

def try_run_main():
    log(f"使用 {sys.executable} 运行 main.py …")
    try:
        subprocess.check_call([sys.executable, "main.py"] + sys.argv[1:], stdout=sys.stdout)
        log("main.py 运行成功")
    except subprocess.CalledProcessError as e:
        print(f"⚠️ main.py 运行失败（{e.returncode}），跳过，继续下一步")

def install_pipreqs():
    log("检测到 pipreqs 未安装，正在安装 pipreqs …")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pipreqs"], stdout=sys.stdout)
    exe = shutil.which("pipreqs")
    if not exe:
        print("❌ pipreqs 安装失败，请检查网络或环境")
        sys.exit(1)
    return exe

def generate_requirements():
    pipreqs_exe = shutil.which("pipreqs")
    if not pipreqs_exe:
        try:
            pipreqs_exe = install_pipreqs()
        except Exception:
            pipreqs_exe = None

    if pipreqs_exe:
        cmd = [pipreqs_exe, ".", "--force", "--encoding", "utf-8"]
        log(f"调用 pipreqs 更新 requirements.txt：{' '.join(cmd)}")
        try:
            subprocess.check_call(cmd, shell=(sys.platform=="win32"), stdout=sys.stdout)
            log("requirements.txt 已用 pipreqs 更新完成")
            return
        except subprocess.CalledProcessError:
            print("⚠️ pipreqs 执行失败，回退到 pip freeze")

    log("使用 pip freeze 更新 requirements.txt …")
    reqs = subprocess.check_output([sys.executable, "-m", "pip", "freeze"])
    with open("requirements.txt", "wb") as f:
        f.write(reqs)
    log("requirements.txt 已用 pip freeze 更新完成")

def install_requirements():
    log("正在安装 requirements.txt 中的依赖 …")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"], stdout=sys.stdout)
    log("依赖安装完成")

def preview_requirements():
    log("requirements.txt 前20行预览：")
    try:
        with open("requirements.txt", encoding="utf-8") as f:
            lines = f.readlines()
        for i, line in enumerate(lines[:20]):
            print(line.rstrip())
        if len(lines) > 20:
            print(f"...（共 {len(lines)} 行，已显示前 20 行）")
    except FileNotFoundError:
        print("❌ 找不到 requirements.txt")

if __name__ == "__main__":
    # 第一次尝试运行 main.py（可注释掉如果不需要）
    try_run_main()

    # 更新 requirements.txt
    generate_requirements()
    preview_requirements()

    # 安装新依赖
    install_requirements()

    # 再次运行 main.py
    log("依赖安装完毕，重新运行 main.py …")
    subprocess.check_call([sys.executable, "main.py"] + sys.argv[1:], stdout=sys.stdout)
    log("全部完成 🎉")
