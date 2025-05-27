a
import subprocess
import time
import os

# --- 設定（適宜修正してください） ---
carmaker_root = r"C:\IPG\CarMaker\13.0\Win64"
project_path = r"C:\Users\YourUser\IPG\YourProject"

cm_exe = os.path.join(carmaker_root, "bin", "win64", "CM.exe")
ipgcontrol_exe = os.path.join(carmaker_root, "bin", "win64", "IPGControl.exe")

# --- 1. CarMaker本体を起動 ---
cm_process = subprocess.Popen([
    cm_exe,
    "-project", project_path
])

# --- 2. 起動待ち（GUIが立ち上がるまで。5〜10秒程度調整） ---
time.sleep(10)

# --- 3. ApplicationConnectを実行 ---
subprocess.run([
    ipgcontrol_exe,
    "-c", "ApplicationConnect",
    "-project", project_path
])

# --- 4. 必要に応じてそのままシナリオ実行など続けてもOK ---
