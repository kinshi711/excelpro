import sys
import time

# Python 3.9 用 CarMaker CMAPI モジュールパス（Pythonバージョンに応じて変更）
sys.path.append(r"C:\IPG\CarMaker\13.0\bin\win64\python3.9")

# CMAPI をインポート
import cmapi

# CarMaker に接続（Application Connect が ON の必要あり）
app = cmapi.App()
app.connect()

# 実行したい .tcl ファイルのパス
tcl_path = r"C:\IPG\CarMaker\13.0\Projects\YourProject\Data\Tests\MyScript.tcl"

# 1行ずつコマンドとして送信
with open(tcl_path, 'r') as file:
    for line in file:
        cmd = line.strip()
        if cmd and not cmd.startswith("#"):
            print(f"Sending: {cmd}")
            try:
                app.send(cmd)
                time.sleep(0.2)  # 必要に応じて調整
            except Exception as e:
                print(f"Error on '{cmd}': {e}")
