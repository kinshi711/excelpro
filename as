@echo off
rem ===== 運用を1つだけ選んでコメント解除 =====
set "DIR=C:\SQLBackups"                        rem (A) ローカル ← 既定
rem set "DIR=\\NAS01\Share\Backups"            rem (B) NAS：UNC直（認証不要/資格情報済み）
rem set "NAS=\\NAS01\Share\Backups"            rem (C) NAS：認証が必要な場合は↓も設定
rem set "USR=DOMAIN\backupsvc"
rem set "PWD=YourPassword"
rem ==========================================

if defined NAS (
  net use "%NAS%" /user:%USR% %PWD% /persistent:no >nul || exit /b 1
  set "DIR=%NAS%"
)

pushd "%DIR%" || exit /b 1
for /f "skip=3 delims=" %%F in ('dir /b /a-d /o:-d *.bak 2^>nul') do del /f /q "%%F"
popd

if defined NAS net use "%NAS%" /delete /y >nul
