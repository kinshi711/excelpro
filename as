@echo off
setlocal EnableExtensions EnableDelayedExpansion

rem === 設定（既存前提）========================================
set "TARGET_DIR=C:\backup"                   & rem .pkgx がある場所
set "LOG_DIR=C:\log"                         & rem ログ保存先（既存前提）
set "LIST_FILE=%LOG_DIR%\_pkgx_list.tmp"     & rem 一時リスト（最後に削除）
for /f %%I in ('powershell -NoProfile -Command "(Get-Date).ToString(\"yyyyMMdd_HHmmss\")"') do set "TS=%%I"
set "LOG_FILE=%LOG_DIR%\pkgx_cleanup_%TS%.log"  & rem ★実行ごとに新規作成（タイムスタンプ付き）
rem ============================================================

rem --- NAS を使う場合（必要時のみ REM を外す）-----------------
REM set "NAS_PATH=\\NAS01\Share\backup"
REM set "NAS_USER=YOUR_USER"
REM set "NAS_PASS=YOUR_PASSWORD"
REM net use "%NAS_PATH%" "%NAS_PASS%" /user:%NAS_USER% /persistent:no >nul
REM if errorlevel 1 ( echo [ERROR] NAS接続失敗: %NAS_PATH% & exit /b 1 )
REM set "TARGET_DIR=%NAS_PATH%"
REM rem set "LOG_DIR=%NAS_PATH%\logs"
REM rem set "LIST_FILE=%LOG_DIR%\_pkgx_list.tmp"
REM rem for /f %%I in ('powershell -NoProfile -Command "(Get-Date).ToString(\"yyyyMMdd_HHmmss\")"') do set "TS=%%I"
REM rem set "LOG_FILE=%LOG_DIR%\pkgx_cleanup_%TS%.log"
rem ------------------------------------------------------------

rem --- 事前チェック（フォルダは作成しない方針） ---
if not exist "%LOG_DIR%\." (
  echo [ERROR] ログフォルダがありません: %LOG_DIR%
  exit /b 1
)

rem ★毎回“新規に”ログを開始（上書き）
> "%LOG_FILE%" echo [%date% %time%] ===== pkgx cleanup start =====
>>"%LOG_FILE%" echo [%date% %time%] [INFO] TARGET_DIR=%TARGET_DIR%

if not exist "%TARGET_DIR%\." (
  >>"%LOG_FILE%" echo [%date% %time%] [ERROR] 対象フォルダがありません: %TARGET_DIR%
  goto :END_ERR
)

rem --- .pkgx の一覧を作成（新しい順）。パターンは12桁日時のみ対象 ---
dir /b /a-d /o:-n "%TARGET_DIR%\Pack_????????????.pkgx" > "%LIST_FILE%" 2>nul

rem --- 件数カウント ---
set /a total=0
for /f "usebackq delims=" %%L in ("%LIST_FILE%") do set /a total+=1
>>"%LOG_FILE%" echo [%date% %time%] [INFO] list count=%total%

if %total%==0 (
  >>"%LOG_FILE%" echo [%date% %time%] [INFO] 対象なし（Pack_????????????.pkgx）
  goto :END_OK
)

rem --- 先頭3件KEEP、4件目以降DELETE ---
set /a i=0, keepcount=0, delcount=0, failcount=0
for /f "usebackq delims=" %%F in ("%LIST_FILE%") do (
  set /a i+=1
  if !i! LEQ 3 (
    set /a keepcount+=1
    >>"%LOG_FILE%" echo [%date% %time%] [KEEP] %%F
  ) else (
    del /f /q "%TARGET_DIR%\%%F"
    if errorlevel 1 (
      set /a failcount+=1
      >>"%LOG_FILE%" echo [%date% %time%] [WARN] 削除失敗 %%F
    ) else (
      set /a delcount+=1
      >>"%LOG_FILE%" echo [%date% %time%] [DEL ] %%F
    )
  )
)

rem --- サマリ出力 ---
if %failcount% GTR 0 (
  >>"%LOG_FILE%" echo [%date% %time%] [INFO] 合計 %total%／保持 %keepcount%／削除 %delcount%／失敗 %failcount%
  goto :END_ERR
) else (
  >>"%LOG_FILE%" echo [%date% %time%] [INFO] 合計 %total%／保持 %keepcount%／削除 %delcount%
  goto :END_OK
)

:END_OK
>>"%LOG_FILE%" echo [%date% %time%] ===== done (OK) =====

rem --- ここで“ログも3本KEEP”にローテーション ------------------
set /a logcount=0
for /f "delims=" %%L in ('
  dir /b /a-d /o:-n "%LOG_DIR%\pkgx_cleanup_????????_??????.log" 2^>nul
') do (
  set /a logcount+=1
  if !logcount! GTR 3 (
    del /f /q "%LOG_DIR%\%%L"
    if errorlevel 1 (
      >>"%LOG_FILE%" echo [%date% %time%] [WARN] ログ削除失敗 %%L
    ) else (
      >>"%LOG_FILE%" echo [%date% %time%] [LOGDEL] %%L
    )
  )
)
goto :CLEANUP

:END_ERR
>>"%LOG_FILE%" echo [%date% %time%] ===== done (ERROR) =====
rem ログのローテーション自体は実行（異常時でもログが膨らみ過ぎないように）
set /a logcount=0
for /f "delims=" %%L in ('
  dir /b /a-d /o:-n "%LOG_DIR%\pkgx_cleanup_????????_??????.log" 2^>nul
') do (
  set /a logcount+=1
  if !logcount! GTR 3 (
    del /f /q "%LOG_DIR%\%%L"
    if errorlevel 1 (
      >>"%LOG_FILE%" echo [%date% %time%] [WARN] ログ削除失敗 %%L
    ) else (
      >>"%LOG_FILE%" echo [%date% %time%] [LOGDEL] %%L
    )
  )
)

:CLEANUP
del /f /q "%LIST_FILE%" >nul 2>&1
endlocal
exit /b 0

