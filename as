<# cleanup.ps1（cmdkey方針・ログ簡素版・goto廃止）
 - Pack_YYYYMMDDhhmm.pkgx を「新しい順で3つ残し、以降削除」
 - 実行ごとに新規ログ（pkgx_cleanup_YYYYMMDD_HHMMSS.log）
 - ログも新しい順で3本だけ残す
 - 認証は cmdkey 事前登録を利用（スクリプトに平文パス不要）
 - ログにはパターン名・KEEP数は出しません
#>

# ===== ここだけ切替 =====
$TargetDir = 'C:\backup'                  # ローカル試験
# $TargetDir = '\\NAS01\Share\backup'     # NAS運用（cmdkey /add:NAS01 の登録名と一致させる）
# ========================

$LogDir      = 'C:\log'
$KeepCount   = 3
$NamePattern = 'Pack_????????????.pkgx'   # 12桁(YYYYMMDDhhmm)

# ログ（毎回 新規）
$ts = Get-Date -Format 'yyyyMMdd_HHmmss'
$LogFile = Join-Path $LogDir ("pkgx_cleanup_{0}.log" -f $ts)

function Write-Log {
  param([string]$Message)
  $stamp = Get-Date -Format '[yyyy/MM/dd HH:mm:ss]'
  $line  = "{0} {1}" -f $stamp, $Message
  if (-not (Test-Path -LiteralPath $LogFile)) {
    Set-Content -LiteralPath $LogFile -Value $line -Encoding UTF8
  } else {
    Add-Content -LiteralPath $LogFile -Value $line -Encoding UTF8
  }
  Write-Host $line
}

function Rotate-Logs {
  $logFiles = Get-ChildItem -LiteralPath $LogDir -Filter 'pkgx_cleanup_????????_??????.log' -File |
              Sort-Object Name -Descending
  $logFiles | Select-Object -Skip $KeepCount | ForEach-Object {
    try   { Remove-Item -LiteralPath $_.FullName -Force -ErrorAction Stop }
    catch { Write-Log ("[WARN] ログ削除失敗 {0} : {1}" -f $_.Name, $_.Exception.Message) }
  }
}

# 事前チェック（フォルダは作らない方針）
if (!(Test-Path -LiteralPath $LogDir))    { Write-Host "[ERROR] ログフォルダがありません: $LogDir";    exit 1 }
if (!(Test-Path -LiteralPath $TargetDir)) { Write-Host "[ERROR] 対象フォルダがありません: $TargetDir"; exit 1 }

Write-Log "===== pkgx cleanup start ====="
Write-Log ("TargetDir={0}" -f $TargetDir)

# 一覧取得（名前降順＝新しい順）
try {
  $files = Get-ChildItem -LiteralPath $TargetDir -Filter $NamePattern -File -ErrorAction Stop |
           Sort-Object Name -Descending
}
catch {
  Write-Log ("[ERROR] 一覧取得に失敗: {0}" -f $_.Exception.Message)
  Write-Log "===== done (ERROR) ====="
  Rotate-Logs
  exit 1
}

if (!$files -or $files.Count -eq 0) {
  Write-Log "[INFO] 対象なし"
  Write-Log "===== done (OK) ====="
  Rotate-Logs
  exit 0
}

# 3件KEEP、以降DELETE
$keep = $files | Select-Object -First $KeepCount
$del  = $files | Select-Object -Skip  $KeepCount

$keep | ForEach-Object { Write-Log ("[KEEP] {0}" -f $_.Name) }

$delOk = 0; $delNg = 0
foreach ($f in $del) {
  try {
    Remove-Item -LiteralPath $f.FullName -Force -ErrorAction Stop
    Write-Log ("[DEL ] {0}" -f $f.Name)
    $delOk++
  }
  catch {
    Write-Log ("[WARN] 削除失敗 {0} : {1}" -f $f.Name, $_.Exception.Message)
    $delNg++
  }
}

Write-Log ("[INFO] 合計 {0}／保持 {1}／削除 {2}／失敗 {3}" -f $files.Count, $keep.Count, $delOk, $delNg)
Write-Log ("===== done ({0}) =====" -f ($(if($delNg -gt 0) {'ERROR'} else {'OK'})))

Rotate-Logs
exit ($(if($delNg -gt 0){1}else{0}))
