@echo off
setlocal EnableExtensions EnableDelayedExpansion
chcp 950 >nul
title 智慧物流系統 - 對外分享 (ngrok)
cd /d "%~dp0"

set "BACKEND_PORT=8000"
set "NGROK="

echo.
echo  ============================================================
echo     智慧物流系統 - 對外分享（ngrok 公開網址）
echo  ============================================================
echo.
echo   本視窗的用途：把運行中的後端，接到一條有密碼保護的公開網址。
echo   啟動前：請先雙擊 啟動系統.bat（後端必須活著）。
echo   停用方式：關閉本視窗，公開網址立即失效。
echo.

REM ---------- 1. 尋找 ngrok（PATH 找不到時自動回退到 winget 安裝位置） ----------
where ngrok >nul 2>&1
if not errorlevel 1 set "NGROK=ngrok"

if not defined NGROK (
    if exist "%LOCALAPPDATA%\Microsoft\WinGet\Links\ngrok.exe" (
        set "NGROK=%LOCALAPPDATA%\Microsoft\WinGet\Links\ngrok.exe"
    )
)

if not defined NGROK (
    for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\Ngrok.Ngrok_*") do (
        if exist "%%D\ngrok.exe" set "NGROK=%%D\ngrok.exe"
    )
)

if not defined NGROK (
    echo  [錯誤]  找不到 ngrok，尚未安裝。
    echo.
    echo         請以管理員 PowerShell 執行:  winget install ngrok.ngrok
    echo         或至官網手動下載:  https://ngrok.com/download
    echo.
    pause
    exit /b 1
)
echo  [1/3] ngrok 已找到: !NGROK!

REM ---------- 2. 檢查 authtoken ----------
!NGROK! config check >nul 2>&1
if errorlevel 1 (
    echo.
    echo  [錯誤]  未設定 authtoken，請先註冊 ngrok 帳號並綁定。
    echo.
    echo         取得 token:  https://dashboard.ngrok.com/get-started/your-authtoken
    echo         於 PowerShell 執行:  ngrok config add-authtoken <你的Token>
    echo.
    pause
    exit /b 1
)
echo  [2/3] authtoken 已設定

REM ---------- 3. 檢查後端是否已在本機運行 ----------
powershell -NoProfile -Command "try{$c=New-Object System.Net.Sockets.TcpClient;$c.Connect('127.0.0.1',%BACKEND_PORT%);$c.Close()}catch{exit 1}" >nul 2>&1
if errorlevel 1 (
    echo.
    echo  [錯誤]  後端 (127.0.0.1:%BACKEND_PORT%) 未在運行。
    echo.
    echo         ngrok 只是窗口，無法幫你啟動後端；沒開後端時
    echo         外人只會看到 502 錯誤。
    echo         請先雙擊 啟動系統.bat，確定後端已啟動後再執行本檔。
    echo.
    pause
    exit /b 1
)
echo  [3/3] 後端運行中 (127.0.0.1:%BACKEND_PORT%)

if not exist "ngrok-policy.yml" (
    echo.
    echo  [錯誤]  找不到 ngrok-policy.yml（密碼保護設定檔）。
    echo         請確認檔案與本 bat 在同一資料夾。
    pause
    exit /b 1
)

echo.
echo  ============================================================
echo     即將啟動 ngrok（瀏覽器會自動開啟本機儀表板）...
echo  ============================================================
echo.
echo     取得公開網址: http://127.0.0.1:4040
echo        (儀表板 Endpoints 頁籤 -> 複製 HTTPS 網址給使用者)
echo.
echo     帳號密碼: 見 ngrok-policy.yml 的 credentials 欄位
echo     要換密碼請編輯該檔後重新執行本檔。
echo.
echo     停用公開: 關閉本視窗。
echo.
start "" http://127.0.0.1:4040
!NGROK! http %BACKEND_PORT% --traffic-policy-file ngrok-policy.yml

echo.
echo  ngrok 已結束，公開網址已失效。
echo  若為異常中斷，請看上方錯誤訊息；修好後重新執行本檔。
echo.
pause
