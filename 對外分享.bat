@echo off
setlocal EnableExtensions
chcp 950 >nul
title 智慧物流系統 - 對外分享 (ngrok)
cd /d "%~dp0"

set "BACKEND_PORT=8000"

echo.
echo  ============================================================
echo     智慧物流系統 - 對外分享（ngrok 公開網址）
echo  ============================================================
echo.
echo   本視窗的用途：把運行中的後端，接到一條有密碼保護的公開網址。
echo   啟動前：請先雙擊 啟動系統.bat（後端必須活著）。
echo   停用方式：關閉本視窗（Y），公開網址立即失效。
echo.

REM ---------- 1. 檢查 ngrok 是否已安裝 ----------
where ngrok >nul 2>&1
if errorlevel 1 (
    echo  [錯誤]  找不到 ngrok，尚未安裝。
    echo.
    echo         請到以下網址安裝其一：
    echo             管理員 PowerShell 執行:  winget install ngrok.ngrok
    echo         或至官網手動下載:  https://ngrok.com/download
    echo         安裝完成後，請重開 PowerShell 再試。
    echo.
    pause
    exit /b 1
)
echo  [1/3] ngrok 已安裝

REM ---------- 2. 檢查 authtoken 是否已設定 ----------
ngrok config check >nul 2>&1
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
echo     即將啟動 ngrok（開啟瀏覽器至本機儀表板）...
echo  ============================================================
echo.
echo     取得公開網址: http://127.0.0.1:4040
echo        (儀表板 Endpoints 頁籤 -> 複製 HTTPS 網址給使用者)
echo.
echo     預設帳號密碼: user / ChangeMe_12345
echo     要換密碼請編輯 ngrok-policy.yml 後重新執行本檔。
echo.
echo     停用公開: 關閉本視窗 (Y)。
echo.
start "" http://127.0.0.1:4040
ngrok http %BACKEND_PORT% --traffic-policy-file ngrok-policy.yml

echo.
echo  ngrok 已結束，公開網址已失效。
timeout /t 3 >nul
