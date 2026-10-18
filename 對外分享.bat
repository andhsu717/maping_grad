@echo off
setlocal EnableExtensions EnableDelayedExpansion
chcp 65001 >nul
title 智慧物流系統 - 對外分享 [ngrok]
cd /d "%~dp0"

set "BACKEND_PORT=8000"
set "NGROK="

echo.
echo  ============================================================
echo     智慧物流系統 - 對外分享【ngrok 公開網址】
echo  ============================================================
echo.

REM ---------- 1. 尋找 ngrok ----------
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
    echo  [錯誤] 找不到 ngrok，尚未安裝。
    echo.
    echo         請以管理員 PowerShell 執行: winget install ngrok.ngrok
    echo         或至官網手動下載: https://ngrok.com/download
    echo.
    pause
    exit /b 1
)
echo  [1/3] ngrok 已找到: !NGROK!

REM ---------- 2. 檢查 authtoken ----------
"!NGROK!" config check >nul 2>&1
if errorlevel 1 (
    echo.
    echo  [錯誤] 未設定 authtoken，請先註冊 ngrok 帳號並綁定。
    echo.
    echo         取得 token: https://dashboard.ngrok.com/get-started/your-authtoken
    echo         於 PowerShell 執行: ngrok config add-authtoken 你的Token
    echo.
    pause
    exit /b 1
)
echo  [2/3] authtoken 已設定

REM ---------- 3. 檢查後端是否已在本機運行 ----------
powershell -NoProfile -Command "$c=New-Object System.Net.Sockets.TcpClient;try{$c.Connect('127.0.0.1',%BACKEND_PORT%);$c.Close();exit 0}catch{exit 1}" >nul 2>&1
if errorlevel 1 (
    echo.
    echo  [錯誤] 後端伺服器 127.0.0.1:%BACKEND_PORT% 未在運行。
    echo.
    echo         ngrok 只是通道，無法幫你啟動後端伺服器；
    echo         請先雙擊【啟動系統.bat】，確定後端已啟動後再執行本檔。
    echo.
    pause
    exit /b 1
)
echo  [3/3] 後端運行中: 127.0.0.1:%BACKEND_PORT%

if not exist "%~dp0ngrok-policy.yml" (
    echo.
    echo  [錯誤] 找不到 ngrok-policy.yml 流量策略設定檔。
    echo         請確認該檔案與本批次檔位於同一個資料夾。
    echo.
    pause
    exit /b 1
)

echo.
echo  ============================================================
echo     即將啟動 ngrok 通道...
echo  ============================================================
echo.
echo     本機管理儀表板: http://127.0.0.1:4040
echo     停止公開分享: 直接關閉本視窗即可。
echo.

start "" http://127.0.0.1:4040
"!NGROK!" http %BACKEND_PORT% --traffic-policy-file "%~dp0ngrok-policy.yml"

echo.
echo  ngrok 已結束，公開網址已失效。
pause