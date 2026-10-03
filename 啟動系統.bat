@echo off
setlocal EnableExtensions EnableDelayedExpansion
title 智慧物流系統 - 主控台
cd /d "%~dp0"

set "BACKEND_PORT=8000"
set "FRONTEND_PORT=8080"
set "PY="
set "PY_ARGS="
set "PY_VER="

echo.
echo  ============================================================
echo     智慧物流動態排程與互動視覺化系統   一鍵啟動
echo  ============================================================
echo.

REM ---------- 1. 檢查必要檔案 ----------
set "MISSING="
if not exist "index.html"           set "MISSING=!MISSING! index.html"
if not exist "optimizer_service.py" set "MISSING=!MISSING! optimizer_service.py"
if not exist "runmap.py"            set "MISSING=!MISSING! runmap.py"
if not "!MISSING!"=="" goto missing_file
echo  [1/6] 必要檔案檢查完成

REM ---------- 2. 偵測 Python ----------
call :try_python py -3
if defined PY goto python_ok
call :try_python python
if defined PY goto python_ok
call :try_python python3
if defined PY goto python_ok
goto no_python

:python_ok
echo  [2/6] Python 環境就緒 : !PY! !PY_ARGS!  版本 !PY_VER!

REM ---------- 3. 檢查並安裝套件 ----------
%PY% %PY_ARGS% -c "import fastapi, uvicorn, requests, pydantic" >nul 2>&1
if errorlevel 1 goto need_install
echo        所需套件已安裝，無需更新
goto install_ok

:need_install
echo        偵測到缺少套件，正在自動安裝，請稍候 ...
%PY% %PY_ARGS% -m pip install --disable-pip-version-check -q fastapi uvicorn requests pydantic
if errorlevel 1 goto install_fail

:install_ok
echo  [3/6] 依賴套件檢查完成

REM ---------- 4. 檢查埠號占用 ----------
set "SKIP_BACKEND=0"
call :is_port_busy !BACKEND_PORT!
if "!PORT_BUSY!"=="1" (
    echo.
    echo  [注意] 埠號 !BACKEND_PORT! 已被占用
    echo         後端服務可能已在執行中，將直接開啟網頁。
    echo         若占用者並非本系統，請先關閉後再重新執行。
    set "SKIP_BACKEND=1"
) else (
    echo  [4/6] 埠號檢查完成
)

call :is_port_busy !FRONTEND_PORT!
if "!PORT_BUSY!"=="1" (
    echo.
    echo  [錯誤] 埠號 !FRONTEND_PORT! 已被占用，無法啟動前端伺服器。
    echo         請關閉占用該埠號的程式後重新執行。
    echo.
    pause
    exit /b 1
)

REM ---------- 5. 啟動後端服務 ----------
if "!SKIP_BACKEND!"=="1" goto backend_ok
echo  [5/6] 正在啟動後端 API 服務 (127.0.0.1:!BACKEND_PORT!) ...
start "智慧物流系統 - 後端服務 API" cmd /k "%PY% %PY_ARGS% optimizer_service.py"
echo        等待後端服務就緒 ...
set "READY=0"
for /l %%i in (1,1,40) do (
    if "!READY!"=="0" (
        call :is_port_busy !BACKEND_PORT!
        if "!PORT_BUSY!"=="1" ( set "READY=1" ) else ( ping -n 2 127.0.0.1 >nul )
    )
)
if "!READY!"=="0" (
    echo.
    echo  [錯誤] 後端服務啟動逾時或失敗
    echo         請手動開啟命令提示字元，執行下列指令查看錯誤訊息
    echo             %PY% %PY_ARGS% optimizer_service.py
    echo.
    pause
    exit /b 1
)
echo        後端服務啟動成功

:backend_ok
REM ---------- 6. 啟動前端服務 ----------
echo  [6/6] 正在啟動前端網頁伺服器 (localhost:!FRONTEND_PORT!) ...
start "智慧物流系統 - 前端網頁伺服器" cmd /k "%PY% %PY_ARGS% runmap.py"
timeout /t 2 /nobreak >nul

echo.
echo  ============================================================
echo     系統已就緒，瀏覽器將自動開啟
echo  ============================================================
echo.
echo     網頁網址 : http://localhost:!FRONTEND_PORT!/index.html
echo     API 文件 : http://127.0.0.1:!BACKEND_PORT!/docs
echo.
echo     請保持以下兩個視窗開啟
echo         智慧物流系統 - 後端服務 API
echo         智慧物流系統 - 前端網頁伺服器
echo.
echo     首次載入需下載地圖圖磚，請稍候數秒
echo     排程計算需連線至 OSRM 公共服務，請確認網路正常
echo.
echo  ------------------------------------------------------------
echo     按任意鍵即可停止系統並關閉所有服務視窗
echo  ------------------------------------------------------------
echo.
pause >nul

echo  正在關閉服務，請稍候 ...
call :kill_port !BACKEND_PORT!
call :kill_port !FRONTEND_PORT!
echo  系統已停止。
timeout /t 1 /nobreak >nul
exit /b 0


REM ============================================================
REM  錯誤處理
REM ============================================================

:missing_file
echo.
echo  [錯誤] 缺少必要檔案 : !MISSING!
echo         請確認本啟動檔與程式檔位於同一資料夾。
echo.
pause
exit /b 1

:no_python
echo.
echo  [錯誤] 找不到可用的 Python
echo.
echo         請到 https://www.python.org/downloads/ 下載安裝
echo         Python 3.9 以上版本，安裝時請務必勾選
echo         Add Python to PATH
echo.
pause
exit /b 1

:install_fail
echo.
echo  [錯誤] 套件安裝失敗，請手動執行下列指令後再試
echo             %PY% %PY_ARGS% -m pip install fastapi uvicorn requests pydantic
echo.
pause
exit /b 1


REM ============================================================
REM  子程序
REM ============================================================

:try_python
rem %1 執行檔名稱, %2 額外參數
%~1 %~2 -c "import sys" >nul 2>&1
if errorlevel 1 exit /b 0
%~1 %~2 -c "import sys;sys.exit(0 if sys.version_info>=(3,9) else 1)" >nul 2>&1
if errorlevel 1 exit /b 0
set "PY=%~1"
set "PY_ARGS=%~2"
for /f "tokens=*" %%V in ('%~1 %~2 -c "import sys;print(sys.version.split()[0])" 2^>nul') do set "PY_VER=%%V"
exit /b 0

:is_port_busy
rem %1 埠號, 設定 PORT_BUSY = 1 代表有程式正在監聽
set "PORT_BUSY=0"
powershell -NoProfile -Command "try{$c=New-Object System.Net.Sockets.TcpClient;$c.Connect('127.0.0.1',%~1);$c.Close()}catch{exit 1}" >nul 2>&1
if not errorlevel 1 set "PORT_BUSY=1"
exit /b 0

:kill_port
rem %1 埠號, 結束所有監聽該埠號的程序
for /f "tokens=5" %%P in ('netstat -ano 2^>nul ^| findstr ":%~1 " ^| findstr "LISTENING"') do (
    echo         結束程序 PID %%P
    taskkill /PID %%P /T /F >nul 2>&1
)
exit /b 0