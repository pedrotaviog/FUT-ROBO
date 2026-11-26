@echo off
setlocal
chcp 65001 >nul

title FUT-ROBO - Inicializador da IHM
pushd "%~dp0"

where py >nul 2>nul
if %errorlevel% equ 0 (
    if /I "%~1"=="--check" (
        echo [OK] Python encontrado pelo comando: py -3
        popd
        exit /b 0
    )
    py -3 ".\app.py"
    set "IHM_EXIT_CODE=%errorlevel%"
    goto :finished
)

where python >nul 2>nul
if %errorlevel% equ 0 (
    if /I "%~1"=="--check" (
        echo [OK] Python encontrado pelo comando: python
        popd
        exit /b 0
    )
    python ".\app.py"
    set "IHM_EXIT_CODE=%errorlevel%"
    goto :finished
)

echo.
echo [ERRO] Python 3 não foi encontrado.
echo Instale o Python 3 e marque a opção "Add Python to PATH".
echo.
pause
popd
exit /b 1

:finished
if not "%IHM_EXIT_CODE%"=="0" (
    echo.
    echo [ERRO] A IHM foi encerrada com o código %IHM_EXIT_CODE%.
    echo Consulte as mensagens acima para identificar o problema.
    echo.
    pause
)

popd
exit /b %IHM_EXIT_CODE%
