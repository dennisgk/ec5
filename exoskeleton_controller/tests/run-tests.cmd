@echo off
setlocal
cd /d "%~dp0.."
if not exist build mkdir build
call "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /WX /I main main\signal_processing.c tests\signal_processing_test.c /Fo:build\ /Fe:build\signal_processing_test.exe
if errorlevel 1 exit /b 1
build\signal_processing_test.exe
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /WX /I main tests\capture_shared_test.c /Fo:build\capture_shared_test.obj /Fe:build\capture_shared_test.exe
if errorlevel 1 exit /b 1
build\capture_shared_test.exe
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /WX /I main main\serial_frame.c tests\serial_frame_test.c /Fo:build\ /Fe:build\serial_frame_test.exe
if errorlevel 1 exit /b 1
build\serial_frame_test.exe
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /WX /I main tests\angle_test.c /Fo:build\angle_test.obj /Fe:build\angle_test.exe
if errorlevel 1 exit /b 1
build\angle_test.exe
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W3 /D_CRT_SECURE_NO_WARNINGS /DCJSON_HIDE_SYMBOLS /I C:\Espressif\frameworks\esp-idf-v5.5.2\components\json\cJSON /c C:\Espressif\frameworks\esp-idf-v5.5.2\components\json\cJSON\cJSON.c /Fo:build\cJSON.obj
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /WX /D_CRT_SECURE_NO_WARNINGS /DCJSON_HIDE_SYMBOLS /I main /I C:\Espressif\frameworks\esp-idf-v5.5.2\components\json\cJSON main\autoconfig_schema.c tests\autoconfig_test.c /Fo:build\ /Fe:build\autoconfig_test.exe /link build\cJSON.obj
if errorlevel 1 exit /b 1
build\autoconfig_test.exe
if errorlevel 1 exit /b 1
cl /nologo /TC /std:c11 /W4 /WX /DSERIAL_ASSEMBLY_MAX=1024 /I main main\serial_frame.c tests\serial_limits_test.c /Fo:build\ /Fe:build\serial_limits_test.exe
if errorlevel 1 exit /b 1
build\serial_limits_test.exe
exit /b %errorlevel%
