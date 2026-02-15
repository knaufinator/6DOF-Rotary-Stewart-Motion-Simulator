@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cl /LD /I ..\src plugin_sine_demo.c /Fe:plugin_sine_demo.dll
cl /LD /I ..\src plugin_assetto_corsa.c /Fe:plugin_assetto_corsa.dll
cl /LD /I ..\src plugin_simtools_udp.c /Fe:plugin_simtools_udp.dll ws2_32.lib
cl /LD /I ..\src plugin_test_signal.c /Fe:plugin_test_signal.dll
del *.obj *.exp *.lib 2>nul

REM Copy to build output so the app finds them at runtime
if not exist "..\build\Release\plugins" mkdir "..\build\Release\plugins"
copy /Y *.dll "..\build\Release\plugins\" >nul 2>&1
echo Done. DLLs copied to build\Release\plugins\
