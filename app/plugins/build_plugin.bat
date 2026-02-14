@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>&1
cl /LD /I ..\src plugin_sine_demo.c /Fe:plugin_sine_demo.dll
del *.obj *.exp 2>nul
echo Done.
