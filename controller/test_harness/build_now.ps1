# Quick build script — Test Harness Firmware (ESP32-S3)
# Usage: .\build_now.ps1          (build only)
#        .\build_now.ps1 -Flash    (build + flash)
#        .\build_now.ps1 -Port COM4 -Flash
param([switch]$Flash, [string]$Port = "")

. 'C:\users\Chris\esp\v5.5\esp-idf\export.ps1'
Set-Location $PSScriptRoot
idf.py build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
if ($Flash) {
    if ($Port -ne "") { idf.py -p $Port flash } else { idf.py flash }
}
