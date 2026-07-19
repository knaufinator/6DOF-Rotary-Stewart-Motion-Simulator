#!/usr/bin/env pwsh
# ESP-IDF Build Script for Mini-6DOF Controller

Write-Host "Setting up ESP-IDF environment..." -ForegroundColor Cyan

$possiblePaths = @(
    "C:\users\Chris\esp\v5.5\esp-idf",
    "C:\users\Chris\esp\v5.2\esp-idf",
    "$env:USERPROFILE\.espressif\esp-idf",
    "C:\esp\esp-idf"
)

$idfPath = $null
foreach ($path in $possiblePaths) {
    if (Test-Path "$path\export.ps1") {
        $idfPath = $path
        Write-Host "Found ESP-IDF at: $path" -ForegroundColor Green
        break
    }
}

if ($null -eq $idfPath) {
    Write-Host "ERROR: Could not find ESP-IDF installation!" -ForegroundColor Red
    exit 1
}

# Source the ESP-IDF environment
Write-Host "Loading ESP-IDF environment..." -ForegroundColor Cyan
. "$idfPath\export.ps1"

# Navigate to project directory
Set-Location $PSScriptRoot

# Set target to regular ESP32 (not S3)
Write-Host "Setting target to ESP32..." -ForegroundColor Cyan
idf.py set-target esp32

# Build the project
Write-Host "Building project..." -ForegroundColor Cyan
idf.py build

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild completed successfully!" -ForegroundColor Green
    Write-Host "Flash with: idf.py -p COM6 flash monitor" -ForegroundColor Yellow
} else {
    Write-Host "`nBuild failed with errors!" -ForegroundColor Red
    exit $LASTEXITCODE
}
