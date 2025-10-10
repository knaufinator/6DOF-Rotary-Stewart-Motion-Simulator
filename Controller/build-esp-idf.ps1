#!/usr/bin/env pwsh
# ESP-IDF Build Script for Stewart Platform

Write-Host "Setting up ESP-IDF environment..." -ForegroundColor Cyan

# Try to find ESP-IDF installation
$possiblePaths = @(
    "C:\users\Chris\esp\v5.2\esp-idf",
    "$env:USERPROFILE\.espressif\esp-idf",
    "$env:USERPROFILE\.espressif\frameworks\esp-idf-v5.2",
    "$env:USERPROFILE\.espressif\frameworks\esp-idf-v5.2.0",
    "C:\esp\esp-idf",
    "C:\Espressif\frameworks\esp-idf-v5.2"
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
    Write-Host "Please install ESP-IDF or set IDF_PATH manually" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Checked paths:" -ForegroundColor Yellow
    foreach ($path in $possiblePaths) {
        Write-Host "  - $path" -ForegroundColor Gray
    }
    exit 1
}

# Source the ESP-IDF environment
Write-Host "Loading ESP-IDF environment..." -ForegroundColor Cyan
. "$idfPath\export.ps1"

# Navigate to project directory
Set-Location $PSScriptRoot

# Set target
Write-Host "Setting target to ESP32-S3..." -ForegroundColor Cyan
idf.py set-target esp32s3

# Build the project
Write-Host "Building project..." -ForegroundColor Cyan
idf.py build

if ($LASTEXITCODE -eq 0) {
    Write-Host "`nBuild completed successfully!" -ForegroundColor Green
} else {
    Write-Host "`nBuild failed with errors!" -ForegroundColor Red
    exit $LASTEXITCODE
}
