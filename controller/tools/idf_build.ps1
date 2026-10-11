param(
    [Parameter(Mandatory=$true)][string]$ProjDir,   # ESP-IDF project dir (top-level CMakeLists.txt)
    [string]$Action    = "build",                     # build | flash | "-p COM10 flash monitor" ...
    [string]$LogPath   = "$env:TEMP\idf_build.log",
    [string]$IdfPath   = $env:IDF_PATH,               # e.g. C:\Users\<you>\esp\v5.5\esp-idf
    [string]$ToolsPath = $env:IDF_TOOLS_PATH,         # default C:\Users\<you>\.espressif
    [string]$Venv      = "",                          # auto-detected under $ToolsPath\python_env
    [switch]$Clean                                    # wipe CMakeCache to force a reconfigure
)
# Headless ESP-IDF build / flash that avoids three Windows traps:
#  1. IDF's export.sh refuses Git Bash (MSYS): run this from PowerShell.
#  2. export.ps1 looks for a venv named after the *default* python (idf5.5_py3.12_env)
#     while the installed venv is for another version (idf5.5_py3.11_env) and aborts with
#     "Python environment is missing". This script never calls export.ps1: it finds an
#     existing venv itself and sets the tool environment through idf_tools.py.
#  3. Harmless CMake stderr lines abort a PowerShell build under $ErrorActionPreference='Stop'.
#     idf.py runs via Start-Process with separate stdout/stderr files and the real exit code.
# Prints IDF_EXIT=0 on success. Verify by the artifact: a fresh build\<project>.bin and
# "Project build complete" in the log. A stale .bin looks exactly like success.
$ErrorActionPreference = 'Stop'
if (-not $ToolsPath) { $ToolsPath = Join-Path $env:USERPROFILE '.espressif' }
if (-not $IdfPath) {
    $cands = Get-ChildItem (Join-Path $env:USERPROFILE 'esp') -Directory -ErrorAction SilentlyContinue |
             ForEach-Object { Join-Path $_.FullName 'esp-idf' } | Where-Object { Test-Path (Join-Path $_ 'tools\idf.py') }
    if (-not $cands) { throw "IDF_PATH not set and no esp-idf found under $env:USERPROFILE\esp" }
    $IdfPath = ($cands | Sort-Object)[-1]
}
# IDF version: tools/cmake/version.cmake holds IDF_VERSION_MAJOR / MINOR; fall back to the
# checkout folder name (e.g. ...\v5.5\esp-idf), then to "any IDF venv present".
$idfVer = ''
$vc = Join-Path $IdfPath 'tools\cmake\version.cmake'
if (Test-Path $vc) {
    $t = Get-Content $vc -Raw
    if ($t -match 'IDF_VERSION_MAJOR\s+(\d+)' ) { $maj = $matches[1] }
    if ($t -match 'IDF_VERSION_MINOR\s+(\d+)' ) { $min = $matches[1] }
    if ($maj -and $min) { $idfVer = "$maj.$min" }
}
if (-not $idfVer -and $IdfPath -match '[\\/]v?(\d+\.\d+)[\\/]') { $idfVer = $matches[1] }
if (-not $Venv) {
    $vdir = Join-Path $ToolsPath 'python_env'
    $pattern = if ($idfVer) { "idf${idfVer}_py*_env" } else { "idf*_py*_env" }
    $vs = Get-ChildItem $vdir -Directory -ErrorAction SilentlyContinue | Where-Object { $_.Name -like $pattern }
    if (-not $vs) { throw "No IDF $idfVer venv under $vdir. Run `"$IdfPath\install.ps1 esp32s3`" once to create one." }
    $Venv = ($vs | Sort-Object Name)[-1].FullName
}
$py = Join-Path $Venv 'Scripts\python.exe'
if (-not (Test-Path $py)) { throw "venv python not found: $py" }
"IDF_PATH=$IdfPath (version $idfVer)"; "VENV=$Venv"
$env:IDF_PATH = $IdfPath; $env:IDF_PYTHON_ENV_PATH = $Venv; $env:IDF_TOOLS_PATH = $ToolsPath

$kv = & $py "$IdfPath\tools\idf_tools.py" export --format key-value
foreach ($line in $kv) {
    if ($line -match '^\s*([A-Za-z_][A-Za-z0-9_]*)=(.*)$') {
        $n = $matches[1]; $v = $matches[2]
        $v = [regex]::Replace($v, '%([A-Za-z_][A-Za-z0-9_]*)%', { param($m) [Environment]::GetEnvironmentVariable($m.Groups[1].Value) })
        Set-Item -Path "Env:$n" -Value $v
    }
}
$env:PATH = "$Venv\Scripts;$IdfPath\tools;$env:PATH"

Set-Location $ProjDir
if ($Clean) {
    Remove-Item build\CMakeCache.txt -Force -ErrorAction SilentlyContinue
    Remove-Item build\CMakeFiles -Recurse -Force -ErrorAction SilentlyContinue
}
$out = $LogPath; $err = "$LogPath.err"
$argList = @("$IdfPath\tools\idf.py") + ($Action -split '\s+')
$ErrorActionPreference = 'Continue'
$p = Start-Process -FilePath $py -ArgumentList $argList -NoNewWindow -Wait -PassThru `
        -RedirectStandardOutput $out -RedirectStandardError $err
"IDF_EXIT=$($p.ExitCode)"
if ($p.ExitCode -ne 0) { "---- stderr tail ----"; Get-Content $err -Tail 25 -ErrorAction SilentlyContinue }
