# One-shot ESPNetLink build (and optional flash) with the exact environment
# the firmware needs. Run from a NATIVE PowerShell window:
#
#   .\build.ps1                    # build only
#   .\build.ps1 -Flash COM9        # build + flash the FRESH build at 460800
#   .\build.ps1 -Clean             # idf.py fullclean first
#
# Why this script exists (bench 2026-09-08): build/ keeps one versioned
# binary per build (espnetlink-fw_v122-41-g<hash>[-dirty].bin) and two
# different commits can share the "41" - a dongle got flashed with a July
# test build picked by hand from that folder and looked like a broken
# WiCAN integration (no WiFi-modem API, LTE never started). Flash ONLY
# through @flash_args, which names the binary this build just produced.
#
#  - Git Bash / MSYS: idf.py prints "MSys/Mingw is no longer supported",
#    builds NOTHING and still exits 0 - this script refuses to run there.
#  - Flashing over the dongle's USB-C: the CH343 bridge port (auto-reset).

param(
    [string]$Flash = "",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

if ($env:MSYSTEM) {
    Write-Error "Run this from native PowerShell, not Git Bash/MSYS (MSYSTEM=$env:MSYSTEM): idf.py builds nothing there while still exiting 0."
}

# ---- ESP-IDF v6.0.2 environment -------------------------------------------
$env:IDF_PATH                     = 'C:\esp\v6.0.2\esp-idf'
$env:IDF_TOOLS_PATH               = 'C:\Espressif'
$env:IDF_PYTHON_ENV_PATH          = 'C:\Espressif\tools\python\v6.0.2\venv'
$env:IDF_PYTHON_CHECK_CONSTRAINTS = 'no'
$env:ESP_IDF_VERSION              = '6.0.2'
$env:ESP_ROM_ELF_DIR              = 'C:\Espressif\tools\esp-rom-elfs\20241011\'
$env:PATH = 'C:\Espressif\tools\python\v6.0.2\venv\Scripts;' +
            'C:\Espressif\tools\ninja\1.12.1;C:\Espressif\tools\cmake\4.0.3\bin;' +
            'C:\Espressif\tools\xtensa-esp-elf\esp-15.2.0_20251204\xtensa-esp-elf\bin;' +
            'C:\Espressif\tools\ccache\4.12.1\ccache-4.12.1-windows-x86_64;' +
            'C:\Espressif\tools\idf-git\2.34.2\cmd;' + $env:PATH

Set-Location $PSScriptRoot

if ($Clean) {
    python "$env:IDF_PATH\tools\idf.py" fullclean
    if ($LASTEXITCODE -ne 0) { Write-Error "fullclean failed" }
}

python "$env:IDF_PATH\tools\idf.py" build
if ($LASTEXITCODE -ne 0) { Write-Error "build failed (exit $LASTEXITCODE)" }

# the binary THIS build produced is the one flash_args names
$appLine = (Get-Content (Join-Path $PSScriptRoot 'build\flash_args')) | Where-Object { $_ -match 'espnetlink-fw_.*\.bin' }
$appBin  = ($appLine -split '\s+')[-1]
$bin     = Join-Path $PSScriptRoot ("build\" + $appBin)
$age     = (Get-Date) - (Get-Item $bin).LastWriteTime
"OK: build\$appBin ($([math]::Round((Get-Item $bin).Length/1MB,2)) MB, written $([math]::Round($age.TotalMinutes,1)) min ago)"
if ($age.TotalMinutes -gt 30) {
    Write-Warning "the binary is older than this run should allow - the build may not have rebuilt anything it should have"
}
$stale = Get-ChildItem (Join-Path $PSScriptRoot 'build') -Filter 'espnetlink-fw_*.bin' | Where-Object { $_.Name -ne $appBin }
if ($stale) {
    Write-Warning ("build\ also holds " + $stale.Count + " OLDER versioned binaries (" + (($stale | Select-Object -First 3 | ForEach-Object Name) -join ', ') + " ...): never flash those by hand, use -Flash or @flash_args")
}

if ($Flash) {
    Set-Location (Join-Path $PSScriptRoot 'build')
    python -m esptool --chip esp32s3 -p $Flash -b 460800 --before default-reset --after hard-reset write-flash "@flash_args"
    if ($LASTEXITCODE -ne 0) { Write-Error "flash failed on $Flash" }
    "OK: flashed $appBin on $Flash"
}
