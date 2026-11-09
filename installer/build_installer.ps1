# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
#
# Builds the Windows installer: dist\ECamWorkbench-<version>-setup.exe (Inno Setup 6).
#   1. the Workbench (..\build.ps1: configure, build, windeployqt with the MSVC runtime)
#   2. the firmware (..\Firmware\build.ps1 -Config Release)
#   3. stages the app (no test programs), the firmware with its flasher and the EDS, the samples and the
#      documents in ProjectData\build\installer_stage
#   4. ISCC installer\ECamWorkbench.iss with the version from src\Brand.h
#
#   .\installer\build_installer.ps1              # everything
#   .\installer\build_installer.ps1 -SkipBuild   # stage and package what is built already

param(
    [switch]$SkipBuild,
    [string]$Iscc = ""
)
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot

$brand = Get-Content -Raw (Join-Path $Root "src\Brand.h")
if ($brand -notmatch '#define ECAM_PRODUCT_VERSION "([0-9.]+)"') { throw "ECAM_PRODUCT_VERSION not found in src\Brand.h" }
$Version = $Matches[1]
$config = Get-Content -Raw (Join-Path $Root "Firmware\ECamFirmware\ECamConfig.h")
if ($config -notmatch '#define ECAM_FIRMWARE_VERSION "([0-9.]+)"') { throw "ECAM_FIRMWARE_VERSION not found" }
if ($Matches[1] -ne $Version) { throw "The app ($Version) and the firmware ($($Matches[1])) versions differ" }
Write-Host "Electronic CAM Workbench $Version"

if (-not $SkipBuild) {
    & (Join-Path $Root "build.ps1")
    & (Join-Path $Root "Firmware\build.ps1") -Config Release
}

$release = Join-Path $Root "ProjectData\build\Release"
$fwDir = Join-Path $Root "Firmware\ECamFirmware\Release"
foreach ($f in @((Join-Path $release "ECamWorkbench.exe"), (Join-Path $release "Qt6Core.dll"), (Join-Path $fwDir "ECam_Firmware.bin"))) {
    if (-not (Test-Path -LiteralPath $f)) { throw "Missing $f (build first, without -SkipBuild)" }
}

$stage = Join-Path $Root "ProjectData\build\installer_stage"
if (Test-Path -LiteralPath $stage) { Remove-Item -LiteralPath $stage -Recurse -Force }
$app = New-Item -ItemType Directory -Force (Join-Path $stage "app")
$fw = New-Item -ItemType Directory -Force (Join-Path $stage "firmware")
$samples = New-Item -ItemType Directory -Force (Join-Path $stage "samples")
$docs = New-Item -ItemType Directory -Force (Join-Path $stage "docs")

# The app: the deployed Release folder without the test and bench programs and build leftovers.
# Programs are listed by what ships, so a new test or bench program stays out by itself.
$shipExe = @("ECamWorkbench.exe", "vc_redist.x64.exe")
Get-ChildItem -LiteralPath $release -File | Where-Object {
    ($_.Extension -ne ".exe" -or $shipExe -contains $_.Name) -and $_.Extension -notin @(".lib", ".pdb", ".exp", ".ilk")
} | Copy-Item -Destination $app
Get-ChildItem -LiteralPath $release -Directory | Where-Object { $_.Name -notlike "*.dir" -and $_.Name -ne "installer_stage" } |
    Copy-Item -Destination $app -Recurse

# Firmware, flasher, EDS.
Copy-Item (Join-Path $fwDir "ECam_Firmware.bin") $fw
if (Test-Path (Join-Path $fwDir "ECam_Firmware.uf2")) { Copy-Item (Join-Path $fwDir "ECam_Firmware.uf2") $fw }
Copy-Item (Join-Path $Root "tools\flash_clearcore.cmd") $fw
New-Item -ItemType Directory -Force (Join-Path $fw "bossac") | Out-Null
Copy-Item (Join-Path $Root "tools\bossac\bossac.exe") (Join-Path $fw "bossac")
Copy-Item (Join-Path $Root "tools\bossac\LICENSE") (Join-Path $fw "bossac\LICENSE.txt")
Copy-Item (Join-Path $PSScriptRoot "flash_firmware.cmd") $fw
New-Item -ItemType Directory -Force (Join-Path $fw "eds") | Out-Null
Copy-Item (Join-Path $Root "Firmware\eds\ECam_ClearCore.eds") (Join-Path $fw "eds")
Copy-Item (Join-Path $Root "Firmware\PROTOCOL.md") $fw

# Samples: the tracked ones only, as committed (an edit in the working tree, such as a job saved over a sample, stays out).
Push-Location $Root
try {
    $tracked = git ls-files samples | Where-Object { $_ -like "*.ecam" -or $_ -like "*.csv" }
    foreach ($s in $tracked) {
        $out = Join-Path $samples (Split-Path $s -Leaf)
        & cmd /c "git show HEAD:$s > `"$out`""
        if ($LASTEXITCODE -ne 0) { throw "git show HEAD:$s failed" }
    }
} finally { Pop-Location }

# Documents and licences.
foreach ($d in @("README.md", "LICENSE", "THIRD_PARTY_NOTICES.md", "Firmware\PROTOCOL.md", "QuickEIP\LICENSE")) {
    $src = Join-Path $Root $d
    $name = if ($d -eq "QuickEIP\LICENSE") { "QuickEIP_LICENSE.txt" } else { Split-Path -Leaf $d }
    Copy-Item $src (Join-Path $docs $name)
}

if ([string]::IsNullOrWhiteSpace($Iscc)) {
    $Iscc = @("${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe", "$env:ProgramFiles\Inno Setup 6\ISCC.exe") |
        Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (-not $Iscc) { throw "ISCC.exe (Inno Setup 6) not found; pass -Iscc" }
New-Item -ItemType Directory -Force (Join-Path $Root "dist") | Out-Null
& $Iscc "/DAppVersion=$Version" "/DStageDir=$stage" (Join-Path $PSScriptRoot "ECamWorkbench.iss")
if ($LASTEXITCODE -ne 0) { throw "ISCC failed with exit code $LASTEXITCODE" }
Get-Item (Join-Path $Root "dist\ECamWorkbench-$Version-setup.exe") | Format-Table Name, Length -AutoSize
