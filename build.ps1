# MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com>
param(
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")]
    [string]$Config = "Release",

    # Use a Visual Studio solution generator by default (MSVC toolchain via VsDevCmd).
    # Pass "Ninja" for Ninja + single-config CMake layout (CMAKE_BUILD_TYPE at configure).
    [string]$Generator = "Visual Studio 17 2022",

    [string]$SourceDir = "",
    [string]$BuildDir = "",
    [string]$VcpkgToolchain = "",
    [string]$VsDevCmd = "",
    [string]$CmakeExe = "",
    [string]$WinDeployQtExe = "",

    [switch]$Clean,
    [switch]$SkipDeploy,
    [switch]$SkipConfigure,

    # Copy the deployed app to a local folder and add a Start-menu shortcut (pin that copy to the
    # taskbar; the build folder is rewritten on every build and may be on a network share).
    [switch]$Install,
    [string]$InstallDir = ""
)

$ErrorActionPreference = "Stop"

function Resolve-PathOrThrow {
    param(
        [Parameter(Mandatory = $true)]
        [string]$PathValue,
        [Parameter(Mandatory = $true)]
        [string]$Name
    )
    $resolved = Resolve-Path -LiteralPath $PathValue -ErrorAction SilentlyContinue
    if (-not $resolved) {
        throw "$Name was not found: $PathValue"
    }
    return $resolved.Path
}

function Find-VsDevCmdBat {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "VsDevCmd")
    }
    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path -LiteralPath $vswhere) {
        $install = & $vswhere `
            -latest `
            -products * `
            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
            -property installationPath 2>$null
        if (-not [string]::IsNullOrWhiteSpace($install)) {
            $bat = Join-Path $install "Common7\Tools\VsDevCmd.bat"
            if (Test-Path -LiteralPath $bat) {
                return $bat
            }
        }
    }
    foreach ($edition in @("Enterprise", "Professional", "Community", "Preview")) {
        $candidate = "C:/Program Files/Microsoft Visual Studio/2022/$edition/Common7/Tools/VsDevCmd.bat"
        if (Test-Path -LiteralPath $candidate) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw 'Could not locate VsDevCmd.bat. Install the "Desktop development with C++" workload or pass -VsDevCmd "...\VsDevCmd.bat".'
}

function Find-CMakeExe {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "CMake")
    }
    $qtCMake = "C:/Qt/Tools/CMake_64/bin/cmake.exe"
    if (Test-Path -LiteralPath $qtCMake) {
        return (Resolve-Path -LiteralPath $qtCMake).Path
    }
    $fromPath = Get-Command cmake -ErrorAction SilentlyContinue
    if ($fromPath) {
        return $fromPath.Source
    }
    throw "cmake.exe was not found. Install CMake, add it to PATH, or pass -CmakeExe."
}

function Find-VcpkgToolchain {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "vcpkg toolchain")
    }
    $defaultMachine = Join-Path ([Environment]::GetFolderPath([Environment+SpecialFolder]::UserProfile)) "vcpkg/scripts/buildsystems/vcpkg.cmake"
    if (Test-Path -LiteralPath $defaultMachine) {
        return (Resolve-Path -LiteralPath $defaultMachine).Path
    }
    throw 'vcpkg.cmake not found. Pass -VcpkgToolchain "...\vcpkg\scripts\buildsystems\vcpkg.cmake" or set up vcpkg in %USERPROFILE%\vcpkg.'
}

function Find-WinDeployQt {
    param([string]$UserPath)
    if (-not [string]::IsNullOrWhiteSpace($UserPath)) {
        return (Resolve-PathOrThrow -PathValue $UserPath -Name "windeployqt")
    }
    $candidate = Join-Path ([Environment]::GetFolderPath([Environment+SpecialFolder]::UserProfile)) "vcpkg/installed/x64-windows/tools/Qt6/bin/windeployqt.exe"
    if (Test-Path -LiteralPath $candidate) {
        return (Resolve-Path -LiteralPath $candidate).Path
    }
    $fromPath = Get-Command windeployqt -ErrorAction SilentlyContinue
    if ($fromPath) {
        return $fromPath.Source
    }
    throw "windeployqt.exe was not found. Pass -WinDeployQtExe or add Qt's bin folder to PATH."
}

function Resolve-AppExe {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ResolvedBuildDir,
        [Parameter(Mandatory = $true)]
        [string]$BuildConfiguration
    )
    $configDir = Join-Path $ResolvedBuildDir $BuildConfiguration
    $roots = @(
        (Join-Path $ResolvedBuildDir "ECamWorkbench.exe"),
        (Join-Path $configDir "ECamWorkbench.exe")
    )
    foreach ($p in $roots) {
        if (Test-Path -LiteralPath $p) {
            return (Resolve-Path -LiteralPath $p).Path
        }
    }
    throw "Built executable ECamWorkbench.exe not found under '$ResolvedBuildDir' (looked root and '$BuildConfiguration' subfolder)."
}

if ([string]::IsNullOrWhiteSpace($SourceDir)) {
    $SourceDir = $PSScriptRoot
}
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $PSScriptRoot "ProjectData/build"
}

$VsDevCmd = Find-VsDevCmdBat -UserPath $VsDevCmd
$CmakeExe = Find-CMakeExe -UserPath $CmakeExe
$VcpkgToolchain = Find-VcpkgToolchain -UserPath $VcpkgToolchain

$SourceDir = Resolve-PathOrThrow -PathValue $SourceDir -Name "SourceDir (ECamUI)"
if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Host "Removing build directory: $BuildDir"
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}
if (-not (Test-Path -LiteralPath $BuildDir)) {
    New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null
}
$ResolvedBuildDir = (Resolve-Path -LiteralPath $BuildDir).Path

Write-Host "Building the Electronic CAM Workbench (ECamWorkbench.exe)..."
Write-Host "  Source: $SourceDir"
Write-Host "  Build : $ResolvedBuildDir"
Write-Host "  Config: $Config"
Write-Host "  Gen   : $Generator"

# VsDevCmd must be its own `call ... &&` segment; do not append cmake on the same line (parse_cmd rejects it).
$configureArgs = @(
    "`"$CmakeExe`"",
    "-S", "`"$SourceDir`"",
    "-B", "`"$ResolvedBuildDir`"",
    "-G", "`"$Generator`"",
    "-DCMAKE_TOOLCHAIN_FILE=`"$VcpkgToolchain`""
)
if ($Generator -match "Ninja") {
    $configureArgs += "-DCMAKE_BUILD_TYPE=$Config"
}

$buildArgs = @(
    "`"$CmakeExe`"",
    "--build", "`"$ResolvedBuildDir`"",
    "--target", "ECamWorkbench", "test_camlaws", "test_runtime", "test_ui", "board_smoke", "board_ui_jog", "board_ui_plc", "test_eip_protocol", "scope_hw", "help_images",
    "--config", $Config
)

$cmdParts = New-Object System.Collections.Generic.List[string]
$cmdParts.Add("call `"$VsDevCmd`" -arch=x64 -host_arch=x64")
if (-not $SkipConfigure) {
    $cmdParts.Add(($configureArgs -join " "))
}
$cmdParts.Add(($buildArgs -join " "))
$cmd = ($cmdParts -join " && ")

Write-Host "Running inside VS dev shell (configure if enabled, then build)..."
cmd.exe /d /s /c $cmd
if ($LASTEXITCODE -ne 0) {
    throw "Build failed with exit code $LASTEXITCODE."
}

Write-Host "Build step completed successfully."

if (-not $SkipDeploy) {
    $WinDeployQtExe = Find-WinDeployQt -UserPath $WinDeployQtExe
    $appExe = Resolve-AppExe -ResolvedBuildDir $ResolvedBuildDir -BuildConfiguration $Config
    Write-Host "Deploying Qt runtime dependencies to: $(Split-Path -Parent $appExe)"
    $deployMode = if ($Config -eq "Debug") { "--debug" } else { "--release" }
    # Run inside the VS dev shell so VCINSTALLDIR is set; otherwise --compiler-runtime
    # silently skips the MSVC runtime DLLs and the app only runs where VC++ is installed.
    cmd.exe /d /s /c "call `"$VsDevCmd`" -arch=x64 -host_arch=x64 >nul && `"$WinDeployQtExe`" $deployMode --force --compiler-runtime `"$appExe`""
    if ($LASTEXITCODE -ne 0) {
        throw "windeployqt failed with exit code $LASTEXITCODE."
    }
    Write-Host "windeployqt completed."

    # Qt 6's --compiler-runtime only drops vc_redist.x64.exe. Also copy the CRT DLLs next to the
    # exe (app-local deployment) so the folder runs on a PC without the VC++ redistributable.
    $vsRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $VsDevCmd))
    $crt = Get-ChildItem -Path (Join-Path $vsRoot "VC\Redist\MSVC\*\x64\Microsoft.VC*.CRT") -Directory -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending | Select-Object -First 1
    if ($crt) {
        Copy-Item -Path (Join-Path $crt.FullName "*.dll") -Destination (Split-Path -Parent $appExe) -Force
        Write-Host "Copied MSVC runtime DLLs from $($crt.FullName)"
    } else {
        Write-Warning "MSVC runtime DLLs not found under $vsRoot\VC\Redist; ship vc_redist.x64.exe instead."
    }
} else {
    Write-Host "Skipping windeployqt (-SkipDeploy)."
}

if ($Install) {
    if ($SkipDeploy) {
        throw "-Install needs the deployed build; do not combine it with -SkipDeploy."
    }
    if (-not $appExe) {
        $appExe = Resolve-AppExe -ResolvedBuildDir $ResolvedBuildDir -BuildConfiguration $Config
    }
    if ([string]::IsNullOrWhiteSpace($InstallDir)) {
        $InstallDir = Join-Path $env:LOCALAPPDATA "Programs\ECamWorkbench"
    }
    $installedExe = Join-Path $InstallDir "ECamWorkbench.exe"
    $running = Get-Process -Name ECamWorkbench -ErrorAction SilentlyContinue |
        Where-Object { $_.Path -and ($_.Path -ieq $installedExe) }
    if ($running) {
        throw "The Electronic CAM Workbench is running from $InstallDir; close it and run the install again."
    }
    New-Item -ItemType Directory -Force -Path $InstallDir | Out-Null
    # Mirror the deployed folder (removes files a newer build no longer ships); skip debug symbols,
    # the test programs and the redistributable installer (the CRT DLLs are already app-local).
    & robocopy (Split-Path -Parent $appExe) $InstallDir /MIR /XF *.pdb *.ilk test_camlaws.exe test_runtime.exe test_ui.exe board_smoke.exe board_ui_jog.exe scope_hw.exe help_images.exe vc_redist.x64.exe /NFL /NDL /NJH /NJS /NP | Out-Null
    if ($LASTEXITCODE -ge 8) {
        throw "Copying to $InstallDir failed (robocopy exit code $LASTEXITCODE)."
    }
    $global:LASTEXITCODE = 0
    # /MIR leaves excluded names alone in the destination, so drop any left by an older install.
    foreach ($extra in @("test_camlaws.exe", "test_runtime.exe", "test_ui.exe", "board_smoke.exe", "board_ui_jog.exe", "scope_hw.exe", "help_images.exe", "vc_redist.x64.exe")) {
        Remove-Item -LiteralPath (Join-Path $InstallDir $extra) -Force -ErrorAction SilentlyContinue
    }
    $programs = [Environment]::GetFolderPath([Environment+SpecialFolder]::Programs)
    # The shortcut from before the rename.
    Remove-Item -LiteralPath (Join-Path $programs "ECamUI.lnk") -Force -ErrorAction SilentlyContinue
    $startMenu = Join-Path $programs "Electronic CAM Workbench.lnk"
    $shell = New-Object -ComObject WScript.Shell
    $lnk = $shell.CreateShortcut($startMenu)
    $lnk.TargetPath = $installedExe
    $lnk.WorkingDirectory = $InstallDir
    $lnk.IconLocation = "$installedExe,0"
    $lnk.Description = "Electronic CAM Workbench: electronic cam profiles for the ClearCore"
    $lnk.Save()
    Write-Host "Installed to $InstallDir"
    Write-Host "Start-menu shortcut: $startMenu (pin this copy to the taskbar)"
}

Write-Host "Done."
