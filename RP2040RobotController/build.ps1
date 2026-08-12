param(
    [string]$BuildDir = "build"
)

$ErrorActionPreference = "Stop"

$DefaultSdkPath = Join-Path $env:USERPROFILE ".pico-sdk\sdk\2.3.0"
$DefaultToolchainPath = Join-Path $env:USERPROFILE ".pico-sdk\toolchain\15_2_Rel1"
$DefaultPicoCmake = Join-Path $env:USERPROFILE ".pico-sdk\cmake\v4.3.4\bin\cmake.exe"
$DefaultPicoNinja = Join-Path $env:USERPROFILE ".pico-sdk\ninja\v1.13.2\ninja.exe"
$DefaultCmake = Join-Path $env:USERPROFILE ".platformio\packages\tool-cmake\bin\cmake.exe"
$DefaultNinja = Join-Path $env:USERPROFILE ".platformio\packages\tool-ninja\ninja.exe"

if (-not $env:PICO_SDK_PATH -and (Test-Path -LiteralPath $DefaultSdkPath)) {
    $env:PICO_SDK_PATH = $DefaultSdkPath
}

if (-not $env:PICO_TOOLCHAIN_PATH -and (Test-Path -LiteralPath $DefaultToolchainPath)) {
    $env:PICO_TOOLCHAIN_PATH = $DefaultToolchainPath
}

if (-not $env:PICO_SDK_PATH) {
    Write-Error "PICO_SDK_PATH is not set and $DefaultSdkPath was not found."
}

if (-not (Test-Path -LiteralPath $env:PICO_SDK_PATH)) {
    Write-Error "PICO_SDK_PATH does not exist: $env:PICO_SDK_PATH"
}

$Cmake = (Get-Command cmake -ErrorAction SilentlyContinue).Source
if (-not $Cmake -and (Test-Path -LiteralPath $DefaultPicoCmake)) {
    $Cmake = $DefaultPicoCmake
}
if (-not $Cmake -and (Test-Path -LiteralPath $DefaultCmake)) {
    $Cmake = $DefaultCmake
}
if (-not $Cmake) {
    Write-Error "cmake was not found on PATH or at $DefaultCmake."
}

$Ninja = (Get-Command ninja -ErrorAction SilentlyContinue).Source
if (-not $Ninja -and (Test-Path -LiteralPath $DefaultPicoNinja)) {
    $Ninja = $DefaultPicoNinja
}
if (-not $Ninja -and (Test-Path -LiteralPath $DefaultNinja)) {
    $Ninja = $DefaultNinja
}
if (-not $Ninja) {
    Write-Error "ninja was not found on PATH, at $DefaultPicoNinja, or at $DefaultNinja."
}

$env:Path = "$(Join-Path $DefaultToolchainPath 'bin');$(Split-Path -Parent $Cmake);$(Split-Path -Parent $Ninja);$env:Path"

New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
& $Cmake -S . -B $BuildDir -G Ninja -DCMAKE_MAKE_PROGRAM="$Ninja" -DPICO_SDK_PATH="$env:PICO_SDK_PATH" -DPICO_BOARD=pico
& $Cmake --build $BuildDir
