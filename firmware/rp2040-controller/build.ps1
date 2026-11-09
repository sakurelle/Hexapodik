param(
    [string]$BuildDir = "build",
    [string]$PicoSdkPath = "$env:USERPROFILE\.pico-sdk\sdk\2.3.0"
)

$cmake = "$env:USERPROFILE\.pico-sdk\cmake\v4.3.4\bin\cmake.exe"
if (-not (Test-Path $cmake)) {
    $cmake = "cmake"
}

$env:PICO_SDK_PATH = $PicoSdkPath
$env:Path = "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin;$env:USERPROFILE\.pico-sdk\ninja\v1.13.2;$env:Path"

& $cmake -S . -B $BuildDir -G Ninja -DPICO_BOARD=pico `
    "-Dpioasm_DIR=$env:USERPROFILE\.pico-sdk\tools\2.3.0\pioasm" `
    "-Dpicotool_DIR=$env:USERPROFILE\.pico-sdk\picotool\2.3.0\picotool"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $cmake --build $BuildDir --target RP2040RobotController
