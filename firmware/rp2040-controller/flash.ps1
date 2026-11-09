param(
    [string]$BuildDir = "build"
)

if (-not (Test-Path "$BuildDir/RP2040RobotController.uf2")) {
    throw "UF2 not found. Build with a configured Pico SDK first."
}

Write-Host "Copy $BuildDir/RP2040RobotController.uf2 to the RP2040 BOOTSEL drive."
