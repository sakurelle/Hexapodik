param(
    [string]$Drive
)

$ErrorActionPreference = "Stop"
$uf2 = Join-Path $PSScriptRoot "build\RP2040RobotController.uf2"

if (-not (Test-Path -LiteralPath $uf2)) {
    Write-Error "UF2 not found: $uf2. Build the project first."
}

if ($Drive) {
    $root = $Drive
    if ($root.Length -eq 1) { $root = "$root`:" }
    $root = "$root\"
    if (-not (Test-Path -LiteralPath $root)) {
        Write-Error "Drive not found: $root"
    }
} else {
    $vol = Get-Volume | Where-Object { $_.FileSystemLabel -eq "RPI-RP2" } | Select-Object -First 1
    if (-not $vol -or -not $vol.DriveLetter) {
        Write-Error "RPI-RP2 drive not found. Hold BOOTSEL while connecting the board, or pass a drive like .\flash.ps1 E:"
    }
    $root = "$($vol.DriveLetter):\"
}

Copy-Item -LiteralPath $uf2 -Destination (Join-Path $root "RP2040RobotController.uf2") -Force
Write-Host "Copied $uf2 to $root"
