[CmdletBinding()]
param(
    [Parameter(Mandatory = $false, Position = 0)]
    [string] $Port
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Get-PlatformIoCommand {
    $pio = Get-Command pio -ErrorAction SilentlyContinue
    if ($pio) {
        return $pio.Source
    }

    $fallback = Join-Path $env:USERPROFILE ".platformio\penv\Scripts\platformio.exe"
    if (Test-Path -LiteralPath $fallback) {
        return $fallback
    }

    Write-Error "PlatformIO command was not found. Install the PlatformIO VS Code extension or add pio to PATH."
    exit 1
}

$pioCommand = Get-PlatformIoCommand

Push-Location $PSScriptRoot
try {
    if ([string]::IsNullOrWhiteSpace($Port)) {
        & $pioCommand device monitor --baud 115200
    } else {
        & $pioCommand device monitor --port $Port --baud 115200
    }

    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}
finally {
    Pop-Location
}

exit 0
