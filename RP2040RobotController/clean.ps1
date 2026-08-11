param(
    [string]$BuildDir = "build"
)

$ErrorActionPreference = "Stop"
$target = Resolve-Path -LiteralPath "." | Select-Object -ExpandProperty Path
$buildPath = Join-Path $target $BuildDir

if (Test-Path -LiteralPath $buildPath) {
    Remove-Item -LiteralPath $buildPath -Recurse -Force
}
