# Wrapper that runs the pyOCD target dump with the dedicated venv interpreter.
#
# Keep this file pure ASCII (Windows PowerShell 5.1 + GBK default codepage).

param(
    [string]$Target = 'stm32f407vg',
    [string]$Uid = '',
    [switch]$NoResume,
    [switch]$NoHalt,

    # Dedicated venv created for pyOCD, so it does not pollute the DSH runtime.
    [string]$PyOcdPython = 'C:\Users\xszx1\.dsh\tools\pyocd\Scripts\python.exe'
)

$script = Join-Path $PSScriptRoot 'dump_target.py'

if (-not (Test-Path $PyOcdPython)) {
    Write-Host "ERROR: pyOCD venv python not found at: $PyOcdPython"
    Write-Host "       Recreate it with: python -m venv C:\Users\xszx1\.dsh\tools\pyocd"
    exit 2
}

if (-not (Test-Path $script)) {
    Write-Host "ERROR: dump script not found at: $script"
    exit 2
}

$pyArgs = @($script, '--target', $Target)
if ($Uid)      { $pyArgs += @('--uid', $Uid) }
if ($NoResume) { $pyArgs += '--no-resume' }
if ($NoHalt)   { $pyArgs += '--no-halt' }

& $PyOcdPython @pyArgs
exit $LASTEXITCODE
