# Keil MDK batch build wrapper for VS Code.
#
# Runs UV4.exe in batch mode, then echoes the build log to stdout so the
# VS Code task problemMatcher can turn errors/warnings into editor problems
# (and so AI agents can read them).
#
# NOTE 1: keep this file pure ASCII. Windows PowerShell 5.1 decodes BOM-less
#         UTF-8 as GBK on zh-CN systems, which silently corrupts non-ASCII text.
# NOTE 2: UV4.exe is a GUI-subsystem binary. PowerShell's call operator (&)
#         does NOT wait for GUI apps, so we must use Start-Process -Wait to
#         block and to obtain a real exit code.

param(
    [ValidateSet('build', 'rebuild', 'clean')]
    [string]$Mode = 'build',

    [string]$UV4 = 'D:\Keil5\UV4\UV4.exe',

    [string]$Project = ''
)

$ErrorActionPreference = 'Continue'

# $PSScriptRoot = <workspaceFolder>\tools  ->  workspace root is its parent
$root = Split-Path -Parent $PSScriptRoot

if (-not $Project) {
    $Project = Join-Path $root 'Project\ROBOCON.uvprojx'
}

if (-not (Test-Path $UV4)) {
    Write-Host "ERROR: UV4.exe not found at: $UV4"
    exit 3
}

if (-not (Test-Path $Project)) {
    Write-Host "ERROR: project not found at: $Project"
    exit 3
}

$projDir = Split-Path -Parent $Project
$log = Join-Path $projDir 'build.log'

if (Test-Path $log) {
    Remove-Item $log -Force -ErrorAction SilentlyContinue
}

$flag = switch ($Mode) {
    'rebuild' { '-r' }
    'clean'   { '-c' }
    default   { '-b' }
}

Write-Host "== Keil $Mode =="
Write-Host "UV4    : $UV4"
Write-Host "Project: $Project"
Write-Host ""

$argList = @($flag, ('"{0}"' -f $Project), '-j0', '-o', ('"{0}"' -f $log))

$proc = Start-Process -FilePath $UV4 `
    -ArgumentList $argList `
    -WorkingDirectory $projDir `
    -Wait -PassThru -NoNewWindow

$code = $proc.ExitCode

# UV4 may still be flushing the log file right after the process exits.
for ($i = 0; $i -lt 40 -and -not (Test-Path $log); $i++) {
    Start-Sleep -Milliseconds 250
}

Write-Host ""
if (Test-Path $log) {
    Get-Content $log
}
else {
    Write-Host "WARNING: UV4 produced no log file."
}

Write-Host ""
Write-Host ("UV4 exit code: {0}   (0=ok, 1=warnings, 2=errors, 3=fatal)" -f $code)

exit $code
