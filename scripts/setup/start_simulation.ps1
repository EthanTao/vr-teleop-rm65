<# Compatibility CLI: keep legacy PowerShell parameters on the Python Servo chain. #>
[CmdletBinding()]
param(
    [ValidateSet('manual', 'pico', 'mock')]
    [string]$InputSource = 'manual',
    [string]$ListenAddress,
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')]
    [string]$Distro = 'RM65-Servo',
    [switch]$CheckOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Start-LocalSimulation {
    param(
        [ValidateSet('manual', 'pico', 'mock')][string]$Source = 'manual',
        [string]$Address,
        [string]$Distribution = 'RM65-Servo',
        [switch]$ReadOnly
    )
    if ($Source -ne 'pico' -and $Address) { throw 'ListenAddress is only used with -InputSource pico.' }
    $root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
    $preferred = Join-Path $env:USERPROFILE 'miniconda3\python.exe'
    $python = if (Test-Path -LiteralPath $preferred -PathType Leaf) { $preferred } else {
        (Get-Command python -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    }
    if ($Source -eq 'mock') {
        $arguments = @('-X', 'utf8', (Join-Path $root 'teleop.py'), '--mode', 'sim', '--target', 'wsl',
            '--wsl-distro', $Distribution, '--input', 'mock')
        if ($ReadOnly) { $arguments += '--print-command' }
    } else {
        # Source is restricted by ValidateSet. All caller-provided values stay in argv.
        $arguments = @('-X', 'utf8', '-c', "from scripts.setup.launcher import cli; raise SystemExit(cli('$Source'))",
            '--distro', $Distribution)
        if ($Address) { $arguments += @('--listen-address', $Address) }
        if ($ReadOnly) { $arguments += '--check-only' }
    }
    Push-Location -LiteralPath $root
    try {
        & $python @arguments | Out-Host
        return [int]$LASTEXITCODE
    } finally { Pop-Location }
}

if ($MyInvocation.InvocationName -ne '.') {
    try { exit (Start-LocalSimulation -Source $InputSource -Address $ListenAddress -Distribution $Distro -ReadOnly:$CheckOnly) }
    catch { [Console]::Error.WriteLine($_.Exception.Message); exit 2 }
}
