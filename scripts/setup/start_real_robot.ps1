<# Compatibility CLI: validation and explicit operator acknowledgement remain in Python. #>
[CmdletBinding()]
param(
    [string]$UdpIp,
    [string]$HardwareValidation,
    [string]$LiveAck,
    [string]$SshHost = 'rm',
    [string]$RemoteDir = 'EthanTao',
    [string]$ArmIp = '192.168.10.18',
    [switch]$DryRun,
    [switch]$CheckOnly
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Start-RealRobot {
    param(
        [string]$BoardUdpIp,
        [string]$ValidationFile,
        [string]$OperatorAck,
        [string]$RemoteHost = 'rm',
        [string]$RemotePath = 'EthanTao',
        [string]$RobotIp = '192.168.10.18',
        [switch]$ReadOnly,
        [switch]$StateOnly
    )
    $root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
    $preferred = Join-Path $env:USERPROFILE 'miniconda3\python.exe'
    $python = if (Test-Path -LiteralPath $preferred -PathType Leaf) { $preferred } else {
        (Get-Command python -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
    }
    # Explicit defaults keep legacy no-argument use non-interactive and fail closed.
    $arguments = @('-X', 'utf8', '-c', "from scripts.setup.launcher import cli; raise SystemExit(cli('real'))",
        '--host', $RemoteHost, '--remote-dir', $RemotePath, '--arm-ip', $RobotIp)
    if ($BoardUdpIp) { $arguments += @('--udp-ip', $BoardUdpIp) }
    if ($ValidationFile) { $arguments += @('--hardware-validation', $ValidationFile) }
    if ($OperatorAck) { $arguments += @('--live-ack', $OperatorAck) }
    if ($StateOnly) { $arguments += '--dry-run' }
    if ($ReadOnly) { $arguments += '--check-only' }
    Push-Location -LiteralPath $root
    try {
        & $python @arguments | Out-Host
        return [int]$LASTEXITCODE
    } finally { Pop-Location }
}

if ($MyInvocation.InvocationName -ne '.') {
    try {
        exit (Start-RealRobot -BoardUdpIp $UdpIp -ValidationFile $HardwareValidation -OperatorAck $LiveAck `
            -RemoteHost $SshHost -RemotePath $RemoteDir -RobotIp $ArmIp -ReadOnly:$CheckOnly -StateOnly:$DryRun)
    } catch { [Console]::Error.WriteLine($_.Exception.Message); exit 2 }
}
