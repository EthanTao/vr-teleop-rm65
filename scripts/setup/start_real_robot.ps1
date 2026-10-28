<#
.SYNOPSIS
Use the existing board-side Servo chain only with explicit现场 parameters.
.DESCRIPTION
No-argument startup explains the required UDP address,现场 validation file,
and LiveAck, then exits without connecting. CheckOnly asks teleop.py to preview
its commands offline. DryRun still connects to the real controller for state
readout when actually started; it does not create motion interfaces.
.EXAMPLE
  .\scripts\setup\start_real_robot.ps1 -DryRun -UdpIp <board-address> -CheckOnly
#>
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

function Get-RealRobotPython {
    $preferred = Join-Path $env:USERPROFILE 'miniconda3\python.exe'
    if (Test-Path -LiteralPath $preferred -PathType Leaf) { return $preferred }
    return (Get-Command python -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
}

function Invoke-RealRobotNative {
    param([string]$Executable, [string[]]$Arguments)
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Command failed with exit code $LASTEXITCODE`: $Executable" }
}

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
    $missing = @()
    if (-not $BoardUdpIp) { $missing += '-UdpIp：机器人能够访问的 Linux 板 IPv4 地址' }
    if (-not $StateOnly) {
        if (-not $ValidationFile) { $missing += '-HardwareValidation：config 目录内的有效现场验证文件' }
        if ($OperatorAck -cne 'RM65 LIVE VALIDATION') { $missing += '-LiveAck：本人显式提供的 "RM65 LIVE VALIDATION"' }
    }
    if ($missing.Count) {
        throw ("尚未启动真机。必须显式提供：`n  " + ($missing -join "`n  ") +
            "`n先准备现场证据；-CheckOnly 仅预览。-DryRun 也会连接真实控制器读取反馈。")
    }
    $parsed = $null
    if (-not [Net.IPAddress]::TryParse($BoardUdpIp, [ref]$parsed) -or
        $parsed.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
        $parsed.ToString() -cne $BoardUdpIp) {
        throw '-UdpIp 必须是机器人能够访问的 Linux 板规范 IPv4 地址。未启动真机。'
    }
    $root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
    $entry = Join-Path $root 'teleop.py'
    if (-not (Test-Path -LiteralPath $entry -PathType Leaf)) { throw "Required project file is missing: $entry" }
    $mode = if ($StateOnly) { 'hardware-dry-run' } else { 'hardware' }
    $arguments = @('-X', 'utf8', $entry, '--mode', $mode, '--target', 'remote', '--host', $RemoteHost,
        '--remote-dir', $RemotePath, '--input', 'pico', '--arm-ip', $RobotIp, '--udp-ip', $BoardUdpIp)
    if (-not $StateOnly) { $arguments += @('--hardware-validation', $ValidationFile, '--live-ack', $OperatorAck) }
    if ($ReadOnly) {
        $arguments += '--print-command'
        Write-Host 'CheckOnly：仅验证参数并预览命令，不连接、同步或部署。'
    } elseif ($StateOnly) {
        Write-Host 'Dry Run：将连接真实控制器读取状态，不创建运动接口。'
    } else {
        Write-Host '真机执行：继续核对现有现场验证准入，不自动回位。'
    }
    $python = Get-RealRobotPython
    Push-Location -LiteralPath $root
    try {
        $env:PYTHONUTF8 = '1'
        Invoke-RealRobotNative $python $arguments
    } finally { Pop-Location }
}

if ($MyInvocation.InvocationName -ne '.') {
    try {
        Start-RealRobot -BoardUdpIp $UdpIp -ValidationFile $HardwareValidation -OperatorAck $LiveAck `
            -RemoteHost $SshHost -RemotePath $RemoteDir -RobotIp $ArmIp -ReadOnly:$CheckOnly -StateOnly:$DryRun
    } catch {
        Write-Host $_.Exception.Message
        exit 2
    }
}
