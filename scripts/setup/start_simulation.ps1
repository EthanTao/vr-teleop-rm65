<#
.SYNOPSIS
Start the local MuJoCo simulation with manual, PICO, or CLI demo input.
.DESCRIPTION
All sources invoke the same teleop.py sim/WSL entry. The default manual input
uses the local browser control panel. Only PICO reads
LAN interfaces or configures this checkout's TCP63901 rule. CheckOnly never
prompts, starts WSL/Docker/ROS, changes networking, or requests elevation;
without InputSource it previews manual input. There is no interactive menu.
.EXAMPLE
  .\scripts\setup\start_simulation.ps1 -InputSource mock -CheckOnly
.EXAMPLE
  .\scripts\setup\start_simulation.ps1 -InputSource pico -ListenAddress 192.168.1.50
#>
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

function Get-PicoNetworkCandidates {
    $candidates = @()
    foreach ($adapter in @(Get-NetAdapter -Physical -ErrorAction Stop | Where-Object { $_.Status -eq 'Up' })) {
        $wireless = [int]$adapter.NdisPhysicalMedium -in @(1, 9)
        foreach ($address in @(Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4 -ErrorAction Stop)) {
            if ($address.AddressState -ne 'Preferred' -or $address.IPAddress -match '^(127\.|169\.254\.|0\.)') { continue }
            $candidates += [pscustomobject]@{
                Address = [string]$address.IPAddress
                Interface = [string]$adapter.Name
                Wireless = $wireless
            }
        }
    }
    return $candidates
}

function Select-PicoListenAddress {
    param([object[]]$Candidates, [string]$Requested)
    if ($Requested) {
        $parsed = $null
        if (-not [Net.IPAddress]::TryParse($Requested, [ref]$parsed) -or
            $parsed.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
            $parsed.ToString() -cne $Requested) {
            throw 'ListenAddress must be a canonical IPv4 address assigned to this computer.'
        }
        $selected = @($Candidates | Where-Object { $_.Address -eq $Requested })
    } else {
        $selected = @($Candidates | Where-Object { $_.Wireless })
        if ($selected.Count -eq 0) { $selected = @($Candidates) }
    }
    if ($selected.Count -ne 1) {
        $available = ($Candidates | ForEach-Object { "$($_.Interface): $($_.Address)" }) -join '; '
        throw "Cannot choose one headset-facing address. Supply -ListenAddress explicitly. Available physical LAN addresses: $available"
    }
    return $selected[0]
}

function Get-PicoPython {
    $preferred = Join-Path $env:USERPROFILE 'miniconda3\python.exe'
    if (Test-Path -LiteralPath $preferred -PathType Leaf) { return $preferred }
    return (Get-Command python -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
}

function Invoke-PicoNative {
    param([string]$Executable, [string[]]$Arguments)
    & $Executable @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Command failed with exit code $LASTEXITCODE`: $Executable" }
}

function Invoke-PicoNetworkMigration {
    param([string]$Helper, [string]$Address, [string]$Distribution, [string]$StateFile)
    if (Test-Path -LiteralPath $StateFile -PathType Leaf) {
        $owned = Get-Content -LiteralPath $StateFile -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($null -eq $owned -or $null -eq $owned.PSObject.Properties['listen_address']) {
            throw "Cannot inspect PICO ownership state: $StateFile"
        }
        if ([string]$owned.listen_address -ne $Address) {
            # The helper validates this checkout's ownership before each removal.
            # A different WiFi address must not strand an old LAN-facing rule.
            & $Helper -Remove -WhatIf
            & $Helper -Remove -Confirm:$false
        }
    }
    & $Helper -ListenAddress $Address -Distro $Distribution -WhatIf
    & $Helper -ListenAddress $Address -Distro $Distribution -Confirm:$false
}

function Invoke-PicoNetworkSetup {
    param([string]$Helper, [string]$Address, [string]$Distribution)
    $project = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
    $stateFile = Join-Path $project 'build\setup\local-servo\pico-portproxy.json'
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = [Security.Principal.WindowsPrincipal]::new($identity)
    if ($principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        Invoke-PicoNetworkMigration -Helper $Helper -Address $Address -Distribution $Distribution -StateFile $stateFile
        return
    }
    # Both executions share one elevated, short-lived process. The script path
    # is encoded as PowerShell data, never interpolated as shell command text.
    $helperLiteral = "'" + $Helper.Replace("'", "''") + "'"
    $addressLiteral = "'" + $Address.Replace("'", "''") + "'"
    $distroLiteral = "'" + $Distribution.Replace("'", "''") + "'"
    $stateLiteral = "'" + $stateFile.Replace("'", "''") + "'"
    $migrationDefinition = (Get-Command Invoke-PicoNetworkMigration -CommandType Function).Definition
    $logPath = Join-Path $project 'build\setup\local-servo\pico-network-last.log'
    [void][IO.Directory]::CreateDirectory((Split-Path -Parent $logPath))
    $logLiteral = "'" + $logPath.Replace("'", "''") + "'"
    $script = @"
`$ErrorActionPreference = 'Stop'
function Invoke-PicoNetworkMigration {
$migrationDefinition
}
try {
    Start-Transcript -LiteralPath $logLiteral -Force | Out-Null
    Invoke-PicoNetworkMigration -Helper $helperLiteral -Address $addressLiteral -Distribution $distroLiteral -StateFile $stateLiteral
    Stop-Transcript | Out-Null
    exit 0
} catch {
    [Console]::Error.WriteLine(`$_.Exception.Message)
    try { Stop-Transcript | Out-Null } catch {}
    exit 1
}
"@
    $encoded = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($script))
    $powershell = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
    Write-Host 'Windows will request administrator permission only for the local PICO forwarding rule.'
    $process = Start-Process -FilePath $powershell -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-EncodedCommand', $encoded) `
        -Verb RunAs -WindowStyle Hidden -Wait -PassThru
    if ($process.ExitCode -ne 0) {
        throw "PICO network setup failed. Inspect $logPath or run connect_pico_wsl.ps1 with -WhatIf in an administrator PowerShell."
    }
    Write-Host "PICO network preview and result: $logPath"
}

function Start-LocalSimulation {
    param([ValidateSet('manual', 'pico', 'mock')][string]$Source = 'manual', [string]$Address, [string]$Distribution, [switch]$ReadOnly)
    $root = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
    $helper = Join-Path $PSScriptRoot 'connect_pico_wsl.ps1'
    $entry = Join-Path $root 'teleop.py'
    foreach ($file in @($entry)) {
        if (-not (Test-Path -LiteralPath $file -PathType Leaf)) { throw "Required project file is missing: $file" }
    }
    if ($Source -ne 'pico' -and $Address) { throw 'ListenAddress is only used with -InputSource pico.' }
    if ($Source -eq 'pico') {
        if (-not (Test-Path -LiteralPath $helper -PathType Leaf)) { throw "Required project file is missing: $helper" }
        $network = Select-PicoListenAddress -Candidates @(Get-PicoNetworkCandidates) -Requested $Address
        Write-Host "PICO manual connection: $($network.Address):63901 (TCP), interface $($network.Interface)"
        Write-Host 'The headset and this Windows address must be on the same reachable LAN.'
        Write-Host 'Select Controller and Send in XRoboToolkit; release right Grip once before holding it to move.'
    } elseif ($Source -eq 'manual') {
        Write-Host '无手柄仿真：仿真服务就绪后打开 http://127.0.0.1:8766 控制面板。'
        Write-Host '仅控制本机仿真，不需要头显或局域网转发。'
    } else {
        Write-Host 'Automatic demo: one 15-second sequence with millimetre-scale targets, then hold.'
        Write-Host 'No keyboard control, headset, or LAN forwarding is used in this mode.'
    }
    $python = Get-PicoPython
    $arguments = @('-X', 'utf8', $entry, '--mode', 'sim', '--target', 'wsl', '--wsl-distro', $Distribution, '--input', $Source)
    if ($ReadOnly) {
        Write-Host "CheckOnly: would start WSL '$Distribution' and verify Docker."
        if ($Source -eq 'pico') { Write-Host 'PICO only: preview/apply the owned TCP63901 rule before launch.' }
        Write-Host "$python $($arguments -join ' ')"
        return
    }
    $wsl = Join-Path $env:SystemRoot 'System32\wsl.exe'
    # docker info starts this distribution if stopped; no other distro, service
    # configuration, global WSL setting, or physical robot is touched.
    Invoke-PicoNative $wsl @('--distribution', $Distribution, '--user', 'root', '--exec', 'docker', 'info', '--format', '{{.ServerVersion}}')
    if ($Source -eq 'pico') {
        Invoke-PicoNetworkSetup -Helper $helper -Address $network.Address -Distribution $Distribution
        Write-Host "Ready for PICO: manually enter $($network.Address), TCP port 63901."
    }
    Write-Host 'Starting MuJoCo simulation. Ctrl+C ends this session.'
    Push-Location -LiteralPath $root
    try {
        $env:PYTHONUTF8 = '1'
        Invoke-PicoNative $python $arguments
    } finally { Pop-Location }
}

if ($MyInvocation.InvocationName -ne '.') {
    Start-LocalSimulation -Source $InputSource -Address $ListenAddress -Distribution $Distro -ReadOnly:$CheckOnly
}
