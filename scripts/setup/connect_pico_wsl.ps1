#Requires -RunAsAdministrator
<#
.SYNOPSIS
Prepare the optional TCP path from a PICO headset to the local WSL simulation.
.DESCRIPTION
Uses the existing WSL NAT network. This script does not change .wslconfig,
start services/distributions, restart anything, or terminate other programs.
The headset must manually connect to ListenAddress, port 63901. XR discovery
broadcasts use UDP 29888 and are deliberately not forwarded here. The SDK's
local gRPC port 60061 stays inside the container.

Run -WhatIf first. Apply only after reviewing the addresses and starting the
intended WSL distribution yourself. The PC Service/container can start later.
Only this checkout's recorded mapping and named firewall rule can be changed.
.EXAMPLE
  .\scripts\setup\connect_pico_wsl.ps1 -ListenAddress 192.168.1.50 -WhatIf
.EXAMPLE
  .\scripts\setup\connect_pico_wsl.ps1 -ListenAddress 192.168.1.50
.EXAMPLE
  .\scripts\setup\connect_pico_wsl.ps1 -Remove -WhatIf
.EXAMPLE
  .\scripts\setup\connect_pico_wsl.ps1 -Remove
#>
[CmdletBinding(SupportsShouldProcess = $true, ConfirmImpact = 'Medium', DefaultParameterSetName = 'Configure')]
param(
    [Parameter(Mandatory = $true, ParameterSetName = 'Configure')]
    [string]$ListenAddress,

    [Parameter(ParameterSetName = 'Configure')]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')]
    [string]$Distro = 'RM65-Servo',

    [Parameter(Mandatory = $true, ParameterSetName = 'Remove')]
    [switch]$Remove
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$ProjectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$StatePath = Join-Path $ProjectRoot 'build\setup\local-servo\pico-portproxy.json'
$Owner = 'rm65.connect_pico_wsl.v1'
$Port = 63901
$Netsh = Join-Path $env:SystemRoot 'System32\netsh.exe'
$Wsl = Join-Path $env:SystemRoot 'System32\wsl.exe'
$hash = [Security.Cryptography.SHA256]::Create()
try {
    $rootBytes = [Text.Encoding]::UTF8.GetBytes($ProjectRoot.ToLowerInvariant())
    $rootHash = ([BitConverter]::ToString($hash.ComputeHash($rootBytes))).Replace('-', '').Substring(0, 16)
} finally { $hash.Dispose() }
$RuleName = "RM65-PICO-WSL-TCP63901-$rootHash"
$RuleDescription = "$Owner; project=$ProjectRoot; TCP63901 only; remove using this script"

function Invoke-NativeReadOrChange {
    param([string]$Executable, [string[]]$Arguments)
    $lines = @(& $Executable @Arguments 2>&1)
    if ($LASTEXITCODE -ne 0) {
        throw "Native command failed ($LASTEXITCODE): $Executable $($Arguments -join ' ')`n$($lines -join '`n')"
    }
    return (($lines | ForEach-Object { ([string]$_).Replace([string][char]0, '') }) -join "`n")
}

function Assert-IPv4 {
    param([string]$Value, [string]$Label)
    $parsed = $null
    if ($Value -notmatch '^\d{1,3}(\.\d{1,3}){3}$' -or
        -not [Net.IPAddress]::TryParse($Value, [ref]$parsed) -or
        $parsed.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork -or
        $parsed.ToString() -cne $Value) {
        throw "$Label must be an explicit canonical IPv4 address."
    }
    $bytes = $parsed.GetAddressBytes()
    if ($bytes[0] -eq 0 -or $bytes[0] -eq 127 -or $bytes[0] -ge 224) {
        throw "$Label must be a unicast LAN IPv4 address; wildcard/loopback/multicast addresses are forbidden."
    }
}

function Get-ProxyEntries {
    # 'dump' emits netsh command syntax, avoiding localized table headings.
    $dump = Invoke-NativeReadOrChange $Netsh @('interface', 'portproxy', 'dump')
    foreach ($line in ($dump -split "`r?`n")) {
        if ($line -notmatch '^\s*add\s+(v[46]tov[46])\s+') { continue }
        $kind = $Matches[1]
        $fields = @{}
        foreach ($match in [regex]::Matches($line, '(listenaddress|listenport|connectaddress|connectport|protocol)=("[^"]*"|\S+)')) {
            $fields[$match.Groups[1].Value] = $match.Groups[2].Value.Trim('"')
        }
        if (-not $fields.ContainsKey('listenport') -or -not $fields.ContainsKey('connectaddress')) {
            throw 'Cannot safely parse an existing portproxy entry; refusing changes.'
        }
        $listenPort = 0
        $connectPort = 0
        if (-not [int]::TryParse($fields.listenport, [ref]$listenPort)) { throw 'Invalid existing listen port.' }
        if ($fields.ContainsKey('connectport')) {
            if (-not [int]::TryParse($fields.connectport, [ref]$connectPort)) { throw 'Invalid existing destination port.' }
        } else { $connectPort = $listenPort }
        $listen = if ($fields.ContainsKey('listenaddress')) { $fields.listenaddress } elseif ($kind.StartsWith('v4')) { '0.0.0.0' } else { '::' }
        [pscustomobject]@{
            Kind = $kind; ListenAddress = $listen; ListenPort = $listenPort
            ConnectAddress = $fields.connectaddress; ConnectPort = $connectPort
            Protocol = if ($fields.ContainsKey('protocol')) { $fields.protocol } else { 'tcp' }
        }
    }
}

function Get-OwnFirewallRule {
    $rules = @(Get-NetFirewallRule -PolicyStore PersistentStore -Name $RuleName -ErrorAction SilentlyContinue)
    if ($rules.Count -gt 1) { throw "Multiple firewall rules named $RuleName; refusing changes." }
    if ($rules.Count -eq 1) { return $rules[0] }
    return $null
}

function Assert-OwnFirewallRule {
    param($Rule, [string]$Address)
    if ($null -eq $Rule) { return }
    $addressFilter = Get-NetFirewallAddressFilter -AssociatedNetFirewallRule $Rule
    $portFilter = Get-NetFirewallPortFilter -AssociatedNetFirewallRule $Rule
    if ($Rule.Description -cne $RuleDescription -or $Rule.Direction -ne 'Inbound' -or
        $Rule.Action -ne 'Allow' -or $Rule.Enabled -ne 'True' -or
        @($addressFilter.LocalAddress).Count -ne 1 -or @($addressFilter.LocalAddress)[0] -ne $Address -or
        @($addressFilter.RemoteAddress).Count -ne 1 -or @($addressFilter.RemoteAddress)[0] -ne 'LocalSubnet' -or
        @($portFilter.LocalPort).Count -ne 1 -or [string]@($portFilter.LocalPort)[0] -ne [string]$Port -or
        [string]$portFilter.Protocol -notin @('TCP', '6')) {
        throw "Firewall rule $RuleName exists but does not match this script's recorded restricted rule; refusing changes."
    }
}

function Save-OwnedState {
    param($Value)
    $folder = Split-Path -Parent $StatePath
    [void][IO.Directory]::CreateDirectory($folder)
    $temporary = "$StatePath.$([Guid]::NewGuid().ToString('N')).tmp"
    $backup = "$StatePath.$([Guid]::NewGuid().ToString('N')).bak"
    try {
        [IO.File]::WriteAllText($temporary, ($Value | ConvertTo-Json -Depth 4), [Text.UTF8Encoding]::new($false))
        if ([IO.File]::Exists($StatePath)) {
            # Windows PowerShell 5.1 converts a null string argument to an empty
            # path for this .NET overload. Use an actual same-volume backup.
            [IO.File]::Replace($temporary, $StatePath, $backup)
        } else { [IO.File]::Move($temporary, $StatePath) }
    } catch {
        # Do not erase a proposed state or backup after a failed commit. The
        # old destination is retained by the atomic replace, and these paths
        # permit diagnosis if the filesystem itself prevented the operation.
        throw "Cannot persist PICO ownership state: $($_.Exception.Message). Inspect state=$StatePath; proposed=$temporary; backup=$backup. No recovery files were deleted."
    }
    if ([IO.File]::Exists($backup)) {
        try { [IO.File]::Delete($backup) } catch {
            # The new state is already committed. Reporting a save failure now
            # would make callers roll back a system change against a new state.
            Write-Warning "PICO ownership state saved; old backup could not be removed: $backup" -WarningAction Continue
        }
    }
}

function Add-OwnedFirewall {
    param($State)
    $created = New-NetFirewallRule -PolicyStore PersistentStore -Name $RuleName -DisplayName 'RM65 PICO local WSL simulation TCP63901' `
        -Description $RuleDescription -Group 'RM65 local simulation' -Direction Inbound -Action Allow -Enabled True `
        -Profile Any -Protocol TCP -LocalPort $Port -LocalAddress $ListenAddress -RemoteAddress LocalSubnet `
        -EdgeTraversalPolicy Block -Confirm:$false
    $previous = $State.firewall_created
    $State.firewall_created = $true
    try { Save-OwnedState $State } catch {
        $saveFailure = $_.Exception.Message
        $State.firewall_created = $previous
        try {
            Assert-OwnFirewallRule $created $ListenAddress
            Remove-NetFirewallRule -InputObject $created -Confirm:$false
        } catch {
            throw "Ownership save failed and the new firewall rule could not be rolled back: $($_.Exception.Message). $saveFailure"
        }
        throw "Ownership save failed; the firewall rule created by this attempt was rolled back. $saveFailure"
    }
}

function Set-OwnedProxy {
    param($State, $Recorded)
    $previousAddress = $State.wsl_address
    $previousDistro = $State.distro
    $previousCreated = $State.portproxy_created
    $verb = if ($null -eq $Recorded) { 'add' } else { 'set' }
    [void](Invoke-NativeReadOrChange $Netsh @('interface', 'portproxy', $verb, 'v4tov4', "listenaddress=$ListenAddress", "listenport=$Port", "connectaddress=$wslAddress", "connectport=$Port", 'protocol=tcp'))
    $State.wsl_address = $wslAddress
    $State.distro = $Distro
    $State.portproxy_created = $true
    try { Save-OwnedState $State } catch {
        $saveFailure = $_.Exception.Message
        $State.wsl_address = $previousAddress
        $State.distro = $previousDistro
        $State.portproxy_created = $previousCreated
        try {
            $present = @(Get-ProxyEntries | Where-Object { $_.Kind -eq 'v4tov4' -and $_.ListenAddress -eq $ListenAddress -and $_.ListenPort -eq $Port })
            if ($present.Count -ne 1 -or $present[0].ConnectAddress -ne $wslAddress -or
                $present[0].ConnectPort -ne $Port -or $present[0].Protocol -ne 'tcp') {
                throw 'Proxy changed outside this attempt; refusing to overwrite it during rollback.'
            }
            if ($null -eq $Recorded) {
                [void](Invoke-NativeReadOrChange $Netsh @('interface', 'portproxy', 'delete', 'v4tov4', "listenaddress=$ListenAddress", "listenport=$Port", 'protocol=tcp'))
            } else {
                [void](Invoke-NativeReadOrChange $Netsh @('interface', 'portproxy', 'set', 'v4tov4', "listenaddress=$ListenAddress", "listenport=$Port", "connectaddress=$($Recorded.ConnectAddress)", "connectport=$($Recorded.ConnectPort)", 'protocol=tcp'))
            }
        } catch {
            throw "Ownership save failed and proxy rollback could not complete: $($_.Exception.Message). $saveFailure"
        }
        throw "Ownership save failed; this attempt's proxy change was rolled back. $saveFailure"
    }
}

function Read-OwnedState {
    if (-not (Test-Path -LiteralPath $StatePath -PathType Leaf)) { return $null }
    try { $value = Get-Content -LiteralPath $StatePath -Raw -Encoding UTF8 | ConvertFrom-Json } catch {
        throw "Cannot read ownership state $StatePath; inspect it before changing any rules."
    }
    foreach ($name in @('version', 'owner', 'project_root', 'rule_name', 'listen_address', 'wsl_address', 'distro', 'portproxy_created', 'firewall_created')) {
        if ($null -eq $value.PSObject.Properties[$name]) { throw "Ownership state is missing '$name'; refusing changes." }
    }
    if ($value.version -ne 1 -or $value.owner -cne $Owner -or $value.project_root -ine $ProjectRoot -or
        $value.rule_name -cne $RuleName -or $value.portproxy_created -isnot [bool] -or $value.firewall_created -isnot [bool]) {
        throw "Ownership state does not belong to this checkout: $StatePath"
    }
    Assert-IPv4 ([string]$value.listen_address) 'Recorded listen address'
    Assert-IPv4 ([string]$value.wsl_address) 'Recorded WSL address'
    return $value
}

function Find-RecordedProxy {
    param($State, [object[]]$Entries)
    $matches = @($Entries | Where-Object { $_.Kind -eq 'v4tov4' -and $_.ListenAddress -eq $State.listen_address -and $_.ListenPort -eq $Port })
    if ($matches.Count -gt 1) { throw 'Multiple entries match the ownership record; refusing changes.' }
    if ($matches.Count -eq 1) {
        $entry = $matches[0]
        if (-not $State.portproxy_created -or $entry.ConnectAddress -ne $State.wsl_address -or
            $entry.ConnectPort -ne $Port -or $entry.Protocol -ne 'tcp') {
            throw 'The recorded portproxy was changed outside this script, or its creation was not confirmed; refusing to alter it.'
        }
        return $entry
    }
    return $null
}

function Assert-NoForeignProxyConflict {
    param([object[]]$Entries, $Recorded, [string]$Address)
    foreach ($entry in $Entries) {
        if ($entry.ListenPort -ne $Port) { continue }
        if ($null -ne $Recorded -and $entry -eq $Recorded) { continue }
        $numeric = $null
        $knownAddress = [Net.IPAddress]::TryParse([string]$entry.ListenAddress, [ref]$numeric)
        if (-not $knownAddress -or $entry.ListenAddress -in @($Address, '0.0.0.0', '::', '*')) {
            throw 'An unowned portproxy entry could conflict with this listen address/port. It will not be overwritten; inspect netsh interface portproxy show all.'
        }
    }
}

function Get-ReviewedObjects {
    param($ExpectedState, $ExpectedStateText)
    # Confirmation can take time. Recheck ownership immediately after it rather
    # than overwriting a rule or mapping that changed while the prompt was open.
    $presentText = if ([IO.File]::Exists($StatePath)) { [IO.File]::ReadAllText($StatePath, [Text.Encoding]::UTF8) } else { $null }
    if ($presentText -cne $ExpectedStateText) { throw 'Ownership state changed during review; rerun the script to inspect the new state.' }
    $presentEntries = @(Get-ProxyEntries)
    $presentFirewall = Get-OwnFirewallRule
    $presentRecorded = $null
    if ($null -ne $ExpectedState) {
        $presentRecorded = Find-RecordedProxy $ExpectedState $presentEntries
        if ($null -ne $presentFirewall -and -not $ExpectedState.firewall_created) {
            throw 'An unconfirmed firewall rule appeared during review; refusing changes.'
        }
        Assert-OwnFirewallRule $presentFirewall $ExpectedState.listen_address
    } elseif ($null -ne $presentFirewall) {
        throw 'A firewall rule appeared during review without ownership; refusing changes.'
    }
    return [pscustomobject]@{ Entries = $presentEntries; Firewall = $presentFirewall; Recorded = $presentRecorded }
}

if (-not (Test-Path -LiteralPath $Netsh -PathType Leaf)) { throw "netsh.exe not found: $Netsh" }
$state = Read-OwnedState
$reviewedStateText = if ([IO.File]::Exists($StatePath)) { [IO.File]::ReadAllText($StatePath, [Text.Encoding]::UTF8) } else { $null }
$entries = @(Get-ProxyEntries)
$firewall = Get-OwnFirewallRule
$recorded = $null
if ($null -ne $state) {
    $recorded = Find-RecordedProxy $state $entries
    if ($null -ne $firewall -and -not $state.firewall_created) {
        throw 'A firewall rule exists without confirmed ownership in the state file; refusing to adopt it.'
    }
    Assert-OwnFirewallRule $firewall $state.listen_address
} elseif ($null -ne $firewall) {
    throw 'A same-name firewall rule already exists without this checkout ownership state; refusing to adopt it.'
}

if ($PSCmdlet.ParameterSetName -eq 'Remove') {
    if ($null -eq $state) {
        Write-Host "No owned state at $StatePath; nothing will be removed."
        return
    }
    $target = "$($state.listen_address):$Port/TCP and firewall rule $RuleName"
    if (-not $PSCmdlet.ShouldProcess($target, 'Remove only this checkout recorded PICO forwarding and state')) { return }
    $reviewed = Get-ReviewedObjects $state $reviewedStateText
    $recorded = $reviewed.Recorded
    $firewall = $reviewed.Firewall
    if ($null -ne $recorded) {
        [void](Invoke-NativeReadOrChange $Netsh @('interface', 'portproxy', 'delete', 'v4tov4', "listenaddress=$($state.listen_address)", "listenport=$Port", 'protocol=tcp'))
        $state.portproxy_created = $false
        Save-OwnedState $state
    }
    if ($null -ne $firewall) {
        Remove-NetFirewallRule -InputObject $firewall -Confirm:$false
        $state.firewall_created = $false
        Save-OwnedState $state
    }
    [IO.File]::Delete($StatePath)
    Write-Host 'Removed this checkout owned mapping, firewall rule and ownership state.'
    return
}

Assert-IPv4 $ListenAddress 'ListenAddress'
$local = @(Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -eq $ListenAddress -and $_.AddressState -eq 'Preferred' })
if ($local.Count -eq 0) { throw 'ListenAddress is not a currently preferred IPv4 address assigned to this Windows computer. Specify its LAN address explicitly.' }
if ($null -ne $state -and $state.listen_address -ne $ListenAddress) {
    throw 'This checkout already owns a different listen address. Review and run -Remove first; no mapping will be moved automatically.'
}
foreach ($serviceName in @('iphlpsvc', 'MpsSvc')) {
    $service = Get-Service -Name $serviceName -ErrorAction SilentlyContinue
    if ($null -eq $service -or $service.Status -ne 'Running') {
        throw "Required Windows service '$serviceName' is missing or not running. Check/start it explicitly; this script does not change service startup types or restart services."
    }
}
if (-not (Test-Path -LiteralPath $Wsl -PathType Leaf)) { throw "wsl.exe not found: $Wsl" }
$wslServices = @(Get-Service -Name 'WslService', 'LxssManager' -ErrorAction SilentlyContinue | Where-Object { $_.Status -eq 'Running' })
if ($wslServices.Count -eq 0) {
    throw 'The WSL service is not running. Start the intended distribution yourself first; this helper will not activate a stopped WSL service.'
}
$installed = (Invoke-NativeReadOrChange $Wsl @('--list', '--quiet')) -split "`r?`n" | ForEach-Object { $_.Trim() }
if ($Distro -notin $installed) { throw "WSL distribution '$Distro' is not installed under this Windows account." }
$running = (Invoke-NativeReadOrChange $Wsl @('--list', '--running', '--quiet')) -split "`r?`n" | ForEach-Object { $_.Trim() }
if ($Distro -notin $running) { throw "WSL distribution '$Distro' is not running. Start it yourself first; this script will not start a distribution." }
$addressJson = Invoke-NativeReadOrChange $Wsl @('--distribution', $Distro, '--exec', 'ip', '-j', '-4', 'addr', 'show', 'dev', 'eth0')
try { $interfaces = @($addressJson | ConvertFrom-Json) } catch { throw 'WSL ip returned invalid JSON; no network settings were changed.' }
$addresses = @($interfaces | ForEach-Object { $_.addr_info } | Where-Object { $_.family -eq 'inet' -and $_.scope -eq 'global' } | ForEach-Object { [string]$_.local } | Sort-Object -Unique)
if ($addresses.Count -ne 1) { throw 'WSL eth0 must have exactly one global IPv4 address in the default NAT setup; refusing to guess a target.' }
$wslAddress = $addresses[0]
Assert-IPv4 $wslAddress 'WSL eth0 address'
if ($wslAddress -eq $ListenAddress) { throw 'WSL and Windows have the same address. This helper is for the existing NAT setup, not mirrored networking.' }

Assert-NoForeignProxyConflict $entries $recorded $ListenAddress
if ($null -eq $recorded) {
    $listeners = @(Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue | Where-Object { $_.LocalAddress -in @($ListenAddress, '0.0.0.0', '::') })
    if ($listeners.Count -ne 0) { throw 'Another Windows listener already covers this address/port; no process will be stopped and no mapping will be created.' }
}
$action = if ($null -ne $recorded) { 'Update this checkout owned TCP forwarding target and keep its restricted firewall rule' } else { 'Create this checkout owned TCP forwarding and restricted firewall rule' }
$target = "${ListenAddress}:$Port/TCP -> ${wslAddress}:$Port ($Distro); state=$StatePath"
if (-not $PSCmdlet.ShouldProcess($target, $action)) { return }
$reviewed = Get-ReviewedObjects $state $reviewedStateText
Assert-NoForeignProxyConflict $reviewed.Entries $reviewed.Recorded $ListenAddress
$recorded = $reviewed.Recorded
$firewall = $reviewed.Firewall
if ($null -eq $recorded) {
    $listeners = @(Get-NetTCPConnection -State Listen -LocalPort $Port -ErrorAction SilentlyContinue | Where-Object { $_.LocalAddress -in @($ListenAddress, '0.0.0.0', '::') })
    if ($listeners.Count -ne 0) { throw 'Another listener appeared during review; refusing changes.' }
}

if ($null -eq $state) {
    $state = [pscustomobject]@{
        version = 1; owner = $Owner; project_root = $ProjectRoot; rule_name = $RuleName
        listen_address = $ListenAddress; wsl_address = $wslAddress; distro = $Distro
        portproxy_created = $false; firewall_created = $false
    }
    Save-OwnedState $state
}
if ($null -eq $firewall) {
    Add-OwnedFirewall $state
}
Set-OwnedProxy $state $recorded
Write-Host "Configured $target"
Write-Host "In PICO, manually enter $ListenAddress (TCP63901), then enable Controller and Send."
Write-Host 'After the container PC Service is running, verify its TCP63901 listener, then test Windows LAN_IP:63901 from another LAN device.'
Write-Host 'A TCP connection check is not proof of fresh XR frames; confirm /rm65/xr/raw in the sim graph when the headset is available.'
