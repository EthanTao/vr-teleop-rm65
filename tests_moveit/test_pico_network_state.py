"""Run the real PS5.1 state functions against temporary files and fake OS APIs."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess

import pytest

POWERSHELL = shutil.which("powershell.exe")
pytestmark = pytest.mark.skipif(os.name != "nt" or POWERSHELL is None, reason="Windows PowerShell state checks")
HELPER = Path(__file__).resolve().parents[1] / "scripts/setup/connect_pico_wsl.ps1"


def literal(value):
    return "'" + str(value).replace("'", "''") + "'"


def run_state_check(tmp_path, body):
    # Parse the production file and load only named function definitions. This
    # never executes its administrator-only script body or real network APIs.
    source = f"""
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$tokens = $null; $errors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile({literal(HELPER)}, [ref]$tokens, [ref]$errors)
if ($errors.Count) {{ throw ($errors | Out-String) }}
$wanted = @('Save-OwnedState','Read-OwnedState','Assert-IPv4','Assert-OwnFirewallRule','Add-OwnedFirewall','Set-OwnedProxy')
$functions = $ast.FindAll({{ param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] }}, $false)
foreach ($function in $functions) {{
    if ($function.Name -in $wanted) {{ . ([scriptblock]::Create($function.Extent.Text)) }}
}}
$StatePath = {literal(tmp_path / 'owned-state.json')}
$Owner = 'test.owner'; $ProjectRoot = 'test.project'; $RuleName = 'test.rule'; $RuleDescription = 'test.description'
$Port = 63901; $ListenAddress = '192.168.1.2'; $wslAddress = '172.20.0.3'; $Distro = 'RM65-Servo'; $Netsh = 'fake-netsh'
$state = [pscustomobject]@{{version=1; owner=$Owner; project_root=$ProjectRoot; rule_name=$RuleName;
    listen_address=$ListenAddress; wsl_address='172.20.0.2'; distro=$Distro;
    portproxy_created=$false; firewall_created=$false}}
{body}
"""
    script = tmp_path / "state_check.ps1"
    script.write_text(source, encoding="utf-8-sig")
    return subprocess.run(
        [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(script)],
        capture_output=True,
        text=True,
        timeout=20,
    )


def test_first_save_and_atomic_overwrite_are_readable_on_ps51(tmp_path):
    result = run_state_check(
        tmp_path,
        """
Save-OwnedState $state
$first = Read-OwnedState
if ($first.firewall_created) { throw 'Initial state was not saved' }
$state.firewall_created = $true
Save-OwnedState $state
$second = Read-OwnedState
if (-not $second.firewall_created) { throw 'Updated state was not saved' }
if (@(Get-ChildItem -LiteralPath (Split-Path -Parent $StatePath) | Where-Object { $_.Name -match '\\.(tmp|bak)$' }).Count) {
    throw 'Successful save left temporary or backup files'
}
""",
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("array_values", [False, True])
@pytest.mark.parametrize("multiple_field", [None, "local", "remote", "port"])
def test_firewall_cim_scalars_and_single_arrays_have_identical_policy(tmp_path, array_values, multiple_field):
    result = run_state_check(
        tmp_path,
        "$asArray = $" + str(array_values).lower() + "\n"
        "$multiple = "
        + (literal(multiple_field) if multiple_field else "$null")
        + "\n"
        + """
$local = if ($asArray) { ,@($ListenAddress) } else { $ListenAddress }
$remote = if ($asArray) { ,@('LocalSubnet') } else { 'LocalSubnet' }
$localPort = if ($asArray) { ,@([string]$Port) } else { [string]$Port }
switch ($multiple) {
    'local' { $local = @($ListenAddress, '192.168.1.99') }
    'remote' { $remote = @('LocalSubnet', '192.168.1.99') }
    'port' { $localPort = @([string]$Port, '63902') }
}
$script:addressFilter = [pscustomobject]@{LocalAddress=$local;RemoteAddress=$remote}
$script:portFilter = [pscustomobject]@{LocalPort=$localPort;Protocol='TCP'}
function Get-NetFirewallAddressFilter { $script:addressFilter }
function Get-NetFirewallPortFilter { $script:portFilter }
$rule = [pscustomobject]@{Description=$RuleDescription;Direction='Inbound';Action='Allow';Enabled='True'}
$rejected = $false
try { Assert-OwnFirewallRule $rule $ListenAddress } catch {
    if ($_.Exception.Message -notlike '*does not match*') { throw }
    $rejected = $true
}
if ($rejected -ne [bool]$multiple) { throw "Wrong firewall policy for array=$asArray, multiple=$multiple" }
""",
    )
    assert result.returncode == 0, result.stderr


def test_failed_save_will_not_rollback_a_proxy_changed_by_someone_else(tmp_path):
    result = run_state_check(
        tmp_path,
        """
$script:calls = [Collections.Generic.List[object]]::new()
function Invoke-NativeReadOrChange {
    param($Executable, [string[]]$Arguments)
    $script:calls.Add($Arguments)
}
function Get-ProxyEntries {
    [pscustomobject]@{Kind='v4tov4';ListenAddress=$ListenAddress;ListenPort=$Port;ConnectAddress='172.20.99.99';ConnectPort=$Port;Protocol='tcp'}
}
Save-OwnedState $state
$before = [IO.File]::ReadAllText($StatePath)
$locked = [IO.File]::Open($StatePath, [IO.FileMode]::Open, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
$failed = $false
try { Set-OwnedProxy $state $null } catch {
    $failed = $true
    if ($_.Exception.Message -notlike '*refusing to overwrite*') { throw }
} finally { $locked.Dispose() }
if (-not $failed -or $script:calls.Count -ne 1) { throw 'Foreign proxy was overwritten during rollback' }
if ([IO.File]::ReadAllText($StatePath) -cne $before) { throw 'Original state was changed' }
""",
    )
    assert result.returncode == 0, result.stderr


def test_failed_atomic_replace_keeps_old_state_and_proposed_recovery_file(tmp_path):
    result = run_state_check(
        tmp_path,
        """
Save-OwnedState $state
$before = [IO.File]::ReadAllText($StatePath)
$state.firewall_created = $true
$locked = [IO.File]::Open($StatePath, [IO.FileMode]::Open, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
$failed = $false
try { Save-OwnedState $state } catch {
    $failed = $true
    if ($_.Exception.Message -notlike '*Cannot persist PICO ownership state*') { throw }
} finally { $locked.Dispose() }
if (-not $failed) { throw 'Expected an actual filesystem replace failure' }
if ([IO.File]::ReadAllText($StatePath) -cne $before) { throw 'Original ownership state was lost' }
$proposed = @(Get-ChildItem -LiteralPath (Split-Path -Parent $StatePath) -Filter 'owned-state.json.*.tmp')
if ($proposed.Count -ne 1) { throw 'Missing proposed recovery state' }
if (-not ((Get-Content -LiteralPath $proposed[0].FullName -Raw | ConvertFrom-Json).firewall_created)) {
    throw 'Recovery state did not retain the proposed change'
}
""",
    )
    assert result.returncode == 0, result.stderr


def test_failed_firewall_state_save_rolls_back_only_created_rule(tmp_path):
    result = run_state_check(
        tmp_path,
        """
function New-NetFirewallRule { [pscustomobject]@{TestId='created-here'} }
function Assert-OwnFirewallRule {
    param($Rule, $Address)
    if ($Rule.TestId -ne 'created-here' -or $Address -ne $ListenAddress) { throw 'Wrong rollback object' }
}
$script:removed = $false
function Remove-NetFirewallRule { $script:removed = $true }
Save-OwnedState $state
$before = [IO.File]::ReadAllText($StatePath)
$locked = [IO.File]::Open($StatePath, [IO.FileMode]::Open, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
$failed = $false
try { Add-OwnedFirewall $state } catch {
    $failed = $true
    if ($_.Exception.Message -notlike '*firewall rule created by this attempt was rolled back*') { throw }
} finally { $locked.Dispose() }
if (-not $failed -or -not $script:removed -or $state.firewall_created) { throw 'Firewall transaction did not roll back' }
if ([IO.File]::ReadAllText($StatePath) -cne $before) { throw 'Original ownership state changed' }
""",
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize("existing", [False, True])
def test_failed_proxy_state_save_restores_previous_target_or_removes_new_mapping(tmp_path, existing):
    result = run_state_check(
        tmp_path,
        "$state.portproxy_created = $" + str(existing).lower() + "\n" + """
$recorded = $null
if ($state.portproxy_created) {
    $recorded = [pscustomobject]@{ConnectAddress=$state.wsl_address;ConnectPort=$Port}
}
$script:calls = [Collections.Generic.List[object]]::new()
function Invoke-NativeReadOrChange {
    param($Executable, [string[]]$Arguments)
    $script:calls.Add($Arguments)
}
function Get-ProxyEntries {
    [pscustomobject]@{Kind='v4tov4';ListenAddress=$ListenAddress;ListenPort=$Port;ConnectAddress=$wslAddress;ConnectPort=$Port;Protocol='tcp'}
}
Save-OwnedState $state
$before = [IO.File]::ReadAllText($StatePath)
$locked = [IO.File]::Open($StatePath, [IO.FileMode]::Open, [IO.FileAccess]::ReadWrite, [IO.FileShare]::None)
$failed = $false
try { Set-OwnedProxy $state $recorded } catch {
    $failed = $true
    if ($_.Exception.Message -notlike '*proxy change was rolled back*') { throw }
} finally { $locked.Dispose() }
if (-not $failed -or $script:calls.Count -ne 2) { throw 'Missing proxy rollback call' }
if ($state.wsl_address -ne '172.20.0.2') { throw 'In-memory state was not restored' }
if ([IO.File]::ReadAllText($StatePath) -cne $before) { throw 'Original state was changed' }
if ($null -eq $recorded) {
    if ($script:calls[1][2] -ne 'delete') { throw 'New mapping was not removed' }
} else {
    if ($script:calls[1][2] -ne 'set' -or 'connectaddress=172.20.0.2' -notin $script:calls[1]) {
        throw 'Previous mapping target was not restored'
    }
}
""",
    )
    assert result.returncode == 0, result.stderr
