"""Exercise only the real registry reader in PS5.1, never the setup script body."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess

import pytest

POWERSHELL = shutil.which("powershell.exe")
pytestmark = pytest.mark.skipif(os.name != "nt" or POWERSHELL is None, reason="Windows PowerShell registry checks")
HELPER = Path(__file__).resolve().parents[1] / "scripts/setup/connect_pico_wsl.ps1"


def literal(value):
    return "'" + str(value).replace("'", "''") + "'"


def run_registry_check(tmp_path, body):
    # Parse, never dot-source the script: #Requires/admin checks and its main
    # body must not run. Load exactly one function, with no native helper.
    source = f"""
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$tokens = $null; $errors = $null
$ast = [Management.Automation.Language.Parser]::ParseFile({literal(HELPER)}, [ref]$tokens, [ref]$errors)
if ($errors.Count) {{ throw ($errors | Out-String) }}
$definitions = @($ast.FindAll({{
    param($node)
    $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Get-ProxyEntries'
}}, $false))
if ($definitions.Count -ne 1) {{ throw 'Expected exactly one Get-ProxyEntries definition' }}
. ([scriptblock]::Create($definitions[0].Extent.Text))

$script:root = 'Registry::HKEY_LOCAL_MACHINE/SYSTEM/CurrentControlSet/Services/PortProxy'
$script:kinds = @('v4tov4', 'v4tov6', 'v6tov4', 'v6tov6')
$script:keys = @{{}}
$script:testCalls = [Collections.Generic.List[string]]::new()
$script:getCalls = [Collections.Generic.List[string]]::new()

function Resolve-FakeKey {{
    param([string]$LiteralPath)
    $normalized = $LiteralPath.Replace([char]92, [char]47)
    foreach ($kind in $script:kinds) {{
        if ($normalized -ceq "$script:root/$kind/tcp") {{ return $kind }}
    }}
    throw "MOCK SAFETY: unexpected registry path $LiteralPath"
}}
# CmdletBinding rejects -Path: the reader must use -LiteralPath. These mocks
# never delegate to the original cmdlets, including for unexpected paths.
function Test-Path {{
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)][string]$LiteralPath)
    $kind = Resolve-FakeKey $LiteralPath
    $script:testCalls.Add($kind)
    return $script:keys.ContainsKey($kind)
}}
function Get-Item {{
    [CmdletBinding()]
    param([Parameter(Mandatory=$true)][string]$LiteralPath)
    $kind = Resolve-FakeKey $LiteralPath
    $script:getCalls.Add($kind)
    if (-not $script:keys.ContainsKey($kind)) {{ throw 'MOCK SAFETY: Get-Item called for absent key' }}
    return $script:keys[$kind]
}}
function Invoke-NativeReadOrChange {{ throw 'MOCK SAFETY: native command attempted' }}
function Add-FakeKey {{
    param([string]$Kind, [System.Collections.IDictionary]$Values)
    $key = [pscustomobject]@{{Values=$Values}}
    $key | Add-Member -MemberType ScriptMethod -Name GetValueNames -Value {{ @($this.Values.Keys) }}
    $key | Add-Member -MemberType ScriptMethod -Name GetValue -Value {{
        param($Name)
        # Preserve non-string registry value types, including singleton arrays.
        return ,$this.Values[$Name]
    }}
    $script:keys[$Kind] = $key
}}
function Assert-AllKindsRead {{
    if (($script:testCalls -join ',') -cne ($script:kinds -join ',')) {{
        throw "Did not probe the four exact registry keys: $($script:testCalls -join ',')"
    }}
    if ($script:getCalls.Count -ne $script:keys.Count) {{ throw 'Present registry keys were not read exactly once' }}
}}
{body}
"""
    script = tmp_path / "registry_check.ps1"
    script.write_text(source, encoding="utf-8-sig")
    return subprocess.run(
        [POWERSHELL, "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass", "-File", str(script)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        timeout=20,
    )


def test_preserves_exact_old_wifi_address_that_netsh_dump_omitted(tmp_path):
    result = run_registry_check(
        tmp_path,
        """
# Actual netsh dump omitted listenaddress for this stored non-wildcard name.
Add-FakeKey 'v4tov4' @{'10.249.123.141/63901'='172.29.253.216/63901'}
$entries = @(Get-ProxyEntries)
Assert-AllKindsRead
if ($entries.Count -ne 1) { throw 'Expected the stored mapping' }
$entry = $entries[0]
if ($entry.Kind -cne 'v4tov4' -or $entry.ListenAddress -cne '10.249.123.141' -or
    $entry.ListenPort -ne 63901 -or $entry.ConnectAddress -cne '172.29.253.216' -or
    $entry.ConnectPort -ne 63901 -or $entry.Protocol -cne 'tcp') {
    throw 'Stored old WiFi endpoint was changed or guessed as wildcard'
}
if (($entry.PSObject.Properties.Name | Sort-Object) -join ',' -cne
    'ConnectAddress,ConnectPort,Kind,ListenAddress,ListenPort,Protocol') { throw 'Entry schema changed' }
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_reads_all_four_kinds_with_ipv6_and_distinct_port_boundaries(tmp_path):
    result = run_registry_check(
        tmp_path,
        """
Add-FakeKey 'v4tov4' @{'192.0.2.1/1'='198.51.100.1/65535'}
Add-FakeKey 'v4tov6' @{'192.0.2.2/63901'='2001:db8::2/63902'}
Add-FakeKey 'v6tov4' @{'2001:db8::3/63903'='198.51.100.3/63904'}
Add-FakeKey 'v6tov6' @{'::/65535'='2001:db8::4/1'}
$entries = @(Get-ProxyEntries)
Assert-AllKindsRead
if ($entries.Count -ne 4) { throw 'A proxy kind was omitted' }
$expected = @(
    @('v4tov4', '192.0.2.1', 1, '198.51.100.1', 65535),
    @('v4tov6', '192.0.2.2', 63901, '2001:db8::2', 63902),
    @('v6tov4', '2001:db8::3', 63903, '198.51.100.3', 63904),
    @('v6tov6', '::', 65535, '2001:db8::4', 1)
)
foreach ($want in $expected) {
    $found = @($entries | Where-Object { $_.Kind -ceq $want[0] })
    if ($found.Count -ne 1) { throw 'Missing or duplicated kind' }
    $entry = $found[0]
    if ($entry.ListenAddress -cne $want[1] -or $entry.ListenPort -ne $want[2] -or
        $entry.ConnectAddress -cne $want[3] -or $entry.ConnectPort -ne $want[4] -or
        $entry.Protocol -cne 'tcp') { throw "Endpoint changed for $($want[0])" }
}
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("empty_key", [False, True], ids=["absent-keys", "empty-key"])
def test_absent_and_empty_registry_keys_return_no_entries(tmp_path, empty_key):
    result = run_registry_check(
        tmp_path,
        ("Add-FakeKey 'v4tov4' @{}\n" if empty_key else "") + """
$entries = @(Get-ProxyEntries)
Assert-AllKindsRead
if ($entries.Count -ne 0) { throw 'Absent/empty registry key invented an entry' }
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_wildcard_is_preserved_only_when_explicitly_stored(tmp_path):
    result = run_registry_check(
        tmp_path,
        """
Add-FakeKey 'v4tov4' @{'0.0.0.0/63901'='172.29.253.216/63901'}
$entries = @(Get-ProxyEntries)
Assert-AllKindsRead
if ($entries.Count -ne 1 -or $entries[0].ListenAddress -cne '0.0.0.0') {
    throw 'Explicitly stored wildcard was not preserved'
}
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize(
    "name,value",
    [
        ("", "'172.29.253.216/63901'"),
        ("63901", "'172.29.253.216/63901'"),
        ("/63901", "'172.29.253.216/63901'"),
        ("   /63901", "'172.29.253.216/63901'"),
        ("10.249.123.141/", "'172.29.253.216/63901'"),
        ("10.249.123.141/abc", "'172.29.253.216/63901'"),
        ("10.249.123.141/1.5", "'172.29.253.216/63901'"),
        ("10.249.123.141/0", "'172.29.253.216/63901'"),
        ("10.249.123.141/-1", "'172.29.253.216/63901'"),
        ("10.249.123.141/65536", "'172.29.253.216/63901'"),
        ("10.249.123.141/2147483648", "'172.29.253.216/63901'"),
        ("10.249.123.141/63901/extra", "'172.29.253.216/63901'"),
        ("10.249.123.141/63901", "''"),
        ("10.249.123.141/63901", "'172.29.253.216'"),
        ("10.249.123.141/63901", "'/63901'"),
        ("10.249.123.141/63901", "'   /63901'"),
        ("10.249.123.141/63901", "'172.29.253.216/'"),
        ("10.249.123.141/63901", "'172.29.253.216/abc'"),
        ("10.249.123.141/63901", "'172.29.253.216/1.5'"),
        ("10.249.123.141/63901", "'172.29.253.216/0'"),
        ("10.249.123.141/63901", "'172.29.253.216/-1'"),
        ("10.249.123.141/63901", "'172.29.253.216/65536'"),
        ("10.249.123.141/63901", "'172.29.253.216/2147483648'"),
        ("10.249.123.141/63901", "'172.29.253.216/63901/extra'"),
        ("10.249.123.141/63901", "$null"),
        ("10.249.123.141/63901", "63901"),
        ("10.249.123.141/63901", "@('172.29.253.216/63901')"),
    ],
)
def test_malformed_registry_entries_fail_closed(tmp_path, name, value):
    result = run_registry_check(
        tmp_path,
        f"Add-FakeKey 'v4tov4' @{{{literal(name)}={value}}}\n" + """
$failure = $null
try { $null = @(Get-ProxyEntries) } catch { $failure = $_.Exception.Message }
if ($null -eq $failure) { throw 'Malformed registry entry was silently accepted or skipped' }
if ($failure -notlike '*portproxy*' -or $failure -notlike '*refusing changes*') {
    throw "Unexpected failure instead of registry validation: $failure"
}
""",
    )
    assert result.returncode == 0, result.stdout + result.stderr
