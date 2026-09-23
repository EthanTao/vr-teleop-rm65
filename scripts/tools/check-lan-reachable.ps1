# 探测某个 LAN 地址:端口是否可达，供 ~/.ssh/config 的 Match exec 做「实验室/宿舍」自动切换。
#
# 用法:
#   powershell -NoProfile -ExecutionPolicy Bypass -File check-lan-reachable.ps1 192.168.0.115 22
#
# 退出码:
#   0 = 可达（在实验室内网，走 LAN 直连）
#   1 = 不可达（大概率不在实验室，让 ssh 回退到 Tailscale 地址）
#
# 注意: ssh 的 Match exec 只在退出码为 0 时才应用该 Host 块，且要求脚本输出为空，
# 所以这里所有输出都丢进 $null，只用退出码表达结果。

param(
    [Parameter(Mandatory = $true)][string]$TargetHost,
    [Parameter(Mandatory = $false)][int]$Port = 22,
    [Parameter(Mandatory = $false)][int]$TimeoutMs = 2500
)

$client = New-Object System.Net.Sockets.TcpClient
try {
    $async = $client.BeginConnect($TargetHost, $Port, $null, $null)
    if (-not $async.AsyncWaitHandle.WaitOne($TimeoutMs, $false)) {
        exit 1
    }
    $client.EndConnect($async)
    exit 0
} catch {
    exit 1
} finally {
    $client.Close()
}
