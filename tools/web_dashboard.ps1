$ErrorActionPreference = "SilentlyContinue"
$UDP_PORT  = 12345
$HTTP_PORT = 8080
$STATE = "$PSScriptRoot\alerts.json"

if (-not (Test-Path $STATE)) { '[]' | Out-File $STATE -Encoding UTF8 }

$udpJob = Start-Job -Name "SG-UDP" -ScriptBlock {
    param($port, $state)
    $u = New-Object System.Net.Sockets.UdpClient $port
    $u.Client.ReceiveTimeout = 800
    $r = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
    while ($true) {
        try {
            $d = $u.Receive([ref]$r)
            $m = [System.Text.Encoding]::UTF8.GetString($d)
            $t = (Get-Date).ToString("HH:mm:ss")
            $arr = @()
            if (Test-Path $state) { $arr = @(Get-Content $state -Raw | ConvertFrom-Json) }
            if ($null -eq $arr -or $arr.Count -eq 0) { $arr = @() }
            $arr += [pscustomobject]@{time=$t; from=$r.Address.ToString(); message=$m}
            if ($arr.Count -gt 50) { $arr = $arr | Select-Object -Last 50 }
            $arr | ConvertTo-Json -Depth 5 | Out-File $state -Encoding UTF8
            Write-Host "[$t] $($r.Address): $m" -ForegroundColor Green
        } catch { }
    }
} -ArgumentList $UDP_PORT, $STATE

Write-Host "SmartGuard Dashboard starting..." -ForegroundColor Cyan
Write-Host "  UDP :$UDP_PORT" -ForegroundColor Cyan
Write-Host "  Web : http://localhost:$HTTP_PORT" -ForegroundColor Yellow

$h = [System.Net.HttpListener]::new()
$h.Prefixes.Add("http://localhost:$HTTP_PORT/")
$h.Start()
Write-Host "Open browser: http://localhost:$HTTP_PORT (Ctrl+C to quit)"

try {
    while ($h.IsListening) {
        Receive-Job -Job $udpJob -Keep
        $ctx = $h.GetContext()
        $arr = @()
        if (Test-Path $STATE) { $arr = @(Get-Content $STATE -Raw | ConvertFrom-Json) }
        if ($null -eq $arr -or $arr.Count -eq 0) { $arr = @() }

        $last = if ($arr.Count -gt 0) { $arr[-1] } else { $null }
        $lastMsg = if ($last) { $last.message } else { "Waiting for STM32 alerts..." }
        $lastClass = if ($last -and $last.message -match "alert|intrusion|fire") { "last-alert fire" } else { "last-alert" }

        $rows = ""
        if ($arr.Count -eq 0) {
            $rows = "<div class='empty'>No messages yet. Trigger an alert to see data here...</div>"
        } else {
            foreach ($a in ($arr | Select-Object -Last 50)) {
                $c = if ($a.message -match "alert|intrusion|fire") { "alert-item warn" } else { "alert-item" }
                $rows += "<div class='$c'><span class='time'>$($a.time)</span><span class='from'>$($a.from)</span><span class='msg'>$($a.message)</span></div>"
            }
        }

        $html = @"
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta http-equiv="refresh" content="2">
<title>SmartGuard</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:sans-serif;background:#0f172a;color:#e2e8f0;min-height:100vh}
.hd{background:linear-gradient(135deg,#1e293b,#334155);padding:20px;text-align:center;border-bottom:2px solid #3b82f6}
.hd h1{font-size:22px;color:#60a5fa}.hd .s{font-size:12px;color:#94a3b8;margin-top:4px}
.w{max-width:760px;margin:20px auto;padding:0 16px}
.last-alert{background:#1e293b;padding:16px 20px;border-radius:12px;border:1px solid #334155;margin-bottom:20px}
.last-alert.fire{border-color:#ef4444;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(239,68,68,.4)}50%{box-shadow:0 0 16px 3px rgba(239,68,68,.25)}}
.last-alert h2{font-size:13px;color:#64748b;margin-bottom:6px}.last-alert .c{font-size:16px;font-weight:bold;word-break:break-all}
.history h2{font-size:13px;color:#64748b;margin-bottom:10px}
.alert-item{background:#1e293b;padding:10px 14px;border-radius:6px;margin-bottom:6px;border-left:3px solid #3b82f6;font-size:13px}
.alert-item.warn{border-left-color:#ef4444;background:#450a0a}
.time{color:#64748b;font-size:11px;margin-right:10px}
.from{color:#64748b;font-size:11px;margin-right:10px}
.empty{text-align:center;color:#64748b;padding:30px;font-size:13px}
</style></head><body>
<div class="hd"><h1>SmartGuard Dashboard</h1><div class="s">UDP :$UDP_PORT | Auto-refresh 2s</div></div>
<div class="w">
  <div class="$lastClass"><h2>Latest</h2><div class="c">$lastMsg</div></div>
  <div class="history"><h2>History</h2>$rows</div>
</div></body></html>
"@

        $buf = [System.Text.Encoding]::UTF8.GetBytes($html)
        $ctx.Response.ContentType = "text/html; charset=utf-8"
        $ctx.Response.ContentLength64 = $buf.Length
        $ctx.Response.OutputStream.Write($buf, 0, $buf.Length)
        $ctx.Response.Close()
    }
} finally {
    $h.Stop()
    Stop-Job $udpJob -ErrorAction SilentlyContinue
    Remove-Job $udpJob -ErrorAction SilentlyContinue
    Write-Host "Stopped." -ForegroundColor Yellow
}
