# SmartGuard UDP Receiver v0.7 (PowerShell 版)
# 监听 12345 端口，接收 STM32 ESP8266 发来的告警 JSON
# 用法: 右键 -> 用 PowerShell 运行，或在 PowerShell 里执行: .\udp_receiver.ps1

$port = 12345
$udp = New-Object System.Net.Sockets.UdpClient
$udp.Client.ReceiveTimeout = 5000
$udp.Bind((New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, $port)))

Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "  SmartGuard UDP Receiver v0.7 (PowerShell)"      -ForegroundColor Cyan
Write-Host "  监听端口: $port"                                 -ForegroundColor Cyan
Write-Host "  本地 IP:  192.168.137.1 (热点网关)"              -ForegroundColor Cyan
Write-Host "  等待 STM32 告警消息... (Ctrl+C 退出)"            -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan

$remoteEP = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)

while ($true) {
    try {
        $data = $udp.Receive([ref]$remoteEP)
        $msg  = [System.Text.Encoding]::UTF8.GetString($data)
        $ts   = (Get-Date).ToString("HH:mm:ss")

        # 尝试解析 JSON
        try {
            $json = $msg | ConvertFrom-Json
            $mtype = $json.type
            if ($mtype -eq "alert") {
                Write-Host "[$ts] ALERT from $($remoteEP.Address): $($json.msg)" -ForegroundColor Red
            } elseif ($mtype -eq "data") {
                Write-Host "[$ts] DATA from $($remoteEP.Address): $($json.payload)" -ForegroundColor Green
            } else {
                Write-Host "[$ts] ??? from $($remoteEP.Address): $msg" -ForegroundColor Yellow
            }
        } catch {
            Write-Host "[$ts] RAW from $($remoteEP.Address): $msg" -ForegroundColor Gray
        }
    } catch [System.Net.Sockets.SocketException] {
        # ReceiveTimeout，循环继续
        continue
    } catch {
        Write-Host "Error: $_" -ForegroundColor Red
    }
}

$udp.Close()
