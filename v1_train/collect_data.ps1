# ============================================================
#  SmartGuard TinyML 数据采集脚本 v2.1 (PowerShell 5 兼容)
#  监听 ESP8266 UDP(12345) + 可选串口，存 CSV
#
#  用法:
#    .\collect_data.ps1                    # 只收 UDP
#    .\collect_data.ps1 -COM COM6          # UDP + 串口
#    .\collect_data.ps1 -Out sensor.csv    # 指定输出
#  Ctrl+C 停止
# ============================================================

param(
    [string]$OutFile = "sensor_data.csv",
    [int]$UDPPort    = 12345,
    [string]$COM     = ""
)

$ErrorActionPreference = "Continue"

# CSV 表头
$header = "ts_ms,tick,light,distance_cm,temp_c,humi_pct,mode,alert,src"
Set-Content -Path $OutFile -Value $header -Encoding UTF8

Write-Host "============================================"
Write-Host " SmartGuard Data Collector"
Write-Host "============================================"
Write-Host " Output : $OutFile"
Write-Host " UDP    : port $UDPPort"
if ($COM) { Write-Host " Serial : $COM 115200" }
Write-Host " Started at $(Get-Date -Format 'HH:mm:ss')"
Write-Host "--------------------------------------------"

# UDP —— PowerShell 5 构造函数直接传端口完成 Bind
$listener = New-Object System.Net.Sockets.UdpClient $UDPPort
$listener.Client.SetSocketOption(
    [System.Net.Sockets.SocketOptionLevel]::Socket,
    [System.Net.Sockets.SocketOptionName]::ReceiveTimeout, 500)
$anyEP = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, $UDPPort)

# 串口 (可选)
$serial = $null
if ($COM) {
    try {
        $serial = New-Object System.IO.Ports.SerialPort $COM, 115200, None, 8, one
        $serial.ReadTimeout = 300
        $serial.Open()
        Write-Host "[OK] Serial $COM opened" -ForegroundColor Green
    } catch {
        Write-Host "[FAIL] Serial: $_" -ForegroundColor Red
    }
}

$rows  = 0
$start = Get-Date

try {
    while ($true) {
        # --- UDP ---
        try {
            $buf = $listener.Receive([ref]$anyEP)
            $msg = [System.Text.Encoding]::UTF8.GetString($buf).Trim()
            $ms  = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()

            if ($msg -match '^\d+,\d+,\d+') {
                $row = "$ms,$msg,UDP"
                Add-Content $OutFile $row -Encoding UTF8
                $rows++
                Write-Host "[UDP] $msg"
            }
        } catch [System.Net.Sockets.SocketException] { }

        # --- Serial ---
        if ($serial -and $serial.IsOpen) {
            try {
                $line = $serial.ReadLine()
                if ($line -and $line -match '^\d+,\d+,\d+') {
                    $ms  = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
                    $row = "$ms,$line,SERIAL"
                    Add-Content $OutFile $row -Encoding UTF8
                    $rows++
                    Write-Host "[SER] $line"
                }
            } catch [System.TimeoutException] { }
        }
    }
} finally {
    $dur = (Get-Date) - $start
    Write-Host "--------------------------------------------"
    Write-Host "Stopped. Duration: $($dur.TotalMinutes.ToString('F1')) min"
    Write-Host "Rows collected:   $rows"
    Write-Host "Saved to:         $OutFile"
    $listener.Close()
    if ($serial) { $serial.Close() }
}
