# ==========================================================================
# SmartGuard TinyML Data Collection Script
# ==========================================================================
# Usage:
#   powershell -ExecutionPolicy Bypass -File collect.ps1
#   powershell -ExecutionPolicy Bypass -File collect.ps1 -PortName COM3
#   powershell -ExecutionPolicy Bypass -File collect.ps1 -MaxSamples 500
#
# Features:
#   1. Opens STM32 serial port (COM8, 115200)
#   2. Captures lines starting with "ML,"
#   3. Appends to sensor_data.csv (preserves old data!)
#   4. Press 1/2 to manually override label on next sample
#   5. Press q to quit and save
# ==========================================================================

param(
    [string]$PortName    = "COM8",
    [int]   $BaudRate    = 115200,
    [string]$OutputFile  = "sensor_data.csv",
    [int]   $MaxSamples  = 0
)

try { Add-Type -AssemblyName System.IO.Ports } catch {
    Write-Host "ERROR: need .NET 4.x+ (System.IO.Ports not found)" -ForegroundColor Red
    exit 1
}

$global:serial         = $null
$global:totalCount     = 0
$global:normalCount    = 0
$global:abnormalCount  = 0
$global:manualLabel    = -1
$global:stopFlag       = $false

$CSV_HEADER = "label,light,dist,temp,humi,mq2,light_dev,dist_delta,temp_dev,humi_dev,mq2_dev"

function Show-Banner {
    Clear-Host
    Write-Host ""
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host "  SmartGuard TinyML Data Collector            " -ForegroundColor Cyan
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host "  Port  : $PortName @ $BaudRate"
    Write-Host "  File  : $OutputFile"
    Write-Host "  Goal  : 300 normal + 200 abnormal (500+)"
    Write-Host "----------------------------------------------" -ForegroundColor Cyan
    Write-Host "  Keys:"
    Write-Host "    1 = mark NEXT as normal (label=0)"
    Write-Host "    2 = mark NEXT as abnormal (label=1)"
    Write-Host "    m = toggle manual/auto mode"
    Write-Host "    s = print statistics"
    Write-Host "    q = quit and save"
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host ""
}

function Show-Status {
    $mode = if ($global:manualLabel -eq -1) { "auto" }
            elseif ($global:manualLabel -eq 0) { "manual->NEXT=0" }
            else { "manual->NEXT=1" }
    $pctN = if ($global:totalCount -gt 0) { [math]::Round($global:normalCount * 100.0 / $global:totalCount, 1) } else { 0 }
    $pctA = if ($global:totalCount -gt 0) { [math]::Round($global:abnormalCount * 100.0 / $global:totalCount, 1) } else { 0 }
    Write-Host ("  [{0}] total={1}  normal={2}({3}%)  abnormal={4}({5}%)" -f $mode, $global:totalCount, $global:normalCount, $pctN, $global:abnormalCount, $pctA)
}

function Ensure-Header {
    if (-not (Test-Path $OutputFile)) {
        Set-Content -Path $OutputFile -Value $CSV_HEADER -Encoding UTF8
        Write-Host "OK: created $OutputFile with header" -ForegroundColor Green
    } else {
        $lines = (Get-Content $OutputFile).Count - 1
        Write-Host "OK: appending to existing $OutputFile ($lines data rows)" -ForegroundColor Yellow
    }
}

function Process-Line([string]$line) {
    if ($line -notmatch '^ML,') { return }
    $csv = $line.Substring(3).Trim()
    if ([string]::IsNullOrWhiteSpace($csv)) { return }
    
    $fields = $csv -split ','
    if ($fields.Count -lt 11) {
        Write-Host ("  WARN: only {0} fields: {1}" -f $fields.Count, $line) -ForegroundColor DarkYellow
        return
    }
    
    if ($global:manualLabel -ne -1) {
        $fields[0] = "$($global:manualLabel)"
        Write-Host ("  MANUAL OVERRIDE -> label={0}" -f $global:manualLabel) -ForegroundColor Magenta
        $global:manualLabel = -1
    }
    
    $label = [int]$fields[0]
    $out = $fields -join ','
    Add-Content -Path $OutputFile -Value $out -Encoding UTF8
    
    $global:totalCount++
    if ($label -eq 0) { $global:normalCount++ } else { $global:abnormalCount++ }
    
    if ($global:totalCount % 5 -eq 0) { Show-Status }
}

function Open-Serial {
    Write-Host ("Opening {0} @ {1}baud ..." -f $PortName, $BaudRate) -ForegroundColor Yellow
    try {
        $global:serial = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, "None", 8, "one"
        $global:serial.ReadTimeout = 100
        $global:serial.Open()
        Start-Sleep -Milliseconds 500
        Write-Host "OK: serial port opened" -ForegroundColor Green
    } catch {
        Write-Host "ERROR: cannot open serial port: $_" -ForegroundColor Red
        Write-Host "  -> Close Serial Assistant / check port number / replug USB" -ForegroundColor Yellow
        exit 1
    }
}

function Close-Serial {
    if ($global:serial -and $global:serial.IsOpen) {
        $global:serial.Close()
        Write-Host "Serial port closed" -ForegroundColor Gray
    }
}

Show-Banner
Ensure-Header
Open-Serial
Write-Host ""
Write-Host "Waiting for data... press q to quit, 1/2 to override label" -ForegroundColor Cyan
Write-Host ""

$buffer = ""

try {
    while (-not $global:stopFlag) {
        try {
            $bytes = New-Object byte[] $global:serial.BytesToRead
            if ($bytes.Length -gt 0) {
                [void]$global:serial.Read($bytes, 0, $bytes.Length)
                $text = [System.Text.Encoding]::UTF8.GetString($bytes)
                $buffer += $text
            }
        } catch [System.TimeoutException] {
        } catch {
            Write-Host ("WARN: serial read error, reconnecting... {0}" -f $_) -ForegroundColor Yellow
            Close-Serial
            Start-Sleep -Seconds 2
            Open-Serial
            $buffer = ""
            continue
        }

        while ($buffer -match "`n") {
            $idx = $buffer.IndexOf("`n")
            $line = $buffer.Substring(0, $idx).TrimEnd("`r")
            $buffer = $buffer.Substring($idx + 1)
            if ($line.Length -gt 0) { Process-Line $line }
        }

        if ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            switch ($key.Key) {
                "D1" {
                    $global:manualLabel = 0
                    Write-Host "  SET NEXT LABEL = 0 (normal)" -ForegroundColor Green
                }
                "D2" {
                    $global:manualLabel = 1
                    Write-Host "  SET NEXT LABEL = 1 (abnormal)" -ForegroundColor Red
                }
                "M" {
                    if ($global:manualLabel -eq -1) {
                        $global:manualLabel = 0
                        Write-Host "  MODE -> manual (press 1/2 to mark)" -ForegroundColor Magenta
                    } else {
                        $global:manualLabel = -1
                        Write-Host "  MODE -> auto (follow STM32)" -ForegroundColor Green
                    }
                }
                "S" { Show-Status }
                "Q" {
                    Write-Host "Quitting..." -ForegroundColor Cyan
                    $global:stopFlag = $true
                }
            }
        }

        if ($MaxSamples -gt 0 -and $global:totalCount -ge $MaxSamples) {
            Write-Host ("Reached target {0} samples, auto-quit" -f $MaxSamples) -ForegroundColor Green
            $global:stopFlag = $true
        }

        Start-Sleep -Milliseconds 50
    }
} finally {
    Close-Serial
    Write-Host ""
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host "  Collection Complete!" -ForegroundColor Green
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host "  File:    $OutputFile"
    Write-Host "  Total:   $($global:totalCount)"
    Write-Host "  Normal:  $($global:normalCount)"
    Write-Host "  Abnormal:$($global:abnormalCount)"
    if ($global:totalCount -gt 0) {
        $abnPct = [math]::Round($global:abnormalCount * 100.0 / $global:totalCount, 1)
        Write-Host ("  Abnormal %: {0}%" -f $abnPct)
        if ($global:abnormalCount -lt 10) {
            Write-Host "  WARNING: too few abnormal samples! Wave hand near HCSR04 or flash light sensor." -ForegroundColor Yellow
        }
    }
    Write-Host ""
    Write-Host "Next step:"
    Write-Host "  python train_sklearn.py --data $OutputFile" -ForegroundColor Yellow
    Write-Host ""
}
