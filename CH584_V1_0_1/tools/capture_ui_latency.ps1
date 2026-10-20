[CmdletBinding()]
param(
    [string[]]$Ports = @('COM10', 'COM13'),
    [ValidateRange(300, 3000000)][int]$Baud = 115200,
    [ValidateRange(1, 300)][int]$Seconds = 30,
    [switch]$Plan
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$logDirectory = Join-Path $projectRoot 'obj/ui_latency_check'
$Ports = @($Ports | Select-Object -Unique)
foreach ($portName in $Ports) {
    if ($portName -notmatch '^COM[0-9]+$') { throw "Invalid port: $portName" }
}
if ($Ports.Count -eq 0) { throw 'At least one port is required.' }

# Plan mode never opens a serial handle or creates a file.
if ($Plan) {
    [pscustomobject]@{
        Ports = $Ports; Baud = $Baud; Seconds = $Seconds
        Mode = 'receive only, 8N1, no handshake, DTR/RTS disabled'
        LogDirectory = $logDirectory
    } | Format-List
    return
}

$handles = @()
$writer = $null
$counts = @{}
try {
    foreach ($portName in $Ports) {
        $port = [System.IO.Ports.SerialPort]::new($portName, $Baud,
            [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
        $handles += $port
        $port.Handshake = [System.IO.Ports.Handshake]::None
        $port.DtrEnable = $false
        $port.RtsEnable = $false
        $port.ReadTimeout = 200
        $port.Open()
        $counts[$portName] = 0
    }
    [void][System.IO.Directory]::CreateDirectory($logDirectory)
    $logPath = Join-Path $logDirectory ('serial_' + [DateTime]::UtcNow.ToString('yyyyMMdd_HHmmss_fffffff') + '.jsonl')
    $writer = [System.IO.StreamWriter]::new($logPath, $false, [System.Text.UTF8Encoding]::new($false))
    $writer.WriteLine((@{ kind = 'start'; utc = [DateTime]::UtcNow.ToString('o');
        ports = $Ports; baud = $Baud; seconds = $Seconds; transmit = $false
        timestampMeaning = 'host read time; not MCU event time' } | ConvertTo-Json -Compress))
    Write-Output "Receiving for $Seconds seconds. Move the menu cursor several times now."
    $elapsed = [System.Diagnostics.Stopwatch]::StartNew()
    $buffer = [byte[]]::new(4096)
    $total = 0
    while ($elapsed.Elapsed.TotalSeconds -lt $Seconds -and $total -lt 4194304) {
        foreach ($port in $handles) {
            $available = $port.BytesToRead
            if ($available -gt 0) {
                $read = $port.Read($buffer, 0, [Math]::Min($available, $buffer.Length))
                $counts[$port.PortName] += $read
                $total += $read
                $writer.WriteLine((@{ kind = 'data'; port = $port.PortName
                    elapsed_ms = [Math]::Round($elapsed.Elapsed.TotalMilliseconds, 3)
                    hex = [System.BitConverter]::ToString($buffer, 0, $read)
                    text = [System.Text.Encoding]::UTF8.GetString($buffer, 0, $read)
                } | ConvertTo-Json -Compress))
            }
        }
        [System.Threading.Thread]::Sleep(10)
    }
    $writer.WriteLine((@{ kind = 'end'; counts = $counts;
        byteLimitReached = ($total -ge 4194304) } | ConvertTo-Json -Compress))
    Write-Output "Log: $logPath"
    $counts | ConvertTo-Json -Compress | Write-Output
}
finally {
    if ($null -ne $writer) { $writer.Dispose() }
    foreach ($port in $handles) {
        if ($port.IsOpen) { $port.Close() }
        $port.Dispose()
    }
}
