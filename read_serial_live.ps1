$port = New-Object System.IO.Ports.SerialPort "COM3", 115200, "None", 8, "One"
# Prevent automatic hardware reset on port open by disabling DTR/RTS init
$port.DtrEnable = $false
$port.RtsEnable = $false
$port.Open()

# Read live logs for 6 seconds
Start-Sleep -Seconds 6

$output = ""
while ($port.BytesToRead -gt 0) {
    $output += $port.ReadExisting()
}
$port.Close()
Write-Output $output
