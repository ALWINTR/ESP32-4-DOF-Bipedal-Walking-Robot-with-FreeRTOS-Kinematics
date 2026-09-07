$port = New-Object System.IO.Ports.SerialPort "COM3", 115200, "None", 8, "One"
$port.Open()

# Normal Boot Reset Sequence (GPIO0 = High, toggle EN)
$port.DtrEnable = $false   # GPIO0 High (Normal boot)
$port.RtsEnable = $true    # EN Low (Reset active)
Start-Sleep -Milliseconds 200
$port.RtsEnable = $false   # EN High (Release reset / Run)

# Wait 8 seconds for boot and WiFi connection
Start-Sleep -Seconds 8

$output = ""
while ($port.BytesToRead -gt 0) {
    $output += $port.ReadExisting()
}
$port.Close()
Write-Output $output
