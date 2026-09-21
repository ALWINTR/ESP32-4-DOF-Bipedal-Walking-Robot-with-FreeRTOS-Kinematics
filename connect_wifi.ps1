$ssid = "ESP32-Biped-Robot"
$password = "12345678"
$xml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
	<name>$ssid</name>
	<SSIDConfig>
		<SSID>
			<name>$ssid</name>
		</SSID>
	</SSIDConfig>
	<connectionType>ESS</connectionType>
	<connectionMode>manual</connectionMode>
	<MSM>
		<security>
			<authEncryption>
				<authentication>WPA2PSK</authentication>
				<encryption>AES</encryption>
				<useOneX>false</useOneX>
			</authEncryption>
			<sharedKey>
				<keyType>passPhrase</keyType>
				<protected>false</protected>
				<keyMaterial>$password</keyMaterial>
			</sharedKey>
		</security>
	</MSM>
</WLANProfile>
"@

$xmlPath = Join-Path $env:TEMP "wifi_profile.xml"
Set-Content -Path $xmlPath -Value $xml
netsh wlan add profile filename=$xmlPath
Start-Sleep -Seconds 1
netsh wlan connect name=$ssid
Start-Sleep -Seconds 5
netsh wlan show interfaces
