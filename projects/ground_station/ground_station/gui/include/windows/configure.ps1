# COPYRIGHT 2025 ESRI
#
# All rights reserved under the copyright laws of the United States
# and applicable international laws, treaties, and conventions.
#
# This material is licensed for use under the Esri Master License
# Agreement (MLA), and is bound by the terms of that agreement.
# You may redistribute and use this code without modification,
# provided you adhere to the terms of the MLA and include this
# copyright notice.
#
# See use restrictions at http://www.esri.com/legal/pdfs/mla_e204_e300/english
#
# For additional information, contact:
# Environmental Systems Research Institute, Inc.
# Attn: Contracts and Legal Services Department
# 380 New York Street
# Redlands, California, 92373
# USA

# configure.ps1
# This script is used to configure the Windows installer for the ArcGIS Maps SDK for Qt.

# Check if the script is running with admin privileges

param (
  [Alias("np")]
  [switch]$DontRunPostInstall = $false,

  [Alias("u")]
  [switch]$UninstallSDK = $false,

  [Alias("h")]
  [switch]$Help = $false,

  [Alias("a")]
  [switch]$AcceptEula = $false
)

function Log {
  param ([string]$Message)
  $timestampedMessage = "$((Get-Date).ToString()) - $Message"
  $timestampedMessage | Out-File -Append -FilePath $LogFile
}

$LogFile = "${env:TEMP}\configure.log"
$Version = "200.8.0"

# Get the directory where the configure.sh script is located.
$ScriptDir = (Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Definition) "qt$Version").Replace("\", "/")

# Define the configuration file path.
$ConfigFile = "${env:ALLUSERSPROFILE}\EsriRuntimeQt\ArcGIS Runtime SDK for Qt $Version.ini"

function Replace-InstallDir {
  param (
    [string]$File
  )
  if (Test-Path $File) {
    (Get-Content $File) -replace 'SDK_INSTALL_DIR = \".*\"', "SDK_INSTALL_DIR = `"$ScriptDir`"" | Set-Content $File
    Log "Replaced SDK_INSTALL_DIR in $File."
  }
  else {
    Log "Warning: $File does not exist."
  }
}

function Install {
  # Prompt user to accept the Master License Agreement before proceeding.
  if (-not $AcceptEula) {
    Write-Host "================================================================================"
    Write-Host "IMPORTANT: You must accept the Esri Master License Agreement before proceeding."
    Write-Host "The agreement can be found in the 'legal' folder as 'EULA.pdf'."
    do {
      $agreementResponse = Read-Host "Do you accept the terms of the Master License Agreement? (yes/no)"
      $agreementResponseLC = $agreementResponse.ToLower()
      if ($agreementResponseLC -eq "yes" -or $agreementResponseLC -eq "y") {
        break
      } elseif ($agreementResponseLC -eq "no" -or $agreementResponseLC -eq "n") {
        Write-Host "You must accept the Master License Agreement to proceed. Exiting."
        exit 1
      } else {
        Write-Host "Invalid response. Please answer 'yes' or 'no'."
      }
    } while ($true)
  }

  # Create the directory if it does not exist.
  $ConfigDir = Split-Path -Parent $ConfigFile
  if (-not (Test-Path $ConfigDir)) {
    New-Item -ItemType Directory -Path $ConfigDir | Out-Null
  }

  # Create the file and update InstallDir.
  try {
    "[Settings]" | Out-File -FilePath $ConfigFile -Encoding UTF8
    "InstallDir=`"$ScriptDir`"" | Out-File -Append -FilePath $ConfigFile -Encoding UTF8
    Log "Created or updated the configuration file."
  }
  catch {
    Log "Error: Failed to create or update the configuration file."
    exit 1
  }

  # Replace <INSTALLDIR> in the specified files.
  Replace-InstallDir "$ScriptDir\sdk\ideintegration\esri_runtime_qt.pri"
  Replace-InstallDir "$ScriptDir\sdk\ideintegration\arcgis_runtime_qml_cpp.pri"

  # Run post install script.
  if (-not $DontRunPostInstall) {
    Log "Running post install script."
    $PostInstallScript = Join-Path $ScriptDir "tools\postInstall.ps1"
    if (Test-Path $PostInstallScript) {
      try {
        & $PostInstallScript
        Log "Post install is successful."
      }
      catch {
        Log "Error: Failed to run post install script."
        exit 1
      }
    }
    else {
      Log "Error: Post install script not found."
      exit 1
    }
  }

  Write-Host "Configuration is complete. Please refer to the log file for more details: $LogFile"
}

function Uninstall {
  # Uninstall ArcGIS Maps SDK for Qt
  Log "Undoing post install script."
  $PostInstallScript = Join-Path $ScriptDir "tools\postInstall.ps1"
  if (Test-Path $PostInstallScript) {
    try {
      & $PostInstallScript -Uninstall
      Log "Post install undo is successful."
    }
    catch {
      Log "Error: Failed to undo post install script."
      exit 1
    }
  }
  else {
    Log "Error: Post install script not found."
    exit 1
  }

  Log "Removing hidden ini file."
  if (Test-Path $ConfigFile) {
    Remove-Item $ConfigFile
    Log "Configuration file removed."
  }
  else {
    Log "File does not exist, nothing to remove."
  }

  Write-Host "Uninstall configuration is complete. Please refer to the log file for more details: $LogFile"
}

if ($Help) {
  Write-Host "================================================================================"
  Write-Host "Example:"
  Write-Host ""
  Write-Host "This script is used to configure the ArcGIS Maps SDK for Qt and integrate documentation and templates into Qt Creator"
  Write-Host ""
  Write-Host "Usage: .\configure.ps1 [OPTION]..."
  Write-Host ""
  Write-Host "Options:"
  Write-Host "  -np DontRunPostInstall(switch)     Don't run the postInstall script(default: `$false)."
  Write-Host "  -u  UninstallSDK(switch)           Uninstall the SDK.(default: `$false)"
  Write-Host "  -h  Help(switch)                   Show this help message.(default: `$false)"
  Write-Host "  -a  AcceptEula(switch)             Accept the EULA for headless installs.(default: `$false)"
  Write-Host ""
  Write-Host "Example: ``configure.ps1 -np``         This will configure but not run the postInstall script."
  Write-Host "Example: ``configure.ps1 -u``          This will uninstall the SDK."
  Write-Host "================================================================================"
  exit 0
}

if ($UninstallSDK) {
  Uninstall
}
else {
  Install
}

# SIG # Begin signature block
# MIIpogYJKoZIhvcNAQcCoIIpkzCCKY8CAQExDzANBglghkgBZQMEAgEFADB5Bgor
# BgEEAYI3AgEEoGswaTA0BgorBgEEAYI3AgEeMCYCAwEAAAQQH8w7YFlLCE63JNLG
# KX7zUQIBAAIBAAIBAAIBAAIBADAxMA0GCWCGSAFlAwQCAQUABCCIMifblNraIMk6
# dGVtnNcc+TAICdL+sx5Ro8cZoI4YOaCCDlgwggawMIIEmKADAgECAhAIrUCyYNKc
# TJ9ezam9k67ZMA0GCSqGSIb3DQEBDAUAMGIxCzAJBgNVBAYTAlVTMRUwEwYDVQQK
# EwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5jb20xITAfBgNV
# BAMTGERpZ2lDZXJ0IFRydXN0ZWQgUm9vdCBHNDAeFw0yMTA0MjkwMDAwMDBaFw0z
# NjA0MjgyMzU5NTlaMGkxCzAJBgNVBAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2VydCwg
# SW5jLjFBMD8GA1UEAxM4RGlnaUNlcnQgVHJ1c3RlZCBHNCBDb2RlIFNpZ25pbmcg
# UlNBNDA5NiBTSEEzODQgMjAyMSBDQTEwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAw
# ggIKAoICAQDVtC9C0CiteLdd1TlZG7GIQvUzjOs9gZdwxbvEhSYwn6SOaNhc9es0
# JAfhS0/TeEP0F9ce2vnS1WcaUk8OoVf8iJnBkcyBAz5NcCRks43iCH00fUyAVxJr
# Q5qZ8sU7H/Lvy0daE6ZMswEgJfMQ04uy+wjwiuCdCcBlp/qYgEk1hz1RGeiQIXhF
# LqGfLOEYwhrMxe6TSXBCMo/7xuoc82VokaJNTIIRSFJo3hC9FFdd6BgTZcV/sk+F
# LEikVoQ11vkunKoAFdE3/hoGlMJ8yOobMubKwvSnowMOdKWvObarYBLj6Na59zHh
# 3K3kGKDYwSNHR7OhD26jq22YBoMbt2pnLdK9RBqSEIGPsDsJ18ebMlrC/2pgVItJ
# wZPt4bRc4G/rJvmM1bL5OBDm6s6R9b7T+2+TYTRcvJNFKIM2KmYoX7BzzosmJQay
# g9Rc9hUZTO1i4F4z8ujo7AqnsAMrkbI2eb73rQgedaZlzLvjSFDzd5Ea/ttQokbI
# YViY9XwCFjyDKK05huzUtw1T0PhH5nUwjewwk3YUpltLXXRhTT8SkXbev1jLchAp
# QfDVxW0mdmgRQRNYmtwmKwH0iU1Z23jPgUo+QEdfyYFQc4UQIyFZYIpkVMHMIRro
# OBl8ZhzNeDhFMJlP/2NPTLuqDQhTQXxYPUez+rbsjDIJAsxsPAxWEQIDAQABo4IB
# WTCCAVUwEgYDVR0TAQH/BAgwBgEB/wIBADAdBgNVHQ4EFgQUaDfg67Y7+F8Rhvv+
# YXsIiGX0TkIwHwYDVR0jBBgwFoAU7NfjgtJxXWRM3y5nP+e6mK4cD08wDgYDVR0P
# AQH/BAQDAgGGMBMGA1UdJQQMMAoGCCsGAQUFBwMDMHcGCCsGAQUFBwEBBGswaTAk
# BggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEEGCCsGAQUFBzAC
# hjVodHRwOi8vY2FjZXJ0cy5kaWdpY2VydC5jb20vRGlnaUNlcnRUcnVzdGVkUm9v
# dEc0LmNydDBDBgNVHR8EPDA6MDigNqA0hjJodHRwOi8vY3JsMy5kaWdpY2VydC5j
# b20vRGlnaUNlcnRUcnVzdGVkUm9vdEc0LmNybDAcBgNVHSAEFTATMAcGBWeBDAED
# MAgGBmeBDAEEATANBgkqhkiG9w0BAQwFAAOCAgEAOiNEPY0Idu6PvDqZ01bgAhql
# +Eg08yy25nRm95RysQDKr2wwJxMSnpBEn0v9nqN8JtU3vDpdSG2V1T9J9Ce7FoFF
# UP2cvbaF4HZ+N3HLIvdaqpDP9ZNq4+sg0dVQeYiaiorBtr2hSBh+3NiAGhEZGM1h
# mYFW9snjdufE5BtfQ/g+lP92OT2e1JnPSt0o618moZVYSNUa/tcnP/2Q0XaG3Ryw
# YFzzDaju4ImhvTnhOE7abrs2nfvlIVNaw8rpavGiPttDuDPITzgUkpn13c5Ubdld
# AhQfQDN8A+KVssIhdXNSy0bYxDQcoqVLjc1vdjcshT8azibpGL6QB7BDf5WIIIJw
# 8MzK7/0pNVwfiThV9zeKiwmhywvpMRr/LhlcOXHhvpynCgbWJme3kuZOX956rEnP
# LqR0kq3bPKSchh/jwVYbKyP/j7XqiHtwa+aguv06P0WmxOgWkVKLQcBIhEuWTatE
# QOON8BUozu3xGFYHKi8QxAwIZDwzj64ojDzLj4gLDb879M4ee47vtevLt/B3E+bn
# KD+sEq6lLyJsQfmCXBVmzGwOysWGw/YmMwwHS6DTBwJqakAwSEs0qFEgu60bhQji
# WQ1tygVQK+pKHJ6l/aCnHwZ05/LWUpD9r4VIIflXO7ScA+2GRfS0YW6/aOImYIbq
# yK+p/pQd52MbOoZWeE4wggegMIIFiKADAgECAhADpxl7f2uCQymlUYtCNDRyMA0G
# CSqGSIb3DQEBCwUAMGkxCzAJBgNVBAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2VydCwg
# SW5jLjFBMD8GA1UEAxM4RGlnaUNlcnQgVHJ1c3RlZCBHNCBDb2RlIFNpZ25pbmcg
# UlNBNDA5NiBTSEEzODQgMjAyMSBDQTEwHhcNMjQwODIxMDAwMDAwWhcNMjUwODIw
# MjM1OTU5WjCBpzELMAkGA1UEBhMCVVMxEzARBgNVBAgTCkNhbGlmb3JuaWExETAP
# BgNVBAcTCFJlZGxhbmRzMTcwNQYDVQQKEy5FbnZpcm9ubWVudGFsIFN5c3RlbXMg
# UmVzZWFyY2ggSW5zdGl0dXRlLCBJbmMuMTcwNQYDVQQDEy5FbnZpcm9ubWVudGFs
# IFN5c3RlbXMgUmVzZWFyY2ggSW5zdGl0dXRlLCBJbmMuMIICIjANBgkqhkiG9w0B
# AQEFAAOCAg8AMIICCgKCAgEAoaG/9yQNxC0jKE8U9WSt6N9kqqk23Htn0V9qLQxk
# KswZ5zuJS3z0BQp4gCw+2Uw1ORYQth4luwgUB6phcJw7j3fdjYcDnRjv7FIQG15N
# IdBfRi/Rc/SKetU6ff2aSRbCLMdZHhQHJJulvFZ0SIBGZvxF1tuH4dkWTUg78hh2
# aytoSe4zeQFubqZ2q8RMllDNuT+i5LjfiuHf/oUdkUmX/IXvhPzkM6e/ZY4Kl8d7
# D1w8+YdtXhVSKkFu5g8Itw/hP054sMoAZqZn7v6iRxaK9uV6ELTu0gdEQSx7zYnc
# fkSCUCxYn+0xldxeW6RgLzW3ubS11qaidGhvDbZCsFJseyROPIjCfcDW4VgHklgV
# m0xiP5UaWWRcm//liMAOfFGP3UvQnPl/5qsOLHoY6LlQBy26kID0k5wisXdykwAo
# yBJl5izFQkK9Qd3KCy2JTpits3jSEWM6Htbw0c9IZqD6Tj9f/CdzBEjfEeXVCfw4
# IVXx41QKuoJB438GByH6GUo1KC0B0AimJ5X23i9QXQYl/AI/9JOxUqUzGTX9cXOh
# xgqPvQgeWCbBfcA6zMoaZvDO+F4rKFVYXklhuzQueIOdwAHuPJIjcvc3ddLIlPN1
# +JSdGnXDqCIoIM0QbLWXNU4/wfqXLJqOjyPqud5TUrJKJfec9cOpuhM5A6KZe6sf
# dhsCAwEAAaOCAgMwggH/MB8GA1UdIwQYMBaAFGg34Ou2O/hfEYb7/mF7CIhl9E5C
# MB0GA1UdDgQWBBRDggEryLaf21m3PtYNvz5q33un/zA+BgNVHSAENzA1MDMGBmeB
# DAEEATApMCcGCCsGAQUFBwIBFhtodHRwOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMw
# DgYDVR0PAQH/BAQDAgeAMBMGA1UdJQQMMAoGCCsGAQUFBwMDMIG1BgNVHR8Ega0w
# gaowU6BRoE+GTWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydFRydXN0
# ZWRHNENvZGVTaWduaW5nUlNBNDA5NlNIQTM4NDIwMjFDQTEuY3JsMFOgUaBPhk1o
# dHRwOi8vY3JsNC5kaWdpY2VydC5jb20vRGlnaUNlcnRUcnVzdGVkRzRDb2RlU2ln
# bmluZ1JTQTQwOTZTSEEzODQyMDIxQ0ExLmNybDCBlAYIKwYBBQUHAQEEgYcwgYQw
# JAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmRpZ2ljZXJ0LmNvbTBcBggrBgEFBQcw
# AoZQaHR0cDovL2NhY2VydHMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0VHJ1c3RlZEc0
# Q29kZVNpZ25pbmdSU0E0MDk2U0hBMzg0MjAyMUNBMS5jcnQwCQYDVR0TBAIwADAN
# BgkqhkiG9w0BAQsFAAOCAgEAAhYQwGB40vub26Tm3w1BKoo+DVdG159DfLD9xWxc
# d2DtpDDP/+11/0DGuNKqQ2oFWatay0iRCFRRB+C+4Rw4wioYxCKqyvCIxfIruMiZ
# AbIXYzZ5d4WEOa8Y0uvRFK6WF1CWEw86bTgZWtOIGCMuHoWd53gKugn0DojiNqWo
# RV8svaHKmsy6QxWvcTshzmllhUmZ9hqpoA6IbUrUdBZtLwHjjTWN/eqdxPOdzZyU
# jGZAYVsakw4Cw7uwlv6zlccTjBMHafbUY1z/Njhsk21YE7LIG1DPrNha8D4qdQC1
# jfNJZeoIAND+flhQmc05peDvP1BuJkJ5LPUMcWhg883aBWKcnOM4o9OUYVs6MHjT
# BhJ0txH9nyYxLxmCwRP+yrAEfE8OLU1tpxpQdlxvjP5XcaDVmbOkVH5rE7ZnFdjH
# HGQJLB/i+S51e1pyEeseb8xVl+9Xmq1y0WgXPh4T5a8KvRaYbYwY6/h+jv3s5yJd
# +hQ/5Xc2ZThMzUtHBge7iAyyl/sZA307s+B6j5x4t3qgh95a9I9yLlFcrXeZjpQ0
# ewtB11dQFMMg7JtJqvVLTpTNViQkKkvjPJkBdpBGafXFq2x5C7WcvTVT9lAb1IOz
# 9uk4wepX6gPKiN2JpOtAZx5OfAnmvIYuN6v1OnHOOVlrs8UfyV3e+z3eTJDS347U
# AIsxghqgMIIanAIBATB9MGkxCzAJBgNVBAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2Vy
# dCwgSW5jLjFBMD8GA1UEAxM4RGlnaUNlcnQgVHJ1c3RlZCBHNCBDb2RlIFNpZ25p
# bmcgUlNBNDA5NiBTSEEzODQgMjAyMSBDQTECEAOnGXt/a4JDKaVRi0I0NHIwDQYJ
# YIZIAWUDBAIBBQCgfDAQBgorBgEEAYI3AgEMMQIwADAZBgkqhkiG9w0BCQMxDAYK
# KwYBBAGCNwIBBDAcBgorBgEEAYI3AgELMQ4wDAYKKwYBBAGCNwIBFTAvBgkqhkiG
# 9w0BCQQxIgQguu2syzzXrE8o+mTu+NQwxWTwTIWCnATOoEPDm9kwZywwDQYJKoZI
# hvcNAQEBBQAEggIAWoRcvwsLe6DVQ9hdd7ittevVM3uRrRa7tVoUgJ6h8tz8CXQd
# Tbt2j5S4/fokmFKXoexz1u4z44uGkmmrQoFEiFurwoka5i1xij4Y2QtZx2u0+6km
# B3ktpX+YIWIOHnsD+kbLKLUHBMVqzC+qtmiJTmW5hyKK43DZVtRevuLyhUMJHWCi
# 7OyEUwf7RjCGxM7ajZK1Dpv5BKxHdXgiL9Rko3iQJ7CUBf0R1F+lPFUu/9/YQ8h/
# 4ETcz+0r6hUKX3NEpAQug8G9oyYMRyFDOddT2yfiMc6XiQsPGEeOi7EWAjrpfmT3
# w+n1WCP4juX0m3MPWdBuof1FJs4CtJ7eEpnltsFrvJMaeC4ygD37aXOIq+rEGfl7
# HDNvwAb9/DnkHqpUXB+d0OGY5KOfNoNOFqP2ePIjP3nCjnFb+ycvPPL8HfMSp0XX
# cV47n7tH/v+vpYbxVdFdcta9sVWcPM6H5jM+lntlwCB9TOfsXEDwFc41L2leLzR1
# X51Mjj4V6jYsmerJxFzJSsYA+bPD6rsvZmTTKDnNmj+SDXcqCD6duv7LOelM2kwS
# q/0T3kQv8jteXALI5YQlufDE86ZdEbVgura0yWZ/ytQ1qj1M3qEKUfmC9PpT1+31
# x82dNymyBeZbKxEgJ6x7VlfElAVVcpL6VML/T5OM/xc7L2zZ+6c+wcil3Ryhghd2
# MIIXcgYKKwYBBAGCNwMDATGCF2IwghdeBgkqhkiG9w0BBwKgghdPMIIXSwIBAzEP
# MA0GCWCGSAFlAwQCAQUAMHcGCyqGSIb3DQEJEAEEoGgEZjBkAgEBBglghkgBhv1s
# BwEwMTANBglghkgBZQMEAgEFAAQgxSQMpizxsT95jd51r/rl/1kREIcI6+Ta235N
# aRCuLsQCEBsYdjOAUvVRJfx3iyCiPJ0YDzIwMjUwNzI1MjEyMzQzWqCCEzowggbt
# MIIE1aADAgECAhAKgO8YS43xBYLRxHanlXRoMA0GCSqGSIb3DQEBCwUAMGkxCzAJ
# BgNVBAYTAlVTMRcwFQYDVQQKEw5EaWdpQ2VydCwgSW5jLjFBMD8GA1UEAxM4RGln
# aUNlcnQgVHJ1c3RlZCBHNCBUaW1lU3RhbXBpbmcgUlNBNDA5NiBTSEEyNTYgMjAy
# NSBDQTEwHhcNMjUwNjA0MDAwMDAwWhcNMzYwOTAzMjM1OTU5WjBjMQswCQYDVQQG
# EwJVUzEXMBUGA1UEChMORGlnaUNlcnQsIEluYy4xOzA5BgNVBAMTMkRpZ2lDZXJ0
# IFNIQTI1NiBSU0E0MDk2IFRpbWVzdGFtcCBSZXNwb25kZXIgMjAyNSAxMIICIjAN
# BgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEA0EasLRLGntDqrmBWsytXum9R/4Zw
# CgHfyjfMGUIwYzKomd8U1nH7C8Dr0cVMF3BsfAFI54um8+dnxk36+jx0Tb+k+87H
# 9WPxNyFPJIDZHhAqlUPt281mHrBbZHqRK71Em3/hCGC5KyyneqiZ7syvFXJ9A72w
# zHpkBaMUNg7MOLxI6E9RaUueHTQKWXymOtRwJXcrcTTPPT2V1D/+cFllESviH8Yj
# oPFvZSjKs3SKO1QNUdFd2adw44wDcKgH+JRJE5Qg0NP3yiSyi5MxgU6cehGHr7zo
# u1znOM8odbkqoK+lJ25LCHBSai25CFyD23DZgPfDrJJJK77epTwMP6eKA0kWa3os
# Ae8fcpK40uhktzUd/Yk0xUvhDU6lvJukx7jphx40DQt82yepyekl4i0r8OEps/FN
# O4ahfvAk12hE5FVs9HVVWcO5J4dVmVzix4A77p3awLbr89A90/nWGjXMGn7FQhmS
# lIUDy9Z2hSgctaepZTd0ILIUbWuhKuAeNIeWrzHKYueMJtItnj2Q+aTyLLKLM0Mh
# eP/9w6CtjuuVHJOVoIJ/DtpJRE7Ce7vMRHoRon4CWIvuiNN1Lk9Y+xZ66lazs2kK
# FSTnnkrT3pXWETTJkhd76CIDBbTRofOsNyEhzZtCGmnQigpFHti58CSmvEyJcAlD
# VcKacJ+A9/z7eacCAwEAAaOCAZUwggGRMAwGA1UdEwEB/wQCMAAwHQYDVR0OBBYE
# FOQ7/PIx7f391/ORcWMZUEPPYYzoMB8GA1UdIwQYMBaAFO9vU0rp5AZ8esrikFb2
# L9RJ7MtOMA4GA1UdDwEB/wQEAwIHgDAWBgNVHSUBAf8EDDAKBggrBgEFBQcDCDCB
# lQYIKwYBBQUHAQEEgYgwgYUwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmRpZ2lj
# ZXJ0LmNvbTBdBggrBgEFBQcwAoZRaHR0cDovL2NhY2VydHMuZGlnaWNlcnQuY29t
# L0RpZ2lDZXJ0VHJ1c3RlZEc0VGltZVN0YW1waW5nUlNBNDA5NlNIQTI1NjIwMjVD
# QTEuY3J0MF8GA1UdHwRYMFYwVKBSoFCGTmh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNv
# bS9EaWdpQ2VydFRydXN0ZWRHNFRpbWVTdGFtcGluZ1JTQTQwOTZTSEEyNTYyMDI1
# Q0ExLmNybDAgBgNVHSAEGTAXMAgGBmeBDAEEAjALBglghkgBhv1sBwEwDQYJKoZI
# hvcNAQELBQADggIBAGUqrfEcJwS5rmBB7NEIRJ5jQHIh+OT2Ik/bNYulCrVvhREa
# fBYF0RkP2AGr181o2YWPoSHz9iZEN/FPsLSTwVQWo2H62yGBvg7ouCODwrx6ULj6
# hYKqdT8wv2UV+Kbz/3ImZlJ7YXwBD9R0oU62PtgxOao872bOySCILdBghQ/ZLcdC
# 8cbUUO75ZSpbh1oipOhcUT8lD8QAGB9lctZTTOJM3pHfKBAEcxQFoHlt2s9sXoxF
# izTeHihsQyfFg5fxUFEp7W42fNBVN4ueLaceRf9Cq9ec1v5iQMWTFQa0xNqItH3C
# PFTG7aEQJmmrJTV3Qhtfparz+BW60OiMEgV5GWoBy4RVPRwqxv7Mk0Sy4QHs7v9y
# 69NBqycz0BZwhB9WOfOu/CIJnzkQTwtSSpGGhLdjnQ4eBpjtP+XB3pQCtv4E5UCS
# Dag6+iX8MmB10nfldPF9SVD7weCC3yXZi/uuhqdwkgVxuiMFzGVFwYbQsiGnoa9F
# 5AaAyBjFBtXVLcKtapnMG3VH3EmAp/jsJ3FVF3+d1SVDTmjFjLbNFZUWMXuZyvgL
# fgyPehwJVxwC+UpX2MSey2ueIu9THFVkT+um1vshETaWyQo8gmBto/m3acaP9Qsu
# Lj3FNwFlTxq25+T4QwX9xa6ILs84ZPvmpovq90K8eWyG2N01c4IhSOxqt81nMIIG
# tDCCBJygAwIBAgIQDcesVwX/IZkuQEMiDDpJhjANBgkqhkiG9w0BAQsFADBiMQsw
# CQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cu
# ZGlnaWNlcnQuY29tMSEwHwYDVQQDExhEaWdpQ2VydCBUcnVzdGVkIFJvb3QgRzQw
# HhcNMjUwNTA3MDAwMDAwWhcNMzgwMTE0MjM1OTU5WjBpMQswCQYDVQQGEwJVUzEX
# MBUGA1UEChMORGlnaUNlcnQsIEluYy4xQTA/BgNVBAMTOERpZ2lDZXJ0IFRydXN0
# ZWQgRzQgVGltZVN0YW1waW5nIFJTQTQwOTYgU0hBMjU2IDIwMjUgQ0ExMIICIjAN
# BgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAtHgx0wqYQXK+PEbAHKx126NGaHS0
# URedTa2NDZS1mZaDLFTtQ2oRjzUXMmxCqvkbsDpz4aH+qbxeLho8I6jY3xL1IusL
# opuW2qftJYJaDNs1+JH7Z+QdSKWM06qchUP+AbdJgMQB3h2DZ0Mal5kYp77jYMVQ
# XSZH++0trj6Ao+xh/AS7sQRuQL37QXbDhAktVJMQbzIBHYJBYgzWIjk8eDrYhXDE
# pKk7RdoX0M980EpLtlrNyHw0Xm+nt5pnYJU3Gmq6bNMI1I7Gb5IBZK4ivbVCiZv7
# PNBYqHEpNVWC2ZQ8BbfnFRQVESYOszFI2Wv82wnJRfN20VRS3hpLgIR4hjzL0hpo
# YGk81coWJ+KdPvMvaB0WkE/2qHxJ0ucS638ZxqU14lDnki7CcoKCz6eum5A19WZQ
# HkqUJfdkDjHkccpL6uoG8pbF0LJAQQZxst7VvwDDjAmSFTUms+wV/FbWBqi7fTJn
# jq3hj0XbQcd8hjj/q8d6ylgxCZSKi17yVp2NL+cnT6Toy+rN+nM8M7LnLqCrO2JP
# 3oW//1sfuZDKiDEb1AQ8es9Xr/u6bDTnYCTKIsDq1BtmXUqEG1NqzJKS4kOmxkYp
# 2WyODi7vQTCBZtVFJfVZ3j7OgWmnhFr4yUozZtqgPrHRVHhGNKlYzyjlroPxul+b
# gIspzOwbtmsgY1MCAwEAAaOCAV0wggFZMBIGA1UdEwEB/wQIMAYBAf8CAQAwHQYD
# VR0OBBYEFO9vU0rp5AZ8esrikFb2L9RJ7MtOMB8GA1UdIwQYMBaAFOzX44LScV1k
# TN8uZz/nupiuHA9PMA4GA1UdDwEB/wQEAwIBhjATBgNVHSUEDDAKBggrBgEFBQcD
# CDB3BggrBgEFBQcBAQRrMGkwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3NwLmRpZ2lj
# ZXJ0LmNvbTBBBggrBgEFBQcwAoY1aHR0cDovL2NhY2VydHMuZGlnaWNlcnQuY29t
# L0RpZ2lDZXJ0VHJ1c3RlZFJvb3RHNC5jcnQwQwYDVR0fBDwwOjA4oDagNIYyaHR0
# cDovL2NybDMuZGlnaWNlcnQuY29tL0RpZ2lDZXJ0VHJ1c3RlZFJvb3RHNC5jcmww
# IAYDVR0gBBkwFzAIBgZngQwBBAIwCwYJYIZIAYb9bAcBMA0GCSqGSIb3DQEBCwUA
# A4ICAQAXzvsWgBz+Bz0RdnEwvb4LyLU0pn/N0IfFiBowf0/Dm1wGc/Do7oVMY2mh
# XZXjDNJQa8j00DNqhCT3t+s8G0iP5kvN2n7Jd2E4/iEIUBO41P5F448rSYJ59Ib6
# 1eoalhnd6ywFLerycvZTAz40y8S4F3/a+Z1jEMK/DMm/axFSgoR8n6c3nuZB9BfB
# wAQYK9FHaoq2e26MHvVY9gCDA/JYsq7pGdogP8HRtrYfctSLANEBfHU16r3J05qX
# 3kId+ZOczgj5kjatVB+NdADVZKON/gnZruMvNYY2o1f4MXRJDMdTSlOLh0HCn2cQ
# LwQCqjFbqrXuvTPSegOOzr4EWj7PtspIHBldNE2K9i697cvaiIo2p61Ed2p8xMJb
# 82Yosn0z4y25xUbI7GIN/TpVfHIqQ6Ku/qjTY6hc3hsXMrS+U0yy+GWqAXam4ToW
# d2UQ1KYT70kZjE4YtL8Pbzg0c1ugMZyZZd/BdHLiRu7hAWE6bTEm4XYRkA6Tl4KS
# FLFk43esaUeqGkH/wyW4N7OigizwJWeukcyIPbAvjSabnf7+Pu0VrFgoiovRDiyx
# 3zEdmcif/sYQsfch28bZeUz2rtY/9TCA6TD8dC3JE3rYkrhLULy7Dc90G6e8Blqm
# yIjlgp2+VqsS9/wQD7yFylIz0scmbKvFoW2jNrbM1pD2T7m3XDCCBY0wggR1oAMC
# AQICEA6bGI750C3n79tQ4ghAGFowDQYJKoZIhvcNAQEMBQAwZTELMAkGA1UEBhMC
# VVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3LmRpZ2ljZXJ0
# LmNvbTEkMCIGA1UEAxMbRGlnaUNlcnQgQXNzdXJlZCBJRCBSb290IENBMB4XDTIy
# MDgwMTAwMDAwMFoXDTMxMTEwOTIzNTk1OVowYjELMAkGA1UEBhMCVVMxFTATBgNV
# BAoTDERpZ2lDZXJ0IEluYzEZMBcGA1UECxMQd3d3LmRpZ2ljZXJ0LmNvbTEhMB8G
# A1UEAxMYRGlnaUNlcnQgVHJ1c3RlZCBSb290IEc0MIICIjANBgkqhkiG9w0BAQEF
# AAOCAg8AMIICCgKCAgEAv+aQc2jeu+RdSjwwIjBpM+zCpyUuySE98orYWcLhKac9
# WKt2ms2uexuEDcQwH/MbpDgW61bGl20dq7J58soR0uRf1gU8Ug9SH8aeFaV+vp+p
# VxZZVXKvaJNwwrK6dZlqczKU0RBEEC7fgvMHhOZ0O21x4i0MG+4g1ckgHWMpLc7s
# Xk7Ik/ghYZs06wXGXuxbGrzryc/NrDRAX7F6Zu53yEioZldXn1RYjgwrt0+nMNlW
# 7sp7XeOtyU9e5TXnMcvak17cjo+A2raRmECQecN4x7axxLVqGDgDEI3Y1DekLgV9
# iPWCPhCRcKtVgkEy19sEcypukQF8IUzUvK4bA3VdeGbZOjFEmjNAvwjXWkmkwuap
# oGfdpCe8oU85tRFYF/ckXEaPZPfBaYh2mHY9WV1CdoeJl2l6SPDgohIbZpp0yt5L
# HucOY67m1O+SkjqePdwA5EUlibaaRBkrfsCUtNJhbesz2cXfSwQAzH0clcOP9yGy
# shG3u3/y1YxwLEFgqrFjGESVGnZifvaAsPvoZKYz0YkH4b235kOkGLimdwHhD5QM
# IR2yVCkliWzlDlJRR3S+Jqy2QXXeeqxfjT/JvNNBERJb5RBQ6zHFynIWIgnffEx1
# P2PsIV/EIFFrb7GrhotPwtZFX50g/KEexcCPorF+CiaZ9eRpL5gdLfXZqbId5RsC
# AwEAAaOCATowggE2MA8GA1UdEwEB/wQFMAMBAf8wHQYDVR0OBBYEFOzX44LScV1k
# TN8uZz/nupiuHA9PMB8GA1UdIwQYMBaAFEXroq/0ksuCMS1Ri6enIZ3zbcgPMA4G
# A1UdDwEB/wQEAwIBhjB5BggrBgEFBQcBAQRtMGswJAYIKwYBBQUHMAGGGGh0dHA6
# Ly9vY3NwLmRpZ2ljZXJ0LmNvbTBDBggrBgEFBQcwAoY3aHR0cDovL2NhY2VydHMu
# ZGlnaWNlcnQuY29tL0RpZ2lDZXJ0QXNzdXJlZElEUm9vdENBLmNydDBFBgNVHR8E
# PjA8MDqgOKA2hjRodHRwOi8vY3JsMy5kaWdpY2VydC5jb20vRGlnaUNlcnRBc3N1
# cmVkSURSb290Q0EuY3JsMBEGA1UdIAQKMAgwBgYEVR0gADANBgkqhkiG9w0BAQwF
# AAOCAQEAcKC/Q1xV5zhfoKN0Gz22Ftf3v1cHvZqsoYcs7IVeqRq7IviHGmlUIu2k
# iHdtvRoU9BNKei8ttzjv9P+Aufih9/Jy3iS8UgPITtAq3votVs/59PesMHqai7Je
# 1M/RQ0SbQyHrlnKhSLSZy51PpwYDE3cnRNTnf+hZqPC/Lwum6fI0POz3A8eHqNJM
# QBk1RmppVLC4oVaO7KTVPeix3P0c2PR3WlxUjG/voVA9/HYJaISfb8rbII01YBwC
# A8sgsKxYoA5AY8WYIsGyWfVVa88nq2x2zm8jLfR+cWojayL/ErhULSd+2DrZ8LaH
# lv1b0VysGMNNn3O3AamfV6peKOK5lDGCA3wwggN4AgEBMH0waTELMAkGA1UEBhMC
# VVMxFzAVBgNVBAoTDkRpZ2lDZXJ0LCBJbmMuMUEwPwYDVQQDEzhEaWdpQ2VydCBU
# cnVzdGVkIEc0IFRpbWVTdGFtcGluZyBSU0E0MDk2IFNIQTI1NiAyMDI1IENBMQIQ
# CoDvGEuN8QWC0cR2p5V0aDANBglghkgBZQMEAgEFAKCB0TAaBgkqhkiG9w0BCQMx
# DQYLKoZIhvcNAQkQAQQwHAYJKoZIhvcNAQkFMQ8XDTI1MDcyNTIxMjM0M1owKwYL
# KoZIhvcNAQkQAgwxHDAaMBgwFgQU3WIwrIYKLTBr2jixaHlSMAf7QX4wLwYJKoZI
# hvcNAQkEMSIEIPOOQH2yxamOxAZXPRNyKGjn1UF2ChDd6hobLtZWCLbzMDcGCyqG
# SIb3DQEJEAIvMSgwJjAkMCIEIEqgP6Is11yExVyTj4KOZ2ucrsqzP+NtJpqjNPFG
# EQozMA0GCSqGSIb3DQEBAQUABIICAJ23XA16WmFoBbxrj64CyqviW+uBuPJXKmc9
# e4q6rPTKZHdIYq38mNntH7KKtEK/j8jd1rCMKlIh++TnqUL4IkRGD9KjnjfmV9kI
# /JBRHTRMLvRZxRGB25ZjnEmUe9cadqyuJtNs1oq7SqoiJGkj2zzinlPyReglMSZI
# lJcIeVnH5G2fvgqk5vRE08BZ6WPqFfei2STTefFeZUc2dtpr66Tj5OcETmZ2+gkJ
# rVsLebUCp0yw+Wy4DLkkFp6lMPVvsXmra3GzSxuzKZx3W0/7sXa+wLlHdXZ95pDu
# 7c5XPyvJgpLreeHWcXuj6UjM/ogQcz7gJMQToH70wrN9ZIclHEsVVEkKKqVL4mjN
# nszteh0XrlcF0yLFtZZ3E07K6OoDhNNBR+w0nOm22tDaKvTc1CrL6XX/NpQHDN82
# U5oWxhJaaBsNv4c2mwXZmR2gUt+mJiPGtIDcGiFJWOrA9/6jd/ECTUVLdqEEP+Ry
# wDlNr1yHDbMKUlxLqsyWC/DaE2DnnO+jFptpS+lIVH/ljhgnRNLuX6ht2bmThA4L
# k8fSnD5IEodrgLT81kG8bbkAFII+WXz/yw3QJ6Lg6OjIdA3qF+9MKgVFQ6DRQQxO
# eIyU9NDVA8ctvVyRrnFDJxiTUxn+oCWANLLaJLO67ls0CUVfdgsVC22ZNcz8WnhC
# 6oodw0OX
# SIG # End signature block
