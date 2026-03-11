param(
  [Parameter(Mandatory = $true)][string]$Firmware,
  [string]$CliPath = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
  [string]$Address = "0x08000000",
  [int]$PollMs = 500,
  [switch]$Once
)

function Require-File([string]$path, [string]$label) {
  if (-not (Test-Path $path)) {
    Write-Error "$label not found: $path"
    exit 2
  }
}

$Firmware = (Resolve-Path $Firmware).Path
Require-File $Firmware "Firmware"
Require-File $CliPath "STM32_Programmer_CLI"

$ext = [System.IO.Path]::GetExtension($Firmware).ToLowerInvariant()

function Test-Target {
  & $CliPath -c port=SWD -r32 0xE0042000 1 > $null 2>&1
  return ($LASTEXITCODE -eq 0)
}

function Flash-Once {
  if ($ext -eq ".hex") {
    $args = @("-c","port=SWD","-w",$Firmware,"-v","-rst")
  } elseif ($ext -eq ".bin") {
    $args = @("-c","port=SWD","-w",$Firmware,$Address,"-v","-rst")
  } else {
    Write-Error "Unsupported firmware extension: $ext (use .hex or .bin)"
    exit 3
  }

  & $CliPath @args
  return ($LASTEXITCODE -eq 0)
}

Write-Host "Waiting for target via SWD..."
while ($true) {
  if (Test-Target) {
    Write-Host "Target detected. Flashing..."
    if (Flash-Once) {
      Write-Host "Flash OK."
      if ($Once) { break }
      Write-Host "Remove target to program next. Waiting for disconnect..."
      while (Test-Target) { Start-Sleep -Milliseconds $PollMs }
      Write-Host "Disconnected. Waiting for next target..."
    } else {
      Write-Host "Flash failed. Retrying..."
      Start-Sleep -Milliseconds $PollMs
    }
  } else {
    Start-Sleep -Milliseconds $PollMs
  }
}

