param(
  [Parameter(Mandatory=$true)][ValidateSet(1,2)][int]$Bot,
  [string]$Port,
  [switch]$CompileOnly
)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
Push-Location $taskRoot
try {
  $taskCli = Join-Path $taskRoot '.tools/arduino-cli.exe'
  if (!(Test-Path -LiteralPath $taskCli)) { $taskCli = 'arduino-cli' }
  $taskConfig = Join-Path $taskRoot 'arduino-cli.yaml'
  $taskHeader = "firmware/bot/bot_config_bot$Bot.h"
  if (!(Test-Path -LiteralPath $taskHeader)) { throw "Run scripts/configure_bot.py --bot $Bot first" }
  Copy-Item -LiteralPath $taskHeader -Destination firmware/bot/bot_config.h -Force
  & $taskCli --config-file $taskConfig compile --fqbn esp8266:esp8266:d1_mini --output-dir ".build/bot$Bot" firmware/bot
  if ($LASTEXITCODE -ne 0) { throw 'Firmware compilation failed' }
  if (!$CompileOnly) {
    if (!$Port) { throw 'Specify -Port COMn from the USB serial board (not Bluetooth)' }
    $taskUploadLog = & $taskCli --config-file $taskConfig upload --fqbn esp8266:esp8266:d1_mini --port $Port --input-dir ".build/bot$Bot" firmware/bot 2>&1
    $taskUploadExit = $LASTEXITCODE
    $taskUploadLog | Write-Output
    # Some CLI builds returned zero even when esptool could not open the port.
    if ($taskUploadExit -ne 0 -or ($taskUploadLog -join "`n") -notmatch 'Hash of data verified') {
      throw 'Firmware upload not verified. Close the serial monitor, check the USB port, then retry.'
    }
  }
} finally { Pop-Location }
