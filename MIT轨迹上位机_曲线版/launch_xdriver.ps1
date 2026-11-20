param([switch]$OfflinePreview, [switch]$TrajectoryOnly)
$ErrorActionPreference = 'Stop'
$xdriverPython = 'F:\人形机器人\安装调试软件\dh_tool\.venv\Scripts\pythonw.exe'
if (-not (Test-Path -LiteralPath $xdriverPython)) { throw 'dh_tool Python environment was not found.' }
$xdriverName = if ($TrajectoryOnly) { 'mit_trajectory' } else { 'xdriver' }
$xdriverEntry = if ($TrajectoryOnly) { 'mit_trajectory_host.py' } else { 'xdriver_console.py' }
$xdriverScript = Join-Path $PSScriptRoot $xdriverEntry
$xdriverLogDir = Join-Path $PSScriptRoot 'launch_logs'
New-Item -ItemType Directory -Path $xdriverLogDir -Force | Out-Null
$xdriverStamp = Get-Date -Format 'yyyyMMdd_HHmmss_fff'
$xdriverArguments = @('-X', 'utf8', ('"' + $xdriverScript + '"'))
if ($OfflinePreview) { $xdriverArguments += '--offline-preview' }
$xdriverLogPrefix = "${xdriverStamp}_${xdriverName}"
$xdriverProcess = Start-Process -FilePath $xdriverPython -ArgumentList $xdriverArguments -WorkingDirectory $PSScriptRoot -WindowStyle Normal -RedirectStandardOutput (Join-Path $xdriverLogDir ($xdriverLogPrefix + '_stdout.log')) -RedirectStandardError (Join-Path $xdriverLogDir ($xdriverLogPrefix + '_stderr.log')) -PassThru
Write-Output ('Window launched; serial remains closed. ProcessId=' + $xdriverProcess.Id)
