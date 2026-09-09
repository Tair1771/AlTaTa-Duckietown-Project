# Windows entry point. Without -Go this performs only the read-only startup check.
param(
    [string]$Label = 'sharp-right',
    [ValidateRange(0.1, 15.0)][double]$Duration = 15.0,
    [switch]$GroundRightPivot,
    [switch]$GroundRollingRight,
    [switch]$GroundStrongRollingRight,
    [switch]$GroundEqualWheels,
    [ValidateSet('straight', 'left', 'right')][string]$JunctionTurn,
    [ValidateRange(0.65, 0.949)][double]$RedStopTriggerBottomFraction = 0.65,
    [switch]$InspectRedLine,
    [switch]$Go
)
$ErrorActionPreference = 'Stop'
$bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$pythonArgs = @()
if (Test-Path -LiteralPath $bundledPython) {
    $pythonExe = $bundledPython
} elseif (Get-Command py.exe -ErrorAction SilentlyContinue) {
    $pythonExe = (Get-Command py.exe).Source
    $pythonArgs = @('-3')
} else {
    throw 'Windows Python is unavailable. Install Python 3 or restore the Codex bundled runtime. Do not retry through WSL.'
}
$scriptPath = Join-Path $PSScriptRoot 'run_duck2_ground_test.py'
$durationWasSpecified = $PSBoundParameters.ContainsKey('Duration')
if (($GroundRightPivot -or $GroundRollingRight -or $GroundStrongRollingRight -or $GroundEqualWheels) -and -not $durationWasSpecified) {
    $Duration = 2.0
}
if ($InspectRedLine -and -not $durationWasSpecified) { $Duration = 2.0 }
$runArgs = @($scriptPath, '--label', $Label, '--duration', $Duration.ToString([Globalization.CultureInfo]::InvariantCulture))
if ($GroundRightPivot) { $runArgs += '--ground-right-pivot' }
if ($GroundRollingRight) { $runArgs += '--ground-rolling-right' }
if ($GroundStrongRollingRight) { $runArgs += '--ground-strong-rolling-right' }
if ($GroundEqualWheels) { $runArgs += '--ground-equal-wheels' }
if ($JunctionTurn) {
    $runArgs += @(
        '--junction-turn', $JunctionTurn,
        '--red-stop-trigger-bottom-fraction',
        $RedStopTriggerBottomFraction.ToString([Globalization.CultureInfo]::InvariantCulture)
    )
}
if ($InspectRedLine) { $runArgs += '--inspect-red-line' }
if ($Go) {
    $runArgs += '--confirm-go'
} elseif (-not $InspectRedLine) {
    $runArgs += '--check-only'
}
& $pythonExe @pythonArgs @runArgs
exit $LASTEXITCODE
