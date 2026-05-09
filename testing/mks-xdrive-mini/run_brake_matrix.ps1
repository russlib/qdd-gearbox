# Brake disable test matrix runner.
#
# Walks through tests T0..T8 in the recommended order, prompting the user
# to power-cycle the H2 between each so latched brake state is cleared.
# After each test, prints the verdict and asks whether to continue.
#
# Usage:
#   .\run_brake_matrix.ps1
#   .\run_brake_matrix.ps1 -StartFrom T2     # skip ahead
#   .\run_brake_matrix.ps1 -OnlyTests T1,T2  # subset

param(
    [string]$StartFrom = "T0",
    [string[]]$OnlyTests = @()
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "mks-python-env.ps1")

$tests = @(
    @{Id="T0"; Desc="Baseline: fresh power-cycle, no BLE";
      Cmd="brake_disable_probe.py --mode none"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T7"; Desc="Heartbeat decoder (passive subscribe; engages brake)";
      Cmd="heartbeat_decoder.py --duration 30"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T1"; Desc="WARM_UP (0x04)";
      Cmd="brake_disable_probe.py --mode warmup"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T2"; Desc="ROLL_DOWN (0x05) with motor RPM ramp cycle";
      Cmd="brake_disable_probe.py --mode rolldown --multi-ramp"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T3"; Desc="HEADLESS (0x00) on fresh boot";
      Cmd="brake_disable_probe.py --mode headless"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T5"; Desc="WARM_UP + GATT disconnect";
      Cmd="brake_disable_probe.py --mode warmup --disconnect-before-spin"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T6"; Desc="HEADLESS + GATT disconnect";
      Cmd="brake_disable_probe.py --mode headless --disconnect-before-spin"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T4"; Desc="POWER_RANGE (0x03) min=0 max=0";
      Cmd="brake_disable_probe.py --mode power-range-0"; PowerCycle=$true; NeedsAC=$true},
    @{Id="T8"; Desc="AC unplugged (no BLE possible; no telemetry; expected RELIABLE PASS)";
      Cmd="brake_disable_probe.py --mode none"; PowerCycle=$false; NeedsAC=$false}
)

if ($StartFrom -ne "T0") {
    $idx = ($tests | ForEach-Object { $_.Id }).IndexOf($StartFrom)
    if ($idx -lt 0) { throw "unknown StartFrom: $StartFrom" }
    $tests = $tests[$idx..($tests.Count-1)]
}
if ($OnlyTests.Count -gt 0) {
    $tests = $tests | Where-Object { $_.Id -in $OnlyTests }
}

Write-Host ""
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "  Saris H2 Brake Disable Test Matrix"               -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Tests in this run:" -ForegroundColor Yellow
foreach ($t in $tests) { Write-Host ("  {0}  {1}" -f $t.Id, $t.Desc) }
Write-Host ""

foreach ($t in $tests) {
    Write-Host ""
    Write-Host "==================================================" -ForegroundColor Cyan
    Write-Host ("  {0}  {1}" -f $t.Id, $t.Desc) -ForegroundColor Cyan
    Write-Host "==================================================" -ForegroundColor Cyan

    if ($t.PowerCycle) {
        if ($t.NeedsAC) {
            Write-Host "  ACTION: Unplug H2 AC, wait 15 s, replug. Spin flywheel by hand to wake." -ForegroundColor Yellow
        }
        Read-Host "  Press ENTER when trainer is ready"
    } else {
        Write-Host "  ACTION: Unplug H2 AC entirely (this test runs without trainer power)." -ForegroundColor Yellow
        Read-Host "  Press ENTER when AC is unplugged"
    }

    Write-Host ""
    Write-Host "  Running: $($t.Cmd)" -ForegroundColor Gray
    $argList = $t.Cmd -split " "
    $script = Join-Path $PSScriptRoot $argList[0]
    $rest = $argList[1..($argList.Count-1)]
    & $script:MksPython $script @rest
    $exit = $LASTEXITCODE

    Write-Host ""
    if ($exit -ne 0) {
        Write-Host "  Test exited with code $exit" -ForegroundColor Red
    }
    $resp = Read-Host "  Continue to next test? [y/N/q]"
    if ($resp -eq "q" -or $resp -eq "n" -or $resp -eq "") { break }
}

Write-Host ""
Write-Host "Matrix run complete. Compare CSV/PNG files in thermal_data\." -ForegroundColor Cyan
