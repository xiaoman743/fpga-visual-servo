# run_sweeps.ps1 -- build + run the P5 delay/stability sweeps with resume.
#
#   .\run_sweeps.ps1                 # full sweep, resume from aux.csv
#   .\run_sweeps.ps1 -Groups a,b     # subset
#   .\run_sweeps.ps1 -Force          # ignore previous results
#   .\run_sweeps.ps1 -Trace          # only the waveform traces
#
# Produces:  results.csv (CSV,... lines), aux.csv (AUX,... lines), run.log
# Everything lives in this ASCII directory; the archived RTL is copied in
# read-only fashion and never modified.

param(
    [string]$Groups = 'a,b,k,m,bnd',
    [switch]$Force,
    [int]$Nf = 400,
    [int]$Reps = 10,
    [int]$KReps = 3,
    [double]$Gain = 0.8,
    [int]$Ox = 750,
    [int]$Oy = 240,
    [double]$R0 = 160
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root
$vbin = 'D:\vavido\Vivado\2020.2\bin'
if (Test-Path $vbin) { $env:PATH = "$vbin;" + $env:PATH }
$xvlog = Join-Path $vbin 'xvlog.bat'
$xelab = Join-Path $vbin 'xelab.bat'
$xsim  = Join-Path $vbin 'xsim.bat'

$results = Join-Path $root 'results.csv'
$aux     = Join-Path $root 'aux.csv'
$log     = Join-Path $root 'run.log'

if ($Force) {
    Remove-Item $results,$aux,cases.txt -ErrorAction SilentlyContinue
    Write-Host 'Force: cleared previous results'
}
if (-not (Test-Path $results)) {
    [IO.File]::WriteAllText($results,
        "kind, mode, depth, plant_gain, seed, converged, ts_frames, e_inf, mp, pp, period_frames`r`n")
}
# NB: PowerShell's Test-Path/Get-Content can be blind to files that the sandbox
# ACL only lets python/Add-Content touch, so header creation goes through python.
& python -c "import os,sys;p=sys.argv[1];h=sys.argv[2];
os.path.exists(p) or open(p,'w').write(h+chr(10))" 'aux.csv' 'kind, mode, depth, plant_gain, seed, mean_delay, max_delay, sat_frac, e_start, e_max, n_used, diverged, e_final, tag' | Out-Null
& python -c "import os,sys;p=sys.argv[1];h=sys.argv[2];
os.path.exists(p) or open(p,'w').write(h+chr(10))" 'sigma_sweep.csv' 'CSV,mean,A,seed,converged,ts_frames,mp,pp,period_frames' | Out-Null
$sigma = Join-Path $root 'sigma_sweep.csv'

# ---------------------------------------------------------------- plan
$planArgs = @('plan.py', '--groups', $Groups, '--out', 'cases.txt', '--skip', 'aux.csv',
              '--nf', $Nf, '--reps', $Reps, '--kreps', $KReps, '--gain', $Gain,
              '--ox', $Ox, '--oy', $Oy, '--r0', $R0)
$planOut = & python @planArgs 2>&1
$planOut | ForEach-Object { Write-Host $_ }
$queued = [int]([regex]::Match(($planOut -join ' '), '(\d+) queued').Groups[1].Value)
if ($queued -eq 0) { Write-Host 'All cases already done - nothing to run.'; exit 0 }

# ---------------------------------------------------------------- build
Write-Host '== xvlog =='
& $xvlog --nolog 'rtl\servo_polar.v' 'rtl\delay_inject.v' 'tb_p5_sweep.v' | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'xvlog failed' }
Write-Host '== xelab =='
& $xelab --nolog 'work.tb_p5_sweep' -s sweep --debug typical | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'xelab failed' }

# ---------------------------------------------------------------- simulate
Write-Host "== xsim ($queued queued cases) =="
$sw = [Diagnostics.Stopwatch]::StartNew()
$out = & $xsim sweep --R 2>&1
$sw.Stop()
$out | Set-Content $log -Encoding UTF8

$csv = $out | Where-Object { $_ -like 'CSV,*' }
$auxl = $out | Where-Object { $_ -like 'AUX,*' }
$sig = $out | Where-Object { $_ -like 'SCSV,*' } | ForEach-Object { $_ -replace '^SCSV,', 'CSV,' }
if ($csv)  { $csv  | Add-Content $results -Encoding ASCII }
if ($auxl) { $auxl | Add-Content $aux   -Encoding ASCII }
if ($sig)  { $sig  | Add-Content $sigma -Encoding ASCII }

$done = ($out | Where-Object { $_ -like '*tb_p5_sweep done*' })
Write-Host ("ran {0} cases in {1:N1} s  (csV rows: {2})" -f $queued, $sw.Elapsed.TotalSeconds, @($csv).Count)
if (-not $done) {
    Write-Warning 'simulation did not reach "done" - inspect run.log'
    $out | Select-Object -Last 12 | ForEach-Object { Write-Host $_ }
    exit 1
}
Write-Host ('results -> ' + $results)
Write-Host ('aux     -> ' + $aux)
