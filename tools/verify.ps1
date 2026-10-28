param(
    [string]$BuildDirectory = 'build/windows',
    [string]$OutputDirectory = 'results/acceptance',
    [switch]$SkipDesktop
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
function Local-Path([string]$relative) {
    $full = [IO.Path]::GetFullPath((Join-Path $projectRoot $relative))
    if (-not $full.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Verification paths must be inside this project.'
    }
    return $full
}
$buildRoot = Local-Path $BuildDirectory
$out = Local-Path $OutputDirectory
if (Test-Path -LiteralPath $out) { throw "Evidence directory already exists: $out" }
$cache = Get-Content -LiteralPath (Join-Path $buildRoot 'CMakeCache.txt')
$configuredRoot = ($cache | Where-Object { $_ -like 'CMAKE_HOME_DIRECTORY:INTERNAL=*' }) -replace '^CMAKE_HOME_DIRECTORY:INTERNAL=', ''
if ([IO.Path]::GetFullPath($configuredRoot) -ne $projectRoot) { throw 'Build belongs to another source directory.' }
$cmake = ($cache | Where-Object { $_ -like 'CMAKE_COMMAND:INTERNAL=*' }) -replace '^CMAKE_COMMAND:INTERNAL=', ''
$ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
$cli = Join-Path $buildRoot 'Release/passagelab_cli.exe'
$desktop = Join-Path $buildRoot 'Release/passagelab_desktop.exe'
New-Item -ItemType Directory -Path $out | Out-Null
function Run-Check([string]$name, [string]$exe, [string[]]$arguments, [int]$expected = 0) {
    & $exe @arguments 2>&1 | Tee-Object -FilePath (Join-Path $out "$name.log")
    if ($LASTEXITCODE -ne $expected) { throw "$name expected exit $expected, got $LASTEXITCODE" }
}
$oldTemp = $env:TEMP
$oldTmp = $env:TMP
$env:TEMP = Join-Path $out 'tmp'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Path $env:TEMP | Out-Null
Push-Location $projectRoot
try {
    Run-Check 'ctest' $ctest @('--test-dir', $buildRoot, '-C', 'Release', '--output-on-failure', '--output-junit', (Join-Path $out 'tests.xml'))
    Run-Check 'references' $cli @('generate-scenes', '--out', (Join-Path $out 'scenes'))
    foreach ($demo in 0,1,2) {
        $run = Join-Path $out "demo-$demo"
        Run-Check "plan-$demo" $cli @('plan', '--demo', "$demo", '--seed', '42', '--seconds', '0', '--out', $run)
        foreach ($path in 'path', 'optimized') {
            Run-Check "fine-$demo-$path" $cli @('validate-path', '--scene', "$run/scene.json", '--path', "$run/$path.json", '--step', '0.02', '--out', "$run/$path-fine.json")
        }
        if (-not $SkipDesktop) {
            Run-Check "screenshot-$demo" $desktop @('--demo', "$demo", '--screenshot', (Join-Path $out "scene-$demo.ppm"))
            $ui = Join-Path $out "interaction-$demo"
            New-Item -ItemType Directory -Path $ui | Out-Null
            Push-Location $ui
            try {
                Run-Check "interaction-$demo" $desktop @('--demo', "$demo", '--interaction-test', "$ui/interaction.json")
            } finally { Pop-Location }
        }
    }
    Run-Check 'budget-exhausted' $cli @('plan', '--demo', '2', '--iterations', '0', '--out', (Join-Path $out 'budget-exhausted')) 2
    Run-Check 'query-benchmark' $cli @('query-benchmark', '--out', (Join-Path $out 'query-benchmark.json'))
    if (-not $SkipDesktop) {
        Run-Check 'desktop-smoke' $desktop @('--smoke-test')
        Run-Check 'desktop-self-test' $desktop @('--ui-self-test', '--self-test-report', "$out/desktop-self-test.json")
        $performanceProcess = New-Object System.Diagnostics.Process
        $performanceProcess.StartInfo.FileName = $desktop
        $performanceProcess.StartInfo.Arguments = '--demo 2 --frames 180 --performance-report "' + "$out/desktop-performance.json" + '"'
        $performanceProcess.StartInfo.UseShellExecute = $false
        $performanceProcess.StartInfo.CreateNoWindow = $true
        $performanceProcess.StartInfo.RedirectStandardOutput = $true
        $performanceProcess.StartInfo.RedirectStandardError = $true
        $null = $performanceProcess.Start()
        $peakBytes = [long]0
        while (-not $performanceProcess.HasExited) {
            $performanceProcess.Refresh()
            $peakBytes = [Math]::Max($peakBytes, $performanceProcess.PeakWorkingSet64)
            Start-Sleep -Milliseconds 20
        }
        $performanceProcess.WaitForExit()
        $performanceProcess.StandardOutput.ReadToEnd() | Set-Content -LiteralPath "$out/desktop-performance.log"
        $performanceProcess.StandardError.ReadToEnd() | Set-Content -LiteralPath "$out/desktop-performance-error.log"
        if ($performanceProcess.ExitCode -ne 0) { throw 'Desktop performance process failed' }
        [ordered]@{ observed_peak_working_set_bytes = $peakBytes; target_bytes = 536870912; passed = ($peakBytes -gt 0 -and $peakBytes -le 536870912); method = 'Process PeakWorkingSet64 sampled every 20ms during scene C playback' } |
            ConvertTo-Json | Set-Content -LiteralPath "$out/desktop-memory.json" -Encoding UTF8
        if ($peakBytes -le 0 -or $peakBytes -gt 536870912) { throw 'Reference-machine memory target missed' }
        $performance = Get-Content -LiteralPath "$out/desktop-performance.json" -Raw | ConvertFrom-Json
        if (-not $performance.target_passed) { throw 'Reference-machine rendering target missed' }
    }
    $report = Get-Content -LiteralPath (Join-Path $out 'demo-0/report.json') -Raw | ConvertFrom-Json
    [ordered]@{
        verified_utc = [DateTime]::UtcNow.ToString('o')
        source_sha256 = $report.source_sha256
        software_version = $report.software_version
        git_commit = $report.git_commit
        git_tracked_dirty = $report.git_tracked_dirty
        desktop_executed = -not $SkipDesktop
        cli_sha256 = (Get-FileHash -LiteralPath $cli -Algorithm SHA256).Hash
        desktop_sha256 = if (-not $SkipDesktop) { (Get-FileHash -LiteralPath $desktop -Algorithm SHA256).Hash } else { $null }
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $out 'verification.json') -Encoding UTF8
} finally {
    Pop-Location
    $env:TEMP = $oldTemp
    $env:TMP = $oldTmp
}
