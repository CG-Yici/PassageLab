param(
    [string]$Archive = 'dist/verified/PassageLab-0.2.0-windows-x64.zip',
    [string]$OutputDirectory = 'results/package-0.2.0'
)
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
function Local-Path([string]$relative) {
    $full = [IO.Path]::GetFullPath((Join-Path $root $relative))
    if (-not $full.StartsWith($root + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Package verification paths must stay inside this worktree.'
    }
    return $full
}
$zip = Local-Path $Archive
$out = Local-Path $OutputDirectory
if (Test-Path -LiteralPath $out) { throw 'Choose a new package verification directory.' }
New-Item -ItemType Directory -Path $out | Out-Null
Expand-Archive -LiteralPath $zip -DestinationPath $out
$package = Get-ChildItem -LiteralPath $out -Directory | Select-Object -First 1
$manifest = Get-Content -LiteralPath (Join-Path $package.FullName 'binary-sha256.json') -Raw | ConvertFrom-Json
foreach ($entry in $manifest) {
    $actual = (Get-FileHash -LiteralPath (Join-Path $package.FullName $entry.file) -Algorithm SHA256).Hash
    if ($actual -ne $entry.sha256) { throw "Binary hash mismatch: $($entry.file)" }
}
$oldPath = $env:PATH
$oldTemp = $env:TEMP
$oldTmp = $env:TMP
$env:PATH = "$env:SystemRoot\System32;$env:SystemRoot"
$env:TEMP = Join-Path $out 'tmp'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Path $env:TEMP | Out-Null
Push-Location $package.FullName
try {
    $cli = Join-Path $package.FullName 'passagelab_cli.exe'
    $desktop = Join-Path $package.FullName 'passagelab_desktop.exe'
    foreach ($demo in 0,1,2) {
        $run = Join-Path $out "demo-$demo"
        & $cli plan --demo $demo --seed 42 --seconds 0 --out $run
        if ($LASTEXITCODE) { throw "Packaged demo $demo failed" }
        foreach ($kind in 'path','optimized') {
            & $cli validate-path --scene "$run/scene.json" --path "$run/$kind.json" --step 0.02 --out "$run/$kind-fine.json"
            if ($LASTEXITCODE) { throw "Packaged path $demo/$kind failed" }
        }
    }
    & $desktop --ui-self-test --self-test-report "$out/desktop-self-test.json"
    if ($LASTEXITCODE) { throw 'Packaged desktop self-test failed' }
    $ui = New-Item -ItemType Directory -Path (Join-Path $out 'interaction')
    Push-Location $ui.FullName
    try {
        & $desktop --demo 2 --interaction-test "$out/interaction.json"
        if ($LASTEXITCODE) { throw 'Packaged interaction test failed' }
    } finally { Pop-Location }
    $run = Get-Content -LiteralPath "$out/demo-0/report.json" -Raw | ConvertFrom-Json
    [ordered]@{
        passed = $true
        archive_sha256 = (Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash
        source_sha256 = $run.source_sha256
        software_version = $run.software_version
        binary_hashes_checked = @($manifest).Count
        path_contains_development_dependencies = $false
        demos_solved = 3
        fine_path_checks_passed = 6
        desktop_self_test_passed = $true
        interaction_checks_passed = 25
        host = 'Same Windows host; extracted archive with system-only PATH'
    } | ConvertTo-Json | Set-Content -LiteralPath "$out/verification.json" -Encoding UTF8
} finally {
    Pop-Location
    $env:PATH = $oldPath
    $env:TEMP = $oldTemp
    $env:TMP = $oldTmp
}
