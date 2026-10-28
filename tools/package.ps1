param([string]$BuildDirectory = 'build/windows', [string]$OutputDirectory = 'dist')
$ErrorActionPreference='Stop'
$projectRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$Version = (Get-Content -LiteralPath (Join-Path $projectRoot 'vcpkg.json') -Raw | ConvertFrom-Json).'version-semver'
$buildRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
if (-not $buildRoot.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'BuildDirectory must be inside this project.'
}
$cache = Get-Content -LiteralPath (Join-Path $buildRoot 'CMakeCache.txt')
$configuredRoot = ($cache | Where-Object { $_ -like 'CMAKE_HOME_DIRECTORY:INTERNAL=*' }) -replace '^CMAKE_HOME_DIRECTORY:INTERNAL=', ''
if ([IO.Path]::GetFullPath($configuredRoot) -ne $projectRoot) { throw 'Build cache belongs to a different source directory; rebuild locally first.' }
$release=Join-Path $buildRoot 'Release'
$outputRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot $OutputDirectory))
if (-not $outputRoot.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'OutputDirectory must be inside this project.'
}
$destination=Join-Path $outputRoot "PassageLab-$Version-windows-x64"
if(Test-Path -LiteralPath $destination){throw "Package directory already exists: $destination. Choose a new version or inspect it before replacing."}
New-Item -ItemType Directory -Path $destination -Force | Out-Null
foreach($name in @('passagelab_cli.exe','passagelab_desktop.exe')){Copy-Item -LiteralPath (Join-Path $release $name) -Destination $destination}
Get-ChildItem -LiteralPath $release -Filter '*.dll' | Copy-Item -Destination $destination
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$redist = Get-ChildItem -LiteralPath (Join-Path $vsRoot 'VC\Redist\MSVC') -Directory |
    Where-Object { $_.Name -match '^\d' } | Sort-Object Name -Descending | Select-Object -First 1
if (-not $redist) { throw 'Visual C++ redistributable files are required for packaging.' }
$crt = Join-Path $redist.FullName 'x64\Microsoft.VC143.CRT'
Get-ChildItem -LiteralPath $crt -Filter '*.dll' | Copy-Item -Destination $destination
Copy-Item -LiteralPath (Join-Path $projectRoot 'assets') -Destination $destination -Recurse
Copy-Item -LiteralPath (Join-Path $projectRoot 'docs') -Destination $destination -Recurse
Copy-Item -LiteralPath (Join-Path $projectRoot 'benchmarks') -Destination $destination -Recurse
foreach($name in @('README.md','LICENSE','THIRD_PARTY_NOTICES.md')){Copy-Item -LiteralPath (Join-Path $projectRoot $name) -Destination $destination}
$licenses=Join-Path $destination 'licenses';New-Item -ItemType Directory -Path $licenses | Out-Null
Get-ChildItem -LiteralPath (Join-Path $projectRoot 'vcpkg_installed\x64-windows\share') -Directory | ForEach-Object {
    $notice=Join-Path $_.FullName 'copyright';if(Test-Path -LiteralPath $notice){Copy-Item -LiteralPath $notice -Destination (Join-Path $licenses ($_.Name+'.txt'))}
}
@'
@echo off
cd /d "%~dp0"
start "" passagelab_desktop.exe
'@ | Set-Content -LiteralPath (Join-Path $destination 'Start PassageLab.cmd') -Encoding ASCII
Copy-Item -LiteralPath (Join-Path $projectRoot 'vcpkg.json') -Destination $destination
@'
PassageLab Windows x64 portable package

Run "Start PassageLab.cmd" to open the viewer. OpenGL 3.3 is required.
CLI example (from this directory):
  passagelab_cli.exe plan --demo 2 --seconds 0 --out my-first-run
Use a new output directory for each run. See docs/viewer.md for controls.
Visual C++ runtime DLLs are included beside the executable.
Source build instructions in README.md apply to the source repository.
'@ | Set-Content -LiteralPath (Join-Path $destination 'START-HERE.txt') -Encoding UTF8
Copy-Item -LiteralPath (Join-Path $projectRoot 'vcpkg_installed\vcpkg\status') -Destination (Join-Path $destination 'dependency-versions.txt')
$hashes = Get-ChildItem -LiteralPath $destination -File | Where-Object { $_.Extension -in '.exe', '.dll' } | ForEach-Object {
    [ordered]@{ file = $_.Name; sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
}
$hashes | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $destination 'binary-sha256.json') -Encoding UTF8
$archive="$destination.zip"
Compress-Archive -LiteralPath $destination -DestinationPath $archive
Get-FileHash -LiteralPath $archive -Algorithm SHA256 | Format-List
Write-Output $archive
