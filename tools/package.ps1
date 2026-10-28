param([string]$Version = '0.1.0')
$ErrorActionPreference='Stop'
$projectRoot=(Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$release=Join-Path $projectRoot 'build\windows\Release'
$destination=Join-Path $projectRoot "dist\PassageLab-$Version-windows-x64"
if(Test-Path -LiteralPath $destination){throw "Package directory already exists: $destination. Choose a new version or inspect it before replacing."}
New-Item -ItemType Directory -Path $destination -Force | Out-Null
foreach($name in @('passagelab_cli.exe','passagelab_desktop.exe')){Copy-Item -LiteralPath (Join-Path $release $name) -Destination $destination}
Get-ChildItem -LiteralPath $release -Filter '*.dll' | Copy-Item -Destination $destination
Copy-Item -LiteralPath (Join-Path $projectRoot 'assets') -Destination $destination -Recurse
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
Copy-Item -LiteralPath (Join-Path $projectRoot 'vcpkg_installed\vcpkg\status') -Destination (Join-Path $destination 'dependency-versions.txt')
$archive="$destination.zip"
Compress-Archive -LiteralPath $destination -DestinationPath $archive
Get-FileHash -LiteralPath $archive -Algorithm SHA256 | Format-List
Write-Output $archive
