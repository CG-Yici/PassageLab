param(
    [switch]$SkipTests,
    [switch]$CleanConfigure,
    [switch]$UseInstalledDependencies,
    [string]$BuildDirectory = 'build/windows'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$manifest = Get-Content -LiteralPath (Join-Path $projectRoot 'vcpkg.json') -Raw | ConvertFrom-Json
$vcpkgRoot = Join-Path $projectRoot '.tools\vcpkg'
$buildRoot = [IO.Path]::GetFullPath((Join-Path $projectRoot $BuildDirectory))
if (-not $buildRoot.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'BuildDirectory must be inside this project.'
}
function Invoke-Checked { param([string]$Exe, [string[]]$Arguments) & $Exe @Arguments; if ($LASTEXITCODE -ne 0) { throw "$Exe failed with exit code $LASTEXITCODE" } }
if (-not $UseInstalledDependencies -and -not (Test-Path -LiteralPath (Join-Path $vcpkgRoot '.git'))) {
    Invoke-Checked git @('clone','https://github.com/microsoft/vcpkg.git',$vcpkgRoot)
}
if ($UseInstalledDependencies) {
    if (-not (Test-Path -LiteralPath (Join-Path $projectRoot 'vcpkg_installed\vcpkg\status')) -or
        -not (Test-Path -LiteralPath (Join-Path $vcpkgRoot 'scripts\buildsystems\vcpkg.cmake'))) {
        throw 'UseInstalledDependencies requires the local vcpkg toolchain and installed packages.'
    }
} else {
    Invoke-Checked git @('-C',$vcpkgRoot,'checkout',$manifest.'builtin-baseline')
    if (-not (Test-Path -LiteralPath (Join-Path $vcpkgRoot 'vcpkg.exe'))) { Invoke-Checked (Join-Path $vcpkgRoot 'bootstrap-vcpkg.bat') @('-disableMetrics') }
}
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere)) { throw 'Install Visual Studio 2022 Desktop development with C++ (including CMake tools and Windows SDK).' }
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$cmake = Join-Path $vsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
if (-not (Test-Path -LiteralPath $cmake)) { throw 'Visual Studio CMake tools are missing.' }
# A temporary drive avoids RC/Ninja failures in Unicode or space-containing dependency paths.
$drive = $null
if (-not $UseInstalledDependencies) {
foreach ($letter in @('P','Q','R','S','T','U','V','W','X','Y','Z')) {
    if (-not (Test-Path "${letter}:\")) { $drive = $letter; break }
}
if (-not $drive) { throw 'No free temporary drive letter for dependency build.' }
Invoke-Checked subst @("${drive}:",$projectRoot)
}
$previousConcurrency = $env:VCPKG_MAX_CONCURRENCY
$previousTemp = $env:TEMP
$previousTmp = $env:TMP
$env:VCPKG_MAX_CONCURRENCY = '8'
$env:TEMP = Join-Path $projectRoot 'build\tmp'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Force -Path $env:TEMP | Out-Null
Push-Location $projectRoot
try {
    if (-not $UseInstalledDependencies) {
    $mapped = "${drive}:"
    Invoke-Checked "$mapped\.tools\vcpkg\vcpkg.exe" @('install','--triplet','x64-windows','--host-triplet','x64-windows',"--x-buildtrees-root=$mapped\.tools\dependency-buildtrees","--x-packages-root=$mapped\.tools\dependency-packages")
    }
    $configure = @('--preset','windows-release','-B',$buildRoot,'-DVCPKG_MANIFEST_INSTALL=OFF',"-DVCPKG_INSTALLED_DIR=$projectRoot/vcpkg_installed")
    if ($CleanConfigure) { $configure += '--fresh' }
    Invoke-Checked $cmake $configure
    Invoke-Checked $cmake @('--build',$buildRoot,'--config','Release','--parallel','6')
    if (-not $SkipTests) { Invoke-Checked $ctest @('--test-dir',$buildRoot,'-C','Release','--output-on-failure') }
} finally {
    Pop-Location
    $env:VCPKG_MAX_CONCURRENCY = $previousConcurrency
    $env:TEMP = $previousTemp
    $env:TMP = $previousTmp
    if ($drive) { & subst "${drive}:" /D }
}
