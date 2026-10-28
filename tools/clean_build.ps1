param([string]$Destination = 'results/clean-source-0.2.0')
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$stage = [IO.Path]::GetFullPath((Join-Path $root $Destination))
if (-not $stage.StartsWith($root + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Clean source destination must remain within this worktree.'
}
if (Test-Path -LiteralPath $stage) { throw "Destination already exists: $stage" }
New-Item -ItemType Directory -Path $stage | Out-Null
foreach ($name in @('CMakeLists.txt','CMakePresets.json','vcpkg.json','cmake','include','src','apps','tests','assets','README.md','LICENSE','THIRD_PARTY_NOTICES.md')) {
    Copy-Item -LiteralPath (Join-Path $root $name) -Destination $stage -Recurse
}
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$vsRoot = & $vswhere -latest -products '*' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
$cmake = Join-Path $vsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$ctest = Join-Path (Split-Path $cmake) 'ctest.exe'
function Check([string]$exe, [string[]]$arguments) {
    & $exe @arguments 2>&1 | Tee-Object -FilePath (Join-Path $stage 'clean-build.log') -Append
    if ($LASTEXITCODE) { throw "$exe failed: $LASTEXITCODE" }
}
$oldTemp = $env:TEMP
$oldTmp = $env:TMP
$env:TEMP = Join-Path $stage 'tmp'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Path $env:TEMP | Out-Null
try {
    Check $cmake @('-S',$stage,'-B',"$stage/build",'-G','Visual Studio 17 2022','-A','x64',
        "-DCMAKE_TOOLCHAIN_FILE=$root/.tools/vcpkg/scripts/buildsystems/vcpkg.cmake",
        '-DVCPKG_MANIFEST_INSTALL=OFF',"-DVCPKG_INSTALLED_DIR=$root/vcpkg_installed")
    Check $cmake @('--build',"$stage/build",'--config','Release','--parallel','6')
    Check $ctest @('--test-dir',"$stage/build",'-C','Release','--output-on-failure','--output-junit',"$stage/tests.xml")
    Check "$stage/build/Release/passagelab_cli.exe" @('plan','--demo','2','--seconds','0','--out',"$stage/demo-c")
    Check "$stage/build/Release/passagelab_desktop.exe" @('--ui-self-test','--self-test-report',"$stage/desktop-self-test.json")
    $run = Get-Content -LiteralPath "$stage/demo-c/report.json" -Raw | ConvertFrom-Json
    [ordered]@{
        passed = $true
        verified_utc = [DateTime]::UtcNow.ToString('o')
        source_sha256 = $run.source_sha256
        source_only_copy = $true
        preexisting_build_artifacts = $false
        reused_pinned_dependency_installation = $true
        host = 'Same Windows host; not a new physical machine or VM'
    } | ConvertTo-Json | Set-Content -LiteralPath "$stage/verification.json" -Encoding UTF8
} finally {
    $env:TEMP = $oldTemp
    $env:TMP = $oldTmp
}
