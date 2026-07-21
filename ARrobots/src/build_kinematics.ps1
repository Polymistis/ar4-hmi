[CmdletBinding()]
param(
    [string]$Python = "python",
    [string]$BuildDirectory,
    [switch]$Install
)

$ErrorActionPreference = "Stop"

$sourceDirectory = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($sourceDirectory)) {
    $sourceDirectory = Split-Path -Parent $MyInvocation.MyCommand.Path
}
if (
    [string]::IsNullOrWhiteSpace($sourceDirectory) -or
    -not (Test-Path -LiteralPath $sourceDirectory -PathType Container)
) {
    throw "Native kinematics source directory could not be resolved"
}
if ([string]::IsNullOrWhiteSpace($BuildDirectory)) {
    $BuildDirectory = Join-Path $sourceDirectory "build-windows-x64"
}

# MSBuild worker processes reject inherited Windows environment keys that differ
# only by case. Normalize the process-local path key before launching build tools.
$processPath = [System.Environment]::GetEnvironmentVariable(
    "Path",
    [System.EnvironmentVariableTarget]::Process
)
if ($null -ne $processPath) {
    [System.Environment]::SetEnvironmentVariable(
        "PATH",
        $null,
        [System.EnvironmentVariableTarget]::Process
    )
    [System.Environment]::SetEnvironmentVariable(
        "Path",
        $processPath,
        [System.EnvironmentVariableTarget]::Process
    )
}

$pythonCommand = Get-Command $Python -ErrorAction Stop
$pythonPath = $pythonCommand.Source
if (-not $pythonPath -or -not (Test-Path -LiteralPath $pythonPath -PathType Leaf)) {
    throw "Python must resolve to an executable file"
}

$pybind11Directory = & $pythonPath -m pybind11 --cmakedir
if ($LASTEXITCODE -ne 0) {
    throw "Unable to resolve the pybind11 CMake package"
}
$pybind11Directory = "$pybind11Directory".Trim()
if (-not (Test-Path -LiteralPath $pybind11Directory -PathType Container)) {
    throw "pybind11 returned an invalid CMake package directory"
}

$cmakeCommand = Get-Command cmake.exe -ErrorAction SilentlyContinue
if ($null -eq $cmakeCommand) {
    $vswherePath = Join-Path ${env:ProgramFiles(x86)} `
        "Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path -LiteralPath $vswherePath -PathType Leaf)) {
        throw "CMake was not found on PATH and Visual Studio discovery is unavailable"
    }
    $visualStudioPath = & $vswherePath `
        -latest `
        -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $visualStudioPath) {
        throw "A Visual Studio C++ build installation was not found"
    }
    $bundledCmake = Join-Path "$visualStudioPath" `
        "Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
    if (-not (Test-Path -LiteralPath $bundledCmake -PathType Leaf)) {
        throw "The Visual Studio CMake executable was not found"
    }
    $cmakePath = $bundledCmake
}
else {
    $cmakePath = $cmakeCommand.Source
}

$configureArguments = @(
    "-S", $sourceDirectory,
    "-B", $BuildDirectory,
    "-A", "x64",
    "-DPYBIND11_FINDPYTHON=ON",
    "-DPython_EXECUTABLE=$pythonPath",
    "-Dpybind11_DIR=$pybind11Directory"
)
& $cmakePath @configureArguments
if ($LASTEXITCODE -ne 0) {
    throw "Native kinematics configuration failed"
}

& $cmakePath --build $BuildDirectory --config Release --parallel 1
if ($LASTEXITCODE -ne 0) {
    throw "Native kinematics compilation failed"
}

$releaseDirectory = Join-Path $BuildDirectory "Release"
$artifacts = @(
    Get-ChildItem -LiteralPath $releaseDirectory -File `
        -Filter "robot_kinematics*.pyd"
)
if ($artifacts.Count -ne 1) {
    throw "Expected one native kinematics artifact; found $($artifacts.Count)"
}

if ($Install) {
    $destination = Join-Path $sourceDirectory ("..\" + $artifacts[0].Name)
    Copy-Item -LiteralPath $artifacts[0].FullName -Destination $destination -Force
    Write-Output "Installed $destination"
}
else {
    Write-Output $artifacts[0].FullName
}
