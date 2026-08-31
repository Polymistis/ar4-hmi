#Requires -Version 5.1
[CmdletBinding()]
param([Parameter(Mandatory)][string]$PythonPath, [Parameter(Mandatory)][string]$PythonInstallerPath,
  [Parameter(Mandatory)][string]$BuildRoot, [Parameter(Mandatory)][long]$SourceDateEpoch)
Set-StrictMode -Version Latest; $ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSEdition -ne 'Desktop' -or $PSVersionTable.PSVersion.ToString() -notlike '5.1.*') { throw 'Windows PowerShell 5.1 is required' }
$Utf8NoBom = [Text.UTF8Encoding]::new($false)
$NativeHash = '6bb0d8cfe8b43317077f942f0ec87f7afaf7424af456c8d230d70985cf81272f'
$InstallerHash = '9d9eb2709ef81bf5cd30db3c2096bdbc4ea10087c22e62f27d356b36f6ae9649'
$RuntimeHashes = @{ 'python.exe' = '4942b86a6597e5aee0128daa00050ed79bc21f6e709a78eb19cbfeb0c2f39ac9'; 'python314.dll' = '0f9857ffdfe010fe6b99328d58c2e3c7472ce75f336bf9c2ad9bd5bca3bce700' }
$RuntimeSigner = '847785B686B2D3879731FA9AA3F1F5D48E85D99E'
$Expected = @{ base = @{ graph = '2be92961a76765f1e2b9970619e45fe062eea7501ea660e130ad3d6272482c2c'; notices = '91a6474646699a8881bcfe76102cddc20300178bb793c57a30e67ef124186670'; warnings = @{ 'WARNING: Ignoring /System/Library/Frameworks/CoreFoundation.framework/CoreFoundation imported from' = 'Pyserial macOS discovery is unreachable on Windows; output and graph checks reject accidental macOS collection.'; 'WARNING: Ignoring /System/Library/Frameworks/IOKit.framework/IOKit imported from' = 'Pyserial macOS discovery is unreachable on Windows; output and graph checks reject accidental macOS collection.' } };
  step = @{ graph = '8707db2b1a85388ed689dcc1ca5f1877d506ffb341ab75ea2e8fe4be4742c2be'; notices = '89d670ae972af975c837d69133e6dd05f07184ca30f8553e13dc2d1e9c1768bc'; warnings = @{ 'WARNING: Ignoring /System/Library/Frameworks/CoreFoundation.framework/CoreFoundation imported from' = 'Pyserial macOS discovery is unreachable on Windows; output and graph checks reject accidental macOS collection.'; 'WARNING: Ignoring /System/Library/Frameworks/IOKit.framework/IOKit imported from' = 'Pyserial macOS discovery is unreachable on Windows; output and graph checks reject accidental macOS collection.'; 'WARNING: Timed out while waiting for the child process to exit!' = 'The isolated OCP dependency scan hits the known child-exit timeout; exact wheel-native closure and packaged worker status checks pass.' } } }
$AppData = @('AR.png', 'defaults.json', 'information.txt', 'LICENSE.txt', 'VisBackdrop.png', 'xbox.png', 'play-icon.png', 'stop-icon.png', 'pp.gif', 'block.jpg', 'display setting.jpg', 'keystone jack.jpg', 'Link Base-1.STL', 'Link Base-2.STL', 'Link Base-3.STL', 'Link 1-1.STL', 'Link 1-2.STL', 'Link 2-1.STL', 'Link 2-2.STL', 'Link 2-3.STL', 'Link 3-1.STL', 'Link 3-2.STL', 'Link 4-1.STL', 'Link 4-2.STL', 'Link 4-3.STL', 'Link 5-1.STL', 'Link 5-2.STL', 'Link 6-1.STL', 'Link 6-2.STL', 'Servo Gripper.STL', 'Welding Torch.STL')
$TtkFixed = @('ttkbootstrap/assets/elements/manifest.json', 'ttkbootstrap/assets/icons/bootstrap.ttf', 'ttkbootstrap/assets/icons/glyphmap.json', 'ttkbootstrap/assets/icons/icon_metrics.json', 'ttkbootstrap/assets/icons/LICENSE')
$StepSolverClosure = @('casadi/_casadi.pyd', 'casadi/libcasadi.dll', 'casadi/libgcc_s_seh-1.dll', 'casadi/libstdc++-6.dll', 'casadi/libwinpthread-1.dll', 'nlopt/_nlopt.pyd')
$HmiExcludes = @('ARrobots.HMI.step_worker', 'OCP', 'adodbapi', 'aiohappyeyeballs', 'aiohttp', 'aiosignal', 'attr', 'attrs', 'cadquery', 'cadquery_ocp', 'cadquery_ocp_proxy', 'casadi', 'ezdxf', 'frozenlist', 'idna', 'llvmlite', 'more_itertools', 'msgpack', 'multidict', 'multimethod', 'nlopt', 'numba', 'propcache', 'runtype', 'scipy', 'trame', 'trame_client', 'trame_common', 'trame_components', 'trame_server', 'trame_vtk', 'trame_vuetify', 'typing_extensions', 'wslink', 'yaml', 'yarl') | Sort-Object
$WorkerExcludes = @('IPython', 'adodbapi', 'aiohappyeyeballs', 'aiohttp', 'aiosignal', 'attr', 'attrs', 'cadquery.cq_directive', 'cadquery.fig',
  'cadquery.occ_impl.jupyter_tools', 'cadquery.occ_impl.nurbs', 'cadquery.vis', 'cadquery_ocp_proxy', 'frozenlist', 'llvmlite', 'more_itertools',
  'msgpack', 'multidict', 'numba', 'propcache', 'scipy', 'trame', 'trame_client', 'trame_common', 'trame_components', 'trame_server', 'trame_vtk', 'trame_vuetify', 'wslink', 'yaml', 'yarl') | Sort-Object
function Assert-Condition([bool]$Condition, [string]$Message) { if (-not $Condition) { throw $Message } }
function Invoke-GitText([object[]]$Arguments) { $global:LASTEXITCODE = $null; $rows = @(& $GitPath @Arguments); Assert-Condition ($global:LASTEXITCODE -eq 0) 'Git source query failed'; return ($rows -join "`n") }
function Get-Hash([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function Get-TextHash([string]$Value) { ([BitConverter]::ToString([Security.Cryptography.SHA256]::Create().ComputeHash($Utf8NoBom.GetBytes($Value)))).Replace('-', '').ToLowerInvariant() }
function Write-Json([string]$Path, $Value) { [IO.File]::WriteAllText($Path, ($Value | ConvertTo-Json -Depth 12 -Compress), $Utf8NoBom) }
function Invoke-Logged([string]$File, [object[]]$Arguments, [string]$Stdout, [string]$Stderr) {
  $prior = $ErrorActionPreference; $ErrorActionPreference = 'Continue'; $global:LASTEXITCODE = $null
  try { & $File @Arguments 1> $Stdout 2> $Stderr; $code = $global:LASTEXITCODE }
  finally { $ErrorActionPreference = $prior }
  if ($null -eq $code -or $code -ne 0) {
    $tail = if (Test-Path -LiteralPath $Stderr) { (Get-Content -LiteralPath $Stderr -Tail 20) -join [Environment]::NewLine } else { '<no stderr>' }
    throw "Command failed with exit $code`: $File $($Arguments -join ' ')`n$tail"
  }
}
function Get-NormalName([string]$Name) { $Name.ToLowerInvariant().Replace('_', '-').Replace('.', '-') }
function Get-LockInventory([string]$Path) {
  $result = @{}
  foreach ($line in Get-Content -LiteralPath $Path) { if ($line -match '^([A-Za-z0-9_.-]+)==([^\s]+)\s') {
    $name = Get-NormalName $Matches[1]; Assert-Condition (-not $result.ContainsKey($name)) "Duplicate lock entry: $name"; $result[$name] = $Matches[2]
  } }
  Assert-Condition ($result.Count -gt 0) "No packages found in $Path"; return $result
}
function Assert-Inventory($ExpectedInventory, [string]$JsonPath) {
  $actual = @{}; foreach ($item in (Get-Content -LiteralPath $JsonPath -Raw | ConvertFrom-Json)) {
    $actual[(Get-NormalName $item.name)] = [string]$item.version
  }
  Assert-Condition ($actual.Count -eq $ExpectedInventory.Count) 'Installed package count differs from the lock'
  foreach ($name in $ExpectedInventory.Keys) { Assert-Condition ($actual.ContainsKey($name) -and $actual[$name] -eq $ExpectedInventory[$name]) "Installed package differs from lock: $name" }
  return $actual
}
function Get-FileRecords([string]$Root) {
  $records = [Collections.Generic.SortedDictionary[string,object]]::new([StringComparer]::Ordinal)
  foreach ($file in Get-ChildItem -LiteralPath $Root -File -Recurse) { $path = $file.FullName.Substring($Root.Length).TrimStart('\').Replace('\', '/'); $records.Add($path, [ordered]@{ path = $path; size = $file.Length; sha256 = Get-Hash $file.FullName }) }
  return @($records.Values)
}
function Get-NormalText([string]$Text, [string]$PassRoot) { $result = $Text.Replace('\', '/'); foreach ($pair in @(@{ path = $PassRoot; token = '<PASS>' }, @{ path = $BuildRoot; token = '<BUILD_ROOT>' }, @{ path = $Repo; token = '<REPO>' }, @{ path = $PythonHome; token = '<PYTHON_ROOT>' }, @{ path = $WindowsRoot; token = '<WINDOWS_ROOT>' })) { $result = [regex]::Replace($result, [regex]::Escape($pair.path.Replace('\', '/')), $pair.token, [Text.RegularExpressions.RegexOptions]::IgnoreCase) }; return $result }
function Get-Warnings([string]$Stderr, [string]$PassRoot) {
  $rows = [Collections.Generic.SortedSet[string]]::new([StringComparer]::Ordinal)
  foreach ($line in Get-Content -LiteralPath $Stderr) { if ($line -match '\bWARNING:\s*(.+)$') { [void]$rows.Add(('WARNING: ' + (Get-NormalText $Matches[1].Trim() $PassRoot))) } }
  return @($rows)
}
function Test-GraphPrefix($Graph, [string]$Prefix) { foreach ($row in @($Graph.pure) + @($Graph.binaries) + @($Graph.datas)) {
    $name = ([string]$row.name).Replace('\', '/'); if ($name.Equals($Prefix, [StringComparison]::OrdinalIgnoreCase) -or $name.StartsWith($Prefix + '.', [StringComparison]::OrdinalIgnoreCase) -or $name.StartsWith($Prefix + '/', [StringComparison]::OrdinalIgnoreCase)) { return $true }
  }
  return $false
}
function Assert-SameStrings($Actual, $ExpectedRows, [string]$Label) { [string[]]$left = @($Actual | ForEach-Object { [string]$_ }); [string[]]$right = @($ExpectedRows | ForEach-Object { [string]$_ }); [Array]::Sort($left, [StringComparer]::Ordinal); [Array]::Sort($right, [StringComparer]::Ordinal); Assert-Condition (($left | ConvertTo-Json -Compress) -ceq ($right | ConvertTo-Json -Compress)) "$Label differs" }
function Assert-Report($Report, [string]$Profile, [string]$Package, [string]$SitePackages) {
  Assert-Condition ($Report.schema -eq 1 -and $Report.profile -eq $Profile) 'Analysis report identity differs'
  $hmi = @($Report.graphs | Where-Object name -eq 'hmi')
  Assert-Condition ($hmi.Count -eq 1) 'Exactly one HMI graph is required'
  Assert-SameStrings @($hmi[0].excludes) $HmiExcludes 'HMI exclusions'
  foreach ($name in $HmiExcludes) { Assert-Condition (-not (Test-GraphPrefix $hmi[0] $name)) "HMI graph contains $name" }
  if ($Profile -eq 'base') {
    Assert-SameStrings @($Report.outputs) @('AR4HMI.exe') 'Base outputs'
    Assert-Condition (@($Report.graphs).Count -eq 1) 'Base report contains a worker graph'
  } else {
    Assert-SameStrings @($Report.outputs) @('AR4HMI.exe', 'AR4StepWorker.exe') 'STEP outputs'
    $worker = @($Report.graphs | Where-Object name -eq 'worker')
    Assert-Condition ($worker.Count -eq 1) 'Exactly one STEP worker graph is required'
    Assert-SameStrings @($worker[0].excludes) $WorkerExcludes 'Worker exclusions'
    foreach ($name in @('cadquery', 'OCP')) { Assert-Condition (Test-GraphPrefix $worker[0] $name) "Worker graph lacks $name" }
    foreach ($name in $WorkerExcludes) { Assert-Condition (-not (Test-GraphPrefix $worker[0] $name)) "Worker graph contains excluded $name" }
    $allowedCasadi = @('_casadi.pyd', 'libcasadi.dll', 'libgcc_s_seh-1.dll', 'libstdc++-6.dll', 'libwinpthread-1.dll')
    foreach ($row in @($worker[0].binaries)) {
      $source = ([string]$row.source).Replace('\', '/')
      if ($source -match '/casadi/[^/]+\.(dll|pyd)$') { Assert-Condition ($allowedCasadi -contains [IO.Path]::GetFileName($source) -and ([string]$row.name).Replace('\', '/') -eq "casadi/$([IO.Path]::GetFileName($source))") "Inactive or relocated CasADi binary collected: $source" }
    }
  }
  Assert-SameStrings @($Report.applicationData) $AppData 'Application data contract'
  $ttkManifest = Get-Content -LiteralPath (Join-Path $SitePackages 'ttkbootstrap\assets\elements\manifest.json') -Raw | ConvertFrom-Json
  $imageFiles = @($ttkManifest.images.psobject.Properties | ForEach-Object { $_.Value.file })
  Assert-Condition ($imageFiles.Count -gt 0 -and -not ($imageFiles -notmatch '^[^/\\]+\.png$')) 'ttkbootstrap element manifest differs'
  Assert-SameStrings @($Report.ttkData) @($TtkFixed + @($imageFiles | ForEach-Object { "ttkbootstrap/assets/elements/$_" })) 'ttkbootstrap data contract'
  $collectionRows = @($Report.graphs | ForEach-Object { @($_.binaries) + @($_.datas) })
  foreach ($group in @($collectionRows | Group-Object { ([string]$_.name).Replace('\', '/').ToLowerInvariant() })) {
    Assert-Condition (@($group.Group | ForEach-Object { "$(([string]$_.source).ToLowerInvariant())|$($_.type)" } | Sort-Object -Unique).Count -le 1) "Conflicting shared destination: $($group.Name)"
  }
  $dataRows = @($Report.graphs | ForEach-Object { @($_.datas) })
  foreach ($name in @($Report.applicationData) + @($Report.ttkData)) {
    $matches = @($dataRows | Where-Object { $_.name.Replace('\', '/') -eq $name.Replace('\', '/') })
    Assert-Condition ($matches.Count -eq 1) "Packaged data mapping differs: $name"
    $destination = Join-Path $Package $name
    Assert-Condition (Test-Path -LiteralPath $destination -PathType Leaf) "Packaged data missing: $name"
    $expectedSource = if ($AppData -contains $name) { Join-Path $Repo $name } else { Join-Path $SitePackages $name }
    Assert-Condition ([IO.Path]::GetFullPath($matches[0].source).Equals([IO.Path]::GetFullPath($expectedSource), [StringComparison]::OrdinalIgnoreCase)) "Packaged data source differs: $name"
    Assert-Condition ((Get-Hash $destination) -eq (Get-Hash $expectedSource)) "Packaged data differs: $name"
  }
}
function Invoke-Worker([string]$Executable, [string]$Directory, [string]$ArgumentLine, [int]$ExpectedStatus) {
  $start = [Diagnostics.ProcessStartInfo]::new($Executable, $ArgumentLine); $start.WorkingDirectory = $Directory
  $start.UseShellExecute = $false; $start.CreateNoWindow = $true; $start.RedirectStandardOutput = $true; $start.RedirectStandardError = $true
  $process = [Diagnostics.Process]::new(); $process.StartInfo = $start; try {
    Assert-Condition $process.Start() 'Packaged STEP worker did not start'; $stdout = $process.StandardOutput.ReadToEndAsync(); $stderr = $process.StandardError.ReadToEndAsync()
    if (-not $process.WaitForExit(30000)) { $process.Kill(); $process.WaitForExit(); throw 'Packaged STEP worker timed out' }
    $process.WaitForExit(); $null = $stdout.Result; $null = $stderr.Result; Assert-Condition ($process.ExitCode -eq $ExpectedStatus) "Worker status $($process.ExitCode), expected $ExpectedStatus"
  } finally { $process.Dispose() }
}
$Repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
foreach ($path in @($PythonPath, $PythonInstallerPath, $BuildRoot)) {
  Assert-Condition ($path -match '^(?:[A-Za-z]:[\\/]|\\\\[^\\/]+[\\/][^\\/]+)') "Path must be fully qualified: $path"
}
$PythonPath = [IO.Path]::GetFullPath($PythonPath); $PythonHome = Split-Path -Parent $PythonPath; $PythonInstallerPath = [IO.Path]::GetFullPath($PythonInstallerPath)
$BuildRoot = [IO.Path]::GetFullPath($BuildRoot); Assert-Condition (-not $BuildRoot.Equals([IO.Path]::GetPathRoot($BuildRoot), [StringComparison]::OrdinalIgnoreCase)) 'Build root cannot be a filesystem root'; $BuildRoot = $BuildRoot.TrimEnd('\')
for ($cursor = Split-Path -Parent $BuildRoot; $cursor; $cursor = Split-Path -Parent $cursor) { $ancestor = Get-Item -LiteralPath $cursor -Force; Assert-Condition (-not ($ancestor.Attributes -band [IO.FileAttributes]::ReparsePoint)) "Build root ancestor is a reparse point: $cursor"; if ($cursor.Equals([IO.Path]::GetPathRoot($cursor), [StringComparison]::OrdinalIgnoreCase)) { break } }
Assert-Condition (Test-Path -LiteralPath $PythonPath -PathType Leaf) 'Python executable is missing'
Assert-Condition (Test-Path -LiteralPath $PythonInstallerPath -PathType Leaf) 'Python installer is missing'
$WindowsRoot = [Environment]::GetEnvironmentVariable('SystemRoot'); Assert-Condition (Test-Path -LiteralPath $WindowsRoot -PathType Container) 'Windows system root is unavailable'
$comparison = [StringComparison]::OrdinalIgnoreCase
Assert-Condition (-not ($BuildRoot.StartsWith($Repo + '\', $comparison) -or $Repo.StartsWith($BuildRoot + '\', $comparison) -or $BuildRoot -eq $Repo)) 'Build root must be external to the checkout'
if (Test-Path -LiteralPath $BuildRoot) {
  $rootItem = Get-Item -LiteralPath $BuildRoot -Force
  Assert-Condition ($rootItem.PSIsContainer -and -not ($rootItem.Attributes -band [IO.FileAttributes]::ReparsePoint)) 'Build root must be a normal directory'
  Assert-Condition (@(Get-ChildItem -LiteralPath $BuildRoot -Force).Count -eq 0) 'Build root must be empty'
} else { New-Item -ItemType Directory -Path $BuildRoot | Out-Null }
[void][DateTimeOffset]::FromUnixTimeSeconds($SourceDateEpoch)
Assert-Condition ((Get-Hash $PythonInstallerPath) -eq $InstallerHash) 'CPython installer hash differs'
$RuntimeFiles = @{ $PythonPath = $RuntimeHashes['python.exe']; (Join-Path $PythonHome 'python314.dll') = $RuntimeHashes['python314.dll'] }
foreach ($path in $RuntimeFiles.Keys) { $signature = Get-AuthenticodeSignature -LiteralPath $path; Assert-Condition ((Get-Hash $path) -eq $RuntimeFiles[$path] -and $signature.Status -eq 'Valid' -and $signature.SignerCertificate.Thumbprint -eq $RuntimeSigner) "Official CPython runtime differs: $path" }
$probeOut = Join-Path $BuildRoot 'python.json'
Invoke-Logged $PythonPath @('-I', '-c',
  "import json,platform,struct,sys,sysconfig;print(json.dumps({'executable':sys.executable,'implementation':platform.python_implementation(),'machine':platform.machine(),'bits':struct.calcsize('P')*8,'version':platform.python_version(),'gilDisabled':bool(sysconfig.get_config_var('Py_GIL_DISABLED'))}))") $probeOut (Join-Path $BuildRoot 'python.stderr')
$probe = Get-Content -LiteralPath $probeOut -Raw | ConvertFrom-Json
Assert-Condition ($probe.implementation -eq 'CPython' -and $probe.version -eq '3.14.7' -and $probe.machine -eq 'AMD64' -and $probe.bits -eq 64 -and -not $probe.gilDisabled) 'Python runtime contract differs'
Assert-Condition ([IO.Path]::GetFullPath($probe.executable) -eq $PythonPath) 'Python probe used another executable'
$DriverHash = Get-Hash $PSCommandPath; $GitPath = (Get-Command git -CommandType Application).Source; Assert-Condition (Test-Path -LiteralPath $GitPath -PathType Leaf) 'Git executable is unavailable'
$Profiles = @{ base = @{ lock = Join-Path $Repo 'requirements-windows-base.lock'; spec = Join-Path $PSScriptRoot 'AR4HMI-base.spec' };
  step = @{ lock = Join-Path $Repo 'requirements-windows-step.lock'; spec = Join-Path $PSScriptRoot 'AR4HMI-step.spec' } }
foreach ($profile in $Profiles.Values) { Assert-Condition (Test-Path -LiteralPath $profile.lock -PathType Leaf) 'Profile lock is missing'; Assert-Condition (Test-Path -LiteralPath $profile.spec -PathType Leaf) 'Profile spec is missing' }
Assert-Condition ((Get-Hash $Profiles.base.lock) -eq '470b487a3c88e9fa7ee51f2d51c62548df47166cb02ed1d68268484208da7862') 'Base lock hash differs'
Assert-Condition ((Get-Hash $Profiles.step.lock) -eq '1aaeed5972586d1b8b83c9f9e5a7c0c1ebf280c1fb8955ea4e98dc4395122a2e') 'STEP lock hash differs'
$SpecHashes = @{ base = Get-Hash $Profiles.base.spec; step = Get-Hash $Profiles.step.spec }
$NoticeProgram = @'
import hashlib, importlib.metadata as md, json, os, re, sys
rows = []
for dist in sorted(md.distributions(), key=lambda d: (d.metadata.get("Name") or "").lower()):
    entries = [(str(entry).replace("\\", "/"), dist.locate_file(entry)) for entry in dist.files or ()]
    selected = {relative:source for relative, source in entries if re.search(r"(^|/)(license|copying|notice)[^/]*$", relative, re.I)}
    for declared in dist.metadata.get_all("License-File") or ():
        declared = declared.replace("\\", "/")
        matches = [(relative, source) for relative, source in entries if re.fullmatch(r"[^/]+\.dist-info/(?:licenses/)?" + re.escape(declared), relative, re.I)]
        if len(matches) != 1: raise ValueError(f"License-File does not resolve exactly once: {dist.metadata.get('Name')}:{declared}")
        selected[matches[0][0]] = matches[0][1]
    if any(not os.path.isfile(source) for source in selected.values()): raise ValueError(f"License inventory contains a missing file: {dist.metadata.get('Name')}")
    licenses = []
    for relative, source in sorted(selected.items()):
        payload = open(source, "rb").read()
        licenses.append({"path":relative,"size":len(payload),"sha256":hashlib.sha256(payload).hexdigest()})
    rows.append({"name":dist.metadata.get("Name") or "","version":dist.version,"license":dist.metadata.get("License-Expression") or dist.metadata.get("License") or "","files":sorted(licenses,key=lambda row:row["path"])})
def native_files(name, patterns):
    try: files = md.distribution(name).files or ()
    except md.PackageNotFoundError: return []
    paths = (str(entry).replace("\\", "/") for entry in files)
    return sorted(path for path in paths if any(re.fullmatch(pattern, path, re.I) for pattern in patterns))
native = native_files("vtk", (r"vtk\.libs/[^/]+\.dll", r"vtkmodules/[^/]+\.pyd"))
native += native_files("cadquery-ocp", (r"cadquery_ocp\.libs/[^/]+\.dll", r"OCP/OCP\.cp314-win_amd64\.pyd"))
python_license = os.path.join(sys.base_prefix, "LICENSE.txt")
payload = open(python_license, "rb").read()
report = {"python":{"path":"LICENSE.txt","size":len(payload),"sha256":hashlib.sha256(payload).hexdigest()},"distributions":rows,"nativeClosure":sorted(native)}
with open(sys.argv[1], "w", encoding="utf-8", newline="\n") as stream: json.dump(report, stream, sort_keys=True, separators=(",",":")); stream.write("\n")
'@
$NoticeScript = Join-Path $BuildRoot 'notice_inventory.py'; [IO.File]::WriteAllText($NoticeScript, $NoticeProgram, $Utf8NoBom)
$processEnvironment = [Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal); foreach ($entry in [Environment]::GetEnvironmentVariables('Process').GetEnumerator()) { $processEnvironment.Add([string]$entry.Key, [string]$entry.Value) }; $savedPath = [Environment]::GetEnvironmentVariable('Path', 'Process')
$savedEnvironment = [Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
$gitIdentityPattern = '^(?i:GIT_(?:DIR|WORK_TREE|INDEX_FILE|OBJECT_DIRECTORY|ALTERNATE_OBJECT_DIRECTORIES|COMMON_DIR|NAMESPACE|CEILING_DIRECTORIES|DISCOVERY_ACROSS_FILESYSTEM|CONFIG(?:_.*)?|ATTR_NOSYSTEM|ATTR_SOURCE|EXTERNAL_DIFF|DIFF_OPTS|GRAFT_FILE|SHALLOW_FILE|NO_REPLACE_OBJECTS|REPLACE_REF_BASE))$'
$environmentNames = @(@('PIP_CONFIG_FILE', 'PYTHONUTF8', 'PYTHONIOENCODING',
  'PYTHONHASHSEED', 'PYTHONNOUSERSITE', 'PYTHONOPTIMIZE', 'SOURCE_DATE_EPOCH', 'PYINSTALLER_CONFIG_DIR',
  'TEMP', 'TMP', 'TMPDIR', 'AR4HMI_ANALYSIS_REPORT', 'PYINSTALLER_RESET_ENVIRONMENT',
  'PYTHONHOME', 'PYTHONPATH', 'PYTHONUSERBASE', 'UPX', 'UPX_DIR', '_MEIPASS2') + @($processEnvironment.Keys | Where-Object { $_ -like 'PYTHON*' -or $_ -match $gitIdentityPattern -or $_ -like 'PYINSTALLER*' -or $_ -like '_PYINSTALLER*' -or $_ -like 'PYI_*' -or $_ -like '_PYI_*' })) | Sort-Object -CaseSensitive -Unique
foreach ($name in $environmentNames) { $savedEnvironment[$name] = if ($processEnvironment.ContainsKey($name)) { $processEnvironment[$name] } else { $null } }
$Results = @(); $States = @{}
try { foreach ($name in @($environmentNames | Where-Object { $_ -like 'PYTHON*' -or $_ -match $gitIdentityPattern -or $_ -like 'PYINSTALLER*' -or $_ -like '_PYINSTALLER*' -or $_ -like 'PYI_*' -or $_ -like '_PYI_*' })) { [Environment]::SetEnvironmentVariable($name, $null, 'Process') }
  $env:PIP_CONFIG_FILE = 'NUL'; $env:PYTHONUTF8 = '1'; $env:PYTHONIOENCODING = 'utf-8'
  $env:PYTHONHASHSEED = '1'; $env:PYTHONNOUSERSITE = '1'; $env:PYTHONOPTIMIZE = '0'; $env:SOURCE_DATE_EPOCH = [string]$SourceDateEpoch; [Environment]::SetEnvironmentVariable('PATH', $null, 'Process'); [Environment]::SetEnvironmentVariable('Path', $null, 'Process'); [Environment]::SetEnvironmentVariable('Path', "$PythonHome;$WindowsRoot\System32;$WindowsRoot", 'Process')
  foreach ($name in @('PYINSTALLER_RESET_ENVIRONMENT', 'UPX', 'UPX_DIR', '_MEIPASS2')) { Remove-Item "Env:$name" -ErrorAction SilentlyContinue }
  $StartCommit = (Invoke-GitText @('--no-replace-objects', '-C', $Repo, 'rev-parse', 'HEAD')).Trim(); $StartTree = (Invoke-GitText @('--no-replace-objects', '-C', $Repo, 'rev-parse', 'HEAD^{tree}')).Trim(); $TrackedDiffHash = Get-TextHash (Invoke-GitText @('--no-replace-objects', '-C', $Repo, 'diff', '--binary', '--no-ext-diff', 'HEAD', '--')); Assert-Condition ($StartCommit -match '^[0-9a-f]{40}$' -and $StartTree -match '^[0-9a-f]{40}$') 'Git source identity is malformed'
  foreach ($profileName in @('base', 'step')) {
    $profile = $Profiles[$profileName]; $envRoot = Join-Path $BuildRoot "env\$profileName"
    $logs = Join-Path $envRoot 'logs'; $venv = Join-Path $envRoot 'venv'; $env:TEMP = Join-Path $envRoot 'temp'; $env:TMP = $env:TEMP; $env:TMPDIR = $env:TEMP
    New-Item -ItemType Directory -Path $logs,$env:TEMP -Force | Out-Null
    Invoke-Logged $PythonPath @('-I', '-m', 'venv', $venv) (Join-Path $logs 'venv.stdout') (Join-Path $logs 'venv.stderr')
    $venvPython = Join-Path $venv 'Scripts\python.exe'
    Invoke-Logged $venvPython @('-s', '-m', 'pip', '--isolated', 'install', '--disable-pip-version-check',
      '--require-hashes', '--only-binary=:all:', '--no-deps', '--no-compile', '-r', $profile.lock) (Join-Path $logs 'pip-install.stdout') (Join-Path $logs 'pip-install.stderr')
    Invoke-Logged $venvPython @('-s', '-m', 'pip', '--isolated', 'check') (Join-Path $logs 'pip-check.stdout') (Join-Path $logs 'pip-check.stderr')
    $listPath = Join-Path $logs 'pip-list.json'
    Invoke-Logged $venvPython @('-s', '-m', 'pip', '--isolated', 'list', '--format=json') $listPath (Join-Path $logs 'pip-list.stderr')
    $versions = Assert-Inventory (Get-LockInventory $profile.lock) $listPath
    $States[$profileName] = @{ python = $venvPython; versions = $versions; sitePackages = Join-Path $venv 'Lib\site-packages' }
  }
  foreach ($pass in 1..2) { foreach ($profileName in @('base', 'step')) {
    $profile = $Profiles[$profileName]; $state = $States[$profileName]; $passRoot = Join-Path $BuildRoot "pass-$pass\$profileName"
    $work = Join-Path $passRoot 'work'; $dist = Join-Path $passRoot 'dist'; $logs = Join-Path $passRoot 'logs'
    $temp = Join-Path $passRoot 'temp'; $config = Join-Path $passRoot 'config'
    foreach ($directory in @($passRoot, $work, $dist, $logs, $temp, $config)) { New-Item -ItemType Directory -Path $directory -Force | Out-Null }
    $env:TEMP = $temp; $env:TMP = $temp; $env:TMPDIR = $temp; $env:PYINSTALLER_CONFIG_DIR = $config
    $env:AR4HMI_ANALYSIS_REPORT = Join-Path $passRoot 'analysis.json'; $venvPython = $state.python
    $noticePath = Join-Path $passRoot 'distribution-notices.json'
    Invoke-Logged $venvPython @('-s', $NoticeScript, $noticePath) (Join-Path $logs 'notices.stdout') (Join-Path $logs 'notices.stderr')
    $notice = Get-Content -LiteralPath $noticePath -Raw -Encoding UTF8 | ConvertFrom-Json
    Assert-Condition (@($notice.nativeClosure -match '^vtk\.libs/').Count -gt 0 -and @($notice.nativeClosure -match '^vtkmodules/').Count -gt 0 -and (($profileName -eq 'base' -and @($notice.nativeClosure -match '^(cadquery_ocp\.libs|OCP)/').Count -eq 0) -or ($profileName -eq 'step' -and @($notice.nativeClosure -match '^cadquery_ocp\.libs/').Count -gt 0 -and @($notice.nativeClosure -match '^OCP/OCP\.cp314-win_amd64\.pyd$').Count -eq 1))) "$profileName wheel-native closure differs"
    $buildOut = Join-Path $logs 'pyinstaller.stdout'; $buildErr = Join-Path $logs 'pyinstaller.stderr'; $comtypesGen = Join-Path $state.sitePackages 'comtypes\gen'; if (Test-Path -LiteralPath $comtypesGen) { Remove-Item -LiteralPath $comtypesGen -Recurse -Force -ErrorAction Stop }; Assert-Condition (-not (Test-Path -LiteralPath $comtypesGen)) 'comtypes generated package reset failed'
    Push-Location $Repo
    try { Invoke-Logged $venvPython @('-s', '-m', 'PyInstaller', '--clean', '--noconfirm',
        '--distpath', $dist, '--workpath', $work, $profile.spec) $buildOut $buildErr }
    finally { Pop-Location }
    Assert-Condition (Test-Path -LiteralPath $env:AR4HMI_ANALYSIS_REPORT -PathType Leaf) 'Analysis report was not emitted'
    $package = Join-Path $dist "AR4HMI-$profileName"
    Assert-Condition (Test-Path -LiteralPath $package -PathType Container) 'Package directory is missing'
    Assert-SameStrings @(Get-ChildItem -LiteralPath $dist -Force | ForEach-Object Name) @("AR4HMI-$profileName") 'Distribution root contents'
    $report = Get-Content -LiteralPath $env:AR4HMI_ANALYSIS_REPORT -Raw -Encoding UTF8 | ConvertFrom-Json
    Assert-Report $report $profileName $package $state.sitePackages
    $executables = @(Get-ChildItem -LiteralPath $package -Filter '*.exe' -File -Recurse | ForEach-Object { $_.FullName.Substring($package.Length).TrimStart('\').Replace('\', '/') })
    Assert-SameStrings $executables @($report.outputs) 'Package executables'
    Assert-Condition (-not (Test-Path -LiteralPath (Join-Path $package '_internal'))) 'Flat package contains _internal'
    $kinematics = @(Get-ChildItem -LiteralPath $package -Filter 'robot_kinematics*' -File -Recurse)
    Assert-Condition ($kinematics.Count -eq 1 -and $kinematics[0].FullName.Equals((Join-Path $package 'ARrobots\robot_kinematics.cp314-win_amd64.pyd'), [StringComparison]::OrdinalIgnoreCase) -and
      (Get-Hash $kinematics[0].FullName) -eq $NativeHash) 'Packaged kinematics artifact differs'
    $peOut = Join-Path $logs 'kinematics-imports.json'
    Invoke-Logged $venvPython @('-I', '-c',
      'import json,pefile,sys;p=pefile.PE(sys.argv[1]);print(json.dumps(sorted(e.dll.decode().lower() for e in p.DIRECTORY_ENTRY_IMPORT)))',
      $kinematics[0].FullName) $peOut (Join-Path $logs 'kinematics-imports.stderr')
    $imports = Get-Content -LiteralPath $peOut -Raw | ConvertFrom-Json
    Assert-Condition ($imports -contains 'python314.dll' -and -not ($imports -match 'python312|python314t')) 'Kinematics Python import differs'
    $records = Get-FileRecords $package
    $paths = @($records | ForEach-Object path)
    $nativeClosure = @($paths | Where-Object { $_ -match '(?i)^(vtk\.libs/[^/]+\.dll|vtkmodules/[^/]+\.pyd|cadquery_ocp\.libs/[^/]+\.dll|OCP/OCP\.cp314-win_amd64\.pyd)$' })
    Assert-SameStrings $nativeClosure @($notice.nativeClosure) "$profileName locked-wheel native closure"
    Assert-Condition (-not ($paths -match '(?i)opencv_videoio_ffmpeg|libcoinmetis|nloptjni|tbb12|(^|/)(_?pyinstaller_hooks_contrib|_distutils_hack|_yaml|adodbapi|aiohappyeyeballs|aiohttp|aiosignal|altgraph|attr|attrs|cadquery_ocp_proxy|distutils-precedence|frozenlist|ipython|llvmlite|more_itertools|msgpack|multidict|numba|ordlookup|pefile|peutils|pip|propcache|pyinstaller|pyyaml|scipy|setuptools|trame(?:_[^/]+)?|win32ctypes|wslink|yaml|yarl)([/\.-]|$)')) 'Denied package artifact was collected'
    if ($profileName -eq 'base') {
      Assert-Condition (-not ($paths -match '(?i)(^|/)(AR4StepWorker\.exe|cadquery|cadquery_ocp|cadquery_ocp_proxy|casadi|nlopt|OCP|numba|llvmlite)([/\.-]|$)')) 'Base package contains a STEP-only artifact'
    } else {
      Assert-SameStrings @($paths | Where-Object { $_ -match '(?i)^(casadi/[^/]+\.(dll|pyd)|nlopt/[^/]+\.(dll|pyd))$' }) $StepSolverClosure 'STEP solver native closure'
      $before = $records | ConvertTo-Json -Depth 4 -Compress
      $env:PYINSTALLER_RESET_ENVIRONMENT = '1'
      Invoke-Worker (Join-Path $package 'AR4StepWorker.exe') $logs '' 64
      [IO.File]::WriteAllText((Join-Path $logs 'malformed.step'), 'not step', $Utf8NoBom)
      Invoke-Worker (Join-Path $package 'AR4StepWorker.exe') $logs 'malformed.step output.stl' 3
      Assert-Condition (-not (Test-Path -LiteralPath (Join-Path $logs 'output.stl'))) 'Malformed STEP produced output'
      Assert-Condition ($before -ceq ((Get-FileRecords $package) | ConvertTo-Json -Depth 4 -Compress)) 'Worker changed package files'
      Remove-Item Env:PYINSTALLER_RESET_ENVIRONMENT -ErrorAction SilentlyContinue
    }
    $normalizedReport = Get-NormalText (Get-Content -LiteralPath $env:AR4HMI_ANALYSIS_REPORT -Raw) $passRoot
    $normalizedPath = Join-Path $passRoot 'analysis-normalized.json'
    [IO.File]::WriteAllText($normalizedPath, $normalizedReport, $Utf8NoBom)
    $Results += [pscustomobject]@{ pass = $pass; profile = $profileName; root = $passRoot;
      package = $package; records = $records; graphHash = Get-Hash $normalizedPath;
      noticeHash = Get-Hash $noticePath; warnings = @(Get-Warnings $buildErr $passRoot);
      versions = $state.versions; lockHash = Get-Hash $profile.lock; specHash = Get-Hash $profile.spec }
  } }
$observed = [ordered]@{}
foreach ($profileName in @('base', 'step')) {
  $runs = @($Results | Where-Object profile -eq $profileName | Sort-Object pass)
  Assert-Condition ($runs.Count -eq 2) "Two $profileName builds are required"
  Assert-Condition ($runs[0].graphHash -eq $runs[1].graphHash) "$profileName analysis graphs differ between passes"
  Assert-Condition ($runs[0].noticeHash -eq $runs[1].noticeHash) "$profileName notice reports differ between passes"
  Assert-SameStrings $runs[0].warnings $runs[1].warnings "$profileName warnings between passes"
  Assert-Condition (($runs[0].records | ConvertTo-Json -Depth 4 -Compress) -ceq
    ($runs[1].records | ConvertTo-Json -Depth 4 -Compress)) "$profileName package files differ between passes"
  $observed[$profileName] = [ordered]@{ graph = $runs[0].graphHash; notices = $runs[0].noticeHash;
    warnings = @($runs[0].warnings) }
}
$observedPath = Join-Path $BuildRoot 'observed-contracts.json'; Write-Json $observedPath $observed
foreach ($profileName in @('base', 'step')) {
  Assert-Condition ($Expected[$profileName].graph -eq $observed[$profileName].graph) "Review graph hash in $observedPath"
  Assert-Condition ($Expected[$profileName].notices -eq $observed[$profileName].notices) "Review notice hash in $observedPath"
  Assert-SameStrings $observed[$profileName].warnings @($Expected[$profileName].warnings.Keys) "Review warning dispositions in $observedPath"
  foreach ($warning in $observed[$profileName].warnings) { Assert-Condition (-not [string]::IsNullOrWhiteSpace([string]$Expected[$profileName].warnings[$warning])) "Warning disposition is blank: $warning" }
}
foreach ($result in $Results) {
  $warningRows = @($result.warnings | ForEach-Object { [ordered]@{ key = $_; disposition = $Expected[$result.profile].warnings[$_] } })
  $manifest = [ordered]@{ schema = 1; profile = $result.profile;
    source = [ordered]@{ commit = $StartCommit; tree = $StartTree; trackedDiffSha256 = $TrackedDiffHash };
    python = [ordered]@{ version = '3.14.7'; installerSha256 = $InstallerHash; executableSha256 = $RuntimeHashes['python.exe']; dllSha256 = $RuntimeHashes['python314.dll']; signerThumbprint = $RuntimeSigner };
    inputs = [ordered]@{ lockSha256 = $result.lockHash; specSha256 = $result.specHash; driverSha256 = $DriverHash };
    tools = [ordered]@{ pyinstaller = $result.versions['pyinstaller']; hooks = $result.versions['pyinstaller-hooks-contrib'] };
    build = [ordered]@{ canonicalRoot = $BuildRoot.Replace('\', '/'); sourceDateEpoch = $SourceDateEpoch;
      environment = [ordered]@{ PATH = '<PYTHON_ROOT>;<WINDOWS_ROOT>/System32;<WINDOWS_ROOT>'; PIP_CONFIG_FILE = 'NUL'; GIT_IDENTITY_OVERRIDES = '<UNSET>'; PYTHONHASHSEED = '1'; PYTHONOPTIMIZE = '0'; PYTHONNODEBUGRANGES = '<UNSET>'; TEMP = '<PASS>/temp'; TMP = '<PASS>/temp'; TMPDIR = '<PASS>/temp'; SOURCE_DATE_EPOCH = [string]$SourceDateEpoch } };
    diagnostics = [ordered]@{ analysisSha256 = $result.graphHash; noticeReportSha256 = $result.noticeHash;
      warnings = $warningRows; redistributionApproved = $false };
    files = $result.records }
  Write-Json (Join-Path $result.root 'manifest.json') $manifest
}
foreach ($profileName in @('base', 'step')) {
  $runs = @($Results | Where-Object profile -eq $profileName | Sort-Object pass)
  $first = Get-Content -LiteralPath (Join-Path $runs[0].root 'manifest.json') -Raw; $second = Get-Content -LiteralPath (Join-Path $runs[1].root 'manifest.json') -Raw
  Assert-Condition ($first -ceq $second) "$profileName manifests differ"
}
Assert-Condition ((Invoke-GitText @('--no-replace-objects', '-C', $Repo, 'rev-parse', 'HEAD')).Trim() -eq $StartCommit -and
  (Invoke-GitText @('--no-replace-objects', '-C', $Repo, 'rev-parse', 'HEAD^{tree}')).Trim() -eq $StartTree) 'Git source identity changed during build'
Assert-Condition ((Get-Hash $PSCommandPath) -eq $DriverHash -and (Get-Hash $PythonInstallerPath) -eq $InstallerHash) 'Build driver or CPython installer changed during build'
Assert-Condition ((Get-TextHash (Invoke-GitText @('--no-replace-objects', '-C', $Repo, 'diff', '--binary', '--no-ext-diff', 'HEAD', '--'))) -eq $TrackedDiffHash -and (Get-Hash $Profiles.base.spec) -eq $SpecHashes.base -and (Get-Hash $Profiles.step.spec) -eq $SpecHashes.step) 'Source inputs changed during build'
foreach ($path in $RuntimeFiles.Keys) { Assert-Condition ((Get-Hash $path) -eq $RuntimeFiles[$path]) "CPython runtime changed during build: $path" }
Assert-Condition ((Get-Hash $Profiles.base.lock) -eq '470b487a3c88e9fa7ee51f2d51c62548df47166cb02ed1d68268484208da7862' -and
  (Get-Hash $Profiles.step.lock) -eq '1aaeed5972586d1b8b83c9f9e5a7c0c1ebf280c1fb8955ea4e98dc4395122a2e') 'Lock changed during build'
Write-Output "Windows package evidence complete: $BuildRoot"
} finally {
  foreach ($name in $environmentNames) { [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process') }
  [Environment]::SetEnvironmentVariable('PATH', $null, 'Process'); [Environment]::SetEnvironmentVariable('Path', $null, 'Process'); [Environment]::SetEnvironmentVariable('Path', $savedPath, 'Process')
}
