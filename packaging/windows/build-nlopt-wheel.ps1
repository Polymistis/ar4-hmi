#Requires -Version 5.1
[CmdletBinding()]
param([Parameter(Mandatory)][string]$InputRoot, [Parameter(Mandatory)][string]$BuildRoot)
Set-StrictMode -Version Latest; $ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSEdition -ne 'Desktop' -or $PSVersionTable.PSVersion.ToString() -notlike '5.1.*') { throw 'Windows PowerShell 5.1 is required' }
$Utf8 = [Text.UTF8Encoding]::new($false); $StrictUtf8 = [Text.UTF8Encoding]::new($false, $true)
$Repo = (Get-Item -LiteralPath ([IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))) -Force).FullName.TrimEnd('\')
$LockPath = Join-Path $PSScriptRoot 'nlopt-build-lock.json'
$Python = 'C:\Program Files\Python314\python.exe'; $PythonHome = Split-Path -Parent $Python
$VsRoot = 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools'; $SdkRoot = 'C:\Program Files (x86)\Windows Kits\10'
$Cmake = Join-Path $VsRoot 'Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$MsBuild = Join-Path $VsRoot 'MSBuild\Current\Bin\amd64\MSBuild.exe'
$ToolBin = Join-Path $VsRoot 'VC\Tools\MSVC\14.50.35717\bin\Hostx64\x64'
$ResourceCompiler = Join-Path $SdkRoot 'bin\10.0.26100.0\x64\rc.exe'; $SystemRoot = [Environment]::GetEnvironmentVariable('SystemRoot')
Add-Type -AssemblyName System.IO.Compression
function Assert-Condition([bool]$Condition, [string]$Message) { if (-not $Condition) { throw $Message } }
function Assert-Equal($Actual, $Expected, [string]$Label) { if ($Actual -cne $Expected) { throw "$Label differs: expected $Expected; observed $Actual" } }
function Get-Hash([string]$Path) { (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant() }
function Get-TextHash([string]$Text) { ([BitConverter]::ToString([Security.Cryptography.SHA256]::Create().ComputeHash($Utf8.GetBytes($Text)))).Replace('-', '').ToLowerInvariant() }
function Assert-Shape($Value, [string[]]$Names, [string]$Label) {
  Assert-Condition ($Value -is [pscustomobject]) "$Label must be an object"; [string[]]$actual = @($Value.PSObject.Properties.Name); [string[]]$wanted = @($Names)
  [Array]::Sort($actual, [StringComparer]::Ordinal); [Array]::Sort($wanted, [StringComparer]::Ordinal); Assert-Equal ($actual -join "`0") ($wanted -join "`0") "$Label properties"
}
function Assert-Relative([string]$Path, [string]$Label) {
  Assert-Condition (-not [string]::IsNullOrWhiteSpace($Path) -and $Path -ceq $Path.Replace('\', '/') -and -not [IO.Path]::IsPathRooted($Path)) "$Label must be a normalized relative path"
  Assert-Condition (-not (@($Path.Split('/')) | Where-Object { $_ -eq '' -or $_ -eq '.' -or $_ -eq '..' })) "$Label contains an unsafe segment"
}
function Assert-Identity($Tuple, [bool]$HasPath, [string]$Label) {
  $count = if ($HasPath) { 3 } else { 2 }; Assert-Condition ($Tuple -is [Collections.IList] -and $Tuple.Count -eq $count) "$Label must contain $count fields"
  $size = if ($HasPath) { 1 } else { 0 }; if ($HasPath) { Assert-Condition ($Tuple[0] -is [string]) "$Label path must be a string"; Assert-Relative $Tuple[0] "$Label path" }
  Assert-Condition (($Tuple[$size] -is [int] -or $Tuple[$size] -is [long]) -and ([long]$Tuple[$size] -gt 0)) "$Label size is invalid"
  Assert-Condition ($Tuple[$size + 1] -is [string] -and $Tuple[$size + 1] -cmatch '^[0-9a-f]{64}$') "$Label SHA-256 is invalid"
}
function Assert-File([string]$Path, [long]$Size, [string]$Sha256, [string]$Label) {
  Assert-Condition (Test-Path -LiteralPath $Path -PathType Leaf) "$Label is missing"; Assert-Equal (Get-Item -LiteralPath $Path).Length $Size "$Label size"; Assert-Equal (Get-Hash $Path) $Sha256 "$Label SHA-256"
}
function Get-IdentityStream($Rows) {
  $ordered = [Collections.Generic.SortedDictionary[string,object]]::new([StringComparer]::Ordinal); foreach ($row in $Rows) { $ordered.Add([string]$row.path, $row) }
  return (@($ordered.Values | ForEach-Object { "$($_.path)|$(([long]$_.size).ToString([Globalization.CultureInfo]::InvariantCulture))|$($_.sha256)`n" }) -join '')
}
function Assert-Aggregate([string]$Text, $Tuple, [string]$Label) { Assert-Equal ([long]$Utf8.GetByteCount($Text)) ([long]$Tuple[0]) "$Label byte count"; Assert-Equal (Get-TextHash $Text) ([string]$Tuple[1]) "$Label SHA-256" }
function Assert-SameStrings($Actual, $Expected, [string]$Label) {
  [string[]]$left = @($Actual); [string[]]$right = @($Expected); [Array]::Sort($left, [StringComparer]::Ordinal); [Array]::Sort($right, [StringComparer]::Ordinal); Assert-Equal ($left -join "`0") ($right -join "`0") $Label
}
function Resolve-Absolute([string]$Path, [string]$Label) {
  Assert-Condition ($Path -match '^(?:[A-Za-z]:[\\/]|\\\\[^\\/]+[\\/][^\\/]+)') "$Label must be fully qualified"; $full = [IO.Path]::GetFullPath($Path)
  if (-not $full.Equals([IO.Path]::GetPathRoot($full), [StringComparison]::OrdinalIgnoreCase)) { $full = $full.TrimEnd('\') }; return $full
}
function Resolve-Prospective([string]$Path, [string]$Label) {
  $cursor = Resolve-Absolute $Path $Label; $tail = [Collections.Generic.Stack[string]]::new()
  while (-not (Test-Path -LiteralPath $cursor)) { $tail.Push([IO.Path]::GetFileName($cursor)); $cursor = [IO.Path]::GetDirectoryName($cursor) }
  $item = Get-Item -LiteralPath $cursor -Force; Assert-Condition $item.PSIsContainer "$Label existing ancestor is not a directory"; $result = $item.FullName
  while ($tail.Count) { $result = Join-Path $result $tail.Pop() }; return (Resolve-Absolute $result $Label)
}
function Test-Within([string]$Path, [string]$Root) { $Path.Equals($Root, [StringComparison]::OrdinalIgnoreCase) -or $Path.StartsWith($Root + '\', [StringComparison]::OrdinalIgnoreCase) }
function Invoke-Logged([string]$File, [object[]]$Arguments, [string]$Stdout, [string]$Stderr, [string]$Label) {
  $prior = $ErrorActionPreference; $ErrorActionPreference = 'Continue'; $global:LASTEXITCODE = $null
  try { & $File @Arguments 1> $Stdout 2> $Stderr; $code = $global:LASTEXITCODE } finally { $ErrorActionPreference = $prior }
  if ($null -eq $code -or $code -ne 0) { $tail = if (Test-Path -LiteralPath $Stderr) { (Get-Content -LiteralPath $Stderr -Tail 20) -join [Environment]::NewLine } else { '<no stderr>' }; throw "$Label failed with exit $code`n$tail" }
}
function Reset-Work {
  $work = [IO.Path]::GetFullPath((Join-Path $BuildRoot 'work')); Assert-Condition ($work.Equals($BuildRoot + '\work', [StringComparison]::OrdinalIgnoreCase)) 'Owned work path escaped the build root'
  if (Test-Path -LiteralPath $work) {
    $item = Get-Item -LiteralPath $work -Force
    Assert-Condition ($item.PSIsContainer -and -not ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)) 'Owned work path is not a normal directory'
    Assert-Condition (-not (Get-ChildItem -LiteralPath $work -Force -Recurse | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint } | Select-Object -First 1)) 'Owned work contains a reparse point'
    Remove-Item -LiteralPath $work -Recurse -Force
  }
  New-Item -ItemType Directory -Path $work | Out-Null; return $work
}
function Expand-LockedArchive($Tuple, [string]$Destination) {
  Assert-Condition (-not (Test-Path -LiteralPath $Destination)) "Archive destination already exists: $Destination"; $path = Join-Path $InputRoot $Tuple[0]
  $stream = [IO.File]::Open($path, [IO.FileMode]::Open, [IO.FileAccess]::Read, [IO.FileShare]::Read); $archive = $null
  try {
    Assert-Equal $stream.Length ([long]$Tuple[1]) "Archive $($Tuple[0]) size"; $sha = [Security.Cryptography.SHA256]::Create()
    try { $digest = ([BitConverter]::ToString($sha.ComputeHash($stream))).Replace('-', '').ToLowerInvariant() } finally { $sha.Dispose() }
    Assert-Equal $digest ([string]$Tuple[2]) "Archive $($Tuple[0]) SHA-256"; $stream.Position = 0; $archive = [IO.Compression.ZipArchive]::new($stream, [IO.Compression.ZipArchiveMode]::Read, $false)
    $records = [Collections.Generic.List[object]]::new(); $destinations = [Collections.Generic.HashSet[string]]::new([StringComparer]::OrdinalIgnoreCase)
    foreach ($entry in $archive.Entries) {
      $name = [string]$entry.FullName; $trimmed = $name.TrimEnd('/'); $segments = @($trimmed.Split('/')); $isDirectory = $name.EndsWith('/')
      Assert-Condition ($trimmed.Length -gt 0 -and -not $name.Contains('\') -and -not $name.Contains(':') -and -not $name.StartsWith('/') -and -not ($segments | Where-Object { $_ -eq '' -or $_ -eq '.' -or $_ -eq '..' })) "Unsafe archive member: $name"
      Assert-Condition ((($entry.ExternalAttributes -shr 16) -band 0xF000) -ne 0xA000) "Symbolic-link archive member: $name"
      $target = [IO.Path]::GetFullPath((Join-Path $Destination ($trimmed.Replace('/', '\')))); Assert-Condition (Test-Within $target $Destination) "Archive member escaped destination: $name"
      Assert-Condition $destinations.Add($target) "Duplicate archive destination: $name"; $records.Add([pscustomobject]@{ entry = $entry; target = $target; directory = $isDirectory })
    }
    New-Item -ItemType Directory -Path $Destination | Out-Null
    foreach ($row in $records) {
      if ($row.directory) { New-Item -ItemType Directory -Path $row.target -Force | Out-Null; continue }
      New-Item -ItemType Directory -Path (Split-Path -Parent $row.target) -Force | Out-Null; $output = [IO.File]::Open($row.target, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
      try { $input = $row.entry.Open(); try { $input.CopyTo($output) } finally { $input.Dispose() } } finally { $output.Dispose() }; [IO.File]::SetLastWriteTimeUtc($row.target, $row.entry.LastWriteTime.UtcDateTime)
    }
  } finally { if ($null -ne $archive) { $archive.Dispose() } else { $stream.Dispose() } }
}
function Expand-Source($Tuple, [string]$Destination, [string]$ExpectedRoot, [string]$Work) {
  $extract = Join-Path $Work ([IO.Path]::GetFileNameWithoutExtension([string]$Tuple[0]) + '-extract'); Expand-LockedArchive $Tuple $extract; $children = @(Get-ChildItem -LiteralPath $extract -Force)
  Assert-Condition ($children.Count -eq 1 -and $children[0].PSIsContainer -and $children[0].Name -ceq $ExpectedRoot) "Archive root differs: $ExpectedRoot"; Move-Item -LiteralPath $children[0].FullName -Destination $Destination
}
function Normalize-Cache([string]$Path) {
  $text = [IO.File]::ReadAllText($Path)
  foreach ($pair in @(@{ from = $BuildVenv; to = '<BUILD_VENV>' }, @{ from = $BuildRoot; to = '<ROOT>' }, @{ from = $PythonHome; to = '<PYTHON>' }, @{ from = $VsRoot; to = '<VS>' }, @{ from = $SdkRoot; to = '<SDK>' })) {
    $text = [regex]::Replace($text, [regex]::Escape($pair.from), $pair.to, [Text.RegularExpressions.RegexOptions]::IgnoreCase)
    $text = [regex]::Replace($text, [regex]::Escape($pair.from.Replace('\', '/')), $pair.to, [Text.RegularExpressions.RegexOptions]::IgnoreCase)
  }
  $text = $text.Replace("`r`n", "`n").Replace('\', '/')
  $pattern = '(?m)^(_Python_(?:INTERPRETER|NUMPY)_SIGNATURE:INTERNAL)=[0-9a-f]+$'
  Assert-Condition ([regex]::Matches($text, $pattern).Count -eq 2) 'Path-derived CMake signature inventory differs'
  return [regex]::Replace($text, $pattern, '$1=<PATH_DERIVED>')
}
Assert-Condition (Test-Path -LiteralPath $LockPath -PathType Leaf) 'NLopt build lock is missing'
$lockBytes = [IO.File]::ReadAllBytes($LockPath)
Assert-Condition ($lockBytes.Length -gt 0 -and -not ($lockBytes.Length -ge 3 -and $lockBytes[0] -eq 0xEF -and $lockBytes[1] -eq 0xBB -and $lockBytes[2] -eq 0xBF)) 'NLopt build lock must be nonempty UTF-8 without BOM'
$Lock = $StrictUtf8.GetString($lockBytes) | ConvertFrom-Json
Assert-Shape $Lock @('schema','redistributionApproved','inputs','tools','transform','build','outputs','probe','notices') 'lock'; Assert-Shape $Lock.inputs @('aggregate','upstream','packaging','pythonInstaller','swig','numpy','pip','setuptools') 'lock.inputs'
$ToolRoles = @('cl.exe','cmake.exe','cvtres.exe','lib.exe','link.exe','msbuild.exe','python.exe','python314.dll','rc.exe','swig.exe','venv-python.exe')
Assert-Shape $Lock.tools (@('aggregate') + $ToolRoles) 'lock.tools'; Assert-Shape $Lock.transform @('path','beforeSha256','afterSha256','needle','replacement') 'lock.transform'
Assert-Shape $Lock.build @('sourceDateEpoch','versionSuffix','compileFlags','linkFlags','cache') 'lock.build'; Assert-Shape $Lock.build.cache @('BUILD_SHARED_LIBS','NLOPT_LUKSAN','NLOPT_PYTHON_SABI') 'lock.build.cache'
Assert-Shape $Lock.outputs @('wheel','extension','normalizedCache') 'lock.outputs'; Assert-Shape $Lock.probe @('reject','exception','lastResult','control') 'lock.probe'; Assert-Shape $Lock.notices @('directory','aggregate','files','buildMode') 'lock.notices'
Assert-Condition ($Lock.schema -is [int] -and $Lock.schema -eq 1 -and $Lock.redistributionApproved -is [bool] -and -not $Lock.redistributionApproved) 'Lock identity or redistribution state differs'
foreach ($name in @('upstream','packaging','pythonInstaller','swig','numpy','pip','setuptools')) { Assert-Identity $Lock.inputs.$name $true "lock.inputs.$name" }; Assert-Identity $Lock.inputs.aggregate $false 'lock.inputs.aggregate'
foreach ($role in $ToolRoles) { Assert-Identity $Lock.tools.PSObject.Properties[$role].Value $false "lock.tools.$role" }; Assert-Identity $Lock.tools.aggregate $false 'lock.tools.aggregate'
foreach ($name in @('path','beforeSha256','afterSha256','needle','replacement')) { Assert-Condition ($Lock.transform.$name -is [string] -and $Lock.transform.$name.Length -gt 0) "lock.transform.$name is invalid" }
Assert-Relative $Lock.transform.path 'lock.transform.path'; Assert-Condition ($Lock.transform.beforeSha256 -cmatch '^[0-9a-f]{64}$' -and $Lock.transform.afterSha256 -cmatch '^[0-9a-f]{64}$') 'Transform SHA-256 is invalid'
Assert-Condition ($Lock.build.sourceDateEpoch -is [int]) 'Build epoch type differs'; Assert-Equal $Lock.build.sourceDateEpoch 315532800 'Build epoch'; Assert-Equal ([string]$Lock.build.versionSuffix) 'post1+ar4hmi.1' 'Version suffix'
Assert-Condition ($Lock.build.compileFlags -is [Collections.IList] -and $Lock.build.compileFlags.Count -eq 3) 'Compile flags type differs'
Assert-Equal ([string]$Lock.build.compileFlags[0]) '/Brepro' 'Compile flag 0'
Assert-Equal ([string]$Lock.build.compileFlags[1]) '/experimental:deterministic' 'Compile flag 1'
Assert-Equal ([string]$Lock.build.compileFlags[2]) '/pathmap:<absolute-source>=C:\ar4hmi-nlopt-src' 'Compile flag 2'
Assert-Condition ($Lock.build.linkFlags -is [Collections.IList] -and $Lock.build.linkFlags.Count -eq 1) 'Link flags type differs'
Assert-Equal ([string]$Lock.build.linkFlags[0]) '/Brepro' 'Link flag'
foreach ($name in @('BUILD_SHARED_LIBS','NLOPT_LUKSAN','NLOPT_PYTHON_SABI')) { Assert-Condition ($Lock.build.cache.$name -is [string]) "lock.build.cache.$name type differs"; Assert-Equal $Lock.build.cache.$name 'OFF' "lock.build.cache.$name" }
Assert-Identity $Lock.outputs.wheel $true 'lock.outputs.wheel'; Assert-Identity $Lock.outputs.extension $true 'lock.outputs.extension'; Assert-Identity $Lock.outputs.normalizedCache $false 'lock.outputs.normalizedCache'
Assert-Equal ($Lock.probe.reject | ConvertTo-Json -Compress) '["LD_LBFGS","LD_VAR1","LD_VAR2","LD_TNEWTON","LD_TNEWTON_RESTART","LD_TNEWTON_PRECOND","LD_TNEWTON_PRECOND_RESTART"]' 'Rejected algorithms'
Assert-Condition ($Lock.probe.exception -is [string] -and $Lock.probe.lastResult -is [int]) 'Rejected probe types differ'
Assert-Equal $Lock.probe.exception 'nlopt.nlopt.invalid_argument' 'Rejected exception'; Assert-Equal $Lock.probe.lastResult -2 'Rejected result'
Assert-Equal ($Lock.probe.control | ConvertTo-Json -Compress) '["LD_MMA",5]' 'Positive control'
Assert-Relative $Lock.notices.directory 'lock.notices.directory'; Assert-Identity $Lock.notices.aggregate $false 'lock.notices.aggregate'; Assert-Identity $Lock.notices.buildMode $true 'lock.notices.buildMode'; Assert-Condition ($Lock.notices.files -is [Collections.IList] -and $Lock.notices.files.Count -eq 16) 'Source-notice lock is invalid'
foreach ($row in $Lock.notices.files) { Assert-Identity $row $true 'lock.notices.files entry'; Assert-Condition ($row[0] -cmatch '^[UP]/') "Unsupported notice source: $($row[0])" }
$MappedSource = ([string]$Lock.build.compileFlags[2]).Substring('/pathmap:<absolute-source>='.Length)
$InputRoot = Resolve-Absolute $InputRoot 'Input root'; Assert-Condition (Test-Path -LiteralPath $InputRoot -PathType Container) 'Input root is missing'
$inputItem = Get-Item -LiteralPath $InputRoot -Force; Assert-Condition (-not ($inputItem.Attributes -band [IO.FileAttributes]::ReparsePoint)) 'Input root must be a normal directory'
$InputRoot = $inputItem.FullName.TrimEnd('\')
$BuildRoot = Resolve-Prospective $BuildRoot 'Build root'; Assert-Condition (-not $BuildRoot.Equals([IO.Path]::GetPathRoot($BuildRoot), [StringComparison]::OrdinalIgnoreCase)) 'Build root cannot be a filesystem root'
Assert-Condition (-not (Test-Within $BuildRoot $Repo) -and -not (Test-Within $Repo $BuildRoot)) 'Build root must be external to the checkout'; Assert-Condition (-not (Test-Within $BuildRoot $InputRoot) -and -not (Test-Within $InputRoot $BuildRoot)) 'Build and input roots must be disjoint'
for ($cursor = $BuildRoot; $cursor; $cursor = Split-Path -Parent $cursor) {
  if (Test-Path -LiteralPath $cursor) { $item = Get-Item -LiteralPath $cursor -Force; Assert-Condition (-not ($item.Attributes -band [IO.FileAttributes]::ReparsePoint)) "Build root ancestor is a reparse point: $cursor" }
  if ($cursor.Equals([IO.Path]::GetPathRoot($cursor), [StringComparison]::OrdinalIgnoreCase)) { break }
}
if (Test-Path -LiteralPath $BuildRoot) { $item = Get-Item -LiteralPath $BuildRoot -Force; Assert-Condition ($item.PSIsContainer -and @(Get-ChildItem -LiteralPath $BuildRoot -Force).Count -eq 0) 'Build root must be an empty directory' } else { New-Item -ItemType Directory -Path $BuildRoot | Out-Null }
$item = Get-Item -LiteralPath $BuildRoot -Force
Assert-Condition ($item.PSIsContainer -and -not ($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -and $item.FullName.Equals($BuildRoot, [StringComparison]::OrdinalIgnoreCase)) 'Created build root identity differs'
function Assert-Inputs {
  $rows = [Collections.Generic.List[object]]::new(); foreach ($name in @('upstream','packaging','pythonInstaller','swig','numpy','pip','setuptools')) { $tuple = $Lock.inputs.$name; $path = Join-Path $InputRoot $tuple[0]; Assert-File $path $tuple[1] $tuple[2] "Input $($tuple[0])"; $rows.Add([pscustomobject]@{ path = $tuple[0]; size = $tuple[1]; sha256 = $tuple[2] }) }
  $entries = @(Get-ChildItem -LiteralPath $InputRoot -Force -Recurse); Assert-Condition (-not ($entries | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint } | Select-Object -First 1)) 'Frozen input tree contains a reparse point'
  $actual = @($entries | Where-Object { -not $_.PSIsContainer } | ForEach-Object { $_.FullName.Substring($InputRoot.Length).TrimStart('\').Replace('\', '/') }); Assert-SameStrings $actual @($rows.path) 'Frozen input inventory'; Assert-Aggregate (Get-IdentityStream $rows) $Lock.inputs.aggregate 'Frozen input stream'
}
Assert-Inputs; $LockHash = Get-Hash $LockPath; $DriverHash = Get-Hash $PSCommandPath
$BuildVenv = Join-Path $BuildRoot 'build-venv'; $BootstrapTemp = Join-Path $BuildRoot 'bootstrap-temp'; $ValidatorPath = Join-Path $BuildRoot 'validate_artifact.py'
$PackagingRootName = [IO.Path]::GetFileNameWithoutExtension([string]$Lock.inputs.packaging[0]); $UpstreamRootName = [IO.Path]::GetFileNameWithoutExtension([string]$Lock.inputs.upstream[0])
Assert-Condition ($PackagingRootName -cmatch '^nlopt-python-([0-9a-f]{40})$') 'Packaging archive name is invalid'; $PackagingCommit = $Matches[1]; Assert-Condition ($UpstreamRootName -cmatch '^nlopt-([0-9a-f]{40})$') 'Upstream archive name is invalid'; $UpstreamCommit = $Matches[1]
$Validator = @"
import base64,csv,hashlib,io,json,math,pathlib,struct,sys,zipfile
wheel,target,extension_name=pathlib.Path(sys.argv[1]),pathlib.Path(sys.argv[2]),sys.argv[3]
rejected=sys.argv[4].split(','); control_name=sys.argv[5]
with zipfile.ZipFile(wheel) as archive:
    infos=archive.infolist(); names=[entry.filename for entry in infos]; paths=[pathlib.PurePosixPath(name) for name in names]
    if len(infos)!=8 or len(set(names))!=8 or any(path.is_absolute() or '..' in path.parts or path.as_posix()!=name or name.endswith('/') or '\\' in name or ':' in name for path,name in zip(paths,names)): raise RuntimeError('wheel member inventory differs')
    if any(entry.date_time!=(1980,1,1,0,0,0) or entry.compress_type!=zipfile.ZIP_DEFLATED for entry in infos): raise RuntimeError('wheel member metadata differs')
    records=[name for name in names if name.endswith('.dist-info/RECORD')]
    if len(records)!=1: raise RuntimeError('wheel RECORD count differs')
    rows=list(csv.reader(io.TextIOWrapper(archive.open(records[0]),encoding='utf-8',newline='')))
    if len(rows)!=8 or any(len(row)!=3 for row in rows) or sorted(row[0] for row in rows)!=sorted(names): raise RuntimeError('wheel RECORD inventory differs')
    for name,digest,size in rows:
        if name==records[0]:
            if digest or size: raise RuntimeError('wheel RECORD self-row differs')
        else:
            payload=archive.read(name); observed='sha256='+base64.urlsafe_b64encode(hashlib.sha256(payload).digest()).rstrip(b'=').decode('ascii')
            if digest!=observed or size!=str(len(payload)): raise RuntimeError('wheel RECORD identity differs: '+name)
    archive.extractall(target)
extension=target.joinpath(*pathlib.PurePosixPath(extension_name).parts).resolve(); resolved_target=target.resolve()
if resolved_target not in extension.parents or not extension.is_file(): raise RuntimeError('wheel extension path differs')
def pe_debug_types(path):
    data=path.read_bytes(); pe=struct.unpack_from('<I',data,0x3c)[0]
    if data[:2]!=b'MZ' or data[pe:pe+4]!=b'PE\0\0': raise RuntimeError('extension PE signature differs')
    sections=struct.unpack_from('<H',data,pe+6)[0]; optional_size=struct.unpack_from('<H',data,pe+20)[0]; optional=pe+24
    directory=optional+(112 if struct.unpack_from('<H',data,optional)[0]==0x20b else 96); rva,size=struct.unpack_from('<II',data,directory+48); table=optional+optional_size; offset=None
    if not size or size%28: raise RuntimeError('extension debug directory differs')
    for index in range(sections):
        entry=table+index*40; virtual_size,address,raw_size,raw=struct.unpack_from('<IIII',data,entry+8)
        if address<=rva<address+max(virtual_size,raw_size): offset=raw+rva-address; break
    if offset is None or offset+size>len(data): raise RuntimeError('extension debug directory is unmapped')
    return sorted(struct.unpack_from('<I',data,offset+index+12)[0] for index in range(0,size,28))
sys.path.insert(0,str(resolved_target)); import nlopt
if pathlib.Path(nlopt._nlopt.__file__).resolve()!=extension: raise RuntimeError('isolated import resolved another extension')
def objective(x,gradient):
    if gradient.size: gradient[0]=2.0*x[0]
    return x[0]*x[0]
results={}
for name in rejected:
    optimizer=nlopt.opt(getattr(nlopt,name),1); optimizer.set_min_objective(objective); optimizer.set_maxeval(10)
    try: results[name]={'unexpectedSuccess':optimizer.optimize([1.0]).tolist(),'lastResult':optimizer.last_optimize_result()}
    except Exception as error: results[name]={'exception':type(error).__module__+'.'+type(error).__qualname__,'message':str(error),'lastResult':optimizer.last_optimize_result()}
control=nlopt.opt(getattr(nlopt,control_name),1); control.set_min_objective(objective); control.set_maxeval(100); control.set_xtol_rel(1e-12); x=float(control.optimize([1.0])[0])
if not math.isfinite(x) or abs(x)>1e-8: raise RuntimeError('positive control result is not finite and near zero')
print(json.dumps({'peDebugTypes':pe_debug_types(extension),'rejected':results,'control':{'algorithm':control_name,'x':x,'lastResult':control.last_optimize_result()}},sort_keys=True,separators=(',',':')))
"@
[IO.File]::WriteAllText($ValidatorPath, $Validator.Replace("`r`n", "`n"), $Utf8)
function Assert-ToolSet([string]$SwigPath, [string]$VenvPython) {
  $paths = @{ 'cl.exe' = Join-Path $ToolBin 'cl.exe'; 'cmake.exe' = $Cmake; 'cvtres.exe' = Join-Path $ToolBin 'cvtres.exe'; 'lib.exe' = Join-Path $ToolBin 'lib.exe'; 'link.exe' = Join-Path $ToolBin 'link.exe'; 'msbuild.exe' = $MsBuild; 'python.exe' = $Python; 'python314.dll' = Join-Path $PythonHome 'python314.dll'; 'rc.exe' = $ResourceCompiler; 'swig.exe' = $SwigPath; 'venv-python.exe' = $VenvPython }
  $rows = [Collections.Generic.List[object]]::new(); foreach ($role in $ToolRoles) { $tuple = $Lock.tools.PSObject.Properties[$role].Value; Assert-File $paths[$role] $tuple[0] $tuple[1] "Tool $role"; $rows.Add([pscustomobject]@{ path = $role; size = $tuple[0]; sha256 = $tuple[1] }) }; Assert-Aggregate (Get-IdentityStream $rows) $Lock.tools.aggregate 'Portable tool stream'
}
$BootstrapVersions = @{}
function Get-BootstrapRequirement([string]$Name) { $tuple = $Lock.inputs.$Name; $leaf = [IO.Path]::GetFileName([string]$tuple[0]); Assert-Condition ($leaf -cmatch "^$([regex]::Escape($Name))-([^-]+)-") "Bootstrap wheel name differs: $Name"; $BootstrapVersions[$Name] = $Matches[1]; return "$Name==$($Matches[1]) --hash=sha256:$($tuple[2])" }
function Write-Provenance([string]$Source, [string]$Upstream, [string]$PassRoot) {
  $noticeRoot = Join-Path $PassRoot $Lock.notices.directory; New-Item -ItemType Directory -Path $noticeRoot -Force | Out-Null; $rows = [Collections.Generic.List[object]]::new()
  foreach ($tuple in $Lock.notices.files) {
    $parts = @(([string]$tuple[0]).Split('/')); $sourceRoot = if ($parts[0] -ceq 'U') { $Upstream } else { $Source }; $expandedRoot = if ($parts[0] -ceq 'U') { $UpstreamRootName } else { $PackagingRootName }
    $tail = $parts[1..($parts.Count - 1)] -join '/'; $relative = "$expandedRoot/$tail"; $from = Join-Path $sourceRoot ($tail.Replace('/', '\'))
    Assert-File $from $tuple[1] $tuple[2] "Notice $($tuple[0])"; $to = Join-Path $noticeRoot ($relative.Replace('/', '\')); New-Item -ItemType Directory -Path (Split-Path -Parent $to) -Force | Out-Null
    Copy-Item -LiteralPath $from -Destination $to; Assert-File $to $tuple[1] $tuple[2] "Copied notice $relative"; $copied = Get-Item -LiteralPath $to; $rows.Add([pscustomobject]@{ path = $relative; size = $copied.Length; sha256 = Get-Hash $to })
  }
  $actual = @(Get-ChildItem -LiteralPath $noticeRoot -File -Recurse | ForEach-Object { $_.FullName.Substring($noticeRoot.Length).TrimStart('\').Replace('\', '/') }); Assert-SameStrings $actual @($rows.path) 'Copied notice inventory'; $noticeStream = Get-IdentityStream $rows; Assert-Aggregate $noticeStream $Lock.notices.aggregate 'Source-notice stream'
  $mode = @('AR4HMI NLopt build mode', "NLopt Python packaging commit: $PackagingCommit", "NLopt upstream commit: $UpstreamCommit", "Build configuration: BUILD_SHARED_LIBS=$($Lock.build.cache.BUILD_SHARED_LIBS); NLOPT_LUKSAN=$($Lock.build.cache.NLOPT_LUKSAN); NLOPT_PYTHON_SABI=$($Lock.build.cache.NLOPT_PYTHON_SABI)", "Wheel: $($Lock.outputs.wheel[0])", "Wheel SHA-256: $($Lock.outputs.wheel[2])", "Algorithms rejected at optimization: $($Lock.probe.reject -join ', ')", "The $($Lock.notices.files.Count) preserved source notices accompany this file.", 'Redistribution approved: false') -join "`n"
  $modePath = Join-Path $PassRoot $Lock.notices.buildMode[0]; New-Item -ItemType Directory -Path (Split-Path -Parent $modePath) -Force | Out-Null; [IO.File]::WriteAllText($modePath, $mode + "`n", $Utf8); Assert-File $modePath $Lock.notices.buildMode[1] $Lock.notices.buildMode[2] 'Build-mode notice'
  return [pscustomobject]@{ notices = Get-TextHash $noticeStream; buildMode = Get-Hash $modePath }
}
function Assert-Probe([string]$Path) {
  $lines = @(Get-Content -LiteralPath $Path); $json = @($lines | Where-Object { $_ -cmatch '^\{.*\}$' }); Assert-Condition ($json.Count -eq 1) 'Runtime probe JSON record count differs'
  $diagnostics = @($lines | Where-Object { $_ -cne $json[0] }); $expectedDiagnostics = @('ERROR - attempting to use NLOPT_LD_LBFGS, but Luksan code disabled') + @('ERROR - attempting to use NLOPT_LD_VAR*, but Luksan code disabled') * 2 + @('ERROR - attempting to use NLOPT_LD_TNEWTON*, but Luksan code disabled') * 4
  Assert-SameStrings $diagnostics $expectedDiagnostics 'Runtime Luksan diagnostics'; $probe = $json[0] | ConvertFrom-Json; Assert-Shape $probe @('peDebugTypes','rejected','control') 'runtime probe'
  Assert-Equal ($probe.peDebugTypes | ConvertTo-Json -Compress) '[13,16]' 'PE debug types'; Assert-SameStrings @($probe.rejected.PSObject.Properties.Name) @($Lock.probe.reject) 'Rejected runtime algorithms'
  foreach ($name in $Lock.probe.reject) {
    $result = $probe.rejected.PSObject.Properties[$name].Value; Assert-Shape $result @('exception','message','lastResult') "runtime probe $name"
    Assert-Condition ($result.exception -is [string] -and $result.message -is [string] -and $result.lastResult -is [int]) "$name probe scalar types differ"
    Assert-Equal $result.exception $Lock.probe.exception "$name exception"; Assert-Equal $result.lastResult $Lock.probe.lastResult "$name last result"
  }
  Assert-Shape $probe.control @('algorithm','x','lastResult') 'runtime control'; Assert-Condition ($probe.control.algorithm -is [string] -and $probe.control.lastResult -is [int] -and ($probe.control.x -is [int] -or $probe.control.x -is [long] -or $probe.control.x -is [decimal] -or $probe.control.x -is [double])) 'Control scalar types differ'; Assert-Equal $probe.control.algorithm $Lock.probe.control[0] 'Control algorithm'; Assert-Equal $probe.control.lastResult $Lock.probe.control[1] 'Control last result'; $x = [double]$probe.control.x; Assert-Condition (-not [double]::IsNaN($x) -and -not [double]::IsInfinity($x) -and [Math]::Abs($x) -le 1e-8) 'Control value is not finite and near zero'
}
function Invoke-Pass([int]$Pass, [string]$VenvPython) {
  $env:TEMP = $BootstrapTemp; $env:TMP = $BootstrapTemp; $env:TMPDIR = $BootstrapTemp; $work = Reset-Work; $passRoot = Join-Path $BuildRoot "pass-$Pass"; $logs = Join-Path $passRoot 'logs'; New-Item -ItemType Directory -Path $logs -Force | Out-Null
  $source = Join-Path $work 'src'; Expand-Source $Lock.inputs.packaging $source $PackagingRootName $work; $upstreamStage = Join-Path $work 'upstream'; Expand-Source $Lock.inputs.upstream $upstreamStage $UpstreamRootName $work
  $upstream = Join-Path $source 'extern\nlopt'; if (Test-Path -LiteralPath $upstream) { Assert-Condition (@(Get-ChildItem -LiteralPath $upstream -Force).Count -eq 0) 'Packaging archive contains extern/nlopt content'; foreach ($child in Get-ChildItem -LiteralPath $upstreamStage -Force) { Move-Item -LiteralPath $child.FullName -Destination $upstream } } else { Move-Item -LiteralPath $upstreamStage -Destination $upstream }
  $transform = Join-Path $source $Lock.transform.path; Assert-Condition (Test-Path -LiteralPath $transform -PathType Leaf) 'Transform source is missing'; Assert-Equal (Get-Hash $transform) ([string]$Lock.transform.beforeSha256) 'Transform source SHA-256'; $text = [IO.File]::ReadAllText($transform, $Utf8); Assert-Equal ([regex]::Matches($text, [regex]::Escape([string]$Lock.transform.needle)).Count) 1 'Transform anchor count'; [IO.File]::WriteAllText($transform, $text.Replace([string]$Lock.transform.needle, [string]$Lock.transform.replacement), $Utf8); Assert-Equal (Get-Hash $transform) ([string]$Lock.transform.afterSha256) 'Transform output SHA-256'
  $swigExtract = Join-Path $work 'swig-extract'; Expand-LockedArchive $Lock.inputs.swig $swigExtract; $swigChildren = @(Get-ChildItem -LiteralPath $swigExtract -Force); Assert-Condition ($swigChildren.Count -eq 1 -and $swigChildren[0].PSIsContainer) 'SWIG archive root differs'; $swig = Join-Path $swigChildren[0].FullName 'swig.exe'; Assert-ToolSet $swig $VenvPython
  $env:Path = "$(Join-Path $BuildVenv 'Scripts');$($swigChildren[0].FullName);$(Split-Path -Parent $Cmake);$SystemRoot\System32;$SystemRoot"; $env:TEMP = Join-Path $work 'temp'; $env:TMP = $env:TEMP; $env:TMPDIR = $env:TEMP; New-Item -ItemType Directory -Path $env:TEMP | Out-Null
  $env:SOURCE_DATE_EPOCH = [string]$Lock.build.sourceDateEpoch; $env:NLOPT_VERSION_SUFFIX = [string]$Lock.build.versionSuffix; $env:CMAKE_GENERATOR = 'Visual Studio 18 2026'; $env:CMAKE_GENERATOR_INSTANCE = $VsRoot; $env:CMAKE_GENERATOR_PLATFORM = 'x64,version=10.0.26100.0'; $env:CMAKE_GENERATOR_TOOLSET = 'v145,host=x64,version=14.50.35717'; $flags = @($Lock.build.compileFlags | ForEach-Object { ([string]$_).Replace('<absolute-source>', $source) }) -join ' '; $env:CFLAGS = $flags; $env:CXXFLAGS = $flags; $env:LINK = @($Lock.build.linkFlags) -join ' '
  $dist = Join-Path $work 'dist'; Push-Location $source; try { Invoke-Logged $VenvPython @('-s','setup.py','bdist_wheel','-d',$dist) (Join-Path $logs 'build.stdout') (Join-Path $logs 'build.stderr') "NLopt pass $Pass" } finally { Pop-Location }
  $wheels = @(Get-ChildItem -LiteralPath $dist -File -Filter '*.whl'); Assert-Condition ($wheels.Count -eq 1 -and $wheels[0].Name -ceq $Lock.outputs.wheel[0]) "Pass $Pass wheel inventory differs"; Assert-File $wheels[0].FullName $Lock.outputs.wheel[1] $Lock.outputs.wheel[2] "Pass $Pass wheel"
  $caches = @(Get-ChildItem -LiteralPath (Join-Path $source 'build') -File -Recurse -Filter 'CMakeCache.txt'); Assert-Condition ($caches.Count -eq 1) "Pass $Pass CMake cache count differs"; $cacheText = [IO.File]::ReadAllText($caches[0].FullName); foreach ($name in @('BUILD_SHARED_LIBS','NLOPT_LUKSAN','NLOPT_PYTHON_SABI')) { Assert-Condition ($cacheText -match "(?m)^$([regex]::Escape($name)):BOOL=$([regex]::Escape([string]$Lock.build.cache.$name))`r?$") "Pass $Pass cache lacks $name" }
  $evidence = @(Get-ChildItem -LiteralPath (Join-Path $source 'build') -File -Recurse | Where-Object { $_.Extension -in @('.vcxproj','.tlog') }); Assert-Condition ($evidence.Count -gt 0) "Pass $Pass compile evidence is missing"; Assert-Condition (@($evidence | Select-String -Pattern 'src[\\/]+algs[\\/]+luksan').Count -eq 0) "Pass $Pass contains a Luksan compile entry"
  $wheelCopy = Join-Path $passRoot $wheels[0].Name; Copy-Item -LiteralPath $wheels[0].FullName -Destination $wheelCopy; Assert-File $wheelCopy $Lock.outputs.wheel[1] $Lock.outputs.wheel[2] "Pass $Pass copied wheel"; $normalized = Normalize-Cache $caches[0].FullName; Assert-Condition ($normalized -match [regex]::Escape("/pathmap:<ROOT>/work/src=$($MappedSource.Replace('\','/'))")) "Pass $Pass normalized cache lacks the mapped source flag"; $normalizedPath = Join-Path $passRoot 'CMakeCache.normalized.txt'; [IO.File]::WriteAllText($normalizedPath, $normalized, $Utf8); Assert-File $normalizedPath $Lock.outputs.normalizedCache[0] $Lock.outputs.normalizedCache[1] "Pass $Pass normalized cache"
  # NLopt's disabled-Luksan branches use printf, so diagnostics intentionally share stdout with the JSON record while stderr remains a failure log.
  $runtime = Join-Path $work 'runtime'; New-Item -ItemType Directory -Path $runtime | Out-Null; $probePath = Join-Path $passRoot 'runtime-probe.json'; $rejectNames = $Lock.probe.reject -join ','; Invoke-Logged $VenvPython @('-I',$ValidatorPath,$wheelCopy,$runtime,$Lock.outputs.extension[0],$rejectNames,$Lock.probe.control[0]) $probePath (Join-Path $logs 'runtime.stderr') "Pass $Pass artifact and runtime validation"
  $extension = Join-Path $runtime ($Lock.outputs.extension[0].Replace('/','\')); Assert-File $extension $Lock.outputs.extension[1] $Lock.outputs.extension[2] "Pass $Pass extension"; $binary = [Text.Encoding]::ASCII.GetString([IO.File]::ReadAllBytes($extension)); Assert-Condition ($binary.IndexOf($BuildRoot, [StringComparison]::OrdinalIgnoreCase) -lt 0 -and $binary.IndexOf($BuildRoot.Replace('\','/'), [StringComparison]::OrdinalIgnoreCase) -lt 0 -and $binary.IndexOf($MappedSource, [StringComparison]::OrdinalIgnoreCase) -ge 0) "Pass $Pass extension path mapping differs"; Assert-Probe $probePath; $provenance = Write-Provenance $source $upstream $passRoot
  return [pscustomobject]@{ pass = $Pass; wheel = Get-Hash $wheelCopy; extension = Get-Hash $extension; normalizedCache = Get-Hash $normalizedPath; notices = $provenance.notices; buildMode = $provenance.buildMode }
}
Assert-Condition (Test-Path -LiteralPath $SystemRoot -PathType Container) 'Windows system root is unavailable'; $pythonTuple = $Lock.tools.PSObject.Properties['python.exe'].Value; $dllTuple = $Lock.tools.PSObject.Properties['python314.dll'].Value; Assert-File $Python $pythonTuple[0] $pythonTuple[1] 'Tool python.exe'; Assert-File (Join-Path $PythonHome 'python314.dll') $dllTuple[0] $dllTuple[1] 'Tool python314.dll'
$dynamicNames = @([Environment]::GetEnvironmentVariables('Process').Keys | Where-Object { $_ -match '^(?i:CMAKE_|PIP_|PYTHON|_PYTHON|DISTUTILS|SOURCE_DATE_EPOCH|NLOPT_VERSION_SUFFIX|CL$|_CL_$|LINK$|INCLUDE$|LIB$|LIBPATH$|TEMP$|TMP$|TMPDIR$)' }); $staticNames = @('Path','PIP_CONFIG_FILE','PIP_NO_INDEX','PIP_DISABLE_PIP_VERSION_CHECK','PYTHONNOUSERSITE','PYTHONDONTWRITEBYTECODE','PYTHONHASHSEED','TEMP','TMP','TMPDIR','SOURCE_DATE_EPOCH','NLOPT_VERSION_SUFFIX','CMAKE_GENERATOR','CMAKE_GENERATOR_INSTANCE','CMAKE_GENERATOR_PLATFORM','CMAKE_GENERATOR_TOOLSET','CFLAGS','CXXFLAGS','LINK'); $EnvironmentNames = @($dynamicNames + $staticNames) | Sort-Object -Unique; $SavedEnvironment = @{}; foreach ($name in $EnvironmentNames) { $SavedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
try {
  foreach ($name in $EnvironmentNames) { [Environment]::SetEnvironmentVariable($name, $null, 'Process') }; $env:Path = "$PythonHome;$SystemRoot\System32;$SystemRoot"; $env:PIP_CONFIG_FILE = 'NUL'; $env:PIP_NO_INDEX = '1'; $env:PIP_DISABLE_PIP_VERSION_CHECK = '1'; $env:PYTHONNOUSERSITE = '1'; $env:PYTHONDONTWRITEBYTECODE = '1'; $env:PYTHONHASHSEED = '1'; New-Item -ItemType Directory -Path $BootstrapTemp | Out-Null; $env:TEMP = $BootstrapTemp; $env:TMP = $BootstrapTemp; $env:TMPDIR = $BootstrapTemp
  Invoke-Logged $Python @('-I','-m','venv',$BuildVenv) (Join-Path $BuildRoot 'venv.stdout') (Join-Path $BuildRoot 'venv.stderr') 'Build environment creation'; $VenvPython = Join-Path $BuildVenv 'Scripts\python.exe'; $venvTuple = $Lock.tools.PSObject.Properties['venv-python.exe'].Value; Assert-File $VenvPython $venvTuple[0] $venvTuple[1] 'Tool venv-python.exe'
  $bootstrap = Join-Path $BuildVenv 'bootstrap.lock'; $requirements = @((Get-BootstrapRequirement 'numpy'),(Get-BootstrapRequirement 'pip'),(Get-BootstrapRequirement 'setuptools')) -join "`n"; [IO.File]::WriteAllText($bootstrap, $requirements + "`n", $Utf8); Invoke-Logged $VenvPython @('-s','-m','pip','--isolated','install','--no-index','--find-links',(Join-Path $InputRoot 'wheels'),'--no-cache-dir','--disable-pip-version-check','--require-hashes','--only-binary=:all:','--no-deps','--no-compile','-r',$bootstrap) (Join-Path $BuildRoot 'pip.stdout') (Join-Path $BuildRoot 'pip.stderr') 'Offline build dependency installation'
  $inventoryPath = Join-Path $BuildRoot 'build-environment.json'; Invoke-Logged $VenvPython @('-I','-c','import importlib.metadata as m,json;r=sorted((d.metadata[''Name''].lower(),d.version) for d in m.distributions());print(json.dumps({''count'':len(r),''packages'':dict(r)},sort_keys=True))') $inventoryPath (Join-Path $BuildRoot 'inventory.stderr') 'Build environment inventory'; $inventory = Get-Content -LiteralPath $inventoryPath -Raw | ConvertFrom-Json; Assert-Shape $inventory @('count','packages') 'Build environment inventory'; Assert-Equal $inventory.count 3 'Build environment package count'; Assert-Shape $inventory.packages @('numpy','pip','setuptools') 'Build environment packages'; foreach ($name in @('numpy','pip','setuptools')) { Assert-Equal ([string]$inventory.packages.$name) ([string]$BootstrapVersions[$name]) "Build environment $name version" }
  $results = @(); foreach ($pass in 1..2) { $results += Invoke-Pass $pass $VenvPython }; Assert-Condition ($results.Count -eq 2) 'Two NLopt passes are required'; foreach ($field in @('wheel','extension','normalizedCache','notices','buildMode')) { Assert-Condition (@($results.$field | Sort-Object -Unique).Count -eq 1) "Cross-pass $field differs" }
  Assert-Inputs; $swigRoots = @(Get-ChildItem -LiteralPath (Join-Path $BuildRoot 'work\swig-extract') -Directory); Assert-Condition ($swigRoots.Count -eq 1) 'Final SWIG root differs'; Assert-ToolSet (Join-Path $swigRoots[0].FullName 'swig.exe') $VenvPython; Assert-Equal (Get-Hash $LockPath) $LockHash 'NLopt build lock'; Assert-Equal (Get-Hash $PSCommandPath) $DriverHash 'NLopt build driver'
  [IO.File]::WriteAllText((Join-Path $BuildRoot 'result.json'), ([ordered]@{ schema = 1; lockSha256 = $LockHash; driverSha256 = $DriverHash; results = $results } | ConvertTo-Json -Depth 5), $Utf8); Write-Output "Luksan-free NLopt evidence complete: $BuildRoot"
} finally { foreach ($name in $EnvironmentNames) { [Environment]::SetEnvironmentVariable($name, $SavedEnvironment[$name], 'Process') } }
