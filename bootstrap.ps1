# bootstrap.ps1 - install the AR4HMI cross-review dispatcher in one clone.
#
# The gate scripts are already tracked in this repository. This installer
# validates the clone and copies the HEAD-trusted dispatcher template to
# .git/hooks/pre-commit without modifying tracked files or Git configuration.
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File bootstrap.ps1
#   powershell -NoProfile -ExecutionPolicy Bypass -File bootstrap.ps1 -TargetRepo <clone-or-subdirectory>
#   powershell -NoProfile -ExecutionPolicy Bypass -File bootstrap.ps1 -SelfTest
#
# A different existing dispatcher is preserved unless -Force is supplied.
# Under -Force, atomic replacement creates a uniquely named backup beside the
# hook. -Force never bypasses an untrusted template, the required logs ignore,
# a set core.hooksPath, a reparse-point destination, or indeterminate Git state.

[CmdletBinding(DefaultParameterSetName = 'Install')]
param(
  [Parameter(ParameterSetName = 'Install')]
  [string]$TargetRepo = $PSScriptRoot,

  [Parameter(ParameterSetName = 'Install')]
  [switch]$Force,

  [Parameter(Mandatory = $true, ParameterSetName = 'SelfTest')]
  [switch]$SelfTest
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:GitConfigEnvironmentNames = @(
  'GIT_CONFIG',
  'GIT_CONFIG_GLOBAL',
  'GIT_CONFIG_SYSTEM',
  'GIT_CONFIG_NOSYSTEM',
  'GIT_CONFIG_PARAMETERS',
  'GIT_CONFIG_COUNT'
)

# This classifier is intentionally identical to the commit-wrapper copies.
# A configured hook path can redirect the per-clone dispatcher, including when
# the configured value is empty, so only the absent-key exit status is safe.
function Resolve-HooksPathGuard {
  param([int]$ExitCode)
  if ($ExitCode -eq 1) { return 'proceed' }
  if ($ExitCode -eq 0) { return 'abort-set' }
  return 'abort-indeterminate'
}

function Invoke-WithGitConfigEnvironmentScrub {
  param([Parameter(Mandatory = $true)][scriptblock]$Action)

  $saved = @{}
  $visited = [System.Collections.Generic.List[string]]::new()
  try {
    foreach ($name in $script:GitConfigEnvironmentNames) {
      $value = [Environment]::GetEnvironmentVariable(
        $name,
        [EnvironmentVariableTarget]::Process
      )
      $saved[$name] = @{
        Present = ($null -ne $value)
        Value = $value
      }
      $visited.Add($name)
      if ($null -ne $value) {
        [Environment]::SetEnvironmentVariable(
          $name,
          $null,
          [EnvironmentVariableTarget]::Process
        )
        if (
          $null -ne
          [Environment]::GetEnvironmentVariable(
            $name,
            [EnvironmentVariableTarget]::Process
          )
        ) {
          throw "bootstrap.ps1: failed to scrub Git configuration environment variable $name"
        }
      }
    }
    & $Action
  } finally {
    $restoreFailures = [System.Collections.Generic.List[string]]::new()
    foreach ($name in $visited) {
      try {
        $entry = $saved[$name]
        if ($entry.Present) {
          [Environment]::SetEnvironmentVariable(
            $name,
            $entry.Value,
            [EnvironmentVariableTarget]::Process
          )
          if (
            [Environment]::GetEnvironmentVariable(
              $name,
              [EnvironmentVariableTarget]::Process
            ) -cne $entry.Value
          ) {
            throw 'restored value did not match'
          }
        } else {
          [Environment]::SetEnvironmentVariable(
            $name,
            $null,
            [EnvironmentVariableTarget]::Process
          )
          if (
            $null -ne
            [Environment]::GetEnvironmentVariable(
              $name,
              [EnvironmentVariableTarget]::Process
            )
          ) {
            throw 'variable remained present'
          }
        }
      } catch {
        $restoreFailures.Add("$name ($($_.Exception.Message))")
      }
    }
    if ($restoreFailures.Count -gt 0) {
      throw (
        'bootstrap.ps1: failed to restore Git configuration environment: ' +
        [string]::Join(', ', $restoreFailures)
      )
    }
  }
}

function Invoke-GitResult {
  param(
    [Parameter(Mandatory = $true)][string]$Repository,
    [Parameter(Mandatory = $true)][string[]]$Arguments
  )

  $stderrPath = Join-Path (
    [System.IO.Path]::GetTempPath()
  ) ('ar4hmi-bootstrap-git-' + [guid]::NewGuid().ToString('N') + '.stderr')
  $savedErrorActionPreference = $ErrorActionPreference
  $lines = @()
  $stderrLines = @()
  $exitCode = $null
  $invocationFailure = $null
  $cleanupFailure = $null
  try {
    try {
      # Keep stdout separate because callers parse exact Git values. Native
      # stderr remains diagnostic even when Git exits successfully.
      $ErrorActionPreference = 'Continue'
      $lines = @(& git -C $Repository @Arguments 2> $stderrPath)
      $exitCode = $LASTEXITCODE
      if (Test-Path -LiteralPath $stderrPath -PathType Leaf) {
        $stderrLines = @(
          Get-Content -LiteralPath $stderrPath -ErrorAction Stop |
            ForEach-Object { "$_" }
        )
      }
    } catch {
      $invocationFailure = $_.Exception
    }
  } finally {
    $ErrorActionPreference = $savedErrorActionPreference
    try {
      if (Test-Path -LiteralPath $stderrPath) {
        Remove-Item -LiteralPath $stderrPath -Force -ErrorAction Stop
      }
    } catch {
      $cleanupFailure = $_.Exception
    }
  }
  if ($null -ne $cleanupFailure) {
    throw "bootstrap.ps1: failed to remove Git stderr capture: $($cleanupFailure.Message)"
  }
  if (Test-Path -LiteralPath $stderrPath) {
    throw "bootstrap.ps1: Git stderr capture remains after cleanup: $stderrPath"
  }
  if ($null -ne $invocationFailure) {
    throw "bootstrap.ps1: Git invocation failed: $($invocationFailure.Message)"
  }
  $stdoutText = (
    @($lines | ForEach-Object { "$_" }) -join [Environment]::NewLine
  ).Trim()
  $stderrText = (
    @($stderrLines | ForEach-Object { "$_" }) -join [Environment]::NewLine
  ).Trim()
  $diagnosticParts = @(
    $stdoutText,
    $stderrText
  ) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
  return [pscustomobject]@{
    ExitCode = $exitCode
    Lines = @($lines | ForEach-Object { "$_" })
    Text = $stdoutText
    ErrorText = $stderrText
    DiagnosticText = ($diagnosticParts -join [Environment]::NewLine)
  }
}

function Get-GitSingleLine {
  param(
    [Parameter(Mandatory = $true)][string]$Repository,
    [Parameter(Mandatory = $true)][string[]]$Arguments,
    [Parameter(Mandatory = $true)][string]$Operation
  )

  $result = Invoke-GitResult -Repository $Repository -Arguments $Arguments
  $values = @(
    $result.Lines |
      ForEach-Object { $_.Trim() } |
      Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
  )
  if ($result.ExitCode -ne 0 -or $values.Count -ne 1) {
    throw "bootstrap.ps1: $Operation failed or returned an ambiguous value (exit $($result.ExitCode)): $($result.DiagnosticText)"
  }
  return $values[0]
}

function Resolve-RepositoryRoot {
  param([Parameter(Mandatory = $true)][string]$Candidate)

  if ([string]::IsNullOrWhiteSpace($Candidate)) {
    throw 'bootstrap.ps1: TargetRepo must not be empty'
  }
  $resolved = Resolve-Path -LiteralPath $Candidate -ErrorAction Stop
  if (-not (Test-Path -LiteralPath $resolved.Path -PathType Container)) {
    throw "bootstrap.ps1: TargetRepo is not a directory: $Candidate"
  }
  $topLevel = Get-GitSingleLine `
    -Repository $resolved.Path `
    -Arguments @('rev-parse', '--show-toplevel') `
    -Operation 'repository-root resolution'
  $topLevelResolved = Resolve-Path -LiteralPath $topLevel -ErrorAction Stop
  if (-not (Test-Path -LiteralPath $topLevelResolved.Path -PathType Container)) {
    throw "bootstrap.ps1: resolved Git top-level is not a directory: $topLevel"
  }
  return $topLevelResolved.Path
}

function Assert-TrustedGatePath {
  param(
    [Parameter(Mandatory = $true)][string]$Repository,
    [Parameter(Mandatory = $true)][string]$RelativePath,
    [switch]$RequireCleanContent
  )

  $absolutePath = Join-Path $Repository $RelativePath
  if (-not (Test-Path -LiteralPath $absolutePath -PathType Leaf)) {
    throw "bootstrap.ps1: required gate file is missing or not a leaf file: $RelativePath"
  }
  $item = Get-Item -LiteralPath $absolutePath -Force
  if (($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
    throw "bootstrap.ps1: required gate file must not be a reparse point: $RelativePath"
  }

  $tracked = Invoke-GitResult `
    -Repository $Repository `
    -Arguments @('cat-file', '-e', "HEAD:$RelativePath")
  if ($tracked.ExitCode -ne 0) {
    throw "bootstrap.ps1: required gate file is not tracked in HEAD: $RelativePath ($($tracked.DiagnosticText))"
  }

  if ($RequireCleanContent) {
    $headOid = Get-GitSingleLine `
      -Repository $Repository `
      -Arguments @('rev-parse', "HEAD:$RelativePath") `
      -Operation "HEAD object lookup for $RelativePath"
    $workingOid = Get-GitSingleLine `
      -Repository $Repository `
      -Arguments @('hash-object', '--', $RelativePath) `
      -Operation "working-tree object lookup for $RelativePath"
    if ($headOid -cne $workingOid) {
      throw "bootstrap.ps1: required gate file differs from HEAD: $RelativePath; commit and review the file before installation"
    }
    return $headOid
  }
  return $null
}

function Get-DispatcherPath {
  param([Parameter(Mandatory = $true)][string]$Repository)

  $gitPath = Get-GitSingleLine `
    -Repository $Repository `
    -Arguments @('rev-parse', '--git-path', 'hooks/pre-commit') `
    -Operation 'dispatcher-path resolution'
  if ([System.IO.Path]::IsPathRooted($gitPath)) {
    return [System.IO.Path]::GetFullPath($gitPath)
  }
  return [System.IO.Path]::GetFullPath((Join-Path $Repository $gitPath))
}

function Test-FilesEqual {
  param(
    [Parameter(Mandatory = $true)][string]$First,
    [Parameter(Mandatory = $true)][string]$Second
  )

  if (
    -not (Test-Path -LiteralPath $First -PathType Leaf) -or
    -not (Test-Path -LiteralPath $Second -PathType Leaf)
  ) {
    return $false
  }
  $firstItem = Get-Item -LiteralPath $First -Force
  $secondItem = Get-Item -LiteralPath $Second -Force
  if ($firstItem.Length -ne $secondItem.Length) {
    return $false
  }
  $firstHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $First).Hash
  $secondHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $Second).Hash
  return $firstHash -ceq $secondHash
}

# Remove a directory tree WITHOUT `Remove-Item -Recurse`. Package cleanup policy
# bans the recursive force-delete (the dangerous footgun) even for the gate's own
# scratch dirs; this is the package's canonical no-Recurse cleanup.
# Enumerates files (-Force so HIDDEN/system entries -- e.g. a scratch repo's `.git`
# tree -- and read-only git loose objects are included), deletes them individually
# (per-file -Force), then deletes the now-empty dirs bottom-up (longest path first),
# then the root. Without -Force on the enumeration, a hidden `.git` is skipped and
# the non-empty root removal throws. Best-effort (SilentlyContinue): scratch cleanup
# must never abort the run. Safe on $null / nonexistent paths. (Byte-identical
# across the three wrapper copies.)
function Remove-TreeNoRecurse {
  param([string]$Path)
  if ([string]::IsNullOrWhiteSpace($Path) -or -not (Test-Path -LiteralPath $Path)) { return }
  Get-ChildItem -LiteralPath $Path -File -Recurse -Force -ErrorAction SilentlyContinue | ForEach-Object {
    Remove-Item -LiteralPath $_.FullName -Force -ErrorAction SilentlyContinue
  }
  Get-ChildItem -LiteralPath $Path -Directory -Recurse -Force -ErrorAction SilentlyContinue |
    Sort-Object { $_.FullName.Length } -Descending |
    ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force -ErrorAction SilentlyContinue }
  Remove-Item -LiteralPath $Path -Force -ErrorAction SilentlyContinue
}

function Export-TrustedHeadBlob {
  param(
    [Parameter(Mandatory = $true)][string]$Repository,
    [Parameter(Mandatory = $true)][string]$RelativePath,
    [Parameter(Mandatory = $true)][string]$ExpectedOid,
    [Parameter(Mandatory = $true)][string]$Destination
  )

  if ($RelativePath -notmatch '^[A-Za-z0-9._/-]+$') {
    throw "bootstrap.ps1: unsafe Git object path: $RelativePath"
  }
  $gitCommand = Get-Command git -ErrorAction Stop
  if (
    [string]::IsNullOrWhiteSpace($gitCommand.Source) -or
    -not (Test-Path -LiteralPath $gitCommand.Source -PathType Leaf)
  ) {
    throw 'bootstrap.ps1: Git executable could not be resolved'
  }

  $startInfo = [System.Diagnostics.ProcessStartInfo]::new()
  $startInfo.FileName = $gitCommand.Source
  $startInfo.WorkingDirectory = $Repository
  $startInfo.Arguments = "cat-file blob HEAD:$RelativePath"
  $startInfo.UseShellExecute = $false
  $startInfo.CreateNoWindow = $true
  $startInfo.RedirectStandardOutput = $true
  $startInfo.RedirectStandardError = $true
  $process = [System.Diagnostics.Process]::new()
  $process.StartInfo = $startInfo
  $started = $false
  try {
    $started = $process.Start()
    if (-not $started) {
      throw 'bootstrap.ps1: Git blob export process did not start'
    }
    $destinationStream = [System.IO.File]::Open(
      $Destination,
      [System.IO.FileMode]::CreateNew,
      [System.IO.FileAccess]::Write,
      [System.IO.FileShare]::None
    )
    try {
      $process.StandardOutput.BaseStream.CopyTo($destinationStream)
      $destinationStream.Flush($true)
    } finally {
      $destinationStream.Dispose()
    }
    $standardError = $process.StandardError.ReadToEnd()
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) {
      throw "bootstrap.ps1: Git blob export failed (exit $($process.ExitCode)): $standardError"
    }
  } finally {
    if ($started -and -not $process.HasExited) {
      try {
        $process.Kill()
        $process.WaitForExit()
      } catch {
        if (-not $process.HasExited) {
          throw "bootstrap.ps1: failed to terminate Git blob export process: $($_.Exception.Message)"
        }
      }
    }
    $process.Dispose()
  }

  $exportedOid = Get-GitSingleLine `
    -Repository $Repository `
    -Arguments @('hash-object', '--no-filters', '--', $Destination) `
    -Operation 'exported dispatcher verification'
  if ($exportedOid -cne $ExpectedOid) {
    throw "bootstrap.ps1: exported dispatcher object mismatch (expected $ExpectedOid, received $exportedOid)"
  }
}

function Install-Dispatcher {
  param(
    [Parameter(Mandatory = $true)][string]$RepositoryCandidate,
    [Parameter(Mandatory = $true)][bool]$AllowReplacement
  )

  $repository = Resolve-RepositoryRoot -Candidate $RepositoryCandidate
  $templateRelative = 'scripts/git-hooks/dispatcher'
  Assert-TrustedGatePath `
    -Repository $repository `
    -RelativePath 'scripts/git-hooks/pre-commit'
  $templateHeadOid = Assert-TrustedGatePath `
    -Repository $repository `
    -RelativePath $templateRelative `
    -RequireCleanContent
  Assert-TrustedGatePath `
    -Repository $repository `
    -RelativePath '.gitignore' `
    -RequireCleanContent

  $logsIgnoreResult = Invoke-GitResult `
    -Repository $repository `
    -Arguments @(
      'check-ignore',
      '--no-index',
      '--verbose',
      '--',
      'logs/bootstrap-install-probe'
    )
  if ($logsIgnoreResult.ExitCode -eq 1) {
    throw 'bootstrap.ps1: the tracked root .gitignore must exclude logs/ before dispatcher installation'
  }
  if ($logsIgnoreResult.ExitCode -ne 0) {
    throw "bootstrap.ps1: logs/ ignore verification failed (exit $($logsIgnoreResult.ExitCode)): $($logsIgnoreResult.DiagnosticText)"
  }
  $ignoreEvidence = @(
    $logsIgnoreResult.Lines |
      Where-Object { -not [string]::IsNullOrWhiteSpace($_) }
  )
  $ignoreFields = if ($ignoreEvidence.Count -eq 1) {
    @($ignoreEvidence[0].Split("`t"))
  } else {
    @()
  }
  if (
    $ignoreFields.Count -ne 2 -or
    $ignoreFields[0] -notmatch '^\.gitignore:[1-9][0-9]*:.+$' -or
    $ignoreFields[1] -cne 'logs/bootstrap-install-probe'
  ) {
    throw "bootstrap.ps1: logs/ must be excluded by the tracked root .gitignore rather than another ignore source: $($logsIgnoreResult.DiagnosticText)"
  }

  # No target-repository filesystem mutation may precede the effective
  # hook-path decision.
  $hooksPathResult = Invoke-GitResult `
    -Repository $repository `
    -Arguments @('config', '--get', 'core.hooksPath')
  switch (Resolve-HooksPathGuard -ExitCode $hooksPathResult.ExitCode) {
    'abort-set' {
      $display = if ([string]::IsNullOrEmpty($hooksPathResult.Text)) {
        '<empty>'
      } else {
        $hooksPathResult.Text
      }
      throw "bootstrap.ps1: core.hooksPath is set to '$display'; unset the value at the reported Git configuration scope before installing the per-clone dispatcher. -Force does not bypass this guard."
    }
    'abort-indeterminate' {
      throw "bootstrap.ps1: core.hooksPath lookup exited $($hooksPathResult.ExitCode); refusing installation under indeterminate Git configuration: $($hooksPathResult.DiagnosticText)"
    }
  }

  $dispatcherPath = Get-DispatcherPath -Repository $repository
  if (Test-Path -LiteralPath $dispatcherPath -PathType Container) {
    throw "bootstrap.ps1: dispatcher destination is a directory: $dispatcherPath"
  }
  if (Test-Path -LiteralPath $dispatcherPath -PathType Leaf) {
    $dispatcherItem = Get-Item -LiteralPath $dispatcherPath -Force
    if (($dispatcherItem.Attributes -band [System.IO.FileAttributes]::ReparsePoint) -ne 0) {
      throw "bootstrap.ps1: dispatcher destination must not be a reparse point: $dispatcherPath"
    }
    $existingOid = Get-GitSingleLine `
      -Repository $repository `
      -Arguments @('hash-object', '--no-filters', '--', $dispatcherPath) `
      -Operation 'existing dispatcher verification'
    if ($existingOid -ceq $templateHeadOid) {
      Write-Host "[bootstrap] dispatcher already matches the HEAD-trusted template: $dispatcherPath"
      return
    }
    if (-not $AllowReplacement) {
      throw "bootstrap.ps1: a different pre-commit hook already exists at '$dispatcherPath'. Inspect the hook, then rerun with -Force for atomic replacement with backup."
    }
  }

  $hooksDirectory = Split-Path -Parent $dispatcherPath
  if (Test-Path -LiteralPath $hooksDirectory) {
    if (-not (Test-Path -LiteralPath $hooksDirectory -PathType Container)) {
      throw "bootstrap.ps1: dispatcher parent is not a directory: $hooksDirectory"
    }
    $hooksDirectoryItem = Get-Item -LiteralPath $hooksDirectory -Force
    if (
      ($hooksDirectoryItem.Attributes -band
        [System.IO.FileAttributes]::ReparsePoint) -ne 0
    ) {
      throw "bootstrap.ps1: dispatcher parent must not be a reparse point: $hooksDirectory"
    }
  } else {
    New-Item -ItemType Directory -Path $hooksDirectory -Force | Out-Null
  }

  $temporaryPath = Join-Path $hooksDirectory (
    '.pre-commit.install-' + [guid]::NewGuid().ToString('N') + '.tmp'
  )
  $backupPath = $null
  try {
    Export-TrustedHeadBlob `
      -Repository $repository `
      -RelativePath $templateRelative `
      -ExpectedOid $templateHeadOid `
      -Destination $temporaryPath
    if ([System.IO.Path]::DirectorySeparatorChar -ne '\') {
      & chmod u+x -- $temporaryPath
      if ($LASTEXITCODE -ne 0) {
        throw "bootstrap.ps1: chmod failed for temporary dispatcher: $temporaryPath"
      }
    }

    if (Test-Path -LiteralPath $dispatcherPath -PathType Leaf) {
      $backupPath = Join-Path $hooksDirectory (
        'pre-commit.backup-' +
        (Get-Date).ToUniversalTime().ToString('yyyyMMddTHHmmssZ') +
        '-' +
        [guid]::NewGuid().ToString('N')
      )
      [System.IO.File]::Replace(
        $temporaryPath,
        $dispatcherPath,
        $backupPath,
        $true
      )
    } else {
      [System.IO.File]::Move($temporaryPath, $dispatcherPath)
    }

    $installedOid = Get-GitSingleLine `
      -Repository $repository `
      -Arguments @('hash-object', '--no-filters', '--', $dispatcherPath) `
      -Operation 'installed dispatcher verification'
    if ($installedOid -cne $templateHeadOid) {
      throw 'bootstrap.ps1: installed dispatcher failed verification'
    }
  } finally {
    if (Test-Path -LiteralPath $temporaryPath) {
      Remove-Item -LiteralPath $temporaryPath -Force -ErrorAction SilentlyContinue
    }
  }

  Write-Host "[bootstrap] installed HEAD-trusted dispatcher: $dispatcherPath"
  if ($null -ne $backupPath) {
    Write-Host "[bootstrap] previous hook backup: $backupPath"
  }
}

function Write-Utf8WithoutBom {
  param(
    [Parameter(Mandatory = $true)][string]$Path,
    [Parameter(Mandatory = $true)][string]$Content
  )

  $encoding = [System.Text.UTF8Encoding]::new($false)
  [System.IO.File]::WriteAllText($Path, $Content, $encoding)
}

function New-SelfTestRepository {
  param(
    [Parameter(Mandatory = $true)][string]$Root,
    [Parameter(Mandatory = $true)][string]$Name,
    [switch]$UseCandidateDispatcher
  )

  $repository = Join-Path $Root $Name
  New-Item -ItemType Directory -Path $repository -Force | Out-Null
  $init = Invoke-GitResult `
    -Repository $repository `
    -Arguments @('-c', 'init.templateDir=', 'init', '-q')
  if ($init.ExitCode -ne 0) {
    throw "self-test git init failed: $($init.DiagnosticText)"
  }
  foreach ($setting in @(
    @('config', 'user.name', 'BootstrapSelfTest'),
    @('config', 'user.email', 'bootstrap-selftest@example.invalid')
  )) {
    $configured = Invoke-GitResult -Repository $repository -Arguments $setting
    if ($configured.ExitCode -ne 0) {
      throw "self-test git config failed: $($configured.DiagnosticText)"
    }
  }

  $templatePath = Join-Path $repository 'scripts/git-hooks/dispatcher'
  $gatePath = Join-Path $repository 'scripts/git-hooks/pre-commit'
  New-Item -ItemType Directory -Path (Split-Path -Parent $templatePath) -Force |
    Out-Null
  if ($UseCandidateDispatcher) {
    $candidateDispatcher = Join-Path $PSScriptRoot 'scripts/git-hooks/dispatcher'
    if (-not (Test-Path -LiteralPath $candidateDispatcher -PathType Leaf)) {
      throw "self-test candidate dispatcher is missing: $candidateDispatcher"
    }
    [System.IO.File]::Copy($candidateDispatcher, $templatePath)
  } else {
    Write-Utf8WithoutBom -Path $templatePath -Content "#!/bin/sh`nexit 0`n"
  }
  Write-Utf8WithoutBom -Path $gatePath -Content "#!/bin/sh`nexit 0`n"
  Write-Utf8WithoutBom `
    -Path (Join-Path $repository '.gitignore') `
    -Content "logs/`n"
  $added = Invoke-GitResult `
    -Repository $repository `
    -Arguments @(
      'add',
      '--',
      '.gitignore',
      'scripts/git-hooks/dispatcher',
      'scripts/git-hooks/pre-commit'
    )
  if ($added.ExitCode -ne 0) {
    throw "self-test git add failed: $($added.DiagnosticText)"
  }
  $committed = Invoke-GitResult `
    -Repository $repository `
    -Arguments @('commit', '-q', '-m', 'self-test fixture')
  if ($committed.ExitCode -ne 0) {
    throw "self-test git commit failed: $($committed.DiagnosticText)"
  }
  return $repository
}

function Invoke-BootstrapChild {
  param(
    [Parameter(Mandatory = $true)][string]$Repository,
    [switch]$ChildForce
  )

  $arguments = @(
    '-NoProfile',
    '-ExecutionPolicy',
    'Bypass',
    '-File',
    $PSCommandPath,
    '-TargetRepo',
    $Repository
  )
  if ($ChildForce) {
    $arguments += '-Force'
  }
  $savedErrorActionPreference = $ErrorActionPreference
  try {
    $ErrorActionPreference = 'Continue'
    $output = @(& powershell @arguments 2>&1)
    $exitCode = $LASTEXITCODE
  } finally {
    $ErrorActionPreference = $savedErrorActionPreference
  }
  return [pscustomobject]@{
    ExitCode = $exitCode
    Text = (@($output | ForEach-Object { "$_" }) -join [Environment]::NewLine)
  }
}

function Invoke-BootstrapSelfTest {
  $failures = 0
  $scratchRoot = Join-Path (
    [System.IO.Path]::GetTempPath()
  ) ('ar4hmi-bootstrap-selftest-' + [guid]::NewGuid().ToString('N'))

  function Assert-SelfTest {
    param(
      [Parameter(Mandatory = $true)][string]$Name,
      [Parameter(Mandatory = $true)][bool]$Condition,
      [string]$Detail = ''
    )
    if ($Condition) {
      Write-Host "[SelfTest] PASS $Name"
    } else {
      Write-Host "[SelfTest] FAIL ${Name}: $Detail"
      $script:BootstrapSelfTestFailures++
    }
  }

  function Invoke-SkipCommit {
    param(
      [Parameter(Mandatory = $true)][string]$Repository,
      [Parameter(Mandatory = $true)][string]$Message
    )

    $processEnvironment = [Environment]::GetEnvironmentVariables(
      [EnvironmentVariableTarget]::Process
    )
    $hadSkipValue = $processEnvironment.Contains('CROSS_REVIEW_SKIP')
    $savedSkipValue = [Environment]::GetEnvironmentVariable(
      'CROSS_REVIEW_SKIP',
      [EnvironmentVariableTarget]::Process
    )
    try {
      [Environment]::SetEnvironmentVariable(
        'CROSS_REVIEW_SKIP',
        '1',
        [EnvironmentVariableTarget]::Process
      )
      $result = Invoke-GitResult `
        -Repository $Repository `
        -Arguments @('commit', '--allow-empty', '-q', '-m', $Message)
      return $result
    } finally {
      if ($hadSkipValue) {
        [Environment]::SetEnvironmentVariable(
          'CROSS_REVIEW_SKIP',
          $savedSkipValue,
          [EnvironmentVariableTarget]::Process
        )
      } else {
        [Environment]::SetEnvironmentVariable(
          'CROSS_REVIEW_SKIP',
          $null,
          [EnvironmentVariableTarget]::Process
        )
      }
    }
  }

  $script:BootstrapSelfTestFailures = 0
  try {
    New-Item -ItemType Directory -Path $scratchRoot -Force | Out-Null

    Assert-SelfTest `
      -Name 'unset core.hooksPath classification' `
      -Condition ((Resolve-HooksPathGuard -ExitCode 1) -eq 'proceed')
    Assert-SelfTest `
      -Name 'set core.hooksPath classification' `
      -Condition ((Resolve-HooksPathGuard -ExitCode 0) -eq 'abort-set')
    Assert-SelfTest `
      -Name 'indeterminate core.hooksPath classification' `
      -Condition ((Resolve-HooksPathGuard -ExitCode 2) -eq 'abort-indeterminate')

    $environmentProbeName = 'GIT_CONFIG_GLOBAL'
    $environmentProbeValue = Join-Path $scratchRoot 'probe-global-config'
    try {
      Set-Item `
        -LiteralPath "Env:$environmentProbeName" `
        -Value $environmentProbeValue `
        -ErrorAction Stop
      $probeCaughtExpectedFailure = $false
      $probeFailureMessage = ''
      try {
        Invoke-WithGitConfigEnvironmentScrub -Action {
          if (
            $null -ne
            [Environment]::GetEnvironmentVariable(
              'GIT_CONFIG_GLOBAL',
              [EnvironmentVariableTarget]::Process
            )
          ) {
            throw 'self-test environment probe was not scrubbed'
          }
          throw 'self-test action sentinel'
        }
      } catch {
        $probeFailureMessage = $_.Exception.Message
        $probeCaughtExpectedFailure = (
          $probeFailureMessage -ceq 'self-test action sentinel'
        )
      }
      $probeRestored = (
        [Environment]::GetEnvironmentVariable(
          $environmentProbeName,
          [EnvironmentVariableTarget]::Process
        ) -ceq $environmentProbeValue
      )
      Assert-SelfTest `
        -Name 'Git configuration environment is restored after a failing action' `
        -Condition ($probeCaughtExpectedFailure -and $probeRestored) `
        -Detail (
          "caughtExpected=$probeCaughtExpectedFailure restored=$probeRestored message=[$probeFailureMessage]"
        )
    } finally {
      Remove-Item `
        -LiteralPath "Env:$environmentProbeName" `
        -ErrorAction SilentlyContinue
    }

    $normal = New-SelfTestRepository -Root $scratchRoot -Name 'normal'
    $warningAlias = (
      "!f() { printf 'single-value\n'; " +
      "printf 'benign-warning\n' >&2; }; f"
    )
    $warningAliasConfigured = Invoke-GitResult `
      -Repository $normal `
      -Arguments @('config', 'alias.bootstrap-warning', $warningAlias)
    if ($warningAliasConfigured.ExitCode -ne 0) {
      throw "self-test warning alias setup failed: $($warningAliasConfigured.DiagnosticText)"
    }
    $warningResult = Invoke-GitResult `
      -Repository $normal `
      -Arguments @('bootstrap-warning')
    $warningValue = Get-GitSingleLine `
      -Repository $normal `
      -Arguments @('bootstrap-warning') `
      -Operation 'self-test warning-bearing single-line lookup'
    Assert-SelfTest `
      -Name 'Git stderr remains diagnostic rather than parsed stdout' `
      -Condition (
        $warningResult.ExitCode -eq 0 -and
        $warningResult.Lines.Count -eq 1 -and
        $warningResult.Text -ceq 'single-value' -and
        $warningResult.ErrorText -match 'benign-warning' -and
        $warningValue -ceq 'single-value'
      ) `
      -Detail $warningResult.DiagnosticText

    $normalResult = Invoke-BootstrapChild -Repository $normal
    $normalHook = Get-DispatcherPath -Repository $normal
    $normalTemplate = Join-Path $normal 'scripts/git-hooks/dispatcher'
    Assert-SelfTest `
      -Name 'normal install succeeds' `
      -Condition ($normalResult.ExitCode -eq 0) `
      -Detail $normalResult.Text
    Assert-SelfTest `
      -Name 'normal install copies exact trusted template' `
      -Condition (Test-FilesEqual -First $normalTemplate -Second $normalHook)

    $idempotentResult = Invoke-BootstrapChild -Repository $normal
    $normalBackups = @(
      Get-ChildItem `
        -LiteralPath (Split-Path -Parent $normalHook) `
        -Filter 'pre-commit.backup-*' `
        -File
    )
    Assert-SelfTest `
      -Name 'matching dispatcher install is idempotent' `
      -Condition (
        $idempotentResult.ExitCode -eq 0 -and
        $normalBackups.Count -eq 0
      ) `
      -Detail $idempotentResult.Text

    $lineEnding = New-SelfTestRepository `
      -Root $scratchRoot `
      -Name 'line-ending-mismatch'
    $lineEndingHook = Get-DispatcherPath -Repository $lineEnding
    New-Item `
      -ItemType Directory `
      -Path (Split-Path -Parent $lineEndingHook) `
      -Force |
      Out-Null
    Write-Utf8WithoutBom `
      -Path $lineEndingHook `
      -Content "#!/bin/sh`r`nexit 0`r`n"
    $lineEndingBefore = [System.IO.File]::ReadAllBytes($lineEndingHook)
    $lineEndingResult = Invoke-BootstrapChild -Repository $lineEnding
    $lineEndingAfter = [System.IO.File]::ReadAllBytes($lineEndingHook)
    Assert-SelfTest `
      -Name 'line-ending-only dispatcher mismatch requires Force' `
      -Condition (
        $lineEndingResult.ExitCode -ne 0 -and
        [System.Linq.Enumerable]::SequenceEqual(
          [byte[]]$lineEndingBefore,
          [byte[]]$lineEndingAfter
        )
      ) `
      -Detail $lineEndingResult.Text

    $conflict = New-SelfTestRepository -Root $scratchRoot -Name 'conflict'
    $conflictHook = Get-DispatcherPath -Repository $conflict
    New-Item `
      -ItemType Directory `
      -Path (Split-Path -Parent $conflictHook) `
      -Force |
      Out-Null
    Write-Utf8WithoutBom -Path $conflictHook -Content "#!/bin/sh`nexit 7`n"
    $conflictBefore = [System.IO.File]::ReadAllBytes($conflictHook)
    $conflictResult = Invoke-BootstrapChild -Repository $conflict
    $conflictAfter = [System.IO.File]::ReadAllBytes($conflictHook)
    Assert-SelfTest `
      -Name 'different existing hook fails without Force' `
      -Condition (
        $conflictResult.ExitCode -ne 0 -and
        [System.Linq.Enumerable]::SequenceEqual(
          [byte[]]$conflictBefore,
          [byte[]]$conflictAfter
        )
      ) `
      -Detail $conflictResult.Text

    $forceResult = Invoke-BootstrapChild -Repository $conflict -ChildForce
    $conflictTemplate = Join-Path $conflict 'scripts/git-hooks/dispatcher'
    $forceBackups = @(
      Get-ChildItem `
        -LiteralPath (Split-Path -Parent $conflictHook) `
        -Filter 'pre-commit.backup-*' `
        -File
    )
    $backupMatches = (
      $forceBackups.Count -eq 1 -and
      [System.Linq.Enumerable]::SequenceEqual(
        [byte[]]$conflictBefore,
        [byte[]][System.IO.File]::ReadAllBytes($forceBackups[0].FullName)
      )
    )
    Assert-SelfTest `
      -Name 'Force atomically replaces and backs up a different hook' `
      -Condition (
        $forceResult.ExitCode -eq 0 -and
        (Test-FilesEqual -First $conflictTemplate -Second $conflictHook) -and
        $backupMatches
      ) `
      -Detail $forceResult.Text

    $configured = New-SelfTestRepository -Root $scratchRoot -Name 'configured'
    $configuredSet = Invoke-GitResult `
      -Repository $configured `
      -Arguments @('config', 'core.hooksPath', 'custom-hooks')
    if ($configuredSet.ExitCode -ne 0) {
      throw "self-test core.hooksPath setup failed: $($configuredSet.DiagnosticText)"
    }
    $configuredResult = Invoke-BootstrapChild -Repository $configured -ChildForce
    $configuredDefaultHook = Join-Path $configured '.git/hooks/pre-commit'
    $configuredCustomHook = Join-Path $configured 'custom-hooks/pre-commit'
    Assert-SelfTest `
      -Name 'set core.hooksPath aborts before any hook write, including under Force' `
      -Condition (
        $configuredResult.ExitCode -ne 0 -and
        -not (Test-Path -LiteralPath $configuredDefaultHook) -and
        -not (Test-Path -LiteralPath $configuredCustomHook)
      ) `
      -Detail $configuredResult.Text

    $unignored = New-SelfTestRepository `
      -Root $scratchRoot `
      -Name 'unignored-logs'
    Write-Utf8WithoutBom `
      -Path (Join-Path $unignored '.gitignore') `
      -Content "# logs are intentionally unignored in this fixture`n"
    $unignoredAdded = Invoke-GitResult `
      -Repository $unignored `
      -Arguments @('add', '--', '.gitignore')
    $unignoredCommitted = Invoke-GitResult `
      -Repository $unignored `
      -Arguments @('commit', '-q', '-m', 'remove logs ignore')
    if (
      $unignoredAdded.ExitCode -ne 0 -or
      $unignoredCommitted.ExitCode -ne 0
    ) {
      throw (
        'self-test unignored fixture setup failed: ' +
        $unignoredAdded.DiagnosticText +
        [Environment]::NewLine +
        $unignoredCommitted.DiagnosticText
      )
    }
    $unignoredResult = Invoke-BootstrapChild -Repository $unignored
    $unignoredHook = Join-Path $unignored '.git/hooks/pre-commit'
    Assert-SelfTest `
      -Name 'missing logs ignore aborts before any hook write' `
      -Condition (
        $unignoredResult.ExitCode -ne 0 -and
        -not (Test-Path -LiteralPath $unignoredHook)
      ) `
      -Detail $unignoredResult.Text

    $emptyConfigured = New-SelfTestRepository `
      -Root $scratchRoot `
      -Name 'empty-configured'
    $emptyConfigPath = Get-GitSingleLine `
      -Repository $emptyConfigured `
      -Arguments @('rev-parse', '--git-path', 'config') `
      -Operation 'self-test config-path resolution'
    if (-not [System.IO.Path]::IsPathRooted($emptyConfigPath)) {
      $emptyConfigPath = Join-Path $emptyConfigured $emptyConfigPath
    }
    [System.IO.File]::AppendAllText(
      $emptyConfigPath,
      "`n[core]`n`thooksPath =`n",
      [System.Text.UTF8Encoding]::new($false)
    )
    $emptyLookup = Invoke-GitResult `
      -Repository $emptyConfigured `
      -Arguments @('config', '--get', 'core.hooksPath')
    if (
      $emptyLookup.ExitCode -ne 0 -or
      -not [string]::IsNullOrEmpty($emptyLookup.Text)
    ) {
      throw "self-test empty core.hooksPath setup failed: $($emptyLookup.DiagnosticText)"
    }
    $emptyResult = Invoke-BootstrapChild -Repository $emptyConfigured
    $emptyDefaultHook = Join-Path $emptyConfigured '.git/hooks/pre-commit'
    Assert-SelfTest `
      -Name 'empty core.hooksPath is treated as configured before any hook write' `
      -Condition (
        $emptyResult.ExitCode -ne 0 -and
        -not (Test-Path -LiteralPath $emptyDefaultHook)
      ) `
      -Detail $emptyResult.Text

    $subdirectory = New-SelfTestRepository `
      -Root $scratchRoot `
      -Name 'subdirectory'
    $nested = Join-Path $subdirectory 'nested/path'
    New-Item -ItemType Directory -Path $nested -Force | Out-Null
    $subdirectoryResult = Invoke-BootstrapChild -Repository $nested
    $subdirectoryHook = Get-DispatcherPath -Repository $subdirectory
    Assert-SelfTest `
      -Name 'subdirectory target normalizes to the repository top-level' `
      -Condition (
        $subdirectoryResult.ExitCode -eq 0 -and
        (Test-FilesEqual `
          -First (Join-Path $subdirectory 'scripts/git-hooks/dispatcher') `
          -Second $subdirectoryHook)
      ) `
      -Detail $subdirectoryResult.Text

    $dirty = New-SelfTestRepository -Root $scratchRoot -Name 'dirty-template'
    $dirtyTemplate = Join-Path $dirty 'scripts/git-hooks/dispatcher'
    [System.IO.File]::AppendAllText(
      $dirtyTemplate,
      "# unreviewed change`n",
      [System.Text.UTF8Encoding]::new($false)
    )
    $dirtyResult = Invoke-BootstrapChild -Repository $dirty
    $dirtyHook = Join-Path $dirty '.git/hooks/pre-commit'
    Assert-SelfTest `
      -Name 'working dispatcher template must match HEAD' `
      -Condition (
        $dirtyResult.ExitCode -ne 0 -and
        -not (Test-Path -LiteralPath $dirtyHook)
      ) `
      -Detail $dirtyResult.Text

    $dispatcherRepo = New-SelfTestRepository `
      -Root $scratchRoot `
      -Name 'executable-dispatcher' `
      -UseCandidateDispatcher
    $dispatcherInstall = Invoke-BootstrapChild -Repository $dispatcherRepo
    Assert-SelfTest `
      -Name 'candidate dispatcher installs for executable checks' `
      -Condition ($dispatcherInstall.ExitCode -eq 0) `
      -Detail $dispatcherInstall.Text

    $dispatcherGate = Join-Path `
      $dispatcherRepo `
      'scripts/git-hooks/pre-commit'
    Write-Utf8WithoutBom `
      -Path $dispatcherGate `
      -Content "#!/bin/sh`nexit 7`n"
    $trustedGateResult = Invoke-GitResult `
      -Repository $dispatcherRepo `
      -Arguments @(
        'commit',
        '--allow-empty',
        '-q',
        '-m',
        'dispatcher trusted gate probe'
      )
    Assert-SelfTest `
      -Name 'dispatcher executes the HEAD gate instead of the working candidate' `
      -Condition ($trustedGateResult.ExitCode -eq 0) `
      -Detail $trustedGateResult.DiagnosticText

    Remove-Item -LiteralPath $dispatcherGate -Force -ErrorAction Stop
    $missingGateResult = Invoke-GitResult `
      -Repository $dispatcherRepo `
      -Arguments @(
        'commit',
        '--allow-empty',
        '-q',
        '-m',
        'dispatcher missing gate rejection'
      )
    Assert-SelfTest `
      -Name 'dispatcher rejects a missing in-tree gate without an audited skip' `
      -Condition ($missingGateResult.ExitCode -ne 0) `
      -Detail $missingGateResult.DiagnosticText

    $skipResult = Invoke-SkipCommit `
      -Repository $dispatcherRepo `
      -Message 'dispatcher audited skip probe'
    $skipLog = Join-Path `
      $dispatcherRepo `
      'logs/review-skips/review-skip-log.txt'
    $skipRecords = @(
      if (Test-Path -LiteralPath $skipLog -PathType Leaf) {
        Get-Content -LiteralPath $skipLog
      }
    )
    $skipRecordPattern = (
      '^[0-9]{4}-[0-9]{2}-[0-9]{2}T' +
      '[0-9]{2}:[0-9]{2}:[0-9]{2}Z ' +
      'event=review-skip backend=none branch=[^\s]+ ' +
      'staged-tree=([0-9a-f]{40}|[0-9a-f]{64})$'
    )
    Assert-SelfTest `
      -Name 'dispatcher audited skip writes the shared record schema' `
      -Condition (
        $skipResult.ExitCode -eq 0 -and
        $skipRecords.Count -eq 1 -and
        $skipRecords[0] -cmatch $skipRecordPattern
      ) `
      -Detail (
        "exit=$($skipResult.ExitCode) records=[$($skipRecords -join '; ')] diagnostics=[$($skipResult.DiagnosticText)]"
      )

    $skipLogDirectory = Split-Path -Parent $skipLog
    Remove-TreeNoRecurse -Path $skipLogDirectory
    $skipLogRoot = Split-Path -Parent $skipLogDirectory
    if (-not (Test-Path -LiteralPath $skipLogRoot -PathType Container)) {
      New-Item -ItemType Directory -Path $skipLogRoot -Force | Out-Null
    }
    Write-Utf8WithoutBom `
      -Path $skipLogDirectory `
      -Content "skip log collision`n"
    $failedSkipResult = Invoke-SkipCommit `
      -Repository $dispatcherRepo `
      -Message 'dispatcher skip audit failure probe'
    Assert-SelfTest `
      -Name 'dispatcher skip fails closed when the audit path is unavailable' `
      -Condition ($failedSkipResult.ExitCode -ne 0) `
      -Detail $failedSkipResult.DiagnosticText
  } catch {
    Write-Host "[SelfTest] FAIL unexpected self-test exception: $($_.Exception.Message)"
    $script:BootstrapSelfTestFailures++
  } finally {
    try {
      if (Test-Path -LiteralPath $scratchRoot) {
        Remove-TreeNoRecurse -Path $scratchRoot
      }
    } catch {
      Write-Host "[SelfTest] FAIL cleanup raised: $($_.Exception.Message)"
      $script:BootstrapSelfTestFailures++
    }
    if (Test-Path -LiteralPath $scratchRoot) {
      Write-Host "[SelfTest] FAIL cleanup left scratch state: $scratchRoot"
      $script:BootstrapSelfTestFailures++
    } else {
      Write-Host '[SelfTest] PASS cleanup removed all scratch state'
    }
  }

  $failures = $script:BootstrapSelfTestFailures
  Remove-Variable -Name BootstrapSelfTestFailures -Scope Script -ErrorAction SilentlyContinue
  if ($failures -gt 0) {
    throw "bootstrap.ps1 self-test failed with $failures failure(s)"
  }
  Write-Host '[SelfTest] All bootstrap tests passed.'
}

try {
  Invoke-WithGitConfigEnvironmentScrub -Action {
    if ($SelfTest) {
      Invoke-BootstrapSelfTest
    } else {
      Install-Dispatcher `
        -RepositoryCandidate $TargetRepo `
        -AllowReplacement ([bool]$Force)
    }
  }
} catch {
  Write-Error $_.Exception.Message
  exit 1
}

exit 0
