# Cross-review commit wrapper for Claude-as-implementer.
#
# Used when Claude authored the staged changes. Sets the REVIEW_BACKEND
# env var to `codex` so the pre-commit hook routes review to the Codex
# wrapper (cross-review). Then forwards arguments to `git commit`,
# EXCEPT hook-bypass flags (`--no-verify`, `-n`, and clustered short
# options containing `n` such as `-nF` or `-nm`) which are rejected
# before forwarding. The wrapper exists to enforce cross-review
# routing, so it refuses to participate in a hook-bypass path; use
# CROSS_REVIEW_SKIP=1 (audited per your project's AGENTS.md) for the
# documented skip case. Exits with git's exit code, or 1 on an ambiguous
# argument, a bypass-flag rejection, a guarded-environment failure, a failed
# hook-bypass env-var scrub, a set core.hooksPath that would redirect the
# pre-commit dispatcher, or a pre-commit abort.
#
# Usage examples (run from the Claude session):
#   scripts/claude/commit.ps1 -m "task X: short summary"
#   scripts/claude/commit.ps1 -F .commit-msg.tmp
#   scripts/claude/commit.ps1 --amend
#
# Setting REVIEW_BACKEND=codex here is REQUIRED -- the pre-commit
# hook has NO implicit default and rejects bare `git commit` when
# REVIEW_BACKEND is unset. The wrapper makes the implementer-vs-
# reviewer pairing explicit and symmetric with scripts/codex/commit.ps1
# (saved/restored so later commits in the same shell are not
# affected). It also makes future workflow changes (e.g.,
# REVIEW_BACKEND=both for high-stakes commits) a one-line edit in
# the wrapper rather than every commit call site.
#
# The wrapper does NOT skip the pre-commit hook -- it just selects
# which backend reviews. CROSS_REVIEW_SKIP=1 is still the only skip path
# (documented in scripts/git-hooks/pre-commit and your AGENTS.md).
#
# Symmetric counterpart for Codex-as-implementer:
# scripts/codex/commit.ps1 (sets REVIEW_BACKEND=claude).

# Save/restore the caller's REVIEW_BACKEND so the env mutation is
# scoped to THIS `git commit` only. Symmetric with
# scripts/codex/commit.ps1 (leaving REVIEW_BACKEND set would route
# later bare commits to a stale backend, defeating the wrapper's
# purpose).
# Reject hook-bypass flags BEFORE invoking git: the wrapper's comment
# block above promises "does NOT skip the pre-commit hook", but a
# naive `git commit @args` pass-through would forward `--no-verify` /
# `-n` straight to git, bypassing the hook before REVIEW_BACKEND
# routing runs. Reject those flags explicitly so the wrapper's stated
# contract holds against caller misuse.
# Value-taking options for `git commit`. See scripts/codex/commit.ps1
# for the full per-letter rationale + the optional-attached split.
$alwaysValueShort   = @('m','F','c','C','t')
$optionalValueShort = @('S','u')
$valueTakingShortOpts = $alwaysValueShort + $optionalValueShort

# Shared parser used by production AND --self-test (prevents
# parser-vs-self-test divergence). See scripts/codex/commit.ps1 for
# the full per-clause rationale.
function Test-Bypass-Args {
  param([string[]]$ArgList)
  $skipNext = $false
  $afterDoubleDash = $false
  $bypass = $false
  $invalid = $false
  $offending = ''
  foreach ($a in $ArgList) {
    if ($afterDoubleDash) { continue }
    if ($skipNext) { $skipNext = $false; continue }
    if ($a -eq '--') { $afterDoubleDash = $true; continue }
    if ($valueTakingLongOpts -contains $a) { $skipNext = $true; continue }
    if (Test-AbbreviatedValueLongOption -Token $a) {
      $invalid = $true; $offending = $a; break
    }
    if ($a -match '^-([a-zA-Z])$' -and ($alwaysValueShort -ccontains $matches[1])) {
      $skipNext = $true; continue
    }
    if ($a -match '^--no-v') { $bypass = $true; $offending = $a; break }
    if ($a -match '^-[a-zA-Z]' -and $a -notmatch '^--') {
      $letters = $a.Substring(1).ToCharArray()
      for ($i = 0; $i -lt $letters.Length; $i++) {
        $ch = $letters[$i]
        if ($ch -ceq 'n') { $bypass = $true; $offending = $a; break }
        if ($valueTakingShortOpts -ccontains [string]$ch) {
          if ($i -eq $letters.Length - 1 -and $alwaysValueShort -ccontains [string]$ch) {
            $skipNext = $true
          }
          break
        }
        if ($ch -notmatch '[a-zA-Z]') { break }
      }
      if ($bypass) { break }
    }
  }
  return @{ Bypass = $bypass; Invalid = $invalid; Offending = $offending }
}
$valueTakingLongOpts = @(
  '--message', '--file', '--author', '--date', '--template',
  '--reuse-message', '--reedit-message', '--fixup', '--squash',
  '--cleanup', '--pathspec-from-file', '--trailer'
)

# Keep value ownership unambiguous across Git versions. Git accepts long-option
# prefixes, but a wrapper cannot safely infer whether the following token is a
# value without the complete version-specific option inventory.
function Test-AbbreviatedValueLongOption {
  param([string]$Token)
  if (
    [string]::IsNullOrEmpty($Token) -or
    -not $Token.StartsWith('--', [System.StringComparison]::Ordinal) -or
    $Token.Contains('=')
  ) {
    return $false
  }
  foreach ($fullOption in $valueTakingLongOpts) {
    if (
      $Token.Length -lt $fullOption.Length -and
      $fullOption.StartsWith($Token, [System.StringComparison]::Ordinal)
    ) {
      return $true
    }
  }
  return $false
}
# `--gpg-sign` and `--untracked-files` take OPTIONAL values only --
# see scripts/codex/commit.ps1 for the full rationale. Both must use
# the attached `=<value>` form, not the separate-argv form.

# Classify the `git config --get core.hooksPath` exit status for the
# config-redirect guard below. PURE (no git invocation) and shared by the
# production guard AND the --self-test fixtures so both exercise the same
# classification (no prod-vs-selftest drift). Exit 1 = key ABSENT (the
# required UNSET state) -> 'proceed'; exit 0 = key SET to ANY value
# INCLUDING an empty `core.hooksPath=` (still a configured hook-path state
# that can redirect the dispatcher) -> 'abort-set'; anything else = git
# error -> 'abort-indeterminate' (fail closed). Byte-identical to the copy
# in scripts/codex/commit.ps1.
function Resolve-HooksPathGuard {
  param([int]$ExitCode)
  if ($ExitCode -eq 1) { return 'proceed' }
  if ($ExitCode -eq 0) { return 'abort-set' }
  return 'abort-indeterminate'
}

# PURE PATH-assembly helper for Enable-GitUnixToolPathForHook (below): given the
# (already existence-filtered) candidate tool dirs + the current PATH string,
# return the ordered, case-insensitively-deduped parts with the candidates
# PREPENDED, then the current PATH's parts (skipping blanks and any already
# contributed by a candidate). No env mutation and no filesystem I/O, so the
# --self-test drives it directly. Byte-identical to the copies in
# scripts/codex/commit.ps1 and scripts/codex/auto-merge.ps1.
function Get-PrependedToolPathParts {
  param([string[]]$CandidateDirs, [string]$CurrentPath)
  $parts = New-Object 'System.Collections.Generic.List[string]'
  $seen = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
  if ($CandidateDirs) {
    foreach ($c in $CandidateDirs) {
      if (-not [string]::IsNullOrWhiteSpace($c) -and $seen.Add($c)) { [void]$parts.Add($c) }
    }
  }
  foreach ($p in (($CurrentPath -split [System.IO.Path]::PathSeparator))) {
    if (-not [string]::IsNullOrWhiteSpace($p) -and $seen.Add($p)) { [void]$parts.Add($p) }
  }
  return ,$parts.ToArray()
}

# Collect EVERY PATH-spelling VALUE from the process environment BLOCK. A Windows
# process can inherit a DUPLICATE surface with both 'Path' and 'PATH' as distinct
# case-variant entries holding DIFFERENT dirs; [Environment]::GetEnvironmentVariable
# is case-INSENSITIVE and returns only ONE of them (Path == PATH at runtime), so a
# two-lookup read cannot see both -- enumerate the raw block instead.
# [Environment]::GetEnvironmentVariables('Process') returns a case-SENSITIVE
# Hashtable, so both keys survive when present. $EnvEntries defaults to the live
# block; --self-test injects a synthetic case-sensitive Hashtable (the SAME type)
# with both spellings. Returns the values as an array (one per matching entry).
# Byte-identical to the copies in scripts/codex/commit.ps1 and scripts/codex/auto-merge.ps1.
function Get-ProcessPathSpellings {
  param([System.Collections.IDictionary]$EnvEntries)
  if ($null -eq $EnvEntries) { $EnvEntries = [Environment]::GetEnvironmentVariables('Process') }
  $out = New-Object 'System.Collections.Generic.List[string]'
  foreach ($k in @($EnvEntries.Keys)) {
    if ([string]$k -match '(?i)^PATH$') {
      $v = [string]$EnvEntries[$k]
      if (-not [string]::IsNullOrEmpty($v)) { [void]$out.Add($v) }
    }
  }
  return ,$out.ToArray()
}

# PURE: union the given process PATH-spelling VALUES (0..N of them -- see
# Get-ProcessPathSpellings; a duplicate surface yields two DIFFERING strings) into
# the ordered, case-insensitively-deduped parts, candidates PREPENDED. Earlier
# spellings precede later ones (a fresh 'Path' edit before a stale 'PATH'); reading
# only ONE spelling would silently DROP the other's dirs. Delegates the dedup to
# Get-PrependedToolPathParts; no env mutation / no I/O, so --self-test drives it.
# Byte-identical to the copies in scripts/codex/commit.ps1 and scripts/codex/auto-merge.ps1.
function Get-UnionProcessPathParts {
  param([string[]]$Spellings, [string[]]$CandidateDirs = @())
  $joined = (@($Spellings) | Where-Object { -not [string]::IsNullOrEmpty($_) }) -join [System.IO.Path]::PathSeparator
  return Get-PrependedToolPathParts -CandidateDirs $CandidateDirs -CurrentPath $joined
}

# Apply bypass check via the shared Test-Bypass-Args function.
$bypassResult = Test-Bypass-Args -ArgList $args
if ($bypassResult.Invalid) {
  Write-Error "scripts/claude/commit.ps1: abbreviated value-taking long option '$($bypassResult.Offending)' is unsupported; spell the Git option in full so argument ownership remains unambiguous."
  exit 1
}

function Set-ProcessEnvironmentValueVerified {
  param([string]$Name, [string]$Value)
  if ([string]::IsNullOrWhiteSpace($Name) -or $null -eq $Value) {
    throw 'environment assignment requires a name and non-null value'
  }
  Set-Item -LiteralPath "Env:$Name" -Value $Value -ErrorAction Stop
  $actual = [Environment]::GetEnvironmentVariable($Name, 'Process')
  if ($actual -cne $Value) {
    throw "environment value '$Name' did not match after assignment"
  }
}

function Clear-ProcessEnvironmentValueVerified {
  param([string]$Name)
  if ([string]::IsNullOrWhiteSpace($Name)) {
    throw 'environment removal requires a name'
  }
  Remove-Item -LiteralPath "Env:$Name" -ErrorAction SilentlyContinue
  if ($null -ne [Environment]::GetEnvironmentVariable($Name, 'Process')) {
    throw "environment value '$Name' persisted after removal"
  }
}
if ($bypassResult.Bypass) {
  Write-Error "scripts/claude/commit.ps1: refusing $($bypassResult.Offending) -- this wrapper exists to enforce cross-review routing; --no-verify / -n / clustered -n* short options / --no-v[erify] prefix abbreviations would bypass the pre-commit hook entirely. Use CROSS_REVIEW_SKIP=1 (audited per your project's AGENTS.md) for the documented skip path; never --no-verify."
  exit 1
}

# The GIT_CONFIG_* family that can redirect git config resolution -- and thus a
# `core.hooksPath` override -- into a child git process. Defined ONCE here so the
# two scrub sites (the production `git commit` scrub below, and the --self-test
# E2E's scratch-repo setup) cannot drift. See scripts/codex/commit.ps1 for the
# per-variable rationale.
$gitConfigEnvNames = @(
  'GIT_CONFIG',
  'GIT_CONFIG_GLOBAL',
  'GIT_CONFIG_SYSTEM',
  'GIT_CONFIG_NOSYSTEM',
  'GIT_CONFIG_PARAMETERS',
  'GIT_CONFIG_COUNT'
)

if ($args.Count -ge 1 -and $args[0] -eq '--self-test') {
  function Test-Bypass {
    param(
      [string[]]$ArgList,
      [bool]$ExpectBypass,
      [string]$Name,
      [bool]$ExpectInvalid = $false
    )
    # Calls the shared Test-Bypass-Args function above; no local copy.
    $r = Test-Bypass-Args -ArgList $ArgList
    if ($r.Bypass -eq $ExpectBypass -and $r.Invalid -eq $ExpectInvalid) {
      Write-Host ("[SelfTest] PASS $Name" + $(if ($r.Offending) { " ($($r.Offending))" } else { '' }))
      return $true
    } else {
      Write-Host "[SelfTest] FAIL ${Name}: expected bypass=$ExpectBypass invalid=$ExpectInvalid got bypass=$($r.Bypass) invalid=$($r.Invalid)"
      return $false
    }
  }
  $ok = $true
  $environmentProbeName = 'CROSS_REVIEW_ENV_PROBE_' + [guid]::NewGuid().ToString('N')
  try {
    Set-ProcessEnvironmentValueVerified -Name $environmentProbeName -Value 'probe-value'
    $environmentSetPassed = (
      [Environment]::GetEnvironmentVariable($environmentProbeName, 'Process') -ceq 'probe-value'
    )
    Clear-ProcessEnvironmentValueVerified -Name $environmentProbeName
    $environmentClearPassed = (
      $null -eq [Environment]::GetEnvironmentVariable($environmentProbeName, 'Process')
    )
  } catch {
    $environmentSetPassed = $false
    $environmentClearPassed = $false
  } finally {
    Remove-Item -LiteralPath "Env:$environmentProbeName" -ErrorAction SilentlyContinue
  }
  if ($environmentSetPassed -and $environmentClearPassed) {
    Write-Host '[SelfTest] PASS verified environment assignment and removal helpers'
  } else {
    Write-Host '[SelfTest] FAIL verified environment assignment and removal helpers'
    $ok = $false
  }
  $ok = (Test-Bypass @('--no-verify') $true 'reject --no-verify') -and $ok
  $ok = (Test-Bypass @('--no-v') $true 'reject --no-v prefix') -and $ok
  $ok = (Test-Bypass @('--no-verify-message') $true 'reject --no-verify-message (over-reject acceptable)') -and $ok
  $ok = (Test-Bypass @('-n') $true 'reject -n') -and $ok
  $ok = (Test-Bypass @('-nF', '.commit-msg.tmp') $true 'reject -nF cluster') -and $ok
  $ok = (Test-Bypass @('-an') $true 'reject -an cluster') -and $ok
  $ok = (Test-Bypass @('-anFhello') $true 'reject -anFhello cluster') -and $ok
  $ok = (Test-Bypass @('-m', 'message') $false 'accept -m message') -and $ok
  $ok = (Test-Bypass @('-m', '-n: message') $false 'accept -m "-n: message"') -and $ok
  $ok = (Test-Bypass @('-F', '--no-verify-msg.txt') $false 'accept -F --no-verify-msg.txt') -and $ok
  $ok = (Test-Bypass @('--', '-n') $false 'accept -- -n pathspec') -and $ok
  $ok = (Test-Bypass @('-mno-op') $false 'accept -mno-op (m value)') -and $ok
  $ok = (Test-Bypass @('-Fnotes.txt') $false 'accept -Fnotes.txt (F value)') -and $ok
  $ok = (Test-Bypass @('-Sname') $false 'accept -Sname (S optional-value)') -and $ok
  $ok = (Test-Bypass @('-uno') $false 'accept -uno (u optional-value)') -and $ok
  $ok = (Test-Bypass @('-am', 'message') $false 'accept -am cluster (no n)') -and $ok
  $ok = (Test-Bypass @('-am', '-n: message') $false 'accept -am "-n: msg" (m is terminal, msg is value not option)') -and $ok
  $ok = (Test-Bypass @('--message=value') $false 'accept --message=value') -and $ok
  $ok = (Test-Bypass @('--mess', '--no-verify') $false 'reject abbreviated value-taking long option before ambiguous data' $true) -and $ok
  $ok = (Test-Bypass @('--message', '--no-verify') $false 'accept bypass-like text as a full-option value') -and $ok
  # Standalone optional-value reject fixtures.
  $ok = (Test-Bypass @('-S', '--no-verify') $true 'reject -S then --no-verify (S optional-value, no skipNext)') -and $ok
  $ok = (Test-Bypass @('-u', '--no-verify') $true 'reject -u then --no-verify (u optional-value, no skipNext)') -and $ok
  $ok = (Test-Bypass @('--gpg-sign', '--no-verify') $true 'reject --gpg-sign then --no-verify (gpg-sign optional-value, no skipNext)') -and $ok
  # Resolve-HooksPathGuard classification fixtures (mirrors scripts/codex/commit.ps1):
  # the core.hooksPath config-redirect guard below is security-critical routing
  # protection; assert the exit-status mapping directly (unset -> 1, set and
  # set-empty -> 0, git errors -> anything else).
  function Test-HooksPathGuard {
    param([int]$ExitCode, [string]$Want, [string]$Name)
    $r = Resolve-HooksPathGuard -ExitCode $ExitCode
    if ($r -eq $Want) {
      Write-Host "[SelfTest] PASS $Name (-> $r)"
      return $true
    } else {
      Write-Host "[SelfTest] FAIL ${Name}: want $Want got $r"
      return $false
    }
  }
  $ok = (Test-HooksPathGuard 1 'proceed' 'hooksPath exit 1 (key absent/unset) -> proceed') -and $ok
  $ok = (Test-HooksPathGuard 0 'abort-set' 'hooksPath exit 0 (set to any value incl set-empty) -> abort') -and $ok
  $ok = (Test-HooksPathGuard 2 'abort-indeterminate' 'hooksPath exit 2 (config file error) -> fail closed') -and $ok
  $ok = (Test-HooksPathGuard 128 'abort-indeterminate' 'hooksPath exit 128 (fatal git error) -> fail closed') -and $ok

  # Get-PrependedToolPathParts: the PURE PATH-assembly Enable-GitUnixToolPathForHook
  # (prepend git Unix-tool dirs) and auto-merge's Normalize (dedup-only) both rely
  # on. Assert prepend order, case-insensitive dedup (incl. a candidate already in
  # PATH), blank-segment skip, and the no-candidates dedup case (the auto-merge
  # shape). Separators are ';' (Windows [System.IO.Path]::PathSeparator).
  function Test-PathParts {
    param([string[]]$Candidates, [string]$Current, [string]$Want, [string]$Name)
    $got = [string]::Join(';', (Get-PrependedToolPathParts -CandidateDirs $Candidates -CurrentPath $Current))
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name"; return $true }
    Write-Host "[SelfTest] FAIL ${Name}: got [$got] want [$Want]"; return $false
  }
  $ok = (Test-PathParts @('C:\git\usr\bin', 'C:\git\bin') 'C:\a;C:\git\bin;C:\b' 'C:\git\usr\bin;C:\git\bin;C:\a;C:\b' 'PathParts: prepend + dedup of a candidate already in PATH') -and $ok
  $ok = (Test-PathParts @('C:\git\bin') 'C:\a;;C:\A;C:\b' 'C:\git\bin;C:\a;C:\b' 'PathParts: blank-skip + case-insensitive dedup (C:\A == C:\a)') -and $ok
  $ok = (Test-PathParts @() 'C:\a;C:\a;C:\b' 'C:\a;C:\b' 'PathParts: no candidates -> dedup-only (auto-merge Normalize shape)') -and $ok

  # Get-ProcessPathSpellings + Get-UnionProcessPathParts: the duplicate Path/PATH
  # surface fix. A Windows process CAN inherit both 'Path' and 'PATH' as distinct
  # case-variant entries; [Environment]::GetEnvironmentVariable is case-INSENSITIVE
  # (returns one), so Get-ProcessPathSpellings enumerates the raw block instead.
  # GetEnvironmentVariables('Process') returns a case-SENSITIVE Hashtable, faithfully
  # simulated here with New-Object System.Collections.Hashtable (holds both keys).
  function Test-SpellVals {
    param([System.Collections.IDictionary]$Env, [int]$WantCount, [string[]]$WantVals, [string]$Name)
    $got = Get-ProcessPathSpellings -EnvEntries $Env   # NO @() -- the ,$array return double-nests under @()
    $got = @($got)
    $ok2 = (@($got).Count -eq $WantCount)
    foreach ($wv in $WantVals) { if ($got -notcontains $wv) { $ok2 = $false } }
    if ($ok2) { Write-Host "[SelfTest] PASS $Name"; return $true }
    Write-Host "[SelfTest] FAIL ${Name}: got [$([string]::Join('|', $got))] want count=$WantCount vals=[$([string]::Join('|', $WantVals))]"; return $false
  }
  $stDup = New-Object System.Collections.Hashtable
  $stDup['Path'] = 'C:\a;C:\b'; $stDup['PATH'] = 'C:\c'; $stDup['HOME'] = 'x'
  $ok = (Test-SpellVals $stDup 2 @('C:\a;C:\b', 'C:\c') 'Spellings: a DUPLICATE Path+PATH surface yields BOTH distinct values (GetEnvironmentVariable would see one)') -and $ok
  $stOne = New-Object System.Collections.Hashtable
  $stOne['PATH'] = 'C:\only'; $stOne['HOME'] = 'x'
  $ok = (Test-SpellVals $stOne 1 @('C:\only') 'Spellings: a single PATH entry yields one value') -and $ok
  $stEmpty = New-Object System.Collections.Hashtable
  $stEmpty['PATH'] = ''; $stEmpty['HOME'] = 'x'
  $ok = (Test-SpellVals $stEmpty 0 @() 'Spellings: an empty PATH value is skipped') -and $ok
  # Production-boundary smoke: the DEFAULT-arg path enumerates the LIVE process block
  # and returns a non-empty PATH (exercises the real GetEnvironmentVariables read).
  $stLive = Get-ProcessPathSpellings   # NO @() -- see Test-SpellVals note
  $stLive = @($stLive)
  $stLiveOk = ((@($stLive).Count -ge 1) -and (-not [string]::IsNullOrEmpty($stLive[0])))
  if ($stLiveOk) { Write-Host '[SelfTest] PASS Spellings: live process block yields a non-empty PATH (production-boundary read)' } else { Write-Host "[SelfTest] FAIL Spellings: live process block read (count=$(@($stLive).Count))" }
  $ok = $stLiveOk -and $ok

  function Test-UnionParts {
    param([string[]]$Spellings, [string[]]$Candidates, [string]$Want, [string]$Name)
    $got = [string]::Join(';', (Get-UnionProcessPathParts -Spellings $Spellings -CandidateDirs $Candidates))
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name"; return $true }
    Write-Host "[SelfTest] FAIL ${Name}: got [$got] want [$Want]"; return $false
  }
  $ok = (Test-UnionParts @('C:\a;C:\b', 'C:\c') @('C:\git\usr\bin') 'C:\git\usr\bin;C:\a;C:\b;C:\c' 'UnionParts: two differing spellings union (neither dropped), candidate prepended') -and $ok
  $ok = (Test-UnionParts @('C:\a;C:\b', 'C:\c;C:\a') @() 'C:\a;C:\b;C:\c' 'UnionParts: dedup-only union (Normalize shape) preserves both spellings') -and $ok
  $ok = (Test-UnionParts @('C:\a;C:\b') @() 'C:\a;C:\b' 'UnionParts: a single spelling -> that spelling') -and $ok
  $ok = (Test-UnionParts @() @() '' 'UnionParts: no spellings -> empty') -and $ok

  # ---- E2E: the PRODUCTION guard aborts when core.hooksPath is SET ----
  # Pins the PRODUCTION path (git config read -> switch dispatch -> abort exit 1)
  # by running THIS wrapper as a child against a scratch repo whose LOCAL
  # core.hooksPath is set, and asserting exit 1 + the SET diagnostic. The guard
  # aborts before `git commit`, so the scratch repo needs no staged content.
  # Restoration of the scrubbed git-config env on that abort is a PowerShell
  # try/finally language guarantee (the guard runs inside the same try as the
  # commit). The scratch repo is swept with a no-recurse delete (package policy
  # bans Remove-Item -Recurse). Mirrors scripts/codex/commit.ps1.
  #
  # TWO set-forms are exercised so a stdout-based regression cannot hide:
  #   (1) a NON-empty custom path via `git config` (exit 0, non-empty stdout);
  #   (2) an EMPTY value `core.hooksPath=` written RAW into .git/config (exit 0,
  #       EMPTY stdout) -- the exact set-empty state the exit-status guard targets;
  #       a stdout-based (`IsNullOrWhiteSpace`) guard would leak this one. NOTE:
  #       `git config core.hooksPath ""` does NOT produce it -- git returns
  #       `--get` exit 1 for a value set that way -- so the empty case is written raw.
  $guardScratch = Join-Path ([System.IO.Path]::GetTempPath()) ("crg-commit-guard-e2e-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
  # Scrub the git-config redirect family around the scratch-repo setup so a masking
  # parent env (GIT_CONFIG_GLOBAL/SYSTEM/...) cannot redirect where `git config`
  # writes (the scratch's LOCAL core.hooksPath must land for the child to read it)
  # or make `git init` read an unexpected surface. Saved here, restored in finally.
  $e2eSavedGitCfg = @{}
  try {
    # Scrub the redirect family FAIL-CLOSED: verify each removal actually took, so a
    # masking var that a locked-down provider refuses to clear cannot silently leave
    # the setup running against an unexpected config surface.
    $e2eSetupOk = $true
    foreach ($n in $gitConfigEnvNames) {
      $e2eSavedGitCfg[$n] = [Environment]::GetEnvironmentVariable($n)
      if ($null -ne $e2eSavedGitCfg[$n]) {
        Remove-Item -LiteralPath "Env:$n" -ErrorAction SilentlyContinue
        if ($null -ne [Environment]::GetEnvironmentVariable($n)) { $e2eSetupOk = $false }
      }
    }
    if (-not $e2eSetupOk) {
      Write-Host '[SelfTest] FAIL E2E: could not scrub GIT_CONFIG_* for the scratch-repo setup (a masking var could make the child abort for the wrong reason)'
      $ok = $false
    } else {
      New-Item -ItemType Directory -Path $guardScratch -Force | Out-Null
      & git -C $guardScratch init -q 2>&1 | Out-Null
      if ($LASTEXITCODE -ne 0) {
        Write-Host '[SelfTest] FAIL E2E: git init of the scratch repo failed'
        $ok = $false
      } else {
        $guardCfgPath = Join-Path $guardScratch '.git/config'
        # Each case: Setup returns $true on success; Want is the LOCAL value the
        # scratch must hold. A `--unset-all` first clears the prior case so the empty
        # case is not shadowed by the non-empty case's value.
        $guardCases = @(
          @{ Label = "non-empty ('probe-hooks-dir')"; Want = 'probe-hooks-dir'; Setup = { (& git -C $guardScratch config --unset-all core.hooksPath 2>&1 | Out-Null); (& git -C $guardScratch config core.hooksPath probe-hooks-dir 2>&1 | Out-Null); return ($LASTEXITCODE -eq 0) } }
          @{ Label = 'empty (core.hooksPath=)';       Want = '';               Setup = { (& git -C $guardScratch config --unset-all core.hooksPath 2>&1 | Out-Null); Add-Content -LiteralPath $guardCfgPath -Value "[core]`n`thooksPath ="; return $true } }
        )
        foreach ($gc in $guardCases) {
          $setupOk = (& $gc.Setup)
          # Assert the scratch's LOCAL config actually holds the intended value BEFORE
          # invoking the child. Without this, a global/system core.hooksPath on the
          # test machine could make the child abort with the SET diagnostic even when
          # the local setup silently failed -- a false PASS. (`--local` reads only the
          # scratch config; both forms return exit 0, the empty one with a '' value.)
          $localVal = (& git -C $guardScratch config --local --get core.hooksPath 2>&1)
          $localExit = $LASTEXITCODE
          if (-not ($setupOk -and ($localExit -eq 0) -and (("$localVal").Trim() -eq $gc.Want))) {
            Write-Host "[SelfTest] FAIL E2E ($($gc.Label)): scratch LOCAL core.hooksPath was not set as intended (setupOk=$setupOk localExit=$localExit localVal=[$(("$localVal").Trim())] want=[$($gc.Want)]) -- cannot trust the guard result"
            $ok = $false
            continue
          }
          Push-Location -LiteralPath $guardScratch
          try {
            $guardOut = (& powershell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -m 'guard probe' 2>&1 | ForEach-Object { $_.ToString() } | Out-String)
            $guardExit = $LASTEXITCODE
          } finally { Pop-Location }
          # Convert child output objects to raw text before Out-String above.
          # Windows PowerShell otherwise formats redirected native stderr as rich
          # ErrorRecord data and can inject formatter metadata between wrapped
          # message fragments. Whitespace normalization then makes ordinary console
          # wrapping irrelevant without accepting unrelated formatter text.
          $guardOk = (($guardExit -eq 1) -and (($guardOut -replace '\s+', ' ') -match 'core\.hooksPath is SET'))
          if ($guardOk) {
            Write-Host "[SelfTest] PASS E2E: production guard aborts (exit 1) when core.hooksPath is SET -- $($gc.Label)"
          } else {
            Write-Host "[SelfTest] FAIL E2E ($($gc.Label)): expected exit 1 + 'core.hooksPath is SET' (got exit=$guardExit)"
          }
          $ok = $guardOk -and $ok
        }

        & git -C $guardScratch config --local --unset-all core.hooksPath 2>&1 | Out-Null
        $routeUnsetExit = $LASTEXITCODE
        & git -C $guardScratch config --local user.email selftest@local 2>&1 | Out-Null
        $routeEmailExit = $LASTEXITCODE
        & git -C $guardScratch config --local user.name selftest 2>&1 | Out-Null
        $routeNameExit = $LASTEXITCODE
        $routeHookPath = Join-Path $guardScratch '.git/hooks/pre-commit'
        $routeRecordPath = Join-Path $guardScratch '.git/route-probe.txt'
        $routeProbePath = Join-Path $guardScratch 'route-probe.txt'
        $routeFileSetupOk = $true
        try {
          Set-Content -LiteralPath $routeProbePath -Value 'route probe' -Encoding Ascii -ErrorAction Stop
          $routeHookBody = @'
#!/bin/sh
printf '%s|%s|%s|%s\n' "${REVIEW_BACKEND-}" "${CROSS_REVIEW_FALLBACK+x}" "${CROSS_REVIEW_FALLBACK-}" "${GIT_CONFIG_NOSYSTEM+x}" > .git/route-probe.txt
exit 1
'@
          Set-Content -LiteralPath $routeHookPath -Value $routeHookBody -Encoding Ascii -NoNewline -ErrorAction Stop
          & git -C $guardScratch -c core.excludesfile= add route-probe.txt 2>&1 | Out-Null
          if ($LASTEXITCODE -ne 0) { $routeFileSetupOk = $false }
        } catch {
          $routeFileSetupOk = $false
        }
        $routeSetupOk = (
          ($routeUnsetExit -eq 0) -and
          ($routeEmailExit -eq 0) -and
          ($routeNameExit -eq 0) -and
          $routeFileSetupOk
        )
        if (-not $routeSetupOk) {
          Write-Host '[SelfTest] FAIL E2E: production Claude-author route scratch setup failed'
          $ok = $false
        } else {
          $routeSavedFallback = [Environment]::GetEnvironmentVariable('CROSS_REVIEW_FALLBACK')
          $routeSavedFallbackWasSet = $null -ne $routeSavedFallback
          try {
            Set-ProcessEnvironmentValueVerified -Name 'CROSS_REVIEW_FALLBACK' -Value 'codex-no-claude'
            Set-ProcessEnvironmentValueVerified -Name 'GIT_CONFIG_NOSYSTEM' -Value '1'
            Push-Location -LiteralPath $guardScratch
            try {
              $routeOut = (& powershell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -m 'route probe' 2>&1 | ForEach-Object { $_.ToString() } | Out-String)
              $routeExit = $LASTEXITCODE
            } finally { Pop-Location }
          } finally {
            try {
              if ($routeSavedFallbackWasSet) {
                Set-ProcessEnvironmentValueVerified -Name 'CROSS_REVIEW_FALLBACK' -Value $routeSavedFallback
              } else {
                Clear-ProcessEnvironmentValueVerified -Name 'CROSS_REVIEW_FALLBACK'
              }
            } finally {
              Clear-ProcessEnvironmentValueVerified -Name 'GIT_CONFIG_NOSYSTEM'
            }
          }
          $routeRecord = if (Test-Path -LiteralPath $routeRecordPath -PathType Leaf) {
            (Get-Content -LiteralPath $routeRecordPath -Raw).Trim()
          } else {
            ''
          }
          $routeOk = (($routeExit -eq 1) -and ($routeRecord -ceq 'codex|||'))
          if ($routeOk) {
            Write-Host '[SelfTest] PASS E2E: production Claude-author route clears inherited fallback state at the hook boundary'
          } else {
            $routeDiagnostic = ($routeOut -replace '\s+', ' ').Trim()
            Write-Host "[SelfTest] FAIL E2E: production Claude-author route boundary (exit=$routeExit record=[$routeRecord] output=[$routeDiagnostic])"
          }
          $ok = $routeOk -and $ok
        }
      }
    }
  } finally {
    try {
      foreach ($n in $gitConfigEnvNames) {
        try {
          if ($e2eSavedGitCfg.ContainsKey($n) -and $null -ne $e2eSavedGitCfg[$n]) {
            Set-ProcessEnvironmentValueVerified -Name $n -Value $e2eSavedGitCfg[$n]
          } else {
            Clear-ProcessEnvironmentValueVerified -Name $n
          }
        } catch {
          Write-Host "[SelfTest] FAIL E2E: could not restore $n ($($_.Exception.Message))"
          $ok = $false
        }
      }
    } finally {
      if (Test-Path -LiteralPath $guardScratch) {
        Get-ChildItem -LiteralPath $guardScratch -Recurse -File -Force -ErrorAction SilentlyContinue | ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force -ErrorAction SilentlyContinue }
        Get-ChildItem -LiteralPath $guardScratch -Recurse -Directory -Force -ErrorAction SilentlyContinue | Sort-Object { $_.FullName.Length } -Descending | ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force -ErrorAction SilentlyContinue }
        Remove-Item -LiteralPath $guardScratch -Force -ErrorAction SilentlyContinue
      }
    }
  }

  if ($ok) { Write-Host '[SelfTest] All commit-wrapper fixtures passed.'; exit 0 } else { exit 1 }
}

$prevBackend = [Environment]::GetEnvironmentVariable('REVIEW_BACKEND')
$prevBackendWasSet = $null -ne $prevBackend
$prevFallback = [Environment]::GetEnvironmentVariable('CROSS_REVIEW_FALLBACK')
$prevFallbackWasSet = $null -ne $prevFallback

# Capture the hook-affecting git config env vars ($gitConfigEnvNames, defined near
# the top and shared with the --self-test E2E) before invoking git commit. See
# scripts/codex/commit.ps1 for the full rationale (historical regression on the
# GIT_CONFIG_* core.hooksPath bypass vector). Mutation begins inside the
# restore-covered try below.
$savedGitConfigEnv = @{}
foreach ($n in $gitConfigEnvNames) {
  $savedGitConfigEnv[$n] = [Environment]::GetEnvironmentVariable($n)
}

function Enable-GitUnixToolPathForHook {
  # Git for Windows can hand POSIX hooks a duplicate PATH surface where the
  # stale uppercase entry wins over PowerShell's Path edits. Normalize BOTH
  # spellings to one process PATH (via Get-UnionProcessPathParts below) and prepend
  # Git's Unix tools so the dispatcher can resolve mktemp/env/find/rm under
  # PowerShell-launched commits.
  $candidates = @()
  $gitCmd = Get-Command git -ErrorAction SilentlyContinue
  if ($gitCmd -and -not [string]::IsNullOrWhiteSpace($gitCmd.Source)) {
    $gitExeDir = Split-Path -Parent $gitCmd.Source
    $gitRoot = Split-Path -Parent $gitExeDir
    if (-not [string]::IsNullOrWhiteSpace($gitRoot)) {
      $candidates += (Join-Path $gitRoot 'usr\bin')
      $candidates += (Join-Path $gitRoot 'bin')
    }
  }
  if (-not [string]::IsNullOrWhiteSpace($env:ProgramFiles)) {
    $candidates += (Join-Path $env:ProgramFiles 'Git\usr\bin')
    $candidates += (Join-Path $env:ProgramFiles 'Git\bin')
  }

  # Existence-filter the discovered candidates (I/O), then assemble the deduped-
  # prepended parts via the pure, SelfTest-covered Get-UnionProcessPathParts, fed by
  # Get-ProcessPathSpellings (which ENUMERATES the raw process block so a duplicate
  # Path/PATH surface with differing entries does not silently drop one side's dirs).
  $existingCandidates = @($candidates | Where-Object { -not [string]::IsNullOrWhiteSpace($_) -and (Test-Path -LiteralPath $_ -PathType Container) })
  $pathParts = Get-UnionProcessPathParts -Spellings (Get-ProcessPathSpellings) -CandidateDirs $existingCandidates
  # Side-effect only (mutates Env:PATH); the caller captures the ORIGINAL PATH
  # BEFORE invoking this -- inside its restore-covered try -- and restores it in
  # the finally. The Unix-tool prepend is only needed WHILE the hook runs; leaving
  # it mutated would leak into a dot-sourced / persistent-session caller, same
  # class as the GIT_CONFIG_*/REVIEW_BACKEND restore. An empty $pathParts means
  # nothing needs prepending, so PATH is left as-is. Mirrors scripts/codex/commit.ps1
  # and auto-merge.ps1's Normalize-ProcessPathEnvForStartProcess (caller owns save/restore).
  if ($pathParts.Count -eq 0) { return }

  Remove-Item -LiteralPath Env:Path -ErrorAction SilentlyContinue
  Remove-Item -LiteralPath Env:PATH -ErrorAction SilentlyContinue
  # -ErrorAction Stop: the Set is the REQUIRED, fail-closed op. The wrappers run
  # under the default ErrorActionPreference ('Continue'), so WITHOUT -Stop a
  # provider failure is NON-terminating -- the caller would proceed into `git
  # commit` with a cleared/unprepared PATH. -Stop makes it throw so the caller's
  # restore-covered try/catch aborts. Then verify the var actually TOOK (a
  # locked-down provider can report success yet no-op silently -- the same guard
  # the GIT_CONFIG_* scrub uses above). (The two Removes stay SilentlyContinue:
  # the second is an EXPECTED no-op once the first clears the single case-
  # insensitive Env:PATH item, and a failed first clear is corrected by the Set.)
  $joinedPath = [string]::Join([System.IO.Path]::PathSeparator, $pathParts)
  Set-ProcessEnvironmentValueVerified -Name 'PATH' -Value $joinedPath
}

# Capture the ORIGINAL process PATH BEFORE the try so the finally can restore it
# even if Enable-GitUnixToolPathForHook throws MID-mutation (a partial Env:PATH
# edit would otherwise leak into the caller). Enable-... is invoked INSIDE the try
# below -- NOT here -- so a PATH-prep or git-config scrub failure runs the restore
# finally and cannot leak partial process-environment mutation. Mirrors auto-merge.ps1's
# save-before-try discipline around Normalize-ProcessPathEnvForStartProcess. The
# capture UNIONS every PATH spelling from the raw process block (Get-ProcessPathSpellings)
# so the finally restores every dir even when the process carried a duplicate
# Path/PATH surface with differing entries (restoring one spelling would drop the
# other's dirs).
$savedProcessPath = (Get-UnionProcessPathParts -Spellings (Get-ProcessPathSpellings)) -join [System.IO.Path]::PathSeparator

try {
  foreach ($n in $gitConfigEnvNames) {
    if ($null -ne $savedGitConfigEnv[$n]) {
      try {
        Clear-ProcessEnvironmentValueVerified -Name $n
      } catch {
        throw "failed to scrub hook-bypass env var $n ($($_.Exception.Message))"
      }
    }
  }
  Set-ProcessEnvironmentValueVerified -Name 'REVIEW_BACKEND' -Value 'codex'
  Clear-ProcessEnvironmentValueVerified -Name 'CROSS_REVIEW_FALLBACK'

  # Prepare the Git Unix-tool PATH for the pre-commit dispatcher, INSIDE this
  # restore-covered try: a throw here runs the finally (restoring GIT_CONFIG_* +
  # REVIEW_BACKEND + PATH) before the abort propagates via the catch below.
  Enable-GitUnixToolPathForHook

  # PRIMARY config-redirect guard: validate the EFFECTIVE core.hooksPath.
  # The env scrub above covers the GIT_CONFIG_* family, but git also resolves
  # config through HOME / XDG_CONFIG_HOME (which cannot be scrubbed without
  # breaking commit identity). `git config --get core.hooksPath` reads the
  # effective value honoring EVERY config source the upcoming `git commit`
  # will use, so it catches a redirect from any of them. This gate requires
  # core.hooksPath UNSET so the per-clone `.git/hooks/pre-commit` dispatcher
  # fires; ANY set value can redirect the hook to a no-op dir and bypass
  # review (while REVIEW_BACKEND looks correct and no CROSS_REVIEW_SKIP audit
  # entry is written). Refuse to commit if it is set -- one check closes the
  # whole config-redirect class. core.hooksPath lives in shared repo config,
  # so this passes identically in the main checkout and linked worktrees
  # (both leave it unset). Symmetric with scripts/codex/commit.ps1.
  #
  # This guard runs INSIDE the try so the finally below ALWAYS restores the
  # scrubbed GIT_CONFIG_* env + REVIEW_BACKEND even when the guard ABORTS:
  # PowerShell runs a finally block on an `exit` inside its try, so an abort
  # here cannot leak the already-scrubbed git-config env into the caller's
  # shell. (Placing the guard before the try -- outside the finally's reach --
  # would exit past the restoration on a set core.hooksPath.)
  $effectiveHooksPath = (& git config --get core.hooksPath 2>$null)
  $hooksPathExit = $LASTEXITCODE
  # Exit-status decides this, NOT stdout: the mapping lives in the shared,
  # SelfTest-covered Resolve-HooksPathGuard (exit 1 = unset -> proceed; exit 0
  # = set to ANY value incl. an empty `core.hooksPath=` -> abort; other = git
  # error -> fail closed). A stdout-only check (IsNullOrWhiteSpace) would let a
  # set-empty value (exit 0, empty stdout) slip through.
  switch (Resolve-HooksPathGuard -ExitCode $hooksPathExit) {
    'abort-set' {
      Write-Error "scripts/claude/commit.ps1: ABORT -- core.hooksPath is SET (value: '$effectiveHooksPath'); this gate requires it UNSET so the .git/hooks/pre-commit dispatcher fires. A set value (incl. empty, or one from an alternate global/system config via HOME/XDG_CONFIG_HOME or GIT_CONFIG_*) can redirect the pre-commit hook to a no-op dir and bypass review. Unset it (git config --unset core.hooksPath) or investigate the config source."
      exit 1
    }
    'abort-indeterminate' {
      Write-Error "scripts/claude/commit.ps1: ABORT -- 'git config --get core.hooksPath' exited $hooksPathExit (neither 0=set nor 1=unset); failing closed rather than committing with an indeterminate hook-path state."
      exit 1
    }
  }
  & git commit @args
  $gitExit = $LASTEXITCODE
} catch {
  # Routing or PATH preparation failures reach here; the finally below restores
  # the environment before the commit aborts.
  Write-Error "scripts/claude/commit.ps1: ABORT -- failed to prepare the guarded commit environment ($($_.Exception.Message))."
  $gitExit = 1
} finally {
  if ($prevBackendWasSet) {
    try { Set-ProcessEnvironmentValueVerified -Name 'REVIEW_BACKEND' -Value $prevBackend }
    catch { Write-Warning "scripts/claude/commit.ps1: failed to restore REVIEW_BACKEND='$prevBackend' ($($_.Exception.Message))."; $gitExit = 1 }
  } else {
    try { Clear-ProcessEnvironmentValueVerified -Name 'REVIEW_BACKEND' }
    catch { Write-Warning "scripts/claude/commit.ps1: failed to clear REVIEW_BACKEND ($($_.Exception.Message))."; $gitExit = 1 }
  }
  if ($prevFallbackWasSet) {
    try { Set-ProcessEnvironmentValueVerified -Name 'CROSS_REVIEW_FALLBACK' -Value $prevFallback }
    catch { Write-Warning "scripts/claude/commit.ps1: failed to restore CROSS_REVIEW_FALLBACK='$prevFallback' ($($_.Exception.Message))."; $gitExit = 1 }
  } else {
    try { Clear-ProcessEnvironmentValueVerified -Name 'CROSS_REVIEW_FALLBACK' }
    catch { Write-Warning "scripts/claude/commit.ps1: failed to clear CROSS_REVIEW_FALLBACK ($($_.Exception.Message))."; $gitExit = 1 }
  }
  foreach ($n in $gitConfigEnvNames) {
    if ($null -ne $savedGitConfigEnv[$n]) {
      try { Set-ProcessEnvironmentValueVerified -Name $n -Value $savedGitConfigEnv[$n] }
      catch { Write-Warning "scripts/claude/commit.ps1: failed to restore $n ($($_.Exception.Message))."; $gitExit = 1 }
    }
  }
  # Restore the pre-Enable PATH (the Unix-tool prepend was only for the hook).
  # Skip on an empty original (a capture failure, not a real empty PATH). Mirrors
  # scripts/codex/commit.ps1.
  if (-not [string]::IsNullOrEmpty($savedProcessPath)) {
    try {
      Remove-Item -LiteralPath Env:Path -ErrorAction SilentlyContinue
      Remove-Item -LiteralPath Env:PATH -ErrorAction SilentlyContinue
      # -ErrorAction Stop so a provider failure reaches the catch below and the
      # restore warning actually fires (default 'Continue' would swallow it).
      Set-ProcessEnvironmentValueVerified -Name 'PATH' -Value $savedProcessPath
    } catch { Write-Warning "scripts/claude/commit.ps1: failed to restore PATH after the commit ($($_.Exception.Message)); later commands in this shell may retain the Git-Unix-tool-prepended PATH."; $gitExit = 1 }
  }
}
exit $gitExit
