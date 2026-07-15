# Merge-gate wrapper: run an adversarial branch review against the target base,
# only ff-merge the branch into the base when the review surfaces no BLOCKER
# findings (2026-07 severity contract: CLEAN, NOTE-only, and QUALITY-only
# verdicts all pass).
#
# Required for any merge to the integration base branch per the installed
# AGENTS.md. A bare `git merge` to that base is a workflow violation - the
# branch must pass an adversarial review pass first. The base defaults to the
# repo's default branch (origin/HEAD, fallback 'main') unless -Base is given.
#
# Security note: this wrapper NEVER checks out the candidate branch before
# review. The review runs from the base branch's checkout, using the base
# branch's wrapper and prompt template. The candidate branch's tip is named
# via -Tip so codex inspects branch-tip content via `git show <tip>:<path>`
# without that content ever executing. This prevents a candidate branch from
# weakening the review wrapper, prompt, or hook and having the weakened
# version decide whether the branch may merge.
#
# Exit codes (2026-07 severity contract: only BLOCKER aborts a merge; QUALITY is
# a non-blocking follow-up):
#   0 = review passed (CLEAN, NOTE-only, OR QUALITY-only; the wrappers exit 0 for
#       all three -- QUALITY findings are staged by each child wrapper and promoted
#       to logs/review-followups.md by THIS gate after the ff-merge
#       succeeds, for batch triage, not blocked), merge applied
#   1 = RETIRED (was: QUALITY findings blocked the merge). QUALITY no longer blocks
#       a merge, so this gate never returns 1; a stray exit 1 from a backend is an
#       unexpected code and fails the merge closed (exit 3).
#   2 = review surfaced BLOCKER findings WITH a present `VERDICT:` line (even a
#       wrong/malformed-word one), merge NOT applied
#   3 = invocation failure (review error, dirty tree, branch not found,
#       malformed verdict -- INCLUDING a backend verdict with BLOCKER entries but
#       a missing/duplicate `VERDICT:` line, which classifies exit 3 and aborts
#       the merge as malformed, not exit 2 -- etc.)
#
# Cross-provider merge gate:
#   By default the merge gate runs CODEX passes AND ONE CLAUDE pass
#   CONCURRENTLY, then unions the two backends' results (cross-provider
#   diversity catches findings one backend misses). The codex wrapper
#   parallelizes its own passes internally. Total independent coverage is
#   3 passes per merge in every configuration (see Resolve-MergePassMix):
#     claude ON  -> 2 codex + 1 claude
#     claude OFF -> 3 codex
#   -NoClaude disables the claude pass (use when the Claude token pool is
#   exhausted -- main sessions share it; Codex's pool is lightly used). A
#   claude pass that exits 3 ABORTS the merge (fail closed): exit 3 is the
#   Claude wrapper's GENERAL fail-closed code (configured consistency doc
#   unsupported, ancestor CLAUDE.md trust breach, auth failure, malformed/empty
#   output), not only quota, and the merge gate cannot tell those apart from the exit code -- so
#   it never substitutes a codex pass for a claude exit 3. When the pool is
#   genuinely exhausted, re-run with -NoClaude (the pass mix keeps coverage 3).
#
# Same-content dedup pass-reduction: REMOVED in a prior version.
#   An earlier version credited a recent passing COMMIT-gate Staged
#   verdict as one of the three merge passes (dropping one codex pass) when its
#   DIFF-SHA256/REVIEW-TREE-OID/REVIEW-BACKEND/REVIEW-EFFORT headers matched. The
#   merge gate found this FORGEABLE (BLOCKER): verdict artifacts live in
#   `logs/<backend>/reviews/`, which is gitignored and workspace-writable, so any
#   local agent could plant a `*-staged.md` with the computed headers and shave a
#   real merge pass while the gate reported three. No LOCAL artifact is
#   unforgeable in this trust model (git notes etc. are equally local-writable),
#   and concurrency already removed dedup's wall-clock value (the codex wrapper
#   runs its passes in parallel). So the pass-reduction was removed entirely. The
#   merge gate always runs the full pass count. The identity header block stays
#   in both wrappers as cheap provenance/forensics metadata (the exit-1 QUALITY
#   corroboration that had read them was itself removed with the 2026-07 severity
#   contract -- QUALITY no longer blocks a merge, so there is no exit-1 merge to
#   corroborate). DO NOT reintroduce dedup pass-reduction without an UNFORGEABLE
#   receipt mechanism.
#
# Usage:
#   scripts\codex\auto-merge.ps1 -Branch claude-work-feature
#   scripts\codex\auto-merge.ps1 -Branch claude-work-feature -Base main
#   scripts\codex\auto-merge.ps1 -Branch claude-work-feature -NoClaude
#   scripts\codex\auto-merge.ps1 -Branch claude-work-feature -ReasoningEffort high
#   scripts\codex\auto-merge.ps1 -SelfTest   # helper fixtures (no codex/network; the TCK-Git*, EB-Integration, and EB-OriginHead fixtures shell out to git against isolated throwaway TEMP repos -- EB-Integration drives this script end-to-end with an omitted -Base, EB-OriginHead probes origin/HEAD resolution); spawns short-lived local powershell children for the Start-ReviewChild end-to-end cases

[CmdletBinding()]
param(
  # Not [Mandatory] so -SelfTest can run without it. Validated at runtime
  # below (outside SelfTest mode).
  [string]$Branch = '',

  # Project-agnostic: this package ships WITHOUT a hard-coded integration
  # branch. When -Base is omitted it is resolved at runtime from the remote
  # default branch (origin/HEAD), falling back to 'main' -- see
  # Resolve-DefaultBaseBranch below. A concrete install may pass -Base
  # explicitly or pin it in the installed AGENTS.md.
  [string]$Base = '',

  # Disable the cross-provider Claude merge pass (codex-only merge gate).
  # Use when the Claude subscription token pool is exhausted (main sessions
  # share it). The pass-mix resolver keeps total independent coverage at 3.
  [switch]$NoClaude,

  # Codex reasoning-effort override. The merge gate resolves its own EFFECTIVE
  # effort (Resolve-EffectiveCodexEffort: this value when non-empty, else the
  # top-level ~/.codex/config.toml model_reasoning_effort, else `unknown`) and,
  # when a known tier, forwards THAT resolved value to the branch-review codex
  # wrapper's -ReasoningEffort -- pinning the branch review to that exact tier
  # (deterministic, stamped as REVIEW-EFFORT) rather than letting the child
  # re-read config. An empty value here therefore still pins the child to the
  # config-derived tier. The Claude wrapper has no effort knob (it pins
  # CLAUDE_CODE_EFFORT_LEVEL=max internally), so this is codex-only.
  [ValidateSet('', 'minimal', 'low', 'medium', 'high', 'xhigh')]
  [string]$ReasoningEffort = '',

  # Run the helper fixture tests and exit. The fixture block below is the
  # inventory and the success banner the run prints names each group (do not
  # duplicate a helper list here -- it only re-stales on additions). Profile:
  # most are pure in-memory fixtures; a few read small temp files; the
  # Start-ReviewChild end-to-end cases additionally WRITE temp files under
  # $env:TEMP and LAUNCH short-lived local `powershell.exe` children (cleaned up
  # after each case); the TCK-Git* and EB-* fixtures additionally shell out to
  # `git` against isolated throwaway TEMP repos (EB-Integration also invokes this
  # script end-to-end with an omitted -Base). NO codex, NO network -- same SelfTest
  # discipline as auto-review.ps1, but not strictly side-effect-free.
  [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'

# Force UTF-8 for native command pipes and stdin/stdout. Default
# $OutputEncoding is ASCII on Windows PowerShell 5.1; `git show` /
# `git diff` output captured via `&` is round-tripped through the
# platform default codepage, mojibake-ing UTF-8 multi-byte glyphs
# (em-dashes, smart quotes) before this script writes the trusted-
# infra copies. Same defect class as the verdict + prompt reads in
# auto-review.ps1 (fixed in a prior round). Pin explicitly here too.
$OutputEncoding = [System.Text.UTF8Encoding]::new($false)
[Console]::InputEncoding  = [System.Text.UTF8Encoding]::new($false)
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)

# ---------------------------------------------------------------------------
# Helpers (SelfTest-covered). No codex/network. MOST are pure in-memory
# functions; the I/O exceptions in THIS section are clearly marked in their own
# headers -- Start-ReviewChild (writes child-args/exit/runner files + launches a
# local powershell child), Resolve-ReviewChildExit / Test-ChildExitConsistency
# (read a file / a process handle), and Get-GitObjectKind (shells out to `git
# cat-file -t`; its TCK-Git* fixture builds an isolated throwaway TEMP repo).
# None of these touch codex/network. The git-probing copy/centralization helpers
# (Resolve-CentralLogDir, Copy-RunArtifactsToCentralLogs) live in the separate
# "Non-pure runtime helpers" section below; each per-function header states PURE
# vs NON-pure explicitly.
# ---------------------------------------------------------------------------

# Resolve the EFFECTIVE codex reasoning-effort. BYTE-IDENTICAL to the codex
# wrapper's Resolve-EffectiveCodexEffort (scripts/codex/auto-review.ps1) so the
# merge gate pins the branch review to the SAME tier it resolves here. `unknown`
# means "do not pin" (the child inherits codex's own default). PURE +
# SelfTest-covered.
function Resolve-EffectiveCodexEffort {
  param([string]$ExplicitEffort, [string]$ConfigText)
  # Canonical codex reasoning-effort tiers. Any resolved value outside this set
  # (a typo in config, a future tier this gate does not know) collapses to
  # `unknown`, which means the child is not pinned (inherits codex default). This
  # MUST mirror the codex wrapper's -ReasoningEffort ValidateSet (minus the empty
  # string, which means "no explicit override" here, handled before this).
  $validTiers = @('minimal', 'low', 'medium', 'high', 'xhigh')
  if (-not [string]::IsNullOrWhiteSpace($ExplicitEffort)) {
    $e = $ExplicitEffort.Trim().ToLowerInvariant()
    if ($validTiers -contains $e) { return $e } else { return 'unknown' }
  }
  if ([string]::IsNullOrWhiteSpace($ConfigText)) { return 'unknown' }
  # Only consider the top-level table: cut the text at the first TOML section
  # header line (`[...]`). A key inside a profile/table is not the global default.
  $topLevel = $ConfigText
  $secMatch = [regex]::Match($ConfigText, '(?m)^\s*\[')
  if ($secMatch.Success) { $topLevel = $ConfigText.Substring(0, $secMatch.Index) }
  # Accept the documented TOML value forms: double-quoted ("xhigh"), single-
  # quoted ('xhigh'), or bare (xhigh), each with BALANCED quotes, plus optional
  # trailing whitespace and an optional inline `# comment`. The bare alternative's
  # [A-Za-z]+ cannot start on a `"`/`'`, so a MISMATCHED quote (e.g. `"xhigh` with
  # no closing quote) matches nothing here -> `unknown` (fail safe; do not pin).
  $m = [regex]::Match($topLevel, '(?m)^\s*model_reasoning_effort\s*=\s*(?:"(?<v>[A-Za-z]+)"|''(?<v>[A-Za-z]+)''|(?<v>[A-Za-z]+))\s*(?:#.*)?$')
  if (-not $m.Success) { return 'unknown' }
  $v = $m.Groups['v'].Value.Trim().ToLowerInvariant()
  if ($validTiers -contains $v) { return $v } else { return 'unknown' }
}

# Read ~/.codex/config.toml text for Resolve-EffectiveCodexEffort. NON-pure; any
# failure returns '' so the resolver yields `unknown`. BYTE-IDENTICAL to the
# codex wrapper's Get-CodexConfigText.
function Get-CodexConfigText {
  # Join-Path is INSIDE the try: if $env:USERPROFILE is unset/empty (malformed
  # environment), Join-Path throws under $ErrorActionPreference='Stop' and would
  # abort the review over a config-effort lookup that is meant to fail SAFE. Any
  # failure (bad USERPROFILE, missing file, read error) returns '' so the
  # resolver yields `unknown` (do-not-pin), never a crash.
  try {
    $cfg = Join-Path $env:USERPROFILE '.codex\config.toml'
    if (Test-Path -LiteralPath $cfg -PathType Leaf) {
      return [System.IO.File]::ReadAllText($cfg, [System.Text.UTF8Encoding]::new($false))
    }
  } catch { }
  return ''
}

# Full verdict classifier (inlined from scripts/codex/auto-review.ps1
# Get-VerdictExitCode -- byte-identical contract). PURE. Kept in parity across the
# three gate scripts (the GV-* fixtures lock it) so the verdict contract classifies
# identically wherever it runs. NOTE: the merge gate's runtime decision reads the
# child WRAPPER exit codes directly (Resolve-ReviewChildExit / the switch below),
# so after the 2026-07 severity contract removed the exit-1 QUALITY corroboration
# this copy is exercised only by the GV-* SelfTest fixtures -- it is retained for
# contract parity and future reuse, not called on the merge path.
# Exit codes (2026-07 contract: only BLOCKER/malformed abort): 0 = CLEAN, NOTE-only,
# OR QUALITY-only (PASS -- QUALITY is non-blocking), 2 = BLOCKER WITH a present
# `VERDICT:` line (even a wrong one), 3 = malformed / inconsistent / empty / a
# BLOCKER verdict with a missing or duplicate `VERDICT:` line (fail closed). Exit 1
# (was QUALITY/legacy) is retired.
function Get-VerdictExitCode {
  param([string]$Verdict)

  $base = @{
    ExitCode      = 3
    VerdictText   = ''
    Diagnostic    = ''
    BlockerCount  = 0
    QualityCount  = 0
    LegacyNbCount = 0
    NoteCount     = 0
  }

  if ([string]::IsNullOrWhiteSpace($Verdict)) {
    $base.Diagnostic = 'verdict is empty'
    return $base
  }

  $base.BlockerCount  = ([regex]::Matches($Verdict, '(?m)^BLOCKER:')).Count
  $base.QualityCount  = ([regex]::Matches($Verdict, '(?m)^QUALITY:')).Count
  $base.LegacyNbCount = ([regex]::Matches($Verdict, '(?m)^NON-BLOCKER:')).Count
  $base.NoteCount     = ([regex]::Matches($Verdict, '(?m)^NOTE:')).Count

  # Reject duplicate VERDICT lines as malformed; capture the single (or zero)
  # VERDICT line VERBATIM into VerdictText. This MIRRORS the authoritative
  # scripts/codex/auto-review.ps1 Get-VerdictExitCode EXACTLY: it does NOT
  # validate the verdict WORD up front -- a malformed word (e.g. `VERDICT: FOO`)
  # is captured as-is and only rejected later by each per-severity branch's
  # literal `-eq` comparison, so BLOCKER precedence (below) wins over a PRESENT-but-
  # wrong/malformed verdict word (exit 2). A MISSING verdict line (zero `VERDICT:`
  # lines -> VerdictText empty) is the exception: the BLOCKER branch fails closed
  # (exit 3) there, since a verdict with no VERDICT: line is malformed output, not a
  # trustworthy blocker signal. (A prior copy validated the word against
  # `(CLEAN|NON-BLOCKING|BLOCKED)` BEFORE category + BLOCKER and capped category
  # counts at 4096; both diverged from the wrapper. Kept behaviorally identical
  # here, locked by the GV-* fixtures.)
  $verdictLines = @($Verdict -split "`n" | Where-Object { $_ -match '^VERDICT:' })
  if ($verdictLines.Count -gt 1) {
    $base.VerdictText = $verdictLines[0].Trim()
    $base.Diagnostic = "verdict contains $($verdictLines.Count) VERDICT: lines (expected exactly 1)"
    return $base
  }
  $base.VerdictText = if ($verdictLines.Count -eq 1) { $verdictLines[0].Trim() } else { '' }

  # Per-category enumeration: each of the 8 named categories exactly once,
  # count bounded at 10000 (same bound as the wrapper), per-category sum ==
  # per-severity sum. Category validation runs BEFORE BLOCKER precedence so a
  # malformed category block fails closed rather than being read as a blocker.
  $requiredCategories = @(
    'PLAN-DRIFT', 'SILENT-FAILURE', 'TOMBSTONE-OR-SHIM',
    'CROSS-CRATE-CONTRACT', 'LOADER-OR-ASSET-EDGE',
    'CONVENTION-ADHERENCE', 'TEST-QUALITY', 'DOC-VS-CODE-DRIFT'
  )
  $categoryTotal = 0
  foreach ($cat in $requiredCategories) {
    $rx = [regex]("(?m)^" + [regex]::Escape($cat) + ":\s*(?<v>none|\d+)\s*$")
    $matches = $rx.Matches($Verdict)
    if ($matches.Count -ne 1) {
      $base.Diagnostic = "category '$cat' appears $($matches.Count) times (expected exactly 1)"
      return $base
    }
    $v = $matches[0].Groups['v'].Value
    if ($v -ne 'none') {
      $parsed = 0
      if (-not [int]::TryParse($v, [ref]$parsed) -or $parsed -gt 10000) {
        $base.Diagnostic = "category '$cat' has count '$v' that is non-numeric or exceeds 10000"
        return $base
      }
      $categoryTotal += $parsed
    }
  }
  $severityTotal = $base.BlockerCount + $base.QualityCount + $base.LegacyNbCount + $base.NoteCount
  if ($categoryTotal -ne $severityTotal) {
    $base.Diagnostic = "per-category sum ($categoryTotal) does not match per-severity sum ($severityTotal); verdict is malformed"
    return $base
  }

  # BLOCKER precedence -- but a MISSING VERDICT: line is malformed output, not a
  # blocker verdict. Distinguish the two cases per the prompt's precedence note
  # and the AGENTS.md verdict contract:
  #   * ZERO VERDICT: lines (VerdictText == '') -> malformed output -> exit 3
  #     (fail closed; the reviewer never emitted a verdict line at all).
  #   * a VERDICT: line is PRESENT but inconsistent/malformed-word + BLOCKER
  #     -> exit 2 (BLOCKER precedence over a wrong line; the more conservative
  #     signal wins -- a blocker WAS reported).
  if ($base.BlockerCount -gt 0) {
    if ($base.VerdictText -eq '') {
      $base.ExitCode = 3
      $base.Diagnostic = 'BLOCKER findings but no VERDICT: line (missing verdict is malformed output)'
      return $base
    }
    $base.ExitCode = 2
    return $base
  }
  # QUALITY (or legacy NON-BLOCKER) without BLOCKER -> exit 0 (PASS, non-blocking)
  # under the 2026-07 severity contract, in parity with the auto-review.ps1
  # classifiers. A wrong/missing verdict line is still malformed -> exit 3 (default).
  if ($base.QualityCount -gt 0 -or $base.LegacyNbCount -gt 0) {
    if ($base.VerdictText -ne 'VERDICT: NON-BLOCKING') {
      $base.Diagnostic = "QUALITY/legacy findings but verdict is '$($base.VerdictText)' (expected 'VERDICT: NON-BLOCKING')"
      return $base
    }
    $base.ExitCode = 0
    return $base
  }
  if ($base.NoteCount -gt 0) {
    if ($base.VerdictText -ne 'VERDICT: NON-BLOCKING') {
      $base.Diagnostic = "NOTE-only findings but verdict is '$($base.VerdictText)' (expected 'VERDICT: NON-BLOCKING')"
      return $base
    }
    $base.ExitCode = 0
    return $base
  }
  if ($base.VerdictText -ne 'VERDICT: CLEAN') {
    $base.Diagnostic = "verdict has zero findings but is not 'VERDICT: CLEAN'"
    return $base
  }
  $base.ExitCode = 0
  return $base
}

# Resolve the merge-gate pass mix. Returns @{ Codex=<int>; Claude=<int> }.
# Total independent coverage is always 3:
#   claude ON  -> 2 codex + 1 claude
#   claude OFF -> 3 codex
# (Dedup pass-reduction was removed in a prior version -- see the header note -- so there
# is no longer a DedupHit dimension; the gate always runs the full count.) PURE.
function Resolve-MergePassMix {
  param([bool]$IncludeClaude)
  $claude = if ($IncludeClaude) { 1 } else { 0 }
  $codex = 3 - $claude
  return @{ Codex = $codex; Claude = $claude }
}

# Union the two backend exit codes into the merge gate's decision. PURE +
# SelfTest-covered. Under the 2026-07 severity contract a valid backend verdict is
# 0 (PASS -- incl. non-blocking QUALITY/NOTE) or 2 (BLOCKER). ANY other value -- a
# native crash, a Ctrl-C, a legacy/pre-contract wrapper's RETIRED exit 1, or a future
# code -- is NOT a valid verdict and fails the merge CLOSED (Exit 3), never collapsing
# to a passing union. (exit 3 from a backend is normally caught by the caller's own
# fail-closed branches first; this helper rejects a stray 3 defensively too.) Returns
# @{ Exit = 0|2|3; Diagnostic = <reason or ''> }.
function Resolve-MergeUnionExit {
  param([int]$CodexExit, [int]$ClaudeExit)
  foreach ($pair in @(@{ Name = 'codex'; Code = $CodexExit }, @{ Name = 'claude'; Code = $ClaudeExit })) {
    if ($pair.Code -notin @(0, 2)) {
      return @{ Exit = 3; Diagnostic = "$($pair.Name) review returned unexpected exit code $($pair.Code) (not a valid verdict 0/2; exit 1 was retired with the 2026-07 severity contract, QUALITY now passes as 0)" }
    }
  }
  if ($CodexExit -eq 2 -or $ClaudeExit -eq 2) { return @{ Exit = 2; Diagnostic = '' } }
  return @{ Exit = 0; Diagnostic = '' }
}

# Normalize a detected `origin/HEAD` short-ref (e.g. 'origin/main') to a base
# branch name, defaulting to 'main' when detection yielded nothing. PURE (no
# git), so it is SelfTestable -- the non-pure detection lives in
# Resolve-DefaultBaseBranch. Only the leading `origin/` segment is stripped so
# a slashed default branch ('origin/release/x' -> 'release/x') survives intact.
function Get-NormalizedDefaultBase {
  param([string]$DetectedRef)
  if ([string]::IsNullOrWhiteSpace($DetectedRef)) { return 'main' }
  $name = ($DetectedRef -replace '^origin/', '').Trim()
  if ([string]::IsNullOrWhiteSpace($name)) { return 'main' }
  return $name
}

# Resolve the default base branch for a project-agnostic install: read the
# remote default branch from `origin/HEAD`. Any failure (no remote, HEAD unset,
# git error) yields '' so Get-NormalizedDefaultBase falls back to 'main'. NON-
# pure (shells out to git); callers pass an explicit -Base to bypass it.
function Resolve-DefaultBaseBranch {
  $ref = ''
  try {
    $out = (& git symbolic-ref --short refs/remotes/origin/HEAD 2>$null)
    if ($LASTEXITCODE -eq 0 -and $out) { $ref = ($out | Select-Object -First 1).Trim() }
  } catch { $ref = '' }
  return (Get-NormalizedDefaultBase -DetectedRef $ref)
}

# Decide whether the configured consistency doc is in a branch's
# `git diff --name-status` output. The Claude merge wrapper
# (scripts/claude/auto-review.ps1) FAILS CLOSED (exit 3) whenever the configured
# consistency doc is in the changed-path set -- its Get-PlanConsistencyReport
# precompute is not ported -- so the default cross-provider mix (which launches a
# Claude pass) would ABORT an otherwise-valid doc-touching branch. The merge
# gate detects the doc here and forces the codex-only mix (same counts as
# -NoClaude) so the documented consistency-doc path completes under Codex review
# alone. This mirrors the Claude wrapper's OWN detection EXACTLY: NAME-STATUS
# lines have the shape `<status>\t<path>` (e.g. `M\tPLAN.md`) or
# `R<score>\t<old>\t<new>` for renames, so every tab-separated column AFTER the
# first (status) column is a path column and BOTH rename path columns are checked.
# (Matching by raw diff text would false-positive on this script's own comments
# mentioning the doc.)
#
# $DocName defaults to PLAN.md so the in-isolation PNS SelfTest fixtures pass
# unchanged; the runtime caller passes the NORMALIZED canonical git-path form of
# $env:CROSS_REVIEW_CONSISTENCY_DOC (from Resolve-ConsistencyDocConfig, i.e.
# $cfgMergeDoc.Doc), never the raw env value, and only invokes this when that env
# var resolved to State='valid' (a fresh install forces no codex-only mix; an
# invalid config already aborted the merge upstream). The backslash normalization
# retained below is belt-and-suspenders for any direct caller.
# PURE + SelfTest-covered.
function Test-PlanInNameStatus {
  param(
    [string]$NameStatus,
    [string]$DocName = 'PLAN.md'
  )
  if ([string]::IsNullOrWhiteSpace($NameStatus)) { return $false }
  # Normalize $DocName to git path form: name-status path columns use forward
  # slashes, but on Windows the configured doc (from CROSS_REVIEW_CONSISTENCY_DOC)
  # may carry backslashes (`docs\PLAN.md`). Without this, a backslash value would
  # never equal a `docs/PLAN.md` path column and a doc-touching branch would keep
  # the cross-provider mix instead of the documented codex-only mix.
  $docNorm = $DocName -replace '\\', '/'
  foreach ($line in ($NameStatus -split "`r?`n")) {
    if ([string]::IsNullOrWhiteSpace($line)) { continue }
    $parts = $line -split "`t"
    for ($pi = 1; $pi -lt $parts.Length; $pi++) {
      if ($parts[$pi].Trim() -eq $docNorm) { return $true }
    }
  }
  return $false
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

# Normalize + validate the raw CROSS_REVIEW_CONSISTENCY_DOC env value. The value,
# IF consumed RAW elsewhere, WOULD fail OPEN: a whitespace-only or whitespace-
# PADDED value (e.g. "PLAN.md ") is truthy but never EQUALS a trimmed
# name-status path column, so the consistency gating silently disables while
# review proceeds -- a fail-OPEN in a fail-CLOSED gate. This helper is the single
# normalization+validation point; ALL THREE consistency-doc sites (codex
# auto-review precompute, claude fail-closed routing, auto-merge codex-only-mix
# forcing) call a BYTE-IDENTICAL copy of it (there is no shared module to import,
# so the copies are kept identical by hand and each carries its own SelfTest).
#
# Returns @{ State = 'off' | 'valid' | 'invalid'; Doc = <normalized>; Reason = <string> }:
#   off     - env var UNSET (or $null): the legitimate default; no special-casing.
#   valid   - non-empty after trim, repo-relative (not absolute, no `..` escape);
#             Doc is the trimmed + `\`->`/`-normalized git-path form to match.
#   invalid - SET but whitespace-only / absolute / contains a `..` escape segment.
#             A deliberately-SET-but-invalid value is a BROKEN GATE CONFIG and the
#             caller FAILS CLOSED (never silently disables); Reason explains why.
# PURE (takes the raw value as a param) + SelfTest-covered. NOTE: $RawValue is
# DELIBERATELY untyped (no `[string]`) so a $null (env var UNSET) stays $null and
# is distinguished from an empty string '' (env var SET to empty, an invalid
# config). A `[string]` annotation would coerce $null -> '' and conflate the two.
function Resolve-ConsistencyDocConfig {
  param($RawValue)
  if ($null -eq $RawValue) { return @{ State = 'off'; Doc = ''; Reason = '' } }
  $trimmed = ([string]$RawValue).Trim()
  if ($trimmed.Length -eq 0) {
    return @{ State = 'invalid'; Doc = ''; Reason = 'CROSS_REVIEW_CONSISTENCY_DOC is set to an empty or whitespace-only value' }
  }
  $norm = $trimmed -replace '\\', '/'
  # POSIX-absolute (leading `/`) is detected PRE-collapse: the segment-collapse
  # below drops the leading empty segment, which would otherwise turn `/etc/x`
  # into the relative `etc/x`. A `.`-prefix cannot express a posix-absolute
  # (`.//etc` collapses to `etc`), so a leading `/` on $norm is unambiguously
  # absolute.
  if ($norm.StartsWith('/')) {
    return @{ State = 'invalid'; Doc = ''; Reason = "CROSS_REVIEW_CONSISTENCY_DOC='$norm' is an absolute path; it must be a repo-relative path" }
  }
  # CANONICALIZE to git-name-status form, then validate the canonical result. git
  # name-status / diff headers emit canonical paths (no `.` segments, no redundant
  # `//` or trailing `/`), but a consumer may set `./PLAN.md`, `docs/./PLAN.md`,
  # `PLAN.md/`, etc. -- without canonicalization those reach State=valid yet never
  # equal the exact name-status column, so the gate silently skips (a fail-OPEN).
  # The DRIVE-absolute check runs on the CANONICAL result (AFTER collapse): a
  # value like `./C:/PLAN.md` PASSES a pre-collapse drive check (it starts with
  # `.`), then collapses to the drive-absolute `C:/PLAN.md`, so it must be
  # re-checked post-collapse. Split on `/`, drop empty (`//`, trailing `/`) and
  # `.` segments, reject `..` escapes, rejoin, THEN reject if drive-rooted.
  $segs = New-Object 'System.Collections.Generic.List[string]'
  foreach ($seg in ($norm -split '/')) {
    if ($seg -eq '' -or $seg -eq '.') { continue }   # collapse `//`, trailing `/`, and `.` segments
    if ($seg -eq '..') {
      return @{ State = 'invalid'; Doc = ''; Reason = "CROSS_REVIEW_CONSISTENCY_DOC='$norm' contains a '..' path-escape segment; it must stay within the repo" }
    }
    $segs.Add($seg) | Out-Null
  }
  if ($segs.Count -eq 0) {
    # All segments were `.`/empty (e.g. `./` or `.`): no actual file path.
    return @{ State = 'invalid'; Doc = ''; Reason = "CROSS_REVIEW_CONSISTENCY_DOC='$norm' does not name a file (only '.'/separator segments)" }
  }
  $canon = $segs -join '/'
  if ([System.IO.Path]::IsPathRooted($canon) -or $canon -match '^[A-Za-z]:' -or $canon.StartsWith('/')) {
    return @{ State = 'invalid'; Doc = ''; Reason = "CROSS_REVIEW_CONSISTENCY_DOC='$norm' resolves to the absolute path '$canon'; it must be a repo-relative path" }
  }
  return @{ State = 'valid'; Doc = $canon; Reason = '' }
}

# The configured consistency doc must resolve to a git BLOB (a tracked FILE) in
# the reviewed tree. `git cat-file -e` is NOT enough -- it succeeds for a TREE
# (directory) too, so a directory-valued config (e.g. `docs/`) would pass and
# then silently skip / mis-route. Callers resolve the object KIND (`git cat-file
# -t`, or 'missing' on a nonzero exit) and pass it here; only 'blob' is valid.
# PURE + SelfTest-covered so the contract is testable without a git tree. (Codex
# BLOCKER, merge gate.)
function Test-ConsistencyDocKind {
  param([string]$Kind)
  return ($Kind -eq 'blob')
}

# Resolve the git OBJECT KIND of a repo-relative path within a tree-ish:
# 'blob' (tracked file), 'tree' (directory), 'missing' (path absent / bad ref),
# or the raw `cat-file -t` kind otherwise. Isolated so the `git cat-file -t`
# boundary the consistency-doc blob check rides is integration-testable against a
# real throwaway repo (the TCK-Git* SelfTest fixtures drive it). (Codex
# TEST-QUALITY.)
function Get-GitObjectKind {
  param([string]$TreeRef, [string]$Path)
  # 2>&1 + SilentlyContinue: a missing path makes `git cat-file -t` write to
  # stderr and exit nonzero; capture+discard it (return 'missing') so a typo/dir
  # config resolves cleanly instead of surfacing a NativeCommandError mid-gate.
  $prev = $ErrorActionPreference; $ErrorActionPreference = 'SilentlyContinue'
  try {
    $t = (& git cat-file -t "${TreeRef}:${Path}" 2>&1)
    if ($LASTEXITCODE -ne 0) { return 'missing' }
    return "$t".Trim()
  } finally { $ErrorActionPreference = $prev }
}

# Build a single Windows command-line string from an argument array, quoting
# each element per the CommandLineToArgvW rules so multi-word values (notably
# the multi-word `-Title`) survive the `Start-Process -ArgumentList` boundary.
# `Start-Process -ArgumentList <array>` joins elements with spaces WITHOUT
# quoting, which splits a multi-word value into several args and breaks the
# child wrapper's parameter binding (wrapper startup failure before any
# verdict). Passing ONE pre-quoted string instead preserves argument boundaries.
# PURE + SelfTest-covered.
function Convert-ToProcArgString {
  param([string[]]$ArgList)
  if ($null -eq $ArgList) { return '' }
  $quoted = foreach ($a in $ArgList) {
    if ($null -eq $a) { $a = '' }
    if ($a.Length -gt 0 -and ($a.IndexOfAny([char[]]@(' ', "`t", '"')) -lt 0)) {
      # No space/tab/quote -> emit verbatim.
      $a
    } else {
      # Wrap in double-quotes; escape per CommandLineToArgvW: a run of
      # backslashes immediately before a quote (or the closing quote) is
      # doubled, and each embedded quote is backslash-escaped.
      $sb = [System.Text.StringBuilder]::new()
      [void]$sb.Append('"')
      $backslashes = 0
      foreach ($ch in $a.ToCharArray()) {
        if ($ch -eq '\') {
          $backslashes++
        } elseif ($ch -eq '"') {
          [void]$sb.Append('\' * ($backslashes * 2 + 1))
          [void]$sb.Append('"')
          $backslashes = 0
        } else {
          if ($backslashes -gt 0) { [void]$sb.Append('\' * $backslashes); $backslashes = 0 }
          [void]$sb.Append($ch)
        }
      }
      if ($backslashes -gt 0) { [void]$sb.Append('\' * ($backslashes * 2)) }
      [void]$sb.Append('"')
      $sb.ToString()
    }
  }
  return ($quoted -join ' ')
}

# PURE PATH-assembly helper (shared with the commit wrappers): given
# (existence-filtered) candidate dirs + the current PATH, return the ordered,
# case-insensitively-deduped parts with candidates PREPENDED, then the current
# parts (skipping blanks / already-seen). No env mutation, no I/O -- SelfTest
# drives it. Byte-identical to the copies in scripts/codex/commit.ps1 and
# scripts/claude/commit.ps1; Normalize below passes NO candidates (it only
# collapses the duplicate Path/PATH surface for Start-Process).
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
# Byte-identical to the copies in scripts/codex/commit.ps1 and scripts/claude/commit.ps1.
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
# Byte-identical to the copies in scripts/codex/commit.ps1 and scripts/claude/commit.ps1.
function Get-UnionProcessPathParts {
  param([string[]]$Spellings, [string[]]$CandidateDirs = @())
  $joined = (@($Spellings) | Where-Object { -not [string]::IsNullOrEmpty($_) }) -join [System.IO.Path]::PathSeparator
  return Get-PrependedToolPathParts -CandidateDirs $CandidateDirs -CurrentPath $joined
}

function Normalize-ProcessPathEnvForStartProcess {
  # PowerShell 5.1 can inherit both Path and PATH in the process environment.
  # Start-Process builds a case-insensitive dictionary from that block and
  # throws before launch when both spellings are present. Collapse to one Path
  # entry so child-runner tests and production review launches share the same
  # reliable process boundary.
  # UNION every PATH spelling from the raw process block (Get-ProcessPathSpellings)
  # so a duplicate Path/PATH surface with DIFFERING entries does not silently drop
  # one side's dirs (the exact case this function exists to collapse). No candidates
  # -- dedup only. SelfTest-covered.
  $pathParts = Get-UnionProcessPathParts -Spellings (Get-ProcessPathSpellings)
  if ($pathParts.Count -eq 0) { return }

  Remove-Item -LiteralPath Env:Path -ErrorAction SilentlyContinue
  Remove-Item -LiteralPath Env:PATH -ErrorAction SilentlyContinue
  # -ErrorAction Stop so a provider failure THROWS -> the Start-Process try/catch
  # at the call site fails the merge CLOSED (the default 'Continue' active here
  # would swallow it and launch Start-Process with an un-collapsed Path/PATH).
  Set-Item -LiteralPath Env:Path -Value ([string]::Join([System.IO.Path]::PathSeparator, $pathParts)) -ErrorAction Stop
}

# Launch ONE review-wrapper child and make its exit code DETERMINISTIC to read
# back, closing the PowerShell 5.1 `Start-Process -PassThru` quirk (observed live):
# `.ExitCode` returns $null after WaitForExit() unless the process
# HANDLE was accessed before the process exited. We fix that two ways at once,
# mirroring the codex wrapper's own proven idiom (scripts/codex/auto-review.ps1
# pass-runner: pre-seed an exit file, child writes $LASTEXITCODE into it, parent
# reads the FILE):
#   (1) An exit-FILE is the authoritative source. We pre-seed it with a non-zero
#       '999' sentinel BEFORE launch, so a child that dies before recording its
#       real code still leaves a fail-closed marker (Resolve-ReviewChildExit
#       rejects any value outside {0,1,2,3} as $null -> fail closed). The
#       generated runner re-issues the SAME
#       `powershell.exe -File <wrapper> <args>` invocation the parent used to make
#       directly, captures that child's $LASTEXITCODE, and writes it to the file.
#       Routing the wrapper through `powershell.exe` (an EXE) inside the runner
#       lets the runner SURVIVE the wrapper's `exit N` to record the code -- a
#       direct `& <wrapper.ps1>` would terminate the runner on the wrapper's exit
#       before it could write the file.
#   (2) `$null = $proc.Handle` is cached on the RUNNER process the instant it is
#       spawned, making the in-memory `.ExitCode` reliable. The caller USES that
#       via Test-ChildExitConsistency as a TAMPER CHECK: the exit file is a
#       mutable default-ACL input, so a readable handle .ExitCode that MISMATCHES
#       the file value means the file cannot be trusted -> the merge FAILS CLOSED
#       (the handle is NOT file-backed, so it is the tamper detector). The runner
#       is a thin shim ending in `exit $code` with the same $code it wrote, so the
#       two sources agree under normal operation; only no-readable-handle cases
#       (launch failure / PS .ExitCode quirk) fall back to the file's own
#       {0,1,2,3} fail-closed validation.
# The args are passed via a per-child args FILE (one token per line, UTF-8
# no-BOM), exactly like the codex wrapper's codex-args.txt, so the wrapper's
# multi-word -Title survives without any nested Start-Process arg-quoting. Every
# arg token here is a CLI flag, a fixed token, a %TEMP% path, the branch SHA, the
# ValidateSet effort tier, or the single multi-word -Title -- none can contain a
# newline, so line-delimited round-trips losslessly. Returns a PSCustomObject
# { Proc; ExitFile } the caller waits on then resolves via Resolve-ReviewChildExit.
function Start-ReviewChild {
  param(
    [Parameter(Mandatory = $true)][string]$RunDir,
    [Parameter(Mandatory = $true)][string[]]$WrapperArgs,
    [Parameter(Mandatory = $true)][System.Text.Encoding]$Utf8NoBom
  )
  $argsFile   = Join-Path $RunDir 'child-args.txt'
  $exitFile   = Join-Path $RunDir 'child-exit.txt'
  $runnerPath = Join-Path $RunDir 'child-runner.ps1'

  # Serialize the wrapper argv (one token per line) for the runner to reconstruct.
  [System.IO.File]::WriteAllLines($argsFile, [string[]]$WrapperArgs, $Utf8NoBom)
  # Pre-seed the exit file with a fail-closed sentinel BEFORE launch.
  [System.IO.File]::WriteAllText($exitFile, '999', $Utf8NoBom)

  # ASCII-only runner body (the wrapper file is BOM-less UTF-8 read as ANSI by
  # PS 5.1, so non-ASCII here would mojibake). It reads the argv array, invokes
  # the wrapper as a SEPARATE powershell.exe child (so the wrapper's `exit N`
  # becomes that child's exit code, captured in $LASTEXITCODE without killing the
  # runner), records the code to child-exit.txt, and exits with it.
  $runnerBody = @'
$ErrorActionPreference = 'Continue'
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
$work      = $PSScriptRoot
$argsFile  = Join-Path $work 'child-args.txt'
$exitFile  = Join-Path $work 'child-exit.txt'
$wrapperArgs = [System.IO.File]::ReadAllLines($argsFile, $utf8NoBom)
& powershell.exe @wrapperArgs
$code = $LASTEXITCODE
if ($null -eq $code) { $code = 999 }
[System.IO.File]::WriteAllText($exitFile, ([string]$code), $utf8NoBom)
exit $code
'@
  [System.IO.File]::WriteAllText($runnerPath, $runnerBody, $Utf8NoBom)

  $runnerArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $runnerPath)
  # Catch a Start-Process LAUNCH failure explicitly (and SEPARATELY from handle
  # caching -- see below). The script runs under $ErrorActionPreference='Stop',
  # so a launch throw would otherwise abort this function BEFORE it returns -- the
  # caller's $claudeChild/$codexChild would never be assigned and control would
  # route to the outer catch. Catching it and returning the sentinel-backed object
  # (Proc=$null, ExitFile = the pre-seeded '999' file) keeps the contract uniform:
  # the caller ALWAYS gets an object, Resolve-ReviewChildExit reads the '999'
  # sentinel -> $null -> fail closed. So a failed launch of a REQUIRED child fails
  # the merge closed via the normal resolve path rather than an opaque outer-catch.
  $proc = $null
  # Capture the effective PATH before Normalize collapses the Path/PATH duplicate
  # surface for the Start-Process env-block build; restore it in the finally after
  # the child has inherited the collapsed block at launch, so the mutation does not
  # persist into a dot-sourced / persistent-session caller (same restore discipline
  # as the commit wrappers). The capture UNIONS every PATH spelling from the raw
  # process block (Get-ProcessPathSpellings) and dedups (Normalize adds no foreign
  # dirs), so the restored value carries every dir even across a duplicate surface --
  # the only residual change is the benign single Path spelling, which a Set-Item
  # restore cannot avoid.
  $savedNormalizePath = (Get-UnionProcessPathParts -Spellings (Get-ProcessPathSpellings)) -join [System.IO.Path]::PathSeparator
  try {
    Normalize-ProcessPathEnvForStartProcess
    $proc = Start-Process -FilePath 'powershell.exe' -ArgumentList (Convert-ToProcArgString -ArgList $runnerArgs) -NoNewWindow -PassThru -ErrorAction Stop
  } catch {
    Write-Host "[auto-merge] WARNING: failed to launch the review runner ($($_.Exception.Message)) - the pre-seeded '999' sentinel in '$exitFile' will drive a fail-closed resolution."
    $proc = $null
  } finally {
    if (-not [string]::IsNullOrEmpty($savedNormalizePath)) {
      # -ErrorAction Stop + inner try/catch so a provider failure is DIAGNOSED
      # (matches the commit wrappers' restore path) rather than silently leaving
      # the session PATH modified. The inner catch keeps the throw from escaping
      # this finally (which would otherwise mask any in-flight exception).
      try {
        Remove-Item -LiteralPath Env:Path -ErrorAction SilentlyContinue
        Remove-Item -LiteralPath Env:PATH -ErrorAction SilentlyContinue
        Set-Item -LiteralPath Env:Path -Value $savedNormalizePath -ErrorAction Stop
      } catch { Write-Warning "[auto-merge] failed to restore process PATH after the review launch ($($_.Exception.Message)); later commands in this session may retain the collapsed Path spelling." }
    }
  }
  # Handle caching is a SEPARATE concern with a SEPARATE catch. If the launch
  # SUCCEEDED ($proc not null) but accessing .Handle throws, the child is STILL
  # ALIVE -- it must NOT be nulled out (that would orphan a running process and
  # let the caller skip WaitForExit and let finally delete its scratch dir
  # mid-run). The handle cache is only a belt for the in-memory .ExitCode; the
  # exit FILE is authoritative regardless, so a handle-cache failure is non-fatal:
  # keep $proc, just lose the .ExitCode cross-check (Test-ChildExitConsistency
  # already degrades gracefully when .ExitCode is unreadable).
  if ($null -ne $proc) {
    try { $null = $proc.Handle }
    catch { Write-Host "[auto-merge] WARNING: could not cache the review runner's process handle ($($_.Exception.Message)) - the child is still live (kept); the exit FILE remains authoritative and the .ExitCode cross-check may be skipped." }
  }
  [PSCustomObject]@{ Proc = $proc; ExitFile = $exitFile }
}

# Resolve a review child's exit code from its run-owned exit FILE (authoritative)
# after the caller has WaitForExit()'d. Missing / empty / unparsable file, OR a
# parsed value outside the readable wrapper-code set {0,1,2,3} (e.g. the '999' crash
# sentinel) => $null, which the caller MUST treat as fail-closed (it aborts the
# merge on a $null). $BackendName + the file path are surfaced in the loud
# diagnostic so an operator can inspect the run dir. PURE-ish (reads one file).
function Resolve-ReviewChildExit {
  param(
    [Parameter(Mandatory = $true)][string]$ExitFile,
    [Parameter(Mandatory = $true)][string]$BackendName
  )
  if (-not (Test-Path -LiteralPath $ExitFile)) {
    Write-Host "[auto-merge] $BackendName review exit file MISSING ($ExitFile) - treating as fail-closed."
    return $null
  }
  $raw = ''
  try {
    $raw = ([System.IO.File]::ReadAllText($ExitFile, [System.Text.Encoding]::UTF8)).Trim()
  } catch {
    Write-Host "[auto-merge] $BackendName review exit file UNREADABLE ($ExitFile): $($_.Exception.Message) - treating as fail-closed."
    return $null
  }
  $parsed = 0
  if (-not [int]::TryParse($raw, [ref]$parsed)) {
    Write-Host "[auto-merge] $BackendName review exit file UNPARSABLE (got '$raw' from $ExitFile) - treating as fail-closed."
    return $null
  }
  # Only {0,1,2,3} are readable wrapper codes (0 = PASS incl. non-blocking QUALITY,
  # 2 = BLOCKER, 3 = fail-closed; 1 is the RETIRED QUALITY code -- readback-recognized
  # but rejected as an invalid verdict by the later {0,2} union check). Anything else
  # -- the pre-seeded '999' sentinel left when the
  # runner crashed before recording a real code, or any other garbage -- means
  # the child never produced a verdict. Fail closed loudly, naming the file. The
  # exit-3 pass-through is deliberate: it preserves the wrapper's own
  # fail-closed code so the caller's backend-specific exit-3 handling still fires.
  if ($parsed -notin @(0, 1, 2, 3)) {
    Write-Host "[auto-merge] $BackendName review exit file holds non-verdict code $parsed (from $ExitFile; '999' is the runner's crash sentinel) - treating as fail-closed."
    return $null
  }
  return $parsed
}

# Tamper-resistant cross-check between the exit FILE and the handle-cached process
# .ExitCode (the reason `$null = $proc.Handle` is accessed at spawn). The runner
# is a thin shim ending in `exit $code` with the SAME $code it wrote to the file,
# so under normal operation handle.ExitCode == file value ALWAYS. The exit file
# lives under default-ACL $env:TEMP and is therefore a MUTABLE input: a same-user
# process could overwrite it. The process .ExitCode is NOT file-backed, so it is
# the tamper detector. RETURNS:
#   $true  -- the file value is TRUSTWORTHY: either no readable handle exists to
#             cross-check against (Start-Process failed / .ExitCode unreadable --
#             the file's own {0,1,2,3} fail-closed validation in
#             Resolve-ReviewChildExit still applies), OR the handle value MATCHES
#             the file value.
#   $false -- a READABLE handle value MISMATCHES the (non-null) file value: the
#             file cannot be trusted (tampered, or the runner exited with a
#             different code than it recorded). The caller MUST fail the merge
#             CLOSED rather than union the file value.
# A $null FileExit is not cross-checked (the caller already fails closed on it).
function Test-ChildExitConsistency {
  param(
    [Parameter(Mandatory = $true)][string]$BackendName,
    $FileExit,                                   # int or $null (from Resolve-ReviewChildExit)
    $Proc                                        # System.Diagnostics.Process or $null
  )
  if ($null -eq $Proc) {
    Write-Host "[auto-merge] WARN: $BackendName review process handle is null (Start-Process failed) - cannot cross-check; relying on the exit file's own {0,1,2,3} fail-closed validation (file=$(if ($null -eq $FileExit){'null'}else{$FileExit}))."
    return $true
  }
  $handleExit = $null
  try { $handleExit = $Proc.ExitCode } catch { $handleExit = $null }
  if ($null -eq $handleExit) {
    Write-Host "[auto-merge] WARN: $BackendName review process .ExitCode is unreadable despite the cached handle - cannot cross-check; relying on the exit file's own fail-closed validation (file=$(if ($null -eq $FileExit){'null'}else{$FileExit}))."
    return $true
  }
  # Only cross-check when the file produced a usable verdict code. A $null FileExit
  # already failed closed at the call site, where the raw handle value differing
  # is expected (the file was rejected) -- not a tamper signal.
  if ($null -ne $FileExit -and $handleExit -ne $FileExit) {
    Write-Host "[auto-merge] SECURITY: $BackendName review exit-code MISMATCH (file=$FileExit, handle=$handleExit) - the run-owned exit file disagrees with the live process exit code. The file is a mutable input; this divergence (tamper, or a runner that exited differently than it recorded) means the file value cannot be trusted -> failing the merge CLOSED."
    return $false
  }
  return $true
}

# ---------------------------------------------------------------------------
# Non-pure runtime helpers. Resolve-CentralLogDir git-probes and is NOT SelfTest-
# covered; Copy-RunArtifactsToCentralLogs copies files but its preserve/delete
# RETURN CONTRACT is SelfTest-covered (CRA-* fixtures) via a -CentralDirOverride
# test seam that avoids the git probe. Publish-ReviewFollowups likewise git-probes
# at runtime but its append/header/no-op contract is SelfTest-covered (PRF-*
# fixtures) via an -OrchDirOverride test seam.
# ---------------------------------------------------------------------------

# Promote a child wrapper's STAGED QUALITY follow-ups to the durable
# logs/review-followups.md index. Called ONLY after the ff-merge has
# succeeded (merge-gate BLOCKER 2026-07-10: a child that appended on its own
# pass could record follow-ups for a merge that subsequently aborted on the
# exit union, the branch pin, or the ff-merge -- so the children stage to
# CROSS_REVIEW_FOLLOWUPS_PENDING paths and THIS single post-merge point promotes).
# NON-pure I/O twin of the wrappers' direct-append mode: same main-repo
# resolution (git common-dir parent, two-step probe), same header, same
# exclusive-handle retry against concurrent writers. Any failure warns and
# degrades -- the merge is already applied, so a lost follow-up record must
# not fail the gate. -OrchDirOverride is a SelfTest-only seam that skips the
# git probe.
function Publish-ReviewFollowups {
  param(
    [string]$PendingPath,
    [string]$BackendName,
    [string]$OrchDirOverride
  )
  try {
    if ([string]::IsNullOrWhiteSpace($PendingPath) -or -not (Test-Path -LiteralPath $PendingPath)) { return }
    $block = [System.IO.File]::ReadAllText($PendingPath, [System.Text.Encoding]::UTF8)
    if ([string]::IsNullOrWhiteSpace($block)) { return }
    $enc = New-Object System.Text.UTF8Encoding($false)
    $logsDir = $OrchDirOverride
    if ([string]::IsNullOrWhiteSpace($logsDir)) {
      # Submodule-aware root selection (Get-FollowupIndexDir <- Get-ReviewLogDir):
      # the common-dir PARENT for a normal repo / linked worktree, the working-tree
      # TOP-LEVEL for a submodule (whose common-dir parent is `.git/modules`).
      $commonDir = $null
      $absOut = & git rev-parse --path-format=absolute --git-common-dir 2>$null
      if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($absOut)) {
        $commonDir = ($absOut | Select-Object -First 1).Trim()
      } else {
        $plainOut = & git rev-parse --git-common-dir 2>$null
        if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($plainOut)) {
          try { $commonDir = (Resolve-Path -LiteralPath (($plainOut | Select-Object -First 1).Trim()) -ErrorAction Stop).Path } catch { $commonDir = $null }
        }
      }
      $topLevel = $null
      $topOut = & git rev-parse --show-toplevel 2>$null
      if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($topOut)) {
        $topLevel = ($topOut | Select-Object -First 1).Trim()
      }
      $logsDir = Get-FollowupIndexDir -CommonDir $commonDir -TopLevel $topLevel
      if ([string]::IsNullOrWhiteSpace($logsDir)) { $logsDir = Join-Path (Get-Location).Path 'logs' }
    }
    if (-not (Test-Path -LiteralPath $logsDir -PathType Container)) {
      New-Item -ItemType Directory -Path $logsDir -Force | Out-Null
    }
    $followupFile = Join-Path $logsDir 'review-followups.md'
    $hdr = "# Review follow-ups (non-blocking QUALITY findings)`n`n" +
           "Append-only index of QUALITY findings from PASSING adversarial-gate runs.`n" +
           "Under the 2026-07 severity contract QUALITY does NOT block (only BLOCKER does),`n" +
           "so these are recorded here for a follow-up session to fix as classes in one`n" +
           "pass, then prune the handled entries. Gitignored execution-state.`n`n"
    $hdrBytes = $enc.GetBytes($hdr)
    $bytes = $enc.GetBytes($block)
    # Header-create AND append under ONE exclusive handle (FileShare.None +
    # OpenOrCreate + length check), with a bounded retry on sharing violations --
    # same protocol as the wrappers' direct-append mode, so concurrent writers
    # (another repo's commit gate, a standalone wrapper run) cannot clobber a
    # freshly appended entry.
    $attempt = 0
    while ($true) {
      try {
        $fs = [System.IO.File]::Open($followupFile, [System.IO.FileMode]::OpenOrCreate, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
        try {
          if ($fs.Length -eq 0) { $fs.Write($hdrBytes, 0, $hdrBytes.Length) }
          else { [void]$fs.Seek(0, [System.IO.SeekOrigin]::End) }
          $fs.Write($bytes, 0, $bytes.Length)
        } finally { $fs.Dispose() }
        break
      } catch {
        $attempt++
        if ($attempt -ge 10) { throw }
        Start-Sleep -Milliseconds 50
      }
    }
    Write-Host "[auto-merge] promoted $BackendName QUALITY follow-ups to $followupFile"
  } catch {
    Write-Host "[auto-merge] WARN: could not promote $BackendName QUALITY follow-ups: $($_.Exception.Message)"
  }
}

# PURE path-selection for the centralized logs dir, byte-identical to the
# Codex/Claude wrappers' Get-ReviewLogDir. Given the resolved git common dir and
# (when known) the working-tree top-level, pick the logs/<backend>/reviews
# destination. SUBMODULE layout (common dir under `<superproject>/.git/modules/
# <name>`) uses the working-tree TOP-LEVEL (its `<common-dir>/..` is `.git/modules`,
# outside the submodule tree); if the top-level is unknown, return $null, NEVER the
# wrong parent. NORMAL / LINKED-WORKTREE uses `<common-dir>/..` (for a worktree
# that is the MAIN repo root -- the centralization that keeps merge forensics with
# the commit-gate artifacts). PURE + SelfTest-covered.
function Get-ReviewLogDir {
  param([string]$CommonDir, [string]$TopLevel, [string]$Backend)
  if ([string]::IsNullOrWhiteSpace($CommonDir)) { return $null }
  $commonNorm = $CommonDir -replace '\\', '/'
  if ($commonNorm -match '(?i)/\.git/modules/') {
    if ([string]::IsNullOrWhiteSpace($TopLevel)) { return $null }
    return [System.IO.Path]::GetFullPath((Join-Path $TopLevel (Join-Path 'logs' (Join-Path $Backend 'reviews'))))
  }
  return [System.IO.Path]::GetFullPath((Join-Path $CommonDir (Join-Path '..' (Join-Path 'logs' (Join-Path $Backend 'reviews')))))
}

# Follow-up-index dir (`<root>/logs`, holding review-followups.md): the SAME
# submodule-aware root selection as Get-ReviewLogDir, derived as its grandparent
# (the helper returns `<root>/logs/<backend>/reviews`) so the index always lands
# beside the verdict artifacts -- a submodule's common-dir parent is
# `.git/modules`, NOT its working tree, and a common-dir-parent shortcut would
# write the index where the batch-triage scan never looks. $null when the
# helper fails safe (caller falls back to the cwd logs dir). PURE +
# SelfTest-covered; byte-identical to both wrappers' copy.
function Get-FollowupIndexDir {
  param([string]$CommonDir, [string]$TopLevel)
  $reviews = Get-ReviewLogDir -CommonDir $CommonDir -TopLevel $TopLevel -Backend 'x'
  if ([string]::IsNullOrWhiteSpace($reviews)) { return $null }
  return (Split-Path -Parent (Split-Path -Parent $reviews))
}

# Resolve the centralized shared-logs `logs/<backend>/reviews` dir -- the SAME
# destination the wrappers' Resolve-DefaultReviewOutDir DEFAULT picks (via the
# shared Get-ReviewLogDir: the git-common-dir parent for normal/worktree layouts,
# the submodule working-tree top-level for submodule installs) -- used ONLY as the
# destination to COPY this run's verdict artifacts into for FORENSICS (these
# artifacts are also read by the top-level trend analyzer, which scans BOTH
# logs/codex/reviews and logs/claude/reviews by default). It is NEVER a trust
# input: the branch review writes to a run-OWNED scratch -OutDir (created below),
# and the merge decision reads only the child WRAPPER exit codes -- no
# merge-decision trust input reads the (gitignored) shared logs. (The exit-1
# QUALITY corroboration that had read only the run-owned dir was removed with
# the 2026-07 severity contract.) Returns $null
# when git cannot resolve a common dir (or a submodule with no resolvable
# top-level), in which case the forensic copy is skipped (the run-owned artifacts
# still exist in the scratch dir for the duration of the run). NON-pure (git
# probe); path SELECTION delegates to the SelfTest-covered Get-ReviewLogDir.
function Resolve-CentralLogDir {
  param([string]$Backend)
  $commonDir = $null
  $absOut = & git rev-parse --path-format=absolute --git-common-dir 2>$null
  if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($absOut)) {
    $commonDir = ($absOut | Select-Object -First 1).Trim()
  } else {
    $plainOut = & git rev-parse --git-common-dir 2>$null
    if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($plainOut)) {
      try { $commonDir = (Resolve-Path -LiteralPath (($plainOut | Select-Object -First 1).Trim()) -ErrorAction Stop).Path }
      catch { $commonDir = $null }
    }
  }
  if ([string]::IsNullOrWhiteSpace($commonDir)) { return $null }
  $topLevel = $null
  $topOut = & git rev-parse --show-toplevel 2>$null
  if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($topOut)) {
    $topLevel = ($topOut | Select-Object -First 1).Trim()
  }
  return Get-ReviewLogDir -CommonDir $commonDir -TopLevel $topLevel -Backend $Backend
}

# Copy a run-owned backend artifact dir into the centralized shared logs for
# forensics and the top-level trend analyzer (which scans BOTH logs/codex/reviews
# and logs/claude/reviews by default). Called from the
# finally block BEFORE the run scratch dir is (conditionally) deleted (copy-then-
# delete, copy first, on EVERY exit path). RETURNS $true when the caller must
# PRESERVE the run dir because consolidation could not safely complete -- so a
# copy/enumerate failure NEVER results in silent artifact loss: the run dir is
# kept instead of deleted (see the finally), and the failure is reported LOUDLY.
#
# FAIL-LOUD + PRESERVE contract (hardened after a sibling session's
# abort silently lost multiple verdicts when the run dir was
# reaped with the artifacts uncopied): every failure mode is reported VISIBLY,
# names what was affected, and signals PRESERVE so nothing is lost --
#   - EMPTY/whitespace $RunBackendDir (no dir was ever named) -> return $false
#     SILENTLY (genuinely nothing to consolidate; safe to delete).
#   - NON-empty $RunBackendDir that is MISSING / not a container at cleanup (an
#     EXPECTED per-backend dir that vanished) -> LOUD WARN + return $true
#     (preserve the parent run dir; its artifacts may have been lost). This is
#     NOT "safe to delete" -- a vanished expected dir is an anomaly.
#   - unresolvable / uncreatable central dir -> LOUD skip line + return $true
#     (preserve; the artifacts stay in the run dir for manual recovery).
#   - run-dir enumeration FAILURE (-ErrorAction Stop in try/catch; a
#     SilentlyContinue here would mask the failure as "0 files" and let the dir be
#     deleted) -> LOUD line + return $true (preserve; we cannot even tell what is
#     in the dir).
#   - PER-FILE copy failure -> continue copying the rest (one bad file must not
#     abort the whole consolidation) but accumulate the failures, emit a LOUD
#     summary naming the count + the central path, and return $true (preserve the
#     run dir so the un-copied originals survive).
#   - a run dir that exists but holds ZERO VERDICT files (after the verdict-shape
#     filter -- it may still hold transient Start-ReviewChild scaffolding, which is
#     NOT centralized and goes away with the run dir) -> LOUD note + return $false
#     (nothing to lose; safe to delete -- usually the child produced no verdict).
#   - full success -> concise line + return $false (every verdict file centralized;
#     safe to delete).
# Still NON-FATAL to the merge decision (the trust read already happened against
# the run-owned dir); loudness is about forensic integrity, not gating. NON-pure.
# RETURNS $true when the caller should PRESERVE the run dir (consolidation could
# not safely complete -- a NON-empty run dir path that is missing/not a container,
# an unresolvable/uncreatable central dir, a run-dir enumeration failure, or one
# or more per-file copy failures), so the finally never silently deletes artifacts
# it could not centralize. RETURNS $false only when deletion is provably safe: the
# $RunBackendDir argument is EMPTY (no dir was ever named -- nothing to lose), OR
# enumeration succeeded and found ZERO VERDICT files after the verdict-shape filter
# (a scaffolding-only dir is safe to delete -- the scaffolding is transient), OR
# every VERDICT file copied successfully (centralized duplicates exist). On
# uncertainty it returns $true (preserve) -- a conservative default that never
# loses data.
# NON-pure (file copy), but the preserve/delete RETURN CONTRACT is SelfTest-
# covered (the CRA-* fixtures) via the -CentralDirOverride test seam, which lets
# the SelfTest drive every branch hermetically without the git-probing resolver.
function Copy-RunArtifactsToCentralLogs {
  param(
    [string]$RunBackendDir,
    [string]$Backend,
    # Test-only seam: when bound (even to ''), it is used as the central dir
    # instead of calling the git-probing Resolve-CentralLogDir, so the SelfTest
    # can drive every return branch hermetically (no git). $null (the default,
    # i.e. param NOT passed) preserves the production path exactly. An empty-
    # string override exercises the unresolvable-central branch.
    $CentralDirOverride = $null,
    # Test-only seam: when bound, REPLACES the copied file's measured destination
    # byte length in the post-copy integrity check, so the SelfTest can force the
    # length-mismatch branch (a truncated/partial copy that a faithful in-process
    # Copy-Item cannot reproduce). NOT passed (the default) preserves the
    # production path exactly (the real Get-Item length is used).
    $SimulateDestLengthForTest = $null
  )
  # An EMPTY $RunBackendDir means there is genuinely nothing to consolidate (no
  # dir was ever named) -> safe to delete, no warning. But a NON-empty path that
  # is MISSING or not a container at consolidation time is an ANOMALY: the gate
  # creates this per-backend dir up front, so its disappearance means artifacts
  # may have been lost -> WARN and return the PRESERVE signal (never silently
  # report safe-to-delete for an expected-but-vanished backend dir). Disabled
  # backends are already skipped at the call site ($includeClaude), so a call
  # here always expects the dir to exist.
  if ([string]::IsNullOrWhiteSpace($RunBackendDir)) { return $false }
  if (-not (Test-Path -LiteralPath $RunBackendDir -PathType Container)) {
    Write-Host "[auto-merge] forensics WARNING: expected '$Backend' run dir '$RunBackendDir' is MISSING or not a directory at cleanup - cannot consolidate; PRESERVING the parent run dir (its artifacts may have been lost)."
    return $true
  }
  $central = if ($PSBoundParameters.ContainsKey('CentralDirOverride')) { $CentralDirOverride } else { Resolve-CentralLogDir -Backend $Backend }
  if ([string]::IsNullOrWhiteSpace($central)) {
    Write-Host "[auto-merge] forensics WARNING: could not resolve central log dir for '$Backend' - the central-log copy is SKIPPED; PRESERVING the run dir so this run's '$Backend' verdict artifacts are not lost."
    return $true
  }
  try {
    if (-not (Test-Path -LiteralPath $central -PathType Container)) {
      New-Item -ItemType Directory -Path $central -Force -ErrorAction Stop | Out-Null
    }
  } catch {
    Write-Host "[auto-merge] forensics WARNING: could not create central log dir '$central' for '$Backend' ($($_.Exception.Message)) - copy SKIPPED; PRESERVING the run dir so '$Backend' artifacts are not lost."
    return $true
  }
  # Enumerate with -ErrorAction Stop inside try/catch: a SilentlyContinue here
  # would convert an access/IO/enumeration failure into an empty set, which would
  # then be misreported as "NO files" and let the run dir be deleted -- silently
  # losing artifacts and defeating the fail-loud contract. On enumeration
  # failure, report it loudly and PRESERVE the run dir (we cannot even tell what
  # is in it).
  $allFiles = $null
  try {
    $allFiles = @(Get-ChildItem -LiteralPath $RunBackendDir -File -ErrorAction Stop)
  } catch {
    Write-Host "[auto-merge] forensics WARNING: could not enumerate '$Backend' run dir '$RunBackendDir' ($($_.Exception.Message)) - cannot centralize; PRESERVING the run dir so any '$Backend' artifacts are not lost."
    return $true
  }
  # Copy ONLY verdict artifacts -- the exact `review-<YYYYMMDD>-<HHMMSS>-...`
  # {.md,.jsonl,.stderr.log} shape the auto-review prune recognizes. Start-ReviewChild
  # leaves scaffolding (child-args.txt, child-exit.txt, child-runner.ps1) in the run
  # dir; copying it into the gitignored central logs would orphan it -- the prune's
  # name regex never matches it, so it would never age out (slow unbounded growth).
  # The scaffolding is transient run-dir state that goes away with the run dir, so it
  # is intentionally NOT preserved. (Mirrors $verdictMdRe/$verdictJsonlRe/$verdictStderrRe
  # in auto-review.ps1's prune.)
  $verdictNameRe = [regex]'^review-\d{8}-\d{6}-.+\.(md|jsonl|stderr\.log)$'
  $files = @($allFiles | Where-Object { $verdictNameRe.IsMatch($_.Name) })
  if ($files.Count -eq 0) {
    Write-Host "[auto-merge] forensics NOTE: run dir for '$Backend' ($RunBackendDir) holds NO verdict files to centralize - nothing to preserve (the child may have produced no verdict; any run scaffolding is transient and goes with the run dir)."
    return $false
  }
  $copied = 0
  $failed = @()
  foreach ($f in $files) {
    try {
      # Per-file copy isolated so ONE unreadable/locked file cannot abort the
      # whole consolidation and drop the files that WOULD have copied (the
      # pre-hardening behavior aborted the loop on the first failure).
      $destPath = Join-Path $central $f.Name
      Copy-Item -LiteralPath $f.FullName -Destination $destPath -Force -ErrorAction Stop
      # Verify the copy landed intact BEFORE counting success: the finally cleanup
      # deletes the run-owned source, so a truncated/partial copy mis-counted as
      # success would lose the verdict artifact. Require the destination to exist
      # (Get-Item -Stop throws if missing) and its byte length to match the source;
      # a mismatch falls to $failed and PRESERVES the run dir. (Codex BLOCKER.)
      $destItem = Get-Item -LiteralPath $destPath -ErrorAction Stop
      $destLen = if ($PSBoundParameters.ContainsKey('SimulateDestLengthForTest')) { $SimulateDestLengthForTest } else { $destItem.Length }
      if ($destLen -ne $f.Length) {
        throw "destination byte length $destLen != source $($f.Length) (truncated/partial copy)"
      }
      $copied++
    } catch {
      $failed += "$($f.Name) ($($_.Exception.Message))"
    }
  }
  if ($failed.Count -gt 0) {
    Write-Host "[auto-merge] forensics WARNING: centralizing '$Backend' artifacts to '$central' - $copied/$($files.Count) copied, $($failed.Count) FAILED: $($failed -join '; '). PRESERVING the run dir so the un-copied '$Backend' artifacts are not lost; the central trail is INCOMPLETE until they are recovered."
    return $true
  }
  Write-Host "[auto-merge] forensics: centralized $copied/$($files.Count) '$Backend' artifact(s) to '$central'."
  return $false
}

# ---------------------------------------------------------------------------
# SelfTest: helper fixtures. Exits before any codex/network. Most are pure
# in-memory fixtures; the Start-ReviewChild end-to-end cases additionally write
# temp files and launch short-lived local powershell.exe children (cleaned up),
# and the TCK-Git* and EB-* fixtures shell out to `git` against isolated throwaway
# TEMP repos (EB-Integration additionally invokes this script end-to-end).
# ---------------------------------------------------------------------------
if ($SelfTest) {
  $failures = 0

  # The 8 required categories, all `none` (the base for a CLEAN/no-finding
  # verdict). A finding fixture flips ONE category to `1` to match its severity.
  $allCatsNone = @"
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none
"@

  # --- Resolve-MergePassMix (claude on/off; total coverage always 3) ---
  function Test-Mix {
    param([string]$Name, [bool]$Claude, [int]$WantCodex, [int]$WantClaude)
    $r = Resolve-MergePassMix -IncludeClaude $Claude
    $total = $r.Codex + $r.Claude
    if ($r.Codex -eq $WantCodex -and $r.Claude -eq $WantClaude -and $total -eq 3) {
      Write-Host "[SelfTest] PASS $Name (codex=$($r.Codex) claude=$($r.Claude) total=$total)"
    } else {
      Write-Host "[SelfTest] FAIL ${Name}: got codex=$($r.Codex) claude=$($r.Claude) total=$total; expected codex=$WantCodex claude=$WantClaude total=3"
      $script:failures++
    }
  }
  Test-Mix 'MIX-ClaudeOn'  $true  2 1
  Test-Mix 'MIX-ClaudeOff' $false 3 0

  # --- Resolve-MergeUnionExit (backend-exit union + {0,2} validation; the 2026-07
  # contract retired exit 1, so a stray 1 -- or any non-{0,2} -- fails the merge closed) ---
  function Test-Union {
    param([string]$Name, [int]$Cx, [int]$Cl, [int]$WantExit)
    $r = Resolve-MergeUnionExit -CodexExit $Cx -ClaudeExit $Cl
    if ($r.Exit -eq $WantExit) { Write-Host "[SelfTest] PASS $Name (-> $($r.Exit))" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got $($r.Exit), expected $WantExit (diag: $($r.Diagnostic))"; $script:failures++ }
  }
  Test-Union 'MU-BothPass'                       0 0 0
  Test-Union 'MU-CodexBlocker'                   2 0 2
  Test-Union 'MU-ClaudeBlocker'                  0 2 2
  Test-Union 'MU-BothBlocker'                    2 2 2
  Test-Union 'MU-CodexRetiredExit1-FailClosed'   1 0 3
  Test-Union 'MU-ClaudeRetiredExit1-FailClosed'  0 1 3
  Test-Union 'MU-CodexExit3-FailClosed'          3 0 3
  Test-Union 'MU-Unexpected4-FailClosed'         0 4 3

  # --- Convert-ToProcArgString (Start-Process arg quoting; multi-word -Title) ---
  function Test-ProcArg {
    param([string]$Name, [string[]]$ArgList, [string]$Want)
    $got = Convert-ToProcArgString -ArgList $ArgList
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got [$got] expected [$Want]"; $script:failures++ }
  }
  Test-ProcArg 'PA-MultiWordTitle' @('-Scope', 'Branch', '-Title', 'Pre-merge review (codex): b (sha) -> main') '-Scope Branch -Title "Pre-merge review (codex): b (sha) -> main"'
  Test-ProcArg 'PA-NoSpaceVerbatim' @('-File', 'C:\path\no-space.ps1', '-Tip', 'deadbeef') '-File C:\path\no-space.ps1 -Tip deadbeef'

  # --- Get-PrependedToolPathParts (the pure PATH-assembly Normalize uses via the
  # no-candidates dedup path; byte-identical to the commit-wrapper copies) ---
  function Test-PathParts {
    param([string]$Name, [string[]]$Candidates, [string]$Current, [string]$Want)
    $got = [string]::Join(';', (Get-PrependedToolPathParts -CandidateDirs $Candidates -CurrentPath $Current))
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got [$got] expected [$Want]"; $script:failures++ }
  }
  Test-PathParts 'PP-DedupOnly (Normalize shape)' @() 'C:\a;C:\a;C:\b' 'C:\a;C:\b'
  Test-PathParts 'PP-BlankSkip + case-insensitive dedup' @() 'C:\a;;C:\A;C:\b' 'C:\a;C:\b'
  Test-PathParts 'PP-Prepend + dedup of candidate already in PATH' @('C:\git\usr\bin', 'C:\git\bin') 'C:\a;C:\git\bin;C:\b' 'C:\git\usr\bin;C:\git\bin;C:\a;C:\b'

  # --- Get-ProcessPathSpellings + Get-UnionProcessPathParts (the duplicate Path/PATH
  # surface fix Normalize relies on; byte-identical to the commit-wrapper copies).
  # GetEnvironmentVariable is case-INSENSITIVE (sees one spelling), so
  # Get-ProcessPathSpellings enumerates the raw block; GetEnvironmentVariables returns
  # a case-SENSITIVE Hashtable, simulated here with New-Object System.Collections.Hashtable. ---
  function Test-SpellVals {
    param([string]$Name, [System.Collections.IDictionary]$Env, [int]$WantCount, [string[]]$WantVals)
    $got = Get-ProcessPathSpellings -EnvEntries $Env   # NO @() -- the ,$array return double-nests under @()
    $got = @($got)
    $ok = (@($got).Count -eq $WantCount)
    foreach ($wv in $WantVals) { if ($got -notcontains $wv) { $ok = $false } }
    if ($ok) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got [$([string]::Join('|', $got))] expected count=$WantCount vals=[$([string]::Join('|', $WantVals))]"; $script:failures++ }
  }
  $stDup = New-Object System.Collections.Hashtable
  $stDup['Path'] = 'C:\a;C:\b'; $stDup['PATH'] = 'C:\c'; $stDup['HOME'] = 'x'
  Test-SpellVals 'Spellings: a DUPLICATE Path+PATH surface yields BOTH distinct values (GetEnvironmentVariable would see one)' $stDup 2 @('C:\a;C:\b', 'C:\c')
  $stOne = New-Object System.Collections.Hashtable
  $stOne['PATH'] = 'C:\only'; $stOne['HOME'] = 'x'
  Test-SpellVals 'Spellings: a single PATH entry yields one value' $stOne 1 @('C:\only')
  $stEmpty = New-Object System.Collections.Hashtable
  $stEmpty['PATH'] = ''; $stEmpty['HOME'] = 'x'
  Test-SpellVals 'Spellings: an empty PATH value is skipped' $stEmpty 0 @()
  # Production-boundary smoke: the DEFAULT-arg path enumerates the LIVE process block.
  $stLive = Get-ProcessPathSpellings   # NO @() -- see Test-SpellVals note
  $stLive = @($stLive)
  if ((@($stLive).Count -ge 1) -and (-not [string]::IsNullOrEmpty($stLive[0]))) { Write-Host '[SelfTest] PASS Spellings: live process block yields a non-empty PATH (production-boundary read)' }
  else { Write-Host "[SelfTest] FAIL Spellings: live process block read (count=$(@($stLive).Count))"; $script:failures++ }

  function Test-UnionParts {
    param([string]$Name, [string[]]$Spellings, [string[]]$Candidates, [string]$Want)
    $got = [string]::Join(';', (Get-UnionProcessPathParts -Spellings $Spellings -CandidateDirs $Candidates))
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got [$got] expected [$Want]"; $script:failures++ }
  }
  Test-UnionParts 'UP-TwoDifferingSpellings union (Normalize shape, neither dropped)' @('C:\a;C:\b', 'C:\c;C:\a') @() 'C:\a;C:\b;C:\c'
  Test-UnionParts 'UP-TwoDifferingSpellings + candidate prepended' @('C:\a;C:\b', 'C:\c') @('C:\git\usr\bin') 'C:\git\usr\bin;C:\a;C:\b;C:\c'
  Test-UnionParts 'UP-SingleSpelling -> that spelling' @('C:\a;C:\b') @() 'C:\a;C:\b'
  Test-UnionParts 'UP-NoSpellings -> empty' @() @() ''
  Test-ProcArg 'PA-SpacePathQuoted' @('-File', 'C:\my dir\x.ps1') '-File "C:\my dir\x.ps1"'

  # --- Get-VerdictExitCode parity with scripts/codex/auto-review.ps1 (bound
  # 10000 not 4096; BLOCKER precedence over a PRESENT malformed verdict WORD
  # -> exit 2; and BLOCKER with a MISSING VERDICT line -> exit 3 fail-closed) ---
  function Test-GV {
    param([string]$Name, [string]$Verdict, [int]$WantExit)
    $cls = Get-VerdictExitCode -Verdict $Verdict
    if ($cls.ExitCode -eq $WantExit) { Write-Host "[SelfTest] PASS $Name (-> $($cls.ExitCode))" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got $($cls.ExitCode), expected $WantExit"; $script:failures++ }
  }
  # Category count in (4096, 10000] is ACCEPTED. 5000 BLOCKER findings + matching
  # category count -> exit 2.
  $gvBlockerLines = (1..5000 | ForEach-Object { "BLOCKER: f${_}.rs:1 - x" }) -join "`n"
  $gvHiCount = ($allCatsNone -replace 'PLAN-DRIFT: none', 'PLAN-DRIFT: 5000') + "`nVERDICT: BLOCKED`n`n" + $gvBlockerLines + "`n"
  Test-GV 'GV-CountWithinWrapperBound' $gvHiCount 2
  # Count > 10000 is rejected (matches the wrapper's 10000 bound).
  $gvOver = ($allCatsNone -replace 'PLAN-DRIFT: none', 'PLAN-DRIFT: 10001') + "`nVERDICT: BLOCKED`n`nBLOCKER: f.rs:1 - x`n"
  Test-GV 'GV-CountOverWrapperBound' $gvOver 3
  # BLOCKER + a MALFORMED verdict word -> exit 2 (BLOCKER precedence wins).
  $gvBlkMalformed = ($allCatsNone -replace 'PLAN-DRIFT: none', 'PLAN-DRIFT: 1') + "`nVERDICT: FOO`n`nBLOCKER: f.rs:1 - x`n"
  Test-GV 'GV-BlockerOverMalformedVerdict' $gvBlkMalformed 2
  # QUALITY + malformed verdict word (no BLOCKER) -> exit 3.
  $gvQualMalformed = ($allCatsNone -replace 'TEST-QUALITY: none', 'TEST-QUALITY: 1') + "`nVERDICT: FOO`n`nQUALITY: f.rs:1 - x`n"
  Test-GV 'GV-QualityMalformedVerdictRejected' $gvQualMalformed 3
  # CLEAN -> 0 and BLOCKED -> 2 as before; QUALITY-only -> 0 under the 2026-07
  # severity contract (non-blocking), in parity with the auto-review.ps1 classifiers.
  Test-GV 'GV-Clean'   ("$allCatsNone`nVERDICT: CLEAN`n") 0
  Test-GV 'GV-Quality' (($allCatsNone -replace 'TEST-QUALITY: none', 'TEST-QUALITY: 1') + "`nVERDICT: NON-BLOCKING`n`nQUALITY: f.rs:1 - x`n") 0
  Test-GV 'GV-Blocker' (($allCatsNone -replace 'PLAN-DRIFT: none', 'PLAN-DRIFT: 1') + "`nVERDICT: BLOCKED`n`nBLOCKER: f.rs:1 - x`n") 2
  Test-GV 'GV-DupVerdict' ("$allCatsNone`nVERDICT: CLEAN`nVERDICT: CLEAN`n") 3
  # BLOCKER + ZERO VERDICT: lines -> 3 (missing line is malformed output; distinct
  # from GV-BlockerOverMalformedVerdict where a VERDICT line is PRESENT but wrong
  # -> 2). Parity with codex wrapper V21. (Merge-gate BLOCKER.)
  Test-GV 'GV-BlockerMissingVerdict' (($allCatsNone -replace 'PLAN-DRIFT: none', 'PLAN-DRIFT: 1') + "`nBLOCKER: f.rs:1 - x`n") 3

  # --- Test-PlanInNameStatus (PLAN.md-in-name-status detection for the
  # codex-only-mix forcing) ---
  function Test-PlanNS {
    param([string]$Name, [string]$NameStatus, [bool]$Want, [string]$DocName = 'PLAN.md')
    $got = Test-PlanInNameStatus -NameStatus $NameStatus -DocName $DocName
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name (-> $got)" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got $got, expected $Want"; $script:failures++ }
  }
  Test-PlanNS 'PNS-Modified'      "M`tPLAN.md"                       $true
  Test-PlanNS 'PNS-Added'         "A`tPLAN.md"                       $true
  Test-PlanNS 'PNS-RenameToPlan'  "R096`tdocs/OLD.md`tPLAN.md"       $true   # PLAN.md as rename DEST (2nd path col)
  Test-PlanNS 'PNS-RenameFromPlan' "R096`tPLAN.md`tdocs/NEW.md"      $true   # PLAN.md as rename SRC (1st path col)
  Test-PlanNS 'PNS-MultiLine'     "M`tsrc/foo.rs`nM`tPLAN.md"        $true   # PLAN.md on a later line
  Test-PlanNS 'PNS-OtherOnly'     "M`tsrc/foo.rs`nA`tdocs/bar.md"    $false
  Test-PlanNS 'PNS-Substringy'    "M`tdocs/PLAN.md.bak"              $false  # path mentions PLAN.md but is not it
  Test-PlanNS 'PNS-PlanSubdir'    "M`tsub/PLAN.md"                   $false  # only top-level PLAN.md fails the claude gate
  Test-PlanNS 'PNS-Empty'         ''                                $false
  # Windows-style configured DocName (backslash) must still match git's forward-
  # slash path column (normalized inside Test-PlanInNameStatus).
  Test-PlanNS 'PNS-BackslashDoc'  "M`tdocs/PLAN.md"                  $true  'docs\PLAN.md'
  Test-PlanNS 'PNS-BackslashDocNoMatch' "M`tsrc/foo.rs"             $false 'docs\PLAN.md'

  # --- Resolve-EffectiveCodexEffort (BYTE-IDENTICAL to the codex wrapper; the
  # merge gate pins the branch review to the SAME tier it resolves) ---
  function Test-RE {
    param([string]$Name, [string]$Explicit, [string]$Config, [string]$Want)
    $got = Resolve-EffectiveCodexEffort -ExplicitEffort $Explicit -ConfigText $Config
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name (-> $got)" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got '$got', expected '$Want'"; $script:failures++ }
  }
  $reCfgXhigh = "model = `"gpt-5.5`"`nmodel_reasoning_effort = `"xhigh`"`n`n[profiles.fast]`nmodel_reasoning_effort = `"low`"`n"
  $reCfgNoKey = "model = `"gpt-5.5`"`n`n[tui]`ntheme = `"dark`"`n"
  $reCfgProfileOnly = "model = `"gpt-5.5`"`n`n[profiles.fast]`nmodel_reasoning_effort = `"medium`"`n"
  Test-RE 'RE-ExplicitWins'       'high'   $reCfgXhigh       'high'
  Test-RE 'RE-ExplicitOverConfig' 'medium' $reCfgXhigh       'medium'
  Test-RE 'RE-ConfigTopLevel'     ''       $reCfgXhigh       'xhigh'
  Test-RE 'RE-ConfigProfileOnly'  ''       $reCfgProfileOnly 'unknown'
  Test-RE 'RE-ConfigNoKey'        ''       $reCfgNoKey       'unknown'
  Test-RE 'RE-EmptyConfig'        ''       ''                'unknown'
  Test-RE 'RE-ExplicitCaseFold'   'XHigh'  $reCfgNoKey       'xhigh'
  Test-RE 'RE-ConfigNoQuotes'     ''       "model_reasoning_effort = high`n" 'high'
  Test-RE 'RE-InvalidConfig'      ''       "model_reasoning_effort = ""xtreme""`n" 'unknown'  # non-tier config value -> unknown
  Test-RE 'RE-InvalidExplicit'    'bogus'  "model_reasoning_effort = ""xhigh""`n" 'unknown'  # non-tier explicit value -> unknown
  # TOML form coverage (CROSS-CRATE-CONTRACT): inline comments + single quotes.
  Test-RE 'RE-ConfigTrailingComment' '' "model_reasoning_effort = ""high"" # prefer high`n" 'high'
  Test-RE 'RE-ConfigSingleQuote'      '' "model_reasoning_effort = 'medium'`n" 'medium'
  Test-RE 'RE-ConfigSingleQuoteComment' '' "model_reasoning_effort = 'low'   # note`n" 'low'
  Test-RE 'RE-ConfigBareComment'       '' "model_reasoning_effort = xhigh # bare`n" 'xhigh'
  Test-RE 'RE-ConfigMismatchedQuote'   '' "model_reasoning_effort = ""high`n" 'unknown'

  # --- Resolve-ReviewChildExit (deterministic exit-file readback; the fix for
  # the PS 5.1 -PassThru/null-ExitCode quirk). Readable wrapper codes {0,1,2,3} pass
  # through (1 = the retired QUALITY code -- readback-recognized but rejected by the
  # {0,2} union check); the '999' crash sentinel and any other non-verdict / unparsable /
  # missing content fail closed as $null. Hermetic: writes fixture files into a
  # private temp dir, cleans each up with single-file Remove-Item (no -Recurse).
  $rceUtf8 = [System.Text.UTF8Encoding]::new($false)
  $rceDir = Join-Path $env:TEMP ("crg-auto-merge-selftest-rce-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
  New-Item -ItemType Directory -Path $rceDir -Force | Out-Null
  function Test-RCE {
    param([string]$Name, [string]$FileContent, $WantExit, [bool]$Missing = $false)
    $f = Join-Path $rceDir ("exit-" + $Name + ".txt")
    if (-not $Missing) { [System.IO.File]::WriteAllText($f, $FileContent, $rceUtf8) }
    $got = Resolve-ReviewChildExit -ExitFile $f -BackendName 'selftest'
    $ok = if ($null -eq $WantExit) { $null -eq $got } else { $got -eq $WantExit }
    if ($ok) { Write-Host "[SelfTest] PASS RCE-$Name (-> $(if ($null -eq $got) { 'null' } else { $got }))" }
    else { Write-Host "[SelfTest] FAIL RCE-${Name}: got $(if ($null -eq $got) { 'null' } else { $got }), expected $(if ($null -eq $WantExit) { 'null' } else { $WantExit })"; $script:failures++ }
    if (-not $Missing -and (Test-Path -LiteralPath $f)) { Remove-Item -LiteralPath $f -ErrorAction SilentlyContinue }
  }
  Test-RCE 'Clean'        '0'        0
  Test-RCE 'Quality'      '1'        1
  Test-RCE 'Blocker'      '2'        2
  Test-RCE 'WrapperFail'  '3'        3      # legitimate wrapper exit 3 passes through (handled by caller)
  Test-RCE 'Sentinel999'  '999'      $null  # runner crash sentinel -> fail closed
  Test-RCE 'Whitespace'   "  2  "    2      # trimmed
  Test-RCE 'Empty'        ''         $null
  Test-RCE 'Garbage'      'CLEAN'    $null  # non-numeric -> fail closed
  Test-RCE 'NegativeCode' '-1'       $null  # parses but not a verdict code -> fail closed
  Test-RCE 'Missing'      ''         $null  $true   # file absent -> fail closed
  Remove-Item -LiteralPath $rceDir -ErrorAction SilentlyContinue  # empty dir (all fixture files removed above)

  # --- Start-ReviewChild END-TO-END (the runtime path the RCE fixtures alone do
  # NOT exercise): real Start-Process launch, generated runner, args-file
  # round-trip, nested `& powershell.exe -File <wrapper> <args>`, handle cache,
  # parent WaitForExit, exit-file readback, AND Test-ChildExitConsistency. The
  # "wrapper" is a .ps1 that takes the SAME arg shape production passes after
  # -File (named -Scope/-Tip + a MULTI-WORD -Title), records what it received, and
  # `exit N`s. Each case asserts BOTH the verdict code AND that every arg
  # round-tripped through child-args.txt + Convert-ToProcArgString intact (the
  # multi-word -Title unsplit) -- so a launch/quoting/arg-preservation defect
  # FAILS here instead of slipping through. Each case gets its own run dir
  # (Start-ReviewChild writes child-args.txt / child-exit.txt / child-runner.ps1
  # there). Hermetic: per-case files + dir removed after (single-file, no -Recurse).
  $srcUtf8 = [System.Text.UTF8Encoding]::new($false)
  $srcRoot = Join-Path $env:TEMP ("crg-auto-merge-selftest-src-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
  New-Item -ItemType Directory -Path $srcRoot -Force | Out-Null
  function Test-SRC {
    param([int]$Target)
    $caseDir = Join-Path $srcRoot "case-$Target"
    New-Item -ItemType Directory -Path $caseDir -Force | Out-Null
    # Wrapper stand-in that exercises the SAME arg shape production passes AFTER
    # -File: named options (-Scope/-Tip) AND a MULTI-WORD -Title (the value
    # Convert-ToProcArgString must keep intact through Start-Process and the
    # runner must reconstruct losslessly from child-args.txt). It records the
    # bound parameter VALUES to args-seen.txt and exits with $Target, so a break
    # in arg preservation after -File -- or a split multi-word -Title -- makes the
    # round-trip assertion below FAIL rather than silently pass.
    $seenPath = Join-Path $caseDir 'args-seen.txt'
    $wrapperPath = Join-Path $caseDir 'wrapper.ps1'
    $wrapperBody = @"
param([string]`$Scope, [string]`$Tip, [string]`$Title)
[System.IO.File]::WriteAllText('$($seenPath.Replace("'", "''"))', "Scope=`$Scope|Tip=`$Tip|Title=`$Title", [System.Text.UTF8Encoding]::new(`$false))
exit $Target
"@
    [System.IO.File]::WriteAllText($wrapperPath, $wrapperBody, $srcUtf8)
    $expectTitle = "Pre-merge review (selftest): b ($Target) -> main"
    $wrapperArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $wrapperPath,
                     '-Scope', 'Branch', '-Tip', "deadbeef$Target", '-Title', $expectTitle)
    $child = Start-ReviewChild -RunDir $caseDir -WrapperArgs $wrapperArgs -Utf8NoBom $srcUtf8
    if ($null -ne $child.Proc) { $child.Proc.WaitForExit() }
    $got = Resolve-ReviewChildExit -ExitFile $child.ExitFile -BackendName 'selftest-src'
    # The file value and the handle .ExitCode AGREE for a normal child, so the
    # consistency cross-check must return $true (no tamper/mismatch). Capture it.
    $consistent = Test-ChildExitConsistency -BackendName 'selftest-src' -FileExit $got -Proc $child.Proc
    $handleExit = $null
    try { if ($null -ne $child.Proc) { $handleExit = $child.Proc.ExitCode } } catch { $handleExit = '<threw>' }
    # Verify the wrapper saw the EXACT args (round-trip through child-args.txt +
    # Convert-ToProcArgString, including the multi-word -Title unsplit).
    $argsSeen = if (Test-Path -LiteralPath $seenPath) { [System.IO.File]::ReadAllText($seenPath, [System.Text.Encoding]::UTF8) } else { '<missing>' }
    $expectSeen = "Scope=Branch|Tip=deadbeef$Target|Title=$expectTitle"
    $exitOk = ($got -eq $Target -and $handleExit -eq $Target)
    $argsOk = ($argsSeen -eq $expectSeen)
    if ($exitOk -and $argsOk -and $consistent) {
      Write-Host "[SelfTest] PASS SRC-Exit$Target (file=$got handle=$handleExit; args round-tripped incl multi-word -Title; consistency-check=true)"
    } else {
      if (-not $exitOk) { Write-Host "[SelfTest] FAIL SRC-Exit${Target}: file=$(if ($null -eq $got){'null'}else{$got}) handle=$(if ($null -eq $handleExit){'null'}else{$handleExit}), expected both $Target" }
      if (-not $argsOk) { Write-Host "[SelfTest] FAIL SRC-Exit${Target} args round-trip: got [$argsSeen], expected [$expectSeen]" }
      if (-not $consistent) { Write-Host "[SelfTest] FAIL SRC-Exit${Target} consistency-check returned false for a matching file/handle pair" }
      $script:failures++
    }
    foreach ($n in 'wrapper.ps1', 'args-seen.txt', 'child-args.txt', 'child-exit.txt', 'child-runner.ps1') {
      $p = Join-Path $caseDir $n
      if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -ErrorAction SilentlyContinue }
    }
    Remove-Item -LiteralPath $caseDir -ErrorAction SilentlyContinue
  }
  Test-SRC 0
  Test-SRC 1
  Test-SRC 2
  Test-SRC 3
  Remove-Item -LiteralPath $srcRoot -ErrorAction SilentlyContinue  # empty dir (all case dirs removed above)

  # --- Test-ChildExitConsistency TAMPER CHECK (the security-critical negative
  # path). The SRC cases above only exercise the MATCHING path; this pins the
  # MISMATCH -> $false behavior so a regression that returned $true for a forged
  # child-exit.txt (handle != file) cannot pass green. Launch ONE real child that
  # exits with a known code (2), keep its process handle, then drive the four
  # outcomes directly. Hermetic: own run dir, single-file cleanup.
  $tceUtf8 = [System.Text.UTF8Encoding]::new($false)
  $tceDir = Join-Path $env:TEMP ("crg-auto-merge-selftest-tce-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
  New-Item -ItemType Directory -Path $tceDir -Force | Out-Null
  $tceWrapper = Join-Path $tceDir 'wrapper.ps1'
  [System.IO.File]::WriteAllText($tceWrapper, "exit 2`n", $tceUtf8)
  $tceArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $tceWrapper)
  $tceChild = Start-ReviewChild -RunDir $tceDir -WrapperArgs $tceArgs -Utf8NoBom $tceUtf8
  if ($null -ne $tceChild.Proc) { $tceChild.Proc.WaitForExit() }
  function Test-TCE {
    param([string]$Name, $FileExit, $Proc, [bool]$Want)
    $got = Test-ChildExitConsistency -BackendName 'selftest-tce' -FileExit $FileExit -Proc $Proc -InformationAction SilentlyContinue
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS TCE-$Name (-> $got)" }
    else { Write-Host "[SelfTest] FAIL TCE-${Name}: got $got, expected $Want"; $script:failures++ }
  }
  # The live child exited 2. (a) file matches handle -> consistent ($true).
  Test-TCE 'Match'        2     $tceChild.Proc $true
  # (b) file MISMATCHES handle (forged/misrecorded) -> NOT consistent ($false).
  #     THIS is the security-critical assertion: a tampered child-exit.txt of 0
  #     while the real process exited 2 must be REJECTED.
  Test-TCE 'MismatchForged' 0   $tceChild.Proc $false
  Test-TCE 'MismatchBlocker' 1  $tceChild.Proc $false   # any divergence rejects
  # (c) null process handle (launch failure / unreadable) -> cannot cross-check,
  #     fall back to the file's own validation -> $true.
  Test-TCE 'NullProc'     2     $null          $true
  # (d) null FileExit (already failed closed at the call site) -> not cross-
  #     checked here -> $true.
  Test-TCE 'NullFileExit' $null $tceChild.Proc $true
  foreach ($n in 'wrapper.ps1', 'child-args.txt', 'child-exit.txt', 'child-runner.ps1') {
    $p = Join-Path $tceDir $n
    if (Test-Path -LiteralPath $p) { Remove-Item -LiteralPath $p -ErrorAction SilentlyContinue }
  }
  Remove-Item -LiteralPath $tceDir -ErrorAction SilentlyContinue

  # --- Copy-RunArtifactsToCentralLogs preserve/delete RETURN CONTRACT. The
  # finally relies on this return to decide PRESERVE vs delete the run dir. EIGHT
  # return branches exist; SEVEN are forced with hermetic fixtures here -- empty-
  # arg run dir (-> $false), missing/non-container run dir (-> $true), unresolvable
  # central (-> $true), uncreatable central (-> $true, via an unmapped-drive
  # override; the ONLY conditional fixture -- skipped if no free drive letter),
  # zero-VERDICT-files dir (-> $false), per-file copy failure (-> $true), and clean
  # copy (-> $false). The remaining branch -- run-dir ENUMERATION failure (-> $true
  # with a loud WARN) -- is verified by inspection: it shares the IDENTICAL preserve-
  # and-warn shape as the forced -> $true cases, and a Get-ChildItem enumeration
  # throw needs ACL/handle states a portable hermetic fixture cannot reliably set.
  # The clean-copy case ALSO pins the verdict-shape FILTER: only
  # `review-<ts>-...{.md,.jsonl,.stderr.log}` artifacts are centralized; child-runner
  # scaffolding is excluded (CRA-ScaffoldingExcluded) and a scaffolding-only run dir
  # reports zero verdicts -> $false (CRA-ScaffoldingOnly). Verdict fixtures use the
  # real timestamped name shape so they pass the filter. Hermetic via the
  # -CentralDirOverride seam (no git probe). Asserts BOTH the return value AND, where
  # relevant, that a loud diagnostic fired and the right files landed centrally.
  # Single-file cleanup (no -Recurse). $craRoot temp dir removed at the end.
  $craUtf8 = [System.Text.UTF8Encoding]::new($false)
  $craRoot = Join-Path $env:TEMP ("crg-auto-merge-selftest-cra-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
  New-Item -ItemType Directory -Path $craRoot -Force | Out-Null
  function Assert-CRA {
    param([string]$Name, $Got, $Want, [string]$Out, [string]$MustContain = '')
    $ok = ($Got -eq $Want)
    # PowerShell 5.1 wraps InformationRecord text at host width during Out-String.
    # Normalize whitespace so assertions test diagnostic content, not console width.
    $normalizedOut = $Out -replace '\s+', ' '
    $normalizedMustContain = $MustContain -replace '\s+', ' '
    if ($MustContain -ne '' -and ($normalizedOut -notmatch [regex]::Escape($normalizedMustContain))) { $ok = $false }
    if ($ok) { Write-Host "[SelfTest] PASS CRA-$Name (-> $Got)" }
    else { Write-Host "[SelfTest] FAIL CRA-${Name}: got return=$Got (want $Want)$(if($MustContain -ne ''){"; output missing '$MustContain'"})"; $script:failures++ }
  }
  # (1) clean copy -> $false (safe to delete), VERDICT files present centrally,
  # Start-ReviewChild scaffolding EXCLUDED (the verdict-shape filter). Verdict
  # names use the real `review-<YYYYMMDD>-<HHMMSS>-...` shape the filter + prune
  # match; the scaffolding files (child-args.txt, child-runner.ps1, child-exit.txt)
  # must NOT be centralized (they would orphan in the gitignored central logs --
  # the prune never ages them out).
  $craRun1 = Join-Path $craRoot 'run1'; New-Item -ItemType Directory -Path $craRun1 -Force | Out-Null
  $craCen1 = Join-Path $craRoot 'central1'
  # Cover ALL THREE verdict shapes the filter accepts (.md, .jsonl, .stderr.log) so
  # a regression dropping any one shape from the centralization fails this fixture.
  [System.IO.File]::WriteAllText((Join-Path $craRun1 'review-20260615-211344-staged.md'), 'A', $craUtf8)
  [System.IO.File]::WriteAllText((Join-Path $craRun1 'review-20260615-211344-staged-pass1.jsonl'), 'B', $craUtf8)
  [System.IO.File]::WriteAllText((Join-Path $craRun1 'review-20260615-211344-staged-pass1.stderr.log'), 'C', $craUtf8)
  # Scaffolding that must be skipped:
  [System.IO.File]::WriteAllText((Join-Path $craRun1 'child-args.txt'), 'ARGS', $craUtf8)
  [System.IO.File]::WriteAllText((Join-Path $craRun1 'child-runner.ps1'), 'RUNNER', $craUtf8)
  [System.IO.File]::WriteAllText((Join-Path $craRun1 'child-exit.txt'), '0', $craUtf8)
  $craRet1 = $null
  $craOut1 = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRun1 -Backend 'codex' -CentralDirOverride $craCen1 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet1 = $_ } else { $_ } } | Out-String)
  Assert-CRA 'CleanCopy' $craRet1 $false $craOut1 'centralized 3/3'
  if ((Test-Path (Join-Path $craCen1 'review-20260615-211344-staged.md')) -and (Test-Path (Join-Path $craCen1 'review-20260615-211344-staged-pass1.jsonl')) -and (Test-Path (Join-Path $craCen1 'review-20260615-211344-staged-pass1.stderr.log'))) { Write-Host '[SelfTest] PASS CRA-CleanCopyFilesPresent' } else { Write-Host '[SelfTest] FAIL CRA-CleanCopyFilesPresent'; $script:failures++ }
  if ((-not (Test-Path (Join-Path $craCen1 'child-args.txt'))) -and (-not (Test-Path (Join-Path $craCen1 'child-runner.ps1'))) -and (-not (Test-Path (Join-Path $craCen1 'child-exit.txt')))) { Write-Host '[SelfTest] PASS CRA-ScaffoldingExcluded' } else { Write-Host '[SelfTest] FAIL CRA-ScaffoldingExcluded (run scaffolding leaked into central logs)'; $script:failures++ }
  # (1b) a run dir with ONLY scaffolding (no verdict) -> $false (nothing to
  # preserve; scaffolding is transient and goes with the run dir).
  $craRun1b = Join-Path $craRoot 'run1b'; New-Item -ItemType Directory -Path $craRun1b -Force | Out-Null
  $craCen1b = Join-Path $craRoot 'central1b'
  [System.IO.File]::WriteAllText((Join-Path $craRun1b 'child-args.txt'), 'ARGS', $craUtf8)
  [System.IO.File]::WriteAllText((Join-Path $craRun1b 'child-runner.ps1'), 'RUNNER', $craUtf8)
  $craRet1b = $null
  $craOut1b = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRun1b -Backend 'codex' -CentralDirOverride $craCen1b 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet1b = $_ } else { $_ } } | Out-String)
  Assert-CRA 'ScaffoldingOnly' $craRet1b $false $craOut1b 'NO verdict files'
  # (2) empty run dir -> $false (nothing to lose).
  $craRun2 = Join-Path $craRoot 'run2'; New-Item -ItemType Directory -Path $craRun2 -Force | Out-Null
  $craRet2 = $null
  $craOut2 = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRun2 -Backend 'codex' -CentralDirOverride (Join-Path $craRoot 'central2') 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet2 = $_ } else { $_ } } | Out-String)
  Assert-CRA 'EmptyDir' $craRet2 $false $craOut2 'holds NO verdict files'
  # (3) unresolvable central ('' override) -> $true (PRESERVE), loud skip.
  $craRun3 = Join-Path $craRoot 'run3'; New-Item -ItemType Directory -Path $craRun3 -Force | Out-Null
  [System.IO.File]::WriteAllText((Join-Path $craRun3 'review-20260615-211344-staged.md'), 'X', $craUtf8)
  $craRet3 = $null
  $craOut3 = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRun3 -Backend 'codex' -CentralDirOverride '' 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet3 = $_ } else { $_ } } | Out-String)
  Assert-CRA 'UnresolvableCentral' $craRet3 $true $craOut3 'PRESERVING the run dir'
  # (4) per-file copy failure (exclusive lock on a source file) -> $true
  # (PRESERVE), names the lost file, partial count, OTHER file still copied.
  $craRun4 = Join-Path $craRoot 'run4'; New-Item -ItemType Directory -Path $craRun4 -Force | Out-Null
  $craCen4 = Join-Path $craRoot 'central4'; New-Item -ItemType Directory -Path $craCen4 -Force | Out-Null
  [System.IO.File]::WriteAllText((Join-Path $craRun4 'review-20260615-211344-ok.md'), 'OK', $craUtf8)
  $craBad = Join-Path $craRun4 'review-20260615-211344-bad.md'
  [System.IO.File]::WriteAllText($craBad, 'BAD', $craUtf8)
  $craLock = [System.IO.File]::Open($craBad, [System.IO.FileMode]::Open, [System.IO.FileAccess]::Read, [System.IO.FileShare]::None)
  $craRet4 = $null
  try {
    $craOut4 = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRun4 -Backend 'codex' -CentralDirOverride $craCen4 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet4 = $_ } else { $_ } } | Out-String)
  } finally { $craLock.Close(); $craLock.Dispose() }
  Assert-CRA 'PerFileFail' $craRet4 $true $craOut4 'review-20260615-211344-bad.md'
  if (Test-Path (Join-Path $craCen4 'review-20260615-211344-ok.md')) { Write-Host '[SelfTest] PASS CRA-PerFileFailOtherCopied' } else { Write-Host '[SelfTest] FAIL CRA-PerFileFailOtherCopied'; $script:failures++ }
  # (4b) INTEGRITY MISMATCH: a copy that SUCCEEDS (Copy-Item returns) but lands a
  # DIFFERENT byte length than the source (a truncated/partial copy) must be
  # counted as a FAILURE -> $true (PRESERVE run dir) + loud WARN naming the
  # mismatch, never silently counted as copied (the finally cleanup deletes the
  # run-owned source, so a mis-counted truncated copy would lose the verdict
  # artifact). Forced via the -SimulateDestLengthForTest seam, since a faithful
  # in-process Copy-Item cannot produce a length mismatch. (Codex TEST-QUALITY.)
  $craRunM = Join-Path $craRoot 'runM'; New-Item -ItemType Directory -Path $craRunM -Force | Out-Null
  $craCenM = Join-Path $craRoot 'centralM'; New-Item -ItemType Directory -Path $craCenM -Force | Out-Null
  [System.IO.File]::WriteAllText((Join-Path $craRunM 'review-20260101-000000-trunc.md'), 'CONTENT', $craUtf8)
  $craRetM = $null
  $craOutM = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRunM -Backend 'codex' -CentralDirOverride $craCenM -SimulateDestLengthForTest 0 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRetM = $_ } else { $_ } } | Out-String)
  Assert-CRA 'IntegrityMismatch' $craRetM $true $craOutM 'destination byte length'
  # (5) UNCREATABLE central dir -> $true (PRESERVE), loud skip. Forced hermetically
  # by pointing the override at a path on an UNMAPPED drive letter: the function's
  # `Test-Path -PathType Container` returns $false (no throw -- unmapped drive is
  # just "not found"), then `New-Item -Directory -Force` throws "Cannot find
  # drive" into the catch. (`-Force` cannot conjure a missing drive, unlike a
  # colliding file/parent which it would overwrite -- which is why the file-path
  # approach does NOT force this branch.)
  $craUnmapped = $null
  $craUsedDrives = (Get-PSDrive -PSProvider FileSystem -ErrorAction SilentlyContinue).Name
  foreach ($dl in [char[]]('Q', 'X', 'Y', 'Z', 'W', 'V', 'U', 'T')) {
    if ($craUsedDrives -notcontains "$dl") { $craUnmapped = "$dl"; break }
  }
  if ($null -ne $craUnmapped) {
    $craRun5 = Join-Path $craRoot 'run5'; New-Item -ItemType Directory -Path $craRun5 -Force | Out-Null
    [System.IO.File]::WriteAllText((Join-Path $craRun5 'review-20260615-211344-staged.md'), 'Y', $craUtf8)
    $craBadCentral = "${craUnmapped}:\crg-selftest-nope\central"
    $craRet5 = $null
    $craOut5 = (Copy-RunArtifactsToCentralLogs -RunBackendDir $craRun5 -Backend 'codex' -CentralDirOverride $craBadCentral 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet5 = $_ } else { $_ } } | Out-String)
    Assert-CRA 'UncreatableCentral' $craRet5 $true $craOut5 'could not create central log dir'
  } else {
    Write-Host '[SelfTest] SKIP CRA-UncreatableCentral (no unmapped drive letter available; branch is otherwise inspection-covered)'
  }
  # (6) MISSING/non-container run backend dir -> $true (PRESERVE), loud WARN. An
  # expected per-backend dir that has vanished at cleanup is an anomaly; the
  # function must never silently report safe-to-delete for it. Forced by passing a
  # non-existent run dir path.
  # Assert the RETURN VALUE only (the safety-critical contract: a missing expected
  # run dir must return $true=PRESERVE, never silently safe-to-delete). The loud
  # WARNING that accompanies it IS emitted (the same Write-Host path used by every
  # other forensics line), but its CONTENT is not re-asserted here: a Write-Host
  # immediately before a function `return` is not reliably captured by a
  # 6>&1/-InformationVariable pipe under PS 5.1 (the deeper-context cases like
  # UncreatableCentral capture fine; this shallow-return case does not), so a
  # content match would be a flaky PS-version artifact, not a real contract check.
  $craMissingRun = Join-Path $craRoot 'run6-does-not-exist'
  # Copy-RunArtifactsToCentralLogs is a SIMPLE function (no [CmdletBinding()] /
  # [Parameter()]), so it does not honor common parameters like -InformationAction
  # -- such a token would land inert in $args. This case asserts only the boolean
  # return, so the function's WARN line printing to the host is harmless. (Contrast
  # Test-ChildExitConsistency above, which IS advanced and DOES honor the idiom.)
  $craRet6 = Copy-RunArtifactsToCentralLogs -RunBackendDir $craMissingRun -Backend 'codex' -CentralDirOverride (Join-Path $craRoot 'central6')
  Assert-CRA 'MissingRunDir' $craRet6 $true ''
  # Empty-string run dir -> $false (genuinely nothing named; no warning).
  $craRet7 = $null
  $craOut7 = (Copy-RunArtifactsToCentralLogs -RunBackendDir '' -Backend 'codex' -CentralDirOverride (Join-Path $craRoot 'central7') 6>&1 | ForEach-Object { if ($_ -is [bool]) { $craRet7 = $_ } else { $_ } } | Out-String)
  Assert-CRA 'EmptyRunDirArg' $craRet7 $false $craOut7
  # Cleanup WITHOUT -Recurse (agent deletion-safety rule): files then dirs.
  Get-ChildItem -LiteralPath $craRoot -Recurse -File -ErrorAction SilentlyContinue | ForEach-Object { Remove-Item -LiteralPath $_.FullName -ErrorAction SilentlyContinue }
  Get-ChildItem -LiteralPath $craRoot -Recurse -Directory -ErrorAction SilentlyContinue | Sort-Object { $_.FullName.Length } -Descending | ForEach-Object { Remove-Item -LiteralPath $_.FullName -ErrorAction SilentlyContinue }
  Remove-Item -LiteralPath $craRoot -ErrorAction SilentlyContinue

  # --- Resolve-ConsistencyDocConfig (CROSS_REVIEW_CONSISTENCY_DOC normalization;
  # the env-var fail-OPEN-vs-fail-CLOSED contract). The HELPER FUNCTION is kept
  # BYTE-IDENTICAL across the codex auto-review, claude auto-review, and auto-merge
  # copies (verified by hash). The fixture CASES below are equivalent across the
  # three (same inputs + expected outputs); the codex auto-review copy additionally
  # carries inline explanatory comments, so the fixture BYTES are not identical --
  # only the helper is. ---
  function Test-RCD {
    param([string]$Name, $Raw, [string]$WantState, [string]$WantDoc)
    $got = Resolve-ConsistencyDocConfig -RawValue $Raw
    # Reason contract: an 'invalid' result MUST carry a non-empty diagnostic
    # (INSTALL.md promises a fail-closed message + the callers print it); 'off'
    # and 'valid' MUST have an empty Reason. Asserting Reason here catches a
    # regression that returns the right State/Doc but drops the diagnostic.
    $reasonOk = if ($WantState -eq 'invalid') { -not [string]::IsNullOrEmpty($got.Reason) } else { [string]::IsNullOrEmpty($got.Reason) }
    $ok = ($got.State -eq $WantState) -and ($got.Doc -eq $WantDoc) -and $reasonOk
    if ($ok) { Write-Host "[SelfTest] PASS $Name (State=$($got.State) Doc='$($got.Doc)')" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got State=$($got.State) Doc='$($got.Doc)' Reason='$($got.Reason)', wanted State=$WantState Doc='$WantDoc' (Reason $(if ($WantState -eq 'invalid') { 'non-empty' } else { 'empty' }))"; $script:failures++ }
  }
  Test-RCD 'RCD-Null'          $null         'off'     ''
  Test-RCD 'RCD-Valid'         'PLAN.md'     'valid'   'PLAN.md'
  Test-RCD 'RCD-ValidNested'   'docs/PLAN.md' 'valid'  'docs/PLAN.md'
  Test-RCD 'RCD-Padded'        'PLAN.md '    'valid'   'PLAN.md'
  Test-RCD 'RCD-PaddedBoth'    '  PLAN.md  ' 'valid'   'PLAN.md'
  Test-RCD 'RCD-Backslash'     'docs\PLAN.md' 'valid'  'docs/PLAN.md'
  Test-RCD 'RCD-Empty'         ''            'invalid' ''
  Test-RCD 'RCD-Whitespace'    '   '         'invalid' ''
  Test-RCD 'RCD-Tab'           "`t"          'invalid' ''
  Test-RCD 'RCD-AbsDrive'      'C:/PLAN.md'  'invalid' ''
  Test-RCD 'RCD-AbsDriveBack'  'C:\PLAN.md'  'invalid' ''
  Test-RCD 'RCD-AbsPosix'      '/etc/PLAN.md' 'invalid' ''
  Test-RCD 'RCD-DotDot'        '../PLAN.md'  'invalid' ''
  Test-RCD 'RCD-DotDotMid'     'docs/../../x.md' 'invalid' ''
  Test-RCD 'RCD-DotSlash'      './PLAN.md'   'valid'   'PLAN.md'
  Test-RCD 'RCD-DotSlashBack'  '.\PLAN.md'   'valid'   'PLAN.md'
  Test-RCD 'RCD-DotMid'        'docs/./PLAN.md' 'valid' 'docs/PLAN.md'
  Test-RCD 'RCD-DotMidBack'    'docs\.\PLAN.md' 'valid' 'docs/PLAN.md'
  Test-RCD 'RCD-TrailingSlash' 'PLAN.md/'    'valid'   'PLAN.md'
  Test-RCD 'RCD-DoubleSlash'   'docs//PLAN.md' 'valid' 'docs/PLAN.md'
  Test-RCD 'RCD-DotOnly'       '.'           'invalid' ''
  Test-RCD 'RCD-DotSlashOnly'  './'          'invalid' ''
  Test-RCD 'RCD-DotDriveHidden' './C:/PLAN.md' 'invalid' ''
  Test-RCD 'RCD-DotDriveHiddenBack' '.\C:\PLAN.md' 'invalid' ''
  Test-RCD 'RCD-PosixAbsDot'   '/./etc/PLAN.md' 'invalid' ''

  # --- Test-ConsistencyDocKind (a SHAPE-valid config must ALSO resolve to a git
  # BLOB in the reviewed tree; `cat-file -e` accepts a tree, so a directory-valued
  # config (`docs/`) must be rejected by the KIND check) ---
  function Test-TCK {
    param([string]$Name, [string]$Kind, [bool]$WantValid)
    $got = [bool](Test-ConsistencyDocKind -Kind $Kind)
    if ($got -eq $WantValid) { Write-Host "[SelfTest] PASS $Name (Kind='$Kind' -> $got)" }
    else { Write-Host "[SelfTest] FAIL ${Name}: Kind='$Kind' gave $got, wanted $WantValid"; $script:failures++ }
  }
  Test-TCK 'TCK-Blob'    'blob'    $true     # a tracked FILE -> valid
  Test-TCK 'TCK-Tree'    'tree'    $false    # a DIRECTORY (e.g. `docs/`) -> rejected (the BLOCKER this guards)
  Test-TCK 'TCK-Missing' 'missing' $false    # object absent (typo) -> rejected
  Test-TCK 'TCK-Empty'   ''        $false    # empty kind (cat-file produced nothing) -> rejected
  Test-TCK 'TCK-Commit'  'commit'  $false    # a commit/tag object -> rejected (only a blob is a file)

  # Integration: drive the REAL `git cat-file -t` boundary Get-GitObjectKind wraps,
  # against an ISOLATED throwaway repo under $env:TEMP, so a directory (`docs/`) ->
  # 'tree' and a typo -> 'missing' (NOT 'blob') is proven end-to-end, not just via
  # synthetic kind strings. git + writable TEMP are HARD package prerequisites, so a
  # setup failure FAILS (not skips -- a skipped test that still prints "all passed"
  # is the false-clean the gate exists to prevent). The repo is built with isolated
  # identity / signing / hooks so it neither depends on nor perturbs the host git
  # config; cleanup runs in `finally` via Remove-TreeNoRecurse (the temp tree is a
  # git repo whose loose objects are read-only on Windows -> per-file -Force, never
  # -Recurse). (Codex TEST-QUALITY.)
  $tckTmp = Join-Path ([System.IO.Path]::GetTempPath()) ("crg-tck-" + [guid]::NewGuid().ToString('N').Substring(0, 12))
  $tckBuilt = $false; $r1 = ''; $r2 = ''; $r3 = ''
  try {
    New-Item -ItemType Directory -Path (Join-Path $tckTmp 'docs') -Force | Out-Null
    Set-Content -LiteralPath (Join-Path $tckTmp 'docs/PLAN.md') -Value '# plan'
    Push-Location $tckTmp
    try {
      & git init -q 2>$null
      # Keep the fixture hermetic: a locked/unreadable host excludesfile must
      # not fail this Get-GitObjectKind boundary test before the commit.
      & git -c core.excludesfile= add -A 2>$null
      # -c identity/gpgsign + --no-verify: do not depend on the host's global git
      # identity, signing, or hooks (any would otherwise fail/perturb the commit).
      & git -c core.excludesfile= -c user.email='selftest@invalid' -c user.name='crg-selftest' -c commit.gpgsign=false commit -q --no-verify -m 'tck fixture' 2>$null
      $head = "$(& git rev-parse --verify HEAD 2>$null)".Trim()
      if ($LASTEXITCODE -eq 0 -and -not [string]::IsNullOrWhiteSpace($head)) {
        # Resolve kinds HERE -- inside the temp repo -- since Get-GitObjectKind
        # runs `git` in the current dir and the temp commit's objects live here.
        $r1 = Get-GitObjectKind -TreeRef $head -Path 'docs/PLAN.md'
        $r2 = Get-GitObjectKind -TreeRef $head -Path 'docs'
        $r3 = Get-GitObjectKind -TreeRef $head -Path 'docs/NOPE.md'
        $tckBuilt = $true
      }
    } finally { Pop-Location }
  } catch { $tckBuilt = $false } finally {
    Remove-TreeNoRecurse -Path $tckTmp
  }
  if ($tckBuilt) {
    if ($r1 -eq 'blob')    { Write-Host "[SelfTest] PASS TCK-GitBlob (tracked file -> blob)" }    else { Write-Host "[SelfTest] FAIL TCK-GitBlob: got '$r1', wanted blob"; $script:failures++ }
    if ($r2 -eq 'tree')    { Write-Host "[SelfTest] PASS TCK-GitTree (directory -> tree)" }        else { Write-Host "[SelfTest] FAIL TCK-GitTree: got '$r2', wanted tree"; $script:failures++ }
    if ($r3 -eq 'missing') { Write-Host "[SelfTest] PASS TCK-GitMissing (typo -> missing)" }       else { Write-Host "[SelfTest] FAIL TCK-GitMissing: got '$r3', wanted missing"; $script:failures++ }
  } else {
    Write-Host "[SelfTest] FAIL TCK-Git* (could not build the throwaway git repo -- git + writable TEMP are hard package prerequisites)"; $script:failures++
  }

  # --- EB-Integration: omitted -Base resolves through the real main flow (git) ---
  # The empty-default BLOCKER (a `-Base ''` default whose runtime assignment was
  # missing) passed the pure DB-* normalization fixtures yet broke every real run.
  # This drives the SCRIPT end-to-end with NO -Base in a throwaway repo: $Base must
  # resolve to a NON-EMPTY default (origin/HEAD, fallback 'main') BEFORE the
  # HEAD/base preflight, so the run exits 3 naming 'main', NOT ''. It exits at the
  # base-mismatch check (HEAD on 'feat' != resolved 'main') BEFORE any review
  # launch, so it stays git-only (no codex/network). git + writable TEMP are hard
  # package prerequisites; a setup failure FAILS (not skips). Cleanup via
  # Remove-TreeNoRecurse. (Codex BLOCKER + TEST-QUALITY.)
  $ebTmp = Join-Path ([System.IO.Path]::GetTempPath()) ("crg-eb-" + [guid]::NewGuid().ToString('N').Substring(0, 12))
  $ebBuilt = $false; $ebExit = $null; $ebText = ''
  try {
    New-Item -ItemType Directory -Path $ebTmp -Force | Out-Null
    Set-Content -LiteralPath (Join-Path $ebTmp 'f.txt') -Value 'x'
    Push-Location $ebTmp
    try {
      & git init -q 2>$null
      # Ignore host excludesfile state; this fixture only needs the file staged.
      & git -c core.excludesfile= add -A 2>$null
      & git -c core.excludesfile= -c user.email='selftest@invalid' -c user.name='crg-selftest' -c commit.gpgsign=false commit -q --no-verify -m 'eb fixture' 2>$null
      # A feature branch distinct from the resolved default ('main') so the
      # preflight hits the HEAD/base mismatch and exits BEFORE any review launch.
      & git -c core.excludesfile= checkout -q -b feat 2>$null
      if ($LASTEXITCODE -eq 0) {
        $oldEap = $ErrorActionPreference
        try {
          # The child is expected to exit 3 through a preflight diagnostic; keep
          # that stderr as assertable output instead of promoting it to a parent
          # SelfTest harness failure.
          $ErrorActionPreference = 'Continue'
          $ebText = (& powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -Branch feat 2>&1 | Out-String)
          $ebExit = $LASTEXITCODE
        } finally {
          $ErrorActionPreference = $oldEap
        }
        $ebBuilt = $true
      }
    } finally { Pop-Location }
  } catch { $ebBuilt = $false } finally {
    Remove-TreeNoRecurse -Path $ebTmp
  }
  if ($ebBuilt) {
    # The resolved base must be non-empty: the preflight names 'main' (the
    # origin/HEAD fallback, since the throwaway repo has no remote), never ''.
    # The empty-default regression would print "expected ''" and FAIL here.
    $ebResolvedNonEmpty = ($ebText -match "expected 'main'") -and -not ($ebText -match "expected ''")
    if ($ebExit -eq 3 -and $ebResolvedNonEmpty) {
      Write-Host "[SelfTest] PASS EB-Integration: omitted -Base resolved to non-empty 'main' through the main-flow preflight"
    } else {
      Write-Host "[SelfTest] FAIL EB-Integration: omitted -Base did not resolve to a non-empty base (exit=$ebExit; expected 3 naming 'main', not ''). text=$ebText"
      $script:failures++
    }
  } else {
    Write-Host "[SelfTest] FAIL EB-Integration (could not build the throwaway git repo -- git + writable TEMP are hard package prerequisites)"; $script:failures++
  }

  # --- EB-OriginHead: Resolve-DefaultBaseBranch reads origin/HEAD (PRIMARY path) ---
  # EB-Integration above covers only the no-remote 'main' FALLBACK. This builds a
  # throwaway repo with a local `refs/remotes/origin/HEAD` symbolic ref (synthesized
  # without a real remote: create refs/remotes/origin/trunk at HEAD, then point
  # origin/HEAD at it) and asserts Resolve-DefaultBaseBranch returns the pointed-to
  # branch 'trunk', NOT the 'main' fallback -- exercising the `git symbolic-ref
  # --short refs/remotes/origin/HEAD` detection. git-only; no codex/network. Setup
  # failure FAILS (not skips). Cleanup via Remove-TreeNoRecurse.
  $ohTmp = Join-Path ([System.IO.Path]::GetTempPath()) ("crg-oh-" + [guid]::NewGuid().ToString('N').Substring(0, 12))
  $ohBuilt = $false; $ohResolved = ''
  try {
    New-Item -ItemType Directory -Path $ohTmp -Force | Out-Null
    Set-Content -LiteralPath (Join-Path $ohTmp 'f.txt') -Value 'x'
    Push-Location $ohTmp
    try {
      & git init -q 2>$null
      # Ignore host excludesfile state; this fixture only needs the file staged.
      & git -c core.excludesfile= add -A 2>$null
      & git -c core.excludesfile= -c user.email='selftest@invalid' -c user.name='crg-selftest' -c commit.gpgsign=false commit -q --no-verify -m 'oh fixture' 2>$null
      & git update-ref refs/remotes/origin/trunk HEAD 2>$null
      & git symbolic-ref refs/remotes/origin/HEAD refs/remotes/origin/trunk 2>$null
      if ($LASTEXITCODE -eq 0) {
        $ohResolved = Resolve-DefaultBaseBranch
        $ohBuilt = $true
      }
    } finally { Pop-Location }
  } catch { $ohBuilt = $false } finally {
    Remove-TreeNoRecurse -Path $ohTmp
  }
  if ($ohBuilt) {
    if ($ohResolved -eq 'trunk') {
      Write-Host "[SelfTest] PASS EB-OriginHead: Resolve-DefaultBaseBranch reads origin/HEAD -> 'trunk' (primary path, not the 'main' fallback)"
    } else {
      Write-Host "[SelfTest] FAIL EB-OriginHead: origin/HEAD not resolved to 'trunk' (got '$ohResolved'; the fallback would give 'main')"
      $script:failures++
    }
  } else {
    Write-Host "[SelfTest] FAIL EB-OriginHead (could not build the throwaway git repo -- git + writable TEMP are hard package prerequisites)"; $script:failures++
  }

  # --- Get-ReviewLogDir (common-dir/top-level -> logs dir SELECTION) fixtures.
  # Byte-identical helper to the wrappers'; pins the submodule-vs-normal-vs-
  # worktree branch + the submodule-no-top-level -> $null fail-safe. ---
  function Test-ReviewLogDir {
    param([string]$Name, [string]$CommonDir, [string]$TopLevel, [string]$WantContains, [bool]$WantNull)
    $r = Get-ReviewLogDir -CommonDir $CommonDir -TopLevel $TopLevel -Backend 'codex'
    $ok = if ($WantNull) { $null -eq $r } else { ($null -ne $r) -and (($r -replace '\\','/') -like "*$WantContains*") }
    if ($ok) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got '$r'"; $script:failures++ }
  }
  Test-ReviewLogDir 'RLD-Normal'         'C:/proj/.git'               'C:/proj'        'C:/proj/logs/codex/reviews'      $false
  Test-ReviewLogDir 'RLD-Worktree'       'C:/main/.git'               'C:/main/.wt/w1' 'C:/main/logs/codex/reviews'      $false
  Test-ReviewLogDir 'RLD-Submodule'      'C:/super/.git/modules/sub'  'C:/super/sub'   'C:/super/sub/logs/codex/reviews' $false
  Test-ReviewLogDir 'RLD-SubmoduleNoTop' 'C:/super/.git/modules/sub'  ''               '' $true
  Test-ReviewLogDir 'RLD-EmptyCommon'    ''                           'C:/proj'        '' $true

  # --- Get-FollowupIndexDir (follow-up index root; submodule-aware, derived
  # from Get-ReviewLogDir) fixtures ---
  function Test-FollowupIndexDir {
    param([string]$Name, [string]$CommonDir, [string]$TopLevel, [string]$Want, [bool]$WantNull)
    $r = Get-FollowupIndexDir -CommonDir $CommonDir -TopLevel $TopLevel
    $ok = if ($WantNull) { $null -eq $r } else { ($null -ne $r) -and ((($r -replace '\\','/').TrimEnd('/')) -eq $Want) }
    if ($ok) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: got '$r'"; $script:failures++ }
  }
  Test-FollowupIndexDir 'FID-Normal'         'C:/proj/.git'              'C:/proj'      'C:/proj/logs'      $false
  Test-FollowupIndexDir 'FID-Worktree'       'C:/main/.git'              'C:/main/.wt/w1' 'C:/main/logs'    $false
  # SUBMODULE: the index must land under the submodule's OWN tree (<top>/logs),
  # never under the .git/modules parent where batch triage never looks.
  Test-FollowupIndexDir 'FID-Submodule'      'C:/super/.git/modules/sub' 'C:/super/sub' 'C:/super/sub/logs' $false
  Test-FollowupIndexDir 'FID-SubmoduleNoTop' 'C:/super/.git/modules/sub' ''             '' $true

  # --- Get-NormalizedDefaultBase (project-agnostic base resolution; PURE) ---
  function Test-NormBase {
    param([string]$Name, [string]$In, [string]$Want)
    $got = Get-NormalizedDefaultBase -DetectedRef $In
    if ($got -eq $Want) { Write-Host "[SelfTest] PASS $Name ('$In' -> '$got')" }
    else { Write-Host "[SelfTest] FAIL ${Name}: '$In' -> '$got' (expected '$Want')"; $script:failures++ }
  }
  Test-NormBase 'DB-OriginMain'        'origin/main'         'main'
  Test-NormBase 'DB-OriginMaster'      'origin/master'       'master'
  Test-NormBase 'DB-OriginTrunk'       'origin/trunk'        'trunk'
  Test-NormBase 'DB-BareName'          'main'                'main'
  Test-NormBase 'DB-EmptyFallback'     ''                    'main'
  Test-NormBase 'DB-WhitespaceFallback' '   '                'main'
  Test-NormBase 'DB-NestedRef'         'origin/release/2026' 'release/2026'

  # --- Publish-ReviewFollowups (post-merge promotion of staged QUALITY
  # follow-ups; the -OrchDirOverride seam skips the runtime git probe) ---
  $prfDir   = Join-Path ([System.IO.Path]::GetTempPath()) ('crg-prf-' + [System.IO.Path]::GetRandomFileName())
  New-Item -ItemType Directory -Path $prfDir -Force | Out-Null
  $prfOrch  = Join-Path $prfDir 'logs'
  $prfIndex = Join-Path $prfOrch 'review-followups.md'
  $prfPend1 = Join-Path $prfDir 'p1.md'
  $prfPend2 = Join-Path $prfDir 'p2.md'
  try {
    [System.IO.File]::WriteAllText($prfPend1, "## t1  backend=codex  verdict=v1.md`n- QUALITY: a.rs:1`n`n")
    [System.IO.File]::WriteAllText($prfPend2, "## t2  backend=claude  verdict=v2.md`n- QUALITY: b.rs:2`n`n")
    Publish-ReviewFollowups -PendingPath $prfPend1 -BackendName 'codex' -OrchDirOverride $prfOrch
    $prfText1 = if (Test-Path -LiteralPath $prfIndex) { [System.IO.File]::ReadAllText($prfIndex) } else { '' }
    if ($prfText1.StartsWith('# Review follow-ups') -and $prfText1.Contains('- QUALITY: a.rs:1')) {
      Write-Host '[SelfTest] PASS PRF-HeaderAndFirstAppend'
    } else { Write-Host '[SelfTest] FAIL PRF-HeaderAndFirstAppend'; $script:failures++ }
    Publish-ReviewFollowups -PendingPath $prfPend2 -BackendName 'claude' -OrchDirOverride $prfOrch
    $prfText2 = if (Test-Path -LiteralPath $prfIndex) { [System.IO.File]::ReadAllText($prfIndex) } else { '' }
    $prfHdrCount = ([regex]::Matches($prfText2, [regex]::Escape('# Review follow-ups (non-blocking QUALITY findings)'))).Count
    if ($prfText2.Contains('- QUALITY: a.rs:1') -and $prfText2.Contains('- QUALITY: b.rs:2') -and ($prfHdrCount -eq 1)) {
      Write-Host '[SelfTest] PASS PRF-SecondAppendNoTruncateOneHeader'
    } else { Write-Host '[SelfTest] FAIL PRF-SecondAppendNoTruncateOneHeader'; $script:failures++ }
    $prfLenBefore = (Get-Item -LiteralPath $prfIndex).Length
    Publish-ReviewFollowups -PendingPath (Join-Path $prfDir 'missing.md') -BackendName 'codex' -OrchDirOverride $prfOrch
    $prfLenAfter = (Get-Item -LiteralPath $prfIndex).Length
    if ($prfLenAfter -eq $prfLenBefore) { Write-Host '[SelfTest] PASS PRF-MissingPendingNoOp' }
    else { Write-Host '[SelfTest] FAIL PRF-MissingPendingNoOp'; $script:failures++ }
  } finally {
    Remove-Item -LiteralPath $prfPend1 -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $prfPend2 -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $prfIndex -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $prfOrch -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $prfDir -ErrorAction SilentlyContinue
  }

  if ($failures -eq 0) {
    Write-Host "[SelfTest] All auto-merge helper fixtures passed (pass-mix, union-exit (incl. retired-exit-1 fail-closed), proc-arg, path-assembly (Get-PrependedToolPathParts, Get-UnionProcessPathParts, Get-ProcessPathSpellings), verdict-parity, plan-name-status, effective-effort, review-child-exit, start-review-child-e2e, child-exit-consistency incl the security-critical mismatch->fail-closed negative cases, consistency-doc-config, consistency-doc-kind, review-log-dir, default-base-normalization, omitted-Base resolution (fallback + origin/HEAD), copy-run-artifacts incl a conditional uncreatable-central case that runs only when an unmapped drive letter is available, publish-review-followups promotion), zero failures."
    exit 0
  } else {
    Write-Host "[SelfTest] $failures failures."
    exit 1
  }
}

# Runtime requirement: -Branch is mandatory outside SelfTest.
if ([string]::IsNullOrWhiteSpace($Branch)) {
  Write-Host "[auto-merge] ERROR: -Branch <branch> is required (it is optional only for -SelfTest)."
  exit 3
}

# Project-agnostic base resolution: with no -Base, resolve the repo's default
# branch (origin/HEAD, fallback 'main'). MUST run before the preflight below --
# an unresolved empty $Base makes the HEAD/base check compare against '' and
# abort every run (the empty-default regression the EB-Integration SelfTest guards).
if ([string]::IsNullOrWhiteSpace($Base)) {
  $Base = Resolve-DefaultBaseBranch
  Write-Host "[auto-merge] -Base not given; resolved default base branch -> '$Base'"
}

# Pre-flight: branch exists, base is current branch, tracked tree clean.
$branchExists = (& git rev-parse --verify --quiet "refs/heads/$Branch" 2>$null)
if (-not $branchExists) {
  Write-Host "[auto-merge] ERROR: branch '$Branch' does not exist"
  exit 3
}

$currentBranch = (& git rev-parse --abbrev-ref HEAD).Trim()
if ($currentBranch -ne $Base) {
  Write-Host "[auto-merge] ERROR: HEAD is on '$currentBranch', expected '$Base'"
  Write-Host "[auto-merge] Run 'git checkout $Base' first."
  exit 3
}

# Tracked-only cleanliness check (`--untracked-files=no` semantics): only
# tracked changes block the merge. Untracked build output, caches, and the
# gate's own generated artifacts (logs/) do not.
& git diff --quiet
$unstagedDirty = ($LASTEXITCODE -ne 0)
& git diff --cached --quiet
$stagedDirty = ($LASTEXITCODE -ne 0)
if ($unstagedDirty -or $stagedDirty) {
  Write-Host "[auto-merge] ERROR: tracked working tree on '$Base' is not clean"
  & git status --short --untracked-files=no
  exit 3
}

# Confirm ff-merge is actually possible.
$mergeBase = (& git merge-base $Base $Branch).Trim()
$baseHead = (& git rev-parse $Base).Trim()
$baseSha = $baseHead
if ($mergeBase -ne $baseHead) {
  Write-Host "[auto-merge] ERROR: '$Branch' is not a strict descendant of '$Base'."
  Write-Host "[auto-merge] merge-base=$mergeBase, $Base HEAD=$baseHead"
  Write-Host "[auto-merge] Rebase '$Branch' onto '$Base' first."
  exit 3
}

# Pin the candidate branch tip to a concrete SHA. The review will run against
# this SHA and the merge will use this SHA. If the branch ref moves between
# the review and the merge, the script aborts before merging. This prevents
# a TOCTOU window where reviewed content is not the merged content.
$branchSha = (& git rev-parse $Branch).Trim()
if (-not $branchSha) {
  Write-Host "[auto-merge] ERROR: could not resolve '$Branch' to a SHA"
  exit 3
}
Write-Host "[auto-merge] Pinned '$Branch' to $branchSha"

# Resolve the merge gate's EFFECTIVE codex reasoning-effort (used to PIN the
# branch-review codex child below so the review runs at a deterministic, stamped
# tier rather than re-reading config). `unknown` -> do not pin (child inherits
# codex's own default).
$mergeEffort = Resolve-EffectiveCodexEffort -ExplicitEffort $ReasoningEffort -ConfigText (Get-CodexConfigText)
Write-Host "[auto-merge] codex review effort: $mergeEffort"

# Resolve the cross-provider pass mix. Total independent coverage is always 3.
# $NoClaude disables the cross-provider claude pass.
$includeClaude = -not $NoClaude.IsPresent

# Consistency-doc exception (OPT-IN via CROSS_REVIEW_CONSISTENCY_DOC): the Claude
# merge wrapper fails closed (exit 3) whenever the configured consistency doc is
# in the changed-path set (its consistency precompute is not ported), and a claude
# exit 3 ABORTS the merge. So when the branch diff touches the configured doc,
# force the codex-only mix (same counts as -NoClaude) -- otherwise a valid
# doc-touching branch could never merge through the default mix. A fresh install
# (env var unset) has NO special-casing: no doc forces codex-only. Use NAME-STATUS
# (not raw diff text) so this script's own doc-mentioning comments do not
# false-trigger; resolve it from the already-pinned base/branch SHAs. A detection
# error fails toward LESS surprise (keep the requested mix) but is logged; the
# dominant case is a clean read.
# Normalize + validate the env value via the shared helper (trim, repo-relative,
# `\`->`/`). A whitespace-only / padded / absolute / `..`-escaping value is a
# BROKEN GATE CONFIG: it must FAIL CLOSED (abort the merge, exit 3), never
# silently keep the cross-provider mix while the gate config is broken (the
# fail-OPEN bug -- a doc-touching branch would then run a Claude pass that itself
# fails closed, aborting opaquely, OR mis-route the mix). $cfg.Doc is the
# normalized doc to match.
$cfgMergeDoc = Resolve-ConsistencyDocConfig -RawValue $env:CROSS_REVIEW_CONSISTENCY_DOC
if ($cfgMergeDoc.State -eq 'invalid') {
  Write-Host "[auto-merge] ERROR: $($cfgMergeDoc.Reason). The consistency-doc routing is a fail-closed config; refusing to merge with a broken CROSS_REVIEW_CONSISTENCY_DOC (fix or unset it). Aborting."
  exit 3
}
$consistencyDoc = $cfgMergeDoc.Doc
if ($cfgMergeDoc.State -eq 'valid') {
  # SHAPE-valid is not enough: a typo'd doc (`PLNA.md`) OR a directory (`docs/` ->
  # collapses to `docs`) is shape-valid yet is not a tracked FILE, so the branch-
  # diff test below never matches and the mix silently stays cross-provider -- the
  # fail-OPEN this closes. Require the configured doc to resolve to a git BLOB in
  # the branch tree (`cat-file -t`, not `-e`, which accepts a tree); a missing/
  # tree/non-blob object is a BROKEN CONFIG -> fail closed (exit 3), never a silent
  # mis-route. (Codex BLOCKER, merge gate.)
  $docKind = Get-GitObjectKind -TreeRef $branchSha -Path $consistencyDoc
  if (-not (Test-ConsistencyDocKind -Kind $docKind)) {
    Write-Host "[auto-merge] ERROR: CROSS_REVIEW_CONSISTENCY_DOC='$consistencyDoc' does not name a tracked FILE in the branch tree ($branchSha) (kind='$docKind') - a directory-valued or absent consistency doc is a broken gate config; refusing to merge (fix or unset it). Aborting."
    exit 3
  }
}
if ($includeClaude -and $cfgMergeDoc.State -eq 'valid') {
  try {
    # --no-ext-diff --no-textconv: this name-status drives a routing decision (it
    # forces codex-only when the consistency doc is in the branch), so a
    # tree-configured external diff driver / textconv filter must not be able to
    # alter the path list and flip the decision. Same trust-boundary rationale as
    # the auto-review wrappers' evidence calls.
    $branchNameStatus = (& git diff --no-color --no-ext-diff --no-textconv --name-status "$baseSha...$branchSha" 2>$null | Out-String)
    $diffExit = $LASTEXITCODE
    if ($diffExit -ne 0) {
      # A NONZERO git exit is swallowed by `2>$null` and would otherwise fail the
      # `-eq 0` test SILENTLY (only a TERMINATING error reaches the catch below).
      # Log it explicitly before keeping the requested mix so the "is logged"
      # contract in the comment above holds for the nonzero-exit path too.
      Write-Host "[auto-merge] WARN: '$consistencyDoc' detection git diff --name-status exited $diffExit - cannot determine whether the branch touches the consistency doc; keeping the requested pass mix"
    } elseif (Test-PlanInNameStatus -NameStatus $branchNameStatus -DocName $consistencyDoc) {
      Write-Host "[auto-merge] $consistencyDoc in branch diff - Claude wrapper's consistency fail-closed contract applies; using codex-only pass mix"
      $includeClaude = $false
    }
  } catch {
    Write-Host "[auto-merge] $consistencyDoc detection error ($($_.Exception.Message)) - keeping the requested pass mix"
  }
}

$passMix = Resolve-MergePassMix -IncludeClaude $includeClaude
Write-Host "[auto-merge] pass mix: $($passMix.Codex) codex + $($passMix.Claude) claude = 3 independent passes"

# Snapshot the base-branch review infra to a tempdir so the running gate is
# guaranteed to come from $Base, not from the candidate branch.
#
# A SEPARATE fresh per-run scratch dir owned by THIS auto-merge invocation. Both
# child wrappers are launched with an explicit -OutDir into a per-backend subdir
# here. The gate-owned writers are the review process (its verdict
# `review-*-branch-*.md` and its staged `followups.pending.md`, promoted
# post-merge by Publish-ReviewFollowups or discarded with the run dir on abort)
# and Start-ReviewChild (the runner scaffolding --
# child-args.txt, child-exit.txt, child-runner.ps1). NOTE this dir is under
# $env:TEMP with default ACLs -- that is NOT an OS-enforced no-other-writer
# boundary (a same-user process could write here). The dir is auto-merge-CREATED
# fresh for THIS run, so no pre-existing planted file can be inside it. (The exit-1
# QUALITY corroboration that had also read ONLY these run-owned dirs, filtered to the
# verdict shape `review-*-branch-*.md`, was REMOVED with the 2026-07 severity
# contract -- the merge decision now reads only the child WRAPPER exit codes, never
# these artifacts, so artifact provenance no longer gates the merge.) After the
# review, the run's VERDICT artifacts (the
# `review-*` {.md,.jsonl,.stderr.log} shape) are COPIED into the centralized
# shared logs for forensics and the top-level trend analyzer (which scans BOTH
# backend log dirs by default); the Start-ReviewChild scaffolding
# (child-args.txt / child-exit.txt / child-runner.ps1) is TRANSIENT and is NOT
# centralized -- it is discarded with the run dir (the wrappers do NOT
# auto-centralize when an explicit -OutDir is given). The
# finally block then deletes BOTH the trusted-infra dir AND the run dir
# CONDITIONALLY: it preserves the trusted-infra dir while any child is still
# running (a live child reads its -PromptPath from it), and preserves the run dir
# when a child is still running OR consolidation could not centralize every
# VERDICT artifact (a scaffolding-only run dir is safe to delete; see the finally
# for details).
#
# These directory paths are assigned BEFORE the main try/catch (the catch maps
# review/invocation failures to exit 3) because the finally block must reference
# them for cleanup. The New-Item creations are wrapped in their OWN fail-closed
# try/catch: a setup failure here would otherwise terminate under
# $ErrorActionPreference='Stop' with a raw exit (1), which the exit contract does
# not treat as a valid review verdict (exit 1 is retired under the 2026-07 severity
# contract -- QUALITY passes as 0). On failure, clean up whatever was created and
# exit 3 (invocation failure).
$trustedDir = Join-Path $env:TEMP "crg-auto-merge-trusted-$([guid]::NewGuid().ToString('N').Substring(0,8))"
$runDir = Join-Path $env:TEMP "crg-auto-merge-run-$([guid]::NewGuid().ToString('N').Substring(0,8))"
$runCodexDir  = Join-Path $runDir 'codex'
$runClaudeDir = Join-Path $runDir 'claude'
try {
  New-Item -ItemType Directory -Path $trustedDir -Force | Out-Null
  New-Item -ItemType Directory -Path (Join-Path $trustedDir 'scripts\codex') -Force | Out-Null
  New-Item -ItemType Directory -Path $runCodexDir -Force | Out-Null
  if ($includeClaude) { New-Item -ItemType Directory -Path $runClaudeDir -Force | Out-Null }
} catch {
  Write-Host "[auto-merge] ERROR: could not create scratch directories ($($_.Exception.Message)) - merge ABORTED (fail closed)."
  Remove-TreeNoRecurse -Path $trustedDir
  Remove-TreeNoRecurse -Path $runDir
  exit 3
}

# Extract the review infra from BASE so a candidate branch cannot weaken the
# reviewer that decides whether it may merge. The shared prompt template + the
# codex wrapper are always needed. When the claude cross-provider pass is
# enabled, the claude wrapper is extracted from base too (same `git show`
# mechanism) -- otherwise a candidate branch could ship a weakened
# scripts/claude/auto-review.ps1 and have THAT decide the claude pass.
#
# This whole extraction block runs BEFORE the main review try/catch (that try
# maps review/invocation failures to exit 3), still under
# $ErrorActionPreference='Stop'. Wrap it in its OWN fail-closed try/catch so ANY
# setup terminating error -- the scripts\claude New-Item, the per-file outDir
# New-Item, or the WriteAllText -- maps to invocation-failure exit 3, not
# PowerShell's raw exit 1 (no longer a valid review verdict -- exit 1 is retired
# under the 2026-07 severity contract). The explicit `git show` non-zero check
# below keeps its own specific exit-3 message.
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)
try {
  $reviewInfra = @(
    'scripts/codex/auto-review.ps1',
    'scripts/codex/review-prompt-template.md'
  )
  if ($includeClaude) {
    $reviewInfra += 'scripts/claude/auto-review.ps1'
    New-Item -ItemType Directory -Path (Join-Path $trustedDir 'scripts\claude') -Force | Out-Null
  }
  foreach ($f in $reviewInfra) {
    $outPath = Join-Path $trustedDir $f.Replace('/', '\')
    $outDir = Split-Path $outPath -Parent
    if (-not (Test-Path $outDir)) {
      New-Item -ItemType Directory -Path $outDir -Force | Out-Null
    }
    # `git show` returns lines as a string array in PowerShell. Joining with
    # LF and forcing UTF-8-no-BOM preserves the script's line structure so the
    # extracted file actually parses as code (a previous version collapsed it
    # to a single comment line via `Set-Content -NoNewline`).
    $lines = & git show "${baseSha}:$f"
    if ($LASTEXITCODE -ne 0) {
      Write-Host "[auto-merge] ERROR: could not read $f from base '$Base' ($baseSha)"
      Remove-TreeNoRecurse -Path $trustedDir
      Remove-TreeNoRecurse -Path $runDir
      exit 3
    }
    $content = ($lines -join "`n") + "`n"
    [System.IO.File]::WriteAllText($outPath, $content, $utf8NoBom)
  }
} catch {
  Write-Host "[auto-merge] ERROR: could not extract trusted review infra ($($_.Exception.Message)) - merge ABORTED (fail closed)."
  Remove-TreeNoRecurse -Path $trustedDir
  Remove-TreeNoRecurse -Path $runDir
  exit 3
}

$trustedWrapper = Join-Path $trustedDir 'scripts\codex\auto-review.ps1'
$trustedPrompt  = Join-Path $trustedDir 'scripts\codex\review-prompt-template.md'
$trustedClaudeWrapper = Join-Path $trustedDir 'scripts\claude\auto-review.ps1'

Write-Host "[auto-merge] Running branch review of $branchSha (=$Branch) vs '$Base' (trusted infra at $trustedDir; run-owned out dirs at $runDir)..."

try {
  # Launch the codex wrapper (its own passes run concurrently internally) AND,
  # when enabled, the claude wrapper (one pass) CONCURRENTLY. -Tip is the pinned
  # SHA so review and merge target the same content even if $Branch moves. Each
  # child gets an explicit -OutDir into its run-owned subdir (its verdict artifacts
  # feed the post-merge audit + the forensics copy; the exit-1 corroboration that
  # had used this as a provenance boundary was removed with the 2026-07 contract).
  $codexArgs = @(
    '-ExecutionPolicy', 'Bypass', '-File', $trustedWrapper,
    '-Scope', 'Branch', '-Target', $Base, '-Tip', $branchSha,
    '-PromptPath', $trustedPrompt,
    '-ReviewPasses', "$($passMix.Codex)",
    '-OutDir', $runCodexDir,
    '-Title', "Pre-merge review (codex): $Branch ($branchSha) -> $Base"
  )
  # Pin the branch-review codex child to $mergeEffort (the gate's resolved
  # effective tier) whenever it is a known tier, so the review runs at a
  # deterministic, stamped tier rather than re-reading config. When $mergeEffort
  # is `unknown` (config unreadable), the child inherits codex's own default.
  if ($mergeEffort -ne 'unknown') {
    $codexArgs += @('-ReasoningEffort', $mergeEffort)
  }
  Write-Host "[auto-merge] launching codex review ($($passMix.Codex) pass(es), concurrent internally) ..."
  # Parent-directed follow-up staging (merge-gate BLOCKER 2026-07-10): each child
  # wrapper STAGES its QUALITY follow-ups at the CROSS_REVIEW_FOLLOWUPS_PENDING path instead
  # of appending them to the durable index -- this gate can still abort on the
  # exit union, the branch-pin verification, or the ff-merge itself AFTER a child
  # passes, and follow-ups for an unmerged branch must never reach the index.
  # Promotion happens below, only after the ff-merge succeeds. A child's env is
  # captured at process creation, so the per-child path is set immediately before
  # each launch and the variable is restored right after the launches.
  $savedFollowupsPending  = $env:CROSS_REVIEW_FOLLOWUPS_PENDING
  $codexFollowupsPending  = Join-Path $runCodexDir 'followups.pending.md'
  $claudeFollowupsPending = $null
  $env:CROSS_REVIEW_FOLLOWUPS_PENDING = $codexFollowupsPending
  # Launch via Start-ReviewChild: the wrapper runs in a thin runner that records
  # its $LASTEXITCODE to a run-owned exit FILE (the authoritative readback) and
  # whose process handle is handle-cached at spawn. Both close the PS 5.1
  # `Start-Process -PassThru` null-ExitCode quirk that, observed live,
  # printed an empty exit code and forced the gate to fail closed on EVERY merge.
  $codexChild = Start-ReviewChild -RunDir $runCodexDir -WrapperArgs $codexArgs -Utf8NoBom $utf8NoBom
  $codexProc = $codexChild.Proc

  $claudeProc = $null
  $claudeChild = $null
  if ($passMix.Claude -gt 0) {
    $claudeArgs = @(
      '-ExecutionPolicy', 'Bypass', '-File', $trustedClaudeWrapper,
      '-Scope', 'Branch', '-Target', $Base, '-Tip', $branchSha,
      '-PromptPath', $trustedPrompt,
      '-OutDir', $runClaudeDir,
      '-Title', "Pre-merge review (claude): $Branch ($branchSha) -> $Base"
    )
    Write-Host "[auto-merge] launching claude review (1 pass, cross-provider) CONCURRENTLY ..."
    $claudeFollowupsPending = Join-Path $runClaudeDir 'followups.pending.md'
    $env:CROSS_REVIEW_FOLLOWUPS_PENDING = $claudeFollowupsPending
    $claudeChild = Start-ReviewChild -RunDir $runClaudeDir -WrapperArgs $claudeArgs -Utf8NoBom $utf8NoBom
    $claudeProc = $claudeChild.Proc
  }
  # Both children captured their pending paths at launch; restore the caller's
  # value so THIS process's later git/PS activity never runs with a stale one.
  if ($null -eq $savedFollowupsPending) { Remove-Item Env:CROSS_REVIEW_FOLLOWUPS_PENDING -ErrorAction SilentlyContinue }
  else { $env:CROSS_REVIEW_FOLLOWUPS_PENDING = $savedFollowupsPending }

  # WAIT FOR EVERY LAUNCHED CHILD BEFORE branching on any result. The default
  # pass mix launches codex AND claude CONCURRENTLY, so an early `exit 3` taken
  # between the two waits (e.g. on an unreadable codex exit) would leave the
  # other child still running -- the finally block would then copy+delete the
  # scratch dirs while that child is mid-write, orphaning the process and racing
  # its artifacts. Joining both first makes cleanup and forensics deterministic
  # on every exit path. These are INTENTIONALLY unbounded waits: the gate's whole
  # job here is to wait for the review to finish, which legitimately takes many
  # minutes; an artificial timeout would abandon a still-valid review. WaitForExit
  # is idempotent and safe on an already-exited process. (A hung child would block
  # here indefinitely -- that is acceptable for an interactive gate the operator
  # is watching; the BOUNDED wait lives only in the finally's exception-path
  # defensive-join, where preserving the run dir on timeout matters.)
  if ($null -ne $codexProc)  { $codexProc.WaitForExit() }
  if ($null -ne $claudeProc) { $claudeProc.WaitForExit() }

  # Resolve BOTH exit codes from their run-owned exit FILES (the deterministic
  # source written by each runner) now that both children have exited. Cross-
  # cross-check each against the handle-cached process .ExitCode. The exit file is
  # a MUTABLE input (default-ACL $env:TEMP), so a readable handle value that
  # MISMATCHES the file value means the file cannot be trusted (tamper, or a
  # runner that exited differently than it recorded) -> Test-ChildExitConsistency
  # returns $false and we FAIL THE MERGE CLOSED before unioning. When no handle
  # value is readable (launch failure / PS .ExitCode quirk), there is nothing to
  # cross-check and the file's own {0,1,2,3} fail-closed validation stands.
  $codexExit = Resolve-ReviewChildExit -ExitFile $codexChild.ExitFile -BackendName 'codex'
  if (-not (Test-ChildExitConsistency -BackendName 'codex' -FileExit $codexExit -Proc $codexProc)) {
    Write-Host "[auto-merge] codex review exit-code failed the file-vs-process cross-check - merge ABORTED (fail closed)."
    exit 3
  }

  # Claude is resolved whenever it was SUPPOSED to run ($claudeChild exists, i.e.
  # $passMix.Claude -gt 0), NOT merely when its PROCESS launched. Start-ReviewChild
  # CATCHES a Start-Process launch failure and still returns its object with
  # Proc=$null and the pre-seeded '999' exit FILE, so $claudeChild is always
  # assigned when claude was intended -- and Resolve-ReviewChildExit reads that
  # '999' -> $null -> the branch below fails the merge closed. Gating on
  # $claudeProc instead would silently treat a required-but-failed-to-launch
  # Claude pass as neutral 0 and let the gate proceed Codex-only WITHOUT the
  # explicit -NoClaude decision (the exact silent-drop this guard prevents).
  # Neutral 0 is reserved for the genuinely DISABLED path ($claudeChild is $null).
  $claudeExit = 0   # neutral ONLY when claude is disabled (no claude contribution to the union)
  if ($null -ne $claudeChild) {
    $claudeExit = Resolve-ReviewChildExit -ExitFile $claudeChild.ExitFile -BackendName 'claude'
    if (-not (Test-ChildExitConsistency -BackendName 'claude' -FileExit $claudeExit -Proc $claudeProc)) {
      Write-Host "[auto-merge] claude review exit-code failed the file-vs-process cross-check - merge ABORTED (fail closed)."
      exit 3
    }
  }

  # NOW branch on the resolved results -- both children are joined, so every exit
  # path below leaves no orphan and the finally block consolidates+deletes
  # cleanly. A $null resolution (missing/empty/unparsable file, or a non-verdict
  # code such as the '999' crash sentinel) means that child never recorded a real
  # verdict -> fail closed, naming the file (diagnostic emitted in Resolve-...).
  if ($null -eq $codexExit) {
    Write-Host "[auto-merge] codex review produced no readable exit code - merge ABORTED (fail closed)."
    exit 3
  }
  Write-Host "[auto-merge] codex review exit code: $codexExit"

  # Branch on $claudeChild (intent to run claude), NOT $claudeProc (process
  # launched) -- a required Claude pass whose Start-Process failed must still hit
  # the null-exit fail-closed path below, never be silently skipped.
  if ($null -ne $claudeChild) {
    if ($null -eq $claudeExit) {
      Write-Host "[auto-merge] claude review produced no readable exit code - merge ABORTED (fail closed)."
      exit 3
    }
    Write-Host "[auto-merge] claude review exit code: $claudeExit"

    # Claude exit 3 is the Claude wrapper's GENERAL fail-closed code -- it is
    # raised for a configured-consistency-doc-in-diff (precompute not ported;
    # opt-in via CROSS_REVIEW_CONSISTENCY_DOC), an ancestor CLAUDE.md
    # trust-boundary breach, auth failure, malformed/empty output, and other
    # wrapper failures, NOT only for quota exhaustion. The merge gate sees only
    # the exit code and cannot distinguish a quota miss from a deliberate
    # fail-closed decision, so it does NOT substitute: a claude exit 3 ABORTS
    # the merge (fail closed). When the shared Claude token pool is genuinely
    # exhausted, the operator re-runs with -NoClaude (a positive, explicit
    # action) -- the pass mix then keeps total coverage at 3 with codex passes.
    # exit 0/2 ARE real verdicts and are unioned directly below (exit 1 is the retired
    # QUALITY code -- the union validation rejects it as unexpected and fails closed).
    if ($claudeExit -eq 3) {
      Write-Host "[auto-merge] claude review failed closed (exit 3) - merge ABORTED."
      Write-Host "[auto-merge] If the Claude token pool is exhausted, re-run with -NoClaude; otherwise fix the cited fail-closed condition (consistency-doc/CLAUDE.md/auth/malformed)."
      exit 3
    }
  }

  # Forensics: run-owned verdict artifacts are copied into the centralized shared
  # logs in the finally block below (both backends' artifacts feed the top-level
  # trend analyzer, which scans both log dirs by default) so they are preserved on
  # EVERY exit path -- including the Claude exit-3, codex exit-3, the union-validation
  # fail-closed (a stray non-{0,2} code, e.g. a retired exit 1), and the BLOCKER switch
  # exit (exit 2), all of which leave this try before reaching the ff-merge and would
  # otherwise have their artifacts deleted by finally uncopied. (There is no QUALITY
  # switch exit under the 2026-07 contract -- a QUALITY-only verdict exits 0 from the
  # child wrapper and proceeds through the PASS branch.) Non-fatal to
  # the MERGE DECISION (that is made below against the run-owned dirs, never the
  # shared logs), but the consolidation itself is fail-LOUD -- a copy failure is
  # reported visibly and names what was lost (see Copy-RunArtifactsToCentralLogs),
  # because a silently-missing forensic trail is the bug this hardening removed.

  # Union the codex and claude exits: worst severity wins.
  # exit 3 (either side) -> fail closed; any 2 -> BLOCKED; else 0. (Claude exit 3
  # already failed the gate closed above; a codex exit 3 here still fails the whole
  # gate closed.) Under the 2026-07 severity contract QUALITY is NON-BLOCKING: the
  # wrappers exit 0 for a QUALITY-only verdict, so there is no exit-1 union to honor
  # -- only a BLOCKER (exit 2) aborts the merge. Each child wrapper STAGES its
  # QUALITY-bearing PASS's follow-ups (CROSS_REVIEW_FOLLOWUPS_PENDING); they are promoted
  # to logs/review-followups.md only after the ff-merge succeeds.
  if ($codexExit -eq 3) {
    Write-Host "[auto-merge] codex review invocation failed (exit 3) - merge ABORTED (fail closed)."
    exit 3
  }
  # Validate BOTH backend exits are in the known verdict set {0,2} and union them via
  # the pure, SelfTest-covered Resolve-MergeUnionExit. Claude exit 3 already fails
  # closed above and codex exit 3 fails closed just above -- but ANY OTHER value (a
  # child killed by Ctrl-C, a native crash, a parse failure with an unusual code, a
  # legacy/pre-contract wrapper still returning the RETIRED exit 1, or future code 4)
  # is NOT a valid verdict and must NOT collapse to a passing union; the helper fails
  # closed (Exit 3) on it (a stray exit 1 included -- no longer a valid QUALITY signal).
  $union = Resolve-MergeUnionExit -CodexExit $codexExit -ClaudeExit $claudeExit
  if ($union.Exit -eq 3) {
    Write-Host "[auto-merge] $($union.Diagnostic) - merge ABORTED (fail closed)."
    exit 3
  }
  $unionExit = $union.Exit

  Write-Host ""
  Write-Host "[auto-merge] union review verdict: $unionExit (codex=$codexExit, claude=$claudeExit)"

  switch ($unionExit) {
    0 {
      Write-Host "[auto-merge] PASS - proceeding with ff-merge (verdicts may contain informational NOTE and/or non-blocking QUALITY findings; the wrappers exit 0 for CLEAN, NOTE-only, and QUALITY-only, and each QUALITY-bearing PASS's staged follow-ups are promoted to logs/review-followups.md after the ff-merge succeeds)."
    }
    2 {
      Write-Host "[auto-merge] BLOCKER findings - merge ABORTED."
      Write-Host "[auto-merge] Fix the findings and re-run."
      exit 2
    }
    default {
      Write-Host "[auto-merge] Unexpected union review state (exit $unionExit) - merge ABORTED."
      exit 3
    }
  }

  # Verify the branch ref still points at the pinned SHA. If anything moved
  # the branch between review and merge, abort - what was reviewed is not
  # what would be merged.
  $branchShaNow = (& git rev-parse $Branch).Trim()
  if ($branchShaNow -ne $branchSha) {
    Write-Host "[auto-merge] ERROR: '$Branch' moved during review."
    Write-Host "[auto-merge] reviewed=$branchSha, now=$branchShaNow"
    Write-Host "[auto-merge] Re-run the merge against the current branch tip."
    exit 3
  }

  Write-Host "[auto-merge] ff-merge $branchSha (=$Branch) -> '$Base'"
  & git merge --ff-only $branchSha
  if ($LASTEXITCODE -ne 0) {
    Write-Host "[auto-merge] ERROR: ff-merge failed"
    exit 3
  }

  $newHead = (& git rev-parse --short=12 HEAD).Trim()
  Write-Host "[auto-merge] OK - '$Base' is now at $newHead"

  # The merge is applied -- NOW the children's staged QUALITY follow-ups may
  # reach the durable index (they describe content that actually landed).
  Publish-ReviewFollowups -PendingPath $codexFollowupsPending -BackendName 'codex'
  if (-not [string]::IsNullOrWhiteSpace($claudeFollowupsPending)) {
    Publish-ReviewFollowups -PendingPath $claudeFollowupsPending -BackendName 'claude'
  }
  exit 0
}
catch {
  Write-Host "[auto-merge] EXCEPTION: $_"
  exit 3
}
finally {
  # This block runs on EVERY exit path (the try body returns normally, every
  # `exit N` -- PowerShell runs finally before the process exits -- AND the
  # catch). Order within it: (1) bounded defensive-join of any still-running
  # child, then (2) UNCONDITIONAL consolidation copy, then (3) a GUARDED delete.
  # The copy ALWAYS runs before any Remove-Item (copy-before-delete); the DELETE
  # is conditional (skipped when a review child is still running or consolidation
  # could not centralize -- see below). So copy is unconditional, delete is
  # conditional, and a child is joined before either. This is the forensic-
  # integrity guarantee a sibling session's abort lacked (its
  # run dir was reaped with multiple verdicts uncopied).
  #
  # DEFENSIVE JOIN: the normal flow already WaitForExit()s both children before
  # any exit, but an exception thrown BETWEEN launching a child and joining it
  # (or before the join is reached) could land here with a child still running --
  # deleting $runDir under it would race/lose its in-flight verdict write. So
  # re-join any still-live launched child FIRST. A review pass can run many
  # minutes, so the bound is generous (matching the gate's own long-review
  # reality); an unbounded wait in finally must still be avoided. CRITICALLY, the
  # timeout RESULT is captured: if a child is STILL running after the wait, the
  # run dir is NOT deleted below (deleting under a live child is the exact race
  # this change closes) -- it is PRESERVED for forensics and to let the child
  # finish writing, with a loud stable warning. $codexProc/$claudeProc are $null
  # on an early exit before launch, in which case there is nothing to join.
  $childStillRunning = $false
  foreach ($pj in @(@{ N = 'codex'; P = $codexProc }, @{ N = 'claude'; P = $claudeProc })) {
    if ($null -eq $pj.P) { continue }
    # Whole check wrapped: a HasExited/WaitForExit access on an odd host state
    # must NEVER throw out of finally (that would skip the consolidation below).
    # $observedRunning is initialized CONSERVATIVELY to $true (assume the child is
    # running until we PROVE it has exited) and flips to $false ONLY on a confirmed
    # exit -- HasExited returning true, or WaitForExit(bound) returning true. So a
    # throw from HasExited OR WaitForExit leaves it $true and the dir is PRESERVED:
    # on ANY uncertainty about the child's state we never delete under a
    # possibly-live child. (The earlier version initialized it $false and only
    # flipped true after HasExited returned false, so a HasExited throw wrongly
    # left it $false and allowed deletion -- fixed.)
    $observedRunning = $true
    try {
      if ($pj.P.HasExited) {
        $observedRunning = $false   # confirmed already exited before the wait
      } else {
        Write-Host "[auto-merge] forensics: $($pj.N) review child still running at cleanup - joining (bounded $([int](600000/60000))min) before consolidation to avoid racing its verdict write."
        if ($pj.P.WaitForExit(600000)) { $observedRunning = $false }   # confirmed done within the bound
      }
    } catch {
      # Left $observedRunning = $true: the child's state could not be verified, so
      # preserve conservatively (never delete under a possibly-live child).
    }
    if ($observedRunning) {
      Write-Host "[auto-merge] forensics WARNING: $($pj.N) review child STILL running (or its state is unverifiable) after the bounded wait - PRESERVING the run dir (NOT deleting) so its in-flight verdict is not lost; manually inspect/reap '$runDir' later."
      $childStillRunning = $true
    }
  }

  # $preserveRunDir is the OR of every reason NOT to delete the run dir: a still-
  # running child (above) AND a consolidation that could not safely complete
  # (Copy-RunArtifactsToCentralLogs returns $true when it could not centralize --
  # unresolvable central dir, enumeration failure, or a per-file copy failure).
  # On uncertainty we PRESERVE rather than silently delete artifacts.
  $preserveRunDir = $childStillRunning

  # Copy-RunArtifactsToCentralLogs is fail-LOUD internally (names lost files) and
  # returns whether the run dir must be preserved; this extra try/catch only
  # guarantees finally never THROWS (which would mask the script's real exit
  # code) -- and on such a throw we PRESERVE (we cannot confirm consolidation
  # succeeded). $includeClaude is the top-level always-in-scope flag (vs
  # $claudeProc, which is $null on an early exit).
  try {
    if (Copy-RunArtifactsToCentralLogs -RunBackendDir $runCodexDir -Backend 'codex') { $preserveRunDir = $true }
    if ($includeClaude) {
      if (Copy-RunArtifactsToCentralLogs -RunBackendDir $runClaudeDir -Backend 'claude') { $preserveRunDir = $true }
    }
  } catch {
    Write-Host "[auto-merge] forensics WARNING: centralizing run artifacts in finally threw ($($_.Exception.Message)) - PRESERVING the run dir since consolidation could not be confirmed; artifacts for this run may be INCOMPLETE in the central logs."
    $preserveRunDir = $true
  }
  # The trusted-infra dir holds the prompt + wrapper copies a LIVE child still
  # reads (codex/claude wrappers receive -PromptPath $trustedPrompt and read it
  # mid-review), so it MUST be preserved while any child is still running --
  # deleting it under a live child would break that child's in-flight review and
  # lose its verdict. It carries no verdict artifacts, so a copy failure alone
  # (run dir preserved but child exited) does NOT require keeping it; gate it on
  # $childStillRunning specifically.
  if ($childStillRunning) {
    Write-Host "[auto-merge] forensics: trusted-infra dir '$trustedDir' PRESERVED (a review child was still running and may still read its -PromptPath); not auto-deleted this run."
  } else {
    Remove-TreeNoRecurse -Path $trustedDir
  }
  # Delete the run dir ONLY after consolidation AND only when nothing needs
  # preserving (no live child writing into it, and consolidation completed
  # cleanly). Otherwise PRESERVE it -- artifacts must never be deleted from under
  # a live child or lost when they could not be centralized.
  if ($preserveRunDir) {
    $reason = if ($childStillRunning) { 'a review child was still running at cleanup' } else { 'consolidation could not centralize all verdict artifacts' }
    Write-Host "[auto-merge] forensics: run dir '$runDir' PRESERVED ($reason); it is not auto-deleted this run - inspect/reap it manually once resolved."
  } else {
    Remove-TreeNoRecurse -Path $runDir
  }
}
