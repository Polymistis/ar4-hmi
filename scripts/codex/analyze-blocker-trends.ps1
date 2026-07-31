# Cross-review verdict trend analyzer.
#
# Reads verdict markdown files from one or more reviews directories
# (default: BOTH logs/codex/reviews AND logs/claude/reviews so the
# report aggregates findings across both reviewer backends), clusters
# BLOCKER (and QUALITY / legacy NON-BLOCKER / NOTE) findings by
# archetype keyword, and emits a markdown report enumerating the most-
# frequent classes with example cites.
#
# PURPOSE (read this before reading the report):
# The report exists to reduce REPEAT-CLASS BLOCKERs by improving worker
# discipline upstream. It is NOT a review-throughput metric. Use it to
# identify archetypes worth folding into worker dispatch templates,
# AGENTS.md, the review prompt's hazards section, or an anti-pattern
# guide. The output does not contain any cycle-count or time-to-merge
# statistics by design.
#
# Cross-backend aggregation: the default scans both backends because
# defect-class archetypes are properties of the codebase, not of which
# reviewer caught them; aggregating across both reviewer backends gives
# the most complete picture for the worker-discipline-improvement goal.
# Pass a single -ReviewsDir to slice per-backend when needed.
#
# Usage:
#   scripts\codex\analyze-blocker-trends.ps1                              # writes docs/blocker-trends.md (both backends by default)
#   scripts\codex\analyze-blocker-trends.ps1 -OutPath foo.md              # custom output path
#   scripts\codex\analyze-blocker-trends.ps1 -SinceDays 30                # only verdicts whose mtime is within N days
#   scripts\codex\analyze-blocker-trends.ps1 -ReviewsDir logs/codex/reviews    # codex-only slice
#   scripts\codex\analyze-blocker-trends.ps1 -ReviewsDir logs/claude/reviews   # claude-only slice
#   scripts\codex\analyze-blocker-trends.ps1 -ReviewsDir <p1>,<p2>        # explicit multi-dir
#
# Exit codes:
#   0 = success, report written
#   1 = fail-closed refusal to write a report. Either an invocation failure (no
#       configured reviews dir exists, no verdict files found across the existing
#       dirs, writer failure, etc.) OR a corpus-integrity abort: one or more
#       scanned verdict files are GATE-FAILED-CLOSED or malformed (per
#       Get-VerdictSuspectReason), which would silently drop or miscount evidence,
#       so the run aborts before overwriting the report.

[CmdletBinding()]
param(
  # Accepts one or more reviews directories. Default scans both backend
  # log dirs so the aggregated report covers the full cross-review
  # picture. A non-existent path in the list is skipped (so a single-
  # backend install still works on the default); the script fails
  # closed only when ZERO configured paths exist.
  [string[]]$ReviewsDir = @('logs/codex/reviews', 'logs/claude/reviews'),
  [string]$OutPath = 'docs/blocker-trends.md',
  # 0 = no cutoff (full history). Negative values are rejected at the
  # parameter binding boundary so a typo like `-SinceDays -1` cannot
  # silently fall through to full-history mode.
  [ValidateRange(0, [int]::MaxValue)]
  [int]$SinceDays = 0,
  # SelfTest mode: runs the in-memory SelfTest fixtures (the fixture block below
  # is the authoritative inventory) plus an end-to-end fail-closed
  # check that creates a throwaway temp dir under $env:TEMP, writes a bogus
  # markdown file into it, re-invokes this script with that dir, and asserts the
  # analyzer rejects the input. Touches the filesystem under $env:TEMP only --
  # never the repo, never the network, never git. Exits before the main report-
  # generation pipeline. Same SelfTest discipline as
  # scripts/codex/auto-review.ps1 -Scope SelfTest.
  [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'
$utf8NoBom = [System.Text.UTF8Encoding]::new($false)

# ---------------------------------------------------------------------------
# Archetype classifier.
# ---------------------------------------------------------------------------
# Each archetype has a friendly name + a list of keyword/regex patterns.
# A finding is classified by walking the archetype list IN ORDER and
# assigning it to the FIRST one whose pattern set matches. Order is
# most-specific to least-specific so the more-targeted archetype wins
# when multiple patterns hit (this mirrors the prompt template's
# "one primary category per finding" rule -- the analyzer uses the same
# discipline so the trend report is comparable across review periods).
#
# Patterns are case-insensitive regex. A finding line is the one-line
# summary text the reviewer emitted (the line starting with the
# severity prefix, e.g. "BLOCKER: foo.rs:10 - dropped Option").
# Either backend (codex or claude) produces this same shape per the
# shared verdict format.
#
# UNCLASSIFIED is the catch-all. A populated UNCLASSIFIED bucket in the
# report is a signal that the keyword list needs to grow.

$archetypes = @(
  # WORKFLOW-INFRA first: a finding citing `scripts/codex/...`,
  # `auto-review`, `auto-merge`, or `AGENTS.md` is ABOUT the review
  # system itself, even if the broken behavior happens to involve PLAN
  # text or asset paths. Putting this before PLAN-DRIFT keeps
  # "review-wrapper bug that misclassifies PLAN checkboxes" categorized
  # as a workflow-infra defect rather than as PLAN drift -- the actual
  # PLAN.md is fine; the wrapper is the broken artifact.
  @{
    Name = 'WORKFLOW-INFRA'
    Patterns = @(
      'scripts/codex/',
      'scripts/claude/',
      'scripts/git-hooks/',
      'bootstrap\.ps1',
      'auto-review',
      'auto-merge',
      'pre-commit',
      '\bcore\.hooksPath\b',
      '\bhooksPath\b',
      # Gate dispatcher ONLY -- qualified so a generic application "event
      # dispatcher" finding does not land in WORKFLOW-INFRA (which is first-match).
      # "pre-commit dispatcher" also matches 'pre-commit' above; "per-clone
      # dispatcher" is the non-redundant gate phrasing. A bare `\bdispatcher\b`
      # over-matched ordinary dispatcher findings.
      '(?:pre-commit|per-clone)\s+dispatcher',
      'commit wrapper',
      'commit-wrapper',
      'git-config',
      'review-prompt-template',
      'AGENTS\.md',
      # Named review-system documentation files. NOT a wildcard `docs/`
      # prefix -- that would over-match unrelated project docs. Each
      # entry pins one specific known review-infra doc. Add your own
      # project's review-system doc filenames here as they are produced;
      # the UNCLASSIFIED bucket surfaces unmapped ones (fail-loud).
      'docs/audit-protocol',
      'docs/blocker-trends'
    )
  },
  @{
    Name = 'PLAN-DRIFT'
    Patterns = @(
      'PLAN\.md',
      '\bmilestone\b',
      '\bcheckbox\b',
      'range claim',
      'status[ -]table',
      'sequencing',
      '\bM\d+\.\d+'
    )
  },
  @{
    Name = 'SILENT-FAILURE'
    Patterns = @(
      '\bdropped\b',
      '\bsilently\b',
      '\bsuppressed\b',
      '\bignored\b',
      'exits\s+0',
      'exits successfully',     # real verdict phrasing for --check returning 0 on bad input
      'false[- ]green',
      '\bno-op\b',
      'accepted\b.*\binvalid\b',
      'silently receives',
      'silently load',
      'missing diagnostic',
      'unwrap_or_default',
      # Audit-added patterns (UNCLASSIFIED reduction):
      'broken fallback',
      'fallback cannot',         # "fallback cannot classify ..."
      'lost.*playback',          # cues that silently lost audible fallback
      'reports?.*\bPASS\b',      # test instrumentation reporting false PASS
      'warning-only'             # "warning-only and skipped"
    )
  },
  @{
    Name = 'TOMBSTONE-OR-SHIM'
    Patterns = @(
      '\btombstone\b',
      'removed[- ]code',
      'renamed[- ]to[- ]underscore',
      'empty quarantine',
      '\bformerly\b',
      'keep for compat',
      '\bdead\s+code\b'
    )
  },
  # Audit-added archetype: STALE-DOC.
  # Distinct from TOMBSTONE-OR-SHIM (which is residue of removed code) --
  # STALE-DOC is documentation/comments/help-text describing OLD behavior
  # of code that has since changed. Ordering in the $archetypes array:
  # WORKFLOW-INFRA -> PLAN-DRIFT -> SILENT-FAILURE -> TOMBSTONE-OR-SHIM ->
  # STALE-DOC -> CROSS-CRATE-CONTRACT -> LOADER-OR-ASSET-EDGE -> CONVENTION.
  # STALE-DOC sits AFTER TOMBSTONE-OR-SHIM so a finding that
  # names both stays as TOMBSTONE (closer to the real defect), and AFTER
  # SILENT-FAILURE so a finding that mentions both stays as SILENT-FAILURE
  # (the silent behavior is the load-bearing defect, not the stale wording).
  # Placed BEFORE CROSS-CRATE-CONTRACT and the loader/asset/convention
  # buckets so the "still describes/says/claims" markers win over generic
  # validator/loader keywords when the finding is fundamentally a stale-doc.
  @{
    Name = 'STALE-DOC'
    Patterns = @(
      'still describes',
      'still says',
      'still claim',
      'still claims',
      'still names',
      'still documents',
      'documentation says',
      'docs still',
      'comment still',
      'comments still',
      'help still',
      '\bStale\b.*\b(documentation|doc|comment|docstring)\b'
    )
  },
  @{
    Name = 'CROSS-CRATE-CONTRACT'
    Patterns = @(
      'accepts\b.*\bdocument',
      'no longer re-exports?',
      '\bvalidator\b',
      'request shape',
      'cross[- ]crate',
      'docstring.*reject',
      'system[- ]set ordering'
    )
  },
  @{
    Name = 'LOADER-OR-ASSET-EDGE'
    Patterns = @(
      # Keep these aligned with the LOADER-OR-ASSET-EDGE surface vocabulary in
      # review-prompt-template.md and dispatch-checklist.ps1 (binary-format
      # header/length checks, identity-fallback on invalid input, asset-root
      # env-var fall-through, format-invariant drops) so a finding worded from
      # the prompt does not fall into UNCLASSIFIED.
      '\bbinary[- ]asset\b',
      '\bbinary[- ]format\b',
      'header[- ]length',
      'header-declared',
      'format[- ]invariant',
      'identity[- ]fallback',
      'fallback to identity',
      '\bbind[- ]matrix\b',
      'ASSET_ROOT',
      'asset[- ]root',
      '\bloader\b',
      'asset_repair',
      'asset.*texture',
      '\bjoint\b',
      'singular.*matrix',
      # Audit-added patterns (UNCLASSIFIED reduction):
      'nonexistent.*clip',       # animation/audio clip referenced but absent
      'nonexistent.*asset',
      'missing.*clip',
      'missing.*asset reference'
    )
  },
  @{
    Name = 'CONVENTION'
    Patterns = @(
      'CLAUDE\.md',
      'backwards[- ]compat',
      'comment.*explains',
      'mechanic[- ]explaining',
      'error handling.*impossible',
      'unwrap at boundary',
      # Audit-added patterns (UNCLASSIFIED reduction):
      '\blint gate\b',           # "trips the lint gate"
      'static[- ]analysis'
    )
  }
)

function Get-FindingArchetype {
  param([string]$Text)
  $lower = $Text  # patterns are case-insensitive; no need to lower
  foreach ($a in $archetypes) {
    foreach ($p in $a.Patterns) {
      if ($lower -match "(?i)$p") {
        return $a.Name
      }
    }
  }
  return 'UNCLASSIFIED'
}

# Max category-count the wrapper's `\d+` tail tolerates. Mirror of the same
# constant in scripts/codex/dispatch-checklist.ps1 -- both scripts consume the
# same verdict logs and must apply the same category-tail bound so they agree on
# which verdicts are trustworthy evidence.
$script:CategoryCountMax = 10000

# Classify a category line's tail as none|count|invalid, the SAME way the
# wrapper's `:\s*(?<v>none|\d+)\s*$` regex matches it. BYTE-IDENTICAL to the
# helper in scripts/codex/dispatch-checklist.ps1 -- keep the two copies in sync
# (the analyzer's suspect check and the checklist's trustworthiness gate must
# classify tails identically).
function Get-CategoryTailKind {
  param([string]$Tail)
  $t = $Tail.Trim()
  # CASE-SENSITIVE `none` compare (-ceq): the wrapper's contract regex is
  # `:\s*(?<v>none|\d+)\s*$` (case-sensitive), so `None` / `NONE` are NOT the
  # `none` sentinel -- they fall through to 'invalid', matching how the wrapper
  # treats a non-`none|\d+` tail. A case-insensitive -eq would wrongly accept
  # `None` as a zero-count category and admit a wrong-case tail as clean evidence.
  if ($t -ceq 'none') { return @{ Kind = 'none'; Value = 0 } }
  # Require a DIGITS-ONLY shape (on the already-trimmed tail) before TryParse,
  # classifying a tail the same way the wrapper's `:\s*(?<v>none|\d+)\s*$` regex
  # matches it. NOTE leading/trailing WHITESPACE is NOT a difference: the wrapper
  # allows `\s*` around the value and trims, and $t is `$Tail.Trim()`, so ` 1 `
  # is a valid count to BOTH. Only a SIGNED or otherwise non-`\d+` tail (e.g.
  # `+1`, `1.0`, `1a`) fails `^\d+$` here AND fails `(?<v>none|\d+)` in the
  # wrapper -- the wrapper then treats the line as a non-matching category line
  # (counts toward neither `none` nor a digit count).
  $parsed = 0
  if ($t -match '^\d+$' -and [int]::TryParse($t, [ref]$parsed) -and $parsed -le $script:CategoryCountMax) {
    return @{ Kind = 'count'; Value = $parsed }
  }
  return @{ Kind = 'invalid'; Value = 0 }
}

# ---------------------------------------------------------------------------
# Fail-closed per-file suspect check: returns a human-readable REASON string
# when the verdict file is NOT trustworthy review evidence, else $null. The
# analyzer calls this BEFORE parsing each verdict file so a truncated,
# fail-closed, or malformed verdict is quarantined and NAMED, never silently
# counted or dropped into the trend report. This is the SUPERSET bar
# dispatch-checklist (Get-VerdictStructure / Test-VerdictBlock) applies, not just
# the wrapper's (Get-VerdictExitCode): it rejects everything the wrapper fails
# closed on (exit 3, possibly banner-less) AND the STRICTER-than-wrapper shapes
# dispatch-checklist rejects -- over/underfilled category ROW blocks, malformed
# category TAILS (`+1`, `1.0`), and WRONG-CASE severity prefixes (`blocker:`) --
# because both scripts consume the same
# verdict logs and must agree on which files are trustworthy evidence (a file one
# rejects but the other counts would desync the corpus). $VerdictWordRe is the
# parse loop's own
# verdict-line regex (its `(?<v>)` group yields the well-formed verdict WORD);
# the coherence check extracts the word through it, so the suspect check and the
# parse loop share ONE regex and cannot drift.
# ---------------------------------------------------------------------------
function Get-VerdictSuspectReason {
  param([string]$Content, [regex]$VerdictWordRe)
  # Case policy MIRRORS the wrappers (and dispatch-checklist), which is ASYMMETRIC:
  #  - The VERDICT line + word and the GATE FAILED CLOSED banner are matched CASE-
  #    INSENSITIVELY. The wrappers detect the verdict with `-match '^VERDICT:'` and
  #    compare the word with case-insensitive `-ne` (auto-review.ps1
  #    Get-VerdictExitCode), so `verdict: clean` is an ACCEPTED clean verdict --
  #    quarantining it here would wrongly diverge from a wrapper-accepted log.
  #  - SEVERITY prefixes and CATEGORY lines / `none` are matched CASE-SENSITIVELY
  #    (the wrapper counts `^BLOCKER:` and `^<CAT>:\s*(none|\d+)` case-sensitively);
  #    a WRONG-CASE severity prefix is quarantined by the scan below as a finding
  #    the case-sensitive parser would silently under-count.
  if ($Content -match '(?m)^=====\s*GATE FAILED CLOSED\s*=====') {
    return 'GATE FAILED CLOSED banner (fail-closed evidence)'
  }
  if ($Content -notmatch '(?m)^VERDICT:') {
    return 'no VERDICT: line (truncated / missing verdict -- the wrapper and dispatch-checklist fail closed)'
  }
  # Reject WRONG-CASE severity prefixes anywhere: a column-0 line like `blocker:` /
  # `Quality:` matches a severity NAME case-insensitively but NOT the exact-case
  # `^(BLOCKER|QUALITY|NON-BLOCKER|NOTE):` that the wrapper and this analyzer's
  # $severityRe count with, so it is INVISIBLE to finding-counting -- a real finding
  # written (or corrupted) in the wrong case would leave a `VERDICT: CLEAN` verdict
  # falsely clean AND pollute the corpus with an uncounted finding. Fail closed on
  # any such line rather than trust a block the case-sensitive parser under-counts.
  foreach ($sm in ([regex]'(?im)^(?<sev>BLOCKER|QUALITY|NON-BLOCKER|NOTE):').Matches($Content)) {
    $sev = $sm.Groups['sev'].Value
    if ($sev -cne $sev.ToUpperInvariant()) {
      return "a severity-prefix line uses non-uppercase '$sev`:' -- the wrapper's case-sensitive parser under-counts it, so a real finding could hide under a falsely-clean verdict (untrustworthy)"
    }
  }
  # Every `=====` marker must be a well-formed REVIEW PASS marker (the banner is
  # handled above), AND the pass SEQUENCE must be complete: each n/N within the
  # producer's 1..10 range, n <= N, unique pass numbers, one agreed total N, and
  # exact 1..N coverage. An unknown marker (which could hide findings after it),
  # or a truncated / duplicate / mismatched-total sequence, is malformed evidence
  # (dispatch-checklist Get-VerdictStructure enforces the same; a lone
  # `REVIEW PASS 1/2` block is a silently dropped pass).
  $passMarkerRe = [regex]'^===== REVIEW PASS (?<n>\d+)/(?<total>\d+) =====\s*$'
  $passNums = @{}
  $passTotals = New-Object 'System.Collections.Generic.HashSet[int]'
  $passMarkerCount = 0
  foreach ($mk in [regex]::Matches($Content, '(?m)^=====.*=====\s*$')) {
    $pm = $passMarkerRe.Match($mk.Value)
    if (-not $pm.Success) {
      return 'an unrecognized `===== ... =====` marker (not a REVIEW PASS marker) -- malformed evidence'
    }
    $passMarkerCount++
    $n = 0; $tot = 0
    if (-not [int]::TryParse($pm.Groups['n'].Value, [ref]$n) -or $n -lt 1 -or $n -gt 10 -or `
        -not [int]::TryParse($pm.Groups['total'].Value, [ref]$tot) -or $tot -lt 1 -or $tot -gt 10) {
      return 'a REVIEW PASS marker n/N is outside the producer 1..10 range -- malformed evidence'
    }
    if ($n -gt $tot) { return 'a REVIEW PASS marker has n > N -- malformed evidence' }
    if ($passNums.ContainsKey($n)) { return 'a REVIEW PASS marker number is duplicated -- malformed evidence' }
    $passNums[$n] = 1
    [void]$passTotals.Add($tot)
  }
  if ($passMarkerCount -gt 0) {
    if ($passTotals.Count -ne 1) { return 'REVIEW PASS markers disagree on the total N -- malformed evidence' }
    $declaredTotal = @($passTotals)[0]
    if ($passNums.Count -ne $declaredTotal) { return "REVIEW PASS sequence is truncated ($($passNums.Count) of $declaredTotal passes present) -- malformed evidence" }
    for ($p = 1; $p -le $declaredTotal; $p++) {
      if (-not $passNums.ContainsKey($p)) { return "REVIEW PASS $p of $declaredTotal is missing -- malformed evidence" }
    }
  }
  # Split on the pass markers (as the wrapper / dispatch-checklist do). When
  # markers are present the FIRST segment is the leading provenance-header
  # fragment (DIFF-SHA256/REVIEW-* lines, no VERDICT): it must contain ONLY those
  # header lines + blanks. A NO-marker artifact (the Claude single-verdict shape)
  # is ONE pass block. Each non-header block must have EXACTLY ONE VERDICT line,
  # and if that word is malformed the block must carry a BLOCKER (precedence).
  $headerLineRe = [regex]'^(DIFF-SHA256:|REVIEW-TREE-OID:|REVIEW-BACKEND:|REVIEW-EFFORT:|REVIEW-SEVERITY-CONTRACT:)'
  $lineVerdictRe = [regex]'(?im)^VERDICT:[^\r\n]*'   # case-INSENSITIVE, mirroring the wrappers' `-match '^VERDICT:'`
  # Category-block validation is UNCONDITIONAL: EVERY verdict-shaped artifact the
  # analyzer counts MUST carry the complete eight-category block, because the
  # wrapper (Get-VerdictExitCode) and dispatch-checklist (Test-VerdictBlock) both
  # reject a categoryless verdict (exit 3 / not-clean-evidence). There is NO
  # pre-category legacy exemption: this package shipped WITH the eight-category
  # format, so a categoryless `VERDICT: CLEAN` / `NON-BLOCKING` / `BLOCKER`
  # artifact is malformed (truncated or corrupt), NOT a legit historical log the
  # wrapper never saw, and must be quarantined -- admitting it would let evidence
  # the other two log-consumers reject enter the trends corpus. (An earlier port
  # carried a `$categoryEra` header/category-presence exemption; the gate flagged
  # it as admitting current malformed logs, so it was removed.)
  $catNames = @('PLAN-DRIFT','SILENT-FAILURE','TOMBSTONE-OR-SHIM','CROSS-CRATE-CONTRACT','LOADER-OR-ASSET-EDGE','CONVENTION-ADHERENCE','TEST-QUALITY','DOC-VS-CODE-DRIFT')
  # Loose category-header regex (ANY tail) shared by the invalid-tail scan below
  # and the per-block row-balance check further down. The per-category cardinality
  # check inside the segment loop uses the STRICT `none|\d+` form instead.
  $catExactRe = [regex]'(?m)^(?<cat>PLAN-DRIFT|SILENT-FAILURE|TOMBSTONE-OR-SHIM|CROSS-CRATE-CONTRACT|LOADER-OR-ASSET-EDGE|CONVENTION-ADHERENCE|TEST-QUALITY|DOC-VS-CODE-DRIFT):\s*(?<rest>.*)$'
  # [STRICTER-than-wrapper, matches dispatch-checklist Get-VerdictStructure]: NO
  # category line may carry a malformed tail (a signed / non-`\d+` value like `+1`,
  # `1.0`, `1a`, or an over-max count). The wrapper's `none|\d+` count regex is
  # BLIND to such a tail (it sees no valid count line and can return CLEAN, not
  # exit 3), but a malformed tail means the artifact is an untrustworthy DATA
  # SOURCE -- dispatch-checklist rejects it (Get-CategoryFindings would misparse
  # it), so the analyzer must reject it too or the two log-consumers disagree on
  # the corpus. Caught here as a whole-content scan (an EXTRA malformed tail
  # alongside a valid `<cat>: none` line passes the per-category cardinality check,
  # which counts only the strict-form line).
  foreach ($ctm in $catExactRe.Matches($Content)) {
    if ((Get-CategoryTailKind -Tail $ctm.Groups['rest'].Value).Kind -eq 'invalid') {
      return "a category line has a malformed tail (not none|<digits<=$($script:CategoryCountMax)>) -- untrustworthy evidence (dispatch-checklist rejects it as a misparseable data source)"
    }
  }
  $segments = [regex]::Split($Content, '(?m)^=====.*=====\s*$')
  for ($i = 0; $i -lt $segments.Count; $i++) {
    $blk = $segments[$i]
    if ($segments.Count -gt 1 -and $i -eq 0) {
      foreach ($ln in ($blk -split "`r?`n")) {
        if ([string]::IsNullOrWhiteSpace($ln)) { continue }
        if (-not $headerLineRe.IsMatch($ln)) {
          return 'the leading header fragment (before the first REVIEW PASS marker) has a non-header line -- a finding/verdict hidden there is malformed evidence'
        }
      }
      continue
    }
    $verdictLines = $lineVerdictRe.Matches($blk)
    if ($verdictLines.Count -ne 1) {
      return "a review-pass block has $($verdictLines.Count) VERDICT: line(s) (exactly one required -- the wrapper and dispatch-checklist fail closed on zero or duplicate)"
    }
    # Per-block verdict-word / severity COHERENCE (mirrors Get-VerdictExitCode):
    # BLOCKER present -> BLOCKED (precedence; any word, incl. malformed, is kept);
    # else QUALITY/NON-BLOCKER/NOTE present -> the word must be NON-BLOCKING; else
    # zero findings -> the word must be CLEAN. A VALID word inconsistent with the
    # findings is wrapper exit-3 evidence -- and the Claude wrapper writes it
    # banner-less (claude/auto-review.ps1 write-then-exit-3), so coherence is the
    # ONLY thing that catches a claude reject.
    # CASE-SENSITIVE severity-prefix checks (-cmatch): a lowercase `blocker:` /
    # `quality:` is NOT a finding to the wrapper's case-sensitive severity parser,
    # so it must not count as one here -- otherwise a lowercase-prefix verdict
    # passes coherence but the downstream case-sensitive extractor records zero
    # findings, and a malformed file is miscounted instead of quarantined.
    $blkHasBlocker = ($blk -cmatch '(?m)^BLOCKER:')
    if (-not $blkHasBlocker) {
      $vw = $null
      $vwm = $VerdictWordRe.Match($blk)
      if ($vwm.Success) { $vw = $vwm.Groups['v'].Value }
      if (($blk -cmatch '(?m)^QUALITY:') -or ($blk -cmatch '(?m)^NON-BLOCKER:') -or ($blk -cmatch '(?m)^NOTE:')) {
        if ($vw -ne 'NON-BLOCKING') {
          return 'a review-pass block has QUALITY/NON-BLOCKER/NOTE findings but its VERDICT is not NON-BLOCKING (wrapper-rejected incoherence)'
        }
      } elseif ($vw -ne 'CLEAN') {
        return 'a review-pass block has zero findings but its VERDICT is not CLEAN (wrapper-rejected incoherence)'
      }
    }
    # Eight-category block (mirrors Get-VerdictExitCode / Test-VerdictBlock),
    # required of EVERY non-header review-pass block (UNCONDITIONAL -- see the note
    # above the segment loop): the wrapper rejects any categoryless pass, so a
    # zero-category pass inside a multi-pass artifact, OR a bare categoryless single
    # verdict, is malformed. Each category exactly once as `none|<digits<=
    # CategoryCountMax>`, the category-count sum == this block's severity-finding-
    # line count, AND (below) each category's declared count == its indented
    # finding-row count.
    $catTotal = 0
    foreach ($cat in $catNames) {
      $cms = [regex]::Matches($blk, ('(?m)^' + [regex]::Escape($cat) + ':\s*(?<v>none|\d+)\s*$'))
      if ($cms.Count -ne 1) {
        return "a review-pass block has $($cms.Count) '$cat' category line(s) (exactly one required; the wrapper exits 3 on a missing or duplicate category -- a categoryless verdict is malformed, not legacy-exempt)"
      }
      $cv = $cms[0].Groups['v'].Value
      if ($cv -ne 'none') {
        $cparsed = 0
        if (-not [int]::TryParse($cv, [ref]$cparsed) -or $cparsed -gt $script:CategoryCountMax) {
          return "a review-pass block has an out-of-range '$cat' count (the wrapper exits 3)"
        }
        $catTotal += $cparsed
      }
    }
    $bN = ([regex]::Matches($blk, '(?m)^BLOCKER:')).Count
    $nbN = ([regex]::Matches($blk, '(?m)^QUALITY:')).Count + ([regex]::Matches($blk, '(?m)^NON-BLOCKER:')).Count + ([regex]::Matches($blk, '(?m)^NOTE:')).Count
    if ($catTotal -ne ($bN + $nbN)) {
      return "a review-pass block's category-count sum ($catTotal) != its severity-finding-line count ($($bN + $nbN)) (the wrapper exits 3 on the mismatch)"
    }
    # Row balance (mirrors Test-VerdictBlock): each `<CATEGORY>: <n>` must have
    # EXACTLY n indented finding rows -- neither fewer (UNDERFILL) nor more
    # (OVERFILL); a `none`/`0` category must have ZERO indented rows. The count-
    # sum check above catches a whole-block miscount, but NOT a per-category
    # imbalance another category compensates (e.g. `PLAN-DRIFT: 2` with 1 row
    # while a sibling over-rows to keep the sum). dispatch-checklist's
    # Get-CategoryFindings would misparse THAT, so both log-consumers reject it.
    # $inCatBlock stays set across a `none` block (declRemaining 0) so an
    # unexpected indented row under `none` is caught as overfill.
    $inCatBlock = $false
    $declRemaining = 0
    foreach ($line in ($blk -split "`r?`n")) {
      $cm = $catExactRe.Match($line)
      if ($cm.Success) {
        if ($inCatBlock -and $declRemaining -gt 0) {
          return 'a review-pass block UNDERFILLS a category (fewer indented finding rows than its declared count) -- dispatch-checklist rejects it as misparseable'
        }
        $k = Get-CategoryTailKind -Tail $cm.Groups['rest'].Value
        $inCatBlock = $true
        $declRemaining = if ($k.Kind -eq 'count') { $k.Value } else { 0 }
        continue
      }
      if ($inCatBlock) {
        if ($line -match '^\s+\S') {
          if ($declRemaining -le 0) {
            return 'a review-pass block OVERFILLS a category (more indented finding rows than its declared count) -- dispatch-checklist rejects it as misparseable'
          }
          $declRemaining--
        } elseif (-not [string]::IsNullOrWhiteSpace($line)) {
          if ($declRemaining -gt 0) {
            return 'a review-pass block UNDERFILLS a category (fewer indented finding rows than its declared count) -- dispatch-checklist rejects it as misparseable'
          }
          $inCatBlock = $false
        }
        # A blank line is tolerated inside an active block (does not end it).
      }
    }
    if ($inCatBlock -and $declRemaining -gt 0) {
      return 'a review-pass block UNDERFILLS a category at end-of-block (fewer indented finding rows than its declared count) -- dispatch-checklist rejects it as misparseable'
    }
  }
  return $null
}

# ---------------------------------------------------------------------------
# SelfTest mode: runs the fixture families below (each family's own section
# header is the authoritative inventory, so this stays non-enumerated and does
# not drift as fixtures are added): the Get-FindingArchetype classifier
# (A-series), the Get-VerdictSuspectReason fail-closed suspect check (S-series),
# and the end-to-end CLI fail-closed + multi-directory + multi-pass-dedupe checks
# over throwaway temp dirs (B-series). Exits before the main report pipeline.
# Touches the filesystem under $env:TEMP only -- never the repo, network, or git.
# ---------------------------------------------------------------------------
if ($SelfTest) {
  $failures = 0
  function Assert-Archetype {
    param([string]$Name, [string]$Text, [string]$ExpectedArchetype)
    $got = Get-FindingArchetype -Text $Text
    if ($got -eq $ExpectedArchetype) {
      Write-Host "[SelfTest] PASS $Name (-> $got)"
    } else {
      Write-Host "[SelfTest] FAIL ${Name}: got '$got', expected '$ExpectedArchetype'"
      Write-Host "  text was: $Text"
      $script:failures++
    }
  }

  # One fixture per archetype, drawn from the actual verdict-log
  # patterns the classifier should catch. Order matches the classifier's
  # most-specific-first ordering.
  Assert-Archetype 'A1: PLAN-DRIFT (PLAN.md cite)' `
    'PLAN.md:3759 - M88.15 is closed via M88.3 while M88.3 remains gated' 'PLAN-DRIFT'
  Assert-Archetype 'A2: PLAN-DRIFT (milestone keyword without PLAN.md path)' `
    'milestone M88.5 referenced but undefined' 'PLAN-DRIFT'
  Assert-Archetype 'A3: SILENT-FAILURE (dropped + silently)' `
    'Clock corrections are silently dropped when the peer lacks the clock component' 'SILENT-FAILURE'
  Assert-Archetype 'A4: SILENT-FAILURE (exits 0 on bad input)' `
    '--check exits successfully when the selected asset root contains zero files' 'SILENT-FAILURE'
  Assert-Archetype 'A5: TOMBSTONE-OR-SHIM (tombstone keyword)' `
    'Removed feature startup wiring left a tombstone comment at the old call site' 'TOMBSTONE-OR-SHIM'
  Assert-Archetype 'A6: TOMBSTONE-OR-SHIM (renamed-to-underscore)' `
    'Empty quarantine shim keeps a renamed-to-underscore unused parameter' 'TOMBSTONE-OR-SHIM'
  Assert-Archetype 'A7: CROSS-CRATE-CONTRACT (validator keyword)' `
    'validate_request_shape validator accepts target ids for variants documented as targetless' 'CROSS-CRATE-CONTRACT'
  Assert-Archetype 'A8: LOADER-OR-ASSET-EDGE (binary-asset)' `
    'Binary-asset parsing ignores the header-declared total length' 'LOADER-OR-ASSET-EDGE'
  Assert-Archetype 'A9: LOADER-OR-ASSET-EDGE (ASSET_ROOT)' `
    'ASSET_ROOT misconfiguration falls through to cwd/manifest roots' 'LOADER-OR-ASSET-EDGE'
  # A9b pins the prompt/dispatch surface wording (binary-format header/length,
  # format-invariant) so a finding phrased from review-prompt-template.md does
  # not regress into UNCLASSIFIED.
  Assert-Archetype 'A9b: LOADER-OR-ASSET-EDGE (binary-format header-length)' `
    'src/foo/loader.rs:12 - binary-format parser ignores the header-length field and drops a format-invariant' 'LOADER-OR-ASSET-EDGE'
  Assert-Archetype 'A11: WORKFLOW-INFRA (scripts/codex/ path)' `
    'scripts/codex/auto-review.ps1:294 - Checkbox conflict phase turns documented advisory PLAN checkboxes into merge-blocking false positives' 'WORKFLOW-INFRA'
  Assert-Archetype 'A12: CONVENTION (CLAUDE.md)' `
    'CLAUDE.md violation: error handling for impossible scenarios' 'CONVENTION'
  # A13 originally went UNCLASSIFIED; after the audit added
  # 'fallback cannot' to SILENT-FAILURE, this same text now correctly
  # classifies as SILENT-FAILURE. Updated to reflect the new classification
  # rather than removing the fixture (the text is real verdict prose).
  Assert-Archetype 'A13: SILENT-FAILURE (fallback cannot classify -- audit add)' `
    'src/foo/physics.rs:538 - Empty-collision fallback cannot classify steep grounded contacts' 'SILENT-FAILURE'
  # Audit-added pattern fixtures (each pins one new keyword set
  # introduced to drain the UNCLASSIFIED bucket; specific baseline
  # counts intentionally omitted because they drift as new verdicts
  # land and would cause comment-vs-report doc drift findings).
  Assert-Archetype 'A14: SILENT-FAILURE (lost audible playback)' `
    'src/foo/audio.rs:366 - The starter environmental cues lost audible fallback playback' 'SILENT-FAILURE'
  # Fixture text uses a path OUTSIDE the review-infra path family so the
  # audit-added 'reports?.*PASS' pattern can win; review-infra paths
  # would correctly classify as WORKFLOW-INFRA instead.
  Assert-Archetype 'A15: SILENT-FAILURE (reports false PASS in user code)' `
    'src/foo/input.rs:777 - Build automation can report a PASS without proving the click used the raycast-updated ghost' 'SILENT-FAILURE'
  Assert-Archetype 'A16: LOADER-OR-ASSET-EDGE (nonexistent clip)' `
    "src/foo/visuals.rs:3001 - De-quarantining authored asset visuals now exposes a nonexistent animation clip" 'LOADER-OR-ASSET-EDGE'
  Assert-Archetype 'A17: CONVENTION (lint gate)' `
    'src/foo/widget.rs:268 - Manual absolute-difference arithmetic trips the lint gate' 'CONVENTION'
  # A18 keeps a genuine UNCLASSIFIED catch-all so the bucket itself
  # stays exercised; uses generic prose without any keyword match.
  Assert-Archetype 'A18: UNCLASSIFIED catch-all (no archetype keyword)' `
    'foo:1 - the widget pattern is suboptimal here' 'UNCLASSIFIED'
  # Audit-pass-2 fixtures (each pins one new keyword set introduced to
  # drain the UNCLASSIFIED bucket; baseline counts intentionally
  # omitted because they drift as new verdicts land).
  Assert-Archetype 'A19: STALE-DOC (still describes pattern)' `
    'src/foo/cli.rs:223 - `--port` help still describes the old default-port behavior' 'STALE-DOC'
  Assert-Archetype 'A20: STALE-DOC (documentation says pattern)' `
    'src/foo/tests/mode.rs:35 - Test documentation says `--server` returns AppMode::Server' 'STALE-DOC'
  Assert-Archetype 'A21: STALE-DOC (Stale ... documentation prefix)' `
    'src/foo/tests/predicate.rs:46 - Stale test documentation describes pre-refactor predicate polarity' 'STALE-DOC'
  Assert-Archetype 'A22: WORKFLOW-INFRA (docs/audit-protocol doc)' `
    'docs/audit-protocol.md:3 - Protocol section misses analyzer-keyword-edit verification step' 'WORKFLOW-INFRA'
  Assert-Archetype 'A23: WORKFLOW-INFRA (scripts/codex/ path)' `
    'scripts/codex/analyze-blocker-trends.ps1:101 - analyzer keyword-list edits lack analyzer SelfTest verification' 'WORKFLOW-INFRA'
  Assert-Archetype 'A24: SILENT-FAILURE (warning-only and skipped)' `
    'src/foo/data.rs:196 - Missing required data dirs are warning-only and skipped' 'SILENT-FAILURE'
  Assert-Archetype 'A25: WORKFLOW-INFRA (review-prompt-template edit)' `
    'scripts/codex/review-prompt-template.md:33 - Template section reorders categories without analyzer rule update' 'WORKFLOW-INFRA'
  # A26 pins the scripts/claude/ classifier prefix added when the
  # default -ReviewsDir aggregation began covering Claude verdicts.
  # Without this pattern, a scripts/claude/commit.ps1 finding would
  # fall into UNCLASSIFIED unless its summary happened to contain
  # another WORKFLOW-INFRA keyword.
  Assert-Archetype 'A26: WORKFLOW-INFRA (scripts/claude/ path)' `
    'scripts/claude/commit.ps1:142 - Bypass-flag parser rejects --no-verbose alongside --no-verify' 'WORKFLOW-INFRA'
  # Review-log fixtures from the bootstrap/core.hooksPath audit. These
  # are review-system infrastructure even when the finding is reported
  # against INSTALL.md rather than a scripts/ path.
  # The citation below is synthetic classifier input, not a live source claim.
  Assert-Archetype 'A27: WORKFLOW-INFRA (bootstrap.ps1 path)' `
    'bootstrap.ps1:1 - Synthetic installer workflow finding' 'WORKFLOW-INFRA'
  Assert-Archetype 'A28: WORKFLOW-INFRA (core.hooksPath doc drift)' `
    "INSTALL.md:54 - bootstrap's core.hooksPath handling is described as only legacy cleanup" 'WORKFLOW-INFRA'
  # A29-A34 pin the remaining WORKFLOW-INFRA keyword patterns so a future
  # keyword-list edit that drops or reroutes one fails SelfTest (audit-protocol
  # requires keyword-list edits to add fixtures). A29 pins the `scripts/git-hooks/`
  # PATH keyword -- it cites a scripts/git-hooks/ path and is classified via that
  # PATH keyword (its summary carries no keyword). A30-A34 each cite a NON-workflow
  # file path so classification is driven by the summary KEYWORD, not the path.
  # The git-specific keywords hooksPath / git-config are near-always gate-scoped
  # (unlike the generic "dispatcher" that needed the A35 negative fixture), so
  # their positive pins A33/A34 suffice -- a non-gate false match is not realistic.
  Assert-Archetype 'A29: WORKFLOW-INFRA (scripts/git-hooks/ path)' `
    'scripts/git-hooks/post-merge:10 - hook does not forward REVIEW_BACKEND to the merge gate' 'WORKFLOW-INFRA'
  Assert-Archetype 'A30: WORKFLOW-INFRA (gate dispatcher keyword)' `
    'src/app/wiring.rs:10 - the per-clone dispatcher install step was skipped' 'WORKFLOW-INFRA'
  Assert-Archetype 'A31: WORKFLOW-INFRA (commit wrapper keyword)' `
    'src/app/foo.rs:10 - the commit wrapper does not restore REVIEW_BACKEND after committing' 'WORKFLOW-INFRA'
  Assert-Archetype 'A32: WORKFLOW-INFRA (commit-wrapper hyphenated keyword)' `
    'src/app/foo.rs:12 - a commit-wrapper self-edit is not caught by the routing guard' 'WORKFLOW-INFRA'
  Assert-Archetype 'A33: WORKFLOW-INFRA (git-config keyword)' `
    'src/app/foo.rs:14 - the git-config scrub does not restore on a failure path' 'WORKFLOW-INFRA'
  Assert-Archetype 'A34: WORKFLOW-INFRA (bare hooksPath keyword)' `
    'src/app/foo.rs:16 - the effective hooksPath resolves via HOME and can redirect the hook' 'WORKFLOW-INFRA'
  # NEGATIVE: a generic application "event dispatcher" must NOT land in
  # WORKFLOW-INFRA -- the over-match the qualified dispatcher pattern fixes.
  Assert-Archetype 'A35: generic event dispatcher is NOT WORKFLOW-INFRA (qualified pattern)' `
    'src/game/audio.rs:12 - the audio event dispatcher mixes voices past the channel cap' 'UNCLASSIFIED'

  # -------------------------------------------------------------------------
  # Suspect-check fixtures (S-series): Get-VerdictSuspectReason must fail closed
  # on every verdict shape the wrapper (Get-VerdictExitCode) and dispatch-
  # checklist (Get-VerdictStructure / Test-VerdictBlock) reject -- a GATE-FAILED-
  # CLOSED banner, a truncated/missing verdict, a malformed or incomplete REVIEW
  # PASS sequence, an incoherent verdict-word/severity combination, a malformed
  # eight-category block, an over/UNDERfilled category ROW block (S35-S37), a
  # malformed category TAIL (S38 / wrong-case `None` tail S44), or a wrong-case
  # severity PREFIX the case-sensitive wrapper parser under-counts (S41-S43) --
  # while passing every well-formed
  # verdict, including valid multi-finding row blocks, a wrapper-accepted
  # lowercase `verdict:` (S45), and a real full-header Codex artifact (S46)
  # (positive controls S1/S6/S8/S17/S26/S30/S31/S39/S40/S45/S46). A regression here
  # lets a malformed
  # verdict silently pollute or abort the trend corpus, or (a row-balance false
  # positive) aborts the whole run on a valid verdict.
  # -------------------------------------------------------------------------
  function Assert-Eq {
    param([string]$Name, $Actual, $Expected)
    if ($Actual -eq $Expected) {
      Write-Host "[SelfTest] PASS $Name"
    } else {
      Write-Host "[SelfTest] FAIL ${Name}: got '$Actual', expected '$Expected'"
      $script:failures++
    }
  }
  $sVre = [regex]'(?im)^VERDICT:\s*(?<v>CLEAN|NON-BLOCKING|BLOCKED)\s*$'
  # S1: a well-formed CLEAN verdict -- complete eight-category block (all none),
  # zero findings, VERDICT: CLEAN -- is NOT suspect (the baseline positive control).
  $s1 = @"
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S1: a complete-category CLEAN verdict is NOT suspect (baseline positive control)' `
    (Get-VerdictSuspectReason -Content $s1 -VerdictWordRe $sVre) $null
  Assert-Eq 'S2: GATE FAILED CLOSED is suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content "===== GATE FAILED CLOSED =====`nVERDICT: BLOCKED`n" -VerdictWordRe $sVre)) $true
  Assert-Eq 'S3: no-verdict-no-severity is suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content "random prose, no verdict, no severity prefix`n" -VerdictWordRe $sVre)) $true
  # S4 (inverted): a severity line with NO VERDICT line IS suspect -- a truncated/
  # missing-VERDICT file must not reach the parse loop and pollute trends (the
  # wrapper + checklist fail closed on it).
  Assert-Eq 'S4: a severity line alone (no VERDICT line) IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content "BLOCKER: foo.rs:1 - x`n" -VerdictWordRe $sVre)) $true
  # S5: a malformed VERDICT word with NO BLOCKER finding IS suspect (the wrapper
  # fails closed on it).
  Assert-Eq 'S5: a malformed VERDICT word with no BLOCKER IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content "QUALITY: foo.rs:1 - x`nVERDICT: MAYBE`n" -VerdictWordRe $sVre)) $true
  # S6: a malformed VERDICT word WITH a BLOCKER is NOT suspect -- BLOCKER
  # precedence makes it BLOCKED (mirrors the wrapper's Get-VerdictExitCode); it
  # must be kept and counted, not aborted. Full category block so the ONLY
  # unusual thing is the malformed word (isolates BLOCKER precedence).
  $s6 = @"
PLAN-DRIFT: 1
  foo.rs:1 - x
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: MAYBE

BLOCKER: foo.rs:1 - x
"@
  Assert-Eq 'S6: a malformed VERDICT word WITH a BLOCKER is NOT suspect (BLOCKER precedence)' `
    (Get-VerdictSuspectReason -Content $s6 -VerdictWordRe $sVre) $null
  # S7: a MULTI-PASS artifact whose pass1 is valid but pass2 has a malformed
  # VERDICT word with no BLOCKER IS suspect -- the per-block check must not let a
  # valid sibling pass mask the malformed one. BOTH passes carry the full category
  # block, so pass1 is genuinely valid and the suspect signal is pass2's word.
  $s7 = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: 1
  foo.rs:1 - x
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: NON-BLOCKING

QUALITY: foo.rs:1 - x
===== REVIEW PASS 2/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: 1
  bar.rs:2 - y
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: MAYBE

QUALITY: bar.rs:2 - y
"@
  Assert-Eq 'S7: a valid pass + a malformed no-BLOCKER pass IS suspect (per-block)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s7 -VerdictWordRe $sVre)) $true
  # S8: a multi-pass artifact where the malformed-word pass carries a BLOCKER is
  # NOT suspect (that pass is BLOCKED via precedence, not fail-closed). BOTH passes
  # carry the full category block.
  $s8 = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: 1
  foo.rs:1 - x
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: NON-BLOCKING

QUALITY: foo.rs:1 - x
===== REVIEW PASS 2/2 =====
PLAN-DRIFT: 1
  bar.rs:2 - y
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: MAYBE

BLOCKER: bar.rs:2 - y
"@
  Assert-Eq 'S8: a multi-pass artifact whose malformed pass has a BLOCKER is NOT suspect' `
    (Get-VerdictSuspectReason -Content $s8 -VerdictWordRe $sVre) $null
  # S9: a review-pass block with ZERO VERDICT lines IS suspect (the wrapper
  # requires exactly one per pass).
  # Pass1 is fully valid so the loop REACHES pass2, whose missing VERDICT line is
  # the isolated defect (a categoryless pass1 would be quarantined first).
  $s9 = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: 1
  foo.rs:1 - x
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: NON-BLOCKING

QUALITY: foo.rs:1 - x
===== REVIEW PASS 2/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: 1
  bar.rs:2 - y
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

QUALITY: bar.rs:2 - y
"@
  Assert-Eq 'S9: a review-pass block with ZERO VERDICT lines IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s9 -VerdictWordRe $sVre)) $true
  # S10: a review-pass block with DUPLICATE VERDICT lines IS suspect (exactly one
  # required per pass).
  $s10 = @"
===== REVIEW PASS 1/1 =====
QUALITY: foo.rs:1 - x
VERDICT: NON-BLOCKING
VERDICT: CLEAN
"@
  Assert-Eq 'S10: a review-pass block with DUPLICATE VERDICT lines IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s10 -VerdictWordRe $sVre)) $true
  # S11: an UNKNOWN `===== ... =====` marker IS suspect (a finding could hide after
  # it; only REVIEW PASS markers and the GATE FAILED CLOSED banner are recognized).
  $s11 = @"
===== REVIEW PASS 1/1 =====
QUALITY: foo.rs:1 - x
VERDICT: NON-BLOCKING
===== EXTRA =====
BLOCKER: hidden.rs:1 - sneaky
"@
  Assert-Eq 'S11: an unrecognized ===== marker IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s11 -VerdictWordRe $sVre)) $true
  # S12: a finding hidden in the LEADING header fragment (before the first marker)
  # IS suspect -- the whole-content parse loop would otherwise count it.
  $s12 = @"
BLOCKER: hidden.rs:1 - sneaky
===== REVIEW PASS 1/1 =====
QUALITY: foo.rs:1 - x
VERDICT: NON-BLOCKING
"@
  Assert-Eq 'S12: a finding hidden in the leading header fragment IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s12 -VerdictWordRe $sVre)) $true
  # S13-S19: EVERY review-pass block must carry the full well-formed eight-category
  # block (the requirement is UNCONDITIONAL -- the wrapper rejects any categoryless
  # or malformed-category pass, exit 3 / no banner, and dispatch-checklist rejects
  # it too). Covered by: S18 (headerless partial -> suspect), S31 (headerless
  # complete -> valid), S32 (headerless zero-category -> suspect), S33 (multi-pass
  # with a zero-category pass -> suspect), S34 (header-bearing category-FREE ->
  # suspect). (S13-S17/S19 are header-bearing category-bearing.)
  # S13: claude single-verdict shape, MISSING a category (only 7 present).
  $s13 = @"
DIFF-SHA256: abc
REVIEW-BACKEND: claude

PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none

VERDICT: CLEAN
"@
  Assert-Eq 'S13: a category-bearing block MISSING a category IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s13 -VerdictWordRe $sVre)) $true
  # S14: DUPLICATE category line.
  $s14 = @"
DIFF-SHA256: abc

PLAN-DRIFT: none
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S14: a category-bearing block with a DUPLICATE category IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s14 -VerdictWordRe $sVre)) $true
  # S15: OVERSIZED category count (> 10000).
  $s15 = @"
DIFF-SHA256: abc

PLAN-DRIFT: 20000
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
"@
  Assert-Eq 'S15: a category-bearing block with an OVERSIZED category count IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s15 -VerdictWordRe $sVre)) $true
  # S16: category-count sum != severity-finding count.
  $s16 = @"
DIFF-SHA256: abc

PLAN-DRIFT: 1
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED
"@
  Assert-Eq 'S16: a category-bearing block with a category/severity sum MISMATCH IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s16 -VerdictWordRe $sVre)) $true
  # S17: a VALID category-bearing block (header + all 8 categories, sum matches) -- NOT suspect.
  $s17 = @"
DIFF-SHA256: abc

PLAN-DRIFT: 1
  PLAN.md:1 - x
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
"@
  Assert-Eq 'S17: a valid category-bearing block is NOT suspect (positive control)' `
    (Get-VerdictSuspectReason -Content $s17 -VerdictWordRe $sVre) $null
  # S18: a PARTIAL category block (only PLAN-DRIFT present, the other 7 missing) IS
  # suspect -- every review-pass block must carry the FULL eight-category block
  # (the wrapper rejects a partial block, exit 3, with or without a header).
  $s18 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: 1
  PLAN.md:1 - x
VERDICT: BLOCKED
BLOCKER: PLAN.md:1 - x
"@
  Assert-Eq 'S18: a headerless artifact with a PARTIAL category block IS suspect (category-line presence)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s18 -VerdictWordRe $sVre)) $true
  # S19: codex pass-block shape (header + markers) with a category-malformed block IS suspect.
  $s19 = @"
DIFF-SHA256: abc
REVIEW-BACKEND: codex

===== REVIEW PASS 1/1 =====
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none

VERDICT: CLEAN
"@
  Assert-Eq 'S19: a category-bearing codex pass-block missing a category IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s19 -VerdictWordRe $sVre)) $true
  # S20-S30: pass-SEQUENCE + verdict-word/severity COHERENCE, mirroring dispatch-
  # checklist Get-VerdictStructure and the wrapper Get-VerdictExitCode. S20: a
  # TRUNCATED sequence (1 of 2 passes).
  $s20 = @"
===== REVIEW PASS 1/2 =====
VERDICT: CLEAN
"@
  Assert-Eq 'S20: a truncated REVIEW PASS sequence (1 of 2) IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s20 -VerdictWordRe $sVre)) $true
  # S21: DUPLICATE pass number.
  $s21 = @"
===== REVIEW PASS 1/2 =====
VERDICT: CLEAN
===== REVIEW PASS 1/2 =====
VERDICT: CLEAN
"@
  Assert-Eq 'S21: a duplicate REVIEW PASS number IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s21 -VerdictWordRe $sVre)) $true
  # S22: MISMATCHED totals across passes.
  $s22 = @"
===== REVIEW PASS 1/2 =====
VERDICT: CLEAN
===== REVIEW PASS 2/3 =====
VERDICT: CLEAN
"@
  Assert-Eq 'S22: REVIEW PASS markers with disagreeing totals ARE suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s22 -VerdictWordRe $sVre)) $true
  # S23: QUALITY finding under VERDICT: CLEAN (valid word, incoherent).
  $s23 = @"
QUALITY: foo.rs:1 - x
VERDICT: CLEAN
"@
  Assert-Eq 'S23: a QUALITY finding under VERDICT: CLEAN IS suspect (incoherent)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s23 -VerdictWordRe $sVre)) $true
  # S24: NOTE finding under VERDICT: BLOCKED (no BLOCKER; incoherent).
  $s24 = @"
NOTE: foo.rs:1 - x
VERDICT: BLOCKED
"@
  Assert-Eq 'S24: a NOTE finding under VERDICT: BLOCKED IS suspect (incoherent)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s24 -VerdictWordRe $sVre)) $true
  # S25: zero findings under VERDICT: BLOCKED (incoherent).
  $s25 = @"
VERDICT: BLOCKED
"@
  Assert-Eq 'S25: zero findings under VERDICT: BLOCKED IS suspect (incoherent)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s25 -VerdictWordRe $sVre)) $true
  # S26: a COMPLETE 2-pass sequence, both passes coherent and category-complete --
  # NOT suspect (positive).
  $s26 = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: 1
  foo.rs:1 - x
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: NON-BLOCKING

QUALITY: foo.rs:1 - x
===== REVIEW PASS 2/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S26: a complete 2-pass coherent sequence is NOT suspect (positive control)' `
    (Get-VerdictSuspectReason -Content $s26 -VerdictWordRe $sVre) $null
  # S27: a pass total N above the producer 1..10 range IS suspect.
  $s27 = @"
===== REVIEW PASS 1/11 =====
VERDICT: CLEAN
"@
  Assert-Eq 'S27: a REVIEW PASS total N above the producer 1..10 range IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s27 -VerdictWordRe $sVre)) $true
  # S28: a pass number n greater than the total N IS suspect.
  $s28 = @"
===== REVIEW PASS 3/2 =====
VERDICT: CLEAN
"@
  Assert-Eq 'S28: a REVIEW PASS with n > N IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s28 -VerdictWordRe $sVre)) $true
  # S29: a legacy NON-BLOCKER finding under VERDICT: CLEAN IS suspect (incoherent --
  # NON-BLOCKER, like QUALITY/NOTE, requires NON-BLOCKING).
  $s29 = @"
NON-BLOCKER: foo.rs:1 - x
VERDICT: CLEAN
"@
  Assert-Eq 'S29: a legacy NON-BLOCKER finding under VERDICT: CLEAN IS suspect' `
    ($null -ne (Get-VerdictSuspectReason -Content $s29 -VerdictWordRe $sVre)) $true
  # S30: a legacy NON-BLOCKER finding under VERDICT: NON-BLOCKING is NOT suspect
  # (coherent). Full category block isolates the NON-BLOCKER/NON-BLOCKING coherence.
  $s30 = @"
PLAN-DRIFT: none
SILENT-FAILURE: 1
  foo.rs:1 - x
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: NON-BLOCKING

NON-BLOCKER: foo.rs:1 - x
"@
  Assert-Eq 'S30: a legacy NON-BLOCKER finding under VERDICT: NON-BLOCKING is NOT suspect' `
    (Get-VerdictSuspectReason -Content $s30 -VerdictWordRe $sVre) $null
  # S31: a HEADERLESS artifact with a COMPLETE, coherent eight-category block is
  # NOT suspect -- valid complete-category evidence that simply lacks the
  # provenance header block. Proves the category requirement accepts a well-formed
  # headerless block, not only header-bearing ones.
  $s31 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: 1
  PLAN.md:1 - x
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
"@
  Assert-Eq 'S31: a headerless artifact with a COMPLETE coherent category block is NOT suspect (valid)' `
    (Get-VerdictSuspectReason -Content $s31 -VerdictWordRe $sVre) $null
  # S32: a headerless verdict with NO category block IS suspect -- a categoryless
  # verdict is malformed (the wrapper rejects it, exit 3), NOT a legacy-exempt
  # historical log. There is NO pre-category exemption (see the unconditional note
  # in Get-VerdictSuspectReason); this fixture pins that removal.
  $s32 = @"
===== REVIEW PASS 1/1 =====
VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
"@
  Assert-Eq 'S32: a headerless verdict with NO category block IS suspect (categoryless is malformed; no legacy exemption)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s32 -VerdictWordRe $sVre)) $true
  # S33: a MULTI-PASS artifact whose pass 1 has a COMPLETE category block but pass 2
  # has ZERO category lines IS suspect -- EVERY pass must carry the full block (the
  # wrapper rejects the categoryless pass), so pass 2 is malformed even though pass
  # 1 is valid.
  $s33 = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: 1
  PLAN.md:1 - x
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
===== REVIEW PASS 2/2 =====
VERDICT: CLEAN
"@
  Assert-Eq 'S33: a multi-pass artifact with a zero-category pass IS suspect (every pass must carry the category block)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s33 -VerdictWordRe $sVre)) $true
  # S34: a HEADER-BEARING but category-FREE verdict IS suspect -- categories are
  # required of EVERY verdict regardless of the provenance header (the wrapper
  # exits 3 on the missing categories). The header does NOT exempt it.
  $s34 = @"
DIFF-SHA256: abc
REVIEW-BACKEND: claude

VERDICT: CLEAN
"@
  Assert-Eq 'S34: a header-bearing category-free verdict IS suspect (categories required regardless of header)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s34 -VerdictWordRe $sVre)) $true
  # S35: UNDERFILL -- `PLAN-DRIFT: 2` with only 1 indented finding row (the count
  # sum still matches because 2 BLOCKER severity lines are present, so this passes
  # the sum check and is caught ONLY by row balance -- the dispatch-parity gap the
  # gate flagged).
  $s35 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: 2
  PLAN.md:1 - x
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
BLOCKER: PLAN.md:2 - y
"@
  Assert-Eq 'S35: an UNDERFILLED category row block (2 declared, 1 row) IS suspect (row balance)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s35 -VerdictWordRe $sVre)) $true
  # S36: OVERFILL -- `PLAN-DRIFT: 1` with 2 indented finding rows (sum matches via 1
  # BLOCKER line; caught only by row balance).
  $s36 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: 1
  PLAN.md:1 - x
  PLAN.md:2 - y
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
"@
  Assert-Eq 'S36: an OVERFILLED category row block (1 declared, 2 rows) IS suspect (row balance)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s36 -VerdictWordRe $sVre)) $true
  # S37: a `none` category with an indented finding row IS suspect (overfill under
  # a zero-count category; sum 0 == 0 severity lines, so caught only by row balance).
  $s37b = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: none
  PLAN.md:1 - x
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S37: a `none` category with an indented row IS suspect (overfill under zero count)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s37b -VerdictWordRe $sVre)) $true
  # S38: an EXTRA malformed category TAIL (`PLAN-DRIFT: +1` alongside a valid
  # `PLAN-DRIFT: none`) IS suspect -- the per-category cardinality check counts only
  # the strict `none|\d+` line and misses the `+1`, so the whole-content invalid-
  # tail scan is what catches it (dispatch-checklist rejects the same shape).
  $s38 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: none
PLAN-DRIFT: +1
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S38: an extra malformed category tail (`+1`) IS suspect (whole-content invalid-tail scan)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s38 -VerdictWordRe $sVre)) $true
  # S39: a VALID multi-finding row block (`PLAN-DRIFT: 2` with EXACTLY 2 indented
  # rows + 2 BLOCKER lines) is NOT suspect -- proves row balance ACCEPTS a well-
  # formed multi-row category and does not false-positive.
  $s39 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: 2
  PLAN.md:1 - x
  PLAN.md:2 - y
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
BLOCKER: PLAN.md:2 - y
"@
  Assert-Eq 'S39: a valid multi-finding row block (2 declared, 2 rows) is NOT suspect (row-balance positive control)' `
    (Get-VerdictSuspectReason -Content $s39 -VerdictWordRe $sVre) $null
  # S40: a VALID block with TWO counted categories, each with its exact row count,
  # is NOT suspect -- proves row balance resets $declRemaining correctly across
  # adjacent counted categories.
  $s40 = @"
===== REVIEW PASS 1/1 =====
PLAN-DRIFT: 1
  PLAN.md:1 - x
SILENT-FAILURE: 1
  src/foo.rs:2 - y
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:1 - x
QUALITY: src/foo.rs:2 - y
"@
  Assert-Eq 'S40: a valid two-counted-category block (each 1 declared, 1 row) is NOT suspect (row-balance positive control)' `
    (Get-VerdictSuspectReason -Content $s40 -VerdictWordRe $sVre) $null
  # S41: a LOWERCASE `blocker:` prefix with a malformed VERDICT word IS suspect --
  # the wrapper's severity parser is case-SENSITIVE, so `blocker:` is NOT a BLOCKER
  # to it; the case-sensitive coherence check here must agree (zero real findings +
  # a non-CLEAN/malformed word -> incoherent). A case-INSENSITIVE check would see a
  # phantom BLOCKER, grant BLOCKER-precedence, and wrongly pass the file.
  $s41 = @"
blocker: foo.rs:1 - x
VERDICT: FOO
"@
  Assert-Eq 'S41: a lowercase `blocker:` prefix + malformed VERDICT IS suspect (case-sensitive severity parity)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s41 -VerdictWordRe $sVre)) $true
  # S42: a LOWERCASE `quality:` prefix under VERDICT: NON-BLOCKING IS suspect -- the
  # case-sensitive parser counts zero findings, so the word must be CLEAN, not
  # NON-BLOCKING. A case-INSENSITIVE check would see a phantom QUALITY and wrongly
  # accept NON-BLOCKING as coherent.
  $s42 = @"
quality: foo.rs:1 - x
VERDICT: NON-BLOCKING
"@
  Assert-Eq 'S42: a lowercase `quality:` prefix under NON-BLOCKING IS suspect (case-sensitive severity parity)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s42 -VerdictWordRe $sVre)) $true
  # S43: a lowercase `blocker:` line under a COMPLETE all-none block + VERDICT:
  # CLEAN IS suspect -- the case-sensitive parser counts zero findings (so the
  # block looks coherent AND category-valid), but the wrong-case severity line
  # means a real BLOCKER hides under a falsely-clean verdict. The wrong-case
  # severity scan catches it (coherence + category alone would pass it).
  $s43 = @"
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN

blocker: hidden.rs:1 - sneaky
"@
  Assert-Eq 'S43: a lowercase `blocker:` under a complete all-none CLEAN verdict IS suspect (wrong-case severity scan)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s43 -VerdictWordRe $sVre)) $true
  # S44: an EXTRA wrong-case `PLAN-DRIFT: None` beside a valid `PLAN-DRIFT: none`
  # IS suspect -- Get-CategoryTailKind's -ceq classifies `None` as 'invalid' (not
  # the `none` sentinel), so the whole-content invalid-tail scan catches it. A
  # case-insensitive -eq would misclassify `None` as 'none' and miss the extra line
  # (which strict cardinality also misses, counting only the lowercase `none`).
  $s44 = @"
PLAN-DRIFT: none
PLAN-DRIFT: None
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S44: an extra wrong-case `PLAN-DRIFT: None` beside a valid `none` IS suspect (case-sensitive tail classification)' `
    ($null -ne (Get-VerdictSuspectReason -Content $s44 -VerdictWordRe $sVre)) $true
  # S45: a wrapper-ACCEPTED lowercase `verdict: clean` with a complete all-none
  # block is NOT suspect -- the wrappers detect the verdict case-insensitively
  # (`-match '^VERDICT:'` + case-insensitive word compare), so quarantining it would
  # diverge from a log the wrappers accept as clean. (VERDICT case is insensitive;
  # severity/category case stays sensitive -- see the case-policy note in the fn.)
  $s45 = @"
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

verdict: clean
"@
  Assert-Eq 'S45: a wrapper-accepted lowercase `verdict: clean` with a complete block is NOT suspect (case-insensitive VERDICT parity)' `
    (Get-VerdictSuspectReason -Content $s45 -VerdictWordRe $sVre) $null
  # S46: a REAL Codex artifact -- the full provenance header (DIFF-SHA256,
  # REVIEW-TREE-OID, REVIEW-BACKEND, REVIEW-EFFORT, and the 2026-07
  # REVIEW-SEVERITY-CONTRACT era stamp) before the first `===== REVIEW
  # PASS =====` marker, then a valid block -- is NOT suspect. Pins that the leading-
  # header-fragment check accepts ALL stamped lines (a typo dropping any from
  # $headerLineRe would otherwise quarantine every Codex verdict; auto-review.ps1
  # relies on this).
  $s46 = @"
DIFF-SHA256: abc123
REVIEW-TREE-OID: def456
REVIEW-BACKEND: codex
REVIEW-EFFORT: xhigh
REVIEW-SEVERITY-CONTRACT: blocker-only

===== REVIEW PASS 1/1 =====
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN
"@
  Assert-Eq 'S46: a real Codex artifact (full header block + valid block) is NOT suspect (header-fragment positive control)' `
    (Get-VerdictSuspectReason -Content $s46 -VerdictWordRe $sVre) $null

  # End-to-end fail-closed: an empty temp dir must exit 1 without
  # writing a report. Builds a one-off temp dir, re-invokes this script
  # with -ReviewsDir pointing at it, and asserts non-zero exit. The
  # write-blocked path is documented but not exercised in-memory; this
  # covers the documented contract via real CLI invocation.
  $tempDir = Join-Path $env:TEMP ("trend-selftest-" + [guid]::NewGuid().ToString('N').Substring(0,12))
  $tempDir2 = Join-Path $env:TEMP ("trend-selftest-2-" + [guid]::NewGuid().ToString('N').Substring(0,12))
  $tempOut = Join-Path $env:TEMP ("trend-selftest-out-" + [guid]::NewGuid().ToString('N').Substring(0,12) + '.md')
  # Distinct per-fixture output paths so the multi-pass fixtures B8-B11 each
  # prove they read a FRESH report of their OWN input (not a stale report left
  # by an earlier fixture).
  $tempOutB8  = Join-Path $env:TEMP ("trend-selftest-out-b8-"  + [guid]::NewGuid().ToString('N').Substring(0,12) + '.md')
  $tempOutB9  = Join-Path $env:TEMP ("trend-selftest-out-b9-"  + [guid]::NewGuid().ToString('N').Substring(0,12) + '.md')
  $tempOutB10 = Join-Path $env:TEMP ("trend-selftest-out-b10-" + [guid]::NewGuid().ToString('N').Substring(0,12) + '.md')
  $tempOutB11 = Join-Path $env:TEMP ("trend-selftest-out-b11-" + [guid]::NewGuid().ToString('N').Substring(0,12) + '.md')
  $tempOutB12 = Join-Path $env:TEMP ("trend-selftest-out-b12-" + [guid]::NewGuid().ToString('N').Substring(0,12) + '.md')
  New-Item -ItemType Directory -Path $tempDir -Force | Out-Null
  New-Item -ItemType Directory -Path $tempDir2 -Force | Out-Null
  try {
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOut 2>&1 | Out-Null
    $subExit = $LASTEXITCODE
    if ($subExit -ne 0) {
      Write-Host "[SelfTest] PASS B1: empty reviews dir -> exit $subExit"
    } else {
      Write-Host "[SelfTest] FAIL B1: empty reviews dir exited 0; expected non-zero"
      $script:failures++
    }
    if (Test-Path -LiteralPath $tempOut) {
      Write-Host "[SelfTest] FAIL B1: empty reviews dir overwrote report at $tempOut"
      $script:failures++
    } else {
      Write-Host "[SelfTest] PASS B2: empty reviews dir did not write report"
    }

    # Verify the filename-pattern filter: write a non-verdict-shaped
    # markdown file in the temp dir and confirm the analyzer still
    # rejects the dir as having zero verdict-shaped files.
    $bogus = Join-Path $tempDir 'not-a-verdict.md'
    Set-Content -LiteralPath $bogus -Value "BLOCKER: foo.rs:1 - example`nVERDICT: BLOCKED`n" -Encoding UTF8
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOut 2>&1 | Out-Null
    $subExit2 = $LASTEXITCODE
    if ($subExit2 -ne 0 -and -not (Test-Path -LiteralPath $tempOut)) {
      Write-Host "[SelfTest] PASS B3: non-verdict-shaped *.md ignored (exit $subExit2, no report)"
    } else {
      Write-Host "[SelfTest] FAIL B3: non-verdict-shaped *.md leaked through; exit=$subExit2 report-exists=$(Test-Path -LiteralPath $tempOut)"
      $script:failures++
    }

    # End-to-end report generation with exactly ONE finding. Exercises the
    # singleton-bucket path that produced blank section headers before the
    # @() wrap was added inside Write-ArchetypeSection. Asserts the
    # generated report shows a "(1 findings)" header AND the archetype
    # subheader's "-- 1 findings" line.
    # Fixture text uses a PLAN.md path so the analyzer's classifier
    # picks PLAN-DRIFT (matching the per-category block). The category
    # block at the top of the verdict is consumed by the WRAPPER's
    # Get-VerdictExitCode; the analyzer instead classifies the
    # severity-prefixed finding line independently via its own
    # most-specific-first keyword list.
    $verdict = Join-Path $tempDir 'review-20260522-160000-selftest-single.md'
    $verdictBody = @"
PLAN-DRIFT: 1
  PLAN.md:99 -- example milestone-drift finding for SelfTest
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:99 -- example milestone-drift finding for SelfTest
"@
    # Remove the bogus non-verdict-shape file so the only input is the
    # singleton fixture (otherwise the empty-input test polluted state
    # leaks here).
    Remove-Item -LiteralPath $bogus -ErrorAction SilentlyContinue
    [System.IO.File]::WriteAllText($verdict, $verdictBody, [System.Text.UTF8Encoding]::new($false))
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOut 2>&1 | Out-Null
    $subExit3 = $LASTEXITCODE
    if ($subExit3 -eq 0 -and (Test-Path -LiteralPath $tempOut)) {
      $reportText = [System.IO.File]::ReadAllText($tempOut, [System.Text.Encoding]::UTF8)
      $sectionOk = $reportText -match '\(1 findings\)'
      $archetypeOk = $reportText -match 'PLAN-DRIFT -- 1 findings'
      if ($sectionOk -and $archetypeOk) {
        Write-Host "[SelfTest] PASS B4: singleton bucket renders count 1 in section + archetype headers"
      } else {
        Write-Host "[SelfTest] FAIL B4: singleton count missing in report"
        Write-Host "  section_ok=$sectionOk archetype_ok=$archetypeOk"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B4: singleton fixture invocation did not produce a report; exit=$subExit3"
      $script:failures++
    }

    # Multi-directory fixtures (B5/B6/B7) pin the cross-backend
    # aggregation behavior added when -ReviewsDir became [string[]].
    # tempDir still contains the singleton verdict from B4; B5 writes
    # a second verdict into tempDir2 and asserts the report aggregates
    # both. B6 asserts a missing dir is skipped (not fatal). B7 asserts
    # all-missing fails closed.
    # The BLOCKER finding text must contain a SILENT-FAILURE classifier
    # keyword (e.g., "silently", "dropped", "suppressed") so the
    # archetype lands in the expected bucket; literal "silent-failure"
    # alone matches no keyword and would classify UNCLASSIFIED. The
    # per-category enumeration block above is parsed by the wrapper's
    # Get-VerdictExitCode; the analyzer instead classifies the
    # severity-prefixed finding line independently via its keyword list.
    $verdict2 = Join-Path $tempDir2 'review-20260522-160001-selftest-multidir.md'
    $verdictBody2 = @"
PLAN-DRIFT: none
SILENT-FAILURE: 1
  src/foo.rs:1 -- mutation path silently dropped value (multi-dir SelfTest fixture)
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: src/foo.rs:1 -- mutation path silently dropped value (multi-dir SelfTest fixture)
"@
    [System.IO.File]::WriteAllText($verdict2, $verdictBody2, [System.Text.UTF8Encoding]::new($false))

    # B5/B6/B7 invoke the child analyzer with a multi-element array
    # for -ReviewsDir. `powershell.exe -File script.ps1 -X a,b` does
    # NOT bind a [string[]] from a comma-separated CLI value (the
    # comma becomes part of a single string element); the working
    # form is `-Command "& 'script.ps1' -X 'a','b'"` which lets the
    # in-process parser see the comma as the array literal operator.
    #
    # SAFETY: every path interpolated into a single-quoted PowerShell
    # string literal must have embedded apostrophes doubled (PowerShell
    # single-quoted-string escape rule). $env:TEMP CAN contain an
    # apostrophe (e.g., a Windows account name like "O'Brien"), so the
    # raw-interpolation form is unsafe. The PS-literal escape (replace
    # ' with '') is applied to every embedded path before construction.
    $psLiteralEsc = { param([string]$s) ($s -replace "'", "''") }
    $psCmdPathEsc = & $psLiteralEsc $PSCommandPath
    $tempOutEsc   = & $psLiteralEsc $tempOut

    # B5: invoke with both dirs. The header should say "2 verdict
    # files across 2 reviews dir(s)" and the report should classify
    # both findings.
    if (Test-Path -LiteralPath $tempOut) { Remove-Item -LiteralPath $tempOut -ErrorAction SilentlyContinue }
    $tempDirEsc  = & $psLiteralEsc $tempDir
    $tempDir2Esc = & $psLiteralEsc $tempDir2
    $cmd5 = "& '$psCmdPathEsc' -ReviewsDir '$tempDirEsc','$tempDir2Esc' -OutPath '$tempOutEsc'"
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -Command $cmd5 2>&1 | Out-Null
    $subExit5 = $LASTEXITCODE
    if ($subExit5 -eq 0 -and (Test-Path -LiteralPath $tempOut)) {
      $reportText5 = [System.IO.File]::ReadAllText($tempOut, [System.Text.Encoding]::UTF8)
      # Verify the source-count header AND the actual cross-dir
      # finding aggregation: the tempDir verdict from B4 contributes
      # a PLAN-DRIFT BLOCKER; the tempDir2 verdict contributes a
      # SILENT-FAILURE BLOCKER. A regression that counted both input
      # files but dropped the second dir's finding would pass a header-
      # only check; assert both archetype subheaders AND the total
      # BLOCKER count.
      $sourceOk     = $reportText5 -match '2 verdict files across 2 reviews dir'
      $planDriftOk  = $reportText5 -match 'PLAN-DRIFT -- 1 findings'
      $silentFailOk = $reportText5 -match 'SILENT-FAILURE -- 1 findings'
      $totalOk      = $reportText5 -match 'BLOCKER=2'
      if ($sourceOk -and $planDriftOk -and $silentFailOk -and $totalOk) {
        Write-Host "[SelfTest] PASS B5: multi-dir aggregation scans both dirs and buckets both findings"
      } else {
        Write-Host "[SelfTest] FAIL B5: multi-dir aggregation incomplete; source=$sourceOk plan=$planDriftOk silent=$silentFailOk total=$totalOk"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B5: multi-dir invocation did not produce a report; exit=$subExit5"
      $script:failures++
    }

    # B6: one existing dir + one missing dir. Should succeed with a
    # skip-note in the header; the existing dir's verdict still
    # processes.
    $missingDir = Join-Path $env:TEMP ("trend-selftest-missing-" + [guid]::NewGuid().ToString('N').Substring(0,12))
    if (Test-Path -LiteralPath $tempOut) { Remove-Item -LiteralPath $tempOut -ErrorAction SilentlyContinue }
    $missingDirEsc = & $psLiteralEsc $missingDir
    $cmd6 = "& '$psCmdPathEsc' -ReviewsDir '$tempDirEsc','$missingDirEsc' -OutPath '$tempOutEsc'"
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -Command $cmd6 2>&1 | Out-Null
    $subExit6 = $LASTEXITCODE
    if ($subExit6 -eq 0 -and (Test-Path -LiteralPath $tempOut)) {
      $reportText6 = [System.IO.File]::ReadAllText($tempOut, [System.Text.Encoding]::UTF8)
      # Verify the skip-note AND that the surviving directory's
      # finding still processes. A regression that emitted the skip
      # note but silently dropped the existing-dir verdicts would
      # pass a header-only check.
      $skippedOk    = $reportText6 -match 'skipped non-existent'
      $survivingOk  = $reportText6 -match 'PLAN-DRIFT -- 1 findings'
      $totalOk      = $reportText6 -match 'BLOCKER=1'
      if ($skippedOk -and $survivingOk -and $totalOk) {
        Write-Host "[SelfTest] PASS B6: missing dir skipped; surviving dir's verdict processed"
      } else {
        Write-Host "[SelfTest] FAIL B6: skip-or-process incomplete; skipped=$skippedOk surviving=$survivingOk total=$totalOk"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B6: one-missing invocation should have succeeded; exit=$subExit6"
      $script:failures++
    }

    # B7: ALL configured dirs missing. Should fail closed (exit 1)
    # without writing a report.
    $missingA = Join-Path $env:TEMP ("trend-selftest-mA-" + [guid]::NewGuid().ToString('N').Substring(0,12))
    $missingB = Join-Path $env:TEMP ("trend-selftest-mB-" + [guid]::NewGuid().ToString('N').Substring(0,12))
    if (Test-Path -LiteralPath $tempOut) { Remove-Item -LiteralPath $tempOut -ErrorAction SilentlyContinue }
    $missingAEsc = & $psLiteralEsc $missingA
    $missingBEsc = & $psLiteralEsc $missingB
    $cmd7 = "& '$psCmdPathEsc' -ReviewsDir '$missingAEsc','$missingBEsc' -OutPath '$tempOutEsc'"
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -Command $cmd7 2>&1 | Out-Null
    $subExit7 = $LASTEXITCODE
    if ($subExit7 -ne 0 -and -not (Test-Path -LiteralPath $tempOut)) {
      Write-Host "[SelfTest] PASS B7: all-missing dirs fail closed (exit $subExit7, no report)"
    } else {
      Write-Host "[SelfTest] FAIL B7: all-missing dirs should fail closed; exit=$subExit7 report-exists=$(Test-Path -LiteralPath $tempOut)"
      $script:failures++
    }

    # B8: a multi-VERDICT file (the standard artifact shape -- the per-pass
    # concatenation auto-review.ps1 always writes for a multi-pass review).
    # The verdict distribution must take the UNION (worst) verdict, NOT the
    # first match: a file with pass1 CLEAN + pass2 BLOCKED is a BLOCKED outcome.
    # (Distinct from the multi-DIR fixtures B5/B6/B7 above: B8-B11 each use a
    # SINGLE reviews dir holding ONE multi-pass artifact.) Remove the B4
    # singleton verdict first so $tempDir holds only this fixture's input.
    Remove-Item -LiteralPath $verdict -ErrorAction SilentlyContinue
    $multi = Join-Path $tempDir 'review-20260608-000000-selftest-multi.md'
    $multiBody = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: none
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: CLEAN

===== REVIEW PASS 2/2 =====
PLAN-DRIFT: 1
  PLAN.md:7 -- multi-pass union selftest finding
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:7 -- multi-pass union selftest finding
"@
    [System.IO.File]::WriteAllText($multi, $multiBody, [System.Text.UTF8Encoding]::new($false))
    # Distinct output path per fixture: a child that exits without writing must
    # FAIL the fixture, not silently pass against a stale report. Remove first,
    # then assert Test-Path after.
    Remove-Item -LiteralPath $tempOutB8 -ErrorAction SilentlyContinue
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOutB8 2>&1 | Out-Null
    $subExit8 = $LASTEXITCODE
    if ($subExit8 -eq 0 -and (Test-Path -LiteralPath $tempOutB8)) {
      $rpt8 = [System.IO.File]::ReadAllText($tempOutB8, [System.Text.Encoding]::UTF8)
      # Assert on input-UNIQUE text so a wrong/stale report cannot satisfy the
      # generic count fragments.
      $b8Text = $rpt8 -match 'multi-pass union selftest finding'
      if (($rpt8 -match 'BLOCKED=1') -and ($rpt8 -match 'CLEAN=0') -and $b8Text) {
        Write-Host "[SelfTest] PASS B8: multi-VERDICT file counts union (worst) verdict as BLOCKED"
      } else {
        Write-Host "[SelfTest] FAIL B8: multi-VERDICT union verdict not BLOCKED (expected BLOCKED=1, CLEAN=0, own finding text; b8Text=$b8Text)"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B8: multi-VERDICT fixture invocation did not produce a report; exit=$subExit8"
      $script:failures++
    }

    # B9: a per-pass concatenation where the SAME finding line repeats across
    # both pass blocks. The analyzer must dedupe within the file so the finding
    # counts ONCE, not once per pass -- otherwise concatenation inflates
    # archetype counts and thresholds.
    Remove-Item -LiteralPath $multi -ErrorAction SilentlyContinue
    $dup = Join-Path $tempDir 'review-20260609-000000-selftest-dup.md'
    $dupBody = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: 1
  PLAN.md:9 -- repeated finding
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:9 -- repeated finding

===== REVIEW PASS 2/2 =====
PLAN-DRIFT: 1
  PLAN.md:9 -- repeated finding
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:9 -- repeated finding
"@
    [System.IO.File]::WriteAllText($dup, $dupBody, [System.Text.UTF8Encoding]::new($false))
    Remove-Item -LiteralPath $tempOutB9 -ErrorAction SilentlyContinue
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOutB9 2>&1 | Out-Null
    $subExit9 = $LASTEXITCODE
    if ($subExit9 -eq 0 -and (Test-Path -LiteralPath $tempOutB9)) {
      $rptDup9 = [System.IO.File]::ReadAllText($tempOutB9, [System.Text.Encoding]::UTF8)
      $b9Text = $rptDup9 -match 'repeated finding'
      if (($rptDup9 -match 'PLAN-DRIFT -- 1 findings') -and -not ($rptDup9 -match 'PLAN-DRIFT -- 2 findings') -and $b9Text) {
        Write-Host "[SelfTest] PASS B9: repeated finding across pass blocks deduped to 1"
      } else {
        Write-Host "[SelfTest] FAIL B9: repeated finding not deduped (expected PLAN-DRIFT -- 1 findings, own finding text; b9Text=$b9Text)"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B9: dup fixture invocation did not produce a report; exit=$subExit9"
      $script:failures++
    }

    # B10: the SAME finding TEXT under DIFFERENT severities across passes
    # (pass1 QUALITY, pass2 BLOCKER). The analyzer must dedupe by normalized
    # text and keep the HIGHEST severity, so it counts ONCE at BLOCKER -- not
    # once per (severity,text) pair.
    Remove-Item -LiteralPath $dup -ErrorAction SilentlyContinue
    $mixed = Join-Path $tempDir 'review-20260609-010000-selftest-mixedsev.md'
    $mixedBody = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: 1
  PLAN.md:9 -- mixed-severity dup
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: NON-BLOCKING

QUALITY: PLAN.md:9 -- mixed-severity dup

===== REVIEW PASS 2/2 =====
PLAN-DRIFT: 1
  PLAN.md:9 -- mixed-severity dup
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:9 -- mixed-severity dup
"@
    [System.IO.File]::WriteAllText($mixed, $mixedBody, [System.Text.UTF8Encoding]::new($false))
    Remove-Item -LiteralPath $tempOutB10 -ErrorAction SilentlyContinue
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOutB10 2>&1 | Out-Null
    $subExit10 = $LASTEXITCODE
    if ($subExit10 -eq 0 -and (Test-Path -LiteralPath $tempOutB10)) {
      $rptMixed10 = [System.IO.File]::ReadAllText($tempOutB10, [System.Text.Encoding]::UTF8)
      $countOnce = ($rptMixed10 -match 'PLAN-DRIFT -- 1 findings') -and -not ($rptMixed10 -match 'PLAN-DRIFT -- 2 findings')
      $unionBlocked = ($rptMixed10 -match 'BLOCKED=1') -and ($rptMixed10 -match 'NON-BLOCKING=0')
      # Prove the retained occurrence is the HIGHEST severity (BLOCKER), not the
      # QUALITY one: the "Total findings" tally must read BLOCKER=1, QUALITY=0.
      $keptHighest = $rptMixed10 -match 'Total findings: BLOCKER=1, QUALITY=0,'
      $b10Text = $rptMixed10 -match 'mixed-severity dup'
      if ($countOnce -and $unionBlocked -and $keptHighest -and $b10Text) {
        Write-Host "[SelfTest] PASS B10: mixed-severity dup deduped to 1 at highest severity (BLOCKER=1, QUALITY=0) + union BLOCKED"
      } else {
        Write-Host "[SelfTest] FAIL B10: mixed-severity dup not deduped to 1/highest (count_once=$countOnce union_blocked=$unionBlocked kept_highest=$keptHighest b10Text=$b10Text)"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B10: mixed-severity fixture invocation did not produce a report; exit=$subExit10"
      $script:failures++
    }

    # B11: the SAME finding under WHITESPACE + CASE variants across passes (pass1
    # "Norm   Variant FINDING" with collapsed-able spaces + caps, pass2 the same
    # text lower-cased with single spaces). A raw-trim implementation keys on the
    # trimmed-but-otherwise-verbatim text and would keep these as TWO findings;
    # only the normalized-text dedupe (collapse whitespace + lower-invariant)
    # collapses them to ONE. This pins the NORMALIZATION itself -- B8/B9/B10 repeat
    # BYTE-IDENTICAL text and would pass even a raw-trim dedupe.
    Remove-Item -LiteralPath $mixed -ErrorAction SilentlyContinue
    $norm = Join-Path $tempDir 'review-20260609-020000-selftest-normvar.md'
    $normBody = @"
===== REVIEW PASS 1/2 =====
PLAN-DRIFT: 1
  PLAN.md:13 -- Norm   Variant FINDING
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:13 -- Norm   Variant FINDING

===== REVIEW PASS 2/2 =====
PLAN-DRIFT: 1
  PLAN.md:13 -- norm variant finding
SILENT-FAILURE: none
TOMBSTONE-OR-SHIM: none
CROSS-CRATE-CONTRACT: none
LOADER-OR-ASSET-EDGE: none
CONVENTION-ADHERENCE: none
TEST-QUALITY: none
DOC-VS-CODE-DRIFT: none

VERDICT: BLOCKED

BLOCKER: PLAN.md:13 -- norm variant finding
"@
    [System.IO.File]::WriteAllText($norm, $normBody, [System.Text.UTF8Encoding]::new($false))
    Remove-Item -LiteralPath $tempOutB11 -ErrorAction SilentlyContinue
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOutB11 2>&1 | Out-Null
    $subExit11 = $LASTEXITCODE
    if ($subExit11 -eq 0 -and (Test-Path -LiteralPath $tempOutB11)) {
      $rptNorm11 = [System.IO.File]::ReadAllText($tempOutB11, [System.Text.Encoding]::UTF8)
      # Whitespace + case variants of one finding must dedupe to 1 (normalization),
      # NOT 2 (raw trim). Assert on the archetype subheader count.
      $b11Once = ($rptNorm11 -match 'PLAN-DRIFT -- 1 findings') -and -not ($rptNorm11 -match 'PLAN-DRIFT -- 2 findings')
      if ($b11Once) {
        Write-Host "[SelfTest] PASS B11: whitespace/case variants dedupe to 1 (normalized, not raw-trim)"
      } else {
        Write-Host "[SelfTest] FAIL B11: whitespace/case variants not normalized to 1 (raw-trim would leave 2)"
        $script:failures++
      }
    } else {
      Write-Host "[SelfTest] FAIL B11: normalization-variant fixture invocation did not produce a report; exit=$subExit11"
      $script:failures++
    }

    # B12: a MALFORMED verdict file drives the MAIN scan through the fail-closed
    # suspect abort -- Get-VerdictSuspectReason flags it, the loop records it in
    # $suspectFiles and skips it (continue), and the run exits 1 WITHOUT writing a
    # report (the corpus-integrity abort). Covers the scan -> suspect -> continue
    # -> abort -> named-offender path the in-memory S fixtures cannot reach. Uses a
    # categoryless `VERDICT: CLEAN` (suspect: the wrapper requires the full eight-
    # category block). Clears $tempDir first so it holds ONLY this input, and uses
    # a distinct output path removed first so a stale report cannot mask a failure.
    Get-ChildItem -LiteralPath $tempDir -Filter '*.md' -File -ErrorAction SilentlyContinue |
      ForEach-Object { Remove-Item -LiteralPath $_.FullName -ErrorAction SilentlyContinue }
    $suspect = Join-Path $tempDir 'review-20260608-000000-selftest-suspect.md'
    [System.IO.File]::WriteAllText($suspect, "VERDICT: CLEAN`n", [System.Text.UTF8Encoding]::new($false))
    Remove-Item -LiteralPath $tempOutB12 -ErrorAction SilentlyContinue
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -ReviewsDir $tempDir -OutPath $tempOutB12 2>&1 | Out-Null
    $subExit12 = $LASTEXITCODE
    if ($subExit12 -ne 0 -and -not (Test-Path -LiteralPath $tempOutB12)) {
      Write-Host "[SelfTest] PASS B12: a malformed (categoryless) verdict aborts the scan fail-closed (exit $subExit12, no report written)"
    } else {
      Write-Host "[SelfTest] FAIL B12: malformed verdict should abort fail-closed; exit=$subExit12 report-exists=$(Test-Path -LiteralPath $tempOutB12)"
      $script:failures++
    }
  } finally {
    # Remove files individually then the now-empty directories non-
    # recursively. Recursive cleanup is deliberately avoided here so
    # that a script bug pointed at the wrong dir cannot blow away a
    # tree of files. The SelfTest dirs only ever hold a few files;
    # the individual-file path is cheap and safe.
    # $tempOut and the per-fixture report outputs (the $tempOutB* paths) live in
    # $env:TEMP (outside $tempDir/$tempDir2), so remove them explicitly here.
    foreach ($p in @($tempOut, $tempOutB8, $tempOutB9, $tempOutB10, $tempOutB11, $tempOutB12)) {
      if ($p -and (Test-Path -LiteralPath $p)) {
        Remove-Item -LiteralPath $p -ErrorAction SilentlyContinue
      }
    }
    foreach ($d in @($tempDir, $tempDir2)) {
      if (Test-Path -LiteralPath $d) {
        Get-ChildItem -LiteralPath $d -File -ErrorAction SilentlyContinue | ForEach-Object {
          Remove-Item -LiteralPath $_.FullName -ErrorAction SilentlyContinue
        }
        Remove-Item -LiteralPath $d -ErrorAction SilentlyContinue
      }
    }
  }

  if ($failures -eq 0) {
    Write-Host "[SelfTest] All analyze-blocker-trends tests passed."
    exit 0
  } else {
    Write-Host "[SelfTest] $failures failures."
    exit 1
  }
}

# ---------------------------------------------------------------------------
# Resolve + validate the reviews directories.
# ---------------------------------------------------------------------------
# A path in $ReviewsDir that does not exist is SKIPPED (not an error)
# because the default scans both backend log dirs and a single-backend
# install will only have one of them. The script fails closed only when
# ZERO configured paths exist as containers.
$reviewsDirAbsList = New-Object 'System.Collections.Generic.List[string]'
$missingDirs = New-Object 'System.Collections.Generic.List[string]'
foreach ($dir in $ReviewsDir) {
  if (Test-Path -LiteralPath $dir -PathType Container) {
    $reviewsDirAbsList.Add( (Resolve-Path -LiteralPath $dir).Path )
  } else {
    $missingDirs.Add($dir)
  }
}
if ($reviewsDirAbsList.Count -eq 0) {
  Write-Host "[trends] ERROR: none of the configured reviews directories exist:"
  foreach ($d in $ReviewsDir) { Write-Host "[trends]          $d" }
  exit 1
}
foreach ($d in $missingDirs) {
  Write-Host "[trends] note: skipping non-existent reviews dir: $d"
}

$cutoff = if ($SinceDays -gt 0) { (Get-Date).AddDays(-$SinceDays) } else { [DateTime]::MinValue }
# Restrict to the actual verdict naming pattern emitted by auto-review.ps1:
# `review-<YYYYMMDD>-<HHMMSS>-<scope>.md`. The wrapper composes the timestamp
# via `Get-Date -Format 'yyyyMMdd-HHmmss'` and the scope tag (commit-<sha>,
# branch-<sha>-vs-<base>, staged, uncommitted) into that exact prefix; grep
# `auto-review.ps1` for the `$verdictFile = Join-Path` site to verify the
# contract if it ever changes. Filtering by pattern (not just *.md)
# closes the silent-success-on-mispointed-dir gap that even a verdict-line
# lower bound cannot: the prompt template at scripts/codex/review-prompt-
# template.md contains literal example `BLOCKER:`, `QUALITY:`, `NOTE:`, and
# `VERDICT: CLEAN` lines, so a `-ReviewsDir scripts/codex` run would
# otherwise overwrite the trend report with prompt examples masquerading as
# verdicts. (Historical regression.)
$verdictNameRe = [regex]'^review-\d{8}-\d{6}-.+\.md$'
$reviewFilesList = New-Object 'System.Collections.Generic.List[object]'
foreach ($abs in $reviewsDirAbsList) {
  $matched = Get-ChildItem -LiteralPath $abs -Filter '*.md' -File |
    Where-Object { $verdictNameRe.IsMatch($_.Name) -and $_.LastWriteTime -ge $cutoff }
  foreach ($f in $matched) { $reviewFilesList.Add($f) | Out-Null }
}
$reviewFiles = $reviewFilesList

if ($reviewFiles.Count -eq 0) {
  Write-Host "[trends] ERROR: no verdict-shaped .md files found across configured reviews dirs:"
  foreach ($d in $reviewsDirAbsList) { Write-Host "[trends]          $d" }
  Write-Host "[trends]        Expected pattern: review-<YYYYMMDD>-<HHMMSS>-<scope>.md"
  Write-Host "[trends]        cutoff: $cutoff"
  exit 1
}

# ---------------------------------------------------------------------------
# Parse each verdict file: capture VERDICT line + per-severity findings.
# ---------------------------------------------------------------------------
# Storage shape:
#   $findings = array of @{
#     File       = <verdict shortname>
#     FileDate   = <DateTime>
#     Severity   = 'BLOCKER' | 'QUALITY' | 'NON-BLOCKER' | 'NOTE'
#     Text       = <single-line finding summary>
#     Archetype  = <classifier output>
#   }

$findings = New-Object 'System.Collections.Generic.List[object]'
# Fail-closed quarantine list: any verdict file Get-VerdictSuspectReason flags
# (GATE-FAILED-CLOSED banner, truncated/missing verdict, or a malformed pass
# sequence / incoherent block) is recorded here and skipped; the abort below
# names EVERY offender and refuses to write a report that would silently drop or
# miscount them. Because a flagged file never reaches the counting below, no
# unparseable "OTHER" bucket is reachable -- every counted file has a valid-word
# VERDICT or a BLOCKER (see the BLOCKER-precedence union in the loop).
$suspectFiles = New-Object 'System.Collections.Generic.List[string]'
$verdictCounts = @{ CLEAN = 0; 'NON-BLOCKING' = 0; BLOCKED = 0 }
$oldestMtime = [DateTime]::MaxValue
$newestMtime = [DateTime]::MinValue

# Severity prefixes the analyzer recognizes. Both QUALITY (post-rewrite) and
# legacy NON-BLOCKER are parsed because the verdict log spans the rename;
# the report buckets them separately so the rename is visible in trends.
$severityRe = [regex]'(?m)^(?<sev>BLOCKER|QUALITY|NON-BLOCKER|NOTE):\s*(?<text>.*)$'
# The VERDICT regex is case-INSENSITIVE (mirroring the wrappers' `-match '^VERDICT:'`
# + case-insensitive word compare in Get-VerdictExitCode), while $severityRe above
# stays case-SENSITIVE -- the wrapper counts `^BLOCKER:` etc. case-sensitively. This
# asymmetry is the wrapper's; see Get-VerdictSuspectReason's case-policy note.
$verdictLineRe = [regex]'(?im)^VERDICT:\s*(?<v>CLEAN|NON-BLOCKING|BLOCKED)\s*$'

foreach ($file in $reviewFiles) {
  $shortName = $file.Name
  $fileFindings = @{}  # reset per file: dedupe by normalized text, keep highest severity
  if ($file.LastWriteTime -lt $oldestMtime) { $oldestMtime = $file.LastWriteTime }
  if ($file.LastWriteTime -gt $newestMtime) { $newestMtime = $file.LastWriteTime }

  # Verdict files are written by auto-review.ps1 as UTF-8 NO BOM. Read
  # explicitly with UTF-8 because Get-Content -Raw under Windows
  # PowerShell 5.1 uses the legacy ANSI default codepage and corrupts
  # multi-byte glyphs (em-dashes, smart quotes) into mojibake. The
  # corrupted text then propagates verbatim into the generated trend
  # report. (Historical regression.)
  $content = [System.IO.File]::ReadAllText($file.FullName, [System.Text.Encoding]::UTF8)

  # FAIL-CLOSED: a GATE-FAILED-CLOSED or unparseable / malformed / incoherent
  # verdict file is suspect. Record it (with its dir + reason) and skip parsing;
  # the abort after the loop names EVERY offender. Never counted toward the
  # report, never silently dropped -- the run aborts before any report is
  # written. Applies dispatch-checklist's SUPERSET trustworthiness bar (everything
  # the wrapper's Get-VerdictExitCode rejects PLUS row-balance / invalid-tail /
  # wrong-case severity), so any verdict EITHER log-consumer would reject cannot
  # pollute the trends corpus.
  $suspectReason = Get-VerdictSuspectReason -Content $content -VerdictWordRe $verdictLineRe
  if ($null -ne $suspectReason) {
    # Record the FULL path (not just $shortName): the default scan covers both the
    # codex and claude review dirs, so same-named verdicts across dirs would be
    # ambiguous by filename alone. The full path lets the operator locate and
    # quarantine the exact offender.
    $suspectFiles.Add("$($file.FullName) -- $suspectReason") | Out-Null
    continue
  }

  # A verdict file may carry MULTIPLE VERDICT: lines -- auto-review.ps1's .md
  # artifact is the per-pass concatenation of every review pass (one VERDICT
  # block per pass). Take the UNION (worst) verdict for the distribution, NOT
  # the first match: a pass1-CLEAN / pass2-BLOCKED file is a BLOCKED outcome,
  # not CLEAN. A single-pass artifact has exactly one VERDICT: line and reduces
  # to the same answer. (Per-pass finding lines below are deduped per file by
  # normalized text, keeping the highest severity, so a finding that repeats
  # across passes counts once -- otherwise the multi-pass concatenation
  # inflates archetype counts and thresholds.)
  # Union (worst) verdict with wrapper BLOCKER-PRECEDENCE (Get-VerdictExitCode):
  # a BLOCKER finding forces BLOCKED even when the VERDICT word is CLEAN or
  # malformed. Every file here passed Get-VerdictSuspectReason, so it has EITHER
  # a valid-word VERDICT ($vAll non-empty) OR a BLOCKER finding -- the branches
  # are exhaustive and no unparseable "OTHER" bucket is reachable. The `^BLOCKER:`
  # test is CASE-SENSITIVE (-cmatch), matching the wrapper and the $severityRe
  # extraction below (a lowercase `blocker:` is not a finding to either).
  $vAll = @($verdictLineRe.Matches($content) | ForEach-Object { $_.Groups['v'].Value })
  if (($content -cmatch '(?m)^BLOCKER:') -or ($vAll -contains 'BLOCKED')) {
    $verdictCounts['BLOCKED']++
  } elseif ($vAll -contains 'NON-BLOCKING') {
    $verdictCounts['NON-BLOCKING']++
  } else {
    $verdictCounts['CLEAN']++
  }

  foreach ($m in $severityRe.Matches($content)) {
    $sev = $m.Groups['sev'].Value
    $text = $m.Groups['text'].Value.Trim()
    if ([string]::IsNullOrWhiteSpace($text)) { continue }
    # Dedupe repeated pass findings per file by NORMALIZED finding text
    # (severity excluded from the key), keeping the HIGHEST severity seen for
    # that text across the per-pass concatenation -- so one issue surfacing in
    # multiple passes counts ONCE, and the same finding reported as BLOCKER in
    # one pass and QUALITY in another counts once at BLOCKER. Counting once per
    # pass, or once per (severity,text) pair, inflates archetype counts and
    # thresholds.
    $textKey = ($text -replace '\s+', ' ').ToLowerInvariant()
    $rank = switch ($sev) { 'BLOCKER' { 3 } 'QUALITY' { 2 } 'NON-BLOCKER' { 2 } 'NOTE' { 1 } default { 0 } }
    if (-not $fileFindings.ContainsKey($textKey) -or $fileFindings[$textKey].Rank -lt $rank) {
      $fileFindings[$textKey] = @{ Sev = $sev; Text = $text; Rank = $rank }
    }
  }

  foreach ($ff in $fileFindings.Values) {
    $arch = Get-FindingArchetype -Text $ff.Text
    $findings.Add([pscustomobject]@{
      File      = $shortName
      FileDate  = $file.LastWriteTime
      Severity  = $ff.Sev
      Text      = $ff.Text
      Archetype = $arch
    }) | Out-Null
  }
}

# ---------------------------------------------------------------------------
# Fail-closed abort on ANY suspect (fail-closed / malformed) evidence, across
# ALL scanned dirs -- BEFORE emitting a report. Names every offender so it can
# be quarantined (moved out of the scanned dir) or repaired. A GATE-FAILED-CLOSED
# verdict, or a file with no parseable / coherent verdict, is neither counted nor
# silently dropped: it aborts the whole run. (Get-VerdictSuspectReason.)
# ---------------------------------------------------------------------------
if ($suspectFiles.Count -gt 0) {
  Write-Host "[trends] ERROR: $($suspectFiles.Count) scanned verdict file(s) are fail-closed or malformed; refusing to write a report that would silently drop or miscount them:"
  foreach ($s in $suspectFiles) { Write-Host "[trends]   - $s" }
  Write-Host "[trends]        Quarantine (move out of the scanned dir) or repair the file(s), then re-run."
  exit 1
}

# ---------------------------------------------------------------------------
# Fail-closed lower bound on parseable verdicts.
# ---------------------------------------------------------------------------
# The mere presence of *.md files in the configured reviews dirs is
# not enough proof that we're looking at review verdicts -- a
# mispointed `-ReviewsDir docs` run would happily find arbitrary
# markdown and produce a zero-finding "trend report" that overwrites
# the docs/blocker-trends.md output. Require at least one recognized
# `VERDICT:` line OR at least one severity prefix across the entire
# input set. If neither is present, the input is not a verdict log
# and the analyzer fails closed instead of writing a silently-empty
# report. (Historical regression.)
$parseableVerdicts = $verdictCounts.CLEAN + $verdictCounts['NON-BLOCKING'] + $verdictCounts.BLOCKED
if ($parseableVerdicts -eq 0 -and $findings.Count -eq 0) {
  $scannedDirsJoined = ($reviewsDirAbsList -join ', ')
  Write-Host "[trends] ERROR: scanned $($reviewFiles.Count) *.md files across configured reviews dir(s) [$scannedDirsJoined] but found zero recognized VERDICT: lines and zero severity-prefixed findings."
  Write-Host "[trends]        The directories do not appear to contain review verdicts (codex or claude backend)."
  Write-Host "[trends]        Refusing to overwrite '$OutPath' with an empty trend report."
  exit 1
}

# ---------------------------------------------------------------------------
# Emit the markdown report.
# ---------------------------------------------------------------------------

$timestamp = (Get-Date).ToString('yyyy-MM-ddTHH:mm:sszzz')
$dateRange = "{0:yyyy-MM-dd} to {1:yyyy-MM-dd}" -f $oldestMtime, $newestMtime

# Wrap each Where-Object result in @() so an empty result is an empty
# array (whose .Count is 0) rather than $null (whose .Count evaluates to
# nothing and prints as blank in the wrapper log line).
$blockerCount = @($findings | Where-Object { $_.Severity -eq 'BLOCKER' }).Count
$qualityCount = @($findings | Where-Object { $_.Severity -eq 'QUALITY' }).Count
$legacyNbCount = @($findings | Where-Object { $_.Severity -eq 'NON-BLOCKER' }).Count
$noteCount = @($findings | Where-Object { $_.Severity -eq 'NOTE' }).Count

$sb = [System.Text.StringBuilder]::new()
[void]$sb.AppendLine('# Cross-review BLOCKER Trends Report')
[void]$sb.AppendLine('')
[void]$sb.AppendLine("Generated: $timestamp")
$sourceDirsJoined = ($ReviewsDir | ForEach-Object { "``$_``" }) -join ', '
[void]$sb.AppendLine("Source: $($reviewFiles.Count) verdict files across $($reviewsDirAbsList.Count) reviews dir(s): $sourceDirsJoined")
if ($missingDirs.Count -gt 0) {
  $missingJoined = ($missingDirs | ForEach-Object { "``$_``" }) -join ', '
  [void]$sb.AppendLine("(skipped non-existent: $missingJoined)")
}
[void]$sb.AppendLine("Date range (file mtime): $dateRange")
[void]$sb.AppendLine("Verdict distribution: CLEAN=$($verdictCounts.CLEAN), NON-BLOCKING=$($verdictCounts['NON-BLOCKING']), BLOCKED=$($verdictCounts.BLOCKED)")
[void]$sb.AppendLine("Total findings: BLOCKER=$blockerCount, QUALITY=$qualityCount, legacy NON-BLOCKER=$legacyNbCount, NOTE=$noteCount")
[void]$sb.AppendLine('')
[void]$sb.AppendLine('## Purpose')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('This report exists to reduce REPEAT-CLASS BLOCKERs by improving worker')
[void]$sb.AppendLine('discipline upstream. It is NOT a review-throughput metric. Use it to')
[void]$sb.AppendLine('identify archetypes worth folding into worker dispatch templates,')
[void]$sb.AppendLine('AGENTS.md, the review prompt''s hazards section, or an anti-pattern')
[void]$sb.AppendLine('guide. The classifier follows the same "one primary category per')
[void]$sb.AppendLine('finding" discipline as the review prompt template, so archetypes here')
[void]$sb.AppendLine('are directly comparable to the prompt''s category enumeration.')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('Cycle-count and time-to-merge statistics are intentionally omitted.')
[void]$sb.AppendLine('Quality of merged code is the only metric.')
[void]$sb.AppendLine('')

function Write-ArchetypeSection {
  param(
    [string]$Title,
    [string]$Severity,
    [System.Collections.Generic.List[object]]$All
  )
  # @() wrap so a single-result bucket stays an array; without it
  # PowerShell 5.1 returns a scalar PSObject whose .Count evaluates to
  # the wrong value (blank in the section header). Same defect pattern
  # as the top-level severity counts above. (Historical regression.)
  $bucket = @($All | Where-Object { $_.Severity -eq $Severity })
  [void]$sb.AppendLine("---")
  [void]$sb.AppendLine('')
  [void]$sb.AppendLine("## $Title ($($bucket.Count) findings)")
  [void]$sb.AppendLine('')
  if ($bucket.Count -eq 0) {
    [void]$sb.AppendLine('_(none)_')
    [void]$sb.AppendLine('')
    return
  }
  # Group by archetype, sort by count desc, then by archetype name asc
  # so equal-count groups have a stable order (without the secondary
  # key, group order depends on PowerShell/filesystem ordering and the
  # generated report churns across runs). The hashtable-expression
  # form is the PowerShell idiom for multi-key sort.
  $byArch = $bucket | Group-Object -Property Archetype | Sort-Object `
    @{ Expression = 'Count'; Descending = $true }, `
    @{ Expression = 'Name';  Descending = $false }
  foreach ($g in $byArch) {
    $arch = $g.Name
    $entries = $g.Group | Sort-Object -Property FileDate -Descending
    $newest = $entries[0].FileDate.ToString('yyyy-MM-dd')
    $oldest = $entries[-1].FileDate.ToString('yyyy-MM-dd')
    [void]$sb.AppendLine("### $arch -- $($g.Count) findings")
    [void]$sb.AppendLine("Newest: $newest. Oldest: $oldest.")
    [void]$sb.AppendLine('')
    $examples = $entries | Select-Object -First 5
    [void]$sb.AppendLine('Top examples (most recent first):')
    foreach ($e in $examples) {
      $dateStr = $e.FileDate.ToString('yyyy-MM-dd')
      [void]$sb.AppendLine("- ``$($e.File)`` [$dateStr]: $($e.Text)")
    }
    if ($g.Count -gt 5) {
      [void]$sb.AppendLine("_(+$($g.Count - 5) more)_")
    }
    [void]$sb.AppendLine('')
  }
}

Write-ArchetypeSection -Title 'BLOCKER archetype clusters (most frequent first)' -Severity 'BLOCKER' -All $findings
Write-ArchetypeSection -Title 'QUALITY findings (post-rewrite severity tier)' -Severity 'QUALITY' -All $findings
Write-ArchetypeSection -Title 'Legacy NON-BLOCKER findings (pre-rewrite)' -Severity 'NON-BLOCKER' -All $findings
Write-ArchetypeSection -Title 'NOTE findings (informational, non-gating)' -Severity 'NOTE' -All $findings

[void]$sb.AppendLine("---")
[void]$sb.AppendLine('')
[void]$sb.AppendLine('## Methodology')
[void]$sb.AppendLine('')
$cutoffNote = if ($SinceDays -gt 0) {
  " whose file mtime is within the last $SinceDays days (``-SinceDays $SinceDays``)"
} else {
  ' (no mtime cutoff; the full history is in scope)'
}
[void]$sb.AppendLine("Source: ``*.md`` files under the configured reviews dir(s) ($sourceDirsJoined)")
[void]$sb.AppendLine('whose name matches the verdict-shape pattern')
[void]$sb.AppendLine('`review-<YYYYMMDD>-<HHMMSS>-<scope>.md` (this is the exact prefix that')
[void]$sb.AppendLine('`auto-review.ps1` emits, so the filter excludes non-verdict markdown that')
[void]$sb.AppendLine("may happen to live in the directory)$cutoffNote.")
[void]$sb.AppendLine('Each selected file is a Codex or Claude verdict (the analyzer treats')
[void]$sb.AppendLine('both backends identically; the verdict format is shared). The analyzer extracts finding')
[void]$sb.AppendLine('lines matched by the regex `^(BLOCKER|QUALITY|NON-BLOCKER|NOTE):` and')
[void]$sb.AppendLine('classifies the one-line summary into an archetype via a most-specific-')
[void]$sb.AppendLine('first keyword list (see `scripts/codex/analyze-blocker-trends.ps1` for')
[void]$sb.AppendLine('the keyword sets).')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('A populated `UNCLASSIFIED` bucket is a signal that the keyword list')
[void]$sb.AppendLine('needs to grow. Treat that section as "next archetypes to name."')
[void]$sb.AppendLine('')
[void]$sb.AppendLine('Re-run with `-SinceDays <N>` to scope to a recent window.')

# Ensure output directory exists.
$outDir = Split-Path -Parent -Path $OutPath
if ($outDir -and -not (Test-Path -LiteralPath $outDir -PathType Container)) {
  New-Item -ItemType Directory -Path $outDir -Force | Out-Null
}

$outAbs = if ([System.IO.Path]::IsPathRooted($OutPath)) {
  $OutPath
} else {
  Join-Path (Get-Location) $OutPath
}

try {
  [System.IO.File]::WriteAllText($outAbs, $sb.ToString(), $utf8NoBom)
} catch {
  Write-Host "[trends] ERROR: failed to write report to $outAbs : $_"
  exit 1
}

Write-Host "[trends] wrote report to $outAbs"
Write-Host "[trends]   verdicts scanned: $($reviewFiles.Count)"
Write-Host "[trends]   total findings:   BLOCKER=$blockerCount  QUALITY=$qualityCount  legacy NON-BLOCKER=$legacyNbCount  NOTE=$noteCount"
exit 0
