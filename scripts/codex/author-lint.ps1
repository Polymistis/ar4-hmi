#Requires -Version 5.1
<#
.SYNOPSIS
  Author-side mechanical linter for narrative / PLAN / doc content, plus the
  count-drift classes (inventory-count + magnitude) over staged source-file COMMENT lines.

.DESCRIPTION
  Catches, BEFORE the review gate runs, the small set of mechanical defect
  classes the Codex/Claude gate repeatedly raises on authored Markdown and
  PLAN content -- and, in staged mode, the count-drift classes (inventory-count +
  magnitude) over the COMMENT lines of staged .rs/.ps1/.toml/.sh source files
  (code lines are never scanned). These classes are script-detectable at ~0 token
  cost, so running this pre-pass keeps the gate a FAILSAFE rather than a debugger.
  The gate stays authoritative for everything this linter does not encode.

  Run it before the first gate commit and paste its report into the
  completion report. It ALSO runs mechanically: scripts/git-hooks/pre-commit
  invokes it in staged mode as a fail-fast pre-pass before the AI backend --
  ERROR-tier findings abort the commit there; CROSS_REVIEW_SKIP_AUTHOR_LINT=1 skips
  ONLY that pre-pass (the AI review still runs; the escape for a linter
  false positive). The hook executes HEAD's copy of this file (tempdir
  extraction, never the working tree), so an edit here takes effect on the
  NEXT commit after it lands. Manual runs remain the author-side default
  before the first gate commit of a round.

  Check classes (v1). Tiers were set by CALIBRATION against the real doc tree
  (`-Tree`), not a priori: an error-tier check must fire ZERO false positives
  on the current (gate-clean) tree; one that cannot is downgraded to advisory.
    ERROR-tier (any finding -> exit 3; exit 1 is deliberately NOT used for
    findings -- an uncaught crash under $ErrorActionPreference='Stop' makes
    `powershell -File` exit 1, so the pre-commit pre-pass reads 3 as
    "findings, abort" and 1/anything-else as "linter broke, warn and fail
    open to the AI gate"):
      LINE-ANCHOR  bare `<name>.<ext>:<line>` source citations in tracked
                   narrative docs (both `/` and `\` separators). Violates the
                   symbol-anchor rule (name the grep-able symbol, not the
                   drifting line number). ERROR-tier, EXCEPT a doc carrying the
                   EXPLICIT frozen-snapshot MARKER -- an HTML comment on its own
                   line near the top, matched by the regex
                   `^\s*<!--\s*frozen-snapshot\s*-->\s*$` (case-insensitive) within
                   the doc's first ~30 lines (Test-HasFrozenSnapshotMarker) -- which
                   DOWNGRADES to advisory, honoring the rule's exemption for frozen
                   snapshot line numbers. FILENAME AND DATE NO LONGER MATTER: a
                   dated-in-name doc WITHOUT the marker stays ERROR (it may be live
                   authority), and a NON-dated doc WITH the marker is exempt. The
                   author declares frozen status explicitly; there is no filename
                   inference and no env-var denylist. A machine-GENERATED report is
                   skipped entirely (machine-emitted format, not authored prose) but
                   ONLY when it BOTH sits at a known tracked generated-report path
                   (docs/blocker-trends.md; $script:GeneratedDocPathPattern)
                   AND carries a generator signature -- a `Generated:` stamp alone
                   is NOT enough, so an authored doc cannot self-exempt
                   (Test-IsGeneratedContent). The full-tree audit surfaces all real
                   bare anchors, including any pre-existing debt elsewhere in the doc tree.
      SELF-NARR    narration of the PENDING change's OWN landing -- the deictic
                   "this/here + landing-event" form (Get-SelfNarrationFindings):
                   "this merge" / "this landing", "as of this entry" /
                   "add(s|ing) this entry", and "landed ... here" (a deictic
                   "here" within a short word window of "landed"). ERROR when the
                   doc is History.md (the retrospective landing log, where a
                   not-yet-landed self-reference is a factual defect -- the merge
                   SHA is unknowable while the entry rides the branch); ADVISORY in
                   any other narrative doc. CALIBRATED so the ERROR tier fires only
                   on genuine self-narration: retrospective prose about an
                   already-landed merge ("the merge landed at <sha>", "merged
                   cleanly") has no this/here pointer and stays clean, and a bare
                   "this entry" is NOT matched (legitimate timestamping like "at
                   the time of this entry" would false-positive). History.md is the
                   package's illustrative landing-log convention; a consumer without
                   one simply never trips the ERROR tier (all forms stay advisory).
      LOCAL-PROOF  a proof/verification CLAIM co-located with a LOCAL-ONLY evidence
                   reference (Get-LocalEvidenceFindings). ERROR only in the TIGHT
                   form -- a word-bounded proof word (prove/proves/proved/proven/
                   proving | verify/verifies/verified/verifying/verification/
                   verifiable; word-bounded so "approve"
                   never matches) on the same line as the gitignored review-artifact
                   PATH `logs/(codex|claude)/reviews` (either `/` or `\` separator,
                   Windows-first), OR a `proof`/`evidence` NOUN within a short
                   window of that path (so "Proof: <path>" flags while a distant
                   "evidence snapshot ... <path>" description does not). That path
                   is THE local evidence store, and the full-tree calibration found
                   the pairing does not arise in legitimate prose, so the tight form is
                   high-precision ERROR. The looser marker variants are ADVISORY
                   (see below) by the same calibration.
      INVENTORY    an EXACT INVENTORY/COUNT of repo/project state asserted in
                   prose (Get-InventoryAssertionFindings) -- a cardinal number
                   the moment the inventory changes goes stale and no reader can
                   trust it. Noun-AGNOSTIC, verb-gated shapes: "there
                   is|are N <word>", "N <word> is|are defined|tracked|supported|
                   registered|covered|available|in place", "covers|contains|
                   includes|totals N <word>". ERROR in .md prose -- EXCEPT a count
                   inside a DATED entry body of the ROOT History.md log, which is
                   SKIPPED (a dated entry is a static, append-only snapshot whose
                   counts cannot drift). The exemption is CALLER-applied by
                   Invoke-FileChecks and narrow on TWO axes -- root-path (NOT a
                   nested History.md) + a dated-entry-body mask (the preamble and
                   undated `## ` sections stay in class) -- not inside this detector
                   (2026-07-09 operator decision). NO frozen-snapshot downgrade
                   otherwise -- the marker exemption is line-anchor-scoped (the digit lookbehind
                   excludes identifier/version/date/SHA digits; a FULL illustrative
                   inline-code example -- the whole claim in one backtick span -- is
                   exempt so a doc can NAME the class, while a claim whose number
                   ALONE is backticked still flags). This SAME detector ALSO runs over the
                   COMMENT lines of staged .rs/.ps1/.toml/.sh source files
                   ($script:SourceExtPattern; Get-StagedSourceFindings), one tier
                   SOFTER (ADVISORY) there since comment extraction has edge cases,
                   with a code-echo exemption (a comment count equal to a code
                   literal on a NEARBY line -- within a small window -- is
                   describing that adjacent constant, not asserting an inventory;
                   an unrelated same number elsewhere in the file does NOT
                   exempt). Fix direction: magnitude phrasing or delete the
                   sentence -- never a "corrected" number, which just re-seeds the
                   drift. The reviewer-side contract mirrors this in
                   review-prompt-template.md.
    ADVISORY-tier (reported; does NOT by itself set the findings exit):
      DEAD-REF     a repo-relative path (first segment a real repo top-level
                   DIRECTORY) that is ABSENT FROM THE TRACKED INDEX -- a typo or
                   a dead/local-only reference a fresh clone cannot resolve (e.g.
                   a deleted or gitignored-only
                   logs/codex/probe-pack-45ff). Machine-local operational targets
                   (~/, drive-absolute, $env:, <placeholder>, globs, URLs) are
                   exempt; dated docs are NOT (a dead path there is still
                   surfaced as advisory). ADVISORY in v1 BY
                   CALIBRATION: the full-tree run surfaced irreducible false
                   positives that no mechanical rule separates from real dead
                   refs -- a gitignored OPERATIONAL target
                   (a generated report under logs/) is indistinguishable
                   from a gitignored EVIDENCE path by ignore
                   status; plus intentional "this path does not exist"
                   references and doc-relative paths in package docs. Surfaces
                   candidates for author review; the gate stays the failsafe.
                   (Promoting to error-tier needs doc-relative resolution +
                   an evidence-vs-instruction signal -- a future extension.)
      TAG          a milestone tag `M<n>...` referenced in PLAN.md whose
                   top-level milestone `M<n>` has no defining header. Advisory
                   in v1: PLAN's suffix scheme is rich, so only a wholly
                   unknown top-level milestone is surfaced (zero false
                   positives over finer suffixes). Promote to a finer
                   closure check once calibrated against the full tag scheme.
      MAGNITUDE    an exact transient count in narrative prose where a magnitude
                   statement ("many rounds") carries the same information.
                   Implemented subset: round/iteration/attempt/pass/retry/try
                   counts in digit-first ("6 rounds") and unit-first ("round 6")
                   forms, PLUS the UNIT-AGNOSTIC verb form `took N` -- the count
                   after "took" regardless of the trailing noun, so "took 6
                   minutes" / "took 6 files" also match. The digit-first form
                   ADDITIONALLY covers the aggregate/duration nouns
                   (task/commit/file/row/site/entry/follow-up/finding/fixture/
                   assertion/instance/wave/stream + minute/hour/second/day), so a
                   bare duration "6 minutes", an assertion total "47 assertions",
                   and an "18-task chain" all match; those nouns stay OUT of the
                   unit-first form because their unit-first shape is usually a
                   stable NAME ("Wave 2", "day 3", "row 7"), not a count.
                   ENUMERATED-CASE totals extend this family: a spelled-out "all
                   <N> <enumeration-noun>" / "both <noun>" ("all three cases"),
                   and a `(1) ... (2) ...` inline enumeration paired with an "all
                   N" total, both restate an exact count a magnitude phrasing ("all
                   cases" / "every case") carries. Advisory: some counts are
                   load-bearing; the author judges. EXEMPT (like INVENTORY) for a
                   count inside a DATED entry body of the ROOT History.md log: a
                   dated entry is a static append-only snapshot, so the caller
                   Invoke-FileChecks DROPS both count classes on those lines
                   (root-path + dated-entry-body mask; the preamble, undated
                   sections, and nested History.md stay in class). (2026-07-09.)
      LOCAL-PROOF  (advisory forms; the ERROR tight form is above) a proof word
                   with a BARE local marker gitignored | local-only | untracked,
                   OR the loose "operator-side"/"local" + "evidence". These are
                   ADVISORY by CALIBRATION: the current tree co-locates these
                   markers with proof words in ordinary git prose ("verify ...
                   --untracked-files") and in META-DISCUSSION of the rule itself
                   ("gitignored logs ... cannot prove provenance"), neither of
                   which is the defect -- so the class is surfaced, not blocked
                   (mirrors the DEAD-REF advisory-by-calibration outcome).
      LOCAL-STATE  a machine-local install/wiring state asserted as repo fact
                   (Get-MachineLocalStateFindings): an "installed and wired" /
                   "is wired via" / "wired via" claim on the same line as a
                   MACHINE-LOCAL config path (~/.claude ... or
                   .claude/settings.local.json; either `/` or `\` separator) -- state a fresh clone cannot
                   confirm. EXEMPT when the line already flags it "machine-local" /
                   "local-only" (how the hook docs correctly caveat it), so only
                   UN-caveated assertions surface. Advisory: many legitimate
                   operational instructions name these paths; the author confirms.

  DEFERRED (NOT built in v1 -- documented so a later session knows the
  boundary; building fragile heuristics for these now would add false
  positives, so they are left to the gate):
    - canonical-field presence on touched PLAN milestones (needs a declared
      PLAN field schema),
    - enumeration-vs-source diff (needs machine-readable enumeration
      declarations in the doc),
    - quantitative-claim re-derivation against a named source list.

.PARAMETER Staged
  Default scope. Scan the ADDED lines of the staged diff
  (`git diff --cached --unified=0`) across tracked narrative docs, PLUS the
  ADDED COMMENT lines of staged .rs/.ps1/.toml/.sh source files (the
  inventory-count + magnitude classes only; code lines are never scanned). This
  is the author-side pre-commit use: it flags only what THIS commit introduces.
  Milestone-tag closure runs over the full staged PLAN.md content when PLAN.md
  is staged (closure is a whole-file property).

.PARAMETER Paths
  Scan the full current content of the named files instead of the staged diff.
  Narrative docs only -- source-file comment scanning is staged-mode-specific
  (it needs the added-line diff and its nearby-code context).

.PARAMETER Tree
  Scan the full content of every tracked narrative doc. Used for calibration
  and full audits.

.PARAMETER OutPath
  Optional. Also write the report to this path (best-effort sibling-temp + move;
  the report always prints to stdout regardless).

.PARAMETER SelfTest
  Run in-memory fixtures (positive / negative / exemption) for every check class
  -- LINE-ANCHOR (LA*), DEAD-REF (DR*), frozen-snapshot marker (SNAP*), TAG*,
  MAGNITUDE (MAG*, incl. the enumerated-case totals MAG17-23), SELF-NARR (SN*,
  incl. the History-ERROR-vs-elsewhere-ADVISORY tier and the retrospective/
  timestamping negatives), LOCAL-PROOF (LP*, the tight-ERROR path form vs the
  calibrated-down adjective/loose ADVISORY forms), LOCAL-STATE (LS*), INVENTORY
  (INV*, the verb-gated inventory-count SHAPE: ERROR in .md with
  inline-code/lookbehind exemptions and NO frozen-snapshot downgrade -- the
  marker exemption is line-anchor-scoped), and the source-comment path
  (SRC*, Get-SourceLineParts comment/code split per extension + the staged
  ADVISORY inventory + magnitude scan + the nearby-line code-echo exemption +
  comment-fence context incl. block-doc `*`-continuation lines) --
  the path/snapshot/index classifiers, AND the staged per-file DECISION logic
  (Get-StagedFileFindings: added-line filter + whole-file PLAN TAG bypass, where
  several gate BLOCKERs hid, PLUS the root-History.md dated-entry-body count-drift
  exemption STG7* and its dated-entry-body mask helper Get-HistoryEntryBodyLineSet
  HEB*), the source-file SELECTION pattern (SEL*, the one
  source-specific step of the staged-source loop), PLUS E2E child-process cases:
  E2E1/E2E2 pin the findings EXIT CONTRACT the pre-commit pre-pass switches on
  (error-tier `-Paths` doc -> exit 3, clean -> exit 0), and E2E3/E2E4 exercise the
  staged-source LOOP end to end via Invoke-E2EStagedSource -- it seeds an ALTERNATE
  index (GIT_INDEX_FILE) from HEAD, stages a temp `.rs` into it (asserting the git
  setup succeeded), runs staged scope as a child, and CHECKS all temp artifacts
  (alt index, its git .lock, probe source) are cleaned -- so the real worktree
  index is NEVER touched. E2E3 asserts a
  comment inventory claim surfaces (advisory, exit 0); E2E4 asserts a CODE-ONLY
  change takes the no-content report path, not a scanned-file result. These E2E
  cases resolve the repo root via git and write/single-file-remove temp files, so
  SelfTest is NOT purely in-memory anymore (it needs a git repo + filesystem
  writes; still no network, no temp git REPO -- E2E3/E2E4 use an alternate INDEX
  FILE in the SAME repo, removable with a single-file delete). Repo-LEVEL
  gate flows stay covered by the pre-commit hook's own `--self-test` (which builds
  + tears down its own scratch repo with a no-recurse delete); this script still
  avoids temp git repos since ITS cleanup would need a recursive `.git` removal,
  which the package cleanup policy avoids. (Codex TEST-QUALITY.)

.NOTES
  PowerShell 5.1 compatible. Sibling of scripts/codex/dispatch-checklist.ps1;
  mirrors its UTF-8 (read UTF-8 / write UTF-8-no-BOM) and pure-helper-plus-
  injected-seam testability conventions. It does NOT mirror that script's
  MoveFileEx atomic write: the optional -OutPath report uses a best-effort move
  (the report always prints to stdout, so the file copy is a convenience).
#>
[CmdletBinding()]
param(
  [switch]$Staged,
  [string[]]$Paths,
  [switch]$Tree,
  [string]$OutPath,
  [switch]$SelfTest
)

$ErrorActionPreference = 'Stop'
$script:Utf8NoBom = [System.Text.UTF8Encoding]::new($false)

# Pin UTF-8 for native command pipes BEFORE any git read. Windows PowerShell 5.1
# defaults $OutputEncoding to ASCII, so `git ls-files` / `git show` / `git diff`
# output is decoded lossily and UTF-8 Markdown can mojibake before scanning --
# masking or inventing a finding. auto-review.ps1 / auto-merge.ps1 pin the same
# trio so the documented UTF-8 read contract holds for the default staged scope.
# Set BARE (no try/catch swallow) like those siblings: a host that cannot apply
# the pin must FAIL LOUD via $ErrorActionPreference='Stop' -- swallowing the
# failure and continuing would run the scan under a lossy decoder and risk a
# FALSE-CLEAN lint, the one outcome this linter exists to prevent. (Codex BLOCKER:
# swallowed encoding setup defeated the UTF-8 contract.)
$OutputEncoding = $script:Utf8NoBom
[Console]::InputEncoding = $script:Utf8NoBom
[Console]::OutputEncoding = $script:Utf8NoBom

# Narrative docs this linter governs (full-content scan). Source files legitimately
# contain paths and line references in CODE, so only Markdown is scanned this way.
$script:NarrativeDocPattern = '\.md$'

# Source files whose COMMENT lines carry prose claims (staged mode only): the
# count-drift classes (inventory-assertion SHAPE + MAGNITUDE) run over the extracted
# comment text of added lines, never the code (a numeric literal in code is correct
# where it lives). The comment syntax is resolved per extension (Get-SourceLineParts).
$script:SourceExtPattern = '\.(rs|ps1|toml|sh)$'

# Repo-relative PATHS of KNOWN machine-generated reports whose body is intended
# FORMAT (file:line cites, counts), not authored prose -- the ONLY files eligible
# for the generated-doc skip. This is the security boundary: an authored doc at
# any other path can never self-exempt by typing a `Generated:` stamp (see
# Test-IsGeneratedContent). Currently the sole TRACKED generator output is
# analyze-blocker-trends.ps1 -> docs/blocker-trends.md (its default -OutPath);
# dispatch-checklist's default OutPath is gitignored, and author-lint's own reports are
# conventionally written to gitignored logs/. -OutPath is an UNRESTRICTED string,
# so this PATH gate -- not the destination convention -- is what guarantees a
# report written to ANY tracked path is still scanned unless it is this exact
# leaf. Anchored to the exact leaf so a future docs/codex-<other>.md is not
# blanket-exempt. (Codex BLOCKER: a content-only signature was
# spoofable.)
# CASE-SENSITIVE (no `(?i)`; matched with `-cnotmatch`): this is the spoof
# boundary, so a case-variant authored path like `docs/Blocker-Trends.md`
# must NOT match and must NOT be skipped. (Codex BLOCKER: a
# case-insensitive gate let a wrong-cased authored doc self-exempt.)
# Path matches the analyzer's default -OutPath (docs/blocker-trends.md) and the
# install docs' generate-this-file instruction, so a generated trend report is
# skipped rather than scanned as authored prose and flagged on its machine-
# emitted citations. If you configure a different -OutPath, update this pattern.
$script:GeneratedDocPathPattern = '^docs/blocker-trends\.md$'

# Extensions a bare line-anchor can name -- the TEXT file types cited with a
# `:line` in this repo's docs. Used only inside the LINE-ANCHOR regex. Includes
# text shader/markup formats (wgsl/glsl/xml) that legitimately carry line refs.
# BINARY assets (glb -- the binary glTF container -- png, etc.) are intentionally
# absent: a `:NNN` after a binary name is not a line citation. The list is a
# CALIBRATED subset of text exts actually cited in this repo's docs, not an
# exhaustive enumeration -- other text formats (gltf JSON, html, css) are simply
# not commonly cited here and stay out until they are. (Claude NOTE:
# shader/asset exts; Codex: gltf is text, not binary.)
$script:AnchorExtAlternation = 'rs|ps1|py|toml|ron|sh|md|txt|json|lock|yaml|yml|cfg|ini|bat|cmd|js|ts|cs|wgsl|glsl|xml'

# ---------------------------------------------------------------------------
# Pure helpers (no I/O; SelfTest exercises these directly).
# ---------------------------------------------------------------------------

function Get-ContainedRelPath {
  # Canonicalize a caller-supplied path and enforce that it is INSIDE the repo
  # root, returning the repo-relative path (forward-slash) iff contained, else
  # $null. The caller turns $null into a stable ERROR + nonzero exit. PURE (string
  # + Path math only; no filesystem probe) so SelfTest can drive the
  # outside-repo / `..`-escape rejection cases directly.
  #
  # Containment BEFORE repo-relative derivation is the whole point (BLOCKER):
  # the old `-Paths` code sliced `.Substring($repoRoot.Length)`
  # off a rooted path with NO containment check (an outside abs path became an
  # unrelated in-repo SUFFIX) and joined a relative `..\outside.md` directly under
  # the root (escaping the repo while using repo-index resolution) -- either way a
  # mispointed invocation could exit CLEAN against the WRONG file.
  #   - ROOTED requested path -> GetFullPath as-is.
  #   - RELATIVE requested path -> resolved against the REPO ROOT (preserving the
  #     repo-relative -Paths semantics), then GetFullPath (collapses `..`).
  # Inside iff the canonical path IS the root or starts with `<root><sep>`.
  # Comparison is ORDINAL (case-SENSITIVE) -- git's index is case-sensitive, so a
  # path differing from the root only by case (e.g. `C:\repo\PROJ\...` vs
  # `C:\repo\proj`) is a DIFFERENT location on a case-sensitive checkout and MUST be
  # rejected (BLOCKER: OrdinalIgnoreCase here accepted the case-only
  # outside path, then sliced it against $repoRoot -> a false-clean WRONG-file scan).
  # On a case-insensitive FS the only cost is an over-reject of a case-variant in-repo
  # path, which fails LOUD (exit 2) -- never a silent wrong-file scan.
  param([string]$RequestedPath, [string]$RepoRootFull)
  if ([string]::IsNullOrWhiteSpace($RequestedPath) -or [string]::IsNullOrWhiteSpace($RepoRootFull)) { return $null }
  # Drive-relative Windows paths (`C:foo.md` -- a drive letter + colon NOT followed by a
  # separator) are "rooted" per IsPathRooted() yet GetFullPath() resolves them from the
  # process cwd on that drive, NOT from the repo root -- so a mispointed `-Paths
  # C:docs/foo.md` could pass containment against the WRONG in-repo file. Reject them; a
  # fully qualified `C:\...` (separator after the colon) is unaffected. (Codex BLOCKER.)
  if ($RequestedPath -match '^[A-Za-z]:[^\\/]') { return $null }
  $rootFull = [System.IO.Path]::GetFullPath($RepoRootFull)
  $rootPrefix = $rootFull.TrimEnd('\','/') + [System.IO.Path]::DirectorySeparatorChar
  $candidateFull = $null
  try {
    if ([System.IO.Path]::IsPathRooted($RequestedPath)) {
      $candidateFull = [System.IO.Path]::GetFullPath($RequestedPath)
    } else {
      $candidateFull = [System.IO.Path]::GetFullPath((Join-Path $rootFull $RequestedPath))
    }
  } catch { return $null }
  $isInside = $candidateFull.Equals($rootFull, [System.StringComparison]::Ordinal) -or
              $candidateFull.StartsWith($rootPrefix, [System.StringComparison]::Ordinal)
  if (-not $isInside) { return $null }
  return $candidateFull.Substring($rootFull.Length).TrimStart('\','/') -replace '\\','/'
}

function Test-HasFrozenSnapshotMarker {
  # A doc is a FROZEN snapshot (its line-anchors downgrade to advisory) IFF it
  # carries the EXPLICIT in-file marker -- an HTML comment on its own line, near the
  # top:
  #     <!-- frozen-snapshot -->
  # Detection: a line matches the regex
  #     ^\s*<!--\s*frozen-snapshot\s*-->\s*$
  # (case-insensitive on the token) within the doc's FIRST $HeadLines lines (~30).
  # FILENAME AND DATE NO LONGER MATTER -- the doc declares its own frozen status.
  #
  # Why a positive marker, not a filename guess: inferring "frozen" from a dated
  # filename + a category token (audit/handoff/...) is a WEAK proxy in two
  # directions -- a doc can be dated-in-name yet still actively maintained (a hybrid
  # audit whose sections are current authority), and a fresh install with no
  # configured denylist would exempt ALL dated docs by default. An explicit marker
  # the AUTHOR places is unambiguous: absent -> error-tier (the symbol-anchor rule
  # applies), present -> advisory (the author has declared this a frozen historical
  # record). The marker is an HTML comment so it is INVISIBLE in rendered Markdown.
  #
  # The marker must be a REAL HTML comment in NORMAL Markdown context -- invisible in
  # rendered Markdown. A marker token sitting INSIDE a ``` / ~~~ fenced code block, or
  # on a CommonMark indented-code line (>=4 leading spaces, or 0-3 spaces then a tab), RENDERS
  # as visible code, NOT an invisible comment, so it must NOT grant the exemption:
  # accepting it would let a top-of-file fenced/indented code SAMPLE of the marker
  # falsely demote a doc's LINE-ANCHOR findings to advisory (a false-clean). So the
  # head window is fence-tracked (the SAME Get-FencedLineFlags the anchor/dead-ref
  # checks use), and an indented-code marker line is rejected -- the accept-set is
  # exactly the documented marker shape. (Codex CROSS-CRATE-CONTRACT.)
  #
  # PURE: accepts the doc text via -Content so SelfTest drives it without the
  # filesystem. -Path is accepted but unused for the decision (kept so callers that
  # only have a path can pass it for a future read; the content is authoritative).
  param(
    [string]$Path,
    [string]$Content,
    [int]$HeadLines = 30
  )
  if ([string]::IsNullOrEmpty($Content)) { return $false }
  $head = @($Content -split "`n" | Select-Object -First $HeadLines | ForEach-Object { $_ -replace "`r$", '' })
  $fenced = Get-FencedLineFlags -Lines $head
  for ($i = 0; $i -lt $head.Count; $i++) {
    if ($fenced[$i]) { continue }                                  # inside a code fence -> visible code, not a comment
    if ($head[$i] -match '^(?: {4,}| {0,3}\t)') { continue }       # CommonMark indented code (4+ spaces, OR 0-3 spaces then a tab that expands to col 4) -> visible code, not a comment
    if ($head[$i] -match '^\s*<!--\s*frozen-snapshot\s*-->\s*$') { return $true }
  }
  return $false
}

function Test-IsGeneratedContent {
  # A generated report has a machine-emitted body where file:line and counts are
  # intended FORMAT, not hand-authored prose -- so it is skipped. Two INDEPENDENT
  # gates must BOTH pass, the PATH gate first:
  #   1. PATH (the security boundary): the repo-relative path must match a KNOWN
  #      generated-report location ($script:GeneratedDocPathPattern). An authored
  #      doc at any other path can NEVER self-exempt, no matter what text it
  #      carries -- an author cannot relocate PLAN.md/AGENTS.md/docs/foo.md to the
  #      trends path. This is what closes the spoof.
  #   2. CONTENT (a secondary sanity check): a `Generated:` head line TOGETHER with
  #      a specific generator signature, so a hand-edited / half-written file even
  #      AT the generated path is still scanned rather than skipped.
  # (Codex BLOCKER: the earlier content-only signature was
  # spoofable -- any staged Markdown placing `Generated: ... by author-lint.ps1`
  # in its first 12 lines skipped LINE-ANCHOR/DEAD-REF/MAGNITUDE/TAG validation.)
  param([string]$Path, [string]$Content)
  if ([string]::IsNullOrEmpty($Path)) { return $false }
  $rel = ([string]$Path -replace '\\', '/')
  $rel = $rel -replace '^\./', ''
  if ($rel -cnotmatch $script:GeneratedDocPathPattern) { return $false }
  if ([string]::IsNullOrEmpty($Content)) { return $false }
  $head = ($Content -split "`n" | Select-Object -First 12) -join "`n"
  if ($head -notmatch '(?im)^\s*Generated:\s') { return $false }
  return ($head -match '(?im)\bby author-lint\.ps1\b' -or
          $head -match '(?im)^\s*Source:\s.*\bverdict' -or
          $head -match '(?im)^#\s*Cross-review\s+BLOCKER\s+Trends')
}

function Build-PathSetFromFileList {
  # Given a list of repo file paths (e.g. `git ls-files` -- the INDEX), return a
  # hashtable of every file path PLUS all its ancestor directory prefixes, all
  # normalized to `/`. The DEAD-REF staged resolver keys on this so both a file
  # token (`scripts/codex/x.ps1`) and a directory token (`scripts/codex`)
  # resolve against the pending commit. Pure (no I/O) so SelfTest drives it.
  param([string[]]$Files)
  # ORDINAL (case-sensitive) comparer -- see Get-TopLevelDirSet: git paths are
  # case-sensitive, so a default case-insensitive `@{}` would resolve a
  # wrong-cased reference against a tracked path and miss a real DEAD-REF.
  # (Codex QUALITY.)
  $set = [System.Collections.Hashtable]::new([System.StringComparer]::Ordinal)
  if (-not $Files) { return $set }
  foreach ($raw in $Files) {
    $f = ([string]$raw -replace '\\', '/').Trim()
    if ([string]::IsNullOrWhiteSpace($f)) { continue }
    $set[$f] = $true
    $parts = $f -split '/'
    $acc = ''
    for ($k = 0; $k -lt $parts.Count - 1; $k++) {
      $acc = if ($acc) { "$acc/$($parts[$k])" } else { $parts[$k] }
      if ($acc) { $set[$acc] = $true }
    }
  }
  return $set
}

function Get-TopLevelDirSet {
  # The set of repo top-level DIRECTORY names for the DEAD-REF first-segment
  # gate -- built DETERMINISTICALLY from the index + .gitignore, NOT a worktree
  # listing. A clean clone or review snapshot lacks gitignored roots like
  # `logs/` on disk, so keying the gate on local presence would let the
  # canonical dead/local-only evidence path (`logs/codex/probe-pack-45ff`) pass as
  # prose depending on environment. Tracked dirs come from the index; gitignored
  # evidence/build roots come from .gitignore. Pure (no I/O). (Codex BLOCKER.)
  param([string[]]$IndexFiles, [string]$GitignoreText)
  # CASE-INSENSITIVE on purpose (default `@{}`), in deliberate ASYMMETRY with the
  # ORDINAL index resolver (Build-PathSetFromFileList). This set is only the
  # first-segment INTENT gate in Get-DeadRefFindings: "does this token even look
  # like a repo-relative path?". A wrong-cased top-level segment (`Scripts/...`)
  # must still pass the intent gate so the token PROCEEDS to the ordinal resolver,
  # which then fails to resolve `Scripts/...` against tracked `scripts/...` and
  # flags the DEAD-REF. If THIS set were ordinal, `Scripts` would miss the gate and
  # be dropped as prose -- silently skipping the exact wrong-case class the ordinal
  # resolver exists to catch. (Codex BLOCKER: ordinal-here defeated
  # the case-sensitivity fix.)
  $set = @{}
  if ($IndexFiles) {
    foreach ($f in $IndexFiles) {
      $n = ([string]$f -replace '\\', '/').Trim()
      if ($n -match '^([^/]+)/') { $set[$Matches[1]] = $true }   # first seg of a tracked path
    }
  }
  if ($GitignoreText) {
    foreach ($line in ($GitignoreText -split "`n")) {
      $l = ($line -replace '\\', '/').Trim()
      if (-not $l -or $l.StartsWith('#') -or $l.StartsWith('!')) { continue }
      $l = $l.TrimStart('/')
      if ($l -match '^([^/*?]+)/') { $set[$Matches[1]] = $true }   # dir pattern with a slash (logs/, target/build)
      elseif ($l -match '^([^/*?]+)$' -and $l -notmatch '\.[A-Za-z0-9]+$') { $set[$l] = $true }   # bare root dir, e.g. /target
    }
  }
  return $set
}

function Get-FencedLineFlags {
  # Return a bool[] parallel to $Lines: $true where the line sits INSIDE a
  # ``` or ~~~ fenced code block (the fence delimiter lines themselves count as
  # inside, so an anchor written on the fence line is also skipped). Illustrative
  # citations inside fences are examples, not live references. Both CommonMark
  # fence chars are honored so an anchor inside a ~~~ block is not an error-tier
  # false positive. (Claude NOTE.)
  # The opener CHARACTER and LENGTH are tracked so a block closes only on the SAME
  # fence char with a run AT LEAST as long (CommonMark: a closing fence must match
  # the opener char and be >= its length). So a `~~~` inside a ``` block (different
  # char) and a ``` inside a ```` block (shorter run) are both CONTENT, not a close
  # -- otherwise they would prematurely end the fence and surface later in-block
  # anchors as live findings. (Codex QUALITY: mixed delimiters; then
  # shorter same-char fence.)
  param([string[]]$Lines)
  $flags = New-Object 'bool[]' $Lines.Count
  $inFence = $false
  $fenceChar = ''
  $fenceLen = 0
  for ($i = 0; $i -lt $Lines.Count; $i++) {
    # Capture the fence run AND any trailing content. An OPENING fence may carry an
    # info string (```rust); a CLOSING fence may NOT -- only trailing whitespace
    # (CommonMark). So `~~~text` / ```rust seen INSIDE a block is content, not a
    # close. (Codex QUALITY: trailing-content close.)
    # At most 3 leading SPACES (CommonMark): 4+ spaces or a leading tab is an
    # indented code block, NOT a fence delimiter -- accepting arbitrary `\s*` would
    # let `    ~~~` open a synthetic fence and suppress live error-tier findings.
    # (Codex BLOCKER.)
    $mm = [regex]::Match($Lines[$i], '^ {0,3}(```+|~~~+)(.*)$')
    if ($mm.Success) {
      $run = $mm.Groups[1].Value
      $delim = $run.Substring(0, 1)                   # '`' or '~'
      $runLen = $run.Length
      $bare = ($mm.Groups[2].Value -match '^\s*$')    # only whitespace after the run
      if (-not $inFence) {
        $inFence = $true; $fenceChar = $delim; $fenceLen = $runLen   # open (info string allowed)
        $flags[$i] = $true
      } elseif ($delim -eq $fenceChar -and $runLen -ge $fenceLen -and $bare) {
        $inFence = $false; $fenceChar = ''; $fenceLen = 0            # valid close: same char, >= len, no trailing content
        $flags[$i] = $true
      } else {
        $flags[$i] = $true                            # different char / shorter run / trailing-content -> content
      }
    } else {
      $flags[$i] = $inFence
    }
  }
  return ,$flags
}

function Get-LineAnchorFindings {
  # $Records: array of @{ Line = <int>; Text = <string> }. ERROR-tier by default.
  # A doc carrying the EXPLICIT frozen-snapshot marker ($IsFrozenSnapshot, from
  # Test-HasFrozenSnapshotMarker) DOWNGRADES to advisory -- the symbol-anchor rule
  # leaves frozen snapshot line numbers as-is, so a hard ERROR would reject
  # convention-exempt content; advisory (not skip) keeps it surfaced, never
  # silently suppressed. A doc WITHOUT the marker stays ERROR regardless of its
  # filename or date. (Marker model; supersedes the filename+date+category
  # inference of the prior Test-IsDatedSnapshotDoc.)
  param(
    [object[]]$Records,
    [bool]$IsFrozenSnapshot
  )
  $out = @()
  if (-not $Records) { return $out }
  $tier = if ($IsFrozenSnapshot) { 'ADVISORY' } else { 'ERROR' }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  # Accept BOTH `/` and `\` separators -- this Windows-first repo writes
  # `scripts\codex\foo.ps1:NNN` in tracked docs. (Codex BLOCKER.)
  $rx = [regex]("[A-Za-z0-9_./\\-]+\.(?:$script:AnchorExtAlternation):\d+")
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $text = [string]$Records[$i].Text
    foreach ($m in $rx.Matches($text)) {
      # Skip ONLY a URL-EMBEDDED token, never a path citation. `file.ext:NNN` is a
      # line-anchor violation whether the path is bare (`foo.rs:5`), repo-relative
      # (`scripts/x.ps1:42`), or absolute (`/scripts/x.ps1:42`, `C:\...:5`) -- all
      # are line citations the symbol-anchor rule discourages, so all stay
      # error-tier. The exemption is exactly a token that sits INSIDE a URL --
      # identified by a `scheme://` immediately before the match with no intervening
      # whitespace -- where the `:digits` is part of the URL (a port OR a path-
      # position number, e.g. `:8080` in `https://host/app.js:8080`), never a source
      # line. An earlier broad leading-`/`/`:` guard wrongly skipped real repo-root
      # and label-prefixed anchors. (Codex BLOCKER: /scripts/...:NNN
      # must flag; the URL-embedded exemption is the correct narrow one.)
      if (($text.Substring(0, $m.Index) + $m.Value) -match '[A-Za-z][A-Za-z0-9+.\-]*://\S*$') { continue }
      $out += @{
        Class = 'LINE-ANCHOR'
        Tier  = $tier
        Line  = [int]$Records[$i].Line
        Match = $m.Value
        Hint  = 'cite the grep-able symbol, not the drifting line number'
      }
    }
  }
  return $out
}

function Get-DeadRefFindings {
  # Flag a repo-relative path token whose first segment is a real repo top-level
  # DIRECTORY (per $IsTopLevel) and that does NOT resolve (per $Resolver). Keying
  # on "first segment is a known top-level DIRECTORY AND unresolved" makes the
  # check precise: prose slash-phrases ("client/server", "read/write") and
  # root-FILE slash-lists ("PLAN.md/CLAUDE.md/auth") are not flagged because
  # their first segment is not a top-level directory, and machine-local targets
  # (~/, drive-abs, $env:, <ph>, globs, URLs) are filtered first. Whole class is
  # ADVISORY tier (see header rationale). NOT exempt on dated docs: a dead path
  # there is still surfaced (advisory), since suppressing the class would leave a
  # newly added local-only evidence path false-clean. (Codex BLOCKER.)
  #   $Resolver:   scriptblock { param($relPath) -> [bool] exists in the index/tree }
  #   $IsTopLevel: scriptblock { param($segment) -> [bool] is a repo top-level DIRECTORY }
  param(
    [object[]]$Records,
    [scriptblock]$Resolver,
    [scriptblock]$IsTopLevel
  )
  $out = @()
  if (-not $Records) { return $out }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  # A candidate path token: a run of path-ish chars containing at least one
  # separator (`/` OR `\` -- this Windows-first repo writes both). Punctuation
  # that commonly trails a path in prose (.,;:)`'"]) is trimmed after capture.
  # The START class excludes `_` (kept mid-token for names like check_registry.py)
  # so Markdown UNDERSCORE emphasis -- `_docs/x.md_` / `__docs/x.md__` -- does not
  # tokenize the wrapper into the first segment (`_docs`) and silently drop a real
  # dead path; the match then starts at `docs` and is flagged. (Codex SILENT-FAILURE:
  # `\`-separated refs were invisible; underscore-wrapped dropped.)
  $rx = [regex]('[A-Za-z0-9.$~<][A-Za-z0-9_./\\-]*[/\\][A-Za-z0-9_./\\-]+')
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $lineText = [string]$Records[$i].Text
    foreach ($m in $rx.Matches($lineText)) {
      # Suffix of a larger absolute / drive / URL path: the regex cannot capture
      # a `:` (drive), a leading `/`, or a URL scheme, so those leave the repo-
      # looking tail starting right after a separator. If the char before the
      # match is a separator or colon, the token is not a standalone repo path.
      # (Codex QUALITY.)
      # Walk back over any leading `_` emphasis chars to the first NON-underscore
      # char before the match. The token start-class excludes `_`, so an absolute /
      # drive / URL path whose segment begins with `_` (`/_docs/x`, `C:\_docs\x`,
      # `https://h/_docs/x`) starts the match AFTER the `_`, leaving `_` as the
      # immediate prev char -- the effective prev char (past the `_` run) is the real
      # separator/colon to test. (Codex QUALITY: separator+underscore
      # bypassed the absolute guard.)
      $j = $m.Index - 1
      while ($j -ge 0 -and $lineText[$j] -eq '_') { $j-- }
      $effPrev = if ($j -ge 0) { $lineText[$j] } else { '' }
      if ($effPrev -eq '/' -or $effPrev -eq '\' -or $effPrev -eq ':') { continue }
      $tok = $m.Value -replace '[.,;:`''")\]\}>]+$', ''
      # Strip a leading angle wrapper BEFORE the dot-relative strip (order matters):
      # `<./logs/...>` must become `logs/...`, so the `<` is removed first, then the
      # `./`. The trailing `>` is already trimmed above. The firstSeg-is-top-level
      # gate then separates a REAL bracketed repo path (`<logs/codex/probe-pack>` ->
      # firstSeg `logs` -> flagged) from a generic `<path/to/file>` placeholder
      # (firstSeg `path` -> prose). A blanket `^<` exemption wrongly suppressed real
      # bracketed dead refs. (Codex BLOCKER; ordering fix same round.)
      $tok = $tok -replace '^<+', ''         # strip leading angle wrapper(s) FIRST
      $tok = $tok -replace '^\.[\\/]+', ''   # then strip a leading ./ or .\ (dot-relative)
      # Markdown UNDERSCORE emphasis close: strip the trailing `_`/`__` ONLY for a
      # genuine balanced wrap -- there was a leading `_` run AND its predecessor is
      # start-of-line or a NON-WORD char (Markdown emphasis needs a left flank). A
      # word-char predecessor (`prefix_docs/...`) is an intra-identifier underscore,
      # not emphasis, so its trailing `_` is left alone (and the `/\:` absolute guard
      # above is separate). This lets `_scripts/x.ps1_` resolve while leaving a real
      # filename underscore intact. (Codex QUALITY.)
      $hadLeadUnderscore = ($j -lt ($m.Index - 1))
      $emphasisLeftFlank = ($j -lt 0 -or $effPrev -notmatch '[A-Za-z0-9]')
      if ($hadLeadUnderscore -and $emphasisLeftFlank) { $tok = $tok -replace '_+$', '' }
      if ([string]::IsNullOrWhiteSpace($tok)) { continue }
      # Glob base vs Markdown punctuation: a wildcard immediately follows the
      # captured token (the wildcard is not in the path char class, so it falls
      # just past the match -- e.g. `logs/codex/reviews/*.md` captures
      # `logs/codex/reviews`). EXEMPT a real glob: a SINGLE `*` (`dir/*`, `*.md`,
      # `prefix-*`) or a `?` that opens a glob single-char (`name?.txt`). Do NOT
      # exempt doubled `**` (Markdown bold close, e.g. `**docs/x.md**`) or a bare
      # trailing `?` (rhetorical punctuation, e.g. `docs/x.md?`) -- those wrap a
      # REAL path that must still be dead-ref checked. (Codex SILENT-FAILURE:
      # the prior `[/\\]?[\*\?]` swallowed Markdown emphasis/punctuation.)
      $endIdx = $m.Index + $m.Length
      $afterCtx = ''
      if ($endIdx -lt $lineText.Length) { $afterCtx = $lineText.Substring($endIdx, [Math]::Min(3, $lineText.Length - $endIdx)) }
      # Glob vs Markdown. The token regex's final `[mid]+` INCLUDES separators and is
      # greedy, so it ALWAYS consumes the separator preceding a wildcard: a globstar
      # `docs/art/**` captures `docs/art/` (trailing sep) + afterCtx `**`, while a
      # Markdown bold close `**docs/x.md**` captures `docs/x.md` (no sep) + afterCtx
      # `**`. afterCtx therefore always BEGINS with the wildcard, never `/<wildcard>`
      # (so no `[/\\]?` alternative is needed), and the captured token's trailing
      # separator is the signal that a following `**` is a real globstar segment, not
      # emphasis. (Codex/Claude: distinguish recursive ** globs from
      # Markdown bold/italic/punctuation; dropped the unreachable `/<wildcard>` arm.)
      $rawEndsSep = ($m.Value -match '[/\\]$')
      $prevChar = if ($m.Index -gt 0) { $lineText[$m.Index - 1] } else { '' }
      if ($afterCtx -match '^\*\*') { if ($rawEndsSep) { continue } }   # `dir/` + `**` globstar (sep eaten); bare ** is Markdown -> flag
      elseif ($afterCtx -match '^\*') {                                # single `*`
        # Markdown ITALIC close (`*path*`) when the match opens with a leading `*`;
        # otherwise a glob (`dir/*`, `*.md`, `prefix-*`). (Codex SILENT-FAILURE:
        # single-* emphasis around a dead path must still flag.)
        if ($prevChar -ne '*') { continue }
      }
      elseif ($afterCtx -match '^\?[\w.]') { continue }                # ?-glob single char (name?.txt), not a bare trailing ?
      # Machine-local / non-repo-relative starts the token char-class CAN produce
      # (the start class `[...$~<]` admits ~ / $ / <). Drive-absolute (`C:/`),
      # posix/UNC-absolute (`/`, `\`), URL (`scheme://`), and inline globs (`*`/`?`)
      # need NO guard here: the token regex char-class excludes `:`, a leading
      # `/`/`\`, and `*`/`?`, so such forms are already handled upstream -- the
      # prev-char `:`/`/`/`\` guard (above) drops absolute/drive/URL tails, and the
      # afterCtx wildcard check (above) drops glob bases. Guards for those were dead
      # branches over structurally-impossible tokens (DR5/DR6/DR14 pass via the
      # upstream guards) and were removed. (Claude QUALITY.)
      if ($tok -match '^~') { continue }                 # home-relative
      if ($tok -match '^\$') { continue }                 # $env:/$var
      # (A leading `<` is STRIPPED above, not exempted here, so a real `<repo/path>`
      # reaches the firstSeg/resolver check; a `<placeholder/...>` falls out via the
      # firstSeg-not-a-repo-dir gate.)
      # Normalize separators for the repo lookups; report the original token.
      $normTok = $tok -replace '\\', '/'
      $firstSeg = ($normTok -split '/')[0]
      if (-not (& $IsTopLevel $firstSeg)) { continue }    # not a repo dir -> prose
      if (& $Resolver $normTok) { continue }              # resolves -> fine
      $out += @{
        Class = 'DEAD-REF'
        Tier  = 'ADVISORY'
        Line  = [int]$Records[$i].Line
        Match = $tok
        Hint  = 'repo-relative path absent from the tracked index (typo or dead/local-only reference a fresh clone cannot resolve)'
      }
    }
  }
  return $out
}

function Get-DefinedMilestoneTags {
  # Milestone-defining headers look like `### M1: ...` / `#### M88.X3a: ...` /
  # `#### M1 Acceptance Criteria`. Return the set of top-level milestone tokens
  # (M<n>) that have at least one defining header.
  param([string]$PlanContent)
  $defined = @{}
  foreach ($line in ($PlanContent -split "`n")) {
    if ($line -match '^#{2,4}\s+(M\d+)(?:[.:\s]|$)') {
      $defined[$Matches[1]] = $true
    }
  }
  return $defined
}

function Get-MilestoneTagFindings {
  # ADVISORY. Flag a referenced milestone tag whose TOP-LEVEL milestone has no
  # defining header anywhere in PLAN.md (a wholly-unknown milestone -- usually
  # a renamed/deleted tag). Conservative by design: finer suffix closure is a
  # documented future extension.
  param([string]$PlanContent)
  $out = @()
  if ([string]::IsNullOrEmpty($PlanContent)) { return $out }
  $defined = Get-DefinedMilestoneTags -PlanContent $PlanContent
  $lines = $PlanContent -split "`n"
  $fenced = Get-FencedLineFlags -Lines $lines
  $rx = [regex]('\bM\d+(?:\.[A-Za-z0-9]+)*\b')
  $seen = @{}
  for ($i = 0; $i -lt $lines.Count; $i++) {
    if ($fenced[$i]) { continue }
    foreach ($m in $rx.Matches($lines[$i])) {
      $tag = $m.Value
      $top = ([regex]::Match($tag, '^M\d+')).Value
      if ($defined.ContainsKey($top)) { continue }
      $key = "$top"
      if ($seen.ContainsKey($key)) { continue }   # report each unknown milestone once
      $seen[$key] = $true
      $out += @{
        Class = 'TAG'
        Tier  = 'ADVISORY'
        Line  = ($i + 1)
        Match = $tag
        Hint  = "milestone $top has no defining header in PLAN.md (renamed/deleted?)"
      }
    }
  }
  return $out
}

function Get-MagnitudeFindings {
  # ADVISORY. Exact transient quantities in narrative prose where a magnitude
  # statement suffices. The unit forms ($rxUnit / $rxUnitFirst) require a
  # process-event unit word adjacent to the number, so stable lookup handles
  # (file:line / dates / SHAs / task ids / versions) carry no such unit and do not
  # match. The verb form ($rxTook) is deliberately UNIT-AGNOSTIC -- it anchors on
  # "took N" instead, so it also catches "took 6 minutes"; the leading `\btook\s`
  # is what keeps it off bare numbers. Lookbehind guards per form: $rxUnit uses
  # `(?<![\w.-])` (digit-start guard incl. the dot, so dotted handles/decimal
  # fractions never shed a prefix -- see the detailed note below); $rxUnitFirst
  # uses `(?<![\w-])` (it guards the unit WORD); $rxTook needs none (the literal
  # `took ` anchor is the guard).
  # This detector is doc-agnostic; the History.md count-drift exemption (a dated
  # entry is a static snapshot whose counts cannot drift) is CALLER-applied by
  # Invoke-FileChecks, which DROPS findings on lines inside a dated entry body of the
  # ROOT History.md log. Every other line and doc is scanned normally.
  param([object[]]$Records)
  $out = @()
  if (-not $Records) { return $out }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  # The count class has these surface forms, all matched so the documented
  # round/iteration/attempt/pass/retry/try coverage actually holds:
  #   digit-first  "6 rounds"   -- $rxUnit
  #   unit-first   "round 6"    -- $rxUnitFirst
  #   verb form    "took 6"     -- $rxTook
  # The digit-first lookbehind (?<![\w.-]) rejects a MATCH STARTING mid-token:
  # inside an identifier/version/dotted handle (task-0004, v2, M19-9, M19.9's
  # trailing 9) or at a decimal's fraction part. A FREE-STANDING decimal is
  # matched IN FULL from its first digit via the optional (\.\d+)? --
  # "0.25 seconds" is one finding ("0.25 seconds", still an exact transient
  # duration), never the fragment "25 seconds"; "M19.9 tasks" matches nothing
  # (the 19 is word-preceded, the 9 dot-preceded). The unit-first form keeps
  # the narrower (?<![\w-]) (its lookbehind guards the unit WORD, not a digit).
  #
  # The DIGIT-FIRST alternation is WIDER than the unit-first one. It adds the
  # unit-agnostic aggregate/duration nouns that leaked past the original
  # process-event list ("18-task chain", "2 rows", "47 assertions",
  # "6 minutes" -- the extension the older docs called "documented, not yet
  # caught"). These nouns are deliberately NOT in $rxUnitFirst: the unit-first
  # shape for them is usually a stable NAME, not a count ("Wave 2", "day 3",
  # "row 7" as a table coordinate), and MAGNITUDE must stay off stable
  # handles. Digit-first "2 waves"/"3 days" is a count; unit-first "Wave 2"
  # is a name.
  $rxUnit = [regex]('(?i)(?<![\w.-])\d+(\.\d+)?\s*-?\s*(?:rounds?|iterations?|attempts?|pass(?:es)?|retr(?:y|ies)|tr(?:y|ies)|tasks?|commits?|files?|rows?|sites?|entr(?:y|ies)|follow-ups?|findings?|fixtures?|assertions?|instances?|waves?|streams?|minutes?|hours?|seconds?|days?)\b')
  # Unit-first: the unit word then a free-standing count ("round 1", "attempt 3").
  # `(?<![\w-])` before the unit keeps "subround"/"bypass 2" from matching; the
  # trailing `\b` ends the digit run. Kept to the PROCESS-EVENT nouns only -- see
  # the digit-first note above for why the aggregate/duration nouns are excluded
  # here. (Codex DOC-VS-CODE-DRIFT.)
  $rxUnitFirst = [regex]('(?i)(?<![\w-])(?:rounds?|iterations?|attempts?|pass(?:es)?|retr(?:y|ies)|tr(?:y|ies))\s+\d+\b')
  # The optional (\.\d+)? consumes a decimal in full so a UNIT-LESS "took 0.25"
  # is matched as "took 0.25" (still an exact transient measurement, still
  # advisory) rather than as the fragment "took 0". A unit-BEARING decimal like
  # "took 0.25 seconds" is instead matched by $rxUnit as "0.25 seconds": the loop
  # below records $rxUnit spans first and suppresses the overlapping $rxTook hit,
  # so only a unit-less took-decimal actually exercises this (\.\d+)? path (MAG16).
  $rxTook = [regex]('(?i)\btook\s+\d+(\.\d+)?\b')
  # ENUMERATED-CASE COUNTING -- the SPELLED-OUT sibling of the digit-first count
  # forms, in the same MAGNITUDE family (Class 'MAGNITUDE', ADVISORY), added
  # because "all three cases" restates an exact total a magnitude phrasing ("all
  # cases" / "every case") carries just as the digit forms do. The digit-first
  # $rxUnit needs a DIGIT, so a spelled total ("all three cases", "both cases")
  # slips past it -- $rxAllCases closes exactly that gap. The surface forms:
  #   (a) $rxAllCases: "all <spelled-N> <enumeration-noun>" / "both <noun>".
  #   (b) the paren-enumeration pairing: a line with >=2 inline `(N)` markers
  #       ($rxEnumParen) TOGETHER WITH an "all <N>" total ($rxAllTotal), i.e.
  #       the "(1) ... (2) ... (3) ... all three" enumerate-then-total shape,
  #       caught even when the total carries no enumeration noun. Restricted to
  #       lines where form (a) did NOT already fire, so a "(1)(2) ... all three
  #       cases" line reports once, not twice.
  # Stable-handle safety is inherited: neither pattern matches a bare digit run
  # (so SHAs / task-ids / dates are untouched), and the unit-first NAME shape
  # ("Wave 2", "day 3") never begins with "all"/"both", so it cannot false-match.
  $rxAllCases = [regex]('(?i)\b(?:all\s+(?:two|three|four|five|six|seven|eight|nine|ten)|both)\s+(?:cases?|instances?|forms?|variants?|scenarios?|places?|sites?|call-?sites?|classes|categories|paths?|branches|reasons?|points?|ways?)\b')
  $rxEnumParen = [regex]('\(\d+\)')
  $rxAllTotal = [regex]('(?i)\ball\s+(?:\d+|two|three|four|five|six|seven|eight|nine|ten)\b')
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $text = [string]$Records[$i].Text
    $hits = @()
    # Track the CHARACTER SPANS of the unit-form matches. A `took N` match is
    # suppressed only when its span OVERLAPS a unit match -- so "took 6 rounds"
    # (the "took 6" and "6 rounds" spans share the "6") yields ONE finding, while
    # the independent "6 rounds ... took 6" (disjoint spans, same number) keeps
    # BOTH. Number-based dedup wrongly dropped the latter's verb-form count.
    # (Codex QUALITY; supersedes the earlier same-number dedup.)
    $unitSpans = @()
    foreach ($m in $rxUnit.Matches($text)) {
      $hits += $m.Value
      $unitSpans += @{ Start = $m.Index; End = ($m.Index + $m.Length) }
    }
    foreach ($m in $rxUnitFirst.Matches($text)) {
      $hits += $m.Value
      $unitSpans += @{ Start = $m.Index; End = ($m.Index + $m.Length) }
    }
    foreach ($m in $rxTook.Matches($text)) {
      $tStart = $m.Index
      $tEnd = $m.Index + $m.Length
      $overlaps = $false
      foreach ($s in $unitSpans) {
        if ($tStart -lt $s.End -and $s.Start -lt $tEnd) { $overlaps = $true; break }
      }
      if ($overlaps) { continue }
      $hits += $m.Value
    }
    foreach ($h in ($hits | Select-Object -Unique)) {
      $out += @{
        Class = 'MAGNITUDE'
        Tier  = 'ADVISORY'
        Line  = [int]$Records[$i].Line
        Match = $h.Trim()
        Hint  = 'prefer magnitude over an exact transient count ("many rounds")'
      }
    }
    # ENUMERATED-CASE totals (form (a) then, only if (a) was silent, form (b) --
    # see the regex block above). Distinct Hint from the transient-count forms
    # so the report tells the author which magnitude rewrite applies.
    $enumHits = @()
    foreach ($m in $rxAllCases.Matches($text)) { $enumHits += $m.Value }
    if ($enumHits.Count -eq 0 -and ($rxEnumParen.Matches($text).Count -ge 2) -and $rxAllTotal.IsMatch($text)) {
      foreach ($m in $rxAllTotal.Matches($text)) { $enumHits += $m.Value }
    }
    foreach ($h in ($enumHits | Select-Object -Unique)) {
      $out += @{
        Class = 'MAGNITUDE'
        Tier  = 'ADVISORY'
        Line  = [int]$Records[$i].Line
        Match = $h.Trim()
        Hint  = 'prefer "all cases"/"every case" over an exact enumerated total'
      }
    }
  }
  return $out
}

function Get-SelfNarrationFindings {
  # Narration of the PENDING change's OWN landing -- the deictic "this/here" +
  # a landing-event noun (merge/landing/entry) form. ERROR when the doc is
  # History.md ($IsHistoryDoc), ADVISORY in any other narrative doc: History
  # entries are retrospective records of what ALREADY landed, so an entry that
  # narrates its own not-yet-landed merge is a factual defect (the merge SHA is
  # not knowable while the entry rides the branch through the gate); elsewhere
  # the same phrasing is a softer smell, so it only advises. History.md is the
  # package's illustrative landing-log convention (see README); a consumer with
  # no such doc simply never trips the ERROR tier.
  #
  # The signature is the DEICTIC pointer at the current change, NOT the bare
  # word "merge"/"landed" -- legitimate RETROSPECTIVE prose about an
  # already-landed merge ("the merge landed at <sha>", "the batch merged
  # cleanly") carries no "this/here" pointer and must stay clean. The phrase set
  # was CALIBRATED so the ERROR tier fires only on genuine self-narration:
  #   - "this merge" / "this landing"  -- deictic + landing EVENT (covers
  #     "after this merge", "immediately after this merge", "this landing's").
  #     These are the highest-precision forms.
  #   - "as of this entry" / "add(s|ing) this entry" -- the documented
  #     entry-deixis forms (a state-as-of-the-entry claim; a commit narrating
  #     that it adds the entry). Deliberately NARROW: a bare "this entry" is NOT
  #     matched, because legitimate timestamping ("at the time of this entry")
  #     uses it and would false-positive at ERROR tier.
  #   - "landed ... here" -- the deictic "here" (this doc/point) within a short
  #     word window of "landed" (covers "landed here" and "landed through the
  #     gate here"). Word-bounded window (<=6 words) so it cannot cross a clause.
  # Lower-precision self-narration (a bare "this entry rides ...", "recordable
  # here") is left to the AI gate on purpose -- under-flagging a borderline form
  # beats an ERROR false positive that blocks a legitimate commit (the same
  # calibration doctrine the LINE-ANCHOR tier note records).
  param(
    [object[]]$Records,
    [bool]$IsHistoryDoc
  )
  $out = @()
  if (-not $Records) { return $out }
  $tier = if ($IsHistoryDoc) { 'ERROR' } else { 'ADVISORY' }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  $patterns = @(
    [regex]('(?i)\bthis\s+merge\b'),
    [regex]('(?i)\bthis\s+landing\b'),
    [regex]('(?i)\bas\s+of\s+this\s+entry\b'),
    [regex]('(?i)\b(?:add|adds|adding)\s+this\s+entry\b'),
    # "landed" then, within up to 6 intervening words, a deictic "here".
    [regex]("(?i)\blanded\b(?:\s+[\w'-]+){0,6}?\s+here\b")
  )
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $text = [string]$Records[$i].Text
    $hits = @()
    foreach ($rx in $patterns) {
      foreach ($m in $rx.Matches($text)) { $hits += $m.Value }
    }
    foreach ($h in ($hits | Select-Object -Unique)) {
      $out += @{
        Class = 'SELF-NARR'
        Tier  = $tier
        Line  = [int]$Records[$i].Line
        Match = ($h -replace '\s+', ' ').Trim()
        Hint  = 'do not narrate the pending change''s own landing; state already-landed facts (a merge SHA is unknowable while the entry rides the branch)'
      }
    }
  }
  return $out
}

function Get-LocalEvidenceFindings {
  # A proof/verification CLAIM co-located with a LOCAL-ONLY evidence reference --
  # citing something a fresh clone cannot resolve as if it were shareable proof.
  # Tiers set by CALIBRATION against the current doc tree (the header doctrine:
  # ERROR only where the signature fires ZERO false positives):
  #   ERROR (tight): a proof word (prove/proves/proved/proven/proving | verify/
  #     verifies/verified/verifying/verification/verifiable) on the same line as the gitignored review-artifact
  #     PATH `logs/(codex|claude)/reviews` (either `/` or `\` separator), OR a
  #     `proof`/`evidence` NOUN within $NounWindow chars of that path (proximity so
  #     "Proof: <path>" flags but a distant "evidence snapshot ... <path>" does not).
  #     That path is THE local evidence store;
  #     a proof claim next to it is the exact anti-pattern, and the full-tree
  #     calibration found the pairing does not arise in legitimate prose -- so it is high-precision ERROR.
  #   ADVISORY (adjective): a proof word with one of the bare markers
  #     gitignored | local-only | untracked. These are demonstrably lower
  #     precision -- the current tree co-locates them with proof words in
  #     ordinary git prose ("verify ... --untracked-files", cleanup notes) and in
  #     META-DISCUSSION of this very rule ("gitignored logs ... cannot prove
  #     provenance"), which are NOT the defect. So the class is surfaced, not
  #     blocked, and the author judges (mirrors the DEAD-REF advisory-by-
  #     calibration outcome).
  #   ADVISORY (loose): "operator-side"/"local" together with "evidence" -- the
  #     softest form, always advisory.
  # Word-boundaried proof words so "approve"/"approved" (which merely CONTAIN
  # "prove") never match. One finding per line: ERROR outranks the adjective
  # advisory, which outranks the loose advisory.
  param([object[]]$Records)
  $out = @()
  if (-not $Records) { return $out }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  # Word-bounded proof/verification words INCLUDING the common inflections
  # (proved/proven past forms, verifying/verifiable) so `verifying via
  # logs/.../reviews` is not a false-clean. (Codex BLOCKER: the original set
  # omitted verifying/proven.)
  $rxProof    = [regex]('(?i)\b(?:prove|proves|proved|proven|proving|verify|verifies|verified|verifying|verification|verifiable)\b')
  # Accept BOTH separators -- this Windows-first repo writes `logs\codex\reviews`
  # as readily as `logs/codex/reviews` (LINE-ANCHOR/DEAD-REF honor both too).
  # (Codex BLOCKER: a `\`-separated review path bypassed the gate.)
  $rxPathMark = [regex]('logs[\\/](?:codex|claude)[\\/]reviews')
  $rxAdjMark  = [regex]('(?i)\b(?:gitignored|local-only|untracked)\b')
  $rxLooseQ   = [regex]('(?i)(?:\boperator-side\b|\blocal\b)')
  $rxEvidence = [regex]('(?i)\bevidence\b')
  # NOUN forms of the claim (proof/evidence). Caught in the tight ERROR only when
  # WITHIN $NounWindow chars of the review path -- a proximity gate so `Proof:
  # logs/.../reviews` (a citation) flags while `builds the evidence snapshot ...
  # logs/.../reviews` (the noun far from the path, a description) does not. (Codex
  # BLOCKER: noun-form path claims like "Proof: <path>".)
  $rxProofNoun = [regex]('(?i)\b(?:proof|evidence)\b')
  $NounWindow = 30
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $text = [string]$Records[$i].Text
    $proof = $rxProof.Match($text)
    $pathM = $rxPathMark.Match($text)
    # TIGHT ERROR: a verb proof-word ANYWHERE on the line with the review path, OR a
    # proof/evidence NOUN within $NounWindow chars of the path.
    $tightMatch = $null
    if ($proof.Success -and $pathM.Success) {
      $tightMatch = "$($proof.Value) + $($pathM.Value)"
    } elseif ($pathM.Success) {
      foreach ($pm in $rxPathMark.Matches($text)) {
        foreach ($nm in $rxProofNoun.Matches($text)) {
          $gap = [Math]::Max($nm.Index - ($pm.Index + $pm.Length), $pm.Index - ($nm.Index + $nm.Length))
          if ($gap -le $NounWindow) { $tightMatch = "$($nm.Value) + $($pm.Value)"; break }
        }
        if ($tightMatch) { break }
      }
    }
    if ($tightMatch) {
      $out += @{
        Class = 'LOCAL-PROOF'
        Tier  = 'ERROR'
        Line  = [int]$Records[$i].Line
        Match = $tightMatch
        Hint  = 'do not cite a gitignored review-artifact path as proof; a fresh clone cannot resolve it'
      }
      continue
    }
    if ($proof.Success) {
      $adjM = $rxAdjMark.Match($text)
      if ($adjM.Success) {
        $out += @{
          Class = 'LOCAL-PROOF'
          Tier  = 'ADVISORY'
          Line  = [int]$Records[$i].Line
          Match = "$($proof.Value) + $($adjM.Value)"
          Hint  = 'a proof claim beside a local-only marker -- confirm it is not offered as clone-resolvable evidence'
        }
        continue
      }
    }
    $q = $rxLooseQ.Match($text)
    if ($q.Success -and $rxEvidence.IsMatch($text)) {
      $out += @{
        Class = 'LOCAL-PROOF'
        Tier  = 'ADVISORY'
        Line  = [int]$Records[$i].Line
        Match = "$($q.Value) + evidence"
        Hint  = 'operator-side/local evidence is not tracked-tree evidence; state it as such or drop the claim'
      }
    }
  }
  return $out
}

function Get-MachineLocalStateFindings {
  # ADVISORY. A machine-local install/wiring state asserted as repo fact: an
  # "installed and wired" / "is wired via" / "wired via" claim on the same line
  # as a MACHINE-LOCAL config path (~/.claude ... or .claude/settings.local.json;
  # either `/` or `\` separator, Windows-first)
  # -- state that lives on one operator's machine, not in the tracked tree, so a
  # fresh clone cannot confirm it. EXEMPT when the line already flags the state
  # as machine-local (a "machine-local" / "local-only" qualifier present), which
  # is exactly how the existing hook docs correctly caveat it -- so the detector
  # surfaces only UN-caveated assertions. Advisory only: many legitimate
  # operational instructions name these paths, so the author confirms the caveat.
  param([object[]]$Records)
  $out = @()
  if (-not $Records) { return $out }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  $rxWired  = [regex]('(?i)\b(?:installed(?:\s+and\s+wired)?|is\s+wired(?:\s+via)?|wired\s+(?:via|through|in|into)|installed\s+via)\b')
  # Both separators (Windows-first): `~\.claude` / `.claude\settings.local.json`
  # are as common as the `/` forms. (Codex BLOCKER.)
  $rxMLPath = [regex]('(?i)(?:~[\\/]\.claude|\.claude[\\/]settings\.local\.json)')
  $rxCaveat = [regex]('(?i)\b(?:machine-local|machine\slocal|local-only)\b')
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $text = [string]$Records[$i].Text
    if ($rxCaveat.IsMatch($text)) { continue }
    $w = $rxWired.Match($text)
    $p = $rxMLPath.Match($text)
    if ($w.Success -and $p.Success) {
      $out += @{
        Class = 'LOCAL-STATE'
        Tier  = 'ADVISORY'
        Line  = [int]$Records[$i].Line
        Match = "$($w.Value) + $($p.Value)"
        Hint  = 'machine-local wiring asserted as repo fact; mark it machine-local or verify it in the tracked tree'
      }
    }
  }
  return $out
}

function Get-InventoryAssertionFindings {
  # An EXACT INVENTORY/COUNT of repo/project state asserted in prose -- the
  # durable, NOUN-AGNOSTIC signature of the drift-prone count (the count is wrong
  # the instant the inventory changes, and no reader can trust it).
  # High-precision SHAPE families gate on the surrounding VERB, not a noun list,
  # so an unforeseen noun ("223 icons", "7 configs", "96 widgets") is still caught:
  #   1. "there is|are  <N>  <word>"
  #   2. "<N> <word>  is|are  (defined|tracked|supported|registered|covered|available|in place)"
  #   3. "(covers|contains|includes|totals?)  <N>  <word>"
  # ERROR-tier, always (calibrated: the full-tree .md run surfaces only
  # genuine inventory claims -- true-positive debt, like LINE-ANCHOR/SELF-NARR --
  # not legitimate prose, because the digit lookbehind excludes
  # identifier/version/date/SHA digits). NO frozen-snapshot downgrade: the
  # marker exemption is scoped to LINE ANCHORS (the symbol-anchor rule), so an
  # exact count stays ERROR even in a frozen-marked doc and a staged .md
  # inventory claim can never bypass the error-tier pre-pass. (Codex BLOCKER:
  # the prior ADVISORY downgrade contradicted the exact-count contract.)
  # EXEMPTIONS:
  #   - the stable-handle lookbehind (?<![\w.-]) on the digit: an identifier /
  #     version / date / SHA / task-id digit ("M19.9", "item-0003", "v2", "P1") is
  #     never read as a count. Exit codes / tier / protocol values naturally fall
  #     out: none carry a shape VERB ("exit 3" has no "there are"/"covers"/"are
  #     defined"), so the shape never matches them.
  #   - INLINE CODE: a FULL illustrative example -- the WHOLE claim inside one
  #     backtick span (`there are 223 icons`) -- is exempt so a doc can name the
  #     class; but a claim whose NUMBER ALONE is backticked (there are `223`
  #     icons) is a real prose claim and still flags (matching runs on the text
  #     with backtick delimiters removed, exempting only a match wholly inside a
  #     single span). Fenced blocks are skipped as elsewhere.
  # (The source-comment code-echo exemption is applied by the caller
  # Get-StagedSourceFindings, which alone knows the nearby code lines. The
  # History.md count-drift exemption is likewise CALLER-applied by
  # Invoke-FileChecks -- which alone knows the doc identity + line layout -- so
  # this detector stays doc-agnostic and still fires when called directly on
  # History content; the caller then DROPS findings on lines inside a dated entry
  # body of the ROOT History.md log, a static snapshot whose counts cannot drift.)
  param(
    [object[]]$Records
  )
  $out = @()
  if (-not $Records) { return $out }
  $tier = 'ERROR'
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  $rxThereAre = [regex]('(?i)\bthere\s+(?:is|are)\s+(?<![\w.-])(\d+)\s+[A-Za-z][A-Za-z-]*')
  $rxAreState = [regex]('(?i)(?<![\w.-])(\d+)\s+[A-Za-z][A-Za-z-]*\s+(?:are|is)\s+(?:defined|tracked|supported|registered|covered|available|in\s+place)\b')
  $rxCovers   = [regex]('(?i)\b(?:covers|contains|includes|totals?)\s+(?<![\w.-])(\d+)\s+[A-Za-z][A-Za-z-]*')
  $patterns = @($rxThereAre, $rxAreState, $rxCovers)
  for ($i = 0; $i -lt $Records.Count; $i++) {
    if ($fenced[$i]) { continue }
    $text = [string]$Records[$i].Text
    # Inline-code handling. A backtick `...` span that wraps a WHOLE illustrative
    # claim (`there are 223 icons`) is exempt so a doc can NAME the class; but a
    # claim whose NUMBER ALONE is backticked (there are `223` icons) is a real
    # prose claim and MUST still flag. (Codex BLOCKER: blanking every span dropped
    # the number-only-backticked form's only digit and false-cleaned a real
    # claim.) So match on the text with backtick DELIMITERS removed (adjacency
    # restored across an inline `N`), then EXEMPT a match ONLY when its ENTIRE
    # original span lies inside a SINGLE inline-code span. $map carries each
    # flat-string index back to its original offset for that test.
    $spans = @()
    foreach ($cm in ([regex]::Matches($text, '`[^`]*`'))) { $spans += @{ Start = $cm.Index; End = ($cm.Index + $cm.Length) } }
    $flatSb = New-Object System.Text.StringBuilder
    $map = New-Object 'System.Collections.Generic.List[int]'
    for ($k = 0; $k -lt $text.Length; $k++) {
      if ($text[$k] -eq '`') { continue }
      [void]$flatSb.Append($text[$k]); $map.Add($k)
    }
    $flat = $flatSb.ToString()
    $seen = @{}
    foreach ($rx in $patterns) {
      foreach ($m in $rx.Matches($flat)) {
        # Map the flat match back to original offsets; exempt only a FULL example.
        $origStart = $map[$m.Index]
        $origEnd = $map[$m.Index + $m.Length - 1]
        $insideOneSpan = $false
        foreach ($sp in $spans) { if ($origStart -ge $sp.Start -and $origEnd -lt $sp.End) { $insideOneSpan = $true; break } }
        if ($insideOneSpan) { continue }
        $val = ($m.Value -replace '\s+', ' ').Trim()
        if ($seen.ContainsKey($val)) { continue }
        $seen[$val] = $true
        $out += @{
          Class = 'INVENTORY'
          Tier  = $tier
          Line  = [int]$Records[$i].Line
          Match = $val
          Hint  = 'exact inventory count in prose drifts; use magnitude phrasing or delete if the count is the only content (do not "correct" the number)'
        }
      }
    }
  }
  return $out
}

function Get-SourceLineParts {
  # Split ONE source line into its COMMENT text and its CODE text, given the
  # incoming block-comment state. Comment syntax per $Ext (rs: `//` line + `/* */`
  # block; ps1: `#` line + `<# #>` block; sh/toml: `#` line, no block). String
  # literals are honored so a `//`/`#` INSIDE a string ("http://..." , a toml
  # value with `#`) counts as code, not a comment. Backslash escapes are honored
  # for rust strings and sh/toml DOUBLE-quoted strings (a `\"` does NOT close the
  # string), plus ps1 backtick escapes; single-quoted sh/toml strings take no
  # escapes. The scanner handles the common cases; genuinely rare edge forms (rust
  # char-literal `'"'`, raw/here-strings) may mis-split --
  # acceptable because the source-comment finding is ADVISORY (one tier softer than
  # the .md ERROR) and echo-exempted by code literals. PURE; SelfTest drives it.
  # Returns @{ Comment; Code; InBlock }.
  param([string]$Line, [string]$Ext, [bool]$InBlock)
  $s = [string]$Line
  $n = $s.Length
  $comment = New-Object System.Text.StringBuilder
  $code = New-Object System.Text.StringBuilder
  $hasBlock = ($Ext -eq 'rs' -or $Ext -eq 'ps1')
  $blkOpen = switch ($Ext) { 'rs' { '/*' } 'ps1' { '<#' } default { '' } }
  $blkClose = switch ($Ext) { 'rs' { '*/' } 'ps1' { '#>' } default { '' } }
  # Rust: only `"` is a string delim here (a bare `'` is a lifetime, not a string).
  $strChars = if ($Ext -eq 'rs') { @('"') } else { @('"', "'") }
  $inStr = $false; $strCh = ''
  # ps1: track a ${...} braced-variable-name span. A `#` INSIDE `${...}` is a
  # name char, not a comment (`${a#b}` is a variable per PSParser), so the broadened
  # ps1 comment rule below must skip it. Reset per line (a ps1 var name is single-line).
  $inPsBraceVar = $false
  $i = 0
  while ($i -lt $n) {
    $ch = [string]$s[$i]
    if ($InBlock) {
      if ($hasBlock -and ($i + 1 -lt $n) -and $s.Substring($i, 2) -eq $blkClose) { $InBlock = $false; $i += 2; continue }
      [void]$comment.Append($ch); $i++; continue
    }
    if ($inStr) {
      [void]$code.Append($ch)
      # Backslash escapes inside a string: rust strings, AND sh/toml DOUBLE-quoted
      # strings (`\"` is an escaped quote, so it does NOT close the string).
      # Single-quoted sh/toml strings take NO escapes, and ps1 escapes with a
      # backtick (handled below), not a backslash. (Codex CROSS-CRATE-CONTRACT: an
      # escaped quote in an sh/toml string prematurely closed it and mis-split a
      # trailing `#` as a comment.)
      if ($ch -eq '\' -and ($Ext -eq 'rs' -or (($Ext -eq 'sh' -or $Ext -eq 'toml') -and $strCh -eq '"'))) {
        if ($i + 1 -lt $n) { [void]$code.Append($s[$i + 1]) }; $i += 2; continue
      }
      if ($Ext -eq 'ps1' -and $ch -eq '`') { if ($i + 1 -lt $n) { [void]$code.Append($s[$i + 1]) }; $i += 2; continue }
      if ($ch -eq $strCh) { $inStr = $false; $strCh = '' }
      $i++; continue
    }
    if ($hasBlock -and ($i + 1 -lt $n) -and $s.Substring($i, 2) -eq $blkOpen) { $InBlock = $true; $i += 2; continue }
    # ps1: enter/leave a ${...} braced-variable-name span so a `#` inside it is
    # treated as a NAME char (code), not a comment. Opener `${`; the first `}`
    # closes it (a ps1 var name is a single-line, non-nesting run).
    if ($Ext -eq 'ps1' -and (-not $inPsBraceVar) -and ($i + 1 -lt $n) -and $s.Substring($i, 2) -eq '${') {
      [void]$code.Append('${'); $inPsBraceVar = $true; $i += 2; continue
    }
    if ($Ext -eq 'ps1' -and $inPsBraceVar -and $ch -eq '}') {
      [void]$code.Append('}'); $inPsBraceVar = $false; $i++; continue
    }
    if ($Ext -eq 'rs') {
      if (($i + 1 -lt $n) -and $s.Substring($i, 2) -eq '//') { [void]$comment.Append($s.Substring($i)); break }
    } elseif ($ch -eq '#') {
      if ($Ext -eq 'toml') {
        # toml: `#` anywhere outside a string is a comment.
        [void]$comment.Append($s.Substring($i)); break
      } elseif ($Ext -eq 'ps1') {
        # PowerShell starts a comment at a `#` after ANY completed token: `$x=1# c`
        # is `1` (a number, which cannot absorb `#`) then a comment (PSParser
        # confirms). The ONLY `#` that is NOT a comment outside strings and block
        # comments is one inside a `${...}` braced variable name (`${a#b}` is a
        # variable), handled by the $inPsBraceVar guard above -- so here any other
        # `#` is a comment. (Codex QUALITY: the prior separator-adjacency rule,
        # shared with sh, classified token-adjacent `$x=1# ...` as code and missed
        # the comment.)
        if (-not $inPsBraceVar) { [void]$comment.Append($s.Substring($i)); break }
      } else {
        # sh: a `#` starts a comment at line start, after whitespace, or after a
        # command separator (`;&|`) -- a separator starts a new word (`cmd;# note`
        # is a real comment). It does NOT start after `(){}`, which precede `#` in
        # EXPANSION syntax (`${#name}` length, `${name}#suffix` strip,
        # `$(cmd)#suffix`), nor after `$` (`$#`), nor word-embedded (`a#b`) -- all
        # code. DELIBERATE tradeoff: a real grouping-adjacent comment (`(cmd)# n`)
        # is sacrificed to code -- expansion suffixes are the far more common shape,
        # and a missed comment only under-reports an ADVISORY scan, while a false
        # split invents findings from code.
        $prev = if ($i -gt 0) { [string]$s[$i - 1] } else { '' }
        if ($i -eq 0 -or $prev -match '[\s;&|]') { [void]$comment.Append($s.Substring($i)); break }
      }
    }
    if ($strChars -contains $ch) { $inStr = $true; $strCh = $ch; [void]$code.Append($ch); $i++; continue }
    [void]$code.Append($ch); $i++
  }
  return @{ Comment = $comment.ToString(); Code = $code.ToString(); InBlock = $InBlock }
}

function Get-SourceCommentRecords {
  # Block-aware pass over a source file's line records -> comment-text records
  # @{ Line; Text=<NORMALIZED comment portion> } (only lines with non-blank
  # comment text). The block state carries across lines so a multi-line `/* */` /
  # `<# #>` body is scanned. The leading LINE-comment marker (`//`/`///`/`//!` or
  # `#`/`##`) is stripped from each record so (a) the shared fence matcher can see
  # a comment-prefixed code fence (`// ```` -> `` ``` ``) and (b) the reported
  # match is the prose, not the delimiter. (Codex TEST-QUALITY: the un-normalized
  # `//` prefix hid comment fences from Get-FencedLineFlags.)
  # Pure; SelfTest drives it.
  param([object[]]$Records, [string]$Ext)
  $out = @()
  if (-not $Records) { return $out }
  $inBlock = $false
  foreach ($r in $Records) {
    $wasInBlock = $inBlock
    $parts = Get-SourceLineParts -Line ([string]$r.Text) -Ext $Ext -InBlock $inBlock
    $inBlock = [bool]$parts.InBlock
    if (-not [string]::IsNullOrWhiteSpace([string]$parts.Comment)) {
      # Strip a leading line-comment marker (+ one space) so a `// ```` fence line
      # normalizes to `` ``` `` and the fence matcher recognizes it. A block-comment
      # line carries no per-line MARKER, but the conventional doc-block continuation
      # `*` (` * text` inside `/** */`) plays the same role -- normalize it on lines
      # that STARTED inside a block (never `*/`, which the splitter consumes) so a
      # ` * ```` fence line is recognized too. (Codex QUALITY: the leading `*` hid
      # block-doc fences from Get-FencedLineFlags.)
      $txt = [string]$parts.Comment -replace '^\s*(?://+!?|#+)\s?', ''
      if ($wasInBlock) { $txt = $txt -replace '^\s*\*+(?!/)\s?', '' }
      # WasInBlock lets the staged-source segmenter tell a block-comment
      # continuation (no code can precede it) from a fresh line comment after a
      # line-number gap (code intervened) -- see Get-StagedSourceFindings.
      $out += @{ Line = [int]$r.Line; Text = $txt; WasInBlock = $wasInBlock }
    }
  }
  # Return the plain array; every caller wraps with @() so a 0/1-element result
  # normalizes correctly (a `,$out` wrapper double-nests under the caller's @()).
  return $out
}

function Get-SourceCodeNumbersByLine {
  # Per-LINE standalone numeric literals -- integer or decimal, keyed by the FULL
  # token string ("223", "0.25") -- on the CODE portions of a source file
  # (block-aware): a hashtable Line -> { token-string -> $true }. The source-comment code-echo
  # exemption keys on this so it can require the echoing literal to be NEARBY the
  # comment (see Get-StagedSourceFindings), not merely present somewhere in the
  # file. (Codex QUALITY: a whole-file digit set let an UNRELATED
  # constant/string/identifier elsewhere in the file silently exempt a stale
  # comment -- the exemption must be local, "same line or nearby changed hunk".)
  # Pure; SelfTest drives it.
  param([object[]]$Records, [string]$Ext)
  $byLine = @{}
  if (-not $Records) { return $byLine }
  $inBlock = $false
  foreach ($r in $Records) {
    $parts = Get-SourceLineParts -Line ([string]$r.Text) -Ext $Ext -InBlock $inBlock
    $inBlock = [bool]$parts.InBlock
    $s = @{}
    # STANDALONE numeric literals -- integer OR decimal, keyed by the NUMERIC
    # token ("223", "0.25"): a digit run adjacent to a word char or a QUOTE is
    # part of an identifier / string (`ICON_223_NAME`, `"223"`, `"v223"`,
    # `v1.2`), and a `$`-, brace-, or sign-prefixed run (`$2`/`${2}` shell
    # positional / ps1 variable, `-2`/`+2` signed) is not the adjacent POSITIVE
    # literal the exemption describes -- none of those populate the echo set
    # (a rare brace-adjacent real literal only leaves the advisory STANDING --
    # the safe direction -- while a false echo would silently drop findings). A Rust TYPE
    # SUFFIX (`3usize`, `0.25f32`) is stripped so the literal enters under its
    # numeric key. Full-token keys mean a "223" comment claim never matches a
    # `0.223` code literal (or vice versa), while `// took 0.25` beside
    # `= 0.25` CAN exempt, per the documented adjacent-literal exemption.
    # (Codex QUALITY: identifier digits; then quoted-string digits; then decimal
    # support, then `$`/sign false-echo + suffixed-literal misses.)
    foreach ($m in ([regex]::Matches([string]$parts.Code, '(?<![\w."''$+{-])(\d+(?:\.\d+)?)(?:[iu](?:8|16|32|64|128|size)|f32|f64)?(?![\w."''])'))) { $s[$m.Groups[1].Value] = $true }
    $byLine[[int]$r.Line] = $s
  }
  return $byLine
}

function Get-StagedSourceFindings {
  # Staged per-source-file DECISION logic: scan the COMMENT lines this commit adds
  # for the count-drift classes (inventory-assertion SHAPE + MAGNITUDE), never the
  # code lines (a numeric literal in code is correct where it lives). The detectors
  # run over CONTIGUOUS comment SEGMENTS (a code line between line-comment blocks
  # resets the fence; a block comment's interior stays one segment) -- so their
  # shared fence matcher sees an UNCHANGED comment-fence opener/closer around an
  # added line WITHIN a segment, without leaking that fence across intervening code
  # -- and the findings are filtered to ADDED comment lines only AFTERWARD. (Codex
  # TEST-QUALITY: filtering before the detector lost fence context; Codex
  # SILENT-FAILURE: the un-segmented stream leaked fence state across code.) A finding is
  # code-echo EXEMPT only when its number appears on the CODE of a line within
  # $EchoWindow lines of the comment (a comment describing an ADJACENT constant,
  # e.g. `// there are 3 icons` above `const ICONS = 3`) -- an unrelated same
  # number elsewhere in the file does NOT exempt. Source-comment inventory findings
  # are DOWNGRADED to ADVISORY (one tier softer than the .md ERROR, per the
  # comment-extraction precision note). Pure (no git); SelfTest drives it.
  param([string]$Path, [string]$StagedText, [hashtable]$AddedSet, [string]$Ext)
  $EchoWindow = 2
  $lines = $StagedText -split "`n"
  $allRecs = @()
  for ($i = 0; $i -lt $lines.Count; $i++) { $allRecs += @{ Line = ($i + 1); Text = ($lines[$i] -replace "`r$", '') } }
  $commentRecs = @(Get-SourceCommentRecords -Records $allRecs -Ext $Ext)
  if (@($commentRecs | Where-Object { $AddedSet.ContainsKey([int]$_.Line) }).Count -eq 0) { return @() }
  $codeByLine = Get-SourceCodeNumbersByLine -Records $allRecs -Ext $Ext
  # SEGMENT the comment records so a `// ``` ` fence opened in one line-comment
  # block does NOT carry its fence state across intervening CODE into a later
  # disjoint comment (which would silently suppress a real finding). A new segment
  # starts at a comment record that is NOT contiguous with the previous one AND did
  # not START inside a block comment: a code line between two line-comment blocks
  # resets the fence, while a block comment's interior stays one segment (code
  # cannot appear between block-comment lines, and a dropped blank interior line is
  # fence content, not a code separator). Fence context still spans UNCHANGED
  # comment lines WITHIN a segment (SRC13). (Codex SILENT-FAILURE: the un-segmented
  # comment stream leaked fence state across code.)
  $segments = @()
  $curSeg = @()
  $prevLine = $null
  foreach ($cr in $commentRecs) {
    if (($null -ne $prevLine) -and ([int]$cr.Line -ne ($prevLine + 1)) -and (-not $cr.WasInBlock)) {
      if ($curSeg.Count -gt 0) { $segments += ,@($curSeg) }
      $curSeg = @()
    }
    $curSeg += $cr
    $prevLine = [int]$cr.Line
  }
  if ($curSeg.Count -gt 0) { $segments += ,@($curSeg) }
  $inv = @()
  $mag = @()
  foreach ($seg in $segments) {
    $inv += @(Get-InventoryAssertionFindings -Records @($seg))
    $mag += @(Get-MagnitudeFindings -Records @($seg))
  }
  $out = @()
  foreach ($f in (@($inv) + @($mag))) {
    if (-not $AddedSet.ContainsKey([int]$f.Line)) { continue }   # keep only findings on ADDED comment lines
    # NEARBY code-echo exemption: the finding's number must appear on the code of
    # a line within +/-$EchoWindow of the comment line to be exempted. Extract the
    # FULL numeric token INCLUDING any decimal (`\d+(?:\.\d+)?`): a decimal magnitude
    # like "took 0.25" compares as "0.25", never the fragment "0", and the code map
    # stores full decimal tokens too, so `// took 0.25` beside `= 0.25` exempts
    # while an unrelated integer `0` nearby cannot suppress it. (Codex BLOCKER;
    # QUALITY: decimal map support.) A SPELLED enumerated total ("all three cases",
    # "both cases") carries no digit -- map the spelled word to its numeral so
    # those can use the documented exemption too (`// covers all three cases`
    # beside `const CASES: usize = 3`). (Codex QUALITY.)
    $num = $null
    $numM = [regex]::Match([string]$f.Match, '\d+(?:\.\d+)?')
    if ($numM.Success) { $num = $numM.Value }
    else {
      $spelledM = [regex]::Match([string]$f.Match, '(?i)\b(both|two|three|four|five|six|seven|eight|nine|ten)\b')
      if ($spelledM.Success) {
        $num = @{ both = '2'; two = '2'; three = '3'; four = '4'; five = '5'; six = '6'; seven = '7'; eight = '8'; nine = '9'; ten = '10' }[$spelledM.Value.ToLowerInvariant()]
      }
    }
    if ($num) {
      $echoed = $false
      for ($ln = ([int]$f.Line - $EchoWindow); $ln -le ([int]$f.Line + $EchoWindow); $ln++) {
        if ($codeByLine.ContainsKey($ln) -and $codeByLine[$ln].ContainsKey($num)) { $echoed = $true; break }
      }
      if ($echoed) { continue }
    }
    if ($f.Class -eq 'INVENTORY') { $f['Tier'] = 'ADVISORY' }   # one tier softer than the .md ERROR
    $f['File'] = $Path
    $out += $f
  }
  return $out
}

function Get-AddedRecordsFromDiff {
  # Parse `git diff --cached --unified=0` text into per-file added-line records.
  # Returns @{ '<path>' = @( @{Line=<int>; Text=<string>} , ... ) }. New-line
  # numbers come from the `@@ -a,b +c,d @@` hunk headers.
  param([string]$DiffText)
  $byFile = @{}
  if ([string]::IsNullOrEmpty($DiffText)) { return $byFile }
  $curFile = $null
  $newLine = 0
  # HUNK STATE is essential: the `--- a/` / `+++ b/` file headers appear only in the
  # PRE-HUNK zone of each file section. Once inside a hunk (after `@@`), a line like
  # `+++ b/scripts/foo.rs:42` is ADDED CONTENT (a Markdown line literally adding
  # `++ b/...`), NOT a header -- matching the header regex unconditionally would drop
  # that added line and silently mask a lint violation on it. A `diff --git` line
  # starts a new file section and resets the zone. (Codex BLOCKER.)
  $inHunk = $false
  foreach ($raw in ($DiffText -split "`n")) {
    $line = $raw -replace "`r$", ''
    if ($line -match '^diff --git ') { $inHunk = $false; $curFile = $null; continue }
    if ($line -match '^@@\s+-\d+(?:,\d+)?\s+\+(\d+)(?:,\d+)?\s+@@') {
      $newLine = [int]$Matches[1]; $inHunk = $true; continue
    }
    if (-not $inHunk) {
      # Pre-hunk header zone: these are real file headers.
      if ($line -match '^\+\+\+\s+b/(.+)$') {
        $curFile = $Matches[1]
        if (-not $byFile.ContainsKey($curFile)) { $byFile[$curFile] = @() }
      }
      # `--- a/` and any other pre-hunk metadata: ignored.
      continue
    }
    if ($null -eq $curFile) { continue }
    # Inside a hunk: classify by the FIRST char only (so `+++ b/...` content counts).
    if ($line.StartsWith('+')) {
      $byFile[$curFile] += @{ Line = $newLine; Text = $line.Substring(1) }
      $newLine++
    } elseif ($line.StartsWith('-')) {
      # deleted line: does not advance the new-file counter
    } elseif ($line.StartsWith('\')) {
      # "\ No newline at end of file": ignore
    } else {
      # context line (rare at unified=0): advances new counter
      $newLine++
    }
  }
  return $byFile
}

function Get-AddedLineSetForPath {
  # Build the hashtable-set of ADDED line numbers for $Path from a
  # Get-AddedRecordsFromDiff result. ContainsKey + a per-record null guard so a
  # MISSING key yields a genuinely EMPTY set -- NOT a one-element @($null) whose
  # `[int]$null.Line` is 0. The staged-mode fail-loud guard ("parsed 0 added lines")
  # depends on the set being empty on a parse miss, so this MUST stay shared between
  # staged mode and its SelfTest (no duplicated copy that can drift). Pure (no I/O);
  # SelfTest drives it. (Codex BLOCKER: synthetic line-0 false-clean.)
  param([hashtable]$AddedByFile, [string]$Path)
  $set = @{}
  if ($AddedByFile -and $AddedByFile.ContainsKey($Path)) {
    foreach ($r in @($AddedByFile[$Path])) {
      if ($null -ne $r) { $set[[int]$r.Line] = $true }
    }
  }
  return $set
}

function Measure-InHunkAddedLines {
  # Count ADDED lines that sit INSIDE a hunk (after `@@`), with the SAME hunk
  # awareness as Get-AddedRecordsFromDiff. The staged fail-loud guard uses this as a
  # cross-check ("the diff has added content but the parser produced 0 records ->
  # fail"). Counting by first-char-inside-hunk means a content `+++ b/...` line (a
  # `+` line in a hunk) IS counted -- the prior `-notmatch '^\+\+\+'` filter wrongly
  # dropped it, so a violation on such a line could exit clean. Pure; SelfTest drives
  # it. (Codex BLOCKER.)
  param([string]$DiffText)
  if ([string]::IsNullOrEmpty($DiffText)) { return 0 }
  $n = 0
  $inHunk = $false
  foreach ($raw in ($DiffText -split "`n")) {
    $line = $raw -replace "`r$", ''
    if ($line -match '^diff --git ') { $inHunk = $false; continue }
    if ($line -match '^@@') { $inHunk = $true; continue }
    if ($inHunk -and $line.StartsWith('+')) { $n++ }
  }
  return $n
}

function Get-HistoryEntryBodyLineSet {
  # For the root History.md log ONLY: return the hashtable-set of line numbers that
  # fall inside a DATED entry BODY -- the lines AFTER a `## YYYY-MM-DD ...` heading
  # (incl. the established slash day-range form `## YYYY-MM-DD/NN ...`)
  # through the line before the NEXT `## ` heading (dated or not) or EOF. The dated
  # heading line itself is EXCLUDED (the waiver is body-only per every contract
  # site; a count-bearing dated title stays in class -- Codex BLOCKER
  # review-20260710). The count-drift exemption applies ONLY to these lines, matching the
  # review contract's "a dated History.md ENTRY BODY": the file PREAMBLE (before the
  # first dated heading) and any UNDATED `## ` section (e.g. `## Notes`) are a static
  # snapshot too, but the contract scopes the waiver to DATED entries, so their
  # counts stay in class. The whole-file gate the first cut used (leaf name ==
  # History.md) over-exempted all three -- preamble, undated sections, and a nested
  # file named History.md -- past the contract; the caller now pairs this mask with a
  # ROOT-path gate. (Codex CROSS-CRATE-CONTRACT review-20260709.)
  # `## ` boundaries are detected only OUTSIDE code fences (shared Get-FencedLineFlags),
  # so a `## ...` line inside a ``` block is not mistaken for a heading. An H3 (`### `)
  # is NOT an H2 boundary (a subsection stays inside its dated entry). Pure; SelfTest
  # drives it.
  param([object[]]$Records)
  $set = @{}
  if (-not $Records) { return $set }
  $texts = @($Records | ForEach-Object { [string]$_.Text })
  $fenced = Get-FencedLineFlags -Lines $texts
  $inDatedEntry = $false
  for ($i = 0; $i -lt $Records.Count; $i++) {
    $text = [string]$Records[$i].Text
    # A level-2 ATX heading (`## `, not `### `) OUTSIDE a fence is an entry boundary:
    # a dated heading opens a dated entry; ANY other `## ` heading closes the dated
    # region (an undated section is not part of a dated entry).
    if (-not $fenced[$i] -and $text -match '^##\s') {
      # The date must END at a boundary: whitespace, EOL, or the established
      # slash day-range form `/NN` itself ending at whitespace/EOL
      # (`## 2026-05-01/02 ...` exists in the root log). A looser tail would
      # accept `## 2026-07-090 notes`, `## 2026-07-09foo`, or `## 2026-07-09/9foo`
      # and open the exempt region for a non-dated section. (Codex BLOCKER
      # review-20260711: prefix leak, range-form exclusion, then malformed
      # range suffixes -- each direction is pinned by HEB7/HEB8.)
      $inDatedEntry = ($text -match '^##\s+\d{4}-\d{2}-\d{2}(\s|$|/\d{2}(\s|$))')
      # The HEADING itself is NOT in the mask -- every contract site scopes the
      # waiver to the entry BODY, so a count-bearing dated title stays in class.
      # (Codex BLOCKER review-20260710: the heading leaked into the exempt set.)
      continue
    }
    if ($inDatedEntry) { $set[[int]$Records[$i].Line] = $true }
  }
  return $set
}

# ---------------------------------------------------------------------------
# Check runner: given a file path, its records (line/text), and the repo
# resolver seams, run every check and return findings tagged with the file.
# ---------------------------------------------------------------------------
function Invoke-FileChecks {
  param(
    [string]$Path,
    [object[]]$Records,
    [string]$PlanContentForClosure,   # full PLAN.md content if this file IS PLAN.md, else $null
    [scriptblock]$Resolver,
    [scriptblock]$IsTopLevel
  )
  $findings = @()
  # Frozen-snapshot status comes from the doc's OWN content (the explicit marker),
  # not its filename. $Records carry the full doc text (one record per line), so
  # reconstruct the head and probe for the marker. (Marker model.)
  $headText = (@($Records | Select-Object -First 30 | ForEach-Object { [string]$_.Text }) -join "`n")
  $isFrozen = Test-HasFrozenSnapshotMarker -Path $Path -Content $headText
  # $isHistory (LEAF name) drives the SELF-NARR tier: History.md (the retrospective
  # landing log) is ERROR, every other narrative doc ADVISORY -- an entry narrating
  # its OWN not-yet-landed merge is a factual defect (the merge SHA is unknowable
  # while the entry rides the branch), so it stays ERROR here. LEAF-based (nested
  # History.md included) since SELF-NARR is a STRICTER treatment -- over-applying it
  # only adds flagging, the safe direction.
  $isHistory = ([System.IO.Path]::GetFileName($Path) -eq 'History.md')
  # Count-drift EXEMPTION (INVENTORY + MAGNITUDE): a dated History entry is a static,
  # append-only snapshot -- once written its counts cannot drift the way live-doc
  # prose does -- so the operator ruled an exact count inside a dated entry is NOT a
  # review finding (2026-07-09). The waiver is NARROWER than SELF-NARR on BOTH axes,
  # matching the review-prompt-template contract ("a dated History.md ENTRY BODY"):
  #   - FILE: only the ROOT History.md log ($relPath -ceq 'History.md' -- CASE-
  #     SENSITIVE, since PowerShell -eq would also exempt a root `history.md`
  #     case variant), NOT a nested file named History.md. A waiver OVER-applied
  #     is the risk here (opposite to SELF-NARR's strictness, whose -eq leaf
  #     match errs STRICTER on a case variant), so the file gate is the exact
  #     root path, exact case, not the leaf.
  #   - LINE: only lines inside a DATED entry body (Get-HistoryEntryBodyLineSet); the
  #     file PREAMBLE and any UNDATED `## ` section stay in class.
  # The detectors still RUN; a finding whose line is exempt is dropped below. DRIFT-
  # staleness waiver ONLY: a History count that mis-states a LIVE contract the SAME
  # diff changes is a factual doc-vs-code defect, but this mechanical SHAPE detector
  # cannot tell drift-bait from a factual mismatch, so it defers that class to the AI
  # gate for the exempt lines (the gate stays the failsafe; the reviewer-side contract
  # is in review-prompt-template.md). LINE-ANCHOR and SELF-NARR are deliberately NOT
  # count-exempted: a `file.ext:NNN` anchor drifts because the cited SOURCE moves (its
  # exemption is the frozen-snapshot marker), and self-narration is a factual defect,
  # not a count. (Codex CROSS-CRATE-CONTRACT review-20260709: the first cut skipped
  # both detectors for the WHOLE file off the leaf name, over-exempting the preamble,
  # undated sections, and nested History.md past the contract.)
  $relPath = ([string]$Path -replace '\\', '/')
  $countExemptLines = if ($relPath -ceq 'History.md') { Get-HistoryEntryBodyLineSet -Records $Records } else { @{} }
  foreach ($f in (Get-LineAnchorFindings -Records $Records -IsFrozenSnapshot $isFrozen)) { $f['File'] = $Path; $findings += $f }
  foreach ($f in (Get-DeadRefFindings -Records $Records -Resolver $Resolver -IsTopLevel $IsTopLevel)) { $f['File'] = $Path; $findings += $f }
  foreach ($f in (Get-MagnitudeFindings -Records $Records)) {
    if ($countExemptLines.ContainsKey([int]$f.Line)) { continue }   # inside a dated History entry body -> count-drift exempt
    $f['File'] = $Path; $findings += $f
  }
  foreach ($f in (Get-SelfNarrationFindings -Records $Records -IsHistoryDoc $isHistory)) { $f['File'] = $Path; $findings += $f }
  foreach ($f in (Get-LocalEvidenceFindings -Records $Records)) { $f['File'] = $Path; $findings += $f }
  foreach ($f in (Get-MachineLocalStateFindings -Records $Records)) { $f['File'] = $Path; $findings += $f }
  # Inventory-assertion SHAPE detector: ERROR in .md prose, with NO frozen-snapshot
  # downgrade (the marker exemption is line-anchor-scoped -- see the detector).
  # No code-echo exemption in a .md doc (there is no code). Source-file
  # COMMENT scanning runs on its own staged path (Get-StagedSourceFindings). A finding
  # on a dated-History-entry-body line is dropped per $countExemptLines above.
  foreach ($f in (Get-InventoryAssertionFindings -Records $Records)) {
    if ($countExemptLines.ContainsKey([int]$f.Line)) { continue }
    $f['File'] = $Path; $findings += $f
  }
  if ($PlanContentForClosure) {
    foreach ($f in (Get-MilestoneTagFindings -PlanContent $PlanContentForClosure)) { $f['File'] = $Path; $findings += $f }
  }
  return $findings
}

function Get-StagedFileFindings {
  # Pure staged per-file DECISION logic (no git): given a staged file's full
  # text, the set of ADDED line numbers, and the resolver seams, return the
  # findings that apply to THIS commit. Full-file records give fence / PLAN
  # tag-closure context; PLAN TAG closure (a WHOLE-FILE property -- a dangling
  # ref can come from DELETING a header, never an added line) then BYPASSES the
  # added-line filter, while line-oriented classes keep it (only NEW instances).
  # Extracted so the staged decision logic (where several gate BLOCKERs hid) is
  # unit-testable without a git repo. (Codex TEST-QUALITY.)
  param([string]$Path, [string]$StagedText, [hashtable]$AddedSet, [scriptblock]$Resolver, [scriptblock]$IsTopLevel)
  $lines = $StagedText -split "`n"
  $allRecs = @()
  for ($i = 0; $i -lt $lines.Count; $i++) { $allRecs += @{ Line = ($i + 1); Text = ($lines[$i] -replace "`r$", '') } }
  $isPlan = ([System.IO.Path]::GetFileName($Path) -eq 'PLAN.md')
  $planContent = if ($isPlan) { $StagedText } else { $null }
  $fileFindings = Invoke-FileChecks -Path $Path -Records $allRecs -PlanContentForClosure $planContent -Resolver $Resolver -IsTopLevel $IsTopLevel
  $out = @()
  foreach ($f in $fileFindings) {
    if ($f.Class -eq 'TAG' -or $AddedSet.ContainsKey([int]$f.Line)) { $out += $f }
  }
  return $out
}

function Format-Report {
  param([object[]]$Findings, [string]$ScopeLabel, [int]$FileCount)
  $sb = [System.Text.StringBuilder]::new()
  [void]$sb.AppendLine("[author-lint] scope: $ScopeLabel ($FileCount file(s) scanned)")
  # Provenance stamp. The generated-doc skip is PATH-gated (Test-IsGeneratedContent
  # requires $script:GeneratedDocPathPattern), so this stamp self-skips a report
  # ONLY if it is written to the allowlisted generated path. -OutPath is an
  # unrestricted string and author-lint reports are conventionally written to
  # gitignored logs/; a report sent to any other tracked path is NOT auto-skipped,
  # which is the intended fail-safe. The `by author-lint.ps1` signature is retained
  # for the OR-branch and provenance. (Codex BLOCKER: content-only
  # skip was spoofable.)
  [void]$sb.AppendLine("Generated: $((Get-Date).ToString('yyyy-MM-ddTHH:mm:sszzz')) by author-lint.ps1")
  $errors = @($Findings | Where-Object { $_.Tier -eq 'ERROR' })
  $advis  = @($Findings | Where-Object { $_.Tier -eq 'ADVISORY' })
  if ($Findings.Count -eq 0) {
    [void]$sb.AppendLine('[author-lint] no findings.')
  } else {
    foreach ($f in ($Findings | Sort-Object @{e={$_.Tier}}, @{e={$_.File}}, @{e={[int]$_.Line}})) {
      [void]$sb.AppendLine(('{0,-8} {1,-12} {2}:{3}  {4}  -> {5}' -f $f.Tier, $f.Class, $f.File, $f.Line, $f.Match, $f.Hint))
    }
  }
  $verdict = if ($errors.Count -gt 0) { 'FAIL (error-tier findings present)' } else { 'OK (no error-tier findings)' }
  [void]$sb.AppendLine("[author-lint] $($errors.Count) error(s), $($advis.Count) advisory. $verdict")
  return $sb.ToString()
}

function Write-ReportFile {
  param([string]$Body, [string]$Path)
  # Best-effort write: full body to a sibling temp, then replace the target, so a
  # mid-write failure does not truncate an existing report. NOTE this is NOT the
  # MoveFileEx atomic-replace the sibling dispatch-checklist.ps1 uses for its
  # GATE artifact -- `Move-Item -Force` is adequate here because -OutPath is an
  # optional convenience copy (the report also always prints to stdout), not a
  # consumed artifact. Do not claim atomic-replacement semantics for it.
  $dir = Split-Path -Parent -Path $Path
  if ($dir -and -not (Test-Path -LiteralPath $dir -PathType Container)) {
    New-Item -ItemType Directory -Path $dir -Force | Out-Null
  }
  $leaf = Split-Path -Leaf -Path $Path
  $targetDir = if ($dir) { $dir } else { '.' }
  $tmp = Join-Path $targetDir (".$leaf.tmp-" + [guid]::NewGuid().ToString('N').Substring(0, 12))
  [System.IO.File]::WriteAllText($tmp, $Body, $script:Utf8NoBom)
  Move-Item -LiteralPath $tmp -Destination $Path -Force
}

# ===========================================================================
# SelfTest
# ===========================================================================
if ($SelfTest) {
  $script:failures = 0
  function Assert-True {
    param([string]$Name, [bool]$Cond, [string]$Detail = '')
    if ($Cond) { Write-Host "[SelfTest] PASS $Name" }
    else { Write-Host "[SelfTest] FAIL ${Name}: $Detail"; $script:failures++ }
  }
  function New-Rec { param([int]$Line, [string]$Text) return @{ Line = $Line; Text = $Text } }
  function Invoke-E2EStagedSource {
    # Alt-index staged-source E2E harness: seed an ALTERNATE index (GIT_INDEX_FILE)
    # from HEAD, stage a temp .rs carrying $Content into it, run staged scope as a
    # child, and return @{ Exit; Text; SrcRel; SetupOk; CleanOk }. The REAL worktree
    # index is NEVER touched. The git setup exit codes are CAPTURED and surfaced via
    # SetupOk (a swallowed read-tree/add failure could otherwise false-pass), and
    # ALL temp artifacts (alt index, its git .lock, probe source) get the same
    # checked Test-Path cleanup as the $e2eDoc fixtures, so a leaked file fails
    # the suite instead of passing green. (Codex BLOCKER.)
    param([string]$RepoRoot, [string]$Content)
    $altIdx = Join-Path $RepoRoot ('.authorlint-altidx-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
    $srcRel = 'authorlint-src-selftest-' + [guid]::NewGuid().ToString('N').Substring(0, 8) + '.rs'
    $src = Join-Path $RepoRoot $srcRel
    $savedIdx = $env:GIT_INDEX_FILE
    $res = @{ Exit = $null; Text = ''; SrcRel = $srcRel; SetupOk = $false; CleanOk = $true }
    try {
      Set-Content -LiteralPath $src -Value $Content -Encoding UTF8
      $env:GIT_INDEX_FILE = $altIdx
      # Capture-to-$null under a LOCAL Continue pref so git's autocrlf warning does
      # not abort the op (a `2>&1 | Out-Null` under Stop swallows the warning AND
      # drops the add, leaving the alt index unseeded).
      $eap = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
      $null = (& git -C $RepoRoot read-tree HEAD 2>&1); $rtExit = $LASTEXITCODE
      $null = (& git -C $RepoRoot add -- $srcRel 2>&1); $addExit = $LASTEXITCODE
      $ErrorActionPreference = $eap
      $res.SetupOk = (($rtExit -eq 0) -and ($addExit -eq 0))
      if ($res.SetupOk) {
        $res.Text = ((& powershell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -Staged 2>&1) | Out-String)
        $res.Exit = $LASTEXITCODE
      } else {
        $res.Text = "SETUP-FAIL read-tree=$rtExit add=$addExit"
      }
    } finally {
      $env:GIT_INDEX_FILE = $savedIdx   # restore ($null clears it); the REAL index was never touched
      # Checked cleanup of EVERY temp artifact -- the alt-index, its possible stray
      # git `.lock`, and the probe src -- so a leaked file fails CleanOk instead of
      # a green self-test. (Codex QUALITY: the .lock was previously removed
      # SilentlyContinue and never updated CleanOk.)
      foreach ($p in @($altIdx, ($altIdx + '.lock'), $src)) {
        if (Test-Path -LiteralPath $p) {
          try { Remove-Item -LiteralPath $p -ErrorAction Stop } catch { $res.CleanOk = $false }
          if (Test-Path -LiteralPath $p) { $res.CleanOk = $false }
        }
      }
    }
    return $res
  }

  # ---- LINE-ANCHOR ----
  $laPos = @(New-Rec 5 'see scripts/codex/auto-review.ps1:1941 for detail')
  $laHits = @(Get-LineAnchorFindings -Records $laPos)
  Assert-True 'LA1: bare file.ext:NNN is flagged' ($laHits.Count -eq 1) "count=$($laHits.Count)"
  Assert-True 'LA1b: flagged match is the anchor token' ($laHits.Count -eq 1 -and $laHits[0].Match -eq 'scripts/codex/auto-review.ps1:1941') "match=$($laHits[0].Match)"
  Assert-True 'LA1c: line number carried through' ($laHits.Count -eq 1 -and $laHits[0].Line -eq 5) "line=$($laHits[0].Line)"

  $laNeg = @(New-Rec 1 'see auto_review_system for detail (no anchor)')
  Assert-True 'LA2: symbol reference without :NNN is clean' (@(Get-LineAnchorFindings -Records $laNeg).Count -eq 0) ''

  $laFence = @(New-Rec 1 '```'; New-Rec 2 'auto-review.ps1:1941'; New-Rec 3 '```')
  Assert-True 'LA4: anchor inside a fenced code block is skipped' (@(Get-LineAnchorFindings -Records $laFence).Count -eq 0) ''

  $laMVerdict = @(New-Rec 9 'never write a bare file.rs:NNN citation')
  Assert-True 'LA5: file.rs:NNN (NNN not digits) is not a false positive' (@(Get-LineAnchorFindings -Records $laMVerdict).Count -eq 0) ''

  $laWin = @(New-Rec 1 'see scripts\codex\auto-review.ps1:1941 for detail')
  $laWinHits = @(Get-LineAnchorFindings -Records $laWin)
  Assert-True 'LA6: a Windows-separator anchor (scripts\codex\..:NNN) is flagged' ($laWinHits.Count -eq 1) "count=$($laWinHits.Count)"

  $laPy = @(New-Rec 1 'see scripts/check_registry.py:100 for the check')
  Assert-True 'LA7: a Python .py:NNN anchor is flagged (error-tier extension coverage)' (@(Get-LineAnchorFindings -Records $laPy).Count -eq 1) ''

  # Text shader/markup extensions (wgsl/glsl/xml) added to $AnchorExtAlternation.
  $laWgsl = @(New-Rec 1 'the bug is in src/render/water.wgsl:42 today')
  Assert-True 'LA7b: a .wgsl:NNN shader anchor is flagged (new ext coverage)' (@(Get-LineAnchorFindings -Records $laWgsl).Count -eq 1) ''
  $laXml = @(New-Rec 1 'config drift at assets/config/items.xml:17 here')
  Assert-True 'LA7c: an .xml:NNN markup anchor is flagged (new ext coverage)' (@(Get-LineAnchorFindings -Records $laXml).Count -eq 1) ''
  $laGlsl = @(New-Rec 1 'see src/render/post.glsl:88 for the pass')
  Assert-True 'LA7c2: a .glsl:NNN shader anchor is flagged (new ext coverage)' (@(Get-LineAnchorFindings -Records $laGlsl).Count -eq 1) ''
  # A BINARY asset ext is intentionally NOT an anchor ext: `model.glb:42` is not a
  # line citation, so it must NOT be flagged.
  $laGlb = @(New-Rec 1 'the mesh assets/models/main.glb:42 reference')
  Assert-True 'LA7d: a binary .glb:NNN is NOT flagged (binary assets excluded)' (@(Get-LineAnchorFindings -Records $laGlb).Count -eq 0) ''

  $laSnap = @(Get-LineAnchorFindings -Records $laPos -IsFrozenSnapshot $true)
  Assert-True 'LA8: a frozen-snapshot-marked doc downgrades the anchor to advisory (not error, not skipped)' ($laSnap.Count -eq 1 -and $laSnap[0].Tier -eq 'ADVISORY') "count=$($laSnap.Count); tier=$($laSnap[0].Tier)"
  # ONLY a URL-EMBEDDED token is exempt: `app.js:8080` inside `https://host/app.js:8080`
  # is part of the URL (scheme:// before it, no whitespace), not a source line.
  $laUrl = @(New-Rec 1 'the docs live at https://host/app.js:8080 today')
  Assert-True 'LA9: a URL-embedded token (https://host/app.js:8080) is NOT a LINE-ANCHOR (scheme:// guard)' (@(Get-LineAnchorFindings -Records $laUrl -IsFrozenSnapshot $false).Count -eq 0) ''
  # A repo-ROOT absolute anchor IS still a line citation and must flag (the earlier
  # leading-`/` skip wrongly cleared it). (Codex BLOCKER.)
  $laRoot = @(New-Rec 1 'broken at /scripts/codex/auto-review.ps1:42 today')
  Assert-True 'LA9b: a repo-root absolute anchor (/scripts/...:42) IS a LINE-ANCHOR (not skipped)' (@(Get-LineAnchorFindings -Records $laRoot -IsFrozenSnapshot $false).Count -eq 1) ''
  # An anchor inside a ~~~ fenced block is an example, not a live citation.
  $laTilde = @(New-Rec 1 '~~~'; New-Rec 2 'see scripts/foo.ps1:42 here'; New-Rec 3 '~~~')
  Assert-True 'LA10: an anchor inside a ~~~ fence is skipped (both fence chars honored)' (@(Get-LineAnchorFindings -Records $laTilde -IsFrozenSnapshot $false).Count -eq 0) ''
  # A BARE filename anchor (no directory) still flags -- the URL-only exemption must
  # not over-skip. (Regression guard for the URL-only model.)
  $laBare = @(New-Rec 1 'see animation.rs:42 for the bug')
  Assert-True 'LA11: a bare filename anchor (animation.rs:42) IS a LINE-ANCHOR' (@(Get-LineAnchorFindings -Records $laBare -IsFrozenSnapshot $false).Count -eq 1) ''
  # A ~~~ line INSIDE a ``` block is content, not a close: the opener char is
  # tracked, so an anchor after the inner ~~~ but before the matching ``` stays
  # skipped. (Codex QUALITY: mixed delimiters.)
  $laMixed = @(New-Rec 1 '```'; New-Rec 2 '~~~'; New-Rec 3 'see scripts/foo.ps1:42 inside'; New-Rec 4 '```'; New-Rec 5 'live scripts/bar.ps1:7 outside')
  $laMixedHits = @(Get-LineAnchorFindings -Records $laMixed -IsFrozenSnapshot $false)
  Assert-True 'LA12: a ~~~ inside a ``` block does not close it; the post-``` anchor is the only finding' (($laMixedHits.Count -eq 1) -and ($laMixedHits[0].Line -eq 5)) "count=$($laMixedHits.Count); $([string]::Join('|', @($laMixedHits | ForEach-Object { "$($_.Match)@$($_.Line)" })))"
  # CommonMark: a shorter same-char fence does NOT close a longer one. A ``` (3)
  # inside a ```` (4) block is content; only the >=4 fence closes it. (Codex
  # QUALITY: shorter same-char fence.)
  $laLen = @(New-Rec 1 '````'; New-Rec 2 '```'; New-Rec 3 'see scripts/foo.ps1:42 still inside'; New-Rec 4 '````'; New-Rec 5 'live scripts/bar.ps1:7 outside')
  $laLenHits = @(Get-LineAnchorFindings -Records $laLen -IsFrozenSnapshot $false)
  Assert-True 'LA13: a ``` (len 3) does not close a ```` (len 4) block; only the post-fence anchor flags' (($laLenHits.Count -eq 1) -and ($laLenHits[0].Line -eq 5)) "count=$($laLenHits.Count); $([string]::Join('|', @($laLenHits | ForEach-Object { "$($_.Match)@$($_.Line)" })))"
  # CommonMark: a same-char fence line with TRAILING content (an info string) is NOT
  # a close. A `~~~text` inside a ~~~ block is content; only a bare ~~~ closes it.
  # (Codex QUALITY: trailing-content close.)
  $laTrail = @(New-Rec 1 '~~~'; New-Rec 2 '~~~rust'; New-Rec 3 'see scripts/foo.ps1:42 still inside'; New-Rec 4 '~~~'; New-Rec 5 'live scripts/bar.ps1:7 outside')
  $laTrailHits = @(Get-LineAnchorFindings -Records $laTrail -IsFrozenSnapshot $false)
  Assert-True 'LA14: a ~~~text line (trailing content) does not close the block; only the bare-fence-close anchor flags' (($laTrailHits.Count -eq 1) -and ($laTrailHits[0].Line -eq 5)) "count=$($laTrailHits.Count); $([string]::Join('|', @($laTrailHits | ForEach-Object { "$($_.Match)@$($_.Line)" })))"
  # CommonMark: >3 leading spaces (or a tab) is indented code, NOT a fence. An
  # over-indented ~~~ must NOT open a synthetic fence that suppresses live findings.
  # (Codex BLOCKER.)
  $laIndent = @(New-Rec 1 '    ~~~'; New-Rec 2 'see scripts/foo.ps1:42 not fenced')
  Assert-True 'LA15: a 4-space-indented ~~~ does not open a fence; the anchor still flags' (@(Get-LineAnchorFindings -Records $laIndent -IsFrozenSnapshot $false).Count -eq 1) ''
  $laTab = @(New-Rec 1 "`t~~~"; New-Rec 2 'see scripts/foo.ps1:42 not fenced')
  Assert-True 'LA16: a tab-indented ~~~ does not open a fence; the anchor still flags' (@(Get-LineAnchorFindings -Records $laTab -IsFrozenSnapshot $false).Count -eq 1) ''
  # ---- FROZEN-SNAPSHOT MARKER (Test-HasFrozenSnapshotMarker) ----
  # The marker model: a doc is frozen IFF a line matching
  # ^\s*<!--\s*frozen-snapshot\s*-->\s*$ appears within the first ~30 lines. The
  # decision is CONTENT-based -- filename and date no longer matter. (Supersedes the
  # filename+date+category Test-IsDatedSnapshotDoc and its env-var denylist.)
  # PRESENT cases (exempt -> advisory):
  $mkTop = "# Audit Report`n<!-- frozen-snapshot -->`n`nbody cites foo.rs:42"
  Assert-True 'SNAP1: the marker on its own line just under the H1 is detected (frozen)' (Test-HasFrozenSnapshotMarker -Path 'docs/anything.md' -Content $mkTop) ''
  $mkFirst = "<!-- frozen-snapshot -->`n# Title`nbody"
  Assert-True 'SNAP1b: the marker as the very first line is detected' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkFirst) ''
  # FILENAME and DATE are irrelevant: a NON-dated filename WITH the marker is frozen.
  $mkNonDated = "# Notes`n<!-- frozen-snapshot -->`nbody"
  Assert-True 'SNAP1c: a NON-dated filename WITH the marker is frozen (filename no longer matters)' (Test-HasFrozenSnapshotMarker -Path 'docs/notes.md' -Content $mkNonDated) ''
  # Extra inner whitespace around the token and the comment delimiters is tolerated.
  $mkSpaced = "# T`n   <!--   frozen-snapshot   -->   `nbody"
  Assert-True 'SNAP1d: inner/leading/trailing whitespace around the marker is tolerated' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkSpaced) ''
  # The token match is case-INSENSITIVE.
  $mkCase = "# T`n<!-- Frozen-Snapshot -->`nbody"
  Assert-True 'SNAP1e: the marker token is case-insensitive (Frozen-Snapshot)' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkCase) ''
  # A marker within the head window (<=30 lines) is detected; just past it is NOT.
  $mkHead = ((1..29 | ForEach-Object { "line $_" }) -join "`n") + "`n<!-- frozen-snapshot -->"
  Assert-True 'SNAP1f: a marker on line 30 (within the ~30-line head) is detected' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkHead) ''
  $mkDeep = ((1..40 | ForEach-Object { "line $_" }) -join "`n") + "`n<!-- frozen-snapshot -->"
  Assert-True 'SNAP1g: a marker buried past the head window is NOT detected (must be near the top)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkDeep)) ''
  # ABSENT cases (NOT frozen -> error-tier):
  # A DATED filename WITHOUT the marker is NOT frozen -- the whole point of the
  # model (it may be live authority).
  $noMkDated = "# Networking Audit 2026-05-23`nbody cites foo.rs:42, still live authority"
  Assert-True 'SNAP2: a DATED-in-name doc WITHOUT the marker is NOT frozen (date no longer exempts)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/networking-audit-2026-05-23.md' -Content $noMkDated)) ''
  $noMkPlain = "# Title`njust prose, no marker`nbody"
  Assert-True 'SNAP3: a plain doc with no marker is NOT frozen' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $noMkPlain)) ''
  # A marker mention INSIDE prose (not on its own line) is NOT the marker.
  $noMkInline = "# Title`nadd a <!-- frozen-snapshot --> marker to freeze it`nbody"
  Assert-True 'SNAP4: a marker mentioned mid-line (not on its own line) is NOT detected (own-line anchor)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $noMkInline)) ''
  # A malformed/near-miss marker is NOT detected (wrong token, trailing text).
  $noMkWrong = "# T`n<!-- frozen snapshot -->`nbody"
  Assert-True 'SNAP4b: a wrong token (space not hyphen: frozen snapshot) is NOT the marker' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $noMkWrong)) ''
  $noMkTrail = "# T`n<!-- frozen-snapshot --> and more text`nbody"
  Assert-True 'SNAP4c: a marker line with trailing non-whitespace text is NOT detected (anchored line)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $noMkTrail)) ''
  # Malformed-input class: empty / null content is NOT frozen (no throw).
  Assert-True 'SNAP5: empty content is NOT frozen (no throw)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content '')) ''
  Assert-True 'SNAP5b: null content is NOT frozen (no throw)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $null)) ''
  # CRLF line endings: the marker is still detected (the `r is stripped per line).
  $mkCrlf = "# T`r`n<!-- frozen-snapshot -->`r`nbody"
  Assert-True 'SNAP6: the marker is detected under CRLF line endings (trailing CR stripped)' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkCrlf) ''
  # A marker token INSIDE a ``` fenced code block renders as VISIBLE code, not an
  # invisible comment -> must NOT grant the exemption. (Codex CROSS-CRATE-CONTRACT:
  # the raw-line scan accepted a fenced/indented marker.)
  $mkFenced = "# Title`n``````markdown`n<!-- frozen-snapshot -->`n``````" + "`nbody cites foo.rs:42"
  Assert-True 'SNAP7: a marker INSIDE a fenced code block is NOT detected (renders as visible code)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkFenced)) ''
  # A marker on a CommonMark indented-code line (>=4 leading spaces) renders as code.
  $mkIndent4 = "# Title`n    <!-- frozen-snapshot -->`nbody"
  Assert-True 'SNAP8: a 4-space-indented marker is NOT detected (CommonMark indented code)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkIndent4)) ''
  $mkIndentTab = "# Title`n`t<!-- frozen-snapshot -->`nbody"
  Assert-True 'SNAP8b: a tab-indented marker is NOT detected (CommonMark indented code)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkIndentTab)) ''
  # A 1-3 space indent is normal Markdown, NOT indented code -> a real marker there IS
  # detected (the indent guard must not over-reject valid leading whitespace).
  $mkIndent3 = "# Title`n   <!-- frozen-snapshot -->`nbody"
  Assert-True 'SNAP8c: a 3-space-indented marker IS detected (normal Markdown, not indented code)' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkIndent3) ''
  # 0-3 spaces FOLLOWED BY A TAB is also CommonMark indented code (the tab expands to
  # column 4), so a marker there renders as visible code -> must NOT be detected.
  # (Codex BLOCKER: the indent guard rejected only >=4 spaces / a
  # column-0 tab, so a `<=3 spaces><tab>` marker passed as a false exemption.)
  $mkIndentSpaceTab = "# Title`n   `t<!-- frozen-snapshot -->`nbody"
  Assert-True 'SNAP8d: a (0-3 spaces + tab) indent is NOT detected (tab expands to col 4 -> indented code)' (-not (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkIndentSpaceTab)) ''
  # A REAL marker on a normal line AFTER a closed fenced block (within the head) IS
  # detected -- fence tracking must close correctly, not swallow the rest of the head.
  $mkAfterFence = "# Title`n``````text`nexample`n``````" + "`n<!-- frozen-snapshot -->`nbody"
  Assert-True 'SNAP9: a real marker AFTER a closed fence is detected (fence state closes)' (Test-HasFrozenSnapshotMarker -Path 'docs/x.md' -Content $mkAfterFence) ''

  # ---- PATH CONTAINMENT (Get-ContainedRelPath; BLOCKER) ----
  # A caller-supplied -Paths value must be proven inside the repo root BEFORE it is
  # turned into a repo-relative path; outside-repo abs paths and `..`-escapes must
  # be REJECTED ($null), never silently scanned against the repo index.
  $crpRoot = 'C:\repo\proj'
  # Positive: rooted-inside, relative-inside (incl. nested), and the root itself.
  Assert-True 'CRP-RootedInside: an in-repo absolute path -> repo-relative' ((Get-ContainedRelPath -RequestedPath 'C:\repo\proj\docs\foo.md' -RepoRootFull $crpRoot) -eq 'docs/foo.md') ''
  Assert-True 'CRP-RelativeInside: a plain relative path resolves under the repo root' ((Get-ContainedRelPath -RequestedPath 'docs\foo.md' -RepoRootFull $crpRoot) -eq 'docs/foo.md') ''
  Assert-True 'CRP-RelativeNested: a deeper relative path' ((Get-ContainedRelPath -RequestedPath 'scripts/codex/x.md' -RepoRootFull $crpRoot) -eq 'scripts/codex/x.md') ''
  Assert-True 'CRP-RelativeDotCollapses: an interior `.`/`..` that stays inside is allowed' ((Get-ContainedRelPath -RequestedPath 'docs\..\README.md' -RepoRootFull $crpRoot) -eq 'README.md') ''
  # Root-exact: the repo root itself is CONTAINED (the .Equals branch) and yields an
  # empty repo-relative path (the -Paths loop then skips it as a non-Markdown selection).
  Assert-True 'CRP-RootItself: the repo root path itself is contained -> empty rel (NOT $null)' ((Get-ContainedRelPath -RequestedPath $crpRoot -RepoRootFull $crpRoot) -eq '') ''
  # Negative (the BLOCKER cases): outside-repo absolute, and `..`-escape -> $null.
  Assert-True 'CRP-OutsideAbsolute: an absolute path OUTSIDE the repo is rejected ($null)' ($null -eq (Get-ContainedRelPath -RequestedPath 'C:\other\repo\logs\evil.md' -RepoRootFull $crpRoot)) ''
  Assert-True 'CRP-DotDotEscape: a `..`-escape relative path is rejected ($null)' ($null -eq (Get-ContainedRelPath -RequestedPath '..\outside.md' -RepoRootFull $crpRoot)) ''
  Assert-True 'CRP-DotDotEscapeDeep: a multi-segment `..`-escape is rejected ($null)' ($null -eq (Get-ContainedRelPath -RequestedPath 'docs\..\..\outside.md' -RepoRootFull $crpRoot)) ''
  # Sibling-prefix false-positive guard: `C:\repo\proj-other` must NOT count as
  # inside `C:\repo\proj` (the trailing-separator prefix test prevents this).
  Assert-True 'CRP-SiblingPrefix: a sibling dir sharing the root name prefix is rejected ($null)' ($null -eq (Get-ContainedRelPath -RequestedPath 'C:\repo\proj-other\x.md' -RepoRootFull $crpRoot)) ''
  # Case-only sibling: on a case-sensitive checkout `C:\repo\PROJ` is a DIFFERENT dir
  # than `C:\repo\proj`; ordinal containment rejects it (OrdinalIgnoreCase accepted it
  # and sliced it against $repoRoot -> false-clean wrong-file scan -- BLOCKER).
  Assert-True 'CRP-CaseOnlySibling: a path differing from the root only by case is rejected ($null)' ($null -eq (Get-ContainedRelPath -RequestedPath 'C:\repo\PROJ\docs\a.md' -RepoRootFull $crpRoot)) ''
  # Drive-relative: `C:foo` (drive letter + colon, NO separator after it) is "rooted" per
  # IsPathRooted() but GetFullPath() resolves it from the process cwd on drive C:, not the
  # repo root -- rejected so a mispointed `-Paths C:docs/foo.md` cannot pass against the
  # wrong file (Codex BLOCKER).
  Assert-True 'CRP-DriveRelative: a drive-relative Windows path (no separator after the colon) is rejected ($null)' ($null -eq (Get-ContainedRelPath -RequestedPath 'C:docs\foo.md' -RepoRootFull $crpRoot)) ''

  # ---- DEAD-REF ----
  # Resolver/top-level seams: repo has top-level dirs scripts/, logs/, docs/.
  $topSet = @{ 'scripts' = $true; 'logs' = $true; 'docs' = $true; 'crates' = $true }
  $exists = @{ 'scripts/codex/author-lint.ps1' = $true; 'docs/animation-architecture.md' = $true }
  $resolver = { param($p) return $exists.ContainsKey($p) }
  $isTop = { param($s) return $topSet.ContainsKey($s) }

  $drPos = @(New-Rec 11 'cites logs/codex/probe-pack-45ff as evidence')
  $drHits = @(Get-DeadRefFindings -Records $drPos -Resolver $resolver -IsTopLevel $isTop)
  Assert-True 'DR1: unresolved repo-relative path (known top-level seg) is flagged' ($drHits.Count -eq 1) "count=$($drHits.Count)"
  Assert-True 'DR1b: flagged token is the path' ($drHits.Count -eq 1 -and $drHits[0].Match -eq 'logs/codex/probe-pack-45ff') "match=$($drHits[0].Match)"

  $drNeg = @(New-Rec 1 'see scripts/codex/author-lint.ps1 for the linter')
  Assert-True 'DR2: a path present in the index resolver is clean' (@(Get-DeadRefFindings -Records $drNeg -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  # END-TO-END case-sensitivity (uses the REAL builders, exercising the ordinal
  # resolver + case-insensitive top-level asymmetry). A wrong-cased top-level path
  # must reach the resolver (intent gate is case-insensitive) and be flagged
  # DEAD-REF (resolver is ordinal); the correctly-cased path stays clean.
  # (Codex BLOCKER: ordinal top-level set silently dropped this.)
  $ciIdx = Build-PathSetFromFileList -Files @('scripts/codex/x.ps1')
  $ciTop = Get-TopLevelDirSet -IndexFiles @('scripts/codex/x.ps1') -GitignoreText ''
  $ciResolver = { param($p) return $ciIdx.ContainsKey($p) }.GetNewClosure()
  $ciIsTop = { param($s) return $ciTop.ContainsKey($s) }.GetNewClosure()
  $drCaseBad = @(New-Rec 1 'cites Scripts/codex/x.ps1 in the doc')
  Assert-True 'DR2b: a WRONG-CASED top-level path is flagged DEAD-REF end-to-end (ordinal resolver, ci intent gate)' (@(Get-DeadRefFindings -Records $drCaseBad -Resolver $ciResolver -IsTopLevel $ciIsTop).Count -eq 1) ''
  $drCaseOk = @(New-Rec 1 'cites scripts/codex/x.ps1 in the doc')
  Assert-True 'DR2c: the correctly-cased tracked path stays clean end-to-end' (@(Get-DeadRefFindings -Records $drCaseOk -Resolver $ciResolver -IsTopLevel $ciIsTop).Count -eq 0) ''

  $drProse = @(New-Rec 1 'the client/server split and read/write paths')
  Assert-True 'DR3: prose slash-phrase (first seg not a repo entry) is not flagged' (@(Get-DeadRefFindings -Records $drProse -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  $drTilde = @(New-Rec 1 'patch ~/.claude/plugins/cache/local-plugins/agent-orchestrator')
  Assert-True 'DR4: home-relative machine-local target is exempt' (@(Get-DeadRefFindings -Records $drTilde -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  $drDrive = @(New-Rec 1 'the shared C:/cache/build/debug dir')
  Assert-True 'DR5: drive-absolute target is exempt' (@(Get-DeadRefFindings -Records $drDrive -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  $drGlob = @(New-Rec 1 'matches logs/codex/reviews/*.md in the window')
  Assert-True 'DR6: a glob pattern is exempt' (@(Get-DeadRefFindings -Records $drGlob -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # Markdown emphasis / punctuation around a REAL dead path is NOT a glob and must
  # still be flagged. (Codex SILENT-FAILURE.)
  $drBold = @(New-Rec 1 'see **docs/missing-file.md** for the note')
  Assert-True 'DR6b: a path wrapped in Markdown bold (**path**) is still flagged (not a glob)' (@(Get-DeadRefFindings -Records $drBold -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  $drQ = @(New-Rec 1 'did you mean docs/missing-file.md?')
  Assert-True 'DR6c: a path with a trailing rhetorical ? is still flagged (not a glob)' (@(Get-DeadRefFindings -Records $drQ -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  $drQGlob = @(New-Rec 1 'the pattern docs/data?.md sweeps one char')
  Assert-True 'DR6d: a genuine ?-glob single char (docs/data?.md) stays exempt' (@(Get-DeadRefFindings -Records $drQGlob -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # Recursive `**` globstar must stay exempt (authored docs legitimately cite
  # recursive globs like `dir/**/*.ext`). (Codex TEST-QUALITY.)
  $drGlobStar = @(New-Rec 1 'walks docs/art/**/*.png recursively')
  Assert-True 'DR6e: a recursive ** globstar (docs/art/**/...) stays exempt' (@(Get-DeadRefFindings -Records $drGlobStar -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  $drGlobStarEnd = @(New-Rec 1 'the tree docs/art/** holds the set')
  Assert-True 'DR6f: a trailing ** globstar (docs/art/**) stays exempt' (@(Get-DeadRefFindings -Records $drGlobStarEnd -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # Single-* Markdown ITALIC around a dead path must still flag (prevChar is `*`).
  $drItalic = @(New-Rec 1 'see *docs/missing-file.md* for the note')
  Assert-True 'DR6g: a path wrapped in Markdown italic (*path*) is still flagged (not a glob)' (@(Get-DeadRefFindings -Records $drItalic -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  # A genuine single-* glob whose prefix ends mid-token (prefix-*) stays exempt.
  $drPrefixGlob = @(New-Rec 1 'matches logs/codex/probe-pack-* in the dir')
  Assert-True 'DR6h: a prefix-* glob (logs/codex/probe-pack-*) stays exempt' (@(Get-DeadRefFindings -Records $drPrefixGlob -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # Markdown UNDERSCORE emphasis around a dead path must still flag (the start class
  # excludes `_`, so the match begins at the real first segment). (Codex.)
  $drUnder = @(New-Rec 1 'see _docs/missing-file.md_ for the note')
  Assert-True 'DR6i: a path in Markdown italic underscore (_path_) is still flagged' (@(Get-DeadRefFindings -Records $drUnder -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  $drUnder2 = @(New-Rec 1 'see __docs/missing-file.md__ for the note')
  Assert-True 'DR6j: a path in Markdown bold underscore (__path__) is still flagged' (@(Get-DeadRefFindings -Records $drUnder2 -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  # An EXISTING path wrapped in underscore emphasis must resolve (trailing _ stripped)
  # -- not a false DEAD-REF. (Codex QUALITY.)
  $drUnderOk = @(New-Rec 1 'the linter _scripts/codex/author-lint.ps1_ runs first')
  Assert-True 'DR6k: an EXISTING path in underscore emphasis resolves (no false DEAD-REF)' (@(Get-DeadRefFindings -Records $drUnderOk -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # An ABSOLUTE path whose segment begins with `_` must NOT be read as the repo-
  # relative tail: the effective prev char (past the `_`) is the separator `/`.
  # (Codex QUALITY: separator+underscore bypassed the abs guard.)
  $drAbsUnder = @(New-Rec 1 'cached at /_docs/missing-file.md on disk')
  Assert-True 'DR6l: an absolute /_docs/... (separator before underscore) is NOT flagged' (@(Get-DeadRefFindings -Records $drAbsUnder -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # A WORD-flanked underscore (prefix_docs) is an intra-identifier underscore, not
  # emphasis: the trailing _ is not stripped and the firstSeg is prefix_docs (not a
  # repo dir) -> not flagged. (Codex QUALITY: left-flank.)
  $drWordUnder = @(New-Rec 1 'the prefix_docs/missing-file.md_ token here')
  Assert-True 'DR6m: a word-flanked underscore (prefix_docs/...) is not emphasis-stripped into a false finding' (@(Get-DeadRefFindings -Records $drWordUnder -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # An angle-BRACKETED real repo path is flagged (the `<` is stripped, then firstSeg
  # `logs` is a top-level dir and unresolved); a generic `<path/to/file>` placeholder
  # is NOT (firstSeg `path` is not a repo dir). (Codex BLOCKER.)
  $drAngle = @(New-Rec 1 'cites <logs/codex/probe-pack-45ff> as evidence')
  Assert-True 'DR6n: an angle-bracketed real repo path (<logs/...>) is flagged (not blanket-exempt)' (@(Get-DeadRefFindings -Records $drAngle -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  $drPlaceholder = @(New-Rec 1 'replace <path/to/your-file> with the real path')
  Assert-True 'DR6o: a generic <path/to/...> placeholder is NOT flagged (firstSeg not a repo dir)' (@(Get-DeadRefFindings -Records $drPlaceholder -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  # Angle + dot-relative combined: `<./logs/...>` must strip BOTH wrappers (angle
  # first, then ./) so firstSeg is `logs`, not `.`. (Codex SILENT-FAILURE.)
  $drAngleDot = @(New-Rec 1 'see <./logs/codex/probe-pack-45ff> for evidence')
  Assert-True 'DR6p: an angle+dot-relative <./logs/...> is flagged (both wrappers stripped)' (@(Get-DeadRefFindings -Records $drAngleDot -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''

  $drFence = @(New-Rec 1 '```'; New-Rec 2 'logs/codex/does-not-exist'; New-Rec 3 '```')
  Assert-True 'DR7: unresolved path inside a fence is skipped' (@(Get-DeadRefFindings -Records $drFence -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  Assert-True 'DR8: DEAD-REF findings are advisory (calibration outcome)' ($drHits.Count -eq 1 -and $drHits[0].Tier -eq 'ADVISORY') "tier=$($drHits[0].Tier)"

  $drRootFile = @(New-Rec 1 'the PLAN.md/CLAUDE.md/auth/malformed fail-closed set')
  Assert-True 'DR10: root-FILE-seeded slash-list is not a path (first seg not a top-level dir)' (@(Get-DeadRefFindings -Records $drRootFile -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  $drWin = @(New-Rec 1 'cites logs\codex\probe-pack-45ff as evidence')
  $drWinHits = @(Get-DeadRefFindings -Records $drWin -Resolver $resolver -IsTopLevel $isTop)
  Assert-True 'DR11: a Windows-separator unresolved path is flagged (normalized for lookup)' ($drWinHits.Count -eq 1) "count=$($drWinHits.Count)"

  # A path present LOCALLY (gitignored/untracked) but ABSENT from the index
  # resolver must still be flagged -- DEAD-REF resolves against tracked files,
  # not the local filesystem. (Codex BLOCKER.)
  $drLocal = @(New-Rec 1 'see logs/codex/probe-pack-99ff for evidence')
  $localOnlyResolver = { param($p) return $false }   # models index-absent (untracked)
  Assert-True 'DR12: a local-only path absent from the index resolver is flagged' (@(Get-DeadRefFindings -Records $drLocal -Resolver $localOnlyResolver -IsTopLevel $isTop).Count -eq 1) ''

  $drDot = @(New-Rec 1 'see ./logs/codex/probe-pack-45ff for evidence')
  Assert-True 'DR13: a dot-relative path (./logs/..) is flagged after the ./ strip' (@(Get-DeadRefFindings -Records $drDot -Resolver $resolver -IsTopLevel $isTop).Count -eq 1) ''
  $drDrive2 = @(New-Rec 1 'the E:/logs/codex/probe-pack-45ff machine cache')
  Assert-True 'DR14: a drive-absolute path whose tail looks repo-relative is NOT flagged' (@(Get-DeadRefFindings -Records $drDrive2 -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  $drPosix = @(New-Rec 1 'see /logs/codex/x here')
  Assert-True 'DR15: a posix-absolute path is NOT flagged' (@(Get-DeadRefFindings -Records $drPosix -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''
  $drUrl = @(New-Rec 1 'docs at https://host/logs/x online')
  Assert-True 'DR16: a URL path is NOT flagged' (@(Get-DeadRefFindings -Records $drUrl -Resolver $resolver -IsTopLevel $isTop).Count -eq 0) ''

  # ---- MILESTONE-TAG-CLOSURE ----
  $plan = @"
### M1: Engine Bootstrap
#### M1.1: Workspace Setup
### M88.X: Subsystem A
#### M88.X3a: Sub-feature migration
Body references M88.X3a and M1.1 and the unknown M42 milestone.
"@
  $tagHits = @(Get-MilestoneTagFindings -PlanContent $plan)
  Assert-True 'TAG1: a referenced milestone with no defining top-level header is flagged' ($tagHits.Count -eq 1) "count=$($tagHits.Count); $([string]::Join(',', @($tagHits | ForEach-Object { $_.Match })))"
  Assert-True 'TAG1b: the flagged tag is the unknown M42' ($tagHits.Count -eq 1 -and $tagHits[0].Match -eq 'M42') "match=$($tagHits[0].Match)"
  Assert-True 'TAG1c: TAG findings are advisory' ($tagHits.Count -eq 1 -and $tagHits[0].Tier -eq 'ADVISORY') "tier=$($tagHits[0].Tier)"

  $plan2 = @"
### M88: Subsystem
#### M88.X3a: Sub-feature migration
references M88.X3a and M88.9 and M88.X freely
"@
  Assert-True 'TAG2: defined-top-level references (incl. finer suffixes) are clean' (@(Get-MilestoneTagFindings -PlanContent $plan2).Count -eq 0) ''

  # ---- MAGNITUDE ----
  $magPos = @(New-Rec 27 'the chase took 6 rounds and 3 attempts')
  $magHits = @(Get-MagnitudeFindings -Records $magPos)
  Assert-True 'MAG1: exact transient counts are flagged (advisory)' ($magHits.Count -ge 1) "count=$($magHits.Count)"
  Assert-True 'MAG1b: MAGNITUDE findings are advisory' (@($magHits | Where-Object { $_.Tier -ne 'ADVISORY' }).Count -eq 0) ''

  $magNeg = @(New-Rec 1 'the chase took many rounds of high churn')
  Assert-True 'MAG2: magnitude phrasing is clean' (@(Get-MagnitudeFindings -Records $magNeg).Count -eq 0) ''

  $magHandle = @(New-Rec 1 'landed at 06595181 on 2026-06-12 for item-0311 on libfoo 0.15')
  Assert-True 'MAG3: stable handles (sha/date/id/version) are not flagged' (@(Get-MagnitudeFindings -Records $magHandle).Count -eq 0) ''

  $magFence = @(New-Rec 1 '```'; New-Rec 2 'ran 5 passes'; New-Rec 3 '```')
  Assert-True 'MAG4: counts inside a fence are skipped' (@(Get-MagnitudeFindings -Records $magFence).Count -eq 0) ''

  $magSingular = @(New-Rec 27 'it took 1 pass and 1 retry to land')
  $magSingHits = @(Get-MagnitudeFindings -Records $magSingular)
  Assert-True 'MAG5: singular pass/retry counts are flagged (not only plurals)' ((@($magSingHits | Where-Object { $_.Match -match '(?i)pass' }).Count -ge 1) -and (@($magSingHits | Where-Object { $_.Match -match '(?i)retry' }).Count -ge 1)) "matches=$([string]::Join('|', @($magSingHits | ForEach-Object { $_.Match })))"
  $magDup = @(Get-MagnitudeFindings -Records @(New-Rec 1 'it took 6 rounds'))
  Assert-True 'MAG6: "took N <unit>" is not double-counted (one finding, not took-N + N-unit)' (@($magDup).Count -eq 1) "count=$(@($magDup).Count); $([string]::Join('|', @($magDup | ForEach-Object { $_.Match })))"
  $magUF = @(Get-MagnitudeFindings -Records @(New-Rec 1 'fixed in round 3 after attempt 2'))
  Assert-True 'MAG7: unit-first counts ("round 3", "attempt 2") are flagged' ((@($magUF | Where-Object { $_.Match -match '(?i)round\s+3' }).Count -ge 1) -and (@($magUF | Where-Object { $_.Match -match '(?i)attempt\s+2' }).Count -ge 1)) "matches=$([string]::Join('|', @($magUF | ForEach-Object { $_.Match })))"
  $magUFneg = @(Get-MagnitudeFindings -Records @(New-Rec 1 'the bypass 2 subround 3 path stays clean'))
  Assert-True 'MAG8: a unit embedded in an identifier ("bypass 2", "subround 3") does not false-match' (@($magUFneg).Count -eq 0) "matches=$([string]::Join('|', @($magUFneg | ForEach-Object { $_.Match })))"
  # Same number, DISJOINT spans -> span-overlap dedup must keep BOTH the digit-first
  # unit count and the verb-form count (the old number-based dedup dropped one).
  $magDisjoint = @(Get-MagnitudeFindings -Records @(New-Rec 1 '6 rounds in; the last fix took 6 to verify'))
  Assert-True 'MAG9: a non-overlapping same-number "N unit ... took N" keeps both findings' (@($magDisjoint).Count -eq 2) "count=$(@($magDisjoint).Count); $([string]::Join('|', @($magDisjoint | ForEach-Object { $_.Match })))"
  # Aggregate/duration nouns (digit-first only): the unit-agnostic forms that
  # leaked past the original process-event list.
  $magAgg = @(Get-MagnitudeFindings -Records @(New-Rec 1 'an 18-task chain across 2 rows with 47 assertions in 6 minutes'))
  Assert-True 'MAG10: aggregate/duration digit-first counts ("18-task", "2 rows", "47 assertions", "6 minutes") are flagged' (@($magAgg).Count -eq 4) "count=$(@($magAgg).Count); $([string]::Join('|', @($magAgg | ForEach-Object { $_.Match })))"
  $magAgg2 = @(Get-MagnitudeFindings -Records @(New-Rec 1 'left 3 follow-ups and 2 sites unswept'))
  Assert-True 'MAG11: follow-up/site counts are flagged' (@($magAgg2).Count -eq 2) "count=$(@($magAgg2).Count); $([string]::Join('|', @($magAgg2 | ForEach-Object { $_.Match })))"
  # Unit-first NAMES for the aggregate nouns must NOT match: "Wave 2" / "day 3" /
  # "row 7" are stable handles (names/coordinates), not transient counts.
  $magAggNeg = @(Get-MagnitudeFindings -Records @(New-Rec 1 'Wave 2 landed; see day 3 notes and row 7 of the table'))
  Assert-True 'MAG12: unit-first aggregate names ("Wave 2", "day 3", "row 7") are not flagged' (@($magAggNeg).Count -eq 0) "matches=$([string]::Join('|', @($magAggNeg | ForEach-Object { $_.Match })))"
  # Identifier-embedded digits stay excluded for the new nouns too.
  $magAggNeg2 = @(Get-MagnitudeFindings -Records @(New-Rec 1 'item-0303a1 and M19-9 files stay clean'))
  Assert-True 'MAG13: identifier digit runs before the new nouns do not false-match' (@($magAggNeg2).Count -eq 0) "matches=$([string]::Join('|', @($magAggNeg2 | ForEach-Object { $_.Match })))"
  # Dot-preceded digit runs inside HANDLES: dotted milestone/version handles
  # must not shed their prefix and false-match ("M19.9 tasks" is a handle +
  # noun, not a 9-task count; "v1.2 rows" likewise).
  $magAggNeg3 = @(Get-MagnitudeFindings -Records @(New-Rec 1 'M19.9 tasks resolved per v1.2 rows'))
  Assert-True 'MAG14: dotted-handle digits do not false-match' (@($magAggNeg3).Count -eq 0) "matches=$([string]::Join('|', @($magAggNeg3 | ForEach-Object { $_.Match })))"
  # A FREE-STANDING decimal duration is still an exact transient measurement:
  # matched in full from its first digit, never as a fraction fragment.
  $magDec = @(Get-MagnitudeFindings -Records @(New-Rec 1 'the conversion ran in 0.25 seconds flat'))
  Assert-True 'MAG15: a free-standing decimal duration ("0.25 seconds") is one full-decimal finding' ((@($magDec).Count -eq 1) -and ($magDec[0].Match -match '0\.25\s+seconds')) "count=$(@($magDec).Count); matches=$([string]::Join('|', @($magDec | ForEach-Object { $_.Match })))"
  # Verb-form decimal on a NO-UNIT fixture: with a trailing unit word the
  # digit-first form matches first and overlap suppression discards the
  # $rxTook hit, so only a unit-less decimal actually exercises $rxTook's
  # (\.\d+)? path. Matched IN FULL ("took 0.25"), never as the fragment
  # "took 0" -- a took-decimal is still an exact transient measurement, so
  # it stays flagged; only the fragmenting was the defect.
  $magTookDec = @(Get-MagnitudeFindings -Records @(New-Rec 1 'the verify step took 0.25 to run'))
  Assert-True 'MAG16: unit-less "took 0.25" yields one full-decimal verb-form finding (no "took 0" fragment)' ((@($magTookDec).Count -eq 1) -and ($magTookDec[0].Match -match 'took\s+0\.25')) "count=$(@($magTookDec).Count); matches=$([string]::Join('|', @($magTookDec | ForEach-Object { $_.Match })))"

  # ---- MAGNITUDE: enumerated-case counting (spelled-out totals) ----
  # The spelled-out sibling of the digit-count forms; same MAGNITUDE family +
  # ADVISORY tier, distinct enumerated-case Hint.
  $magEnum = @(Get-MagnitudeFindings -Records @(New-Rec 5 'the fix passed all three cases cleanly'))
  Assert-True 'MAG17: "all three cases" (spelled total) is flagged (enumerated-case, advisory)' ((@($magEnum).Count -eq 1) -and ($magEnum[0].Tier -eq 'ADVISORY') -and ($magEnum[0].Match -match '(?i)all three cases') -and ($magEnum[0].Hint -match 'enumerated total')) "count=$(@($magEnum).Count); matches=$([string]::Join('|', @($magEnum | ForEach-Object { $_.Match })))"
  $magBoth = @(Get-MagnitudeFindings -Records @(New-Rec 1 'both cases now share the guard'))
  Assert-True 'MAG18: "both cases" is flagged' ((@($magBoth).Count -eq 1) -and ($magBoth[0].Match -match '(?i)both cases')) "matches=$([string]::Join('|', @($magBoth | ForEach-Object { $_.Match })))"
  # The magnitude REWRITE itself ("all cases" / "every case") must stay clean.
  $magEnumNeg = @(Get-MagnitudeFindings -Records @(New-Rec 1 'the fix now covers all cases and every case alike'))
  Assert-True 'MAG19: the magnitude rewrite ("all cases"/"every case") is NOT flagged' (@($magEnumNeg).Count -eq 0) "matches=$([string]::Join('|', @($magEnumNeg | ForEach-Object { $_.Match })))"
  # Form (b): a (1)(2)(3) inline enumeration paired with a nounless "all N" total.
  $magParen = @(Get-MagnitudeFindings -Records @(New-Rec 1 'the checks (1) alpha (2) beta (3) gamma all three still fail'))
  Assert-True 'MAG20: an inline (1)(2)(3) enumeration paired with a nounless "all three" total is flagged (form b)' ((@($magParen).Count -eq 1) -and ($magParen[0].Match -match '(?i)all three')) "count=$(@($magParen).Count); matches=$([string]::Join('|', @($magParen | ForEach-Object { $_.Match })))"
  # A bare inline enumeration WITHOUT an "all N" total is not the anti-pattern.
  $magParenNeg = @(Get-MagnitudeFindings -Records @(New-Rec 1 'run the steps (1) build (2) test in that order'))
  Assert-True 'MAG21: an inline enumeration with NO "all N" total is clean (no redundant total)' (@($magParenNeg).Count -eq 0) "matches=$([string]::Join('|', @($magParenNeg | ForEach-Object { $_.Match })))"
  # A bounded domain set-noun outside the enumeration-noun list ("all seven
  # regions") is a NAMED set, not a transient enumeration -> not flagged.
  $magDomain = @(Get-MagnitudeFindings -Records @(New-Rec 1 'all seven regions exist today'))
  Assert-True 'MAG22: a bounded domain set-noun ("all seven regions") is not an enumerated-case count' (@($magDomain).Count -eq 0) "matches=$([string]::Join('|', @($magDomain | ForEach-Object { $_.Match })))"
  # A count inside a fence is skipped (shared fence context with the count forms).
  $magEnumFence = @(Get-MagnitudeFindings -Records @(New-Rec 1 '```'; New-Rec 2 'all three cases'; New-Rec 3 '```'))
  Assert-True 'MAG23: an enumerated-case total inside a fence is skipped' (@($magEnumFence).Count -eq 0) ''

  # ---- SELF-NARRATION (Get-SelfNarrationFindings) ----
  # ERROR in History.md (the retrospective landing log), ADVISORY in any other
  # narrative doc. The signature is the deictic "this/here + landing-event" form;
  # retrospective prose about an ALREADY-landed merge stays clean. Phrase set
  # calibrated so only genuine self-narration fires at ERROR.
  $snMerge = @(Get-SelfNarrationFindings -Records @(New-Rec 5 'merge_count 0 -> 1 after this merge') -IsHistoryDoc $true)
  Assert-True 'SN1: "this merge" in History.md is flagged ERROR' ((@($snMerge).Count -eq 1) -and ($snMerge[0].Tier -eq 'ERROR') -and ($snMerge[0].Match -eq 'this merge')) "count=$(@($snMerge).Count); tier=$($snMerge[0].Tier); match=$($snMerge[0].Match)"
  $snLanding = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the pre-fix behavior surfaced during this landing''s own merge runs') -IsHistoryDoc $true)
  Assert-True 'SN2: "this landing" (incl. the "this landing''s" possessive) is flagged' ((@($snLanding).Count -eq 1) -and ($snLanding[0].Match -eq 'this landing')) "matches=$([string]::Join('|', @($snLanding | ForEach-Object { $_.Match })))"
  $snAsOf = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'as of this entry the follow-up work is open') -IsHistoryDoc $true)
  Assert-True 'SN3: "as of this entry" is flagged (documented entry-deixis incident)' (@($snAsOf).Count -eq 1 -and $snAsOf[0].Match -match '(?i)as of this entry') ''
  $snAdd = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the documentation commits that add this entry ride the branch') -IsHistoryDoc $true)
  Assert-True 'SN4: "add this entry" (commit narrating its own entry) is flagged' (@($snAdd).Count -eq 1 -and $snAdd[0].Match -match '(?i)add this entry') ''
  $snHere = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the fix landed through the gate here today') -IsHistoryDoc $true)
  Assert-True 'SN5: "landed through the gate here" is flagged (landed...here word window)' (@($snHere).Count -eq 1 -and $snHere[0].Match -match '(?i)landed through the gate here') "matches=$([string]::Join('|', @($snHere | ForEach-Object { $_.Match })))"
  $snHere2 = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the change landed here after review') -IsHistoryDoc $true)
  Assert-True 'SN5b: the direct "landed here" form is flagged' (@($snHere2).Count -eq 1) ''
  # TIER: the SAME phrase in a NON-History doc is ADVISORY, not ERROR.
  $snAdv = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'notes about this merge') -IsHistoryDoc $false)
  Assert-True 'SN6: self-narration in a NON-History doc is ADVISORY (tier depends on the doc)' ((@($snAdv).Count -eq 1) -and ($snAdv[0].Tier -eq 'ADVISORY')) "tier=$($snAdv[0].Tier)"
  # NEGATIVES (near-miss legitimate forms that must stay clean):
  $snNegRetro = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the merge landed at 06595181 after review') -IsHistoryDoc $true)
  Assert-True 'SN7: retrospective "the merge landed at <sha>" (no this/here deictic) is CLEAN' (@($snNegRetro).Count -eq 0) "matches=$([string]::Join('|', @($snNegRetro | ForEach-Object { $_.Match })))"
  $snNegTime = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'still open at the time of this entry: the asset approach') -IsHistoryDoc $true)
  Assert-True 'SN8: legitimate timestamping "at the time of this entry" (bare this-entry, not as-of/add) is CLEAN' (@($snNegTime).Count -eq 0) "matches=$([string]::Join('|', @($snNegTime | ForEach-Object { $_.Match })))"
  $snNegMerged = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the batch merged cleanly onto master') -IsHistoryDoc $true)
  Assert-True 'SN9: "merged cleanly" (no this/here deictic) is CLEAN' (@($snNegMerged).Count -eq 0) ''
  $snNegLanded = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'the feature landed on master last cycle') -IsHistoryDoc $true)
  Assert-True 'SN10: "landed on master" (no "here") is CLEAN (landed...here needs the deictic)' (@($snNegLanded).Count -eq 0) ''
  $snFence = @(Get-SelfNarrationFindings -Records @(New-Rec 1 '```'; New-Rec 2 'after this merge'; New-Rec 3 '```') -IsHistoryDoc $true)
  Assert-True 'SN11: a self-narration phrase inside a fenced block is skipped' (@($snFence).Count -eq 0) ''
  # Two DISTINCT phrases on one line -> two findings; a repeated phrase dedups.
  $snMulti = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'this merge and this landing both count') -IsHistoryDoc $true)
  Assert-True 'SN12: two distinct self-narration phrases on one line yield two findings' (@($snMulti).Count -eq 2) "count=$(@($snMulti).Count); matches=$([string]::Join('|', @($snMulti | ForEach-Object { $_.Match })))"
  $snDup = @(Get-SelfNarrationFindings -Records @(New-Rec 1 'this merge is this merge again') -IsHistoryDoc $true)
  Assert-True 'SN13: a repeated identical phrase on one line dedups to one finding' (@($snDup).Count -eq 1) "count=$(@($snDup).Count)"

  # ---- LOCAL-EVIDENCE-AS-PROOF (Get-LocalEvidenceFindings) ----
  # ERROR tight = proof word + the gitignored review-artifact PATH; ADVISORY =
  # proof word + a bare local marker (calibrated down -- common git/meta prose),
  # or the loose "operator-side/local + evidence". Word-bounded proof words.
  $lpTight = @(Get-LocalEvidenceFindings -Records @(New-Rec 3 'the guard is verified via logs/codex/reviews/review-x.md today'))
  Assert-True 'LP1: proof word + the logs/codex/reviews PATH is flagged ERROR (tight form)' ((@($lpTight).Count -eq 1) -and ($lpTight[0].Tier -eq 'ERROR') -and ($lpTight[0].Line -eq 3)) "count=$(@($lpTight).Count); tier=$($lpTight[0].Tier)"
  $lpClaude = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'this proves the fix per the logs/claude/reviews output'))
  Assert-True 'LP1b: the claude review path (logs/claude/reviews) also triggers the tight ERROR' ((@($lpClaude).Count -eq 1) -and ($lpClaude[0].Tier -eq 'ERROR')) "tier=$($lpClaude[0].Tier)"
  # WINDOWS separators must trigger the tight ERROR too (this Windows-first repo
  # writes `logs\codex\reviews`). (Codex BLOCKER.)
  $lpWin = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'the guard is verified via logs\codex\reviews\review-x.md'))
  Assert-True 'LP1c: a WINDOWS-separator review path (logs\codex\reviews\...) triggers the tight ERROR' ((@($lpWin).Count -eq 1) -and ($lpWin[0].Tier -eq 'ERROR')) "count=$(@($lpWin).Count); tier=$($lpWin[0].Tier)"
  # Proof-word INFLECTIONS -- verifying/proven/proved/verifiable -- also trip the
  # tight ERROR. (Codex BLOCKER: the original set omitted them.)
  Assert-True 'LP1d: the "verifying" inflection + review path triggers the tight ERROR' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'verifying the fix via logs/codex/reviews/review-x.md') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 1) ''
  Assert-True 'LP1e: the "proven" inflection + review path triggers the tight ERROR' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'this is proven by the logs/claude/reviews output') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 1) ''
  Assert-True 'LP1f: the "proved" inflection + review path triggers the tight ERROR' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'the guard proved correct in logs/codex/reviews/x.md') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 1) ''
  Assert-True 'LP1g: the "verifiable" inflection + review path triggers the tight ERROR' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'this is verifiable in logs/codex/reviews/x.md today') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 1) ''
  # NOUN-form claims: "proof"/"evidence" WITHIN the proximity window of the review
  # path is the tight ERROR too. (Codex BLOCKER: "Proof: <path>".)
  Assert-True 'LP1h: a "Proof:" label citing the review path is the tight ERROR (noun form)' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'Proof: logs/codex/reviews/review-x.md') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 1) ''
  Assert-True 'LP1i: "the evidence is <review path>" is the tight ERROR (noun form, proximate)' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'the evidence is logs/claude/reviews/x.md') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 1) ''
  # NEGATIVE: an "evidence" noun FAR from the path (a description, not a citation) is
  # NOT the tight ERROR -- the proximity gate spares legitimate infra prose that
  # merely co-occurs (e.g. the wrapper's "evidence snapshot ... logs/codex/reviews").
  Assert-True 'LP1j: an "evidence" noun FAR from the review path (description) is NOT the tight ERROR' (@(Get-LocalEvidenceFindings -Records @(New-Rec 1 'builds the isolated evidence snapshot, then writes the verdict to logs/codex/reviews') | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 0) ''
  $lpAdj = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'the verification of the gitignored artifact ran'))
  Assert-True 'LP2: proof word + a bare "gitignored" marker is ADVISORY (calibrated down)' ((@($lpAdj).Count -eq 1) -and ($lpAdj[0].Tier -eq 'ADVISORY')) "tier=$($lpAdj[0].Tier)"
  $lpUntracked = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'verify the untracked files are gone before commit'))
  Assert-True 'LP3: proof word + "untracked" is ADVISORY, NOT error (common git prose stays non-blocking)' ((@($lpUntracked).Count -eq 1) -and ($lpUntracked[0].Tier -eq 'ADVISORY')) "tier=$($lpUntracked[0].Tier)"
  $lpMeta = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'gitignored logs cannot prove provenance on their own'))
  Assert-True 'LP4: a line DESCRIBING the rule ("gitignored logs cannot prove provenance") is ADVISORY, not ERROR' ((@($lpMeta).Count -eq 1) -and ($lpMeta[0].Tier -eq 'ADVISORY')) "tier=$($lpMeta[0].Tier)"
  $lpLoose = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'operator-side evidence shows the abort fired'))
  Assert-True 'LP5: "operator-side" + "evidence" is the loose ADVISORY form' ((@($lpLoose).Count -eq 1) -and ($lpLoose[0].Tier -eq 'ADVISORY')) "tier=$($lpLoose[0].Tier)"
  # NEGATIVES:
  $lpApprove = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'approve the gitignored settings file'))
  Assert-True 'LP6: "approve" (merely CONTAINS "prove", not word-bounded) + gitignored is CLEAN' (@($lpApprove).Count -eq 0) "matches=$([string]::Join('|', @($lpApprove | ForEach-Object { $_.Match })))"
  $lpNoMark = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'the change is verified by the tracked test suite'))
  Assert-True 'LP7: a proof word with NO local marker (verified by the tracked test) is CLEAN' (@($lpNoMark).Count -eq 0) ''
  $lpBareLogs = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'verified the logs are clean at startup'))
  Assert-True 'LP8: a bare "logs" mention (not the reviews PATH) with a proof word is CLEAN' (@($lpBareLogs).Count -eq 0) ''
  $lpEvidenceOnly = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 'the evidence lives in the tracked test suite'))
  Assert-True 'LP9: "evidence" with no local/operator-side qualifier is CLEAN' (@($lpEvidenceOnly).Count -eq 0) ''
  $lpFence = @(Get-LocalEvidenceFindings -Records @(New-Rec 1 '```'; New-Rec 2 'verified via logs/codex/reviews/x.md'; New-Rec 3 '```'))
  Assert-True 'LP10: a tight-form line inside a fenced block is skipped' (@($lpFence).Count -eq 0) ''

  # ---- MACHINE-LOCAL-STATE (Get-MachineLocalStateFindings) ----
  # ADVISORY: an install/wiring claim on a machine-local config path, UNLESS the
  # line already flags it machine-local.
  $lsWired = @(Get-MachineLocalStateFindings -Records @(New-Rec 1 'the deny hook is wired via .claude/settings.local.json'))
  Assert-True 'LS1: "wired via .claude/settings.local.json" (no caveat) is ADVISORY' ((@($lsWired).Count -eq 1) -and ($lsWired[0].Tier -eq 'ADVISORY')) "count=$(@($lsWired).Count); tier=$($lsWired[0].Tier)"
  # WINDOWS separators are recognized too (`~\.claude`, `.claude\settings.local.json`).
  # (Codex BLOCKER.)
  $lsWin = @(Get-MachineLocalStateFindings -Records @(New-Rec 1 'the hook is wired via .claude\settings.local.json now'))
  Assert-True 'LS1c: a WINDOWS-separator machine-local path is flagged' (@($lsWin).Count -eq 1) "count=$(@($lsWin).Count)"
  $lsInstalled = @(Get-MachineLocalStateFindings -Records @(New-Rec 1 'the heartbeat rewrite is installed and wired under ~/.claude/hooks now'))
  Assert-True 'LS2: "installed and wired ... ~/.claude" is flagged' (@($lsInstalled).Count -eq 1) ''
  # NEGATIVE: a "machine-local" caveat on the line exempts it (how the hook docs
  # correctly phrase it -- surface only UN-caveated assertions).
  $lsCaveat = @(Get-MachineLocalStateFindings -Records @(New-Rec 1 'a machine-local hook wired via .claude/settings.local.json'))
  Assert-True 'LS3: a "machine-local" caveat on the line exempts it (correctly-caveated form)' (@($lsCaveat).Count -eq 0) "matches=$([string]::Join('|', @($lsCaveat | ForEach-Object { $_.Match })))"
  # NEGATIVE: a wiring claim with NO machine-local path is not this class.
  $lsNoPath = @(Get-MachineLocalStateFindings -Records @(New-Rec 1 'the converter tool is installed in tools/'))
  Assert-True 'LS4: an install claim with no machine-local path is CLEAN' (@($lsNoPath).Count -eq 0) ''
  $lsFence = @(Get-MachineLocalStateFindings -Records @(New-Rec 1 '```'; New-Rec 2 'wired via .claude/settings.local.json'; New-Rec 3 '```'))
  Assert-True 'LS5: a machine-local-state line inside a fenced block is skipped' (@($lsFence).Count -eq 0) ''

  # ---- INVENTORY-ASSERTION SHAPE (Get-InventoryAssertionFindings) ----
  # ERROR in .md prose; noun-agnostic shape (verb-gated). No frozen downgrade
  # (line-anchor-scoped marker only -- INV4); inline-code + fence exemptions,
  # digit lookbehind. (The source-comment code-echo exemption belongs to
  # Get-StagedSourceFindings -- see SRC10*.)
  $invThere = @(Get-InventoryAssertionFindings -Records @(New-Rec 5 'there are 223 icons for this feature'))
  Assert-True 'INV1: "there are 223 icons" is flagged ERROR (shape 1)' ((@($invThere).Count -eq 1) -and ($invThere[0].Tier -eq 'ERROR') -and ($invThere[0].Match -match '(?i)there are 223 icons')) "count=$(@($invThere).Count); tier=$($invThere[0].Tier); match=$($invThere[0].Match)"
  $invState = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'exactly 12 handlers are registered in the allowlist'))
  Assert-True 'INV2: "12 handlers are registered" is flagged (shape 2)' ((@($invState).Count -eq 1) -and ($invState[0].Match -match '(?i)12 handlers are registered')) "matches=$([string]::Join('|', @($invState | ForEach-Object { $_.Match })))"
  $invCovers = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'the enum covers 7 formats today'))
  Assert-True 'INV3: "covers 7 formats" is flagged (shape 3)' ((@($invCovers).Count -eq 1) -and ($invCovers[0].Match -match '(?i)covers 7 formats')) "matches=$([string]::Join('|', @($invCovers | ForEach-Object { $_.Match })))"
  $invContains = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'the dir contains 96 fixture files total'))
  Assert-True 'INV3b: "contains 96 fixture files" is flagged (shape 3, contains)' (@($invContains).Count -ge 1) "matches=$([string]::Join('|', @($invContains | ForEach-Object { $_.Match })))"
  # The frozen-snapshot MARKER grants NO inventory downgrade -- the marker
  # exemption is scoped to LINE ANCHORS (the symbol-anchor rule), so the detector
  # takes no frozen input at all and an exact count stays ERROR even in a
  # frozen-marked doc. (Codex BLOCKER: the prior ADVISORY downgrade let a staged
  # .md inventory count bypass the error-tier pre-pass.)
  $invFrozen = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'there are 3 icons historically'))
  Assert-True 'INV4: inventory stays ERROR regardless of frozen-snapshot marker (no downgrade path exists)' ((@($invFrozen).Count -eq 1) -and ($invFrozen[0].Tier -eq 'ERROR')) "tier=$($invFrozen[0].Tier)"
  # NEGATIVES:
  $invVer = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'there are v2 items in the queue'))
  Assert-True 'INV5: a version-prefixed token ("there are v2 items") is CLEAN (no cardinal after the verb)' (@($invVer).Count -eq 0) "matches=$([string]::Join('|', @($invVer | ForEach-Object { $_.Match })))"
  $invId = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'the item-0003 rows are tracked upstream'))
  Assert-True 'INV5b: an identifier digit ("item-0003 rows are tracked") is CLEAN (lookbehind excludes it)' (@($invId).Count -eq 0) "matches=$([string]::Join('|', @($invId | ForEach-Object { $_.Match })))"
  $invMs = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'the M19.9 systems are defined already'))
  Assert-True 'INV5c: a milestone-handle digit ("M19.9 systems are defined") is CLEAN (dot/word-preceded digits excluded)' (@($invMs).Count -eq 0) "matches=$([string]::Join('|', @($invMs | ForEach-Object { $_.Match })))"
  $invExit = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'the tool returns exit 3 on a bad input set'))
  Assert-True 'INV5d: an exit code ("returns exit 3") is CLEAN (no shape verb)' (@($invExit).Count -eq 0) "matches=$([string]::Join('|', @($invExit | ForEach-Object { $_.Match })))"
  $invMag = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'there are many icons in the set'))
  Assert-True 'INV5e: magnitude phrasing ("there are many icons") is CLEAN' (@($invMag).Count -eq 0) ''
  # Inline-code exemption: a WHOLE claim in one backtick span is an illustrative
  # example -> exempt (so a doc can name the class).
  $invBacktick = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'a bad claim like `there are 223 icons` in a doc'))
  Assert-True 'INV6: a FULL inline-code example (`there are 223 icons` in one span) is CLEAN (illustrative)' (@($invBacktick).Count -eq 0) "matches=$([string]::Join('|', @($invBacktick | ForEach-Object { $_.Match })))"
  # Number-ONLY backticked -> the surrounding prose is a REAL claim -> STILL flag.
  # (Codex BLOCKER: blanking every span false-cleaned this form.)
  $invPartial = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'there are `223` icons here'))
  Assert-True 'INV6b: a NUMBER-ONLY backticked claim ("there are `223` icons") STILL flags (real prose claim, not a full example)' ((@($invPartial).Count -eq 1) -and ($invPartial[0].Match -match '(?i)there are 223 icons')) "count=$(@($invPartial).Count); matches=$([string]::Join('|', @($invPartial | ForEach-Object { $_.Match })))"
  # A REAL prose claim (no backticks) is still flagged -- inline-code strip does not over-exempt.
  $invReal = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'there are 223 icons and a `code` span nearby'))
  Assert-True 'INV6c: a real claim on a line that ALSO has an unrelated `code` span is still flagged' (@($invReal).Count -eq 1) "matches=$([string]::Join('|', @($invReal | ForEach-Object { $_.Match })))"
  $invFence = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 '```'; New-Rec 2 'there are 223 icons'; New-Rec 3 '```'))
  Assert-True 'INV7: an inventory claim inside a fenced block is skipped' (@($invFence).Count -eq 0) ''
  # (The source-comment code-echo exemption is tested end-to-end via SRC10/SRC10b.)
  # Two distinct claims on one line -> two findings.
  $invMulti = @(Get-InventoryAssertionFindings -Records @(New-Rec 1 'there are 3 cats and the set covers 5 dogs'))
  Assert-True 'INV9: two distinct inventory claims on one line yield two findings' (@($invMulti).Count -eq 2) "count=$(@($invMulti).Count); matches=$([string]::Join('|', @($invMulti | ForEach-Object { $_.Match })))"

  # ---- SOURCE-COMMENT extraction + staged decision (Get-SourceLineParts / Get-StagedSourceFindings) ----
  # Rust: full-line, trailing, string-guarded, and block comments.
  $slpFull = Get-SourceLineParts -Line '// there are 223 icons' -Ext 'rs' -InBlock $false
  Assert-True 'SRC1: a rs full-line // comment extracts its text' ($slpFull.Comment -match '(?i)there are 223 icons') "comment=$($slpFull.Comment)"
  $slpTrail = Get-SourceLineParts -Line 'let x = 5; // note the 3 flags' -Ext 'rs' -InBlock $false
  Assert-True 'SRC2: a rs trailing // comment splits code from comment' (($slpTrail.Comment -match 'note the 3 flags') -and ($slpTrail.Code -match 'let x = 5')) "code=$($slpTrail.Code); comment=$($slpTrail.Comment)"
  $slpStr = Get-SourceLineParts -Line 'let u = "http://host/x";' -Ext 'rs' -InBlock $false
  Assert-True 'SRC3: a // inside a rs string is CODE, not a comment (no false comment)' ([string]::IsNullOrWhiteSpace($slpStr.Comment)) "comment=$($slpStr.Comment)"
  # PowerShell # comment + a # inside a string is code.
  $slpPs = Get-SourceLineParts -Line '$x = 1  # there are 5 configs' -Ext 'ps1' -InBlock $false
  Assert-True 'SRC4: a ps1 # comment extracts its text' ($slpPs.Comment -match '(?i)there are 5 configs') "comment=$($slpPs.Comment)"
  $slpPsStr = Get-SourceLineParts -Line '$h = "a#b#c"' -Ext 'ps1' -InBlock $false
  Assert-True 'SRC4b: a # inside a ps1 string is CODE, not a comment' ([string]::IsNullOrWhiteSpace($slpPsStr.Comment)) "comment=$($slpPsStr.Comment)"
  # sh: `$#` (no whitespace before #) is not a comment.
  $slpSh = Get-SourceLineParts -Line 'echo $# args' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5: sh "$#" (no whitespace before #) is NOT a comment (code)' ([string]::IsNullOrWhiteSpace($slpSh.Comment)) "comment=$($slpSh.Comment)"
  $slpShC = Get-SourceLineParts -Line 'run.sh   # there are 4 steps' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5b: sh " # ..." (whitespace before #) IS a comment' ($slpShC.Comment -match '(?i)there are 4 steps') "comment=$($slpShC.Comment)"
  # A `#` directly after a command separator (no whitespace) IS a comment in
  # sh/ps1 -- a separator starts a new word; `$#` (SRC5) and word-embedded
  # `a#b` stay code. (Codex BLOCKER: the whitespace-only rule misclassified
  # `cmd;# note` lines as code-only.)
  $slpShSep = Get-SourceLineParts -Line 'echo hi;# there are 5 steps' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5c: sh "cmd;#..." (separator-adjacent, no whitespace) IS a comment' ($slpShSep.Comment -match '(?i)there are 5 steps') "comment=$($slpShSep.Comment)"
  $slpPsSep = Get-SourceLineParts -Line '$x = 1;# there are 5 configs' -Ext 'ps1' -InBlock $false
  Assert-True 'SRC5d: ps1 "cmd;#..." (separator-adjacent, no whitespace) IS a comment' ($slpPsSep.Comment -match '(?i)there are 5 configs') "comment=$($slpPsSep.Comment)"
  $slpShWord = Get-SourceLineParts -Line 'echo a#b' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5e: sh word-embedded "a#b" stays CODE (not a comment)' ([string]::IsNullOrWhiteSpace($slpShWord.Comment)) "comment=$($slpShWord.Comment)"
  # sh EXPANSION syntax before `#` is CODE -- the sh separator set deliberately
  # excludes `(){}` (`${#name}` length, `${name}#suffix` strip, `$(cmd)#suffix`).
  # This is sh-SPECIFIC: ps1 has no such expansion forms and treats any `#`
  # outside strings/block-comments/`${...}` as a comment (SRC5i/SRC5j/SRC5k).
  # (Codex QUALITY: a shared separator class false-split sh expansions into
  # comment text.)
  $slpShLen = Get-SourceLineParts -Line 'echo ${#name}' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5f: sh "${#name}" (length expansion) stays CODE' ([string]::IsNullOrWhiteSpace($slpShLen.Comment)) "comment=$($slpShLen.Comment)"
  $slpShStrip = Get-SourceLineParts -Line 'echo ${name}#suffix' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5g: sh "${name}#suffix" stays CODE' ([string]::IsNullOrWhiteSpace($slpShStrip.Comment)) "comment=$($slpShStrip.Comment)"
  $slpShSub = Get-SourceLineParts -Line 'x=$(date)#suffix' -Ext 'sh' -InBlock $false
  Assert-True 'SRC5h: sh "$(cmd)#suffix" stays CODE' ([string]::IsNullOrWhiteSpace($slpShSub.Comment)) "comment=$($slpShSub.Comment)"
  $slpPsParen = Get-SourceLineParts -Line '$y = $(Get-Date)# there are 5 configs' -Ext 'ps1' -InBlock $false
  Assert-True 'SRC5i: ps1 "$(cmd)#..." IS a comment' ($slpPsParen.Comment -match '(?i)there are 5 configs') "comment=$($slpPsParen.Comment)"
  # ps1 starts a comment at a `#` after ANY completed token -- `$x=1# ...` is a
  # comment (PSParser: a number cannot absorb `#`). The prior separator-adjacency
  # rule missed this token-adjacent form. (Codex QUALITY review.)
  $slpPsTok = Get-SourceLineParts -Line '$x=1# there are 5 configs' -Ext 'ps1' -InBlock $false
  Assert-True 'SRC5j: ps1 token-adjacent "$x=1#..." (no separator before #) IS a comment' ($slpPsTok.Comment -match '(?i)there are 5 configs') "comment=$($slpPsTok.Comment); code=$($slpPsTok.Code)"
  # A `#` INSIDE a ps1 `${...}` braced-variable name is a NAME char, NOT a comment
  # (`${a#b}` is a variable per PSParser) -- the $inPsBraceVar guard keeps it code.
  $slpPsBrace = Get-SourceLineParts -Line '${a#b} = 1' -Ext 'ps1' -InBlock $false
  Assert-True 'SRC5k: ps1 "${a#b}" (# inside a braced variable name) stays CODE (not a comment)' ([string]::IsNullOrWhiteSpace($slpPsBrace.Comment)) "comment=$($slpPsBrace.Comment); code=$($slpPsBrace.Code)"
  # toml: `#` outside a string is a comment anywhere.
  $slpToml = Get-SourceLineParts -Line 'key = 1 # there are 5 keys' -Ext 'toml' -InBlock $false
  Assert-True 'SRC6: a toml # comment extracts its text; the value stays code' (($slpToml.Comment -match '(?i)there are 5 keys') -and ($slpToml.Code -match 'key = 1')) "code=$($slpToml.Code); comment=$($slpToml.Comment)"
  # Escaped-quote handling in DOUBLE-quoted sh/toml strings: a `\"` does not close
  # the string, so a `#` after it stays INSIDE the string (code), not a false
  # comment. (Codex CROSS-CRATE-CONTRACT: the escaped quote prematurely closed the
  # string and the trailing `#` mis-split as a comment.)
  $slpShEsc = Get-SourceLineParts -Line 'echo "a\" # there are 5 configs"' -Ext 'sh' -InBlock $false
  Assert-True 'SRC6b: an escaped quote in an sh double-quoted string keeps a trailing # as CODE (no false comment)' ([string]::IsNullOrWhiteSpace($slpShEsc.Comment)) "comment=$($slpShEsc.Comment); code=$($slpShEsc.Code)"
  $slpTomlEsc = Get-SourceLineParts -Line 'key = "a\" # there are 5 keys"' -Ext 'toml' -InBlock $false
  Assert-True 'SRC6c: an escaped quote in a toml basic string keeps a trailing # as CODE (no false comment)' ([string]::IsNullOrWhiteSpace($slpTomlEsc.Comment)) "comment=$($slpTomlEsc.Comment); code=$($slpTomlEsc.Code)"
  # Block-comment records across lines (rust /* */).
  $blkRecs = @(Get-SourceCommentRecords -Records @(New-Rec 1 'fn f() {'; New-Rec 2 '/* there are'; New-Rec 3 '   7 formats */'; New-Rec 4 'let x = 1;') -Ext 'rs')
  Assert-True 'SRC7: rust /* */ block comment lines are captured across lines (code lines excluded)' ((@($blkRecs).Count -eq 2) -and (@($blkRecs | Where-Object { $_.Line -eq 4 }).Count -eq 0)) "lines=$([string]::Join(',', @($blkRecs | ForEach-Object { $_.Line })))"
  # Per-line code-number map: numbers on code portions only, keyed by line.
  $codeByLine = Get-SourceCodeNumbersByLine -Records @(New-Rec 1 'let x = 5; let y = 42; // there are 7 things') -Ext 'rs'
  Assert-True 'SRC8: the per-line code-number map has code literals (5,42) on line 1 but not the comment number (7)' ($codeByLine.ContainsKey(1) -and $codeByLine[1].ContainsKey('5') -and $codeByLine[1].ContainsKey('42') -and (-not $codeByLine[1].ContainsKey('7'))) "keys=$([string]::Join(',', @($codeByLine[1].Keys)))"
  # Staged source decision: an added comment inventory claim -> ADVISORY (downgraded).
  $srcStaged = "fn f() {`n// there are 223 icons for this feature`nlet n = 1;`n}"
  $srcHits = @(Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcStaged -AddedSet @{ 2 = $true } -Ext 'rs')
  $srcInv = @($srcHits | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC9: an ADDED rs comment inventory claim is flagged ADVISORY (one tier softer than .md ERROR)' ((@($srcInv).Count -eq 1) -and ($srcInv[0].Tier -eq 'ADVISORY')) "count=$(@($srcInv).Count); tier=$($srcInv[0].Tier)"
  # Code-echo: the claim is exempt when a NEARBY code line (within the window)
  # carries the literal -- the comment describes that adjacent constant.
  $srcEcho = "fn f() {`n// there are 223 icons for this feature`nconst ICONS: usize = 223;`n}"
  $srcEchoInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcEcho -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10: a comment count echoing an ADJACENT code literal (const on the next line) is code-echo exempt' (@($srcEchoInv).Count -eq 0) "matches=$([string]::Join('|', @($srcEchoInv | ForEach-Object { $_.Match })))"
  # NEGATIVE (Codex QUALITY): an UNRELATED same number FAR from the comment
  # (outside the echo window) must NOT exempt -- a whole-file digit set would have
  # hidden this stale comment.
  $srcFar = "fn f() {`n// there are 999 icons here`nlet a = 1;`nlet b = 2;`nlet c = 3;`nlet d = 4;`nlet e = 5;`nlet x = 999;`n}"
  $srcFarInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcFar -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10b: an UNRELATED same number far from the comment (outside the window) does NOT exempt -- still flagged' ((@($srcFarInv).Count -eq 1) -and ($srcFarInv[0].Tier -eq 'ADVISORY')) "count=$(@($srcFarInv).Count); matches=$([string]::Join('|', @($srcFarInv | ForEach-Object { $_.Match })))"
  # NEGATIVE (Codex QUALITY): the number appearing only INSIDE an IDENTIFIER nearby
  # (ICON_223_NAME) is not a standalone literal -> no exempt.
  $srcIdent = "fn f() {`n// there are 223 icons here`nlet ICON_223_NAME = 1;`n}"
  $srcIdentInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcIdent -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10c: a nearby IDENTIFIER-embedded digit (ICON_223_NAME) does NOT code-echo exempt -- still flagged' (@($srcIdentInv).Count -eq 1) "count=$(@($srcIdentInv).Count); matches=$([string]::Join('|', @($srcIdentInv | ForEach-Object { $_.Match })))"
  # NEGATIVE: the number appearing only inside a PREFIXED STRING nearby ("v223").
  $srcStr = "fn f() {`n// there are 223 icons here`nlet v = ""v223"";`n}"
  $srcStrInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcStr -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10d: a nearby prefixed-string digit ("v223") does NOT code-echo exempt -- still flagged' (@($srcStrInv).Count -eq 1) "count=$(@($srcStrInv).Count); matches=$([string]::Join('|', @($srcStrInv | ForEach-Object { $_.Match })))"
  # NEGATIVE (Codex QUALITY): a bare QUOTED numeric string ("223") is not an
  # integer literal -> must not exempt.
  $srcQstr = "fn f() {`n// there are 223 icons here`nlet v = ""223"";`n}"
  $srcQstrInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcQstr -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10e: a nearby quoted numeric STRING ("223") does NOT code-echo exempt -- still flagged' (@($srcQstrInv).Count -eq 1) "count=$(@($srcQstrInv).Count); matches=$([string]::Join('|', @($srcQstrInv | ForEach-Object { $_.Match })))"
  # NEGATIVE: a DECIMAL literal (0.223) enters the map as its FULL token, which
  # never equals the integer claim "223" -> must not exempt.
  $srcDec = "fn f() {`n// there are 223 icons here`nlet f = 0.223;`n}"
  $srcDecInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcDec -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10f: a nearby decimal fragment (0.223) does NOT code-echo exempt -- still flagged' (@($srcDecInv).Count -eq 1) "count=$(@($srcDecInv).Count); matches=$([string]::Join('|', @($srcDecInv | ForEach-Object { $_.Match })))"
  # POSITIVE guard: a genuine standalone INTEGER literal (= 223) STILL exempts, so
  # the tightened lookarounds did not break the real code-echo case.
  $srcInt = "fn f() {`n// there are 223 icons here`nlet n = 223;`n}"
  $srcIntInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcInt -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10g: a genuine standalone integer literal (= 223) STILL code-echo exempts (real case preserved)' (@($srcIntInv).Count -eq 0) "count=$(@($srcIntInv).Count)"
  # NEGATIVES (Codex QUALITY): a `$`-prefixed run (shell positional / ps1
  # variable) or a SIGNED number is not the adjacent positive literal the
  # exemption describes -> must not exempt.
  $srcPos = "deploy() {`n# there are 2 deploy steps here`necho `$2`n}"
  $srcPosInv = @((Get-StagedSourceFindings -Path 'scripts/x.sh' -StagedText $srcPos -AddedSet @{ 2 = $true } -Ext 'sh') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10h: a nearby shell positional ($2) does NOT code-echo exempt -- still flagged' (@($srcPosInv).Count -eq 1) "count=$(@($srcPosInv).Count); matches=$([string]::Join('|', @($srcPosInv | ForEach-Object { $_.Match })))"
  $srcBraced = "deploy() {`n# there are 2 deploy steps here`necho `${2}`n}"
  $srcBracedInv = @((Get-StagedSourceFindings -Path 'scripts/x.sh' -StagedText $srcBraced -AddedSet @{ 2 = $true } -Ext 'sh') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10h2: a nearby BRACED shell positional (${2}) does NOT code-echo exempt -- still flagged' (@($srcBracedInv).Count -eq 1) "count=$(@($srcBracedInv).Count); matches=$([string]::Join('|', @($srcBracedInv | ForEach-Object { $_.Match })))"
  $srcBracedPs = "function d {`n# there are 2 configs here`nWrite-Host `${2}`n}"
  $srcBracedPsInv = @((Get-StagedSourceFindings -Path 'scripts/x.ps1' -StagedText $srcBracedPs -AddedSet @{ 2 = $true } -Ext 'ps1') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10h3: a nearby ps1 braced variable (${2}) does NOT code-echo exempt -- still flagged' (@($srcBracedPsInv).Count -eq 1) "count=$(@($srcBracedPsInv).Count); matches=$([string]::Join('|', @($srcBracedPsInv | ForEach-Object { $_.Match })))"
  $srcSign = "fn f() {`n// there are 2 modes here`nlet offset = -2;`n}"
  $srcSignInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcSign -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10i: a nearby SIGNED literal (-2) does NOT code-echo exempt -- still flagged' (@($srcSignInv).Count -eq 1) "count=$(@($srcSignInv).Count); matches=$([string]::Join('|', @($srcSignInv | ForEach-Object { $_.Match })))"
  # POSITIVES (same review): Rust TYPE-SUFFIXED literals are real adjacent
  # literals -- the suffix is stripped to the numeric key, so they DO exempt.
  $srcSufInt = "fn f() {`n// there are 3 icons here`nconst N: usize = 3usize;`n}"
  $srcSufIntInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcSufInt -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC10j: a suffixed integer literal (3usize) DOES code-echo exempt' (@($srcSufIntInv).Count -eq 0) "count=$(@($srcSufIntInv).Count)"
  $srcSufDec = "fn f() {`n// the verify step took 0.25 seconds here`nconst T: f32 = 0.25f32;`n}"
  $srcSufDecHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcSufDec -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC10k: a suffixed decimal literal (0.25f32) DOES code-echo exempt' (@($srcSufDecHits).Count -eq 0) "count=$(@($srcSufDecHits).Count)"
  # A CODE line (not a comment) is never scanned, even if it looks shape-like.
  $srcCode = "fn f() {`nlet there_are_widgets = 223;`n}"
  $srcCodeHits = @(Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcCode -AddedSet @{ 2 = $true } -Ext 'rs')
  Assert-True 'SRC11: a CODE line (no comment) is never scanned for inventory shape' (@($srcCodeHits).Count -eq 0) "matches=$([string]::Join('|', @($srcCodeHits | ForEach-Object { $_.Match })))"
  # A comment on a NON-added line is filtered out.
  $srcNonAdded = @(Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcStaged -AddedSet @{ 3 = $true } -Ext 'rs')
  Assert-True 'SRC12: a comment inventory claim on a NON-added line is filtered out (added-line filter)' (@($srcNonAdded | Where-Object { $_.Class -eq 'INVENTORY' }).Count -eq 0) ''
  # A COMMENT-PREFIXED fence (`// ```) around an ADDED claim is skipped -- the
  # fence opener/closer are UNCHANGED lines, so fence context must span the full
  # comment records (not just the added line). (Codex TEST-QUALITY.)
  $srcFence = "fn f() {`n// ``````" + "`n// there are 223 icons`n// ``````" + "`n}"
  $srcFenceInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcFence -AddedSet @{ 3 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC13: an added claim inside a comment-prefixed fence (unchanged `// ``` opener) is skipped' (@($srcFenceInv).Count -eq 0) "matches=$([string]::Join('|', @($srcFenceInv | ForEach-Object { $_.Match })))"
  # Contrast: the SAME added claim WITHOUT the fence flags -- proves the skip is
  # fence-caused, not unconditional.
  $srcNoFence = "fn f() {`n// a note`n// there are 223 icons`n// end`n}"
  $srcNoFenceInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcNoFence -AddedSet @{ 3 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC13b: the same added claim WITHOUT a comment fence flags (ADVISORY)' ((@($srcNoFenceInv).Count -eq 1) -and ($srcNoFenceInv[0].Tier -eq 'ADVISORY')) "count=$(@($srcNoFenceInv).Count)"
  # A BLOCK-doc fence (` * ````) around an added claim is skipped -- the conventional
  # doc-block continuation `*` is normalized before fence detection, so a fenced
  # example inside a `/** */` block does not false-positive. (Codex QUALITY: the
  # leading `*` hid block-doc fences from Get-FencedLineFlags.)
  $srcBlkFence = "fn f() {`n/**`n * ``````" + "`n * there are 223 icons`n * ``````" + "`n */`nlet n = 1;`n}"
  $srcBlkFenceInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcBlkFence -AddedSet @{ 4 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC13c: an added claim inside a BLOCK-doc fence (` * ```` opener) is skipped (leading `*` normalized)' (@($srcBlkFenceInv).Count -eq 0) "matches=$([string]::Join('|', @($srcBlkFenceInv | ForEach-Object { $_.Match })))"
  # Contrast: the SAME block-doc claim WITHOUT a fence still flags -- the `*`
  # normalization enables fence recognition without suppressing detection.
  $srcBlkNoFence = "fn f() {`n/**`n * there are 223 icons`n */`nlet n = 1;`n}"
  $srcBlkNoFenceInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcBlkNoFence -AddedSet @{ 3 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC13d: the same block-doc claim WITHOUT a fence still flags (ADVISORY)' ((@($srcBlkNoFenceInv).Count -eq 1) -and ($srcBlkNoFenceInv[0].Tier -eq 'ADVISORY')) "count=$(@($srcBlkNoFenceInv).Count)"
  # A `// ``` ` comment-fence opener followed by real CODE then an added claim must
  # NOT suppress the claim: the intervening code RESETS the line-comment fence, so
  # the fence from before the code does not carry across it. Contrast with SRC13
  # (fence + claim on CONTIGUOUS comment lines -> suppressed). (Codex SILENT-FAILURE:
  # the compressed comment stream leaked fence state across code.)
  $srcFenceGap = "fn f() {`n// ``````" + "`nlet code = 1;`n// there are 223 icons`n}"
  $srcFenceGapInv = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcFenceGap -AddedSet @{ 4 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'SRC13e: a comment-fence opener + intervening CODE does NOT suppress a later added claim (fence resets across code)' ((@($srcFenceGapInv).Count -eq 1) -and ($srcFenceGapInv[0].Tier -eq 'ADVISORY')) "count=$(@($srcFenceGapInv).Count); matches=$([string]::Join('|', @($srcFenceGapInv | ForEach-Object { $_.Match })))"
  # The staged source path scans the MAGNITUDE class too (not only INVENTORY): a
  # transient count in an added comment surfaces a MAGNITUDE advisory. (Codex
  # TEST-QUALITY.)
  $srcMag = "fn f() {`n// the chase took 6 rounds here`nlet n = 1;`n}"
  $srcMagHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcMag -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC14: a transient count in an added source comment ("took 6 rounds") surfaces a MAGNITUDE advisory' ((@($srcMagHits).Count -ge 1) -and ($srcMagHits[0].Tier -eq 'ADVISORY')) "count=$(@($srcMagHits).Count); matches=$([string]::Join('|', @($srcMagHits | ForEach-Object { $_.Match })))"
  # A source-comment magnitude count that echoes a NEARBY code literal is exempt
  # (the echo filter applies to the magnitude class as well as inventory).
  $srcMagEcho = "fn f() {`n// the retry loop took 6 rounds here`nconst ROUNDS: usize = 6;`n}"
  $srcMagEchoHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcMagEcho -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC14b: a source-comment count ("took 6 rounds") echoing a nearby code literal (= 6) is MAGNITUDE-exempt' (@($srcMagEchoHits).Count -eq 0) "count=$(@($srcMagEchoHits).Count)"
  # A DECIMAL source-comment count ("took 0.25") must NOT be echo-suppressed by an
  # unrelated INTEGER (`= 0`) nearby -- the finding compares as "0.25", not the
  # fragment "0". (Codex BLOCKER.)
  $srcDecMag = "fn f() {`n// the verify step took 0.25 seconds here`nlet n = 0;`n}"
  $srcDecMagHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcDecMag -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC14c: a DECIMAL source-comment count ("took 0.25") is NOT echo-suppressed by an unrelated integer (0) nearby' (@($srcDecMagHits).Count -ge 1) "count=$(@($srcDecMagHits).Count); matches=$([string]::Join('|', @($srcDecMagHits | ForEach-Object { $_.Match })))"
  # POSITIVE decimal echo: a decimal comment quantity beside the MATCHING decimal
  # code literal IS exempt -- the code map stores full decimal tokens. (Codex
  # QUALITY: integers-only map made the documented exemption unreachable for
  # decimals.)
  $srcDecEcho = "fn f() {`n// the verify step took 0.25 seconds here`nconst TIMEOUT: f32 = 0.25;`n}"
  $srcDecEchoHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcDecEcho -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC14d: a decimal comment count ("took 0.25") beside the MATCHING decimal literal (= 0.25) IS echo-exempt' (@($srcDecEchoHits).Count -eq 0) "count=$(@($srcDecEchoHits).Count)"
  # SPELLED enumerated totals use the echo exemption via the word->numeral map
  # ("all three" -> 3); with no matching literal nearby the advisory stands.
  # (Codex QUALITY: spelled totals had no digit to compare.)
  $srcSpellEcho = "fn f() {`n// handles all three cases here`nconst CASES: usize = 3;`n}"
  $srcSpellEchoHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcSpellEcho -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC14e: a spelled total ("all three cases") beside the matching literal (= 3) IS echo-exempt' (@($srcSpellEchoHits).Count -eq 0) "count=$(@($srcSpellEchoHits).Count); matches=$([string]::Join('|', @($srcSpellEchoHits | ForEach-Object { $_.Match })))"
  $srcSpellNo = "fn f() {`n// handles all three cases here`nlet n = 1;`n}"
  $srcSpellNoHits = @((Get-StagedSourceFindings -Path 'crates/x/src/lib.rs' -StagedText $srcSpellNo -AddedSet @{ 2 = $true } -Ext 'rs') | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'SRC14f: the same spelled total with NO matching literal nearby still flags (ADVISORY)' ((@($srcSpellNoHits).Count -ge 1) -and ($srcSpellNoHits[0].Tier -eq 'ADVISORY')) "count=$(@($srcSpellNoHits).Count)"

  # ---- SOURCE-FILE SELECTION ($script:SourceExtPattern) ----
  # The staged-source loop keys file selection on this pattern; a break here would
  # silently stop scanning source comments. SEL pins the pattern directly (fast,
  # pure); the FULL loop (selection + index-blob read + per-file diff parse +
  # Get-StagedSourceFindings assembly) is exercised end-to-end by E2E3 below, using
  # an ALTERNATE index so the shared worktree index is never mutated.
  Assert-True 'SEL1: SourceExtPattern SELECTS .rs/.ps1/.toml/.sh' (('crates/x/src/lib.rs' -match $script:SourceExtPattern) -and ('scripts/x.ps1' -match $script:SourceExtPattern) -and ('Cargo.toml' -match $script:SourceExtPattern) -and ('scripts/x.sh' -match $script:SourceExtPattern)) ''
  Assert-True 'SEL2: SourceExtPattern REJECTS non-source (.md/.txt/.rsx/no-ext)' ((-not ('README.md' -match $script:SourceExtPattern)) -and (-not ('notes.txt' -match $script:SourceExtPattern)) -and (-not ('x.rsx' -match $script:SourceExtPattern)) -and (-not ('Makefile' -match $script:SourceExtPattern))) ''
  Assert-True 'SEL3: the ext token the loop derives is lower-cased (.PS1 -> ps1)' (([System.IO.Path]::GetExtension('scripts/X.PS1')).TrimStart('.').ToLowerInvariant() -eq 'ps1') ''

  # ---- diff parser ----
  $diff = @"
diff --git a/History.md b/History.md
--- a/History.md
+++ b/History.md
@@ -10,0 +11,2 @@
+cites logs/codex/probe-pack-45ff
+took 6 rounds
@@ -40,0 +42,1 @@
+see auto-review.ps1:1941
"@
  $parsed = Get-AddedRecordsFromDiff -DiffText $diff
  Assert-True 'DIFF1: one file parsed' ($parsed.Keys.Count -eq 1 -and $parsed.ContainsKey('History.md')) "keys=$([string]::Join(',', $parsed.Keys))"
  $recs = @($parsed['History.md'])
  Assert-True 'DIFF2: three added lines captured' ($recs.Count -eq 3) "count=$($recs.Count)"
  Assert-True 'DIFF3: first added line number is 11' ($recs.Count -ge 1 -and $recs[0].Line -eq 11) "line=$($recs[0].Line)"
  Assert-True 'DIFF4: second hunk line number is 42' ($recs.Count -eq 3 -and $recs[2].Line -eq 42) "line=$($recs[2].Line)"
  # An ADDED content line shaped like a file header (`++ b/...` -> diff `+++ b/...`)
  # must be captured as content, not parsed as a header. (Codex BLOCKER.)
  $diffPlus = @"
diff --git a/docs/x.md b/docs/x.md
--- a/docs/x.md
+++ b/docs/x.md
@@ -0,0 +1 @@
+++ b/scripts/foo.rs:42
"@
  $parsedPlus = Get-AddedRecordsFromDiff -DiffText $diffPlus
  $plusRecs = @($parsedPlus['docs/x.md'])
  Assert-True 'DIFF5: an added ++ b/... content line is captured (hunk-aware, not a header)' (($parsedPlus.ContainsKey('docs/x.md')) -and ($plusRecs.Count -eq 1) -and ($plusRecs[0].Text -eq '++ b/scripts/foo.rs:42')) "count=$($plusRecs.Count); text=$($plusRecs[0].Text)"
  Assert-True 'DIFF6: the in-hunk added-line counter counts the ++ b/... content line' ((Measure-InHunkAddedLines -DiffText $diffPlus) -eq 1) "n=$(Measure-InHunkAddedLines -DiffText $diffPlus)"
  Assert-True 'DIFF7: a deletion-only hunk has 0 in-hunk added lines' ((Measure-InHunkAddedLines -DiffText "diff --git a/y.md b/y.md`n--- a/y.md`n+++ b/y.md`n@@ -5,1 +4,0 @@`n-gone line") -eq 0) ''

  # ---- tier / exit-code semantics ----
  $mixed = @(New-Rec 1 'anchor auto-review.ps1:1941'; New-Rec 2 'took 6 rounds')
  $mixedFindings = @()
  $mixedFindings += Get-LineAnchorFindings -Records $mixed
  $mixedFindings += Get-MagnitudeFindings -Records $mixed
  $hasError = (@($mixedFindings | Where-Object { $_.Tier -eq 'ERROR' }).Count -gt 0)
  Assert-True 'TIER1: error + advisory mix has an error-tier finding (findings exit 3)' $hasError ''
  $advisoryOnly = @(Get-MagnitudeFindings -Records @(New-Rec 1 'took 6 rounds'))
  Assert-True 'TIER2: advisory-only has no error-tier finding (exit 0)' ((@($advisoryOnly | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 0) -and $advisoryOnly.Count -ge 1) ''

  # ---- report formatting (deterministic, structural) ----
  $rep = Format-Report -Findings $mixedFindings -ScopeLabel 'selftest' -FileCount 1
  Assert-True 'RPT1: report names the error count' ($rep -match 'error\(s\)') ''
  Assert-True 'RPT2: report is structural (no large prose)' ($rep -match 'LINE-ANCHOR') ''

  # ---- generated-doc exemption (PATH gate AND content signature) ----
  # The header must match the REAL generator header (analyze-blocker-trends.ps1
  # emits `# Cross-review BLOCKER Trends Report`); this fixture exercises the
  # header-match signature branch in Test-IsGeneratedContent.
  $genPath = 'docs/blocker-trends.md'
  $genHead = "# Cross-review BLOCKER Trends Report`n`nGenerated: 2026-05-23T02:38:20-04:00`nbody cites auto-review.ps1:99"
  Assert-True 'GEN1: generated-report content AT the known generated path is detected as generated' (Test-IsGeneratedContent -Path $genPath -Content $genHead) ''
  Assert-True 'GEN2: a hand-authored doc is not generated' (-not (Test-IsGeneratedContent -Path 'docs/notes.md' -Content "# Title`n`nsome hand-authored prose, no stamp")) ''
  Assert-True 'GEN3: a bare Generated: line WITHOUT a signature is NOT generated even at the generated path (content gate)' (-not (Test-IsGeneratedContent -Path $genPath -Content "# My Notes`nGenerated: by me yesterday`nsee foo.rs:5")) ''
  # GEN4 uses a NON-matching header (the prior `# Codex BLOCKER Trends Report`,
  # which the header regex no longer matches) so this fixture passes ONLY via the
  # `Source: ... verdict` signature OR-branch -- isolating that branch as a
  # regression guard. (A matching header would let GEN4 pass via the header
  # branch and silently mask a broken Source branch.)
  Assert-True 'GEN4: a Source:...verdict signature AT the generated path IS generated (signature OR-branch)' (Test-IsGeneratedContent -Path $genPath -Content "# Codex BLOCKER Trends Report`nGenerated: 2026-06-13T01:00:00-04:00`nSource: 9 verdict files`nLINE x.rs:5") ''
  # GEN5 is the spoof-defense regression test: the EXACT generated-report head at an
  # AUTHORED path must NOT be skipped -- the path gate closes the content-only bypass.
  Assert-True 'GEN5: generated-report content at an AUTHORED path is NOT generated (path gate closes the spoof)' (-not (Test-IsGeneratedContent -Path 'PLAN.md' -Content $genHead)) ''
  Assert-True 'GEN6: the leaf is anchored -- a sibling docs/codex-*.md is NOT blanket-exempt' (-not (Test-IsGeneratedContent -Path 'docs/codex-other.md' -Content $genHead)) ''
  Assert-True 'GEN7: an empty path is not generated' (-not (Test-IsGeneratedContent -Path '' -Content $genHead)) ''
  # The path gate is the spoof boundary: a CASE-VARIANT authored path must NOT be
  # treated as generated, else a wrong-cased doc self-exempts. (Codex BLOCKER.)
  Assert-True 'GEN8: a case-variant generated path is NOT generated (ordinal path gate closes the wrong-case spoof)' (-not (Test-IsGeneratedContent -Path 'docs/Blocker-Trends.md' -Content $genHead)) ''

  # ---- index path set (staged DEAD-REF resolution) ----
  $pset = Build-PathSetFromFileList -Files @('scripts/codex/x.ps1', 'docs/y.md')
  Assert-True 'IDX1: an indexed file path is in the set' ($pset.ContainsKey('scripts/codex/x.ps1')) ''
  Assert-True 'IDX2: ancestor dir prefixes are in the set' ($pset.ContainsKey('scripts') -and $pset.ContainsKey('scripts/codex')) ''
  Assert-True 'IDX3: an unrelated dir is NOT in the set' (-not $pset.ContainsKey('logs')) ''
  Assert-True 'IDX4: backslash file-list input is normalized to /' ((Build-PathSetFromFileList -Files @('a\b\c.txt')).ContainsKey('a/b/c.txt')) ''
  # Case-sensitivity regression: git paths are case-sensitive, so a wrong-cased
  # reference must NOT resolve against a tracked path (else a real DEAD-REF on a
  # case-sensitive clone is dropped). (Codex QUALITY.)
  Assert-True 'IDX5: the path set is case-SENSITIVE (Scripts/Codex/X.PS1 != scripts/codex/x.ps1)' ((-not $pset.ContainsKey('Scripts/codex/x.ps1')) -and (-not $pset.ContainsKey('Scripts'))) ''

  # ---- top-level dir set (deterministic, index + .gitignore) ----
  $tld = Get-TopLevelDirSet -IndexFiles @('scripts/codex/x.ps1', 'docs/y.md', 'README.md') -GitignoreText "logs/`n.orchestrator/`n# a comment`n!keepme`n*.tmp`ntarget/build`n/vendor`nnotes.md"
  Assert-True 'TLD1: tracked top-level dirs come from the index' ($tld.ContainsKey('scripts') -and $tld.ContainsKey('docs')) ''
  Assert-True 'TLD2: gitignored evidence roots are included (logs, .orchestrator)' ($tld.ContainsKey('logs') -and $tld.ContainsKey('.orchestrator')) ''
  Assert-True 'TLD3: a root-level FILE is not a top-level dir' (-not $tld.ContainsKey('README.md')) ''
  Assert-True 'TLD4: comment / negation / bare-glob gitignore lines are not added' ((-not $tld.ContainsKey('# a comment')) -and (-not $tld.ContainsKey('keepme')) -and (-not $tld.ContainsKey('*.tmp'))) ''
  Assert-True 'TLD5: first segment of a nested ignored dir is added (target/build -> target)' ($tld.ContainsKey('target')) ''
  Assert-True 'TLD6: a bare root dir entry is added (/vendor -> vendor)' ($tld.ContainsKey('vendor')) ''
  Assert-True 'TLD7: a bare gitignore FILE entry (notes.md) is not added as a dir' (-not $tld.ContainsKey('notes.md')) ''
  # The top-level INTENT gate is case-INSENSITIVE (asymmetry with the ordinal index
  # resolver): a wrong-cased first segment must still register as repo-like so the
  # token proceeds to the resolver instead of being dropped as prose. (Codex
  # BLOCKER.)
  Assert-True 'TLD8: the top-level dir set is case-INsensitive (Scripts registers as repo-like)' ($tld.ContainsKey('Scripts') -and $tld.ContainsKey('Logs')) ''

  # ---- staged per-file decision logic (Get-StagedFileFindings) ----
  $stgTop = { param($s) return (@{ 'scripts' = $true }).ContainsKey($s) }
  $stgRes = { param($p) return $false }
  $stgText = "line one`nsee scripts/codex/auto-review.ps1:42 here`nline three"
  $stgAdd = @(Get-StagedFileFindings -Path 'docs/x.md' -StagedText $stgText -AddedSet @{ 2 = $true } -Resolver $stgRes -IsTopLevel $stgTop)
  Assert-True 'STG1: a LINE-ANCHOR on an ADDED line is kept' (@($stgAdd | Where-Object { $_.Class -eq 'LINE-ANCHOR' }).Count -eq 1) "all=$([string]::Join(',', @($stgAdd | ForEach-Object { "$($_.Class)@$($_.Line)" })))"
  $stgNon = @(Get-StagedFileFindings -Path 'docs/x.md' -StagedText $stgText -AddedSet @{ 1 = $true } -Resolver $stgRes -IsTopLevel $stgTop)
  Assert-True 'STG2: a LINE-ANCHOR on a NON-added line is filtered out (added-line filter)' (@($stgNon | Where-Object { $_.Class -eq 'LINE-ANCHOR' }).Count -eq 0) ''
  $stgPlan = "### M1: Real milestone`nbody refs the unknown M42 here"
  $stgPlanHits = @(Get-StagedFileFindings -Path 'PLAN.md' -StagedText $stgPlan -AddedSet @{ 1 = $true } -Resolver $stgRes -IsTopLevel $stgTop)
  Assert-True 'STG3: a PLAN TAG finding on a NON-added line still surfaces (whole-file TAG bypass)' (@($stgPlanHits | Where-Object { $_.Class -eq 'TAG' }).Count -ge 1) ''
  # Regression for the staged-mode missing-diff-key bug -- drives the SAME helper
  # (Get-AddedLineSetForPath) staged mode uses, so a reversion in production fails
  # here. A MISSING key must yield an EMPTY set (no synthetic line 0) so the
  # "parsed 0 added lines" fail-loud guard still fires. (Codex BLOCKER.)
  $stgMissSet = Get-AddedLineSetForPath -AddedByFile @{} -Path 'docs/x.md'
  Assert-True 'STG4: a missing diff-parser key leaves the added-set EMPTY (no synthetic line 0 bypassing fail-loud)' ($stgMissSet.Count -eq 0) "count=$($stgMissSet.Count)"
  # A PRESENT key with records yields exactly those added line numbers.
  $stgPresentSet = Get-AddedLineSetForPath -AddedByFile @{ 'docs/x.md' = @(@{Line=11}, @{Line=42}) } -Path 'docs/x.md'
  Assert-True 'STG4b: a present diff-parser key yields its added line numbers' (($stgPresentSet.Count -eq 2) -and $stgPresentSet.ContainsKey(11) -and $stgPresentSet.ContainsKey(42)) "count=$($stgPresentSet.Count)"
  # The UNGUARDED form (what the bug did) WOULD synthesize line 0 -- pin the contrast.
  $stgBadSet = @{}
  foreach ($r in @((@{})['docs/x.md'])) { $stgBadSet[[int]$r.Line] = $true }
  Assert-True 'STG4c: the unguarded @($hash[$missing]) form synthesizes line 0 (the bug the helper prevents)' (($stgBadSet.Count -eq 1) -and $stgBadSet.ContainsKey(0)) "count=$($stgBadSet.Count)"
  # A frozen-snapshot marker present in the STAGED HEAD must downgrade a NEWLY
  # ADDED line anchor to ADVISORY end-to-end: Invoke-FileChecks reconstructs the
  # head from the staged records and derives $isFrozen, and Get-StagedFileFindings
  # keeps the (advisory) finding because it sits on an added line. The SNAP*/LA8
  # tests cover marker parsing and the tier flag in isolation; this pins the
  # staged wiring between them. (Codex QUALITY.)
  $stgFrozenText = "# Frozen audit`n<!-- frozen-snapshot -->`nsee scripts/codex/auto-review.ps1:42 here"
  $stgFrozenLA = @((Get-StagedFileFindings -Path 'docs/audit.md' -StagedText $stgFrozenText -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'LINE-ANCHOR' })
  Assert-True 'STG5: a staged-head frozen-snapshot marker downgrades a newly added line anchor to ADVISORY (end-to-end marker->staged wiring)' (($stgFrozenLA.Count -eq 1) -and ($stgFrozenLA[0].Tier -eq 'ADVISORY')) "count=$($stgFrozenLA.Count); tier=$($stgFrozenLA[0].Tier)"
  # Contrast: the SAME added anchor WITHOUT the marker stays ERROR -- proves the
  # downgrade is caused by the staged-head marker, not unconditional.
  $stgUnfrozenText = "# Live audit`nstill the live authority`nsee scripts/codex/auto-review.ps1:42 here"
  $stgUnfrozenLA = @((Get-StagedFileFindings -Path 'docs/audit.md' -StagedText $stgUnfrozenText -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'LINE-ANCHOR' })
  Assert-True 'STG5b: WITHOUT a staged-head marker the same added anchor stays ERROR (downgrade is marker-caused)' (($stgUnfrozenLA.Count -eq 1) -and ($stgUnfrozenLA[0].Tier -eq 'ERROR')) "count=$($stgUnfrozenLA.Count); tier=$($stgUnfrozenLA[0].Tier)"
  # The frozen marker downgrades ONLY line anchors: an ADDED inventory claim in
  # the SAME frozen-marked staged doc stays ERROR through the full staged wiring
  # (Invoke-FileChecks derives $isFrozen -> Get-StagedFileFindings -> the
  # inventory detector, which takes no frozen input at all). INV4 pins the
  # helper default; this pins the marker-ignoring WIRING. (Codex QUALITY: the
  # unit fixture alone left the staged marker path uncovered.)
  $stgFrozenInvText = "# Frozen audit`n<!-- frozen-snapshot -->`nthere are 223 icons for this feature"
  $stgFrozenInv = @((Get-StagedFileFindings -Path 'docs/audit.md' -StagedText $stgFrozenInvText -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'STG5c: an added inventory claim in a frozen-marked staged doc stays ERROR (marker downgrades line anchors only, end-to-end)' (($stgFrozenInv.Count -eq 1) -and ($stgFrozenInv[0].Tier -eq 'ERROR')) "count=$($stgFrozenInv.Count); tier=$($stgFrozenInv[0].Tier)"
  # SELF-NARR tier is derived from the PATH LEAF ('History.md' -> ERROR, other
  # narrative docs -> ADVISORY) inside the staged wiring; the SN* fixtures pass
  # the bool explicitly, so THIS pins the leaf-name derivation end-to-end -- a
  # typo in the comparison would silently downgrade History self-narration to
  # ADVISORY with the suite still green. (Claude QUALITY: same wiring-gap class as
  # STG5c.)
  $stgHistText = "# History`nolder entry text`nthis merge lands the gate change"
  $stgHistSN = @((Get-StagedFileFindings -Path 'History.md' -StagedText $stgHistText -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'SELF-NARR' })
  Assert-True 'STG6: History.md self-narration on an added line is ERROR end-to-end (leaf-name wiring)' (($stgHistSN.Count -eq 1) -and ($stgHistSN[0].Tier -eq 'ERROR')) "count=$($stgHistSN.Count); tier=$($stgHistSN[0].Tier)"
  $stgOtherSN = @((Get-StagedFileFindings -Path 'docs/notes.md' -StagedText $stgHistText -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'SELF-NARR' })
  Assert-True 'STG6b: the same self-narration in a non-History doc is ADVISORY (leaf-name contrast)' (($stgOtherSN.Count -eq 1) -and ($stgOtherSN[0].Tier -eq 'ADVISORY')) "count=$($stgOtherSN.Count); tier=$($stgOtherSN[0].Tier)"
  # ---- History dated-entry-body mask (Get-HistoryEntryBodyLineSet) ----
  # The count-drift exemption applies ONLY to lines inside a DATED (`## YYYY-MM-DD`,
  # incl. the `## YYYY-MM-DD/NN` day-range form)
  # entry body; HEB* pin the mask helper in isolation, STG7* the end-to-end wiring.
  $hebRecs = @(New-Rec 1 '# Project History'; New-Rec 2 ''; New-Rec 3 'preamble count line'; New-Rec 4 ''; New-Rec 5 '## 2026-07-09 - entry'; New-Rec 6 ''; New-Rec 7 'body line'; New-Rec 8 ''; New-Rec 9 '## Notes'; New-Rec 10 'undated line')
  $heb = Get-HistoryEntryBodyLineSet -Records $hebRecs
  Assert-True 'HEB1: a PREAMBLE line (before the first dated heading) is NOT in the entry-body set' (-not $heb.ContainsKey(3)) "keys=$([string]::Join(',', @($heb.Keys | Sort-Object)))"
  Assert-True 'HEB2: body lines ARE in the set, the dated HEADING line is NOT (body-only waiver)' ((-not $heb.ContainsKey(5)) -and $heb.ContainsKey(7)) "keys=$([string]::Join(',', @($heb.Keys | Sort-Object)))"
  Assert-True 'HEB3: an UNDATED `## ` heading closes the dated region (its lines are NOT in the set)' ((-not $heb.ContainsKey(9)) -and (-not $heb.ContainsKey(10))) "keys=$([string]::Join(',', @($heb.Keys | Sort-Object)))"
  $hebFence = Get-HistoryEntryBodyLineSet -Records @(New-Rec 1 'preamble'; New-Rec 2 '```'; New-Rec 3 '## 2026-07-09 fake'; New-Rec 4 '```'; New-Rec 5 'still preamble')
  Assert-True 'HEB4: a dated-looking heading INSIDE a code fence does NOT open a dated entry' ($hebFence.Keys.Count -eq 0) "keys=$([string]::Join(',', @($hebFence.Keys | Sort-Object)))"
  $hebH3 = Get-HistoryEntryBodyLineSet -Records @(New-Rec 1 '## 2026-07-09 - entry'; New-Rec 2 '### subsection'; New-Rec 3 'still in the dated entry')
  Assert-True 'HEB5: an H3 (`### `) heading does NOT close the dated entry (only `## ` boundaries; the `## ` heading itself stays out of the body mask)' ((-not $hebH3.ContainsKey(1)) -and $hebH3.ContainsKey(2) -and $hebH3.ContainsKey(3)) "keys=$([string]::Join(',', @($hebH3.Keys | Sort-Object)))"
  $hebEmpty = Get-HistoryEntryBodyLineSet -Records @()
  Assert-True 'HEB6: an empty record set yields an empty mask (no throw)' ($hebEmpty.Count -eq 0) "count=$($hebEmpty.Count)"
  $hebDatePrefix = Get-HistoryEntryBodyLineSet -Records @(New-Rec 1 '## 2026-07-090 notes'; New-Rec 2 'not exempt'; New-Rec 3 '## 2026-07-09foo'; New-Rec 4 'also not exempt'; New-Rec 5 '## 2026-07-09'; New-Rec 6 'exempt body')
  Assert-True 'HEB7: a date-PREFIX non-date heading (`## 2026-07-090 notes`, `## 2026-07-09foo`) does NOT open a dated entry, while a bare `## YYYY-MM-DD` (EOL boundary) does' ((-not $hebDatePrefix.ContainsKey(2)) -and (-not $hebDatePrefix.ContainsKey(4)) -and $hebDatePrefix.ContainsKey(6)) "keys=$([string]::Join(',', @($hebDatePrefix.Keys | Sort-Object)))"
  $hebRange = Get-HistoryEntryBodyLineSet -Records @(New-Rec 1 '## 2026-05-01/02 - first launch'; New-Rec 2 'exempt range body'; New-Rec 3 '## 2026-07-09/x bad'; New-Rec 4 'not exempt'; New-Rec 5 '## 2026-07-09/9foo'; New-Rec 6 'not exempt either'; New-Rec 7 '## 2026-07-09/123 notes'; New-Rec 8 'still not exempt'; New-Rec 9 '## 2026-05-01/02'; New-Rec 10 'exempt eol-range body')
  Assert-True 'HEB8: the established slash day-RANGE heading (`## YYYY-MM-DD/NN ...` or bare `/NN` at EOL) opens a dated entry, while a malformed range tail (`/x`, `/9foo`, `/123 notes`) stays closed' ($hebRange.ContainsKey(2) -and (-not $hebRange.ContainsKey(4)) -and (-not $hebRange.ContainsKey(6)) -and (-not $hebRange.ContainsKey(8)) -and $hebRange.ContainsKey(10)) "keys=$([string]::Join(',', @($hebRange.Keys | Sort-Object)))"

  # ---- count-drift EXEMPTION end-to-end (root History.md, DATED entry body only) ----
  # NARROW waiver: a count is exempt ONLY inside a DATED entry body of the ROOT
  # History.md log. Preamble / undated-section / nested-History.md / non-History
  # counts all stay in class. Wired in Invoke-FileChecks (root-path gate + mask drop),
  # so pin it end-to-end via Get-StagedFileFindings. (User decision 2026-07-09; scoped
  # per Codex CROSS-CRATE-CONTRACT review-20260709 -- the first cut skipped the whole
  # file off the leaf name and over-exempted preamble/undated/nested.)
  $histDated = "# Project History`n`n## 2026-07-09 - entry`n`nthe schema now covers 7 biomes end to end"
  $stgHistDated = @(Get-StagedFileFindings -Path 'History.md' -StagedText $histDated -AddedSet @{ 5 = $true } -Resolver $stgRes -IsTopLevel $stgTop)
  Assert-True 'STG7: a count inside a DATED root-History.md entry body is exempt (no INVENTORY finding)' (@($stgHistDated | Where-Object { $_.Class -eq 'INVENTORY' }).Count -eq 0) "inv=$(@($stgHistDated | Where-Object { $_.Class -eq 'INVENTORY' }).Count)"
  Assert-True 'STG7b: the same dated-entry count yields NO error-tier (blocking) finding at all' (@($stgHistDated | Where-Object { $_.Tier -eq 'ERROR' }).Count -eq 0) "err=$(@($stgHistDated | Where-Object { $_.Tier -eq 'ERROR' }).Count)"
  $histPreamble = "# Project History`n`nthe schema now covers 7 biomes end to end`n`n## 2026-07-09 - entry`n`nbody text"
  $stgHistPre = @((Get-StagedFileFindings -Path 'History.md' -StagedText $histPreamble -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'STG7c: a count in the History.md PREAMBLE (before any dated heading) STILL flags INVENTORY ERROR' ((@($stgHistPre).Count -eq 1) -and ($stgHistPre[0].Tier -eq 'ERROR')) "count=$(@($stgHistPre).Count); tier=$($stgHistPre[0].Tier)"
  $histUndated = "# Project History`n`n## 2026-07-09 - entry`n`nbody`n`n## Notes`n`nthe schema now covers 7 biomes end to end"
  $stgHistUnd = @((Get-StagedFileFindings -Path 'History.md' -StagedText $histUndated -AddedSet @{ 9 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'STG7d: a count under an UNDATED `## ` section STILL flags INVENTORY ERROR (dated-entry-only waiver)' ((@($stgHistUnd).Count -eq 1) -and ($stgHistUnd[0].Tier -eq 'ERROR')) "count=$(@($stgHistUnd).Count); tier=$($stgHistUnd[0].Tier)"
  $stgNested = @((Get-StagedFileFindings -Path 'docs/History.md' -StagedText $histDated -AddedSet @{ 5 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'STG7e: a count in a dated entry of a NESTED docs/History.md STILL flags (root-path gate; only the root log is exempt)' ((@($stgNested).Count -eq 1) -and ($stgNested[0].Tier -eq 'ERROR')) "count=$(@($stgNested).Count); tier=$($stgNested[0].Tier)"
  # The path gate is CASE-SENSITIVE (-ceq): a root case-variant `history.md`
  # must NOT inherit the exemption. (Codex BLOCKER review-20260710: -eq would
  # widen the waiver's accept-set beyond the documented root-History contract.)
  $stgCaseVar = @((Get-StagedFileFindings -Path 'history.md' -StagedText $histDated -AddedSet @{ 5 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'STG7e2: a count in a dated entry of a root CASE-VARIANT history.md STILL flags (case-sensitive path gate)' ((@($stgCaseVar).Count -eq 1) -and ($stgCaseVar[0].Tier -eq 'ERROR')) "count=$(@($stgCaseVar).Count); tier=$($stgCaseVar[0].Tier)"
  $stgDocInv = @((Get-StagedFileFindings -Path 'docs/notes.md' -StagedText $histDated -AddedSet @{ 5 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'INVENTORY' })
  Assert-True 'STG7f: the SAME count in a non-History .md still flags INVENTORY ERROR (exemption is History-scoped)' ((@($stgDocInv).Count -eq 1) -and ($stgDocInv[0].Tier -eq 'ERROR')) "count=$(@($stgDocInv).Count); tier=$($stgDocInv[0].Tier)"
  $histMagDated = "# Project History`n`n## 2026-07-09 - entry`n`nconvergence took 6 rounds of gate iteration"
  $stgHistMag = @((Get-StagedFileFindings -Path 'History.md' -StagedText $histMagDated -AddedSet @{ 5 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'STG7g: a transient (magnitude) count inside a dated root-History entry body is exempt (no MAGNITUDE finding)' (@($stgHistMag).Count -eq 0) "count=$(@($stgHistMag).Count)"
  $histMagPre = "# Project History`n`nconvergence took 6 rounds of gate iteration`n`n## 2026-07-09 - entry`n`nbody"
  $stgHistMagPre = @((Get-StagedFileFindings -Path 'History.md' -StagedText $histMagPre -AddedSet @{ 3 = $true } -Resolver $stgRes -IsTopLevel $stgTop) | Where-Object { $_.Class -eq 'MAGNITUDE' })
  Assert-True 'STG7h: a transient count in the History.md PREAMBLE still flags MAGNITUDE (advisory; dated-entry-only waiver)' (@($stgHistMagPre).Count -ge 1) "count=$(@($stgHistMagPre).Count)"

  # ---- E2E findings-exit contract (SUBPROCESS-level) ----
  # The in-memory TIER fixtures assert finding OBJECTS; only a child-process
  # invocation pins the PROCESS EXIT the pre-commit pre-pass switches on
  # (3 = error-tier findings, 0 = clean/advisory-only). Runs this script as a
  # child against a temp in-repo doc (-Paths requires an in-repo path); the
  # doc is removed afterward. Skipped with a FAIL if the repo root cannot be
  # resolved -- the exit contract is gate-critical, so no silent skip.
  $e2eRepoRoot = (& git rev-parse --show-toplevel 2>$null)
  if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($e2eRepoRoot)) {
    Write-Host '[SelfTest] FAIL E2E: cannot resolve repo root for the exit-contract cases'
    $script:failures++
  } else {
    $e2eDoc = Join-Path $e2eRepoRoot.Trim() ('docs/authorlint-selftest-' + [guid]::NewGuid().ToString('N').Substring(0, 8) + '.md')
    try {
      Set-Content -LiteralPath $e2eDoc -Value "# probe`nsee scripts/codex/auto-review.ps1:42 here" -Encoding UTF8
      & powershell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -Paths $e2eDoc | Out-Null
      Assert-True 'E2E1: error-tier doc -> child process exits 3 (the pre-pass abort signal)' ($LASTEXITCODE -eq 3) "exit=$LASTEXITCODE"
      Set-Content -LiteralPath $e2eDoc -Value "# probe`nclean prose with no findings" -Encoding UTF8
      & powershell -NoProfile -ExecutionPolicy Bypass -File $PSCommandPath -Paths $e2eDoc | Out-Null
      Assert-True 'E2E2: clean doc -> child process exits 0' ($LASTEXITCODE -eq 0) "exit=$LASTEXITCODE"
    } finally {
      # Checked cleanup: a silently-orphaned probe doc would dirty the repo
      # while the suite still reports green, so a failed removal is a
      # self-test FAILURE, not a shrug.
      try {
        Remove-Item -LiteralPath $e2eDoc -ErrorAction Stop
      } catch {
        Write-Host "[SelfTest] FAIL E2E-cleanup: could not remove probe doc ${e2eDoc}: $($_.Exception.Message)"
        $script:failures++
      }
      if (Test-Path -LiteralPath $e2eDoc) {
        Write-Host "[SelfTest] FAIL E2E-cleanup: probe doc still present after removal: $e2eDoc"
        $script:failures++
      }
    }
    # E2E3/E2E4: the STAGED-mode source-comment LOOP end to end -- staged-file
    # selection ($script:SourceExtPattern), index-blob read, per-file diff parse,
    # and the Get-StagedSourceFindings assembly the SRC*/SEL* unit tests do not
    # exercise together -- via Invoke-E2EStagedSource, which uses an ALTERNATE index
    # so the REAL worktree index is never mutated. E2E3: a comment inventory claim
    # surfaces as an advisory at exit 0. E2E4: a CODE-ONLY change has no
    # source-comment content, so it takes the "no content" report path rather than a
    # scanned-file result. (Codex TEST-QUALITY; code-only path + checked git
    # setup/cleanup.)
    $e2e3 = Invoke-E2EStagedSource -RepoRoot $e2eRepoRoot.Trim() -Content "// there are 223 icons for this feature`nfn probe() {}"
    Assert-True 'E2E3: alt-index git setup (read-tree + add) succeeded' ($e2e3.SetupOk) "text=$($e2e3.Text)"
    Assert-True 'E2E3: a staged .rs comment inventory claim surfaces via the staged source loop (advisory, exit 0)' (($e2e3.Exit -eq 0) -and ($e2e3.Text -match 'INVENTORY') -and ($e2e3.Text -match [regex]::Escape($e2e3.SrcRel))) "exit=$($e2e3.Exit); hasInv=$([bool]($e2e3.Text -match 'INVENTORY')); hasFile=$([bool]($e2e3.Text -match [regex]::Escape($e2e3.SrcRel)))"
    Assert-True 'E2E3-cleanup: all temp artifacts (alt-index, its .lock, probe src) removed' ($e2e3.CleanOk) ''
    $e2e4 = Invoke-E2EStagedSource -RepoRoot $e2eRepoRoot.Trim() -Content "fn probe() { let n = 1; }"
    Assert-True 'E2E4: alt-index git setup succeeded' ($e2e4.SetupOk) "text=$($e2e4.Text)"
    Assert-True 'E2E4: a CODE-ONLY staged .rs takes the no-content report path (fileCount not inflated), exit 0' (($e2e4.Exit -eq 0) -and ($e2e4.Text -match 'no staged narrative-doc or source-comment content')) "exit=$($e2e4.Exit); text=$($e2e4.Text.Trim())"
    Assert-True 'E2E4-cleanup: all temp artifacts removed' ($e2e4.CleanOk) ''
  }

  if ($script:failures -eq 0) {
    Write-Host '[SelfTest] All author-lint tests passed.'
    exit 0
  } else {
    Write-Host "[SelfTest] $script:failures failure(s)."
    exit 1
  }
}

# ===========================================================================
# Main
# ===========================================================================

# Resolve repo root from git so the linter works from any cwd (incl. a linked
# worktree). Fail loud if not in a work tree.
try {
  $repoRoot = (& git rev-parse --show-toplevel 2>$null)
  if ($LASTEXITCODE -ne 0 -or [string]::IsNullOrWhiteSpace($repoRoot)) {
    throw 'not inside a git work tree'
  }
  $repoRoot = $repoRoot.Trim()
} catch {
  Write-Host "[author-lint] ERROR: cannot resolve repo root: $_"
  exit 2
}

# Every git read that feeds validation MUST fail loud: a suppressed git failure
# treated as empty output would silently classify real findings as clean, the
# whole point of the linter. (Codex BLOCKER class.)
function Invoke-GitChecked {
  param([string[]]$GitArgs, [string]$What)
  $out = (& git @GitArgs 2>$null)
  if ($LASTEXITCODE -ne 0) {
    Write-Host "[author-lint] ERROR: 'git $($GitArgs -join ' ')' failed (exit $LASTEXITCODE) -- $What; refusing to report clean."
    exit 2
  }
  return $out
}

# Tracked file list (the INDEX), read ONCE and checked. Drives both the DEAD-REF
# top-level set and the index path resolver.
$indexFiles = Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'ls-files') -What 'tracked file list (git ls-files)'

# .gitignore from the INDEX only (the pending commit), never the worktree, so an
# unstaged .gitignore cannot change DEAD-REF classification. DISTINGUISH absent
# from failure: only read when .gitignore is TRACKED (in the index) -- then read
# through the checked helper so a real git failure fails loud rather than
# silently dropping ignored roots like `logs`. Untracked/staged-deleted ->
# genuinely empty. (Codex BLOCKER.)
$giText = ''
if ($indexFiles -contains '.gitignore') {
  $giText = (@(Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'show', ':.gitignore') -What '.gitignore index blob') -join "`n")
}

# Repo top-level DIRECTORY set for the DEAD-REF first-segment anchor -- built
# deterministically from the index + index .gitignore (Get-TopLevelDirSet), NOT
# a worktree listing. Directories only: a slash-list seeded by a root FILE
# ("PLAN.md/CLAUDE.md") is never a path.
$script:TopLevelDirs = Get-TopLevelDirSet -IndexFiles $indexFiles -GitignoreText $giText
$isTopLevel = {
  param($seg)
  return $script:TopLevelDirs.ContainsKey($seg)
}.GetNewClosure()

# DEAD-REF resolves against the INDEX (tracked files) for ALL modes, never the
# local filesystem: a `Test-Path` resolver would let a gitignored / untracked
# local-only artifact (e.g. a retained `logs/codex/probe-pack-45ff`) pass even
# though a fresh clone could not resolve it. The index path set is every tracked
# file plus its ancestor dir prefixes. (Codex BLOCKER.)
$script:IndexPathSet = Build-PathSetFromFileList -Files $indexFiles
$indexResolver = {
  param($relPath)
  $norm = ([string]$relPath -replace '\\', '/').TrimEnd('/')
  return $script:IndexPathSet.ContainsKey($norm)
}.GetNewClosure()

# Decide scope.
$mode = 'staged'
if ($Tree) { $mode = 'tree' }
elseif ($Paths -and $Paths.Count -gt 0) { $mode = 'paths' }
elseif ($Staged) { $mode = 'staged' }

$allFindings = @()
$fileCount = 0

if ($mode -eq 'staged') {
  # Validate the PENDING COMMIT (the index), not the worktree: read each staged
  # file's index blob, resolve referenced paths against the index, and compute
  # fence context from the FULL staged file -- then keep only findings on the
  # lines this commit ADDS. (Codex BLOCKERs: worktree headers/
  # paths could mask or invent a staged finding, and fence context from added
  # lines alone misreads a line added inside a pre-existing fenced block.)
  $stagedFiles = @((Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'diff', '--cached', '--no-color', '--no-ext-diff', '--no-textconv', '--name-only', '--diff-filter=ACMR') -What 'staged file list') |
                   Where-Object { $_ -match $script:NarrativeDocPattern })
  foreach ($path in ($stagedFiles | Sort-Object -Unique)) {
    $stagedRaw = (& git -C $repoRoot show ":$path" 2>$null)
    if ($LASTEXITCODE -ne 0) {
      # --diff-filter=ACMR already excluded deletions, so a failed index read is
      # UNEXPECTED -- fail loud rather than silently omit the file and report a
      # false-clean commit. (Codex BLOCKER.)
      Write-Host "[author-lint] ERROR: cannot read staged blob for '$path' (git show :$path failed)"
      exit 2
    }
    $stagedText = (@($stagedRaw) -join "`n")
    if (Test-IsGeneratedContent -Path $path -Content $stagedText) {
      Write-Host "[author-lint] skipped generated doc (not scanned): $path"
      continue
    }
    # Lines this commit adds (new-file line numbers). PIN the diff flags so user
    # `diff.noprefix` / color config cannot break the `+++ b/<path>` + hunk
    # parser (auto-review.ps1 documents that exact prior failure mode). (Codex
    # BLOCKER.)
    $diffText = (Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'diff', '--cached', '--no-color', '--no-ext-diff', '--no-textconv', '--unified=0', '--src-prefix=a/', '--dst-prefix=b/', '--', $path) -What "staged diff for $path") -join "`n"
    $addedByFile = Get-AddedRecordsFromDiff -DiffText $diffText
    # Shared with the SelfTest fixture: a MISSING parse key must yield an EMPTY set
    # (never a synthetic line 0) so the fail-loud guard below fires. (Codex BLOCKER.)
    $addedSet = Get-AddedLineSetForPath -AddedByFile $addedByFile -Path $path
    # Fail loud on a parse mismatch: the diff has IN-HUNK added lines but the parser
    # found none (an unexpected format) -> never report false-clean. A deletion-only
    # hunk legitimately has 0 added lines. Hunk-aware count (shared helper) so a
    # content `+++ b/...` line is included, not mistaken for a header.
    $rawAddedCount = Measure-InHunkAddedLines -DiffText $diffText
    if ($rawAddedCount -gt 0 -and $addedSet.Count -eq 0) {
      Write-Host "[author-lint] ERROR: parsed 0 added lines from a non-empty diff for '$path' (unexpected git diff format) -- refusing to report clean."
      exit 2
    }
    # A staged PLAN.md still needs whole-file TAG closure even with ZERO added
    # lines: a DELETION-only change that removes a defining `M<n>` header leaves
    # dangling references the closure must catch. Other docs with no added lines
    # have nothing to check. (Codex BLOCKER.)
    $isPlan = ([System.IO.Path]::GetFileName($path) -eq 'PLAN.md')
    if ($addedSet.Count -eq 0 -and -not $isPlan) { continue }
    $fileCount++
    $allFindings += Get-StagedFileFindings -Path $path -StagedText $stagedText -AddedSet $addedSet -Resolver $indexResolver -IsTopLevel $isTopLevel
  }
  # SOURCE-FILE COMMENT scanning (staged only): the count-drift classes over the
  # COMMENT lines a commit ADDS to .rs/.ps1/.toml/.sh files. Code lines are never
  # scanned; a comment count echoing a NEARBY code literal is exempt (Get-StagedSourceFindings).
  # Same index-read + fail-loud git-read pattern as the .md loop. Selection is the
  # only source-specific step (pinned by SEL1-3); the whole loop is exercised
  # end-to-end by E2E3, which uses an ALTERNATE index (GIT_INDEX_FILE) so the shared
  # worktree index is never mutated.
  $stagedSourceFiles = @((Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'diff', '--cached', '--no-color', '--no-ext-diff', '--no-textconv', '--name-only', '--diff-filter=ACMR') -What 'staged source file list') |
                         Where-Object { $_ -match $script:SourceExtPattern })
  foreach ($path in ($stagedSourceFiles | Sort-Object -Unique)) {
    $stagedRaw = (& git -C $repoRoot show ":$path" 2>$null)
    if ($LASTEXITCODE -ne 0) {
      Write-Host "[author-lint] ERROR: cannot read staged blob for '$path' (git show :$path failed)"
      exit 2
    }
    $stagedText = (@($stagedRaw) -join "`n")
    $diffText = (Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'diff', '--cached', '--no-color', '--no-ext-diff', '--no-textconv', '--unified=0', '--src-prefix=a/', '--dst-prefix=b/', '--', $path) -What "staged diff for $path") -join "`n"
    $addedByFile = Get-AddedRecordsFromDiff -DiffText $diffText
    $addedSet = Get-AddedLineSetForPath -AddedByFile $addedByFile -Path $path
    $rawAddedCount = Measure-InHunkAddedLines -DiffText $diffText
    if ($rawAddedCount -gt 0 -and $addedSet.Count -eq 0) {
      Write-Host "[author-lint] ERROR: parsed 0 added lines from a non-empty diff for '$path' (unexpected git diff format) -- refusing to report clean."
      exit 2
    }
    if ($addedSet.Count -eq 0) { continue }
    $ext = ([System.IO.Path]::GetExtension($path)).TrimStart('.').ToLowerInvariant()
    # Count this source file as SCANNED only when the commit adds COMMENT lines to
    # it. A code-only change has no source-comment content to examine, so it must
    # fall through to the "no content" report rather than inflate the scanned-file
    # count (which would mask the no-content path). (Codex QUALITY.)
    $srcSplit = $stagedText -split "`n"
    $srcRecs = @()
    for ($j = 0; $j -lt $srcSplit.Count; $j++) { $srcRecs += @{ Line = ($j + 1); Text = ($srcSplit[$j] -replace "`r$", '') } }
    $addedSrcComments = @(Get-SourceCommentRecords -Records $srcRecs -Ext $ext | Where-Object { $addedSet.ContainsKey([int]$_.Line) })
    if ($addedSrcComments.Count -eq 0) { continue }
    $fileCount++
    $allFindings += @(Get-StagedSourceFindings -Path $path -StagedText $stagedText -AddedSet $addedSet -Ext $ext)
  }
}
elseif ($mode -eq 'paths' -or $mode -eq 'tree') {
  if ($mode -eq 'tree') {
    $tracked = Invoke-GitChecked -GitArgs @('-C', $repoRoot, 'ls-files', '*.md') -What 'tracked markdown list'
    $targetList = @($tracked | Where-Object { $_ -match $script:NarrativeDocPattern })
  } else {
    $targetList = @($Paths)
  }
  # Canonicalize the repo root ONCE for the containment checks below (git emits a
  # forward-slash absolute path; GetFullPath normalizes separators).
  $repoRootFull = [System.IO.Path]::GetFullPath($repoRoot)
  foreach ($path in ($targetList | Sort-Object -Unique)) {
    # CONTAINMENT (BLOCKER): prove a caller-supplied -Paths value is
    # INSIDE the repo root BEFORE deriving its repo-relative form -- otherwise an
    # outside abs path becomes an unrelated in-repo SUFFIX, or a relative
    # `..\outside.md` escapes the repo under repo-index resolution, and a mispointed
    # invocation exits CLEAN against the WRONG file. Get-ContainedRelPath
    # canonicalizes (rooted as-is; relative resolved against the repo root,
    # preserving the repo-relative -Paths semantics), enforces containment, and
    # returns the repo-relative path or $null. (-Tree feeds git ls-files output,
    # already repo-relative + tracked, so it is contained; running it through the
    # same check is harmless belt-and-suspenders.)
    $rel = Get-ContainedRelPath -RequestedPath $path -RepoRootFull $repoRootFull
    if ($null -eq $rel) {
      Write-Host "[author-lint] ERROR: requested path '$path' is OUTSIDE the repo root '$repoRootFull' (or unresolvable) -- refusing to scan (a path outside the repo cannot be validated against the repo index, and silently scanning the wrong file would be a false-clean)."
      exit 2
    }
    if ($rel -notmatch $script:NarrativeDocPattern) { continue }
    $full = Join-Path $repoRoot $rel
    if (-not (Test-Path -LiteralPath $full)) {
      Write-Host "[author-lint] ERROR: target not found: $rel"
      exit 2
    }
    $content = Get-Content -LiteralPath $full -Raw -Encoding UTF8
    if (Test-IsGeneratedContent -Path $rel -Content $content) {
      Write-Host "[author-lint] skipped generated doc (not scanned): $rel"
      continue
    }
    $fileCount++
    $lines = $content -split "`n"
    $records = @()
    for ($i = 0; $i -lt $lines.Count; $i++) { $records += @{ Line = ($i + 1); Text = ($lines[$i] -replace "`r$", '') } }
    $planContent = if ([System.IO.Path]::GetFileName($rel) -eq 'PLAN.md') { $content } else { $null }
    $allFindings += Invoke-FileChecks -Path $rel -Records $records -PlanContentForClosure $planContent -Resolver $indexResolver -IsTopLevel $isTopLevel
  }
}

# An explicit audit (-Tree / -Paths) that examined NOTHING cannot claim "clean":
# a git-listing failure or a mispointed / non-Markdown selection would otherwise
# produce a false-clean exit 0. Staged mode with no narrative docs is a
# legitimate "nothing to check". (Codex BLOCKER.)
if (($mode -eq 'tree' -or $mode -eq 'paths') -and $fileCount -eq 0) {
  Write-Host "[author-lint] ERROR: $mode mode examined 0 files (git-listing failure, empty, or non-Markdown selection) -- cannot report clean."
  exit 2
}
if ($mode -eq 'staged' -and $fileCount -eq 0) {
  Write-Host '[author-lint] no staged narrative-doc or source-comment content to check.'
  exit 0
}

$report = Format-Report -Findings $allFindings -ScopeLabel $mode -FileCount $fileCount
Write-Host $report
if ($OutPath) {
  try { Write-ReportFile -Body $report -Path $OutPath }
  catch { Write-Host "[author-lint] ERROR: failed to write report to ${OutPath}: $_"; exit 2 }
}

# Error-tier findings exit 3, NOT 1: an uncaught crash under
# $ErrorActionPreference='Stop' makes `powershell -File` exit 1, so exit 1
# is effectively reserved as the crash signal. Using a distinct findings
# code lets the pre-commit pre-pass distinguish "real findings -> abort
# the commit" (3) from "the linter itself broke -> warn and fail open to
# the still-running AI gate" (1, or any other unexpected exit).
$errorCount = @($allFindings | Where-Object { $_.Tier -eq 'ERROR' }).Count
if ($errorCount -gt 0) { exit 3 } else { exit 0 }
