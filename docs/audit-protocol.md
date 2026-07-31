# Cross-review surface audit protocol

Audit protocol for a **periodic task** that keeps the adversarial review
system from drifting. Run this manually from whatever session normally
edits the review surface; the audit is small enough that it does not
warrant its own automation layer.

> **Package scope note.** This document ships inside the
> `cross-review-gate/` package and describes the behavior of the
> **packaged** scripts under `cross-review-gate/scripts/`. As of v1.7.0 the
> packaged scripts carry the full cross-provider feature set: a Claude
> cross-provider merge pass, the `Resolve-MergePassMix` pass-mix resolver,
> the merge `-NoClaude` switch, the verdict identity header block (`DIFF-SHA256`
> + `REVIEW-TREE-OID` + `REVIEW-BACKEND` + `REVIEW-EFFORT` +
> `REVIEW-SEVERITY-CONTRACT`, stamped via
> `Get-DiffSha256` / `Resolve-EffectiveCodexEffort` -- provenance/forensics
> metadata; the exit-1 branch-QUALITY corroboration that had read these
> headers was removed with the 2026-07 severity contract), the per-run
> run-owned `-OutDir` layout, the `-ReasoningEffort` passthrough, the
> `auto-merge.ps1 -SelfTest` mode, and the two author-side tools
> (`author-lint.ps1`, `dispatch-checklist.ps1`). Audit the packaged scripts
> directly.

This project-local installation additionally provides the Codex-author commit
`-NoClaude` route documented below. Audit that extension with the installed
scripts rather than attributing the extension to the v1.7.0 package snapshot.

## Why this exists

The review system is implemented as a small set of trusted artifacts.
The system supports two backends (Codex and Claude) that cross-review
each other's work, routed by the `REVIEW_BACKEND` env var read by the
pre-commit hook. The audit covers both backends.

Codex backend:
- `scripts/codex/review-prompt-template.md` (shared with Claude; what each backend reads)
- `scripts/codex/auto-review.ps1` (Codex wrapper)
- `scripts/codex/auto-merge.ps1` (Codex ff-merge gate)

Claude backend:
- `scripts/claude/auto-review.ps1` (Claude wrapper; reuses the prompt template above)
- (no standalone `scripts/claude/auto-merge.ps1` -- the merge gate is the single
  `scripts/codex/auto-merge.ps1`, now CROSS-PROVIDER: it launches the codex and
  the claude wrapper concurrently and unions their verdicts, so a separate
  Claude merge-gate script is not needed)

Shared:
- `INSTALL.md` (fresh-clone setup, author routes, and environment controls)
- `bootstrap.ps1` (fail-closed per-clone dispatcher installer)
- `scripts/git-hooks/dispatcher` (HEAD-trusted per-clone dispatch template)
- `scripts/git-hooks/pre-commit` (the trigger; routes by `REVIEW_BACKEND`)
- `scripts/codex/commit.ps1` (Codex-as-implementer wrapper; defaults to `REVIEW_BACKEND=claude`, while first-position `-NoClaude` selects audit-logged `REVIEW_BACKEND=codex` after confirmed Claude usage-capacity exhaustion without skipping review)
- `scripts/claude/commit.ps1` (cross-review wrapper used by Claude-as-implementer; sets `REVIEW_BACKEND=codex`)
- `AGENTS.md` (the human-facing contract description)
- `scripts/codex/author-lint.ps1` (author-side MECHANICAL pre-pass, NOT a review
  wrapper -- it runs no Codex/Claude review, but the pre-commit hook runs it as a
  fail-fast pre-pass: author-lint EXITS 3 on error-tier findings, which the hook
  reads to ABORT the commit before the AI review; it encodes the
  script-detectable defect classes the gate
  repeatedly raises, so its `-SelfTest` fixtures and its AGENTS.md
  Review-Infrastructure row are audited here for drift against the same convention
  sources as the wrappers)
- `scripts/codex/dispatch-checklist.ps1` (recurrence-ranked dispatch checklist
  generator over the verdict logs -- it shares `analyze-blocker-trends.ps1`'s
  verdict filename-shape guard, UTF-8 I/O, AND per-file normalized-text dedupe,
  but ranks by the eight reviewer CATEGORIES (the analyzer dedupes the same way --
  mirrored, not forked); its `-SelfTest` fixtures and AGENTS.md
  row are audited here too)

The review system is itself code that drifts. Without periodic audit, the
hazards list grows stale (new BLOCKER archetypes accumulate in the verdict
log without being folded into the prompt), the convention sections drift
out of sync with your project's convention docs, and the wrapper's
behavior diverges from the prompt's documented contract. User-facing
observation: "the review keeps catching the same class of issue and the
prompt isn't learning from it."

The audit closes that feedback loop: periodically read the trend report
and the current surface, then edit the surface in place to fold in new
patterns. All edits go through the regular pre-commit gate.

## Goal

This protocol is about producing **working code that catches more real
defects upstream**, not about making the review faster or shorter. Cycle
counts and time-to-merge are not measured. Quality of merged code is the
only metric.

## Actor and cadence

**Actor:** whichever session edits review-system files.

**Cadence:** on demand. Reasonable triggers are (a) the trend report
shows a recurring BLOCKER archetype without a corresponding named hazard;
(b) a recent merge gate caught a class of finding that suggests
prompt/wrapper drift; (c) the user explicitly asks for a review-surface
audit.

The audit is **not** a per-commit step. It runs occasionally — likely
once per multi-week stretch of review activity, or after a significant
review-system change has bedded in.

## Inputs

Each audit run consumes:

1. **`docs/blocker-trends.md`** (or whatever output path you configure
   via `-OutPath`) — the trend report generated by
   `scripts/codex/analyze-blocker-trends.ps1`. Re-run the analyzer at the
   start of each audit with **`-SinceDays 30`** so the report's recency
   window matches the audit-criteria thresholds below. The committed
   copy may be stale by days, and the analyzer's default is
   no-mtime-cutoff (full history) which would conflate old archetypes
   with current ones at threshold-check time:
   `powershell scripts\codex\analyze-blocker-trends.ps1 -SinceDays 30`
2. **`scripts/codex/review-prompt-template.md`** — the current prompt
   (SHARED between both backends; format drift here affects both).
3. **`scripts/codex/auto-review.ps1`** — Codex wrapper (header docs,
   helper docstrings, SelfTest header, exit-code contract).
4. **`scripts/claude/auto-review.ps1`** — Claude wrapper (parallel
   shape; verdict-format contract must match the Codex wrapper exactly
   since the shared prompt and hook routing assume identical exit
   codes). Specifically check: `Get-VerdictExitCode` classifier
   behaviour stays in sync with the Codex copy, the consistency-doc
   fail-closed gate is still in place (gated on
   `CROSS_REVIEW_CONSISTENCY_DOC`; the precompute is not ported to this
   wrapper), and the locked tool set (`Read,Grep,Glob` only) has not relaxed.
5. **`scripts/codex/auto-merge.ps1`** — exit-code mapping (2026-07 severity
   contract: only BLOCKER blocks; `Resolve-MergeUnionExit` validates the
   backend exits to {0,2} and fails closed on anything else, incl. the
   RETIRED exit 1), the staged follow-up promotion (children stage QUALITY
   follow-ups via `CROSS_REVIEW_FOLLOWUPS_PENDING`; `Publish-ReviewFollowups`
   promotes to `logs/review-followups.md` only after the ff-merge succeeds),
   console messages, and the trusted-infra snapshot extracted
   from base. This is the single CROSS-PROVIDER merge gate (it launches the
   codex and claude wrappers concurrently and unions the verdicts); there is
   no separate `scripts/claude/auto-merge.ps1`. Audit its cross-provider pass
   mix (`Resolve-MergePassMix`), the `-NoClaude` switch, the per-run run-owned
   `-OutDir` layout, the `-ReasoningEffort` passthrough, and the
   consistency-doc codex-only-mix forcing (gated on
   `CROSS_REVIEW_CONSISTENCY_DOC`). Run its `-SelfTest`.
5a. **`scripts/codex/commit.ps1`** AND **`scripts/claude/commit.ps1`** —
   cross-review routing wrappers. These run BEFORE the pre-commit hook
   and normally set `REVIEW_BACKEND` to the OTHER backend (Codex implementer →
   Claude reviewer, and vice versa). After confirmed Claude usage-capacity
   exhaustion, the Codex wrapper's first-position `-NoClaude`
   route instead selects Codex review, sets the validated
   `CROSS_REVIEW_FALLBACK=codex-no-claude` marker, and relies on the hook to
   append the local audit record before review. Check `Resolve-CodexCommitRoute`
   for position-zero recognition, exact removal of that wrapper flag, and opaque
   forwarding of every later argument.
   Confirm that reviewer invocation failures never activate the fallback
   automatically. Check the bypass-flag parser (`Test-Bypass-Args`) for parity
   across the wrapper copies, and the
   `core.hooksPath` config-redirect guard (`Resolve-HooksPathGuard`
   classifier) for parity across every hand-kept copy -- both commit
   wrappers AND `bootstrap.ps1` (the installer classifies `core.hooksPath`
   the same way so it never reports success under a state the wrappers
   abort on). Run both `--self-test` suites (which also spawn a subprocess
   E2E pinning the production guard's abort on both a NON-empty and an
   empty/`core.hooksPath=` value),
   verify `REVIEW_BACKEND` and `CROSS_REVIEW_FALLBACK` save/restore + the `GIT_CONFIG*` scrub
   error-handling are not regressed, and confirm the `core.hooksPath`
   guard still runs INSIDE the commit `try/finally` so an abort restores
   the scrubbed env. The pre-
   commit hook forces `REVIEW_BACKEND=both` when a staged change
   touches either commit wrapper, defending against a malicious
   wrapper edit that would otherwise route its own review to itself;
   verify that defense is intact and that a simultaneous fallback request fails
   rather than weakening the forced dual-backend route. Run the pre-commit
   self-test coverage for invalid markers, backend mismatches, mandatory Codex
   review, and durable fallback-log creation.
5b. **`scripts/codex/author-lint.ps1`** — the author-side mechanical
   pre-pass. Runs no AI review (the AI gate stays authoritative), but the
   pre-commit hook runs it as a fail-fast pre-pass: author-lint EXITS 3 on
   error-tier findings, which the hook reads to ABORT the commit (exit 1 is
   reserved as author-lint's crash signal, so a crashed linter fails open to the
   AI gate). It encodes the
   script-detectable defect classes the gate repeatedly raises: at ERROR tier,
   line-anchor citations, self-narration of the pending change's own landing
   (`this merge`/`this landing`/`landed ... here` — ERROR only in `History.md`,
   the retrospective landing log), local-only review-artifact paths cited as
   proof (`logs/(codex|claude)/reviews` beside a proof/verification word), and
   exact inventory/count claims in `.md` prose (the verb-gated `there are <N>
   <word>` / `covers <N> <word>` shapes — magnitude-or-delete, never a corrected
   number; a count inside a dated entry body of the root `History.md` log is
   EXEMPT — the drift-only History exemption, though a count there mis-stating
   a live contract the same diff changes stays a factual finding); at ADVISORY
   tier, dead repo-relative references, dangling milestone
   tags, exact-magnitude transients (incl. spelled enumerated-case totals),
   machine-local install/wiring state, and the looser local-evidence forms. The
   INVENTORY + MAGNITUDE classes ALSO run one tier softer over the COMMENT lines
   of staged `.rs`/`.ps1`/`.toml`/`.sh` files, with a code-echo exemption for a
   count that matches an adjacent code literal. The LINE-ANCHOR class downgrades to advisory
   ONLY for a doc carrying the explicit frozen-snapshot MARKER — an HTML
   comment `<!-- frozen-snapshot -->` on its own line near the top, matched
   by `^\s*<!--\s*frozen-snapshot\s*-->\s*$` (case-insensitive) within the
   first ~30 lines, in normal Markdown context only — a marker inside a
   fenced code block or on a CommonMark indented-code line (>=4 leading
   spaces, or 0-3 spaces then a tab) renders as visible code and does NOT
   grant the exemption. Filename and date do NOT grant the exemption (no
   filename inference, no env-var denylist): a dated-in-name doc without the
   marker stays error-tier, and a non-dated doc with the marker is exempt.
   On any check-class change, re-derive its
   `-SelfTest` case ranges and run `-SelfTest`; confirm its AGENTS.md
   Review-Infrastructure row still matches.
5c. **`scripts/codex/dispatch-checklist.ps1`** — the recurrence-ranked
   dispatch-checklist generator over the verdict logs (it shares
   `analyze-blocker-trends.ps1`'s verdict filename-shape guard, UTF-8 I/O, AND
   per-file normalized-text dedupe (mirrored -- the analyzer dedupes the same way),
   but ranks by reviewer CATEGORY and emits structural-fields-
   only output, dropping agent-authored prose as untrusted input). Run its
   `-SelfTest`; confirm its AGENTS.md row and validation-surface assumptions
   are intact.
5d. **`INSTALL.md`, `bootstrap.ps1`, and
   `scripts/git-hooks/dispatcher`** — the fresh-clone installation surface.
   Confirm the installer obtains the dispatcher from the exact trusted `HEAD`
   blob, treats byte-different installed hooks as conflicts, rejects dirty
   templates, rejects configured `core.hooksPath`, and requires a trusted root
   `.gitignore` that excludes `logs/` before any target-repository write.
   Confirm dispatcher execution selects the trusted `HEAD` gate rather than the
   working candidate, rejects a missing gate by default, and records every
   permitted bootstrap or repair skip with the shared review-audit schema.
   Run `bootstrap.ps1 -SelfTest` and keep installation guidance synchronized
   with the executable checks.
6. **`scripts/git-hooks/pre-commit`** — the trigger hook. It is the
   `REVIEW_BACKEND` router and must keep both backends' trusted-copy
   bootstrap branches working (Codex backend's `REVIEW_INFRA` set and
   Claude backend's `REVIEW_INFRA` set are independent; per-backend
   tempdirs are tracked in a list and cleaned by a single EXIT trap).
   **Trust chain for hook edits:** the per-clone dispatcher installed
   by `bootstrap.ps1` at `.git/hooks/pre-commit` extracts
   `HEAD:scripts/git-hooks/pre-commit` to a temp file and execs THAT
   trusted copy — not the working-tree candidate. So a staged hook
   edit is executed under HEAD's last-good logic AND adversarially
   reviewed against HEAD's wrapper + prompt (the hook is in each
   backend's `REVIEW_INFRA`). The dispatcher itself lives per-clone
   outside the working tree; a commit cannot edit or remove it
   through the review flow. Hook edits still benefit from an
   out-of-band smoke check under both `REVIEW_BACKEND=codex` and
   `REVIEW_BACKEND=claude` because the running gate logic during the
   commit IS HEAD's (not the candidate's), so verifying the candidate
   logic works in isolation is the only way to confirm it before
   it becomes HEAD.
7. **`AGENTS.md`** — verdict severity contract, Codex Reviewer
   Operations section, Review Infrastructure table (now covers both
   backends).
8. **Recent verdict logs** — `logs/codex/reviews/*.md` AND
   `logs/claude/reviews/*.md` from the last ~14 days, for spot-checks
   across both backends (the audit does not re-parse them itself; the
   trend report is the aggregate signal). The analyzer's default
   `-ReviewsDir` is an array of both backend log directories -- the
   `param()` block declares `[string[]]$ReviewsDir =
   @('logs/codex/reviews', 'logs/claude/reviews')` (NOT a single codex-only
   string; the codex-only and claude-only forms shown in the usage examples are
   explicit per-backend slices, not the default) -- so the trend report
   aggregates findings across both reviewer backends by default; passing a single
   directory remains supported for per-backend slicing when needed (a
   non-existent path in the default list is skipped, so a single-backend install
   still works on the default).

The trend report's `UNCLASSIFIED` cluster is a specific input: a
populated `UNCLASSIFIED` bucket signals the analyzer's keyword list
needs growth.

## Audit criteria (priority order)

Work through these in order. Items at the top have the highest
impact-per-effort.

### 1. Hazards list reflects the trend report

The prompt template's `## Project hazards` section enumerates archetypes
the reviewer must scrutinize. Each hazard should correspond to concrete
codebase evidence or a BLOCKER cluster in the trend report.

**Audit:** for each BLOCKER archetype cluster in the trend report with
≥ 3 findings in the last 30 days, verify that the prompt template has a
named hazard for that archetype, with a real SYMBOL-ANCHOR example (the
function / type / system name involved) drawn from a verdict log -- NOT
a bare `file.ext:NNN` line number, which `author-lint.ps1` flags
error-tier in authored docs and which goes stale on the next edit.

**Drift signals:**
- A trend-report cluster with ≥ 3 recent findings has no corresponding
  named hazard in the prompt → add a hazard.
- A named hazard's symbol-anchor example no longer resolves (the cited
  function / type was renamed or removed) → refresh with a current
  symbol from a recent verdict.
- A populated `UNCLASSIFIED` cluster ≥ 5 findings → extend
  `analyze-blocker-trends.ps1`'s keyword list AND, if a new pattern is
  named, add a corresponding hazard.

### 2. "What is NOT a finding" section reflects false-positive history

The prompt template's `## What is NOT a finding` section enumerates
classes of observations Codex should NOT emit as findings. This section
should reflect false-positive patterns the project has discovered the
hard way.

**Audit:** scan the verdict log for findings that were later judged
not-defects (manual override, follow-up commit reverting the "fix",
etc.). For recurring false-positive patterns, the section should have a
guard line.

**Drift signals:**
- A specific false-positive pattern has recurred ≥ 2 times in the trend
  report's recent QUALITY/NOTE buckets without a corresponding "NOT a
  finding" guard line → add the guard.

### 3. Evidence-bundle components are consumed by Codex

`auto-review.ps1` builds `CODEX_REVIEW_EVIDENCE/` with several files
that contract with the prompt:

- `DIFF.patch`, `STAT.txt`, `NAME-STATUS.txt`, `COMMIT-LOG.txt`,
  `PLAN-CONSISTENCY.txt` — Codex is instructed by the prompt's
  scope-doc and/or hazards section to **read** these files.
- `SCOPE.txt` — wrapper-produced, but the wrapper also **injects** the
  same scope text directly into the prompt sent to Codex. Codex reads
  the injected text in its prompt, not the file. This is the
  established convention; do not flag SCOPE.txt as missing-from-prompt
  drift.

**Audit:** for each wrapper-produced evidence file in the
"prompt-reads-it" set above, the prompt's scope-doc section should
mention it AND name what Codex should do with it. The scope-doc's
`PLAN-CONSISTENCY.txt` line is the model; when the consistency check is
enabled (`CROSS_REVIEW_CONSISTENCY_DOC` set), the Project-hazards section
should also instruct the reviewer to read it first and treat each entry as
a BLOCKER candidate (the template's project-hazards guidance notes this).

**Drift signals:**
- A wrapper-produced evidence file in the prompt-reads-it set is not
  mentioned anywhere in the prompt → add a prompt mention.
- A prompt mention references a file the wrapper does not produce →
  either remove the prompt reference or implement the evidence file.

### 4. Verdict format contract adherence

The prompt requires a specific output structure: per-category
enumeration (8 categories, exactly once each), `VERDICT:` line (one of
three values), per-severity entries. The wrapper's `Get-VerdictExitCode`
parses this structure and fails closed on mismatches.

**Audit:** confirm the prompt and the wrapper agree on the contract.
Run all SelfTest suites and confirm each passes. Each suite's own success
banner / `-SelfTest` fixture block IS the authoritative inventory of what it
covers; do NOT trust (or maintain) a helper/fixture list copied into this doc --
re-derive coverage and any count from the suite itself on a fixture add. The
per-suite notes below capture only the DURABLE contracts (cross-suite invariants,
opt-in env behavior), not an enumeration:
- `powershell -NoProfile -ExecutionPolicy Bypass -File bootstrap.ps1 -SelfTest`
  (repository-local dispatcher installer. Durable checks cover the shared
  `core.hooksPath` classifier, Git-config environment isolation, top-level
  normalization, trusted-template and `logs/`-ignore enforcement,
  byte-identical idempotency, collision refusal, atomic `-Force` backup and
  replacement, pre-write rejection, executable trusted-gate dispatch,
  missing-gate rejection, shared skip-audit records, skip-audit failure
  closure, and cleanup postconditions. Child-process installs use throwaway
  repositories under TEMP and no reviewer backend.)
- `powershell scripts\codex\auto-review.ps1 -Scope SelfTest`
  (Codex wrapper. Consistency-doc fixtures run the helper in isolation with its
  default doc name; the runtime consistency check is opt-in via
  `CROSS_REVIEW_CONSISTENCY_DOC`. The `TCK-Git*` fixture shells out to `git`
  against an isolated throwaway repo under TEMP -- git + writable TEMP are package
  prerequisites, so it FAILS rather than skips if unavailable.)
- `powershell scripts\claude\auto-review.ps1 -Scope SelfTest`
  (Claude wrapper. Its verdict-format contract must classify IDENTICALLY to the
  Codex wrapper. The `Resolve-ConsistencyDocConfig` helper is byte-identical to
  the Codex/auto-merge copies (the fixture cases are equivalent though their
  bytes differ). The consistency precompute is NOT ported here; the Claude
  wrapper instead fails closed when the configured consistency doc is in the
  changed-path set -- opt-in via `CROSS_REVIEW_CONSISTENCY_DOC`. As in the Codex
  wrapper, the `TCK-Git*` fixture shells out to `git` against an isolated
  throwaway repo under TEMP -- git + writable TEMP are package prerequisites, so
  it FAILS rather than skips if unavailable.)
- `powershell scripts\codex\auto-merge.ps1 -SelfTest`
  (cross-provider merge gate. Covers pure in-memory helpers AND non-pure ones
  that write temp files / launch short-lived local `powershell.exe` children. No
  codex/network, but the `TCK-Git*` and `EB-*` fixtures DO shell out to `git`
  against isolated throwaway repos under TEMP (EB-Integration also invokes the
  script end-to-end with an omitted -Base; git + writable TEMP are package
  prerequisites, so they FAIL rather than skip if unavailable). The
  `Resolve-ConsistencyDocConfig`, `Test-ConsistencyDocKind`, `Get-GitObjectKind`,
  and `Remove-TreeNoRecurse` helpers are byte-identical to the review-wrapper
  copies; the PATH-assembly helpers `Get-PrependedToolPathParts`,
  `Get-UnionProcessPathParts`, and `Get-ProcessPathSpellings`
  (`Normalize-ProcessPathEnvForStartProcess` calls `Get-UnionProcessPathParts` on the
  spellings `Get-ProcessPathSpellings` ENUMERATES from the raw process block --
  `[Environment]::GetEnvironmentVariable` is case-insensitive, so a duplicate Path/PATH
  surface with differing dirs needs the block enumeration, not a two-lookup read -- and
  the dedup delegates to `Get-PrependedToolPathParts`) are byte-identical to the
  COMMIT-wrapper copies (`scripts/codex/commit.ps1`, `scripts/claude/commit.ps1`),
  which is where the other two copies of each live -- not the auto-review wrappers. The
  PlanInNameStatus fixtures run with the default doc name; the runtime
  codex-only-mix forcing is opt-in via `CROSS_REVIEW_CONSISTENCY_DOC`.)
- `powershell scripts\codex\commit.ps1 --self-test`
  (Codex commit wrapper: the hook-bypass-flag parser, the `core.hooksPath`
  config-redirect guard classifier, and the shared PATH-assembly helpers
  (`Get-PrependedToolPathParts`, `Get-UnionProcessPathParts`, `Get-ProcessPathSpellings`), plus a subprocess E2E that pins the production guard's
  abort against a throwaway scratch repo -- so git + a writable TEMP are exercised
  here too.)
- `powershell scripts\claude\commit.ps1 --self-test`
  (Claude commit wrapper: the same bypass-flag parser, `core.hooksPath` guard
  classifier, and PATH-assembly helpers + production E2E -- both wrappers carry
  parallel copies, so the two suites must stay in lock-step.)
- `sh scripts/git-hooks/pre-commit --self-test` (POSIX-sh entry, via Git-for-Windows
  `sh.exe`; git never passes args to a real pre-commit invocation, so this branch is
  reachable only when invoked explicitly)
  (the pre-commit HOOK's own hermetic fixture suite: builds a scratch git repo with
  STUB powershell infra and re-invokes the hook inside it, asserting the
  runtime-critical guard branches -- backend-required (incl. its ordering BEFORE the
  empty-commit exemption), invalid backend, the untracked commit-wrapper and
  untracked-infra guards, the author-lint pre-pass exit-code contract (3 = findings
  abort, 1 = crash warn-and-proceed, `CROSS_REVIEW_SKIP_AUTHOR_LINT=1` skip), and the
  semantic `-ReasoningEffort` declaration detect. Needs git + a POSIX sh + writable
  TEMP; no reviewer backend runs, the stub wrapper exits 0.)
- `powershell scripts\codex\analyze-blocker-trends.ps1 -SelfTest`
  (the archetype classifier, multi-directory aggregation, and multi-pass
  verdict-union + per-file normalized-text dedupe fixtures, plus an end-to-end
  fail-closed check.)
- `powershell scripts\codex\dispatch-checklist.ps1 -SelfTest`
  (the dispatch-checklist generator -- shares the analyzer's verdict
  filename-shape guard, UTF-8 I/O, AND per-file normalized-text dedupe (mirrored --
  the analyzer dedupes the same way), but ranks by reviewer CATEGORY.)
- `powershell scripts\codex\author-lint.ps1 -SelfTest`
  (the author-side mechanical pre-pass -- a binary pass/fail suite that prints
  `All author-lint tests passed.` on success.)

The analyzer's own SelfTest must also pass: keyword-list edits added
to close UNCLASSIFIED clusters need a corresponding fixture in the
analyzer's `Assert-Archetype` block so the new classification is
pinned and a future keyword move doesn't silently reroute the
archetype.

**Drift signals:**
- Prompt template lists severity tiers that the wrapper's classifier
  does not recognize, or vice versa → align them.
- SelfTest header claims N fixtures but the SelfTest block defines a
  different number → update the count (mechanical fix).
- `AGENTS.md` Verdict Severity Contract describes exit-code routing
  different from `auto-review.ps1`'s implementation → align them.

### 5. Cross-review the wrappers themselves (on-demand, not routine)

Each backend is best-positioned to catch drift in the OTHER backend's
wrapper, because the trade-offs each design makes are visible from
across the boundary in a way they aren't from inside. The cross-review
build-out of this package is the working existence proof: Codex's
adversarial review of the Claude wrapper caught structural defects
across many iterations (sandbox/auth trade-offs, missing deny-list
entries, env-leak patterns, clustered-short-option bypasses, ancestor-
chain assumptions, etc.) that the wrapper's author kept missing from
inside. Use the standard wrapper invocation against a specific commit:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\claude\auto-review.ps1 -Scope Commit -Target <sha>
powershell -NoProfile -ExecutionPolicy Bypass -File scripts\codex\auto-review.ps1  -Scope Commit -Target <sha>
```

(Same `-Scope Branch -Target <base> -Tip <branch>` shape works for
branch-scope spot-checks without going through `auto-merge.ps1`.)

**Cadence: ON-DEMAND, not routine.** The Claude backend consumes
subscription tokens noticeably, and the audit is supposed to be
lightweight. Run cross-wrapper review when:
- A wrapper has just landed a substantive change and you want a
  cross-backend pass before relying on it in production
- A complex / high-stakes commit warrants a second opinion (use the
  OTHER backend, not the same one that already gated)
- A periodic audit run has surfaced wrapper drift and you want
  detailed file:line citations on the affected wrapper

The Codex side has its own token pool (separate billing from Claude
subscription), so Codex-reviews-Claude is the lighter side of the
trade -- prefer that direction when the wrapper under audit is the
Claude wrapper. When the wrapper under audit is the CODEX wrapper,
Claude-reviews-Codex is the right direction (the OTHER backend
rule); use it sparingly given subscription token pressure. Both
directions are valid; neither should be set up on a schedule.

## How the audit ships

Edits to the review surface go through the regular pre-commit gate.
The gate's trusted-copy bootstrap is specifically narrow and applied
**per backend**: for each backend selected by `REVIEW_BACKEND`, the
hook's `REVIEW_INFRA` set is that backend's wrapper plus the shared
prompt template plus this hook (Codex backend: `scripts/codex/auto-review.ps1`
+ `scripts/codex/review-prompt-template.md` + `scripts/git-hooks/pre-commit`;
Claude backend: `scripts/claude/auto-review.ps1` + the same shared
prompt + the same hook). When any file in the selected backend's
`REVIEW_INFRA` is staged, that backend's wrapper + prompt are sourced
from HEAD and the review runs with the HEAD-trusted copies. So staged
edits to either backend's wrapper or the shared prompt are reviewed
against HEAD rather than self-approving.

**`AGENTS.md`, `INSTALL.md`, `bootstrap.ps1`,
`scripts/git-hooks/dispatcher`, `scripts/codex/auto-merge.ps1`,
`scripts/codex/analyze-blocker-trends.ps1`,
`scripts/codex/dispatch-checklist.ps1`, and
`scripts/codex/author-lint.ps1`** are reviewed by the normal gate path (no
trusted-copy bootstrap, since no backend's `REVIEW_INFRA` list includes these
paths). The selected backend's wrapper still runs from HEAD in this case
because the wrapper itself was not staged; normal review applies.

**`scripts/git-hooks/pre-commit` edits run under HEAD's logic, not
the candidate's.** Pre-commit is in `REVIEW_INFRA`, and the dispatcher
at `.git/hooks/pre-commit` (installed by `bootstrap.ps1`) extracts
`HEAD:scripts/git-hooks/pre-commit` and execs the HEAD copy — not the
working-tree candidate. So a staged hook edit is BOTH (a) executed
under HEAD's last-good logic (which catches drift the same way it
catches wrapper/prompt drift) AND (b) reviewed by HEAD's wrapper +
prompt. The candidate logic does not run as the gate until it
becomes HEAD on a subsequent commit. For pre-commit edits: still do
the out-of-band smoke commit + verdict-file spot-check under both
`REVIEW_BACKEND=codex` and `REVIEW_BACKEND=claude` so the candidate
logic is verified in isolation before it becomes HEAD.

Apply full-sweep discipline: every contract-touching edit must update
every cross-reference in one commit. Reactive single-spot patching
tends to ship at least one drift per iteration; sweep every reference
in one pass.

## Before redistributing this package

If you re-package this gate for another project, scrub the packaged scripts
and docs of YOUR project's identifiers FIRST, and search for each identifier in
ALL case and separator variants in one pass. A scrub keyed on a single form
(e.g. an uppercase `<PREFIX>_`) silently misses its lowercase and hyphenated
siblings (`<prefix>_`, `<prefix>-`), which then ship into the next consumer's
tree. Sweep, at minimum:

- **Crate / module / package prefix** — one case-insensitive tracked-tree
  match catches every variant: `git grep -niE '<prefix>[_-]'`.
- **Project name** — the bare word plus any CamelCase / kebab / snake forms.
- **Milestone- or release-tag scheme** — whatever tag shape your planning
  docs use (e.g. an `M<n>.<n>`-style scheme).
- **Internal task / worker / ticket IDs** — the id prefix in every form it
  appears (`<id>-0000`, worker/branch handles, etc.).

The package cannot know a consumer's tokens, so this stays a maintainer step.

## Out of scope (do not do these in the audit)

- Changing the gate's exit-code semantics or severity tier definitions.
  Those are the user's authority; the audit surfaces drift but does not
  rewrite the contract.
- "Optimizing" the review for speed. Time-to-verdict is not a measured
  property; a thorough review that takes longer is the desired behavior
  per the prompt's `## Review goal` section.
- Weakening any fail-closed path. Every documented fail-closed exit
  (malformed verdict, missing categories, etc.) stays.
- Suggesting `CROSS_REVIEW_SKIP=1` as a workaround
  for clusters the audit surfaces. That switch exists for specific
  narrow cases documented in your project's `AGENTS.md`; it is not an
  audit prerogative. (`-AllowNonBlocker` was REMOVED with the 2026-07
  severity contract — QUALITY no longer blocks a merge at all.)
