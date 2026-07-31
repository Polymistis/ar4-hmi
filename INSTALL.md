# AR4HMI Cross-Review Gate Installation

This repository carries the project-configured cross-review gate. Installation
adds only the per-clone dispatcher under `.git/hooks`; all review scripts,
prompts, and policy documents remain tracked project files.

## Step 1 — Prerequisites

- Git with a usable POSIX hook environment. Git for Windows supplies the
  required shell tools.
- Windows PowerShell 5.1 or a compatible PowerShell runtime.
- Claude Code authentication for the default Codex-author review route.
- Codex authentication for Claude-authored reviews and explicit Codex review
  operations.

## Step 2 — Clone the fork

```powershell
git clone https://github.com/Polymistis/ar4-hmi.git
Set-Location ar4-hmi
```

## Step 3 — Install the per-clone dispatcher

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\bootstrap.ps1
```

`bootstrap.ps1` requires the tracked dispatcher template and root `.gitignore`
to match `HEAD`, requires `logs/` to remain ignored, and requires effective
`core.hooksPath` to be unset. No tracked file or Git configuration is changed.
A byte-identical dispatcher makes installation idempotent.

A different existing `.git/hooks/pre-commit` causes a fail-closed abort.
Inspect that hook before using `-Force`; forced replacement is atomic and
creates a uniquely named backup beside the hook. `-Force` never bypasses the
trusted-template, root-ignore, or `core.hooksPath` checks.

Run installer verification independently:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\bootstrap.ps1 -SelfTest
```

## Step 4 — Verify reviewer authentication

```powershell
claude auth status
codex --version
```

Missing authentication is an invocation failure. Repository files must never
contain API keys or authentication tokens.

## Step 5 — Use the author-side commit route

Codex-authored changes:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\author-lint.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\commit.ps1 -m "scope: summary"
```

Claude-authored changes:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\author-lint.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\claude\commit.ps1 -m "scope: summary"
```

Bare `git commit`, `--no-verify`, and ordinary-work
`CROSS_REVIEW_SKIP=1` usage violate the project contract. After confirmed
Claude usage-capacity exhaustion, the explicit first-position Codex fallback
remains reviewed and audit-logged:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\commit.ps1 -NoClaude -m "scope: summary"
```

## Step 6 — Handle findings

`BLOCKER:` aborts the commit. Sweep the complete defect class, restage, rerun
author-lint, and retry the same wrapper. `QUALITY:` passes under the current
severity contract but remains tracked follow-up work. `NOTE:` records adjacent
risk or information. Malformed reviewer output fails closed.

## Step 7 — Integrate branches

Every integration into the target base uses:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\auto-merge.ps1 -Branch <branch>
```

Use `-Base <branch>` for an explicit target. Use `-NoClaude` only after
confirmed Claude usage-capacity exhaustion. Bare integration merges are
prohibited by `AGENTS.md`.

## Step 8 — Run gate verification

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\author-lint.ps1 -SelfTest
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\auto-review.ps1 -Scope SelfTest
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\claude\auto-review.ps1 -Scope SelfTest
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\auto-merge.ps1 -SelfTest
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\analyze-blocker-trends.ps1 -SelfTest
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\codex\dispatch-checklist.ps1 -SelfTest
```

Run the hook fixture suite through Git for Windows shell when bare `sh` is not
on `PATH`:

```powershell
$gitRoot = Split-Path -Parent (Split-Path -Parent (Get-Command git).Source)
& (Join-Path $gitRoot 'bin\sh.exe') .\scripts\git-hooks\pre-commit --self-test
```

## Step 9 — Keep operational state local

Review verdicts, event streams, stderr captures, fallback records, skip
records, and generated checklists belong under the ignored `logs/` tree.
Machine calibration, runtime configuration, captured images, and delivered
archive inputs also remain outside tracked history as defined by `.gitignore`
and `AGENTS.md`.

## Step 10 — Optional environment controls

- `CROSS_REVIEW_PASSES` controls independent Codex review coverage when no
  explicit `-ReviewPasses` argument is present. Integer values are constrained
  to the supported `1..10` range. Empty, whitespace, or non-integer values fail
  closed; an explicit wrapper argument takes precedence.
- `CROSS_REVIEW_COMMIT_EFFORT` forwards a case-insensitive Codex effort tier
  from `minimal`, `low`, `medium`, `high`, or `xhigh`. An unset value leaves
  effort resolution to the Codex wrapper configuration. Claude review has no
  corresponding effort argument.
- `CROSS_REVIEW_CONSISTENCY_DOC` enables consistency checking for one
  repository-relative tracked file. Empty values, absolute paths, escaping
  paths, missing files, and directory targets fail closed with a diagnostic.
  Leave the variable unset when no consistency document is configured.
- `CROSS_REVIEW_PRUNE_TOOL` names an optional safe-delete script that accepts
  review-log paths as positional arguments. A set and valid path handles log
  pruning. An unset variable permits the documented direct `Remove-Item`
  fallback. A set-but-empty, malformed, or missing path warns and skips
  housekeeping; no direct-delete fallback occurs in that state.

Environment controls affect only the current process and descendants unless
configured persistently outside the repository.
