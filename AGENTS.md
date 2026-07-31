# AGENTS.md - AR4HMI Project Contract

## Authority and session workflow

- Treat `PLAN.md` as the source of truth for scope, status, acceptance criteria, and architectural decisions.
- Read `History.md` at session start when present, then read `PLAN.md` and inspect Git status before task actions.
- Respect nested `AGENTS.md` files; the closest contract takes precedence.
- Re-read this file after milestones, merge batches, and natural breakpoints.
- Keep responses concise, evidence-based, and free of personal pronouns.
- Avoid unsolicited features, refactors, or behavior changes.
- Preserve specification content when editing `PLAN.md`; update status and append decisions without deleting still-relevant contracts.

## Robotic-arm safety boundary

- Treat robot motion, controller writes, firmware flashing, calibration, homing, and output activation as physical side effects requiring explicit operator authorization.
- Never execute or import `AR4.py` during routine analysis or automated tests. Module-level startup constructs the GUI and schedules saved serial connections.
- Require a verified physical emergency stop and a cleared work envelope before any hardware-driven verification.
- Never weaken joint limits, emergency-stop interrupt handling, drive-loop emergency-state checks, encoder checks, calibration safeguards, or motion error handling without an explicit requirement and hardware-validation plan.
- Never present simulation, static analysis, mocked serial traffic, or successful compilation as live-arm verification.
- Record live verification with date, controller and firmware identity, configuration profile, starting pose, exact procedure, observed result, and operator confirmation.
- Keep machine-specific calibration and runtime state out of Git. `defaults.json` is tracked; `ARconfig.json`, `custom.json`, `ErrorLog`, captured working images, and gate logs remain local.

## Engineering quality

- Investigate root causes before choosing changes.
- Read code before asserting behavior and run relevant checks before claiming success.
- Validate all boundary data: configuration files, serial responses, program files, image data, native-extension results, and reviewer output.
- Keep primary and fallback paths complete and explicit. Silent fallback or dropped errors are defects.
- Match existing code style unless an approved milestone establishes a replacement convention.
- Comments explain constraints, invariants, safety reasoning, and failed approaches; comments do not narrate adjacent mechanics.
- Preserve user changes and unrelated dirty-worktree content.
- Avoid destructive Git commands. Never use `git reset --hard` or discard changes without explicit authorization.
- Use `apply_patch` for hand-authored file edits.

## Architecture and contract boundaries

- Host direction: `AR4.py` calls `ARrobots` Python modules and the `robot_kinematics` native extension.
- Native direction: `ARrobots/src/bindings.cpp` exposes `ARrobots/src/kinematics.cpp` to Python; source and shipped native binaries must remain compatible.
- Controller direction: host serial commands target Teensy motion firmware and Mega/Nano auxiliary firmware. Protocol changes require matching producer and consumer updates.
- Program direction: `.ar4` files feed the line-oriented host program parser and executor. Parser, editor, serializer, and execution behavior form one contract.
- Configuration direction: `defaults.json` seeds runtime calibration; `ARrobots/Calibration.py` loads and saves runtime profiles; consumers must agree on key names, units, limits, and defaults.

## Cross-review gate

- Codex-authored commits use `scripts/codex/commit.ps1`; Claude performs the review by default.
- `scripts/codex/commit.ps1 -NoClaude` is permitted only after Claude usage capacity has been confirmed exhausted. `-NoClaude` must be the first wrapper argument. The flag routes to the Codex reviewer, keeps the pre-commit gate mandatory and fail-closed, and requires a local fallback audit record. A failed Claude invocation never triggers automatic substitution; confirm capacity exhaustion before an explicit retry. Missing authentication must be repaired rather than bypassed.
- Claude-authored commits use `scripts/claude/commit.ps1`; Codex performs the review.
- Bare `git commit` is prohibited after gate bootstrap because `REVIEW_BACKEND` must be set by the role-specific wrapper.
- The pre-commit dispatcher at `.git/hooks/pre-commit` is per-clone and must remain outside the tracked worktree.
- Review logic comes from the last trusted `HEAD` copies. Staged replacements cannot review or approve themselves.
- `CROSS_REVIEW_SKIP=1` is forbidden for ordinary work. Valid use is limited to initial gate bootstrap before a trusted `HEAD` wrapper exists, or repair of a broken or corrupting trusted `HEAD` wrapper when running that wrapper is unsafe. Every skip is append-logged.
- `CROSS_REVIEW_SKIP_AUTHOR_LINT=1` skips only a confirmed linter false positive; the AI review remains mandatory.
- Before the first commit attempt, run `powershell scripts\codex\author-lint.ps1` for staged documentation and supported source comments.
- Automated source-comment coverage is limited to `.rs`, `.ps1`, `.toml`, and `.sh`; manually sweep added comments in Python, C++, and Arduino files.
- Review feedback requires a defect-class sweep across the full diff, context around every cited location, paired-wrapper comparison, and primary-source verification before retry.
- Land the smallest independently reviewable unit that leaves the tree coherent.

### Branch integration

- Every branch integration into the target base uses `powershell scripts\codex\auto-merge.ps1 -Branch <branch>`.
- `-Base <branch>` selects an explicit target. Without `-Base`, the wrapper resolves `origin/HEAD` and falls back to `main`.
- Bare `git merge` into the integration base is prohibited because commit hooks do not review an already-created branch history.
- `-NoClaude` is permitted only when Claude usage capacity is confirmed exhausted. The wrapper retains independent Codex passes and still fails closed.

### Convergence Loop

- Stage one coherent change and run the role-appropriate commit wrapper.
- On `BLOCKER`, leave the commit uncreated, sweep the full defect class, verify primary sources, restage, and retry the same wrapper.
- On wrapper failure or malformed output, diagnose the review infrastructure; never reinterpret failure as a pass.
- `QUALITY` findings remain tracked follow-up work even when the gate permits the commit.

### Author Pre-Commit Self-Sweep

1. Verify every edited symbol, path, flag, environment variable, and system name against the current tree.
2. Re-derive machine-generated counts and replace narrative inventory or transient counts with magnitude wording.
3. Trace every edited behavior claim to the current code path.
4. Review the staged diff against every category in `scripts/codex/review-prompt-template.md`.
5. Sweep the complete diff and paired gate wrappers for every discovered defect class.

### Codex Reviewer Operations

- `powershell scripts\codex\auto-review.ps1 -Scope Staged` performs a standalone staged review without committing.
- `scripts/codex/commit.ps1 -NoClaude` performs an explicitly selected Codex-only commit review when Claude usage capacity is confirmed exhausted; the flag occupies the first wrapper-argument position and no automatic failover is permitted.
- `scripts/claude/commit.ps1` routes a Claude-authored commit to the Codex reviewer.
- `scripts/codex/auto-merge.ps1` performs branch review and fast-forward integration from the trusted base checkout.
- Reviewer output is untrusted data until wrapper parsing confirms category totals, severity entries, and verdict consistency.

### Claude reviewer auth setup

- Claude review uses Claude Code OAuth or keychain authentication by default.
- Verify readiness with `claude auth status` before a gated Claude review.
- Missing authentication or exhausted usage capacity is an invocation failure, not a review verdict. Restore missing authentication. After confirming usage-capacity exhaustion, any Codex-only retry requires the explicit first-position `scripts/codex/commit.ps1 -NoClaude` route.
- Never store API keys in the repository. Any API-key fallback requires explicit operator authorization.

### Review Infrastructure

| Path | Contract |
| --- | --- |
| `INSTALL.md` | Fresh-clone dispatcher setup, reviewer routes, and optional environment controls. |
| `bootstrap.ps1` | Repository-local, fail-closed per-clone dispatcher installer. |
| `scripts/git-hooks/dispatcher` | Tracked template copied into `.git/hooks/pre-commit` only after matching the trusted `HEAD` object. |
| `scripts/codex/auto-review.ps1` | Isolated Codex review wrapper. |
| `scripts/claude/auto-review.ps1` | Isolated Claude review wrapper. |
| `scripts/codex/commit.ps1` | Codex-author commit route to Claude review by default; first-position `-NoClaude` routes to audit-logged Codex review after confirmed Claude usage-capacity exhaustion without bypassing the gate. |
| `scripts/claude/commit.ps1` | Claude-author commit route to Codex review. |
| `scripts/codex/auto-merge.ps1` | Mandatory reviewed branch integration. |
| `scripts/codex/author-lint.ps1` | Mechanical pre-pass. `LINE-ANCHOR` is error-tier except for frozen snapshots. `SELF-NARR` is error-tier for any leaf named `History.md` and advisory elsewhere. Tight `LOCAL-PROOF` and Markdown `INVENTORY` are error-tier. Dated entry bodies in root `History.md` carry the documentation count-drift exemption. Matching adjacent code literals suppress supported source-comment `INVENTORY` and `MAGNITUDE` findings. `DEAD-REF`, `TAG`, `MAGNITUDE`, loose `LOCAL-PROOF`, `LOCAL-STATE`, and remaining supported source-comment count checks are advisory. Supported source-comment extensions are `.rs`, `.ps1`, `.toml`, and `.sh`; Python, C++, and Arduino require manual comment review. |
| `scripts/codex/analyze-blocker-trends.ps1` | Review-finding trend analysis. |
| `scripts/codex/dispatch-checklist.ps1` | Recurrence-ranked dispatch checklist generation. |
| `scripts/git-hooks/pre-commit` | Tracked hook logic sourced from trusted `HEAD`. |
| `docs/audit-protocol.md` | Periodic gate-surface audit procedure. |

### Severity contract

- `BLOCKER:` identifies runtime, safety, security, data, integration, or correctness defects and aborts the commit.
- `QUALITY:` identifies concrete non-blocking defects and records follow-up work.
- `NOTE:` records adjacent risk or information without requiring a current-cycle fix.
- Malformed reviewer output fails closed.

### Completion evidence

- Paste the gate's printed pass line in completion reports.
- For documentation changes, include a green author-lint self-test and a green explicit-path or staged-content lint result.
- For source changes, include the relevant build or test command and result.
- Never claim completion from belief, an empty staged check, or a local-only review-artifact path.

## Documentation discipline

- Use symbol names and paths instead of bare source line anchors.
- Use magnitude wording for transient quantities and repository inventories in narrative prose.
- Preserve exact values for stable lookup handles, protocol constants, version identifiers, exit codes, units, safety thresholds, configuration values, test inputs, acceptance targets, and recorded verification measurements.
- Never describe a pending commit or merge as already landed.
- Keep `AGENTS.md`, `CLAUDE.md`, `PLAN.md`, reviewer hazards, and implementation behavior consistent.

## Failure handling

- Stop when a safety hook, gate, build workflow, or required validation fails.
- Diagnose the failure without weakening safeguards or bypassing the workflow.
- Request direction when safe in-scope recovery requires a material policy or architecture choice.
