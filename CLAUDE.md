# CLAUDE.md - AR4HMI Claude Contract

Read repository-root `AGENTS.md` in full before any task action. `AGENTS.md` is the authoritative project, safety, documentation, and cross-review contract. Stop if that file cannot be read.

Critical fail-closed rules:

- Never execute or import `AR4.py` during routine analysis or automated tests; module-level startup schedules serial connections.
- Treat serial writes, motion, homing, calibration, firmware flashing, and output activation as physical side effects requiring explicit operator authorization.
- Never equate static, simulated, mocked, or compiled results with live-arm verification.
- Claude-authored commits use `scripts/claude/commit.ps1`, which routes review to Codex.
- Codex-authored commits use Claude review by default; first-position `scripts/codex/commit.ps1 -NoClaude` is the explicit, audit-logged Codex-only fallback after Claude usage capacity is confirmed exhausted. Missing authentication must be repaired, and failed reviewer invocations never fail over automatically.
- Bare `git commit` and ordinary-work `CROSS_REVIEW_SKIP=1` usage are prohibited.
- Read `PLAN.md` before implementation and preserve all still-relevant specifications when updating that file.
