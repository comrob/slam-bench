# Agent Change Log

This file stores concise, append-only summaries of repository changes made by automation/LLM agents.

Policy:
- Add entries here only after changes are tested/validated and approved by user.
- During implementation, track rationale/progress in `docs/changelog/WIP_CHANGE_CONTEXT.md`.

## Entry Format

```markdown
## YYYY-MM-DD HH:MM (UTC)
- Summary: <one-line change summary>
- Files: <comma-separated paths>
- Why: <reason for change>
- Validation: <commands/checks run or "not run">
- Notes: <limitations/follow-up, optional>
```

---

## 2026-03-03 00:00 (UTC)
- Summary: Initialized agent change log structure.
- Files: docs/changelog/AGENT_CHANGELOG.md
- Why: Establish persistent location for LLM-generated change summaries.
- Validation: not run
- Notes: Future agent edits should append entries; do not rewrite history.

## 2026-03-03 00:00 (UTC)
- Summary: Added changelog cross-links and made per-edit changelog append a mandatory LLM rule.
- Files: README.md, docs/LLM_PROJECT_RULES.md, docs/MIGRATION_CONTEXT.md, docs/changelog/AGENT_CHANGELOG.md
- Why: Provide persistent, centralized summaries of agent changes and enforce maintenance discipline.
- Validation: not run
- Notes: Changelog is append-only; new agent edits should add entries with the documented format.

## 2026-03-03 02:15 (UTC)
- Summary: Implemented approval-gated changelog governance with WIP tracking.
- Files: docs/LLM_PROJECT_RULES.md, docs/changelog/WIP_CHANGE_CONTEXT.md, README.md, docs/MIGRATION_CONTEXT.md, docs/changelog/AGENT_CHANGELOG.md
- Why: Prevent noisy/intermediate changelog entries during development; establish clear approval workflow for agent edits.
- Validation: Manual verification of cross-links and markdown formatting; WIP file tested as active task tracker.
- Notes: This entry itself demonstrates the approved workflow—implementation tracked in temporary WIP file, finalized after user approval.

## 2026-03-03 19:15 (UTC)
- Summary: Fixed docker-compose service override behavior in development mode.
- Files: docker-compose.yaml, docker-compose-dev.yaml
- Why: The dev compose file was completely replacing service definitions, stripping volume mounts and environment config. This prevented evaluate_trajectory from mounting reference.txt and caused evaluation pipeline failures.
- Validation: Tested ./orchestrate.sh --stages evaluate before and after fix. Before: container init failed trying to mount /reference_trajectory.txt. After: evaluation runs successfully with proper volume mounts, generates metrics and PDFs.
- Notes: Removed `profiles: ["evaluate", "full"]` from evaluate_trajectory (profiles were preventing direct service invocation). Simplified docker-compose-dev.yaml to only override `build` field, allowing service definitions from base compose to merge properly. All volume mounts, environment variables, and entrypoints now correctly inherited from docker-compose.yaml.
