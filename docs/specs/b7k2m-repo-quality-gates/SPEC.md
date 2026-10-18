# Repository Quality Gates

Spec ID: b7k2m

## Context and Scope

The default branch accepted PR #1 because GitHub reported no branch protection for `main` and no repository rulesets. The only completed status check was `build`, so GitHub had no required review, signed-commit, direct-push, or required-check policy to enforce.

## Requirements

- REQ-001: `main` is the protected default branch. All changes enter through pull requests; direct pushes are blocked.
- REQ-002: Signed commits are required for protected-branch changes.
- REQ-003: Human review is optional. The GitHub-native required approving review count is zero; ordinary PR merges must not depend on an administrator override for missing approvals.
- REQ-004: The required GitHub status check is `Build`, declared repo-locally in `.github/quality-gates.json`.
- REQ-005: The `Build` workflow supports `pull_request`, `push` to `main`, and `merge_group` events so branch protection and merge queue style checks can consume the same required context.
- REQ-006: Native GitHub branch protection or rulesets enforce the declared policy. Workflow-only checks or README text do not count as enforcement.
- REQ-007: Release automation must not push version metadata directly to `main`; protected-branch version updates enter through signed maintainer PRs before publishing.
- REQ-008: Any intentional divergence between `.github/quality-gates.json` and GitHub protection state must be recorded as a waiver or reported as drift.

## Non-Goals

- This crate does not use PR label release intent. No `Label Gate` check is required until release intent is driven by PR labels.
- This spec does not change Rust driver behavior or public API.

## Verification

- VER-001 (covers: REQ-004, REQ-008): Parse `.github/quality-gates.json` and validate it with the style-playbook quality gate checker when available.
- VER-002 (covers: REQ-001, REQ-002, REQ-004, REQ-006): Inspect live GitHub rules for `main`: PRs, signed commits, and the strict `Build` check are required; force pushes remain blocked.
- VER-003 (covers: REQ-004, REQ-005): A PR changing the repository shows `Build` passing before merge. Inspect the workflow's PR, main, and merge-group triggers.
- VER-004 (covers: REQ-003, REQ-008): GitHub's required approving review count is zero and matches the repository declaration. The rule update preserves all unrelated parameters.
- VER-005 (covers: REQ-003, REQ-004): The README points maintainers to the repo-local quality gate contract and documents optional human review.
- VER-006 (covers: REQ-007): The release workflow publishes from the signed version already present on `main` and does not create protected-branch commits.

## Related ADRs

None
