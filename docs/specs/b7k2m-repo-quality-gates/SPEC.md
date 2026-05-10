# Repository Quality Gates

Spec ID: b7k2m

## Problem

The default branch accepted PR #1 because GitHub reported no branch protection for `main` and no repository rulesets. The only completed status check was `build`, so GitHub had no required review, signed-commit, direct-push, or required-check policy to enforce.

## Requirements

- `main` is the protected default branch.
- All changes to `main` enter through pull requests.
- Direct pushes to `main` are blocked.
- Signed commits are required for protected-branch changes.
- The required GitHub status check is `Build`, declared repo-locally in `.github/quality-gates.json`.
- The `Build` workflow supports `pull_request`, `push` to `main`, and `merge_group` events so branch protection and merge queue style checks can consume the same required context.
- Native GitHub branch protection or rulesets enforce the declared policy. Workflow-only checks or README text do not count as enforcement.
- Release automation must not push version metadata directly to `main`; protected-branch version updates enter through signed maintainer PRs before publishing.
- Any intentional divergence between `.github/quality-gates.json` and GitHub protection state must be recorded as a waiver or reported as drift.

## Non-Goals

- This crate does not use PR label release intent. No `Label Gate` check is required until release intent is driven by PR labels.
- This spec does not change Rust driver behavior or public API.

## Acceptance

- `.github/quality-gates.json` validates with the style-playbook quality gate checker.
- GitHub branch protection for `main` requires PRs, blocks direct pushes, requires signed commits, and requires the `Build` check.
- A PR changing the repository must show `Build` passing before merge.
- The README points maintainers to the repo-local quality gate contract.
- The release workflow publishes from the signed version already present on `main` and does not create protected-branch commits.
