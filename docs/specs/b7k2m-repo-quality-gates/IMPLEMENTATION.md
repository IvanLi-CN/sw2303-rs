# Implementation

## Current Shape

- `.github/quality-gates.json` declares the repo-local gate contract for `main`.
- `.github/workflows/build.yml` exposes a stable required check named `Build`.
- `.github/workflows/release.yml` publishes the signed version already present on `main` and pushes only the release tag.
- `README.md` documents the maintainer-facing quality gate.
- GitHub branch protection is aligned to the declaration after the required check exists on the pull request.

## Validation

- Parse `.github/quality-gates.json` as JSON.
- Run the style-playbook quality gate checker against the local declaration.
- Run `cargo fmt -- --check`.
- Run `cargo clippy --all-targets --all-features -- -D warnings`.
- Run `cargo test`.
- Verify GitHub protection state through `gh api` after remote alignment.
