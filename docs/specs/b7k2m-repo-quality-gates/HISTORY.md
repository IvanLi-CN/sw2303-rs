# History

## Origin

PR #1 merged while `main` had no GitHub branch protection and no repository rulesets. The repository needed a repo-local declaration plus GitHub-native enforcement so future merges cannot depend on operator memory.

## Decision

Use a single required check named `Build`, require pull requests for `main`, block direct pushes, and require signed commits. Keep `.github/quality-gates.json` as the source of truth and treat GitHub protection drift as a reportable failure.
