# Repository Guidelines

## Project Structure & Module Organization
- `src/`: Core library code (`driver.rs`, `registers.rs`, `data_types.rs`, `error.rs`, `lib.rs`).
- `tests/`: Integration tests (e.g., `protocol_configuration_tests.rs`).
- `examples/`: Usage examples (may be empty; add runnable snippets here).
- `docs/`: Chip manuals and reference materials.
- `.github/workflows/`: CI for build/release; `lefthook.yml` defines pre-commit hooks.

## Build, Test, and Development Commands
- Build: `cargo build` (features: `cargo build --features async,defmt`).
- Lint: `cargo clippy --all-targets --all-features -D warnings`.
- Format: `cargo fmt` (check only: `cargo fmt -- --check`).
- Test: `cargo test` (with features: `cargo test --features async`).
- Docs: `cargo doc --no-deps` (open locally: `cargo doc --no-deps --open`).
- Hooks: `lefthook install` then rely on `pre-commit` to run fmt + clippy.

## Coding Style & Naming Conventions
- Rustfmt defaults (4-space indent, idiomatic imports). CI and hooks enforce formatting.
- Clippy must be clean with `-D warnings`.
- Naming: modules `snake_case`, types/traits `CamelCase`, constants `SCREAMING_SNAKE_CASE`.
- This crate is `#![no_std]`; avoid `std` and blocking calls. Gate optional code behind features (`async`, `defmt`).

## Testing Guidelines
- Use `cargo test` for unit/integration tests; prefer `embedded-hal-mock` for I2C.
- Place integration tests in `tests/` (`*_tests.rs`), or `mod tests` inside modules for unit tests.
- Tests should be deterministic and not rely on hardware.
- Add tests when changing register maps, protocol behavior, or error handling. No strict coverage threshold, but keep meaningful coverage of protocol branches and bitflags.

## Commit & Pull Request Guidelines
- Conventional Commits are required: `feat(driver): ...`, `fix(registers): ...`, `docs: ...`, `refactor: ...`, `chore: ...`, `ci: ...`.
- One logical change per commit; keep messages imperative and scoped (`feat(status): expose PD/Type-C getters`).
- PRs must include: clear description, why/how, linked issues, impacted features (`async`, `defmt`), and updated docs/examples if applicable.
- CI must pass (build, clippy, fmt, tests). Run hooks locally before pushing.

## Security & Configuration Tips
- Maintain `no_std` compatibility; prefer `core`/`alloc` (if ever added) over `std`.
- Keep dependencies with `default-features = false` unless required.
- Avoid panics in library code; return `Result<T, error::Error<I2cError>>`.
