# AGENTS.md

Canonical guide for AI coding assistants (and humans) working on
`lis2dw12-i2c`. This file is the single source of truth for project
conventions, build/test commands, and code-style rules. The shorter
`.github/copilot-instructions.md` defers to this document.

---

## What this crate is

- **Name / version:** `lis2dw12-i2c` 0.1.0 (`Cargo.toml`).
- **Purpose:** Platform-agnostic, `#![no_std]` Rust driver for the
  STMicroelectronics LIS2DW12 3-axis accelerometer over I²C. See
  `README.md` and the [datasheet][ds].
- **API style:** `async` only, built on `embedded-hal-async` 1.0
  (`I2c` + `DelayNs`) — see `src/lib.rs:14-16, 39-43`.
- **Repository:** <https://github.com/OpenDevicePartnership/lis2dw12-i2c>
  (`Cargo.toml:4`).
- **License:** MIT (`LICENSE`, `Cargo.toml:13`).
- **Edition:** 2024 (`Cargo.toml:12`).
- **MSRV:** Rust **1.85** (`Cargo.toml:14`; enforced by the `msrv` CI job
  in `.github/workflows/check.yml`).
- **`unsafe`:** forbidden crate-wide via `[lints.rust] unsafe_code =
  "forbid"` (`Cargo.toml:29`). Do not introduce `unsafe`.

[ds]: https://www.st.com/resource/en/datasheet/lis2dw12.pdf

## Repository layout

```
.
├── AGENTS.md                       <- this file
├── Cargo.toml                      <- package metadata, lints, deps
├── CODE_OF_CONDUCT.md
├── CODEOWNERS
├── CONTRIBUTING.md                 <- contribution + PR rules
├── LICENSE                         <- MIT
├── README.md                       <- short usage example
├── SECURITY.md
├── deny.toml                       <- cargo-deny config (licenses, sources)
├── rust-toolchain.toml             <- requests rustfmt + clippy components
├── rustfmt.toml                    <- max_width = 120
├── .github/
│   ├── copilot-instructions.md     <- points at AGENTS.md
│   └── workflows/
│       ├── check.yml               <- fmt / clippy / doc / hack / deny / test / msrv / semver
│       └── nostd.yml               <- thumbv8m.main-none-eabihf check
├── .vscode/
└── src/
    ├── lib.rs                      <- driver struct, async methods, conversions
    └── registers.rs                <- Register enum + bilge-based bitfields
```

There is **no** `examples/`, **no** `tests/`, **no** `benches/`. Tests
live as a `#[cfg(test)] mod tests` block inside `src/lib.rs` (see
`src/lib.rs` bottom). The `test` cfg disables `no_std` via
`#![cfg_attr(not(test), no_std)]` (`src/lib.rs:11`).

## Building and testing

All commands below are run from the repository root. Every command in
this section was executed against the working tree (Windows, stable
toolchain) and exited 0 unless noted.

### Pre-flight

```bash
rustup show          # picks up rust-toolchain.toml components
```

### The CI matrix, reproduced verbatim

These mirror `.github/workflows/check.yml` and `.github/workflows/nostd.yml`.
Run them before pushing:

| Job        | Command                                                              | Notes |
|------------|----------------------------------------------------------------------|-------|
| `fmt`      | `cargo fmt --check`                                                  | Stable toolchain. |
| `clippy`   | `cargo clippy -- -Dwarnings`                                         | Run on **stable and beta** in CI. Treat warnings as errors. |
| `doc`      | `RUSTDOCFLAGS="--cfg docsrs" cargo doc --no-deps --all-features`     | CI uses **nightly** for `doc_cfg`-style features; stable works for local sanity. PowerShell: `$env:RUSTDOCFLAGS="--cfg docsrs"; cargo doc --no-deps --all-features`. |
| `test`     | `cargo hack --feature-powerset test`                                 | Needs `cargo install cargo-hack`. `cargo test` also works because there are no features today. |
| `hack`     | `cargo hack --feature-powerset check`                                | Verifies feature combinations stay additive. |
| `deny`     | `cargo deny --all-features check`                                    | Needs `cargo install cargo-deny`. Config in `deny.toml`. |
| `msrv`     | `cargo +1.85 check`                                                  | MSRV from `Cargo.toml:14`. |
| `nostd`    | `rustup target add thumbv8m.main-none-eabihf && cargo check --target thumbv8m.main-none-eabihf` | Confirms the crate still builds for an embedded target. |
| `semver`   | `cargo install cargo-semver-checks && cargo semver-checks`           | Run before bumping the version. |

A minimal local pre-push loop:

```bash
cargo fmt --check
cargo clippy -- -Dwarnings
cargo test
cargo check --target thumbv8m.main-none-eabihf
```

### Notes about the test harness

- The single existing unit test (`tests::test_ff_dur` in `src/lib.rs`)
  uses `tokio` (`[dev-dependencies]`, `Cargo.toml:23`) and
  `embedded-hal-mock` (`Cargo.toml:22`) with the `embedded-hal-async`
  feature. Write new async tests with `#[tokio::test]` and the mock
  `I2c` / `DelayNs` from `embedded-hal-mock`.
- There are no integration tests under `tests/` and no doctests with
  runnable code (the README example is fenced as `rust,ignore`).

## Code conventions

- **Formatting:** `rustfmt` with `max_width = 120` (`rustfmt.toml`).
  Always run `cargo fmt` before committing — `cargo fmt --check` is a
  hard CI gate.
- **`no_std`:** the crate is `#![cfg_attr(not(test), no_std)]`
  (`src/lib.rs:11`). Do not pull in `std`-only crates or APIs. Allocator
  use is currently not required and should not be introduced
  casually — there is no `alloc` import today.
- **`unsafe`:** forbidden crate-wide. Do not add `unsafe { … }`,
  `unsafe fn`, or `unsafe impl`.
- **Async:** the public API is fully `async`. Use `embedded-hal-async`'s
  `I2c` and `DelayNs` traits; never block.
- **Clippy lint policy (`Cargo.toml:31-43`) — all `deny`:**
  `correctness`, `expect_used`, `indexing_slicing`, `panic`,
  `panic_in_result_fn`, `perf`, `suspicious`, `style`, `todo`,
  `unimplemented`, `unreachable`, `unwrap_used`. Practical consequences:
    - Do not use `.unwrap()`, `.expect(...)`, `panic!`, `todo!`,
      `unimplemented!`, or `unreachable!` in non-test code.
    - Index slices with `.get(..)` / pattern destructuring, not `arr[i]`.
    - If a panic is provably impossible, add a localized
      `#[allow(clippy::unwrap_used)]` with a `// panic safety: …`
      comment, exactly as done in `src/lib.rs:178-188`.
- **Error handling:** propagate `I2C::Error` via `?`; return
  `Result<_, I2C::Error>` from public driver methods (see every method
  on `Lis2dw12` in `src/lib.rs`).
- **Bitfields / registers:** authored with [`bilge`][bilge] (`bitsize`,
  `FromBits`, `DebugBits`). Add new registers in `src/registers.rs`
  following the existing `ControlReg1` / `ControlReg2` pattern: one
  `#[bitsize(8)]` struct per device register, doc-comment the bit
  layout, and re-export from `src/lib.rs` via `pub use registers::*;`.
- **Register enum:** keep `Register` (`src/registers.rs:5-43`) the
  single source of truth for register addresses, with a trailing
  comment giving the datasheet mnemonic (e.g. `Control1 = 0x20, //
  CTRL1`).
- **Naming:** snake_case methods, `CamelCase` enums/structs. Constants
  for bit masks use `SCREAMING_SNAKE_CASE` (e.g. `TAP_THRESHOLD_MASK` in
  `src/lib.rs:46-47`).
- **Doc comments:** every public item has a `///` doc comment. New
  public APIs must too — the `doc` CI job builds with `--cfg docsrs` and
  intolerantly surfaces broken intra-doc links.

[bilge]: https://docs.rs/bilge

## Driver specifics

- **I²C addressing:** the device responds to `0x18` (SA0 = GND) or
  `0x19` (SA0 = V+). The `SA0` enum and the `new` / `new_with_sa0_gnd`
  / `new_with_sa0_vplus` constructors make this explicit
  (`src/lib.rs:20-69`). New constructors should keep the SA0 choice
  explicit; never silently default to one address.
- **WHO_AM_I:** expected value `0x44`
  (`registers::WHO_AM_I_EXPECTED`, `src/registers.rs:3`). Use this for
  probe-style helpers if you add them.
- **Register access helpers:** prefer `read_reg`, `write_reg`,
  `read_regs`, `modify_reg_bits`, `modify_reg_field`
  (`src/lib.rs:77-130`) rather than open-coding `i2c.write_read`. They
  centralize the addressing and bit-mask semantics.
- **Sensor math:** unit conversions (`convert_acc_to_gs`,
  `convert_temp_reg_to_celsius`, …) live alongside their callers in
  `src/lib.rs` and operate on raw register integers. Keep new
  conversions pure (`fn`, not `async fn`) and unit-test them in the
  in-file `tests` module.
- **No `defmt` dependency today.** The README example uses
  `defmt::info!` only illustratively. Do not add `defmt` as a hard
  dependency without a feature flag and a `cargo hack` story.

## Commit & PR conventions

The repo's `copilot-instructions.md` and `CONTRIBUTING.md` define the
authoritative rules; the highlights:

- **Subject line:** capitalized, ≤ 50 characters, imperative mood
  ("Fix bug", not "Fixed bug"). Recent history complies — e.g.
  `Update LICENSE copyright and add AI attribution instructions (#11)`,
  `Add accelerometer self-test function to LIS2DW12 driver (#4)`
  (`git log --pretty=%s`).
- **Body:** separated from the subject by a blank line, wrapped at 72
  columns, explains *what* and *why*, not *how*.
- **AI attribution (required for any AI-assisted commit):** every such
  commit MUST include an `Assisted-by` trailer of the form

  ```
  Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL1] [TOOL2]
  ```

  e.g. `Assisted-by: GitHub Copilot:claude-opus-4.7`. Verify the model
  identity for the current session — do not hard-code a value from a
  previous one.
- **`Signed-off-by`:** AI agents MUST NOT add this trailer. Only humans
  can certify the DCO.
- **Clean history:** squash typo/format fixups; every commit must
  build cleanly without warnings (`CONTRIBUTING.md` "Clean Commit
  History").
- **PR etiquette (`CONTRIBUTING.md`):** open as a draft first, ensure
  the `.github/` workflows are green on the draft before requesting
  review. Squash-merge is disabled — keep your branch history tidy.
- **Regressions:** when filing one, run `git bisect` to identify the
  offending commit.

## What not to do

- Do not add `unsafe` code (crate-wide `forbid`).
- Do not introduce `panic!`, `unwrap()`, `expect()`, `todo!()`,
  `unimplemented!()`, `unreachable!()`, or raw slice indexing in
  non-test code — clippy will fail CI.
- Do not switch the crate to `std`. Do not add `alloc` without a clear
  justification and a feature gate.
- Do not change the public API or bump the version without thinking
  about `cargo-semver-checks` (CI job `semver`).
- Do not enable git's `core.autocrlf` in this checkout — all tracked
  files are LF (`git ls-files --eol`).
- Do not force-push shared branches; do not rewrite published history.
- Do not add a `Signed-off-by` trailer from an AI agent.
- Do not commit secrets, API keys, or generated artifacts (`target/`,
  `Cargo.lock` for libraries — currently absent and should stay so).

## How to find more context

- **Datasheet:** <https://www.st.com/resource/en/datasheet/lis2dw12.pdf>
  — authoritative register, timing, and behavior reference.
- **CI definitions:** `.github/workflows/check.yml`,
  `.github/workflows/nostd.yml` — the only ground truth for what "green
  CI" means.
- **`embedded-hal-async`:** <https://docs.rs/embedded-hal-async>.
- **`bilge`:** <https://docs.rs/bilge> for the bitfield DSL used in
  `src/registers.rs`.
- **`embedded-hal-mock`:** <https://docs.rs/embedded-hal-mock> for
  writing async I²C unit tests.
- **OpenDevicePartnership contributing baseline:** `CONTRIBUTING.md`.

---

## Incorporated from `.github/copilot-instructions.md`

The following is the full prior content of
`.github/copilot-instructions.md` (the upstream version, prior to the
pointer note this PR adds). It is reproduced verbatim so that this
document is a strict superset of that file.

> # Copilot Instructions
>
> ## Commit Messages
> - Subject line: capitalized, 50 characters or less, imperative mood (e.g., "Fix bug" not "Fixed bug")
> - Separate subject from body with a blank line
> - Wrap body text at 72 characters
> - Use the body to explain *what* and *why*, not *how*
>
> ## AI Attribution
> Every commit that includes AI-generated or AI-assisted work **must** contain an `Assisted-by` trailer in the commit message:
> ```
> Assisted-by: AGENT_NAME:MODEL_VERSION [TOOL1] [TOOL2]
> ```
> Where:
> - `AGENT_NAME` is the name of the AI tool or framework (e.g., `GitHub Copilot`)
> - `MODEL_VERSION` is the specific model version used (e.g., `claude-opus-4.6`)
> - `[TOOL1] [TOOL2]` are optional specialized analysis tools used (e.g., `coccinelle`, `sparse`, `smatch`, `clang-tidy`)
> Basic development tools (git, cargo, editors) should not be listed.
> AI agents **must** verify their own identity (agent name and model version) before composing the `Assisted-by` trailer — do not assume or hard-code a model name from a previous session.
> AI agents **MUST NOT** add `Signed-off-by` tags. Only humans can certify the Developer Certificate of Origin.
