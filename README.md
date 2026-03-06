# go2-gait-runner

Headless runner that drives the [`quadruped-gait`](https://github.com/takarakasai/articara)
`LinearCrawl` controller on a **real Unitree Go2** over the low-level
`rt/lowcmd` interface (500 Hz), via [`unitree-sdk-rs`](https://github.com/takarakasai/unitree-sdk-rs).

This is the Go2-specific deployment app. The reusable gait algorithms and
the `.misa` model format live in the `articara` repo; this repo only
contains the hardware glue (motor ordering, sign tables, sport-mode
release, body-weight feedforward, telemetry).

## Dependencies

Self-contained: cargo fetches the dependencies from GitHub — no sibling
checkout needed.

- `quadruped-gait` + `misarta` from the `articara` repo (same git source,
  so the `misarta` type shared with `quadruped-gait` unifies; `misarta` is
  articara's submodule).
- `unitree-go2` + `unitree-rpc` from `unitree-sdk-rs`.

These are private repos fetched over SSH using the `github.com-takarakasai`
host alias (see your `~/.ssh/config`). [`.cargo/config.toml`](.cargo/config.toml)
sets `net.git-fetch-with-cli = true` so the system git resolves that alias
and the misarta submodule (libgit2 cannot). Make sure SSH access works:

```sh
ssh -T git@github.com-takarakasai      # should greet you as takarakasai
```

## Build

```sh
git clone ssh://git@github.com-takarakasai/takarakasai/go2-gait-runner.git
cd go2-gait-runner
# Point cyclonedds-sys at a CycloneDDS install for your arch (the FFI
# bindings are committed for x86_64 + aarch64; only the .so is resolved
# here). The Unitree SDK2's bundled thirdparty libs work:
export UNITREE_SDK2_ROOT=/path/to/unitree_sdk2     # has thirdparty/{include,lib/<arch>}
#   or:  export CYCLONEDDS_HOME=/path/to/cyclonedds-install
cargo build --release            # fetches articara + unitree-sdk-rs from GitHub
```

`cyclonedds-sys` does not vendor `libddsc.so`, so the build resolves it from
`UNITREE_SDK2_ROOT/thirdparty/lib/<arch>` (or `CYCLONEDDS_HOME/lib/<arch>`).
The FFI bindings themselves are committed for x86_64 and aarch64; a new arch
needs `unitree-sdk-rs/tools/regen-bindings.sh` (bindgen + libclang). At run
time also set `LD_LIBRARY_PATH` to the same lib dir (see `doc/manual.md`).

### Local co-development against uncommitted articara changes

The git dependencies track `articara`/`unitree-sdk-rs` **main**. To build
against local, not-yet-pushed changes, add a path override in
`.cargo/config.toml` (do not commit) — for example:

```toml
[patch."ssh://git@github.com-takarakasai/takarakasai/articara.git"]
quadruped-gait = { path = "../articara/quadruped-gait" }
misarta = { path = "../articara/misarta" }
```

## Usage

```sh
go2-gait-runner --help
# walk forward, capping swing-foot speed so a high 4-support fraction
# doesn't shake the body (0 disables the cap):
go2-gait-runner run eth0 --vx 0.05 --four-support 0.9 --max-swing-speed 3.0
```
