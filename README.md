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

- `quadruped-gait` from the `articara` repo.
- `misarta` from its own standalone repo (`github.com/takarakasai/misarta`) —
  the same git source `quadruped-gait` and `articara` use, so the shared
  `misarta` type unifies.
- `unitree-go2` + `unitree-rpc` from `unitree-sdk-rs`.

These are fetched over HTTPS from GitHub. [`.cargo/config.toml`](.cargo/config.toml)
sets `net.git-fetch-with-cli = true` so the system git (and your configured
credential helper, for private repos) handles the fetch.

The Go2 model itself (`go2.misa`, loaded at run time) is **not** a cargo
dependency — it lives in the `models/unitree_go2` git submodule, so clone with
`--recurse-submodules` (see Build). Override the path with `--misa PATH`.

## Build

```sh
git clone --recurse-submodules https://github.com/takarakasai/go2-gait-runner.git
cd go2-gait-runner
# (already cloned without --recurse-submodules? fetch the Go2 model submodule:)
#   git submodule update --init models/unitree_go2
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
[patch."https://github.com/takarakasai/articara.git"]
quadruped-gait = { path = "../articara/quadruped-gait" }

[patch."https://github.com/takarakasai/misarta.git"]
misarta = { path = "../misarta" }
```

## Usage

```sh
go2-gait-runner --help
# walk forward, capping swing-foot speed so a high 4-support fraction
# doesn't shake the body (0 disables the cap):
go2-gait-runner run eth0 --vx 0.05 --four-support 0.9 --max-swing-speed 3.0
```

## Learned-policy mode (`policy`)

Built with the default `policy` feature, the `policy` subcommand runs an
exported reinforcement-learning policy (ONNX, via the pure-Rust
[`tract`](https://github.com/sonos/tract) runtime) in place of the analytic
LinearCrawl controller. It reuses the same hardware glue (sport-mode release,
500 Hz `rt/lowcmd`, fold-down on exit); only the joint-target source changes.

```sh
# WASD/arrow-key teleop of a learned crawl:
go2-gait-runner policy eth0 --model exported/policy.onnx
# validate the model offline first (no robot needed):
go2-gait-runner policy x --model exported/policy.onnx --selftest
```

The policy is a small MLP exported from Isaac Lab. The runner reconstructs the
exact training contract:

- **Observation (45-d, proprioceptive)**, in Isaac joint order:
  `base_ang_vel(3) · projected_gravity(3) · velocity_commands(3) ·
  (joint_pos − default)(12) · joint_vel(12) · last_action(12)`. No scaling or
  normalization (trained with `actor_obs_normalization=False`). `base_lin_vel`
  is **deliberately absent** — it can't be measured cleanly under low-level
  control, so the policy is trained without it.
- **Action**: `q_des = default + 0.5 · action`, fed to the on-board PD at
  **kp=25, kd=0.5** (the trained gains), at **50 Hz** (held across 10 of the
  500 Hz ticks).
- **Joint order**: Isaac groups joints by type (all hips, thighs, calves); the
  Go2 SDK groups by leg (FR,FL,RR,RL). The conversion tables `ISAAC_TO_GO2` /
  `GO2_TO_ISAAC` (in `main.rs`) are verified against the live articulation.

Teleop: `W/S` or `↑/↓` forward/back, `A/D` strafe, `←/→` turn, `Space` stop,
`q`/`Esc` quit & fold. Commands are clamped to the trained crawl range
(vx ∈ [−0.3, 0.6], vy ∈ [±0.3], wz ∈ [±0.5]). `--no-keyboard` holds a fixed
`--vx/--vy/--wz`; `--duration S` auto-folds after S seconds.

> ⚠ **First hardware bring-up**: the Isaac (USD) joint *sign* convention is
> assumed to match the Go2 SDK directly (no per-joint flip). Before a full run,
> verify each joint moves in the expected direction at low `--kp` — a flipped
> sign will destabilize the policy.
