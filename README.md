# go2-gait-runner

Headless runner that drives the [`quadruped-gait`](https://github.com/takarakasai/articara)
`LinearCrawl` controller on a **real Unitree Go2** over the low-level
`rt/lowcmd` interface (500 Hz), via [`unitree-sdk-rs`](https://github.com/takarakasai/unitree-sdk-rs).

This is the Go2-specific deployment app. The reusable gait algorithms and
the `.misa` model format live in the `articara` repo; this repo only
contains the hardware glue (motor ordering, sign tables, sport-mode
release, body-weight feedforward, telemetry).

## Checkout layout

It depends on its siblings via path dependencies, so clone them next to
each other under a common parent:

```
<parent>/
  articara/          # provides quadruped-gait, misarta
  unitree-sdk-rs/    # provides unitree-go2, unitree-rpc, cyclonedds-sys
  go2-gait-runner/   # this repo
```

```sh
cd <parent>
git clone git@github.com-takarakasai:takarakasai/articara.git
git clone git@github.com-takarakasai:takarakasai/unitree-sdk-rs.git
git clone git@github.com-takarakasai:takarakasai/go2-gait-runner.git
# articara pulls misarta as a submodule:
git -C articara submodule update --init --recursive
```

## Build

```sh
cd go2-gait-runner
cargo build --release
```

`unitree-sdk-rs`'s `cyclonedds-sys` needs CycloneDDS + generated FFI
bindings. On a fresh x86_64 host without committed bindings, build with
`--features buildtime-bindgen` (needs `libclang`) or run
`unitree-sdk-rs/tools/regen-bindings.sh`. See that repo for DDS setup.

## Usage

```sh
go2-gait-runner --help
# walk forward, capping swing-foot speed so a high 4-support fraction
# doesn't shake the body (0 disables the cap):
go2-gait-runner run eth0 --vx 0.05 --four-support 0.9 --max-swing-speed 3.0
```
