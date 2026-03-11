//! Headless Go2 gait runner.
//!
//! Loads the Go2 from a `.misa` file (no articara app dependency), auto-detects
//! its leg kinematics with [`quadruped_gait::auto_detect_kinematics_config`],
//! builds a gait controller, and — for now — dumps the joint trajectory offline
//! so it can be validated against the Go2 joint limits before any hardware run.
//!
//! Usage:
//! ```text
//! cargo run -p go2-gait-runner -- dump [misa_path]
//! ```
//! The hardware send path (rt/lowcmd via unitree-sdk-rs) is added in a later step.

use std::time::{Duration, Instant};

use misarta::model::Model;
use quadruped_gait::{
    auto_detect_kinematics_config, joint_signs, AnyGaitController, ControllerOutput, GaitConfig,
    GaitGenerator, GaitMode, GaitType, KneePattern, LegId, VelocityCmd, DEFAULT_FOOT_LINKS,
};
use unitree_go2::{
    init_lowcmd, joint, set_crc, topics, LowState, Participant, ReaderQos, WriterQos,
};

/// Live gait-visualization config (`--viz` / `--viz-key` / `--viz-rate` /
/// `--viz-endpoint`).
#[derive(Clone)]
struct VizCfg {
    enabled: bool,
    key: String,
    rate_hz: f64,
    /// Optional Zenoh **listen** endpoint (e.g. `tcp/0.0.0.0:7447`). Set this
    /// when multicast peer discovery isn't available (same host / WSL2 / no
    /// multicast); the viewer then connects to it. `None` = auto-discovery.
    endpoint: Option<String>,
}

/// Zenoh publisher for the live gait stream (`--viz`). Best-effort: publish
/// errors are ignored so visualization can never disturb the control loop.
#[cfg(feature = "viz")]
mod viz_pub {
    use quadruped_gait::viz::GaitVizFrame;
    use quadruped_gait::ControllerOutput;
    use zenoh::Wait;

    pub struct VizPublisher {
        session: zenoh::Session,
        key: String,
        seq: u64,
        period: u32, // publish every `period` control ticks
        since: u32,
    }

    impl VizPublisher {
        /// Open a Zenoh session publishing on `key` at ~`rate_hz`, given the
        /// control timestep `dt`. With `endpoint = Some(ep)` the session listens
        /// on `ep` (TCP) and disables multicast discovery — use this on hosts
        /// without working multicast (the viewer connects to `ep`).
        pub fn new(
            key: &str,
            rate_hz: f64,
            dt: f64,
            endpoint: Option<&str>,
        ) -> Result<Self, String> {
            let mut config = zenoh::Config::default();
            if let Some(ep) = endpoint {
                config
                    .insert_json5("listen/endpoints", &format!("[\"{ep}\"]"))
                    .map_err(|e| format!("zenoh listen endpoint '{ep}': {e}"))?;
                let _ = config.insert_json5("scouting/multicast/enabled", "false");
            }
            let session = zenoh::open(config)
                .wait()
                .map_err(|e| format!("zenoh open: {e}"))?;
            let period = ((1.0 / rate_hz.max(1.0)) / dt).round().max(1.0) as u32;
            Ok(Self {
                session,
                key: key.to_string(),
                seq: 0,
                period,
                since: 0,
            })
        }

        pub fn key(&self) -> &str {
            &self.key
        }

        /// Call every control tick; publishes a JSON [`GaitVizFrame`] at the
        /// configured (downsampled) rate. `signs` is the IK→model sign table
        /// (slot × joint, from `joint_signs`): the controller output is in the
        /// gait/IK convention, so we sign-correct the joints to the model/URDF
        /// convention — exactly what `output_to_go2` sends to the robot — so a
        /// viewer setting `joint_positions` directly renders the *commanded*
        /// pose (e.g. knees bend `<<`, not the IK-sign-flipped `>>`).
        pub fn publish(
            &mut self,
            t_s: f64,
            trunk_z: f64,
            out: &ControllerOutput,
            signs: &[[f64; 3]; 4],
        ) {
            self.since += 1;
            if self.since < self.period {
                return;
            }
            self.since = 0;
            let mut frame = GaitVizFrame::from_output(self.seq, t_s, trunk_z, out);
            for slot in 0..4 {
                for k in 0..3 {
                    frame.joints[3 * slot + k] *= signs[slot][k];
                }
            }
            self.seq += 1;
            if let Ok(json) = serde_json::to_vec(&frame) {
                let _ = self
                    .session
                    .put(&self.key, json)
                    .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                    .wait();
            }
        }
    }
}

/// Go2 standing-crouch joint angles (Menagerie `home` keyframe): the pose at
/// which the gait's nominal stance plane should sit.
const HOME_HIP: f64 = 0.0;
const HOME_THIGH: f64 = 0.9;
const HOME_CALF: f64 = -1.8;

/// 500 Hz control period.
const CONTROL_DT: f64 = 0.002;

/// Go2 joint limits (rad) from `go2.misa`, indexed hip/thigh/calf.
const LIMITS: [(f64, f64); 3] = [
    (-1.0472, 1.0472),   // hip
    (-1.5708, 3.4907),   // thigh
    (-2.7227, -0.83776), // calf
];

/// Build a standing configuration vector (length `model.nq`) by overriding the
/// 12 leg joints on top of the neutral pose.
fn build_home_q(model: &Model<f64>) -> Vec<f64> {
    let mut q = model.neutral_q();
    for leg in ["FL", "FR", "RL", "RR"] {
        set_joint(model, &mut q, &format!("{leg}_hip_joint"), HOME_HIP);
        set_joint(model, &mut q, &format!("{leg}_thigh_joint"), HOME_THIGH);
        set_joint(model, &mut q, &format!("{leg}_calf_joint"), HOME_CALF);
    }
    q
}

fn set_joint(model: &Model<f64>, q: &mut [f64], name: &str, v: f64) {
    if let Some(i) = model.joints.iter().position(|j| j.name == name) {
        q[model.q_idx[i]] = v;
    }
}

/// Map a quadruped-gait joint name to the Go2 LowCmd motor index.
/// Go2 hardware order is FR(0..2), FL(3..5), RR(6..8), RL(9..11) ×
/// (hip, thigh, calf) — note the FL/FR and RL/RR swap vs the gait's
/// FL/FR/RL/RR slot order.
fn go2_motor_index(name: &str) -> Option<usize> {
    let base = match name.get(..2)? {
        "FR" => 0,
        "FL" => 3,
        "RR" => 6,
        "RL" => 9,
        _ => return None,
    };
    let off = joint_kind(name)?;
    Some(base + off)
}

/// Parse a boolean-ish CLI flag.
fn parse_flag(s: &str) -> bool {
    matches!(s, "1" | "on" | "true" | "yes" | "ff" | "y")
}

/// Parsed command line: bare positionals plus `--key value` / `--key=value`
/// named flags. Flags whose name is in [`BOOL_FLAGS`] never consume the next
/// token (so `--ff --vx 0.02` works), and store `"true"` when present.
struct Cli {
    positionals: Vec<String>,
    flags: std::collections::HashMap<String, String>,
}

/// Named flags that act as presence booleans (no value consumed).
const BOOL_FLAGS: &[&str] = &[
    "ff", "grav-ff", "no-release", "restore", "smooth-swing", "level", "viz", "led-3support",
];

fn parse_cli(args: impl Iterator<Item = String>) -> Cli {
    let mut positionals = Vec::new();
    let mut flags = std::collections::HashMap::new();
    let mut it = args.peekable();
    while let Some(a) = it.next() {
        let Some(key) = a.strip_prefix("--") else {
            positionals.push(a);
            continue;
        };
        if let Some((k, v)) = key.split_once('=') {
            flags.insert(k.to_string(), v.to_string());
        } else if BOOL_FLAGS.contains(&key) {
            flags.insert(key.to_string(), "true".to_string());
        } else {
            // Consume the next token as the value unless it is another flag.
            match it.peek() {
                Some(n) if !n.starts_with("--") => {
                    flags.insert(key.to_string(), it.next().unwrap());
                }
                _ => {
                    flags.insert(key.to_string(), "true".to_string());
                }
            }
        }
    }
    Cli { positionals, flags }
}

impl Cli {
    fn str(&self, key: &str) -> Option<&str> {
        self.flags.get(key).map(|s| s.as_str())
    }
    fn f64(&self, key: &str) -> Option<f64> {
        self.flags.get(key).and_then(|s| s.parse().ok())
    }
    fn f32(&self, key: &str) -> Option<f32> {
        self.flags.get(key).and_then(|s| s.parse().ok())
    }
    fn flag(&self, key: &str) -> bool {
        self.flags.get(key).map(|s| parse_flag(s)).unwrap_or(false)
    }
}

/// 0 = hip, 1 = thigh, 2 = calf.
fn joint_kind(name: &str) -> Option<usize> {
    if name.contains("hip") {
        Some(0)
    } else if name.contains("thigh") {
        Some(1)
    } else if name.contains("calf") {
        Some(2)
    } else {
        None
    }
}

/// Slot index in the gait's FL/FR/RL/RR order.
fn gait_slot(name: &str) -> Option<usize> {
    match name.get(..2)? {
        "FL" => Some(0),
        "FR" => Some(1),
        "RL" => Some(2),
        "RR" => Some(3),
        _ => None,
    }
}

/// Convert a [`ControllerOutput`] to 12 Go2-ordered, sign-corrected joint
/// angles (model/URDF convention, radians). Errors on an unmappable joint name.
fn output_to_go2(out: &ControllerOutput, signs: &[[f64; 3]; 4]) -> Result<[f64; 12], String> {
    let mut q = [0.0f64; 12];
    for (name, q_ik) in out.iter_joint_targets() {
        let slot = gait_slot(name).ok_or_else(|| format!("bad joint name {name}"))?;
        let k = joint_kind(name).ok_or_else(|| format!("bad joint name {name}"))?;
        let mi = go2_motor_index(name).ok_or_else(|| format!("no motor for {name}"))?;
        q[mi] = q_ik * signs[slot][k];
    }
    Ok(q)
}

/// Folded lying pose in Go2 motor order (FR/FL/RR/RL × hip/thigh/calf), matching
/// `go2_stand`'s `LIE_POS`. Used for a safe folded exit.
const LIE_POS: [f64; 12] = [
    0.0, 1.36, -2.65, // FR
    0.0, 1.36, -2.65, // FL
    -0.2, 1.36, -2.65, // RR
    0.2, 1.36, -2.65, // RL
];

/// Seconds to ramp the start pose into the gait's nominal stance.
const RAMP_SECS: f64 = 2.0;
/// Seconds to ramp the forward velocity command up (and back down).
const ACCEL_SECS: f64 = 1.5;
/// Seconds to ramp the final stance into the folded pose on exit.
const FOLD_SECS: f64 = 2.0;

/// Full usage for every mode and flag (`-h` / `--help`).
fn print_help() {
    eprint!(
        "\
go2-gait-runner — drive quadruped-gait LinearCrawl on a real Unitree Go2.

USAGE:
  go2-gait-runner <mode> [<iface>] [flags]

MODES:
  dump            Offline: assert the gait stays within Go2 joint limits.
  intent          Offline: quantify forward displacement, foot sweep, lift.
  run    <iface>  Hardware: release sport_mode -> ramp -> in-place -> forward
                  -> fold; reads state back and prints a tracking/tilt summary.
  diag   <iface>  Alias for `run` (identical behaviour).
  release <iface> Deactivate sport_mode (native RPC; replaces go2_motion_ctrl).
  restore <iface> Re-select \"normal\" so the onboard controller takes over.
  checkmode <iface>  Print the currently active motion mode.
  util <cmd> <mode> <iface>  Auxiliary device commands (not gait playback):
                  util lidar <on|off> <iface>  toggle the L1 LiDAR
                  (publishes ON/OFF to rt/utlidar/switch).
                  util led <on|off|0..10> <iface>  head LED: on/off switch or
                  brightness 0..10 (vui service, white).
                  util led-color <white|red|yellow|blue|green|cyan|purple>
                  <iface> [secs] [flash_ms]  head LED colour (vui api 1007;
                  named palette only, no arbitrary RGB; secs default 5).

FLAGS (all optional; <iface> is the 1st positional for run/diag):
  --misa PATH       model .misa file        (default models/unitree_go2/go2.misa)
  --gait MODE       linear-crawl (default) | champ | mpc | centroidal-srbd |
                    full-centroidal. champ/mpc are closed-loop on real-robot
                    state (IMU + leg odometry); linear-crawl/champ are open-loop.
  --gait-type T     footfall pattern for champ/mpc: trot|walk|pace|bound|crawl
                    (default crawl for linear-crawl, trot otherwise; linear-crawl
                    ignores the pattern but uses the preset's timing)
  --vx V            forward speed, m/s       (run/diag default 0.0; intent 0.05)
  --inplace S       in-place phase, seconds  (default 3)        [run/diag]
  --forward S       forward phase, seconds   (default 4)        [run/diag]
  --kp K            position gain            (default 60)        [run/diag]
  --kd K            damping gain             (default 5)         [run/diag]
  --swing H         foot lift height, m      (default 0.04)
  --stance-height M trunk height above the feet, m (default 0.35)
  --cycle S         gait cycle period, s     (default: crawl preset)
  --four-support F  4-support fraction 0..1  (default: crawl preset)
  --sway M          lateral body-sway amplitude, m (default 0 = off)
  --stance-width M  widen the stance outward, m: bigger support, no trunk motion
  --smooth-swing    C2 swing profile: zero accel at lift-off/touchdown (gentler)
  --max-swing-speed V  cap peak swing-foot speed, m/s: auto-slows forward speed
                    so a high --four-support doesn't shake the body (default 3.0;
                    0 = disable = legacy unbounded). Slowing --cycle does NOT help.
  --led-3support    light the head LED during the 3-leg support period (and the
                    --led-margin window before/after it)            [run/diag]
  --led-margin S    seconds before/after the 3-support period to keep the LED
                    lit (default 0.1)                               [run/diag]
  --level           active IMU body-leveling: trim stance feet to hold trunk flat
  --level-gain G    leveling strength, signed (default 0.3; negate if it worsens)
  --ff              enable body-weight support feedforward      [run/diag]
  --ff-scale S      fraction of body weight to support, 1.0=full (default 1.0) [run/diag]
  --csv PATH        write full per-tick telemetry CSV           [run/diag]
  --viz             stream the generated gait over Zenoh for live viewing in
                    the articara GUI (key go2/gait/planned, JSON). On `intent`
                    it streams offline in real time (no robot)  [run/diag/intent]
  --viz-key K       Zenoh key to publish on (default go2/gait/planned)
  --viz-rate HZ     viz publish rate, Hz (default 100)
  --viz-endpoint EP Zenoh listen endpoint (e.g. tcp/0.0.0.0:7447) for hosts
                    without multicast (same PC / WSL2); the viewer connects to
                    it. Default: auto multicast discovery (works on a LAN).
  --no-release      do NOT auto-release sport_mode at startup   [run/diag]
  --restore         re-activate sport_mode after the run        [run/diag]
  -h, --help        show this help

EXAMPLE (validated on slippery flooring):
  go2-gait-runner run eth0 --vx 0.02 --cycle 2.5 --four-support 0.9 \\
      --swing 0.04 --kp 200 --kd 6 --ff
"
    );
}

fn main() {
    let cli = parse_cli(std::env::args().skip(1));
    if cli.flags.contains_key("help")
        || cli.positionals.iter().any(|p| p == "-h" || p == "--help")
    {
        print_help();
        return;
    }
    let mode = cli.positionals.first().map(|s| s.as_str()).unwrap_or("dump");
    let misa = cli
        .str("misa")
        .unwrap_or("models/unitree_go2/go2.misa")
        .to_string();

    // Shared gait tuning flags (used by run/diag/intent).
    let tune = GaitTune {
        swing_h: cli.f64("swing").unwrap_or(0.04),
        cycle_s: cli.f64("cycle"),
        four_support: cli.f64("four-support"),
        sway: cli.f64("sway"),
        smooth_swing: cli.flag("smooth-swing"),
        stance_width: cli.f64("stance-width"),
        max_swing_foot_speed: cli.f64("max-swing-speed"),
        stance_height: cli.f64("stance-height").unwrap_or(0.35),
        gait_mode: {
            let m = cli
                .str("gait")
                .map(|s| {
                    parse_gait_mode(s).unwrap_or_else(|| {
                        eprintln!("error: unknown --gait {s:?} (linear-crawl|champ|mpc|centroidal-srbd|full-centroidal)");
                        std::process::exit(1);
                    })
                })
                .unwrap_or(GaitMode::LinearCrawl);
            m
        },
        gait_type: {
            let mode = cli.str("gait").and_then(parse_gait_mode).unwrap_or(GaitMode::LinearCrawl);
            cli.str("gait-type")
                .map(|s| {
                    parse_gait_type(s).unwrap_or_else(|| {
                        eprintln!("error: unknown --gait-type {s:?} (trot|walk|pace|bound|crawl)");
                        std::process::exit(1);
                    })
                })
                .unwrap_or_else(|| default_gait_type(mode))
        },
    };

    match mode {
        "dump" => {
            // `dump [--misa P] [--stance-height M]`
            if let Err(e) = run_dump(&misa, tune) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        "intent" => {
            // `intent [--misa P] [--vx V] [--swing H] [--cycle S] [--four-support F]`
            // Offline: quantify the gait's forward displacement, foot sweep, lift.
            // With `--viz` it then streams the gait in real time (no robot) so
            // the articara GUI can preview it.
            let vx = cli.f64("vx").unwrap_or(0.05);
            let viz_cfg = VizCfg {
                enabled: cli.flag("viz"),
                key: cli
                    .str("viz-key")
                    .unwrap_or(quadruped_gait::viz::VIZ_KEY_PLANNED)
                    .to_string(),
                rate_hz: cli.f64("viz-rate").unwrap_or(100.0),
                endpoint: cli.str("viz-endpoint").map(|s| s.to_string()),
            };
            if let Err(e) = run_intent(&misa, vx, tune, &viz_cfg) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        "run" | "diag" => {
            // `<run|diag> <iface> [--misa P] [--vx V] [--inplace S] [--forward S]
            //   [--kp K] [--kd K] [--swing H] [--cycle S] [--four-support F]
            //   [--ff] [--ff-scale S]`
            let iface = cli.positionals.get(1).cloned().unwrap_or_default();
            if iface.is_empty() {
                eprintln!(
                    "usage: go2-gait-runner {mode} <iface> [--misa P] [--vx V] \
                     [--inplace S] [--forward S] [--kp K] [--kd K] [--swing H] \
                     [--cycle S] [--four-support F] [--ff] [--ff-scale S] [--csv PATH]"
                );
                std::process::exit(2);
            }
            let vx = cli.f64("vx").unwrap_or(0.0);
            let inplace = cli.f64("inplace").unwrap_or(3.0);
            let forward = cli.f64("forward").unwrap_or(4.0);
            let kp = cli.f32("kp").unwrap_or(60.0);
            let kd = cli.f32("kd").unwrap_or(5.0);
            let ff = cli.flag("ff") || cli.flag("grav-ff");
            let ff_scale = cli.f64("ff-scale").unwrap_or(1.0);
            // Active IMU body-leveling: off unless --level. --level-gain sets the
            // (signed) strength; negate it if leveling makes the tilt worse.
            let level_gain = if cli.flag("level") {
                cli.f64("level-gain").unwrap_or(0.3)
            } else {
                0.0
            };
            let csv = cli.str("csv");
            // Head-LED 3-support indicator: light the LED during the 3-leg
            // support period and `--led-margin` seconds before/after it.
            let led_3support = cli.flag("led-3support");
            let led_margin = cli.f64("led-margin").unwrap_or(0.1);
            let viz_cfg = VizCfg {
                enabled: cli.flag("viz"),
                key: cli
                    .str("viz-key")
                    .unwrap_or(quadruped_gait::viz::VIZ_KEY_PLANNED)
                    .to_string(),
                rate_hz: cli.f64("viz-rate").unwrap_or(100.0),
                endpoint: cli.str("viz-endpoint").map(|s| s.to_string()),
            };

            // Deactivate sport_mode before low-level control unless told not to.
            // Without this the onboard controller fights rt/lowcmd and the joints
            // oscillate. The RpcClient is created and dropped here, before the
            // gait participant comes up.
            if !cli.flag("no-release") {
                if let Err(e) = motion_release(&iface) {
                    eprintln!(
                        "error: failed to release sport_mode: {e}\n\
                         (pass --no-release if you released it some other way)"
                    );
                    std::process::exit(1);
                }
            }

            // `run` and `diag` are the same path now; both always read state back
            // and print the tracking/tilt summary. `diag` is kept as an alias.
            let res = run_hardware(
                &iface, &misa, vx, inplace, forward, kp, kd, tune, ff, ff_scale, level_gain, csv,
                &viz_cfg, led_3support, led_margin,
            );
            if let Err(e) = res {
                eprintln!("error: {e}");
                std::process::exit(1);
            }

            // The gait ends folded on the ground, so by default we leave
            // sport_mode off (safe). `--restore` hands control back afterwards.
            if cli.flag("restore") {
                if let Err(e) = motion_restore(&iface) {
                    eprintln!("warning: failed to restore sport_mode: {e}");
                }
            }
        }
        "release" => {
            let iface = require_iface(&cli);
            if let Err(e) = motion_release(&iface) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        "restore" => {
            let iface = require_iface(&cli);
            if let Err(e) = motion_restore(&iface) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        "checkmode" => {
            let iface = require_iface(&cli);
            match unitree_rpc::MotionSwitcher::new(&iface).and_then(|sw| sw.check_mode()) {
                Ok((form, name)) => {
                    if name.is_empty() {
                        println!("no mode active (sport_mode released)");
                    } else {
                        println!("active mode: name={name:?} form={form:?}");
                    }
                }
                Err(e) => {
                    eprintln!("error: {e}");
                    std::process::exit(1);
                }
            }
        }
        "util" => {
            // Auxiliary device/utility commands, grouped so they don't clutter
            // the gait-playback modes: `util <cmd> <mode> <iface>`.
            if let Err(e) = run_util(&cli) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        other => {
            eprintln!(
                "usage: go2-gait-runner <dump|intent|run|diag|release|restore|checkmode|util> ...   \
                 (got mode {other:?})"
            );
            std::process::exit(2);
        }
    }
}

/// `util <cmd> <mode> <iface>` — auxiliary device commands kept out of the
/// gait-playback path. Currently: `util lidar <on|off> <iface>`.
fn run_util(cli: &Cli) -> Result<(), String> {
    let cmd = cli.positionals.get(1).map(|s| s.as_str()).unwrap_or("");
    let mode = cli.positionals.get(2).map(|s| s.as_str()).unwrap_or("");
    match cmd {
        "lidar" => {
            let on = match mode {
                "on" => true,
                "off" => false,
                _ => {
                    return Err(format!(
                        "usage: go2-gait-runner util lidar <on|off> <iface>   (got mode {mode:?})"
                    ))
                }
            };
            let iface = cli.positionals.get(3).cloned().unwrap_or_default();
            if iface.is_empty() {
                return Err("usage: go2-gait-runner util lidar <on|off> <iface>".into());
            }
            util_lidar(&iface, on)
        }
        "led" => {
            // `util led <on|off|0..10> <iface>`: on/off toggles the head-LED
            // switch; a 0..10 level sets its brightness (white only — VUI has
            // no colour/RGB control).
            if mode.is_empty() {
                return Err("usage: go2-gait-runner util led <on|off|0..10> <iface>".into());
            }
            let iface = cli.positionals.get(3).cloned().unwrap_or_default();
            if iface.is_empty() {
                return Err("usage: go2-gait-runner util led <on|off|0..10> <iface>".into());
            }
            util_led(&iface, mode)
        }
        "led-color" | "led-colour" => {
            // `util led-color <color> <iface> [secs] [flash_ms]`: set the head
            // LED to a named palette colour via VUI api 1007 (`mode` carries the
            // colour). Optional positionals: hold duration (default 5 s) and a
            // flash cycle in ms.
            const USAGE: &str =
                "usage: go2-gait-runner util led-color <white|red|yellow|blue|green|cyan|purple> <iface> [secs] [flash_ms]";
            if mode.is_empty() {
                return Err(USAGE.into());
            }
            let iface = cli.positionals.get(3).cloned().unwrap_or_default();
            if iface.is_empty() {
                return Err(USAGE.into());
            }
            let secs = match cli.positionals.get(4) {
                Some(s) => s
                    .parse()
                    .map_err(|_| format!("secs must be a positive integer (got {s:?})"))?,
                None => 5,
            };
            let flash = match cli.positionals.get(5) {
                Some(s) => Some(
                    s.parse()
                        .map_err(|_| format!("flash_ms must be an integer (got {s:?})"))?,
                ),
                None => None,
            };
            util_led_color(&iface, mode, secs, flash)
        }
        "" => Err("usage: go2-gait-runner util <cmd> <mode> <iface>   (cmd: lidar|led|led-color)".into()),
        other => Err(format!("unknown util cmd {other:?} (supported: lidar, led, led-color)")),
    }
}

/// Toggle the Go2 L1 LiDAR via the `rt/utlidar/switch` topic. `set_on` waits up
/// to 2 s for the onboard utlidar node to match before publishing.
fn util_lidar(iface: &str, on: bool) -> Result<(), String> {
    let sw = unitree_go2::UtlidarSwitch::new(iface).map_err(|e| e.to_string())?;
    sw.set_on(on).map_err(|e| e.to_string())?;
    eprintln!(
        "LiDAR {} (published to rt/utlidar/switch)",
        if on { "ON" } else { "OFF" }
    );
    Ok(())
}

/// Go2 VUI service (`rt/api/vui/request`) — same RPC envelope as the
/// motion_switcher used for sport_mode. Drives the head LED (brightness /
/// switch / colour). 1001/1005 are verified against `unitree_sdk2`'s
/// `vui_api.hpp`; 1007 (SetLedColor) is an undocumented firmware api absent
/// from that header — same `vui` service and `{api_id, parameter}` envelope,
/// confirmed against the WebRTC community driver (go2_webrtc_connect).
const VUI_SERVICE: &str = "vui";
const VUI_API_ID_SET_SWITCH: i64 = 1001;
const VUI_API_ID_SET_BRIGHTNESS: i64 = 1005;
const VUI_API_ID_SET_LED_COLOR: i64 = 1007;

/// Fixed colour palette the VUI SetLedColor (1007) firmware accepts. The head
/// LED is not addressable RGB — only these named colours work; arbitrary
/// r/g/b is not supported by the robot.
const VUI_LED_COLORS: [&str; 7] = ["white", "red", "yellow", "blue", "green", "cyan", "purple"];

/// Head-LED control via the VUI service. `mode` is `on`/`off` (switch, api 1001,
/// `{"enable":0|1}`) or a `0..=10` brightness level (api 1005,
/// `{"brightness":N}`). VUI exposes brightness only — there is no colour/RGB.
fn util_led(iface: &str, mode: &str) -> Result<(), String> {
    let rpc = unitree_rpc::RpcClient::new(VUI_SERVICE, iface).map_err(|e| e.to_string())?;
    match mode {
        "on" | "off" => {
            let enable = i32::from(mode == "on");
            rpc.call(VUI_API_ID_SET_SWITCH, &format!("{{\"enable\":{enable}}}"))
                .map_err(|e| e.to_string())?;
            eprintln!("head LED {} (vui SetSwitch, enable={enable})", mode.to_uppercase());
        }
        _ => {
            let level: i32 = mode
                .parse()
                .map_err(|_| format!("led mode must be on|off|0..10 (got {mode:?})"))?;
            if !(0..=10).contains(&level) {
                return Err(format!("led brightness must be 0..10 (got {level})"));
            }
            rpc.call(
                VUI_API_ID_SET_BRIGHTNESS,
                &format!("{{\"brightness\":{level}}}"),
            )
            .map_err(|e| e.to_string())?;
            eprintln!("head LED brightness = {level}/10 (vui SetBrightness)");
        }
    }
    Ok(())
}

/// Head-LED colour via the VUI service (api 1007, `SetLedColor`). `color` must
/// be one of [`VUI_LED_COLORS`] (the firmware accepts only this named palette —
/// no arbitrary RGB). `secs` is how long the colour holds before the robot
/// reverts to its default; `flash` (ms) makes it blink — the firmware accepts a
/// cycle of `499..=secs*1000`, `None` keeps it solid.
fn util_led_color(iface: &str, color: &str, secs: u32, flash: Option<u32>) -> Result<(), String> {
    if !VUI_LED_COLORS.contains(&color) {
        return Err(format!(
            "led colour must be one of {VUI_LED_COLORS:?} (got {color:?})"
        ));
    }
    if secs == 0 {
        return Err("led colour duration (secs) must be >= 1".into());
    }
    let param = match flash {
        Some(ms) => {
            let max = secs * 1000;
            if !(499..=max).contains(&ms) {
                return Err(format!(
                    "led flash cycle must be 499..={max} ms for a {secs}s hold (got {ms})"
                ));
            }
            format!("{{\"color\":\"{color}\",\"time\":{secs},\"flash_cycle\":{ms}}}")
        }
        None => format!("{{\"color\":\"{color}\",\"time\":{secs}}}"),
    };
    let rpc = unitree_rpc::RpcClient::new(VUI_SERVICE, iface).map_err(|e| e.to_string())?;
    rpc.call(VUI_API_ID_SET_LED_COLOR, &param)
        .map_err(|e| e.to_string())?;
    match flash {
        Some(ms) => eprintln!(
            "head LED colour = {color} for {secs}s, flashing every {ms}ms (vui SetLedColor, api 1007)"
        ),
        None => eprintln!("head LED colour = {color} for {secs}s (vui SetLedColor, api 1007)"),
    }
    Ok(())
}

/// First positional after the mode, or exit with a usage error.
fn require_iface(cli: &Cli) -> String {
    let iface = cli.positionals.get(1).cloned().unwrap_or_default();
    if iface.is_empty() {
        eprintln!("usage: go2-gait-runner {} <iface>", cli.positionals.first().map(|s| s.as_str()).unwrap_or("release"));
        std::process::exit(2);
    }
    iface
}

/// Deactivate the onboard sport_mode controller via the motion_switcher RPC.
fn motion_release(iface: &str) -> Result<(), String> {
    let sw = unitree_rpc::MotionSwitcher::new(iface).map_err(|e| e.to_string())?;
    let n = sw.release().map_err(|e| e.to_string())?;
    eprintln!("sport_mode released ({n} mode(s) released); low-level control is now safe");
    Ok(())
}

/// Hand control back to the onboard sport_mode controller (selects "normal").
fn motion_restore(iface: &str) -> Result<(), String> {
    let sw = unitree_rpc::MotionSwitcher::new(iface).map_err(|e| e.to_string())?;
    sw.restore().map_err(|e| e.to_string())?;
    eprintln!("sport_mode restored (onboard controller will take a standing pose)");
    Ok(())
}

/// Gait tuning knobs shared by run/diag/intent. `None` keeps the crawl preset.
#[derive(Clone, Copy)]
struct GaitTune {
    swing_h: f64,
    cycle_s: Option<f64>,
    four_support: Option<f64>,
    /// Lateral body-sway amplitude (m). `None`/0 keeps the no-sway crawl.
    sway: Option<f64>,
    /// Use the C² (zero accel at lift-off/touchdown) vertical swing profile.
    smooth_swing: bool,
    /// Lateral stance widening (m). `None` keeps the detected stance.
    stance_width: Option<f64>,
    /// Swing-foot feasibility cap (m/s): auto-reduces forward speed so a high
    /// `four_support` doesn't make the swing foot move faster than the
    /// actuators can track (which shakes the body). `None` keeps the crawl
    /// preset default (3.0); `Some(0.0)` disables the guard.
    max_swing_foot_speed: Option<f64>,
    /// Trunk stance height (m): the height the body is held above the feet
    /// during the gait (LinearCrawl). Overrides the auto-detected nominal
    /// foot height. Default 0.35 m.
    stance_height: f64,
    /// Which controller to run (`--gait`). Default [`GaitMode::LinearCrawl`].
    gait_mode: GaitMode,
    /// Footfall pattern / preset (`--gait-type`) for the CHAMP / MPC modes;
    /// ignored by LinearCrawl (it uses its own diagonal sequence). Selects the
    /// [`GaitConfig`] preset via [`GaitConfig::for_type`].
    gait_type: GaitType,
}

/// Parse `--gait`. Accepts a few aliases per mode.
fn parse_gait_mode(s: &str) -> Option<GaitMode> {
    Some(match s.to_ascii_lowercase().as_str() {
        "linear-crawl" | "linearcrawl" | "linear" => GaitMode::LinearCrawl,
        "champ" => GaitMode::Champ,
        "mpc" => GaitMode::Mpc,
        "centroidal-srbd" | "centroidal" | "srbd" => GaitMode::CentroidalSrbd,
        "full-centroidal" | "full" | "fullcentroidal" => GaitMode::FullCentroidal,
        _ => return None,
    })
}

/// Parse `--gait-type`.
fn parse_gait_type(s: &str) -> Option<GaitType> {
    Some(match s.to_ascii_lowercase().as_str() {
        "trot" => GaitType::Trot,
        "walk" => GaitType::Walk,
        "pace" => GaitType::Pace,
        "bound" => GaitType::Bound,
        "crawl" => GaitType::Crawl,
        _ => return None,
    })
}

/// Default footfall pattern for a mode when `--gait-type` is omitted:
/// `crawl` for the (statically stable) LinearCrawl, `trot` otherwise.
fn default_gait_type(mode: GaitMode) -> GaitType {
    match mode {
        GaitMode::LinearCrawl => GaitType::Crawl,
        _ => GaitType::Trot,
    }
}

/// Zero feedforward torque.
const ZERO_TAU: [f64; 12] = [0.0; 12];

/// Build the LinearCrawl controller plus the model (for the body weight),
/// the standing `home_q`, and the IK→model sign table, from a `.misa` file.
fn build_gait(
    misa_path: &str,
    tune: GaitTune,
) -> Result<(Model<f64>, Vec<f64>, AnyGaitController, [[f64; 3]; 4]), String> {
    let parsed = misarta::native::load(misa_path).map_err(|e| format!("load {misa_path}: {e:?}"))?;
    let (model, _vis, _col) =
        misarta::native::build_model(&parsed.file).map_err(|e| format!("build_model: {e:?}"))?;
    let home_q = build_home_q(&model);
    let kin = auto_detect_kinematics_config(&model, &DEFAULT_FOOT_LINKS, &home_q)
        .map_err(|errs| format!("kinematics auto-detect failed: {errs:?}"))?;
    let signs = joint_signs(&model, &kin)?;
    // Select the preset for the requested footfall pattern (LinearCrawl
    // defaults to `crawl`). CHAMP / MPC modes use the pattern's phase offsets;
    // LinearCrawl ignores them but still uses the preset's cycle / swing /
    // four-support sizing.
    let mut cfg = GaitConfig::for_type(tune.gait_type).with_swing_height(tune.swing_h);
    if let Some(c) = tune.cycle_s {
        cfg = cfg.with_cycle_period(c);
    }
    if let Some(f) = tune.four_support {
        cfg = cfg.with_four_support_fraction(f);
    }
    if let Some(s) = tune.sway {
        cfg = cfg.with_lateral_sway(s);
    }
    cfg = cfg.with_smooth_swing(tune.smooth_swing);
    if let Some(w) = tune.stance_width {
        cfg = cfg.with_stance_width(w);
    }
    if let Some(m) = tune.max_swing_foot_speed {
        // `with_max_swing_foot_speed` clamps to >= 0, so `--max-swing-speed 0`
        // disables the guard (legacy unbounded swing).
        cfg = cfg.with_max_swing_foot_speed(m);
    }
    let mut ctrl = AnyGaitController::new(tune.gait_mode, cfg, kin);
    ctrl.set_knee_pattern(KneePattern::BothBack);
    // Hold the trunk at the requested stance height (overrides the
    // auto-detected nominal foot height). LinearCrawl only.
    ctrl.set_body_height_m(tune.stance_height);
    Ok((model, home_q, ctrl, signs))
}

/// Total robot weight (N) = Σ link mass × g. The misarta Go2 model is
/// fixed-base, so `compute_gravity` would only carry the leg-segment weight;
/// the body-support load that actually makes the legs sag has to be applied as
/// a distributed ground reaction (below).
fn body_weight_n(model: &Model<f64>) -> f64 {
    let m: f64 = model.inertias.iter().map(|i| i.mass).sum();
    m * 9.81 * REAL_WEIGHT_FACTOR
}

/// Empirically-calibrated ratio of the **real** Go2's supported weight to the
/// summed link weight of the misarta model. The fixed-base model
/// under-represents the trunk mass, so support FF computed from the raw model
/// weight under-supports the real robot by this factor (validated on hardware).
/// Folding it into [`body_weight_n`] makes `--ff-scale` a true fraction of real
/// body weight: `1.0` = full support, which is the default.
const REAL_WEIGHT_FACTOR: f64 = 1.73;

fn leg_slot(leg: LegId) -> usize {
    match leg {
        LegId::FL => 0,
        LegId::FR => 1,
        LegId::RL => 2,
        LegId::RR => 3,
    }
}

fn leg_base_motor(leg: LegId) -> usize {
    match leg {
        LegId::FR => 0,
        LegId::FL => 3,
        LegId::RR => 6,
        LegId::RL => 9,
    }
}

/// Static body-weight support feedforward (Nm per Go2 motor).
///
/// Distribute the total weight among the current stance feet as vertical
/// ground reactions `fᵢ = (0,0,fzᵢ)` (body frame) that balance both the weight
/// and the moment about the CoM, then convert each to joint torques via the
/// support relation `τ = −Jᵀ·f` (IK convention) and into the motor/model sign
/// convention. Swing legs get zero. Clamped for safety.
///
/// The vertical foot forces are the least-norm solution of the quasi-static
/// balance (with foot positions `(xᵢ,yᵢ)` and CoM `(cx,cy)` in the body frame):
/// ```text
///   Σ fzᵢ = W,   Σ fzᵢ·xᵢ = W·cx,   Σ fzᵢ·yᵢ = W·cy
/// ```
/// Writing `fzᵢ = λ₀ + λ₁·xᵢ + λ₂·yᵢ` (a plane over the support polygon), the
/// three constraints become `(AAᵀ)·λ = b` with `A` the `3×n` matrix of rows
/// `[1, xᵢ, yᵢ]`. This puts more load on the feet nearer the CoM, fixing the
/// equal-split's rear-hip under-support when the CoM is off-centre. Degenerate
/// geometry (collinear feet, `n<3`) falls back to an equal split.
fn support_tau_go2(
    out: &ControllerOutput,
    kin: &quadruped_gait::KinematicsConfig,
    signs: &[[f64; 3]; 4],
    weight_n: f64,
    com_xy: (f64, f64),
) -> [f64; 12] {
    // Stance feet with body-frame foot position and joint angles.
    let stance: Vec<(LegId, f64, f64, f64, f64, f64)> = out
        .legs
        .iter()
        .filter(|l| l.phase.is_stance)
        .map(|l| {
            let p = quadruped_gait::forward_leg_kinematics(
                kin.leg(l.leg),
                l.q_hip,
                l.q_thigh,
                l.q_calf,
            );
            (l.leg, p.x, p.y, l.q_hip, l.q_thigh, l.q_calf)
        })
        .collect();
    let n = stance.len();
    let mut tau = ZERO_TAU;
    if n == 0 {
        return tau;
    }

    // CoM-balanced vertical foot forces (least-norm), falling back to an equal
    // split on degenerate geometry.
    let (cx, cy) = com_xy;
    let (mut s1, mut sx, mut sy, mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for &(_, x, y, ..) in &stance {
        s1 += 1.0;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
        syy += y * y;
    }
    let aat = nalgebra::Matrix3::new(s1, sx, sy, sx, sxx, sxy, sy, sxy, syy);
    let b = nalgebra::Vector3::new(weight_n, weight_n * cx, weight_n * cy);
    let fz: Vec<f64> = match aat.try_inverse() {
        Some(inv) => {
            let lam = inv * b;
            stance
                .iter()
                .map(|&(_, x, y, ..)| (lam[0] + lam[1] * x + lam[2] * y).max(0.0))
                .collect()
        }
        None => vec![weight_n / n as f64; n],
    };

    for (i, &(leg, _, _, qh, qt, qc)) in stance.iter().enumerate() {
        let f = nalgebra::Vector3::new(0.0, 0.0, fz[i]);
        let jac = quadruped_gait::foot_jacobian_body(kin.leg(leg), qh, qt, qc);
        let tau_ik = -(jac.transpose() * f); // [hip, thigh, calf], IK convention
        let slot = leg_slot(leg);
        let base = leg_base_motor(leg);
        for k in 0..3 {
            tau[base + k] = (tau_ik[k] * signs[slot][k]).clamp(-18.0, 18.0);
        }
    }
    tau
}

/// Max per-leg foot-height trim the leveler may command (m), and the max joint
/// delta per joint (rad). Hard clamps so a wrong sign / large gain can only ever
/// nudge the posture, never command a big motion on hardware.
const LEVEL_MAX_DZ: f64 = 0.02;
const LEVEL_MAX_DQ: f64 = 0.08;

/// Active body-leveling correction (joint-angle deltas per Go2 motor).
///
/// Reads the measured trunk `roll` / `pitch` (rad, from the IMU) and trims each
/// **stance** leg's foot height to drive them toward zero — the closed-loop
/// counterpart to the open-loop gait. A foot at body-frame `(x, y)` gets a
/// vertical trim
///
/// ```text
///   dz = gain · (roll · y − pitch · x)
/// ```
///
/// (more-negative `dz` extends the planted leg, pushing that corner of the
/// trunk up). The trim is mapped to joint deltas through the inverse foot
/// Jacobian `dq = J⁻¹·[0,0,dz]`, then into Go2 motor order with the IK→motor
/// sign table — mirroring [`support_tau_go2`]. Swing legs are skipped (they
/// must follow their trajectory). Everything is clamped by [`LEVEL_MAX_DZ`] /
/// [`LEVEL_MAX_DQ`].
///
/// `gain` is signed: if leveling *increases* tilt on the robot (the IMU sign
/// convention is opposite), negate it. Start small (≈0.3) and raise until the
/// 3-leg-phase roll/pitch stops shrinking.
fn level_correction(
    out: &ControllerOutput,
    kin: &quadruped_gait::KinematicsConfig,
    signs: &[[f64; 3]; 4],
    roll: f64,
    pitch: f64,
    gain: f64,
) -> [f64; 12] {
    let mut dq = [0.0f64; 12];
    for l in out.legs.iter().filter(|l| l.phase.is_stance) {
        let p = quadruped_gait::forward_leg_kinematics(
            kin.leg(l.leg),
            l.q_hip,
            l.q_thigh,
            l.q_calf,
        );
        let dz = (gain * (roll * p.y - pitch * p.x)).clamp(-LEVEL_MAX_DZ, LEVEL_MAX_DZ);
        let jac = quadruped_gait::foot_jacobian_body(kin.leg(l.leg), l.q_hip, l.q_thigh, l.q_calf);
        let Some(inv) = jac.try_inverse() else {
            continue; // singular (rare); skip this leg's trim this tick
        };
        let dq_ik = inv * nalgebra::Vector3::new(0.0, 0.0, dz);
        let slot = leg_slot(l.leg);
        let base = leg_base_motor(l.leg);
        for k in 0..3 {
            dq[base + k] = (dq_ik[k] * signs[slot][k]).clamp(-LEVEL_MAX_DQ, LEVEL_MAX_DQ);
        }
    }
    dq
}

/// Offline: quantify what the gait *intends* — per-cycle forward trunk
/// displacement, the swing foot lift, and the stance foot fore/aft sweep.
fn run_intent(
    misa_path: &str,
    vx: f64,
    tune: GaitTune,
    viz_cfg: &VizCfg,
) -> Result<(), String> {
    let swing_h = tune.swing_h;
    let (model, home_q, mut ctrl, signs) = build_gait(misa_path, tune)?;
    let cycle = ctrl.config().cycle_period_s;

    // Report the body-weight support FF at the nominal stance (all 4 legs
    // down), so its sign and magnitude can be sanity-checked offline before
    // sending to hardware.
    let weight = body_weight_n(&model);
    let com = misarta::centroidal::compute_com(&model, &home_q);
    let com_xy = (com.x, com.y);
    eprintln!("CoM (body frame): x={:.4} y={:.4} z={:.4} m", com.x, com.y, com.z);
    ctrl.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
    let out0 = ctrl.tick(CONTROL_DT);
    let gtau = support_tau_go2(&out0, ctrl.kinematics(), &signs, weight, com_xy);
    eprintln!(
        "body weight = {weight:.1} N; stance support FF (Nm), Go2 order: FR[h,t,c]=[{:.2},{:.2},{:.2}] FL=[{:.2},{:.2},{:.2}] RR=[{:.2},{:.2},{:.2}] RL=[{:.2},{:.2},{:.2}]",
        gtau[0], gtau[1], gtau[2], gtau[3], gtau[4], gtau[5], gtau[6], gtau[7], gtau[8], gtau[9], gtau[10], gtau[11]
    );
    ctrl.reset();
    let n = ((2.5 * cycle) / CONTROL_DT).round() as usize; // ~2.5 cycles
    ctrl.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });

    eprintln!(
        "intent: vx={vx} swing_h={swing_h} cycle={cycle:.3}s — expecting ~{:.3} m/cycle forward",
        vx * cycle
    );
    eprintln!("t_s,body_x,trunk_y,FR_footx,FR_footz,FR_phase,FL_footz,RR_footz,RL_footz");

    let mut x0 = None;
    let (mut fr_x_min, mut fr_x_max, mut fr_z_max) = (f64::MAX, f64::MIN, f64::MIN);
    let (mut sway_min, mut sway_max) = (f64::MAX, f64::MIN);
    let mut last_x = 0.0;
    for step in 0..n {
        let out = ctrl.tick(CONTROL_DT);
        let bx = out.body_state.world_position.x;
        if x0.is_none() {
            x0 = Some(bx);
        }
        last_x = bx;
        let fr = out.leg(LegId::FR);
        // The trunk shift equals (nominal − commanded) foot-body Y; with no
        // sway it stays 0. Positive = trunk moved to body-left (+Y). Account for
        // stance widening (FR is on the right, so it shifts the planted foot by
        // −width) so the readout is true sway, not the static widen offset.
        let fr_widen = -tune.stance_width.unwrap_or(0.0);
        let by = (ctrl.kinematics().fr.nominal_foot_body.y + fr_widen) - fr.foot_body.y;
        sway_min = sway_min.min(by);
        sway_max = sway_max.max(by);
        fr_x_min = fr_x_min.min(fr.foot_body.x);
        fr_x_max = fr_x_max.max(fr.foot_body.x);
        // lift = how far the foot rises above its nominal stance z.
        let lift = fr.foot_body.z - ctrl.kinematics().fr.nominal_foot_body.z;
        fr_z_max = fr_z_max.max(lift);
        if step % 50 == 0 {
            let t = step as f64 * CONTROL_DT;
            eprintln!(
                "{t:.3},{bx:.4},{by:+.4},{:.4},{:+.4},{:?},{:+.4},{:+.4},{:+.4}",
                fr.foot_body.x,
                lift,
                fr.phase,
                out.leg(LegId::FL).foot_body.z - ctrl.kinematics().fl.nominal_foot_body.z,
                out.leg(LegId::RR).foot_body.z - ctrl.kinematics().rr.nominal_foot_body.z,
                out.leg(LegId::RL).foot_body.z - ctrl.kinematics().rl.nominal_foot_body.z,
            );
        }
    }
    eprintln!(
        "  lateral sway (trunk_y): {:.4}..{:.4} m (peak-to-peak {:.4} m)",
        sway_min,
        sway_max,
        sway_max - sway_min
    );
    let net = last_x - x0.unwrap_or(0.0);
    eprintln!(
        "\nsummary: net body_x advance over {:.2}s = {net:.4} m ({:.4} m/cycle)",
        n as f64 * CONTROL_DT,
        net / (n as f64 * CONTROL_DT / cycle)
    );
    eprintln!(
        "  FR foot fore/aft sweep = {:.4} m (x {:.4}..{:.4}), peak swing lift = {:.4} m (cmd swing_h={swing_h})",
        fr_x_max - fr_x_min,
        fr_x_min,
        fr_x_max,
        fr_z_max
    );
    if net.abs() < 1e-4 {
        eprintln!("  WARNING: body_x does not advance — forward intent is ~0 in the open-loop trunk.");
    }

    // ── Offline live-viz loop (`--viz`) ─────────────────────────
    // No robot: tick the gait in real time and publish each frame so the
    // articara GUI can preview the generated gait. Runs until interrupted.
    #[cfg(feature = "viz")]
    if viz_cfg.enabled {
        let mut viz = viz_pub::VizPublisher::new(
            &viz_cfg.key,
            viz_cfg.rate_hz,
            CONTROL_DT,
            viz_cfg.endpoint.as_deref(),
        )?;
        let trunk_z = if matches!(tune.gait_mode, GaitMode::LinearCrawl) {
            tune.stance_height
        } else {
            -ctrl.kinematics().fl.nominal_foot_body.z
        };
        eprintln!(
            "viz: streaming offline gait on zenoh key '{}' (~{} Hz){} — Ctrl-C to stop",
            viz_cfg.key,
            viz_cfg.rate_hz,
            viz_cfg
                .endpoint
                .as_deref()
                .map(|e| format!(", listening on {e}"))
                .unwrap_or_default(),
        );
        ctrl.reset();
        ctrl.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
        let mut t = 0.0f64;
        loop {
            let out = ctrl.tick(CONTROL_DT);
            t += CONTROL_DT;
            viz.publish(t, trunk_z, &out, &signs);
            std::thread::sleep(std::time::Duration::from_secs_f64(CONTROL_DT));
        }
    }
    #[cfg(not(feature = "viz"))]
    if viz_cfg.enabled {
        eprintln!("viz: --viz ignored (binary built without the `viz` feature)");
    }

    Ok(())
}

fn run_dump(misa_path: &str, tune: GaitTune) -> Result<(), String> {
    // 1. Load the Go2 model straight from .misa (no articara).
    let parsed = misarta::native::load(misa_path).map_err(|e| format!("load {misa_path}: {e:?}"))?;
    let (model, _vis, _col) =
        misarta::native::build_model(&parsed.file).map_err(|e| format!("build_model: {e:?}"))?;
    eprintln!(
        "loaded {misa_path}: {} joints, nq={}",
        model.num_joints(),
        model.nq
    );

    // 2. Auto-detect kinematics + IK sign table from the misarta model.
    let home_q = build_home_q(&model);
    let kin = auto_detect_kinematics_config(&model, &DEFAULT_FOOT_LINKS, &home_q)
        .map_err(|errs| format!("kinematics auto-detect failed: {errs:?}"))?;
    let signs = joint_signs(&model, &kin)?;

    eprintln!("\n=== detected KinematicsConfig ===");
    for (slot, lk) in [&kin.fl, &kin.fr, &kin.rl, &kin.rr].iter().enumerate() {
        eprintln!(
            "  {:?}: hip_offset=[{:.4},{:.4},{:.4}] hip_to_thigh_y={:.4} upper={:.4} lower={:.4} \
             nominal_foot=[{:.4},{:.4},{:.4}] signs={:?}",
            lk.leg,
            lk.hip_offset.x,
            lk.hip_offset.y,
            lk.hip_offset.z,
            lk.hip_to_thigh_y,
            lk.upper_leg_m,
            lk.lower_leg_m,
            lk.nominal_foot_body.x,
            lk.nominal_foot_body.y,
            lk.nominal_foot_body.z,
            signs[slot],
        );
    }

    // 3. Build the requested gait (default LinearCrawl — statically stable).
    let cfg = GaitConfig::for_type(tune.gait_type);
    let mut ctrl = AnyGaitController::new(tune.gait_mode, cfg, kin);
    ctrl.set_knee_pattern(KneePattern::BothBack);
    // Check joint limits at the same stance height the gait will run at.
    ctrl.set_body_height_m(tune.stance_height);

    // 4. Run two phases: in-place (vx=0) then a slow forward crawl, and check
    //    every commanded angle (after sign correction) against the Go2 limits.
    eprintln!("\n=== trajectory dump (model-convention rad, after sign correction) ===");
    eprintln!("phase,t_s,FR0,FR1,FR2,FL0,FL1,FL2,RR0,RR1,RR2,RL0,RL1,RL2");

    let mut violations = 0usize;
    for (phase, vx, steps) in [("inplace", 0.0_f64, 500usize), ("forward", 0.1, 1000)] {
        ctrl.set_velocity_cmd(VelocityCmd { vx, vy: 0.0, wz: 0.0 });
        for step in 0..steps {
            let out = ctrl.tick(CONTROL_DT);
            // Assemble the 12 Go2-ordered, sign-corrected joint targets.
            let mut q_go2 = [0.0f64; 12];
            for (name, q_ik) in out.iter_joint_targets() {
                let (slot, k) = (
                    gait_slot(name).ok_or_else(|| format!("bad joint name {name}"))?,
                    joint_kind(name).ok_or_else(|| format!("bad joint name {name}"))?,
                );
                let q_model = q_ik * signs[slot][k];
                let mi = go2_motor_index(name).ok_or_else(|| format!("no motor for {name}"))?;
                q_go2[mi] = q_model;
                // Limit check against the model's joint limits.
                let (lo, hi) = LIMITS[k];
                if q_model < lo - 1e-3 || q_model > hi + 1e-3 {
                    if violations < 20 {
                        eprintln!(
                            "  LIMIT! phase={phase} step={step} {name} q={q_model:.4} not in [{lo:.4},{hi:.4}]"
                        );
                    }
                    violations += 1;
                }
            }
            // Print every 100th tick to keep the dump readable.
            if step % 100 == 0 {
                let t = step as f64 * CONTROL_DT;
                let cols: Vec<String> = q_go2.iter().map(|v| format!("{v:.4}")).collect();
                eprintln!("{phase},{t:.3},{}", cols.join(","));
            }
        }
    }

    eprintln!("\nlimit violations: {violations}");
    if violations > 0 {
        return Err(format!("{violations} joint-limit violations — not safe to send"));
    }
    eprintln!("OK: all commanded joint angles within Go2 limits.");
    Ok(())
}

/// Block until a `LowState` arrives, returning the 12 leg-joint angles in Go2
/// motor order (FR/FL/RR/RL × hip/thigh/calf).
fn wait_for_start_pose(reader: &unitree_go2::Reader<LowState>) -> Result<[f64; 12], String> {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut warned = false;
    loop {
        if let Some(s) = reader.poll().map_err(|e| format!("poll: {e}"))? {
            let mut q = [0.0f64; 12];
            for j in 0..joint::NUM_LEG_JOINTS {
                q[j] = s.motor_state[j].q as f64;
            }
            return Ok(q);
        }
        if Instant::now() >= deadline {
            return Err("timeout waiting for LowState (check iface / 192.168.123.x / cabling)".into());
        }
        if !warned {
            eprintln!("... waiting for LowState (check iface / 192.168.123.x / cabling)");
            warned = true;
        }
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// Drive the LinearCrawl gait on the real robot via rt/lowcmd at 500 Hz, while
/// reading state back from `rt/lowstate`.
///
/// Phases (all low-level; sport_mode must already be OFF):
///   A. ramp the captured start pose into the gait's nominal stance (kp 0→kp)
///   B. hold in place (vx=0) for `inplace_secs`
///   C. if `vx_target > 0`: accelerate to vx_target, hold `forward_secs`, decelerate
///   D. fold to the lying pose for a safe exit
///
/// Each B/C tick records the measured joint angles and body roll/pitch, and a
/// per-joint tracking-error + body-tilt summary is printed at the end (use it to
/// Read a gait-slot leg's `(q_hip, q_thigh, q_calf, q̇_hip, q̇_thigh, q̇_calf)`
/// from the Go2 `LowState`, converted to the IK sign convention the
/// kinematics use (`q_ik = q_motor · sign`, since `output_to_go2` applies the
/// same `±1` going the other way). Slot order is FL,FR,RL,RR; the Go2 motor
/// base index per slot is FR=0, FL=3, RR=6, RL=9 (see `go2_motor_index`).
fn leg_q_dq_ik(s: &LowState, slot: usize, signs: &[[f64; 3]; 4]) -> (f64, f64, f64, f64, f64, f64) {
    let base = match slot {
        0 => 3, // FL
        1 => 0, // FR
        2 => 9, // RL
        _ => 6, // RR
    };
    let sg = signs[slot];
    let q = |k: usize| s.motor_state[base + k].q as f64 * sg[k];
    let dq = |k: usize| s.motor_state[base + k].dq as f64 * sg[k];
    (q(0), q(1), q(2), dq(0), dq(1), dq(2))
}

/// Kinematics-based observer of the body's world-frame velocity + pose, used to
/// close the loop for the MPC gait modes (CHAMP / LinearCrawl ignore it).
///
/// Stance-foot odometry: a planted foot is fixed in the world, so the body
/// velocity is the negated joint-driven foot velocity plus the `−ω×r` term,
/// averaged over the stance legs (cf. `articara::leg_odometry`):
/// ```text
///   v_body = mean_stance( −J·q̇  −  ω_body × p_foot ),   v_world = R(yaw)·v_body
/// ```
/// Horizontal only (roll/pitch treated as level); position integrates v_world.
struct BodyObserver {
    pos: nalgebra::Vector3<f64>,
}

impl BodyObserver {
    fn new() -> Self {
        Self {
            pos: nalgebra::Vector3::zeros(),
        }
    }

    /// Feed yaw + angular/linear velocity + integrated position to `ctrl`.
    /// `stance` is the previous tick's per-slot contact schedule.
    fn feed(
        &mut self,
        ctrl: &mut AnyGaitController,
        s: &LowState,
        kin: &quadruped_gait::KinematicsConfig,
        signs: &[[f64; 3]; 4],
        stance: &[bool; 4],
        dt: f64,
    ) {
        let yaw = s.imu_state.rpy[2] as f64;
        let g = s.imu_state.gyroscope;
        let omega_body = nalgebra::Vector3::new(g[0] as f64, g[1] as f64, g[2] as f64);

        let mut v_sum = nalgebra::Vector3::zeros();
        let mut n = 0u32;
        for slot in 0..4 {
            if !stance[slot] {
                continue;
            }
            let (qh, qt, qc, dqh, dqt, dqc) = leg_q_dq_ik(s, slot, signs);
            let lk = kin.leg(LegId::ALL[slot]);
            let jac = quadruped_gait::foot_jacobian_body(lk, qh, qt, qc);
            let qd = nalgebra::Vector3::new(dqh, dqt, dqc);
            let p_foot = quadruped_gait::forward_leg_kinematics(lk, qh, qt, qc);
            v_sum += -(jac * qd) - omega_body.cross(&p_foot);
            n += 1;
        }
        let v_body = if n > 0 {
            v_sum / n as f64
        } else {
            nalgebra::Vector3::zeros()
        };

        // Body→world by yaw only (horizontal).
        let (sn, cs) = (yaw.sin(), yaw.cos());
        let rot = |v: nalgebra::Vector3<f64>| {
            nalgebra::Vector3::new(cs * v.x - sn * v.y, sn * v.x + cs * v.y, v.z)
        };
        let v_world = rot(v_body);
        let omega_world = rot(omega_body);
        self.pos += v_world * dt;

        ctrl.set_body_state_observed(v_world, omega_world);
        ctrl.set_body_pose_observed(yaw, self.pos);
    }
}

/// size kp/kd: large stance tracking error ⇒ the legs sag under load ⇒ raise kp).
/// With `csv_path` set, every recorded tick is also written as a full-telemetry
/// CSV row. Both the `run` and `diag` CLI modes call this single path.
#[allow(clippy::too_many_arguments)]
fn run_hardware(
    iface: &str,
    misa_path: &str,
    vx_target: f64,
    inplace_secs: f64,
    forward_secs: f64,
    kp: f32,
    kd: f32,
    tune: GaitTune,
    ff: bool,
    ff_scale: f64,
    level_gain: f64,
    csv_path: Option<&str>,
    viz_cfg: &VizCfg,
    led_3support: bool,
    led_margin: f64,
) -> Result<(), String> {
    use std::io::Write as _;
    let swing_h = tune.swing_h;
    let (model, home_q, mut ctrl, signs) = build_gait(misa_path, tune)?;
    let weight = body_weight_n(&model) * ff_scale;
    let com = misarta::centroidal::compute_com(&model, &home_q);
    let com_xy = (com.x, com.y);
    ctrl.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
    let stance = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
    ctrl.reset();

    eprintln!(
        "go2-gait-runner: LinearCrawl vx={vx_target} inplace={inplace_secs}s forward={forward_secs}s kp={kp} kd={kd} swing_h={swing_h} stance_height={:.3} cycle={:?} four_support={:?} max_swing_speed={:?} grav_ff={ff} ff_scale={ff_scale} smooth_swing={} level_gain={level_gain} CoM=({:.3},{:.3})",
        tune.stance_height, tune.cycle_s, tune.four_support, tune.max_swing_foot_speed, tune.smooth_swing, com.x, com.y
    );
    eprintln!("  sport_mode released via native RPC (unless --no-release); ensure the area is clear ...");

    let dp = Participant::new(0, Some(iface)).map_err(|e| format!("participant: {e}"))?;
    let cmd_topic = dp
        .create_topic::<unitree_go2::LowCmd>(topics::LOW_CMD)
        .map_err(|e| format!("cmd topic: {e}"))?;
    let writer = dp
        .create_writer(&cmd_topic, WriterQos::low_level_default())
        .map_err(|e| format!("writer: {e}"))?;
    let state_topic = dp
        .create_topic::<LowState>(topics::LOW_STATE)
        .map_err(|e| format!("state topic: {e}"))?;
    let reader = dp
        .create_reader(&state_topic, ReaderQos::low_level_default())
        .map_err(|e| format!("reader: {e}"))?;

    let start = wait_for_start_pose(&reader)?;

    // ── Head-LED 3-support indicator (`--led-3support`) ──────────────
    // The LED is driven over the VUI service, whose RPC is request/response
    // (it blocks until a reply or times out). Calling it from the 500 Hz loop
    // would stall control, so a background thread owns the VUI client and the
    // control loop only sends a target brightness (0/10) when it changes.
    //
    // The VUI client is created *here*, after the gait `dp` participant above,
    // so its identical `CYCLONEDDS_URI` (same iface) doesn't disturb the
    // already-built control participant; both then coexist on domain 0.
    let (led_tx, led_handle) = if led_3support {
        let (tx, rx) = std::sync::mpsc::channel::<i32>();
        let iface_led = iface.to_string();
        let h = std::thread::spawn(move || {
            let rpc = match unitree_rpc::RpcClient::new(VUI_SERVICE, &iface_led) {
                Ok(r) => r,
                Err(e) => {
                    eprintln!("led: vui connect failed: {e} (3-support LED disabled)");
                    return;
                }
            };
            let _ = rpc.call(VUI_API_ID_SET_SWITCH, "{\"enable\":1}"); // LED switch on
            let _ = rpc.call(VUI_API_ID_SET_BRIGHTNESS, "{\"brightness\":0}"); // start dark
            let mut last = 0i32;
            // Block for the next target; coalesce to the newest so a burst of
            // changes collapses to one RPC.
            while let Ok(first) = rx.recv() {
                let mut level = first;
                while let Ok(l) = rx.try_recv() {
                    level = l;
                }
                if level != last {
                    let _ = rpc.call(VUI_API_ID_SET_BRIGHTNESS, &format!("{{\"brightness\":{level}}}"));
                    last = level;
                }
            }
            let _ = rpc.call(VUI_API_ID_SET_BRIGHTNESS, "{\"brightness\":0}"); // off on exit
        });
        eprintln!("led: 3-support indicator ON (margin {led_margin:.3}s; brightness 10/0 via vui)");
        (Some(tx), Some(h))
    } else {
        (None, None)
    };
    // Preview controller, advanced `led_margin` ahead of the real gait, so the
    // LED can be lit *before* a 3-support period begins. The gait is
    // deterministic, so a shadow copy ticked ahead yields the upcoming contact
    // schedule. Contact *timing* is governed by the gait clock (independent of
    // velocity magnitude), so a fixed vx=0 preview gives the right timing.
    let led_margin_ticks = (led_margin / CONTROL_DT).round().max(0.0) as u32;
    let mut led_preview = if led_3support {
        let (_m, _hq, mut pc, _s) = build_gait(misa_path, tune)?;
        pc.reset();
        pc.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
        for _ in 0..led_margin_ticks {
            pc.tick(CONTROL_DT);
        }
        Some(pc)
    } else {
        None
    };
    let mut led_hold = 0u32; // ticks remaining of the post (直後) margin
    let mut led_prev = -1i32; // last brightness sent (force first send)

    let mut cmd = init_lowcmd();
    let loop_start = Instant::now();
    let mut tick: u64 = 0;
    let mut emit = |q: &[f64; 12], tau: &[f64; 12], kp: f32, kd: f32| -> Result<(), String> {
        for j in 0..joint::NUM_LEG_JOINTS {
            let m = &mut cmd.motor_cmd[j];
            m.q = q[j] as f32;
            m.dq = 0.0;
            m.kp = kp;
            m.kd = kd;
            m.tau = tau[j] as f32;
        }
        set_crc(&mut cmd);
        writer.write(&cmd).map_err(|e| format!("write: {e}"))?;
        tick += 1;
        let next = loop_start + Duration::from_secs_f64(CONTROL_DT * tick as f64);
        if let Some(d) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(d);
        }
        Ok(())
    };
    let ticks = |secs: f64| -> u64 { (secs / CONTROL_DT).round().max(1.0) as u64 };

    // Accumulators over the recorded (B + C) phases.
    let mut err_sum = [0.0f64; 12];
    let mut err_max = [0.0f64; 12];
    let mut n_rec = 0u64;
    let mut roll_max = 0.0f64;
    let mut pitch_max = 0.0f64;
    // Yaw drift = straightness ("x方向のみ"). Track deviation from the first
    // recorded heading and the net (final) drift, so a tuning pass can be scored
    // on how little the robot turns off the +x axis.
    let mut yaw_first: Option<f64> = None;
    let mut yaw_dev_max = 0.0f64;
    let mut yaw_last = 0.0f64;
    let mut sample_log = 0u64;

    // Go2 motor order; reused for the CSV columns and the summary table.
    let jnames = [
        "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf", "RR_hip", "RR_thigh",
        "RR_calf", "RL_hip", "RL_thigh", "RL_calf",
    ];

    // Optional full-telemetry CSV (one row per recorded tick, phases B+C).
    let mut csv = match csv_path {
        Some(p) => {
            let f = std::fs::File::create(p).map_err(|e| format!("csv create {p}: {e}"))?;
            let mut w = std::io::BufWriter::new(f);
            let mut hdr = String::from(
                "t_s,phase,roll,pitch,yaw,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,\
                 quat_w,quat_x,quat_y,quat_z,imu_temp,power_v,power_a,\
                 foot0,foot1,foot2,foot3,cmd_vx,cmd_vy,cmd_wz,\
                 support,FR_state,FL_state,RR_state,RL_state",
            );
            for nm in jnames.iter() {
                hdr.push_str(&format!(",{nm}_cmd,{nm}_q,{nm}_dq,{nm}_tau"));
            }
            hdr.push('\n');
            w.write_all(hdr.as_bytes())
                .map_err(|e| format!("csv write: {e}"))?;
            eprintln!("recording full telemetry CSV -> {p}");
            Some(w)
        }
        None => None,
    };

    // Phase A: ramp to stance.
    let ramp_n = ticks(RAMP_SECS);
    for i in 0..ramp_n {
        let p = i as f64 / ramp_n as f64;
        let mut q = [0.0f64; 12];
        for j in 0..12 {
            q[j] = (1.0 - p) * start[j] + p * stance[j];
        }
        emit(&q, &ZERO_TAU, (kp as f64 * p) as f32, kd)?;
    }

    eprintln!("phase,t_s,roll,pitch,FRt_cmd,FRt_act,FRc_cmd,FRc_act");

    // Recording closure body, used in B and C. Takes a pre-polled `LowState`
    // (the loop polls once per tick and shares the sample with the leveling
    // feedback) so the reader isn't drained twice per tick.
    let record = |sample: Option<&LowState>,
                      q_cmd: &[f64; 12],
                      vel: VelocityCmd,
                      // Target contact schedule for this tick, canonical
                      // LegId::ALL order [FL, FR, RL, RR]. `true` = stance
                      // (foot planted), `false` = swing (leg in the air).
                      stance: [bool; 4],
                      phase: &str,
                      t: f64,
                      err_sum: &mut [f64; 12],
                      err_max: &mut [f64; 12],
                      n_rec: &mut u64,
                      roll_max: &mut f64,
                      pitch_max: &mut f64,
                      yaw_first: &mut Option<f64>,
                      yaw_dev_max: &mut f64,
                      yaw_last: &mut f64,
                      sample_log: &mut u64,
                      csv: &mut Option<std::io::BufWriter<std::fs::File>>|
     -> Result<(), String> {
        if let Some(s) = sample {
            for j in 0..joint::NUM_LEG_JOINTS {
                let e = (q_cmd[j] - s.motor_state[j].q as f64).abs();
                err_sum[j] += e;
                err_max[j] = err_max[j].max(e);
            }
            *n_rec += 1;
            let roll = s.imu_state.rpy[0] as f64;
            let pitch = s.imu_state.rpy[1] as f64;
            let yaw = s.imu_state.rpy[2] as f64;
            *roll_max = roll_max.max(roll.abs());
            *pitch_max = pitch_max.max(pitch.abs());
            // Heading drift relative to the first recorded sample. Unwrap the
            // ±π wrap so a small drift near the ±π seam isn't read as ~2π.
            let ref_yaw = *yaw_first.get_or_insert(yaw);
            let mut dyaw = yaw - ref_yaw;
            while dyaw > std::f64::consts::PI {
                dyaw -= 2.0 * std::f64::consts::PI;
            }
            while dyaw < -std::f64::consts::PI {
                dyaw += 2.0 * std::f64::consts::PI;
            }
            *yaw_dev_max = yaw_dev_max.max(dyaw.abs());
            *yaw_last = dyaw;
            if *sample_log % 25 == 0 {
                eprintln!(
                    "{phase},{t:.3},{roll:+.3},{pitch:+.3},{:.3},{:.3},{:.3},{:.3}",
                    q_cmd[1],
                    s.motor_state[1].q,
                    q_cmd[2],
                    s.motor_state[2].q
                );
            }
            *sample_log += 1;

            // Full-telemetry CSV row: IMU, power, foot force, and per-joint
            // commanded/measured position, velocity and estimated torque.
            if let Some(w) = csv.as_mut() {
                let im = &s.imu_state;
                let mut row = format!(
                    "{t:.4},{phase},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},\
                     {:.6},{:.6},{:.6},{:.6},{},{:.3},{:.3},{},{},{},{},{:.5},{:.5},{:.5}",
                    im.rpy[0], im.rpy[1], im.rpy[2],
                    im.gyroscope[0], im.gyroscope[1], im.gyroscope[2],
                    im.accelerometer[0], im.accelerometer[1], im.accelerometer[2],
                    im.quaternion[0], im.quaternion[1], im.quaternion[2], im.quaternion[3],
                    im.temperature, s.power_v, s.power_a,
                    s.foot_force[0], s.foot_force[1], s.foot_force[2], s.foot_force[3],
                    vel.vx, vel.vy, vel.wz,
                );
                // Overall support state + per-leg target swing/stance. The
                // count generalises to any gait (4/3/2/1/0 legs planted);
                // per-leg columns are in the Go2 motor order used for the
                // joint columns (FR, FL, RR, RL), mapped from canonical
                // [FL, FR, RL, RR].
                let n_support = stance.iter().filter(|&&c| c).count();
                let st = |c: bool| if c { "stance" } else { "swing" };
                row.push_str(&format!(
                    ",{n_support}-support,{},{},{},{}",
                    st(stance[1]), st(stance[0]), st(stance[3]), st(stance[2]),
                ));
                for j in 0..12 {
                    let m = &s.motor_state[j];
                    row.push_str(&format!(
                        ",{:.5},{:.5},{:.5},{:.4}",
                        q_cmd[j], m.q, m.dq, m.tau_est
                    ));
                }
                row.push('\n');
                w.write_all(row.as_bytes())
                    .map_err(|e| format!("csv write: {e}"))?;
            }
        }
        Ok(())
    };

    // Latest measured body attitude (roll, pitch, yaw). Updated once per tick
    // from the shared LowState poll and read by the leveling correction below.
    // Declared before the macros so they capture it (macro_rules hygiene
    // resolves outer identifiers at the definition site).
    let mut last_rpy = [0.0f64; 3];

    // Closed-loop observer for the MPC gait modes (no-op for the open-loop
    // CHAMP / LinearCrawl). `kin_obs` is cloned so the observer can borrow it
    // while the macros borrow `ctrl`; `last_stance` is the previous tick's
    // contact schedule (the estimator runs before the current tick).
    let kin_obs = ctrl.kinematics().clone();
    let mut observer = BodyObserver::new();
    let mut last_stance = [true; 4];

    // Live gait-visualization publisher (`--viz`): stream each tick's gait
    // frame over Zenoh for the articara GUI. The trunk-height shown is the
    // gait's body height above the feet (the controller output is horizontal).
    #[cfg(feature = "viz")]
    let viz_trunk_z = if matches!(tune.gait_mode, GaitMode::LinearCrawl) {
        tune.stance_height
    } else {
        -kin_obs.fl.nominal_foot_body.z
    };
    #[cfg(feature = "viz")]
    let mut viz_t = 0.0f64;
    #[cfg(feature = "viz")]
    let mut viz = if viz_cfg.enabled {
        match viz_pub::VizPublisher::new(
            &viz_cfg.key,
            viz_cfg.rate_hz,
            CONTROL_DT,
            viz_cfg.endpoint.as_deref(),
        ) {
            Ok(v) => {
                eprintln!(
                    "viz: publishing gait frames on zenoh key '{}' (~{} Hz){}",
                    v.key(),
                    viz_cfg.rate_hz,
                    viz_cfg
                        .endpoint
                        .as_deref()
                        .map(|e| format!(", listening on {e}"))
                        .unwrap_or_default(),
                );
                Some(v)
            }
            Err(e) => {
                eprintln!("viz: disabled — {e}");
                None
            }
        }
    } else {
        None
    };
    #[cfg(not(feature = "viz"))]
    if viz_cfg.enabled {
        eprintln!("viz: --viz ignored (binary built without the `viz` feature)");
    }

    // Tick the gait, map to Go2 order, compute optional support FF, and apply
    // the optional IMU body-leveling correction (drives measured roll/pitch
    // toward zero by trimming the stance feet — see `level_correction`).
    macro_rules! gait_qtau {
        () => {{
            let out = ctrl.tick(CONTROL_DT);
            // Snapshot this tick's contact schedule for next tick's observer.
            last_stance = [
                out.legs[0].phase.is_stance,
                out.legs[1].phase.is_stance,
                out.legs[2].phase.is_stance,
                out.legs[3].phase.is_stance,
            ];
            #[cfg(feature = "viz")]
            {
                viz_t += CONTROL_DT;
                if let Some(v) = viz.as_mut() {
                    v.publish(viz_t, viz_trunk_z, &out, &signs);
                }
            }
            let mut q = output_to_go2(&out, &signs)?;
            let tau = if ff {
                support_tau_go2(&out, ctrl.kinematics(), &signs, weight, com_xy)
            } else {
                ZERO_TAU
            };
            if level_gain != 0.0 {
                let dq = level_correction(
                    &out, ctrl.kinematics(), &signs, last_rpy[0], last_rpy[1], level_gain,
                );
                for j in 0..12 {
                    q[j] += dq[j];
                }
            }
            // Head-LED 3-support indicator. `now` is this tick's support count;
            // `fut` is the preview (led_margin ahead) → lights *before* a
            // 3-support starts. A hold timer keeps it on `led_margin` *after* it
            // ends. Only a changed brightness is sent (the thread does the RPC).
            if let Some(pc) = led_preview.as_mut() {
                let pv = pc.tick(CONTROL_DT);
                let fut = pv.legs.iter().filter(|l| l.phase.is_stance).count();
                let now = last_stance.iter().filter(|&&s| s).count();
                let trigger = now == 3 || fut == 3;
                if trigger {
                    led_hold = led_margin_ticks;
                } else if led_hold > 0 {
                    led_hold -= 1;
                }
                let want = if trigger || led_hold > 0 { 10 } else { 0 };
                if want != led_prev {
                    if let Some(tx) = led_tx.as_ref() {
                        let _ = tx.send(want);
                    }
                    led_prev = want;
                }
            }
            (q, tau)
        }};
    }

    // Monotonic recorded-time offsets so the CSV's leading `t_s` runs
    // continuously across phases B and C instead of resetting each phase.
    let b_dur = ticks(inplace_secs) as f64 * CONTROL_DT;
    let accel_dur = ticks(ACCEL_SECS) as f64 * CONTROL_DT;

    // Poll the reader once and refresh `last_rpy`; returns the sample so the
    // same read also feeds `record` (avoids draining the reader twice/tick).
    macro_rules! poll_state {
        () => {{
            let s = reader.poll().map_err(|e| format!("poll: {e}"))?;
            if let Some(st) = &s {
                last_rpy = [
                    st.imu_state.rpy[0] as f64,
                    st.imu_state.rpy[1] as f64,
                    st.imu_state.rpy[2] as f64,
                ];
                // Feed the MPC modes their closed-loop state (no-op otherwise).
                observer.feed(&mut ctrl, st, &kin_obs, &signs, &last_stance, CONTROL_DT);
            }
            s
        }};
    }

    // Phase B: in-place (vx=0), recording. `cmd_vel` mirrors the velocity sent
    // to the gait generator each tick so `record` can log it (CSV cmd_* cols).
    // Today this is driven by the CLI `vx_target`; when an external Twist source
    // is wired in it will write the same `cmd_vel` and the columns stay valid.
    let mut cmd_vel = VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 };
    ctrl.set_velocity_cmd(cmd_vel);
    for i in 0..ticks(inplace_secs) {
        let sample = poll_state!();
        let (q, tau) = gait_qtau!();
        emit(&q, &tau, kp, kd)?;
        record(sample.as_ref(), &q, cmd_vel, last_stance, "B", i as f64 * CONTROL_DT, &mut err_sum, &mut err_max, &mut n_rec, &mut roll_max, &mut pitch_max, &mut yaw_first, &mut yaw_dev_max, &mut yaw_last, &mut sample_log, &mut csv)?;
    }

    // Phase C: forward, recording.
    if vx_target > 0.0 {
        let accel_n = ticks(ACCEL_SECS);
        for i in 0..accel_n {
            let v = vx_target * (i as f64 / accel_n as f64);
            cmd_vel = VelocityCmd { vx: v, vy: 0.0, wz: 0.0 };
            ctrl.set_velocity_cmd(cmd_vel);
            let sample = poll_state!();
            let (q, tau) = gait_qtau!();
            emit(&q, &tau, kp, kd)?;
            record(sample.as_ref(), &q, cmd_vel, last_stance, "C", b_dur + i as f64 * CONTROL_DT, &mut err_sum, &mut err_max, &mut n_rec, &mut roll_max, &mut pitch_max, &mut yaw_first, &mut yaw_dev_max, &mut yaw_last, &mut sample_log, &mut csv)?;
        }
        cmd_vel = VelocityCmd { vx: vx_target, vy: 0.0, wz: 0.0 };
        ctrl.set_velocity_cmd(cmd_vel);
        for i in 0..ticks(forward_secs) {
            let sample = poll_state!();
            let (q, tau) = gait_qtau!();
            emit(&q, &tau, kp, kd)?;
            record(sample.as_ref(), &q, cmd_vel, last_stance, "C", b_dur + accel_dur + i as f64 * CONTROL_DT, &mut err_sum, &mut err_max, &mut n_rec, &mut roll_max, &mut pitch_max, &mut yaw_first, &mut yaw_dev_max, &mut yaw_last, &mut sample_log, &mut csv)?;
        }
        for i in 0..accel_n {
            let v = vx_target * (1.0 - i as f64 / accel_n as f64);
            cmd_vel = VelocityCmd { vx: v, vy: 0.0, wz: 0.0 };
            ctrl.set_velocity_cmd(cmd_vel);
            let _sample = poll_state!();
            let (q, tau) = gait_qtau!();
            emit(&q, &tau, kp, kd)?;
        }
    }

    // Phase D: fold to lying pose.
    let cur = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
    let fold_n = ticks(FOLD_SECS);
    for i in 0..fold_n {
        let p = i as f64 / fold_n as f64;
        let mut q = [0.0f64; 12];
        for j in 0..12 {
            q[j] = (1.0 - p) * cur[j] + p * LIE_POS[j];
        }
        emit(&q, &ZERO_TAU, kp, kd)?;
    }
    for _ in 0..ticks(0.5) {
        emit(&LIE_POS, &ZERO_TAU, kp, kd)?;
    }

    // Stop the LED thread: blank the LED, then close the channel so its
    // `recv` returns and the thread exits; join so its final RPC completes.
    if let Some(tx) = led_tx.as_ref() {
        let _ = tx.send(0);
    }
    drop(led_tx);
    if let Some(h) = led_handle {
        let _ = h.join();
    }

    // Summary.
    eprintln!("\n=== summary over {n_rec} samples (B+C) ===");
    eprintln!("  per-joint tracking error |cmd-act| (rad): mean / max");
    for j in 0..12 {
        let mean = if n_rec > 0 { err_sum[j] / n_rec as f64 } else { 0.0 };
        eprintln!("    {:<8} mean={mean:.4}  max={:.4}", jnames[j], err_max[j]);
    }
    eprintln!(
        "  body tilt: max|roll|={roll_max:.3} rad ({:.1} deg)  max|pitch|={pitch_max:.3} rad ({:.1} deg)",
        roll_max.to_degrees(),
        pitch_max.to_degrees()
    );
    eprintln!(
        "  heading (straightness): max|yaw drift|={yaw_dev_max:.3} rad ({:.1} deg)  \
         net drift={yaw_last:+.3} rad ({:+.1} deg)",
        yaw_dev_max.to_degrees(),
        yaw_last.to_degrees()
    );
    eprintln!("  → minimise roll/pitch (sway) and |yaw drift| (off-axis) when tuning.");
    eprintln!("done: gait complete, folded on the ground.");
    Ok(())
}
