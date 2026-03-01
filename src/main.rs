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
    GaitGenerator, GaitMode, KneePattern, LegId, VelocityCmd, DEFAULT_FOOT_LINKS,
};
use unitree_go2::{
    init_lowcmd, joint, set_crc, topics, LowState, Participant, ReaderQos, WriterQos,
};

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
const BOOL_FLAGS: &[&str] = &["ff", "grav-ff"];

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
  run    <iface>  Hardware: ramp -> in-place -> forward -> fold; reads state
                  back and prints a tracking-error + body-tilt summary.
  diag   <iface>  Alias for `run` (identical behaviour).

FLAGS (all optional; <iface> is the 1st positional for run/diag):
  --misa PATH       model .misa file        (default models/unitree_go2/go2.misa)
  --vx V            forward speed, m/s       (run/diag default 0.0; intent 0.05)
  --inplace S       in-place phase, seconds  (default 3)        [run/diag]
  --forward S       forward phase, seconds   (default 4)        [run/diag]
  --kp K            position gain            (default 60)        [run/diag]
  --kd K            damping gain             (default 5)         [run/diag]
  --swing H         foot lift height, m      (default 0.04)
  --cycle S         gait cycle period, s     (default: crawl preset)
  --four-support F  4-support fraction 0..1  (default: crawl preset)
  --sway M          lateral body-sway amplitude, m (default 0 = off)
  --ff              enable body-weight support feedforward      [run/diag]
  --ff-scale S      scale FF to real mass    (default 1.0)       [run/diag]
  --csv PATH        write full per-tick telemetry CSV           [run/diag]
  -h, --help        show this help

EXAMPLE (validated on slippery flooring):
  go2-gait-runner run eth0 --vx 0.02 --cycle 2.5 --four-support 0.9 \\
      --swing 0.04 --kp 200 --kd 6 --ff --ff-scale 1.73
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
    };

    match mode {
        "dump" => {
            // `dump [--misa P]`
            if let Err(e) = run_dump(&misa) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        "intent" => {
            // `intent [--misa P] [--vx V] [--swing H] [--cycle S] [--four-support F]`
            // Offline: quantify the gait's forward displacement, foot sweep, lift.
            let vx = cli.f64("vx").unwrap_or(0.05);
            if let Err(e) = run_intent(&misa, vx, tune) {
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
            let csv = cli.str("csv");
            // `run` and `diag` are the same path now; both always read state back
            // and print the tracking/tilt summary. `diag` is kept as an alias.
            let res = run_hardware(
                &iface, &misa, vx, inplace, forward, kp, kd, tune, ff, ff_scale, csv,
            );
            if let Err(e) = res {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        other => {
            eprintln!("usage: go2-gait-runner <dump|intent|run|diag> ...   (got mode {other:?})");
            std::process::exit(2);
        }
    }
}

/// Gait tuning knobs shared by run/diag/intent. `None` keeps the crawl preset.
#[derive(Clone, Copy)]
struct GaitTune {
    swing_h: f64,
    cycle_s: Option<f64>,
    four_support: Option<f64>,
    /// Lateral body-sway amplitude (m). `None`/0 keeps the no-sway crawl.
    sway: Option<f64>,
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
    let mut cfg = GaitConfig::crawl().with_swing_height(tune.swing_h);
    if let Some(c) = tune.cycle_s {
        cfg = cfg.with_cycle_period(c);
    }
    if let Some(f) = tune.four_support {
        cfg = cfg.with_four_support_fraction(f);
    }
    if let Some(s) = tune.sway {
        cfg = cfg.with_lateral_sway(s);
    }
    let mut ctrl = AnyGaitController::new(GaitMode::LinearCrawl, cfg, kin);
    ctrl.set_knee_pattern(KneePattern::BothBack);
    Ok((model, home_q, ctrl, signs))
}

/// Total robot weight (N) = Σ link mass × g. The misarta Go2 model is
/// fixed-base, so `compute_gravity` would only carry the leg-segment weight;
/// the body-support load that actually makes the legs sag has to be applied as
/// a distributed ground reaction (below).
fn body_weight_n(model: &Model<f64>) -> f64 {
    let m: f64 = model.inertias.iter().map(|i| i.mass).sum();
    m * 9.81
}

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

/// Offline: quantify what the gait *intends* — per-cycle forward trunk
/// displacement, the swing foot lift, and the stance foot fore/aft sweep.
fn run_intent(misa_path: &str, vx: f64, tune: GaitTune) -> Result<(), String> {
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
        // sway it stays 0. Positive = trunk moved to body-left (+Y).
        let by = ctrl.kinematics().fr.nominal_foot_body.y - fr.foot_body.y;
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
    Ok(())
}

fn run_dump(misa_path: &str) -> Result<(), String> {
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

    // 3. Build a LinearCrawl controller (statically stable; safest first gait).
    let cfg = GaitConfig::crawl();
    let mut ctrl = AnyGaitController::new(GaitMode::LinearCrawl, cfg, kin);
    ctrl.set_knee_pattern(KneePattern::BothBack);

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
    csv_path: Option<&str>,
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
        "go2-gait-runner: LinearCrawl vx={vx_target} inplace={inplace_secs}s forward={forward_secs}s kp={kp} kd={kd} swing_h={swing_h} cycle={:?} four_support={:?} grav_ff={ff} ff_scale={ff_scale} CoM=({:.3},{:.3})",
        tune.cycle_s, tune.four_support, com.x, com.y
    );
    eprintln!("  ensure sport_mode is OFF (go2_motion_ctrl release {iface}) and the area is clear ...");

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
                 foot0,foot1,foot2,foot3",
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

    // Recording closure body, used in B and C.
    let record = |reader: &unitree_go2::Reader<LowState>,
                      q_cmd: &[f64; 12],
                      phase: &str,
                      t: f64,
                      err_sum: &mut [f64; 12],
                      err_max: &mut [f64; 12],
                      n_rec: &mut u64,
                      roll_max: &mut f64,
                      pitch_max: &mut f64,
                      sample_log: &mut u64,
                      csv: &mut Option<std::io::BufWriter<std::fs::File>>|
     -> Result<(), String> {
        if let Some(s) = reader.poll().map_err(|e| format!("poll: {e}"))? {
            for j in 0..joint::NUM_LEG_JOINTS {
                let e = (q_cmd[j] - s.motor_state[j].q as f64).abs();
                err_sum[j] += e;
                err_max[j] = err_max[j].max(e);
            }
            *n_rec += 1;
            let roll = s.imu_state.rpy[0] as f64;
            let pitch = s.imu_state.rpy[1] as f64;
            *roll_max = roll_max.max(roll.abs());
            *pitch_max = pitch_max.max(pitch.abs());
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
                     {:.6},{:.6},{:.6},{:.6},{},{:.3},{:.3},{},{},{},{}",
                    im.rpy[0], im.rpy[1], im.rpy[2],
                    im.gyroscope[0], im.gyroscope[1], im.gyroscope[2],
                    im.accelerometer[0], im.accelerometer[1], im.accelerometer[2],
                    im.quaternion[0], im.quaternion[1], im.quaternion[2], im.quaternion[3],
                    im.temperature, s.power_v, s.power_a,
                    s.foot_force[0], s.foot_force[1], s.foot_force[2], s.foot_force[3],
                );
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

    // Tick the gait, map to Go2 order, compute optional support FF.
    macro_rules! gait_qtau {
        () => {{
            let out = ctrl.tick(CONTROL_DT);
            let q = output_to_go2(&out, &signs)?;
            let tau = if ff {
                support_tau_go2(&out, ctrl.kinematics(), &signs, weight, com_xy)
            } else {
                ZERO_TAU
            };
            (q, tau)
        }};
    }

    // Monotonic recorded-time offsets so the CSV's leading `t_s` runs
    // continuously across phases B and C instead of resetting each phase.
    let b_dur = ticks(inplace_secs) as f64 * CONTROL_DT;
    let accel_dur = ticks(ACCEL_SECS) as f64 * CONTROL_DT;

    // Phase B: in-place (vx=0), recording.
    ctrl.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
    for i in 0..ticks(inplace_secs) {
        let (q, tau) = gait_qtau!();
        emit(&q, &tau, kp, kd)?;
        record(&reader, &q, "B", i as f64 * CONTROL_DT, &mut err_sum, &mut err_max, &mut n_rec, &mut roll_max, &mut pitch_max, &mut sample_log, &mut csv)?;
    }

    // Phase C: forward, recording.
    if vx_target > 0.0 {
        let accel_n = ticks(ACCEL_SECS);
        for i in 0..accel_n {
            let v = vx_target * (i as f64 / accel_n as f64);
            ctrl.set_velocity_cmd(VelocityCmd { vx: v, vy: 0.0, wz: 0.0 });
            let (q, tau) = gait_qtau!();
            emit(&q, &tau, kp, kd)?;
            record(&reader, &q, "C", b_dur + i as f64 * CONTROL_DT, &mut err_sum, &mut err_max, &mut n_rec, &mut roll_max, &mut pitch_max, &mut sample_log, &mut csv)?;
        }
        ctrl.set_velocity_cmd(VelocityCmd { vx: vx_target, vy: 0.0, wz: 0.0 });
        for i in 0..ticks(forward_secs) {
            let (q, tau) = gait_qtau!();
            emit(&q, &tau, kp, kd)?;
            record(&reader, &q, "C", b_dur + accel_dur + i as f64 * CONTROL_DT, &mut err_sum, &mut err_max, &mut n_rec, &mut roll_max, &mut pitch_max, &mut sample_log, &mut csv)?;
        }
        for i in 0..accel_n {
            let v = vx_target * (1.0 - i as f64 / accel_n as f64);
            ctrl.set_velocity_cmd(VelocityCmd { vx: v, vy: 0.0, wz: 0.0 });
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
    eprintln!("done: gait complete, folded on the ground.");
    Ok(())
}
