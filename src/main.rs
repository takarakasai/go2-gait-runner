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
    GaitGenerator, GaitMode, KneePattern, VelocityCmd, DEFAULT_FOOT_LINKS,
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

fn main() {
    let mut args = std::env::args().skip(1);
    let mode = args.next().unwrap_or_else(|| "dump".to_string());
    let misa_path = args
        .next()
        .unwrap_or_else(|| "models/unitree_go2/go2.misa".to_string());

    match mode.as_str() {
        "dump" => {
            // `dump [misa_path]`
            if let Err(e) = run_dump(&misa_path) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        "run" => {
            // `run <iface> [misa_path] [vx] [inplace_secs] [forward_secs] [kp] [kd]`
            // Here the 2nd arg is the iface, not the misa path.
            let iface = misa_path; // 2nd positional
            let mut rest = std::env::args().skip(3);
            let misa = rest
                .next()
                .unwrap_or_else(|| "models/unitree_go2/go2.misa".to_string());
            let vx: f64 = rest.next().and_then(|s| s.parse().ok()).unwrap_or(0.0);
            let inplace_secs: f64 = rest.next().and_then(|s| s.parse().ok()).unwrap_or(3.0);
            let forward_secs: f64 = rest.next().and_then(|s| s.parse().ok()).unwrap_or(4.0);
            let kp: f32 = rest.next().and_then(|s| s.parse().ok()).unwrap_or(60.0);
            let kd: f32 = rest.next().and_then(|s| s.parse().ok()).unwrap_or(5.0);
            if iface.is_empty() || iface.ends_with(".misa") {
                eprintln!("usage: go2-gait-runner run <iface> [misa_path] [vx] [inplace_secs] [forward_secs] [kp] [kd]");
                std::process::exit(2);
            }
            if let Err(e) = run_hardware(&iface, &misa, vx, inplace_secs, forward_secs, kp, kd) {
                eprintln!("error: {e}");
                std::process::exit(1);
            }
        }
        other => {
            eprintln!("usage: go2-gait-runner <dump|run> ...   (got mode {other:?})");
            std::process::exit(2);
        }
    }
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

/// Drive the LinearCrawl gait on the real robot via rt/lowcmd at 500 Hz.
///
/// Phases (all low-level; sport_mode must already be OFF):
///   A. ramp the captured start pose into the gait's nominal stance (kp 0→kp)
///   B. hold in place (vx=0) for `inplace_secs`
///   C. if `vx_target > 0`: accelerate to vx_target, hold `forward_secs`, decelerate
///   D. fold to the lying pose for a safe exit
#[allow(clippy::too_many_arguments)]
fn run_hardware(
    iface: &str,
    misa_path: &str,
    vx_target: f64,
    inplace_secs: f64,
    forward_secs: f64,
    kp: f32,
    kd: f32,
) -> Result<(), String> {
    // ── Build the gait (same pipeline as `dump`) ───────────────────────────
    let parsed = misarta::native::load(misa_path).map_err(|e| format!("load {misa_path}: {e:?}"))?;
    let (model, _vis, _col) =
        misarta::native::build_model(&parsed.file).map_err(|e| format!("build_model: {e:?}"))?;
    let home_q = build_home_q(&model);
    let kin = auto_detect_kinematics_config(&model, &DEFAULT_FOOT_LINKS, &home_q)
        .map_err(|errs| format!("kinematics auto-detect failed: {errs:?}"))?;
    let signs = joint_signs(&model, &kin)?;
    let mut ctrl = AnyGaitController::new(GaitMode::LinearCrawl, GaitConfig::crawl(), kin);
    ctrl.set_knee_pattern(KneePattern::BothBack);

    // Nominal stance (Go2 order): the pose the gait holds at vx=0. Sample it
    // with one tick, then reset so the real loop starts from a clean phase.
    ctrl.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
    let stance = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
    ctrl.reset();

    eprintln!(
        "go2-gait-runner: LinearCrawl  vx={vx_target} inplace={inplace_secs}s forward={forward_secs}s kp={kp} kd={kd}"
    );
    eprintln!("  ensure sport_mode is OFF (go2_motion_ctrl release {iface}) and the area is clear ...");

    // ── DDS endpoints ──────────────────────────────────────────────────────
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
    eprintln!(
        "start pose captured: FR=[{:.3},{:.3},{:.3}] stance FR=[{:.3},{:.3},{:.3}]",
        start[0], start[1], start[2], stance[0], stance[1], stance[2]
    );

    // ── 500 Hz emit closure (set 12 motors, CRC, write, pace to cadence) ───
    let mut cmd = init_lowcmd();
    let loop_start = Instant::now();
    let mut tick: u64 = 0;
    let mut emit = |q: &[f64; 12], kp: f32, kd: f32| -> Result<(), String> {
        for j in 0..joint::NUM_LEG_JOINTS {
            let m = &mut cmd.motor_cmd[j];
            m.q = q[j] as f32;
            m.dq = 0.0;
            m.kp = kp;
            m.kd = kd;
            m.tau = 0.0;
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

    // ── Phase A: ramp start → stance, kp 0 → kp ────────────────────────────
    eprintln!("phase A: ramp to stance ({RAMP_SECS}s)");
    let ramp_n = ticks(RAMP_SECS);
    for i in 0..ramp_n {
        let p = i as f64 / ramp_n as f64;
        let mut q = [0.0f64; 12];
        for j in 0..12 {
            q[j] = (1.0 - p) * start[j] + p * stance[j];
        }
        emit(&q, (kp as f64 * p) as f32, kd)?;
    }

    // ── Phase B: in-place (vx=0) ───────────────────────────────────────────
    eprintln!("phase B: in-place vx=0 ({inplace_secs}s)");
    ctrl.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
    for _ in 0..ticks(inplace_secs) {
        let q = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
        emit(&q, kp, kd)?;
    }

    // ── Phase C: forward crawl (accelerate, hold, decelerate) ──────────────
    if vx_target > 0.0 {
        eprintln!("phase C: forward to vx={vx_target} (accel {ACCEL_SECS}s, hold {forward_secs}s, decel {ACCEL_SECS}s)");
        let accel_n = ticks(ACCEL_SECS);
        for i in 0..accel_n {
            let v = vx_target * (i as f64 / accel_n as f64);
            ctrl.set_velocity_cmd(VelocityCmd { vx: v, vy: 0.0, wz: 0.0 });
            let q = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
            emit(&q, kp, kd)?;
        }
        ctrl.set_velocity_cmd(VelocityCmd { vx: vx_target, vy: 0.0, wz: 0.0 });
        for _ in 0..ticks(forward_secs) {
            let q = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
            emit(&q, kp, kd)?;
        }
        for i in 0..accel_n {
            let v = vx_target * (1.0 - i as f64 / accel_n as f64);
            ctrl.set_velocity_cmd(VelocityCmd { vx: v, vy: 0.0, wz: 0.0 });
            let q = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
            emit(&q, kp, kd)?;
        }
        // settle in place briefly
        ctrl.set_velocity_cmd(VelocityCmd { vx: 0.0, vy: 0.0, wz: 0.0 });
        for _ in 0..ticks(0.5) {
            let q = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
            emit(&q, kp, kd)?;
        }
    }

    // ── Phase D: fold to lying pose for a safe exit ────────────────────────
    eprintln!("phase D: fold to lying pose ({FOLD_SECS}s)");
    let cur = output_to_go2(&ctrl.tick(CONTROL_DT), &signs)?;
    let fold_n = ticks(FOLD_SECS);
    for i in 0..fold_n {
        let p = i as f64 / fold_n as f64;
        let mut q = [0.0f64; 12];
        for j in 0..12 {
            q[j] = (1.0 - p) * cur[j] + p * LIE_POS[j];
        }
        emit(&q, kp, kd)?;
    }
    for _ in 0..ticks(0.5) {
        emit(&LIE_POS, kp, kd)?;
    }

    eprintln!("done: gait complete, folded on the ground.");
    Ok(())
}
