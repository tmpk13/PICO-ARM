// Firmware for the BTT SKR Pico (RP2040).
//
// USB CDC ACM serial. Connect with picocom/screen and type:
//
//     help
//     enable all
//     accel x 2000         # ramp limit: 2000 steps/sec/sec (0 = instant)
//     jog x 800            # continuous: positive direction, 800 steps/sec
//     jog x -400           # reverse at 400 steps/sec
//     jog x 0              # stop
//     move x 1600 1000     # one-shot: ~1600 steps at 1000 hz (duration-based)
//     fan 0 on             # SKR Pico fan headers
//     servo 4 1500         # 1500 us pulse on the SERVOS header; 0 = limp
//     disable all
//
// Each axis runs in its own task fed by a Signal, so jog commands take
// effect mid-pulse without waiting for the previous motion to finish.
// This is also the surface the host-side joystick driver writes to.

#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, AtomicI32, AtomicU16, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use heapless::String;
use panic_halt as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type UsbDrv = Driver<'static, USB>;
type Class = CdcAcmClass<'static, UsbDrv>;

#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, UsbDrv>) -> ! {
    usb.run().await
}

// --------------------------------------------------------------------------
// Per-axis task: owns STEP/DIR/EN, waits on a Signal for command changes.
// --------------------------------------------------------------------------

#[derive(Clone, Copy)]
enum AxisCmd {
    Enable(bool),
    /// Signed steps/second. 0 = stop (motion only; doesn't change EN).
    Jog(i32),
    /// Acceleration slew limit, steps/sec/sec. 0 = instant (no ramp).
    Accel(u32),
}

const AXES: usize = 4;
const AXIS_NAMES: [&str; AXES] = ["X", "Y", "Z", "E"];
const FANS: usize = 3;
const SERVOS: usize = 5;
const SERVO_NAMES: [&str; SERVOS] = ["X-STOP", "Y-STOP", "Z-STOP", "E0-STOP", "SERVOS"];
const MAX_HZ: u32 = 40_000;
/// Snap floor for the ramp generator. When the target is well above this
/// rate we don't let |current_v| dwell below it -- otherwise the first
/// step after rest waits a sub-1Hz period (multi-second) for the timer
/// to fire. Skipping the sub-START_HZ band of velocities is the price.
/// Targets below the floor ramp naturally (host heartbeats at 25 ms
/// drive the slew forward).
const START_HZ: i32 = 20;
const FW_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Deadman timeout. If an axis has nonzero velocity and no jog command
/// arrives within this window, motion halts automatically. The host's
/// joystick driver resends the current velocity every ~25 ms (40 Hz), so
/// a single dropped packet or a stalled host thread can't run the arm.
const WATCHDOG_MS: u64 = 200;

type AxisSignal = Signal<CriticalSectionRawMutex, AxisCmd>;

static AXIS_CMD: [AxisSignal; AXES] = [
    Signal::new(),
    Signal::new(),
    Signal::new(),
    Signal::new(),
];

fn axis_index(s: &str) -> Option<usize> {
    match s {
        "x" | "X" => Some(0),
        "y" | "Y" => Some(1),
        "z" | "Z" => Some(2),
        "e" | "E" => Some(3),
        _ => None,
    }
}

// Mirror of what the CLI has last commanded; `status` reads this so we
// don't need a round-trip back from the axis tasks. Atomics so the axis
// tasks could in principle update them too if we ever need to (today only
// `send_cmd` writes).
struct AxisShadow {
    enabled: AtomicBool,
    velocity: AtomicI32,
    accel:    AtomicU32,
}
static SHADOW: [AxisShadow; AXES] = [
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0) },
];

fn send_cmd(idx: usize, cmd: AxisCmd) {
    match cmd {
        AxisCmd::Enable(on) => SHADOW[idx].enabled.store(on, Ordering::Relaxed),
        AxisCmd::Jog(hz)    => SHADOW[idx].velocity.store(hz, Ordering::Relaxed),
        AxisCmd::Accel(a)   => SHADOW[idx].accel.store(a, Ordering::Relaxed),
    }
    AXIS_CMD[idx].signal(cmd);
}

// --------------------------------------------------------------------------
// Fans: 3 plain on/off MOSFET outputs on the SKR Pico's fan headers
// (GP17, GP18, GP20). A small task per fan listens on a Signal so the CLI
// handler is non-blocking; FAN_STATE mirrors the last commanded value for
// `status` and so the host can drive the same pin from multiple buttons
// without round-tripping a query.
// --------------------------------------------------------------------------

type FanSignal = Signal<CriticalSectionRawMutex, bool>;
static FAN_CMD: [FanSignal; FANS] = [
    Signal::new(),
    Signal::new(),
    Signal::new(),
];
static FAN_STATE: [AtomicBool; FANS] = [
    AtomicBool::new(false),
    AtomicBool::new(false),
    AtomicBool::new(false),
];

fn fan_cmd(idx: usize, on: bool) {
    FAN_STATE[idx].store(on, Ordering::Relaxed);
    FAN_CMD[idx].signal(on);
}

#[embassy_executor::task(pool_size = FANS)]
async fn fan_task(idx: usize, mut out: Output<'static>) {
    out.set_low();
    loop {
        let on = FAN_CMD[idx].wait().await;
        if on {
            out.set_high();
        } else {
            out.set_low();
        }
        FAN_STATE[idx].store(on, Ordering::Relaxed);
    }
}

// --------------------------------------------------------------------------
// Servos: the four endstop headers (X/Y/Z/E0-STOP) plus the dedicated
// SERVOS header on GP29. All five sit on different RP2040 PWM slices, so
// each runs an independent 50 Hz channel with its own pulse width.
//
//   index 0 -> X-STOP   GP4   slice 2 chan A
//   index 1 -> Y-STOP   GP3   slice 1 chan B
//   index 2 -> Z-STOP   GP25  slice 4 chan B
//   index 3 -> E0-STOP  GP16  slice 0 chan A
//   index 4 -> SERVOS   GP29  slice 6 chan B
//
// Endstop headers are 3-pin (5V/GND/signal). The 5V pin is shared with the
// onboard switching reg - fine for small hobby servos, but power large /
// stalled servos from an external rail with a common ground.
//
// Pulse width is the only knob: 0 us disables PWM output (servo goes
// limp), otherwise typical values are 500..2500 us with 1500 us centre.
// --------------------------------------------------------------------------

/// 125 MHz sysclk / 64 = 1.953125 MHz tick. top+1 = 39063 -> 49.999 Hz.
const SERVO_DIVIDER: u8 = 64;
const SERVO_TOP: u16 = 39_062;
/// ticks per us = 1_953_125 / 1_000_000.
fn us_to_compare(us: u16) -> u16 {
    ((us as u32 * 1_953_125) / 1_000_000) as u16
}

/// true = pin is on channel A of its slice; false = channel B. Used by
/// the servo task to update the correct `compare_*` field when a new
/// pulse width arrives.
const SERVO_CHAN_A: [bool; SERVOS] = [true, false, false, true, false];

type ServoSignal = Signal<CriticalSectionRawMutex, u16>;
static SERVO_CMD: [ServoSignal; SERVOS] = [
    Signal::new(),
    Signal::new(),
    Signal::new(),
    Signal::new(),
    Signal::new(),
];
static SERVO_STATE: [AtomicU16; SERVOS] = [
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
    AtomicU16::new(0),
];

fn servo_cmd(idx: usize, us: u16) {
    SERVO_STATE[idx].store(us, Ordering::Relaxed);
    SERVO_CMD[idx].signal(us);
}

fn servo_base_config() -> PwmConfig {
    let mut c = PwmConfig::default();
    c.divider = SERVO_DIVIDER.into();
    c.top = SERVO_TOP;
    c.compare_a = 0;
    c.compare_b = 0;
    c
}

#[embassy_executor::task(pool_size = SERVOS)]
async fn servo_task(idx: usize, mut pwm: Pwm<'static>) {
    let mut cfg = servo_base_config();
    pwm.set_config(&cfg);
    loop {
        let us = SERVO_CMD[idx].wait().await;
        let compare = if us == 0 { 0 } else { us_to_compare(us) };
        if SERVO_CHAN_A[idx] {
            cfg.compare_a = compare;
        } else {
            cfg.compare_b = compare;
        }
        pwm.set_config(&cfg);
    }
}

#[embassy_executor::task(pool_size = AXES)]
async fn axis_task(
    idx: usize,
    mut step: Output<'static>,
    mut dir: Output<'static>,
    mut en: Output<'static>,
) {
    en.set_high(); // active-low: high = disabled
    step.set_low();
    dir.set_low();

    let sig = &AXIS_CMD[idx];
    let mut enabled = false;
    // Velocity is tracked in milli-Hz fixed point so a slow ramp (small
    // accel * short dt) doesn't truncate to zero increment.
    let mut target_v: i32 = 0;
    let mut current_milli: i64 = 0;
    let mut accel: u32 = 0; // steps/sec/sec; 0 = instant
    let mut last_cmd_at = Instant::now();
    let mut last_slew_at = Instant::now();
    let mut last_dir_sign: i32 = 0;

    let apply = |cmd: AxisCmd,
                 enabled: &mut bool,
                 target_v: &mut i32,
                 current_milli: &mut i64,
                 accel: &mut u32,
                 last_cmd_at: &mut Instant,
                 en: &mut Output<'static>| {
        *last_cmd_at = Instant::now();
        match cmd {
            AxisCmd::Enable(on) => {
                *enabled = on;
                if on {
                    en.set_low();
                } else {
                    en.set_high();
                    *target_v = 0;
                    *current_milli = 0;
                }
            }
            AxisCmd::Jog(hz) => {
                *target_v = hz;
                if hz != 0 && !*enabled {
                    en.set_low();
                    *enabled = true;
                }
            }
            AxisCmd::Accel(a) => {
                *accel = a;
            }
        }
    };

    loop {
        let cur_hz = (current_milli / 1000) as i32;

        if cur_hz == 0 && target_v == 0 {
            // Truly idle - block until a command arrives.
            let cmd = sig.wait().await;
            apply(cmd, &mut enabled, &mut target_v, &mut current_milli, &mut accel,
                  &mut last_cmd_at, &mut en);
            last_slew_at = Instant::now();
            continue;
        }

        // Sync DIR pin to the current velocity's sign. We do this before
        // the pulse so the driver sees a stable DIR by the time STEP rises;
        // the surrounding code paths add ample setup margin over the
        // TMC2209's 20 ns requirement.
        let sign = cur_hz.signum();
        if sign != 0 && sign != last_dir_sign {
            if sign > 0 { dir.set_high(); } else { dir.set_low(); }
            last_dir_sign = sign;
        }

        // Step pulse only if we're actually moving. When |current| is below
        // 1 Hz we just tick at 1 ms while the ramp builds.
        let stepping = cur_hz != 0;
        if stepping {
            step.set_high();
            Timer::after(Duration::from_micros(2)).await;
            step.set_low();
        }

        let abs_hz = cur_hz.unsigned_abs() as u64;
        let period_us: u64 = if abs_hz > 0 {
            (1_000_000u64 / abs_hz).max(4)
        } else {
            1000
        };
        let wait_us = if stepping {
            period_us.saturating_sub(2).max(2)
        } else {
            period_us
        };

        let result = select(Timer::after(Duration::from_micros(wait_us)), sig.wait()).await;

        // Slew current_milli toward target using the actual elapsed time --
        // works the same whether we got preempted by a command or rode the
        // timer out.
        let now = Instant::now();
        let dt_us = now.duration_since(last_slew_at).as_micros() as i64;
        last_slew_at = now;
        if accel == 0 {
            current_milli = (target_v as i64) * 1000;
        } else {
            let target_milli = (target_v as i64) * 1000;
            let max_dv = (accel as i64).saturating_mul(dt_us) / 1000;
            let diff = target_milli - current_milli;
            let dv = diff.clamp(-max_dv, max_dv);
            current_milli += dv;
            // Snap up to START_HZ once we're on the target's side of zero,
            // but only when the target is above the floor (otherwise a
            // legitimately slow target like 10 Hz would get clamped to 20).
            // Deceleration crossing zero passes through here naturally
            // (opposite signs => not on target side => no snap).
            let floor = (START_HZ as i64) * 1000;
            let target_milli_abs = (target_v as i64).abs() * 1000;
            if target_v != 0 && target_milli_abs > floor {
                let tgt_sign = (target_v as i64).signum();
                let on_target_side = current_milli.signum() == tgt_sign || current_milli == 0;
                if on_target_side && current_milli.abs() < floor {
                    current_milli = tgt_sign * floor;
                }
            }
        }

        match result {
            Either::First(_) => {
                // Deadman: host went silent. Hard halt (zero current, not
                // a graceful decel) so a cable yank doesn't let the arm
                // coast for the configured decel time. EN stays as-is so
                // the motor holds position.
                if now.duration_since(last_cmd_at) > Duration::from_millis(WATCHDOG_MS) {
                    target_v = 0;
                    current_milli = 0;
                    SHADOW[idx].velocity.store(0, Ordering::Relaxed);
                }
            }
            Either::Second(cmd) => {
                apply(cmd, &mut enabled, &mut target_v, &mut current_milli, &mut accel,
                      &mut last_cmd_at, &mut en);
            }
        }
    }
}

// --------------------------------------------------------------------------
// USB helpers
// --------------------------------------------------------------------------

struct Disconnected;

impl From<EndpointError> for Disconnected {
    fn from(_: EndpointError) -> Self {
        Disconnected
    }
}

async fn write_all(class: &mut Class, data: &[u8]) -> Result<(), Disconnected> {
    for chunk in data.chunks(64) {
        class.write_packet(chunk).await?;
    }
    Ok(())
}

async fn writeln(class: &mut Class, s: &str) -> Result<(), Disconnected> {
    write_all(class, s.as_bytes()).await?;
    write_all(class, b"\r\n").await
}

async fn prompt(class: &mut Class) -> Result<(), Disconnected> {
    write_all(class, b"> ").await
}

// --------------------------------------------------------------------------
// Command dispatch
// --------------------------------------------------------------------------

async fn run_command(class: &mut Class, line: &str) -> Result<(), Disconnected> {
    let mut it = line.split_ascii_whitespace();
    let Some(cmd) = it.next() else {
        return Ok(());
    };
    match cmd {
        "help" | "?" => {
            writeln(class, "commands:").await?;
            writeln(class, "  help").await?;
            writeln(class, "  version").await?;
            writeln(class, "  status").await?;
            writeln(class, "  enable  <x|y|z|e|all>").await?;
            writeln(class, "  disable <x|y|z|e|all>").await?;
            writeln(class, "  accel   <axis|all> <hz/sec>    0=instant slew (default)").await?;
            writeln(class, "  jog     <axis> <signed_hz>     0 stops; sign sets DIR").await?;
            writeln(class, "  move    <axis> <steps> [hz]    one-shot, duration-based").await?;
            writeln(class, "  fan     <0|1|2> <on|off>       SKR Pico fan headers").await?;
            writeln(class, "  servo   <0..4> <us>            0=off, 500..2500 typical").await?;
            writeln(class, "max hz 40000.").await?;
        }
        "version" | "ver" => {
            let mut buf: String<64> = String::new();
            let _ = write!(buf, "tmc-new-era firmware v{FW_VERSION}\r\n");
            write_all(class, buf.as_bytes()).await?;
        }
        "status" => {
            let mut buf: String<512> = String::new();
            for i in 0..AXES {
                let _ = write!(
                    buf,
                    "  {}: {} vel={} accel={}\r\n",
                    AXIS_NAMES[i],
                    if SHADOW[i].enabled.load(Ordering::Relaxed) { "EN" } else { "--" },
                    SHADOW[i].velocity.load(Ordering::Relaxed),
                    SHADOW[i].accel.load(Ordering::Relaxed),
                );
            }
            for i in 0..FANS {
                let _ = write!(
                    buf,
                    "  fan{}: {}\r\n",
                    i,
                    if FAN_STATE[i].load(Ordering::Relaxed) { "ON" } else { "off" },
                );
            }
            for i in 0..SERVOS {
                let us = SERVO_STATE[i].load(Ordering::Relaxed);
                let _ = write!(buf, "  servo{} ({}): {} us\r\n", i, SERVO_NAMES[i], us);
            }
            write_all(class, buf.as_bytes()).await?;
        }
        "enable" | "disable" => {
            let on = cmd == "enable";
            let Some(arg) = it.next() else {
                writeln(class, "usage: enable|disable <x|y|z|e|all>").await?;
                return Ok(());
            };
            if arg == "all" {
                for i in 0..AXES {
                    send_cmd(i, AxisCmd::Enable(on));
                }
            } else if let Some(i) = axis_index(arg) {
                send_cmd(i, AxisCmd::Enable(on));
            } else {
                writeln(class, "unknown axis").await?;
                return Ok(());
            }
            writeln(class, if on { "ok enabled" } else { "ok disabled" }).await?;
        }
        "accel" => {
            let (Some(arg), Some(val_s)) = (it.next(), it.next()) else {
                writeln(class, "usage: accel <axis|all> <hz/sec>").await?;
                return Ok(());
            };
            let Ok(a) = val_s.parse::<u32>() else {
                writeln(class, "bad accel").await?;
                return Ok(());
            };
            if arg == "all" {
                for i in 0..AXES {
                    send_cmd(i, AxisCmd::Accel(a));
                }
            } else if let Some(i) = axis_index(arg) {
                send_cmd(i, AxisCmd::Accel(a));
            } else {
                writeln(class, "unknown axis").await?;
                return Ok(());
            }
            let mut buf: String<48> = String::new();
            let _ = write!(buf, "ok accel {} {}\r\n", arg, a);
            write_all(class, buf.as_bytes()).await?;
        }
        "jog" => {
            let (Some(axis_s), Some(hz_s)) = (it.next(), it.next()) else {
                writeln(class, "usage: jog <axis> <signed_hz>").await?;
                return Ok(());
            };
            let Some(idx) = axis_index(axis_s) else {
                writeln(class, "unknown axis").await?;
                return Ok(());
            };
            let Ok(hz) = hz_s.parse::<i32>() else {
                writeln(class, "bad hz").await?;
                return Ok(());
            };
            let clamped = hz.clamp(-(MAX_HZ as i32), MAX_HZ as i32);
            send_cmd(idx, AxisCmd::Jog(clamped));
            let mut buf: String<48> = String::new();
            let _ = write!(buf, "ok jog {} {}\r\n", AXIS_NAMES[idx], clamped);
            write_all(class, buf.as_bytes()).await?;
        }
        "move" => {
            let (Some(axis_s), Some(steps_s)) = (it.next(), it.next()) else {
                writeln(class, "usage: move <axis> <steps> [hz]").await?;
                return Ok(());
            };
            let Some(idx) = axis_index(axis_s) else {
                writeln(class, "unknown axis").await?;
                return Ok(());
            };
            let Ok(steps) = steps_s.parse::<i32>() else {
                writeln(class, "bad steps").await?;
                return Ok(());
            };
            let hz: u32 = match it.next() {
                Some(s) => match s.parse::<u32>() {
                    Ok(v) if v > 0 => v.min(MAX_HZ),
                    _ => {
                        writeln(class, "bad hz").await?;
                        return Ok(());
                    }
                },
                None => 1000,
            };
            if steps == 0 || hz == 0 {
                writeln(class, "ok").await?;
                return Ok(());
            }
            // Issue a signed jog, sleep for the count's nominal duration,
            // then halt. Not exact - host should treat move as approximate.
            let signed = if steps > 0 { hz as i32 } else { -(hz as i32) };
            let duration_us = (steps.unsigned_abs() as u64) * 1_000_000u64 / hz as u64;
            send_cmd(idx, AxisCmd::Jog(signed));
            Timer::after(Duration::from_micros(duration_us)).await;
            send_cmd(idx, AxisCmd::Jog(0));
            writeln(class, "done").await?;
        }
        "fan" => {
            let (Some(idx_s), Some(state_s)) = (it.next(), it.next()) else {
                writeln(class, "usage: fan <0|1|2> <on|off>").await?;
                return Ok(());
            };
            let Ok(idx) = idx_s.parse::<usize>() else {
                writeln(class, "bad fan number").await?;
                return Ok(());
            };
            if idx >= FANS {
                writeln(class, "fan out of range").await?;
                return Ok(());
            }
            let on = match state_s {
                "on" | "1" | "ON" => true,
                "off" | "0" | "OFF" => false,
                _ => {
                    writeln(class, "bad fan state (on|off)").await?;
                    return Ok(());
                }
            };
            fan_cmd(idx, on);
            let mut buf: String<48> = String::new();
            let _ = write!(buf, "ok fan {} {}\r\n", idx, if on { "on" } else { "off" });
            write_all(class, buf.as_bytes()).await?;
        }
        "servo" => {
            let (Some(idx_s), Some(us_s)) = (it.next(), it.next()) else {
                writeln(class, "usage: servo <0..4> <us|off>").await?;
                return Ok(());
            };
            let Ok(idx) = idx_s.parse::<usize>() else {
                writeln(class, "bad servo number").await?;
                return Ok(());
            };
            if idx >= SERVOS {
                writeln(class, "servo out of range (0..4)").await?;
                return Ok(());
            }
            let us: u16 = match us_s {
                "off" | "OFF" | "0" => 0,
                s => match s.parse::<u16>() {
                    // Cap at half the period so a typo can't accidentally
                    // command a 100% duty cycle into the servo.
                    Ok(v) => v.min(10_000),
                    Err(_) => {
                        writeln(class, "bad us (number or 'off')").await?;
                        return Ok(());
                    }
                },
            };
            servo_cmd(idx, us);
            let mut buf: String<64> = String::new();
            let _ = write!(buf, "ok servo {} {} us\r\n", idx, us);
            write_all(class, buf.as_bytes()).await?;
        }
        _ => {
            writeln(class, "unknown command - try 'help'").await?;
        }
    }
    Ok(())
}

async fn cli_session(class: &mut Class) -> Result<(), Disconnected> {
    writeln(class, "").await?;
    let mut banner: String<80> = String::new();
    let _ = write!(banner, "tmc-new-era v{FW_VERSION} - SKR Pico stepper CLI");
    writeln(class, banner.as_str()).await?;
    writeln(class, "type 'help' for commands").await?;
    prompt(class).await?;

    let mut line: String<128> = String::new();
    let mut buf = [0u8; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        for &b in &buf[..n] {
            match b {
                b'\r' | b'\n' => {
                    write_all(class, b"\r\n").await?;
                    if !line.is_empty() {
                        run_command(class, line.as_str()).await?;
                        line.clear();
                    }
                    prompt(class).await?;
                }
                0x7f | 0x08 => {
                    if line.pop().is_some() {
                        write_all(class, b"\x08 \x08").await?;
                    }
                }
                0x03 => {
                    line.clear();
                    write_all(class, b"^C\r\n").await?;
                    prompt(class).await?;
                }
                _ if (0x20..0x7f).contains(&b) => {
                    if line.push(b as char).is_ok() {
                        write_all(class, &[b]).await?;
                    }
                }
                _ => {}
            }
        }
    }
}

// --------------------------------------------------------------------------
// Entry point
// --------------------------------------------------------------------------

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Spawn one task per axis. Pins come straight from the working
    // SKR Pico klipper.cfg.
    spawner.spawn(
        axis_task(
            0,
            Output::new(p.PIN_11, Level::Low),
            Output::new(p.PIN_10, Level::Low),
            Output::new(p.PIN_12, Level::High),
        )
        .expect("axis task pool full"),
    );
    spawner.spawn(
        axis_task(
            1,
            Output::new(p.PIN_6, Level::Low),
            Output::new(p.PIN_5, Level::Low),
            Output::new(p.PIN_7, Level::High),
        )
        .expect("axis task pool full"),
    );
    spawner.spawn(
        axis_task(
            2,
            Output::new(p.PIN_19, Level::Low),
            Output::new(p.PIN_28, Level::Low),
            Output::new(p.PIN_2, Level::High),
        )
        .expect("axis task pool full"),
    );
    spawner.spawn(
        axis_task(
            3,
            Output::new(p.PIN_14, Level::Low),
            Output::new(p.PIN_13, Level::Low),
            Output::new(p.PIN_15, Level::High),
        )
        .expect("axis task pool full"),
    );

    // Fan headers on the SKR Pico: GP17, GP18, GP20. Plain on/off.
    spawner.spawn(
        fan_task(0, Output::new(p.PIN_17, Level::Low))
            .expect("fan task pool full"),
    );
    spawner.spawn(
        fan_task(1, Output::new(p.PIN_18, Level::Low))
            .expect("fan task pool full"),
    );
    spawner.spawn(
        fan_task(2, Output::new(p.PIN_20, Level::Low))
            .expect("fan task pool full"),
    );

    // Servos: four endstop headers repurposed as PWM outputs, plus the
    // dedicated SERVOS header on GP29. See the SERVO_CHAN_A table for the
    // slice/channel assignments these calls assume.
    let cfg0 = servo_base_config();
    spawner.spawn(
        servo_task(0, Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, cfg0.clone()))
            .expect("servo task pool full"),
    );
    spawner.spawn(
        servo_task(1, Pwm::new_output_b(p.PWM_SLICE1, p.PIN_3, cfg0.clone()))
            .expect("servo task pool full"),
    );
    spawner.spawn(
        servo_task(2, Pwm::new_output_b(p.PWM_SLICE4, p.PIN_25, cfg0.clone()))
            .expect("servo task pool full"),
    );
    spawner.spawn(
        servo_task(3, Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, cfg0.clone()))
            .expect("servo task pool full"),
    );
    spawner.spawn(
        servo_task(4, Pwm::new_output_b(p.PWM_SLICE6, p.PIN_29, cfg0))
            .expect("servo task pool full"),
    );

    let driver = Driver::new(p.USB, Irqs);

    let mut usb_cfg = Config::new(0x16c0, 0x27dd);
    usb_cfg.manufacturer = Some("tmc-new-era");
    usb_cfg.product = Some("SKR Pico Stepper CLI");
    usb_cfg.serial_number = Some("SKRPICO-001");
    usb_cfg.max_power = 100;
    usb_cfg.max_packet_size_0 = 64;

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        usb_cfg,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let mut class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);

    let usb = builder.build();
    spawner.spawn(usb_task(usb).expect("usb task pool full"));

    loop {
        class.wait_connection().await;
        let _ = cli_session(&mut class).await;
        // Disconnect: halt and de-energize so a yanked cable doesn't leave
        // a coil powered indefinitely. Fans/servos go off too - a stuck-on
        // output after the host vanished is a footgun.
        for i in 0..AXES {
            send_cmd(i, AxisCmd::Jog(0));
            send_cmd(i, AxisCmd::Enable(false));
        }
        for i in 0..FANS {
            fan_cmd(i, false);
        }
        for i in 0..SERVOS {
            servo_cmd(i, 0);
        }
    }
}
