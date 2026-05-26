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
use embassy_rp::peripherals::{UART1, USB};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use embedded_io_async::{Read, Write};
use heapless::String;
use panic_halt as _;
use static_cell::StaticCell;
use tmc2209::reg as tmc_reg;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
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
    enabled:    AtomicBool,
    velocity:   AtomicI32,
    accel:      AtomicU32,
    /// Last-configured microstep count via the `tmc` command. 0 means
    /// "never set" -- driver is running in standalone mode (MS1/MS2 pins
    /// select microsteps; see NOTES.md).
    microsteps: AtomicU16,
}
static SHADOW: [AxisShadow; AXES] = [
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0), microsteps: AtomicU16::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0), microsteps: AtomicU16::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0), microsteps: AtomicU16::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0), accel: AtomicU32::new(0), microsteps: AtomicU16::new(0) },
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
// TMC2209 UART configuration.
//
// All four drivers share GP8 (TX, half-duplex via a series resistor on the
// SKR Pico) and GP9 (RX). UART addresses come from the MS1/MS2 straps:
//
//   X -> 0   Z -> 1   Y -> 2   E -> 3
//
// We bring up UART1 at 115200 baud (TMC2209 auto-bauds off the 0x05 sync
// byte) and run a single tmc_task that consumes TmcCfg messages from a
// channel and writes GCONF / IHOLD_IRUN / CHOPCONF to the addressed
// driver. The drivers don't reply to writes; we still drain the echoed
// bytes that loop back on our own RX pin so the buffer doesn't overflow
// across multiple updates.
//
// If no `tmc` command is ever sent the UART stays idle and the drivers
// run in standalone mode (MS1/MS2 still select microsteps, see NOTES.md).
// --------------------------------------------------------------------------

/// Sense-resistor value on the SKR Pico, in milliohms. Hardcoded because
/// it's a board-level constant; if you reflash this firmware onto a
/// different carrier with a different Rsense, change this constant.
const RSENSE_MOHM: u64 = 110;

/// Per-axis UART address (MS2,MS1 strap order on the SKR Pico):
/// X=0, Z=1, Y=2, E=3. Indexed by AXES order [X, Y, Z, E].
const TMC_ADDR: [u8; AXES] = [0, 2, 1, 3];

#[derive(Clone, Copy)]
struct TmcCfg {
    axis:        u8,
    microsteps:  u16,
    run_ma:      u32,
    hold_ma:     u32,
    hold_delay:  u8,
    spreadcycle: bool,
    interpolate: bool,
}

static TMC_CHAN: Channel<CriticalSectionRawMutex, TmcCfg, 8> = Channel::new();

/// Convert a desired RMS current (mA) to a (vsense, CS) pair for the
/// IHOLD_IRUN register. Uses the standard formula
///   I_rms = ((CS+1) / 32) * V_fs / (R_sense + 20mohm) / sqrt(2)
/// solved for CS, with sqrt(2) approximated as 14142/10000. Prefers
/// vsense=1 (V_fs = 180 mV, higher resolution) when CS fits; falls back
/// to vsense=0 (V_fs = 325 mV) for higher currents.
fn current_to_vsense_cs(current_ma: u32) -> (bool, u8) {
    if current_ma == 0 {
        return (true, 0);
    }
    let r_eff = RSENSE_MOHM + 20;
    // num = I_mA * R_mohm * 32 * sqrt(2) * 10000  (units: uV * 10000)
    let num = (current_ma as u64) * r_eff * 32 * 14142;
    // Denominator absorbs the *10000 sqrt scale: V_fs_mV * 1000 * 10000.
    // V_fs_mV * 10_000_000 -> rounded division gives (CS+1).
    let denom_v1 = 180u64 * 10_000_000;
    let cs_plus1 = (num + denom_v1 / 2) / denom_v1;
    if cs_plus1 >= 1 && cs_plus1 <= 32 {
        return (true, (cs_plus1 - 1) as u8);
    }
    let denom_v0 = 325u64 * 10_000_000;
    let cs_plus1 = (num + denom_v0 / 2) / denom_v0;
    let cs = cs_plus1.saturating_sub(1).min(31);
    (false, cs as u8)
}

/// Map a microstep count (8/16/32/64/128/256, or 1/2/4) to CHOPCONF.MRES.
/// Returns None for unsupported values.
fn microsteps_to_mres(ms: u16) -> Option<u8> {
    Some(match ms {
        256 => 0,
        128 => 1,
        64  => 2,
        32  => 3,
        16  => 4,
        8   => 5,
        4   => 6,
        2   => 7,
        1   => 8,
        _   => return None,
    })
}

#[embassy_executor::task]
async fn tmc_task(mut uart: BufferedUart) {
    loop {
        let cfg = TMC_CHAN.receive().await;
        apply_tmc(&mut uart, cfg).await;
    }
}

async fn apply_tmc(uart: &mut BufferedUart, cfg: TmcCfg) {
    let axis = cfg.axis as usize;
    if axis >= AXES {
        return;
    }
    let Some(mres) = microsteps_to_mres(cfg.microsteps) else { return };
    let addr = TMC_ADDR[axis];
    let (vsense, irun) = current_to_vsense_cs(cfg.run_ma);
    let (_, ihold)     = current_to_vsense_cs(cfg.hold_ma);

    let mut gconf = tmc_reg::GCONF::default();
    gconf.set_pdn_disable(true);
    gconf.set_mstep_reg_select(true);
    gconf.set_i_scale_analog(false);
    gconf.set_en_spread_cycle(cfg.spreadcycle);

    let mut ihold_irun = tmc_reg::IHOLD_IRUN::default();
    ihold_irun.set_irun(irun);
    ihold_irun.set_ihold(ihold);
    ihold_irun.set_ihold_delay(cfg.hold_delay);

    let mut chop = tmc_reg::CHOPCONF::default();
    chop.set_mres(tmc2209::data::MicroStepResolution::from(mres as u32));
    chop.set_intpol(cfg.interpolate);
    chop.set_vsense(vsense);

    // Send each write datagram, then drain echoed bytes from our own TX
    // (single-wire bus loops them back onto RX). Drivers don't respond to
    // writes so 8 bytes is the full expected echo.
    tmc_write(uart, tmc2209::WriteRequest::new(addr, gconf).bytes()).await;
    tmc_write(uart, tmc2209::WriteRequest::new(addr, ihold_irun).bytes()).await;
    tmc_write(uart, tmc2209::WriteRequest::new(addr, chop).bytes()).await;

    SHADOW[axis].microsteps.store(cfg.microsteps, Ordering::Relaxed);
}

async fn tmc_write(uart: &mut BufferedUart, datagram: &[u8]) {
    let _ = uart.write_all(datagram).await;
    let _ = uart.flush().await;
    let mut echo = [0u8; 8];
    // Echoes arrive ~700 us after the last bit at 115200 baud; 20 ms is
    // overkill but keeps us robust to interrupt latency.
    let _ = with_timeout(Duration::from_millis(20), uart.read_exact(&mut echo)).await;
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
            writeln(class, "  tmc     <axis> <ms> <run_ma> <hold_ma> <hold_delay> <sc> <int>").await?;
            writeln(class, "                                 configure driver over UART").await?;
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
                let ms = SHADOW[i].microsteps.load(Ordering::Relaxed);
                let _ = write!(
                    buf,
                    "  {}: {} vel={} accel={} ms={}\r\n",
                    AXIS_NAMES[i],
                    if SHADOW[i].enabled.load(Ordering::Relaxed) { "EN" } else { "--" },
                    SHADOW[i].velocity.load(Ordering::Relaxed),
                    SHADOW[i].accel.load(Ordering::Relaxed),
                    ms,
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
        "tmc" => {
            // tmc <axis> <microsteps> <run_ma> <hold_ma> <hold_delay> <spreadcycle> <interpolate>
            let parts = [it.next(), it.next(), it.next(), it.next(), it.next(), it.next(), it.next()];
            let [Some(axis_s), Some(ms_s), Some(run_s), Some(hold_s), Some(hd_s), Some(sc_s), Some(int_s)] = parts else {
                writeln(class, "usage: tmc <axis> <ms> <run_ma> <hold_ma> <hold_delay> <sc> <int>").await?;
                return Ok(());
            };
            let Some(idx) = axis_index(axis_s) else {
                writeln(class, "unknown axis").await?;
                return Ok(());
            };
            let (Ok(ms), Ok(run_ma), Ok(hold_ma), Ok(hold_delay)) = (
                ms_s.parse::<u16>(),
                run_s.parse::<u32>(),
                hold_s.parse::<u32>(),
                hd_s.parse::<u8>(),
            ) else {
                writeln(class, "bad numeric arg").await?;
                return Ok(());
            };
            if microsteps_to_mres(ms).is_none() {
                writeln(class, "microsteps must be 1/2/4/8/16/32/64/128/256").await?;
                return Ok(());
            }
            let parse_bool = |s: &str| matches!(s, "1" | "true" | "on" | "yes");
            let cfg = TmcCfg {
                axis: idx as u8,
                microsteps: ms,
                run_ma,
                hold_ma,
                hold_delay,
                spreadcycle: parse_bool(sc_s),
                interpolate: parse_bool(int_s),
            };
            // Channel is bounded; if it fills the host is spamming faster
            // than the UART task can drain. Drop and tell the host so they
            // can retry rather than block the CLI forever.
            if TMC_CHAN.try_send(cfg).is_err() {
                writeln(class, "busy").await?;
                return Ok(());
            }
            let mut buf: String<80> = String::new();
            let _ = write!(buf, "ok tmc {} queued\r\n", AXIS_NAMES[idx]);
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

    // TMC2209 single-wire UART on GP8/GP9 at 115200 baud. The TMC chips
    // baud-detect off the 0x05 sync byte so any baud in 9600..500000 works.
    // RX_BUF is sized for the worst-case "configure four axes back to
    // back" burst (4 axes * 3 writes * 8 bytes = 96 bytes of echo);
    // tmc_task drains between writes anyway, so 128 is comfortable.
    static TMC_TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static TMC_RX_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let tmc_uart = BufferedUart::new(
        p.UART1,
        p.PIN_8,
        p.PIN_9,
        Irqs,
        TMC_TX_BUF.init([0; 64]),
        TMC_RX_BUF.init([0; 128]),
        uart_cfg,
    );
    spawner.spawn(tmc_task(tmc_uart).expect("tmc task pool full"));

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
