// Host-side joystick driver for the SKR Pico firmware.
//
// Reads a TOML config describing which joystick axis/button maps to which
// stepper axis, opens /dev/input/jsN, opens the firmware's USB CDC serial,
// and translates stick deflections into `jog <axis> <signed_hz>` commands.
//
//   tmc-new-era-host                   # uses ./config.toml
//   tmc-new-era-host path/to/cfg.toml
//
// Stops cleanly on Ctrl-C, sending `jog * 0` to every axis first.

use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Write};
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::mpsc::{Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::Deserialize;

// --------------------------------------------------------------------------
// Config
// --------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
struct Config {
    /// Path to the firmware's USB CDC serial device.
    serial: String,
    /// Path to the joystick device. Linux `/dev/input/jsN`.
    device: String,
    /// Integrator tick rate. 40 Hz matches the reference. Clamped to [10,200].
    #[serde(default = "default_poll_hz")]
    poll_hz: u32,
    /// Print every command we send. Useful for first bring-up.
    #[serde(default)]
    log: bool,
    /// Enable all stepper axes on startup.
    #[serde(default = "default_true")]
    enable_on_start: bool,
    /// Send `jog 0` + `disable all` on Ctrl-C.
    #[serde(default = "default_true")]
    safe_stop_on_exit: bool,

    #[serde(default)]
    axes: Vec<AxisMap>,
    #[serde(default)]
    buttons: Vec<ButtonMap>,
}

fn default_poll_hz() -> u32 { 40 }
fn default_true() -> bool { true }

#[derive(Debug, Deserialize, Clone)]
struct AxisMap {
    /// Joystick axis index (`/dev/input/js0` numbering).
    index: usize,
    /// Stepper axis: x | y | z | e | none.
    target: String,
    /// Max signed steps/sec at full deflection.
    sensitivity: f32,
    /// Fraction (0..0.95) of stick travel treated as zero.
    #[serde(default)]
    deadzone: f32,
    /// Flip the sign of the stick reading.
    #[serde(default)]
    invert: bool,
}

#[derive(Debug, Deserialize, Clone)]
struct ButtonMap {
    /// Joystick button index.
    index: usize,
    /// Stepper axis: x | y | z | e | none.
    target: String,
    /// Signed step delta to send on press.
    delta: i32,
    /// Step rate for the nudge.
    #[serde(default = "default_button_hz")]
    hz: u32,
}

fn default_button_hz() -> u32 { 1000 }

// --------------------------------------------------------------------------
// Linux joystick event format (linux/joystick.h)
// --------------------------------------------------------------------------

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct JsEvent {
    time:   u32, // ms since boot
    value:  i16, // -32768..32767 for axis, 0/1 for button
    ev_type: u8, // 0x01 button, 0x02 axis, |0x80 = init/synthetic
    number: u8,
}
const JS_EVENT_SIZE: usize = 8;
const JS_EVENT_BUTTON: u8 = 0x01;
const JS_EVENT_AXIS:   u8 = 0x02;
const JS_EVENT_INIT:   u8 = 0x80;
const JS_MAX: f32 = 32767.0;

fn parse_js_event(buf: &[u8]) -> JsEvent {
    let mut e = JsEvent::default();
    e.time    = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    e.value   = i16::from_le_bytes(buf[4..6].try_into().unwrap());
    e.ev_type = buf[6];
    e.number  = buf[7];
    e
}

// --------------------------------------------------------------------------
// Shared joystick snapshot
// --------------------------------------------------------------------------

#[derive(Default)]
struct Snapshot {
    /// Latest raw axis values (-32768..32767), indexed by js axis number.
    axes: Vec<i16>,
}

// --------------------------------------------------------------------------
// Serial writer thread: owns the port, consumes a channel of command strings.
// --------------------------------------------------------------------------

fn spawn_serial(path: String) -> Result<Sender<String>> {
    let mut port = serialport::new(&path, 115_200)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening serial port {path}"))?;
    let (tx, rx) = channel::<String>();
    thread::Builder::new()
        .name("serial-writer".into())
        .spawn(move || {
            for cmd in rx {
                let _ = port.write_all(cmd.as_bytes());
                let _ = port.flush();
            }
        })?;
    Ok(tx)
}

// --------------------------------------------------------------------------
// Axis name -> firmware token
// --------------------------------------------------------------------------

fn axis_token(s: &str) -> Option<&'static str> {
    match s.to_ascii_lowercase().as_str() {
        "x" => Some("x"),
        "y" => Some("y"),
        "z" => Some("z"),
        "e" => Some("e"),
        "none" | "" => None,
        _ => None,
    }
}

// --------------------------------------------------------------------------
// Joystick reader: blocks reading 8-byte events, updates the snapshot,
// fires button-press callbacks for state transitions.
// --------------------------------------------------------------------------

fn open_joystick(path: &str) -> Result<File> {
    let f = std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc_o_nonblock())
        .open(path)
        .with_context(|| format!("opening joystick {path}"))?;
    Ok(f)
}

// We avoid pulling in `libc` just for one constant.
const fn libc_o_nonblock() -> i32 { 0o4000 }

fn reader_loop(
    mut fd: File,
    snapshot: Arc<Mutex<Snapshot>>,
    on_button_press: impl Fn(u8),
) {
    let mut buttons: HashMap<u8, i16> = HashMap::new();
    let mut buf = [0u8; JS_EVENT_SIZE];
    loop {
        match fd.read(&mut buf) {
            Ok(n) if n == JS_EVENT_SIZE => {
                let ev = parse_js_event(&buf);
                let is_init = ev.ev_type & JS_EVENT_INIT != 0;
                let kind = ev.ev_type & !JS_EVENT_INIT;
                if kind == JS_EVENT_AXIS {
                    let mut s = snapshot.lock().unwrap();
                    if (ev.number as usize) >= s.axes.len() {
                        s.axes.resize(ev.number as usize + 1, 0);
                    }
                    s.axes[ev.number as usize] = ev.value;
                } else if kind == JS_EVENT_BUTTON {
                    let prev = buttons.insert(ev.number, ev.value).unwrap_or(0);
                    if ev.value != 0 && prev == 0 && !is_init {
                        on_button_press(ev.number);
                    }
                }
            }
            Ok(_) => {
                // Short read - shouldn't happen on a joystick fd, just retry.
                thread::sleep(Duration::from_millis(5));
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                thread::sleep(Duration::from_millis(5));
            }
            Err(e) => {
                eprintln!("joystick read error: {e}");
                thread::sleep(Duration::from_millis(500));
            }
        }
        // Cheap check that the fd is still alive via poll() would be nice,
        // but for v1 we rely on the read error path above.
        let _ = fd.as_raw_fd();
    }
}

// --------------------------------------------------------------------------
// Integrator: at poll_hz, computes each mapped axis's signed velocity and
// emits a `jog` command iff it changed. Mirrors the reference's bucketed
// chatter suppression.
// --------------------------------------------------------------------------

fn integrate_loop(
    cfg: &Config,
    snapshot: Arc<Mutex<Snapshot>>,
    tx: Sender<String>,
) {
    let period = Duration::from_secs_f32(1.0 / cfg.poll_hz.clamp(10, 200) as f32);
    // Map: stepper axis token -> (sign, magnitude) last sent. Lets us
    // suppress duplicate commands when the user holds the stick still.
    let mut last: HashMap<&'static str, (i32, u32)> = HashMap::new();
    let mut next_tick = Instant::now();

    loop {
        next_tick += period;
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        } else {
            // Fell behind; resync.
            next_tick = Instant::now() + period;
        }

        let axes_state = snapshot.lock().unwrap().axes.clone();

        for map in &cfg.axes {
            let Some(target) = axis_token(&map.target) else { continue };
            let raw = match axes_state.get(map.index) {
                Some(v) => *v as f32 / JS_MAX,
                None => 0.0,
            };
            let raw = if map.invert { -raw } else { raw };
            let dz = map.deadzone.clamp(0.0, 0.95);
            let mag = raw.abs();
            let signed_v = if mag < dz {
                0.0
            } else {
                let norm = (mag - dz) / (1.0 - dz) * raw.signum();
                norm * map.sensitivity
            };

            // Bucket to nearest 25 steps/s so a noisy stick doesn't spam.
            let (new_sign, new_mag): (i32, u32) = if signed_v.abs() < 1.0 {
                let prev = last.get(&target).copied().unwrap_or((0, 0));
                (prev.0, 0)
            } else {
                let s = if signed_v > 0.0 { 1 } else { -1 };
                let m = ((signed_v.abs() / 25.0).round() as u32).max(1) * 25;
                (s, m)
            };

            let prev = last.get(&target).copied().unwrap_or((0, 0));
            if (new_sign, new_mag) == prev {
                continue;
            }
            // Skip an idle-to-idle nothing.
            if new_mag == 0 && prev.1 == 0 {
                last.insert(target, (new_sign, new_mag));
                continue;
            }
            last.insert(target, (new_sign, new_mag));
            let signed_hz: i32 = if new_mag == 0 {
                0
            } else {
                new_sign * new_mag as i32
            };
            send(&tx, &format!("jog {target} {signed_hz}\r\n"), cfg.log);
        }
    }
}

fn send(tx: &Sender<String>, cmd: &str, log: bool) {
    if log {
        let trimmed = cmd.trim_end();
        println!("> {trimmed}");
    }
    let _ = tx.send(cmd.to_string());
}

// --------------------------------------------------------------------------
// main
// --------------------------------------------------------------------------

fn load_config(path: &str) -> Result<Config> {
    let mut s = String::new();
    File::open(path)
        .with_context(|| format!("opening config {path}"))?
        .read_to_string(&mut s)?;
    let cfg: Config = toml::from_str(&s)
        .with_context(|| format!("parsing config {path}"))?;
    Ok(cfg)
}

fn main() -> Result<()> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "config.toml".into());
    let cfg = Arc::new(load_config(&path)?);

    let tx = spawn_serial(cfg.serial.clone())?;

    // Give the firmware a moment to settle on a fresh connection.
    thread::sleep(Duration::from_millis(200));
    send(&tx, "\r\n", cfg.log);
    if cfg.enable_on_start {
        send(&tx, "enable all\r\n", cfg.log);
    }

    let snapshot = Arc::new(Mutex::new(Snapshot::default()));

    // Joystick reader thread.
    let fd = open_joystick(&cfg.device)
        .with_context(|| format!("opening joystick {}", cfg.device))?;
    let snap_for_reader = snapshot.clone();
    let buttons = cfg.buttons.clone();
    let tx_for_buttons = tx.clone();
    let log_buttons = cfg.log;
    thread::Builder::new().name("js-reader".into()).spawn(move || {
        reader_loop(fd, snap_for_reader, move |btn| {
            if let Some(b) = buttons.iter().find(|b| b.index as u8 == btn) {
                if let Some(target) = axis_token(&b.target) {
                    let cmd = format!("move {target} {} {}\r\n", b.delta, b.hz);
                    send(&tx_for_buttons, &cmd, log_buttons);
                }
            }
        });
    })?;

    // Integrator thread.
    let cfg_for_int = cfg.clone();
    let tx_for_int = tx.clone();
    thread::Builder::new().name("integrator".into()).spawn(move || {
        integrate_loop(&cfg_for_int, snapshot, tx_for_int);
    })?;

    eprintln!(
        "tmc-new-era-host: serial={} device={} poll_hz={} axes={} buttons={}",
        cfg.serial, cfg.device, cfg.poll_hz, cfg.axes.len(), cfg.buttons.len()
    );
    eprintln!("Ctrl-C to stop.");

    // Catch Ctrl-C so we can broadcast `jog 0` + `disable all` before exit.
    install_sigint();
    loop {
        if STOP.load(std::sync::atomic::Ordering::SeqCst) {
            if cfg.safe_stop_on_exit {
                for a in ["x", "y", "z", "e"] {
                    send(&tx, &format!("jog {a} 0\r\n"), cfg.log);
                }
                send(&tx, "disable all\r\n", cfg.log);
                thread::sleep(Duration::from_millis(150));
            }
            std::process::exit(130);
        }
        thread::sleep(Duration::from_millis(100));
    }
}

// --------------------------------------------------------------------------
// Minimal SIGINT trap (no ctrlc / signal-hook dep).
// --------------------------------------------------------------------------

use std::sync::atomic::AtomicBool;
static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigint(_: i32) {
    STOP.store(true, std::sync::atomic::Ordering::SeqCst);
}

fn install_sigint() {
    unsafe extern "C" {
        fn signal(signum: i32, handler: usize) -> usize;
    }
    const SIGINT: i32 = 2;
    unsafe { signal(SIGINT, on_sigint as *const () as usize); }
}
