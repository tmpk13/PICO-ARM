// Host-side joystick driver for the SKR Pico firmware.
//
// Reads a TOML config describing one or more boards. Each board has its
// own USB CDC serial port and its own axis/button bindings, so a single
// joystick can drive multiple SKR Picos (e.g. one arm split across two
// boards). The reader thread is global; integrator state and serial I/O
// are per-board.
//
//   tmc-new-era-host                   # uses ./config.toml
//   tmc-new-era-host path/to/cfg.toml
//
// Stops cleanly on Ctrl-C, sending `jog * 0` + `disable all` to every
// configured board first.

use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use serde::Deserialize;

// --------------------------------------------------------------------------
// Config
// --------------------------------------------------------------------------

#[derive(Debug, Deserialize)]
struct Config {
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

    /// One entry per SKR Pico. Joystick events are dispatched to each
    /// board independently based on its own axes/buttons bindings.
    #[serde(default)]
    boards: Vec<BoardConfig>,
}

fn default_poll_hz() -> u32 { 40 }
fn default_true() -> bool { true }

#[derive(Debug, Deserialize, Clone)]
struct BoardConfig {
    /// Path to this board's USB CDC serial device.
    serial: String,
    /// Friendly label used in log lines. Defaults to the serial path.
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    axes: Vec<AxisMap>,
    #[serde(default)]
    buttons: Vec<ButtonMap>,
}

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
// Per-board serial handle: the label used in logs and the channel to the
// writer thread.
// --------------------------------------------------------------------------

#[derive(Clone)]
struct BoardLink {
    label: String,
    tx: Sender<String>,
}

// --------------------------------------------------------------------------
// Serial setup: open port, version-handshake the firmware, then spawn one
// writer thread and one drainer thread per board.
//
// The drainer is critical: the firmware echoes each character and emits
// "ok jog X N" + a prompt for every command. With ~40 commands/sec on a
// single axis we just barely keep up with the ttyACM read buffer; at
// ~80/sec (two axes simultaneously) the buffer fills, the kernel stops
// ACKing the device's IN endpoint, and the firmware blocks inside
// `write_packet`. That blocks the firmware's CLI task entirely, so new
// commands stop being processed until the host is restarted (which
// reopens the port and clears the buffer). Reading and discarding bytes
// at any rate prevents that backpressure.
// --------------------------------------------------------------------------

const HOST_VERSION: &str = env!("CARGO_PKG_VERSION");

fn open_port(path: &str) -> Result<Box<dyn serialport::SerialPort>> {
    serialport::new(path, 115_200)
        .timeout(Duration::from_millis(200))
        .open()
        .with_context(|| format!("opening serial port {path}"))
}

/// Send `version` to the firmware, wait briefly, and return the parsed
/// `X.Y.Z` it printed. Falls back to the connection banner if the
/// firmware predates the `version` command.
fn handshake_version(port: &mut Box<dyn serialport::SerialPort>) -> Result<String> {
    let _ = port.clear(serialport::ClearBuffer::Input);
    // Stray newline first so we're not stuck inside a partial line, then
    // the actual query. The firmware will echo both lines back.
    port.write_all(b"\r\nversion\r\n")
        .context("writing version handshake")?;
    port.flush().ok();

    let mut accum: Vec<u8> = Vec::new();
    let mut buf = [0u8; 256];
    let deadline = Instant::now() + Duration::from_millis(800);
    while Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(n) if n > 0 => {
                accum.extend_from_slice(&buf[..n]);
                if accum.len() > 4096 {
                    break;
                }
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(anyhow!("reading version: {e}")),
        }
    }
    let text = String::from_utf8_lossy(&accum);
    parse_version(&text).ok_or_else(|| {
        anyhow!(
            "firmware did not return a parseable version (got: {:?})",
            text.chars().take(160).collect::<String>()
        )
    })
}

/// Look for the first `vX.Y.Z` token in a buffer.
fn parse_version(s: &str) -> Option<String> {
    for tok in s.split(|c: char| !c.is_ascii_alphanumeric() && c != '.') {
        let Some(rest) = tok.strip_prefix('v') else { continue };
        let parts: Vec<&str> = rest.split('.').collect();
        if parts.len() == 3
            && parts.iter().all(|p| !p.is_empty() && p.bytes().all(|b| b.is_ascii_digit()))
        {
            return Some(rest.to_string());
        }
    }
    None
}

fn spawn_serial_threads(
    mut port: Box<dyn serialport::SerialPort>,
    label: String,
) -> Result<Sender<String>> {
    let mut reader = port.try_clone().context("cloning serial port for drainer")?;
    let (tx, rx) = channel::<String>();

    thread::Builder::new()
        .name(format!("serial-writer:{label}"))
        .spawn(move || {
            for cmd in rx {
                let _ = port.write_all(cmd.as_bytes());
                let _ = port.flush();
            }
        })?;

    thread::Builder::new()
        .name(format!("serial-drainer:{label}"))
        .spawn(move || {
            let mut buf = [0u8; 512];
            loop {
                match reader.read(&mut buf) {
                    // Discard - firmware's CLI is chatty and we don't act on
                    // its output. Just keep the buffer empty.
                    Ok(_) => {}
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(_) => thread::sleep(Duration::from_millis(50)),
                }
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

fn open_joystick(path: &str) -> std::io::Result<File> {
    // Blocking reads. When the device is unplugged the read returns either
    // EOF (Ok(0)) or an error - both let us break out cleanly. With
    // O_NONBLOCK a disconnected device looks identical to an idle one
    // (perpetual EWOULDBLOCK), which is exactly how the original v1
    // missed disconnects.
    std::fs::OpenOptions::new().read(true).open(path)
}

/// Reader supervisor: keeps trying to open the device, runs an inner read
/// loop, and on disconnect zeros the snapshot so the integrator immediately
/// commands `jog 0` instead of latching the stick's last deflected value.
fn reader_supervisor(
    device: String,
    snapshot: Arc<Mutex<Snapshot>>,
    on_button_press: impl Fn(u8),
    connected: Arc<AtomicBool>,
) {
    let mut warned = false;
    loop {
        match open_joystick(&device) {
            Ok(mut fd) => {
                eprintln!("joystick: connected {device}");
                {
                    let mut s = snapshot.lock().unwrap();
                    for v in s.axes.iter_mut() {
                        *v = 0;
                    }
                }
                connected.store(true, Ordering::SeqCst);
                run_reader(&mut fd, &snapshot, &on_button_press);
                connected.store(false, Ordering::SeqCst);
                {
                    let mut s = snapshot.lock().unwrap();
                    for v in s.axes.iter_mut() {
                        *v = 0;
                    }
                }
                eprintln!("joystick: disconnected, will retry");
                warned = false;
                // Brief settle before re-open so we don't pin a CPU when
                // the device file is briefly present-but-busy during the
                // kernel's hotplug dance.
                thread::sleep(Duration::from_millis(300));
            }
            Err(e) => {
                if !warned {
                    eprintln!("joystick: cannot open {device} ({e}); retrying");
                    warned = true;
                }
                thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

/// Inner reader. Returns when the device disconnects (read error or EOF).
fn run_reader(
    fd: &mut File,
    snapshot: &Arc<Mutex<Snapshot>>,
    on_button_press: &impl Fn(u8),
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
            // Short read (n < event size) or EOF: device is gone.
            Ok(_) => return,
            Err(e) => {
                eprintln!("joystick read error: {e}");
                return;
            }
        }
    }
}

// --------------------------------------------------------------------------
// Integrator: at poll_hz, for each board, computes each mapped axis's
// signed velocity and streams jog commands to that board's serial writer.
//
// We *don't* dedup nonzero values - we resend them every tick so the
// firmware's deadman watchdog stays fed. A single dropped packet (or a
// stalled thread on either side) therefore halts motion at the watchdog
// timeout instead of letting the arm coast. Zero is sent once on the
// transition; the firmware doesn't need a heartbeat to stay stopped.
//
// Last-sent state is per-board so axis "x" on board 0 and axis "x" on
// board 1 track independently.
// --------------------------------------------------------------------------

fn integrate_loop(
    cfg: &Config,
    links: &[BoardLink],
    snapshot: Arc<Mutex<Snapshot>>,
    connected: Arc<AtomicBool>,
) {
    let period = Duration::from_secs_f32(1.0 / cfg.poll_hz.clamp(10, 200) as f32);
    let mut last: Vec<HashMap<&'static str, (i32, u32)>> =
        (0..cfg.boards.len()).map(|_| HashMap::new()).collect();
    let mut next_tick = Instant::now();

    loop {
        next_tick += period;
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        } else {
            next_tick = Instant::now() + period;
        }

        let axes_state = if connected.load(Ordering::SeqCst) {
            snapshot.lock().unwrap().axes.clone()
        } else {
            Vec::new()
        };

        for (board_idx, board) in cfg.boards.iter().enumerate() {
            let link = &links[board_idx];
            for map in &board.axes {
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

                let (new_sign, new_mag): (i32, u32) = if signed_v.abs() < 1.0 {
                    (0, 0)
                } else {
                    let s = if signed_v > 0.0 { 1 } else { -1 };
                    let m = ((signed_v.abs() / 25.0).round() as u32).max(1) * 25;
                    (s, m)
                };

                let prev = last[board_idx].get(&target).copied().unwrap_or((0, 0));
                // Idle -> idle: nothing to send.
                if new_mag == 0 && prev.1 == 0 {
                    continue;
                }
                last[board_idx].insert(target, (new_sign, new_mag));
                let signed_hz: i32 = new_sign * new_mag as i32;
                send(link, &format!("jog {target} {signed_hz}\r\n"), cfg.log);
            }
        }
    }
}

fn send(link: &BoardLink, cmd: &str, log: bool) {
    if log {
        let trimmed = cmd.trim_end();
        println!("[{}] > {trimmed}", link.label);
    }
    let _ = link.tx.send(cmd.to_string());
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
    if cfg.boards.is_empty() {
        bail!("config has no [[boards]] entries");
    }
    Ok(cfg)
}

fn board_label(b: &BoardConfig) -> String {
    b.name.clone().unwrap_or_else(|| b.serial.clone())
}

fn main() -> Result<()> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "config.toml".into());
    let cfg = Arc::new(load_config(&path)?);

    // For each board: open the port, do the version handshake, then hand
    // the port off to a writer + drainer pair. Collect the per-board send
    // channels into `links`.
    let mut links: Vec<BoardLink> = Vec::with_capacity(cfg.boards.len());
    for board in &cfg.boards {
        let label = board_label(board);
        let mut port = open_port(&board.serial)
            .with_context(|| format!("board {label}"))?;
        // Give the firmware a moment to enumerate / settle on a fresh
        // connect (the banner takes a few hundred ms to flush after we
        // open).
        thread::sleep(Duration::from_millis(300));

        let fw_version = handshake_version(&mut port)
            .with_context(|| format!("board {label}"))?;
        eprintln!("board {label}: firmware v{fw_version}, host v{HOST_VERSION}");
        if fw_version != HOST_VERSION {
            bail!(
                "board {label}: version mismatch: firmware v{fw_version} != host v{HOST_VERSION}. \
                 Reflash firmware/target/thumbv6m-none-eabi/release/tmc-new-era.uf2 \
                 or rebuild the host."
            );
        }

        let tx = spawn_serial_threads(port, label.clone())?;
        links.push(BoardLink { label, tx });
    }

    if cfg.enable_on_start {
        for link in &links {
            send(link, "enable all\r\n", cfg.log);
        }
    }

    let snapshot = Arc::new(Mutex::new(Snapshot::default()));
    let connected = Arc::new(AtomicBool::new(false));

    // Joystick reader thread - supervises (re)open / read / disconnect.
    // Captures every board's buttons + tx so a single press can fan out to
    // whichever boards are listening for that index.
    let snap_for_reader = snapshot.clone();
    let device = cfg.device.clone();
    let connected_for_reader = connected.clone();
    let log_buttons = cfg.log;
    let boards_for_buttons: Vec<(BoardLink, Vec<ButtonMap>)> = cfg.boards.iter()
        .zip(links.iter().cloned())
        .map(|(b, link)| (link, b.buttons.clone()))
        .collect();
    thread::Builder::new().name("js-reader".into()).spawn(move || {
        reader_supervisor(
            device,
            snap_for_reader,
            move |btn| {
                for (link, buttons) in &boards_for_buttons {
                    for b in buttons.iter().filter(|b| b.index as u8 == btn) {
                        if let Some(target) = axis_token(&b.target) {
                            let cmd = format!("move {target} {} {}\r\n", b.delta, b.hz);
                            send(link, &cmd, log_buttons);
                        }
                    }
                }
            },
            connected_for_reader,
        );
    })?;

    // Integrator thread.
    let cfg_for_int = cfg.clone();
    let links_for_int = links.clone();
    let connected_for_int = connected.clone();
    thread::Builder::new().name("integrator".into()).spawn(move || {
        integrate_loop(&cfg_for_int, &links_for_int, snapshot, connected_for_int);
    })?;

    let total_axes: usize = cfg.boards.iter().map(|b| b.axes.len()).sum();
    let total_buttons: usize = cfg.boards.iter().map(|b| b.buttons.len()).sum();
    eprintln!(
        "tmc-new-era-host: device={} poll_hz={} boards={} axes={} buttons={}",
        cfg.device, cfg.poll_hz, cfg.boards.len(), total_axes, total_buttons
    );
    eprintln!("Ctrl-C to stop.");

    // Catch Ctrl-C so we can broadcast `jog 0` + `disable all` before exit.
    install_sigint();
    loop {
        if STOP.load(std::sync::atomic::Ordering::SeqCst) {
            if cfg.safe_stop_on_exit {
                for link in &links {
                    for a in ["x", "y", "z", "e"] {
                        send(link, &format!("jog {a} 0\r\n"), cfg.log);
                    }
                    send(link, "disable all\r\n", cfg.log);
                }
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

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigint(_: i32) {
    STOP.store(true, Ordering::SeqCst);
}

fn install_sigint() {
    unsafe extern "C" {
        fn signal(signum: i32, handler: usize) -> usize;
    }
    const SIGINT: i32 = 2;
    unsafe { signal(SIGINT, on_sigint as *const () as usize); }
}
