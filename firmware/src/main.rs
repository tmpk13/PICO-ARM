// Firmware for the BTT SKR Pico (RP2040).
//
// USB CDC ACM serial. Connect with picocom/screen and type:
//
//     help
//     enable all
//     jog x 800            # continuous: positive direction, 800 steps/sec
//     jog x -400           # reverse at 400 steps/sec
//     jog x 0              # stop
//     move x 1600 1000     # one-shot: ~1600 steps at 1000 hz (duration-based)
//     disable all
//
// Each axis runs in its own task fed by a Signal, so jog commands take
// effect mid-pulse without waiting for the previous motion to finish.
// This is also the surface the host-side joystick driver writes to.

#![no_std]
#![no_main]

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
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
}

const AXES: usize = 4;
const AXIS_NAMES: [&str; AXES] = ["X", "Y", "Z", "E"];
const MAX_HZ: u32 = 40_000;

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
}
static SHADOW: [AxisShadow; AXES] = [
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0) },
    AxisShadow { enabled: AtomicBool::new(false), velocity: AtomicI32::new(0) },
];

fn send_cmd(idx: usize, cmd: AxisCmd) {
    match cmd {
        AxisCmd::Enable(on) => SHADOW[idx].enabled.store(on, Ordering::Relaxed),
        AxisCmd::Jog(hz) => SHADOW[idx].velocity.store(hz, Ordering::Relaxed),
    }
    AXIS_CMD[idx].signal(cmd);
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
    let mut velocity: i32 = 0;

    let apply = |cmd: AxisCmd,
                 enabled: &mut bool,
                 velocity: &mut i32,
                 dir: &mut Output<'static>,
                 en: &mut Output<'static>| {
        match cmd {
            AxisCmd::Enable(on) => {
                *enabled = on;
                if on {
                    en.set_low();
                } else {
                    en.set_high();
                    *velocity = 0;
                }
            }
            AxisCmd::Jog(hz) => {
                *velocity = hz;
                if hz != 0 {
                    if !*enabled {
                        en.set_low();
                        *enabled = true;
                    }
                    if hz > 0 {
                        dir.set_high();
                    } else {
                        dir.set_low();
                    }
                }
            }
        }
    };

    loop {
        if velocity == 0 {
            let cmd = sig.wait().await;
            apply(cmd, &mut enabled, &mut velocity, &mut dir, &mut en);
            continue;
        }

        step.set_high();
        Timer::after(Duration::from_micros(2)).await;
        step.set_low();

        let abs_hz = velocity.unsigned_abs() as u64;
        let period_us = (1_000_000u64 / abs_hz.max(1)).max(4);
        let remaining = period_us.saturating_sub(2).max(2);

        match select(Timer::after(Duration::from_micros(remaining)), sig.wait()).await {
            Either::First(_) => {}
            Either::Second(cmd) => {
                apply(cmd, &mut enabled, &mut velocity, &mut dir, &mut en);
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
            writeln(class, "  status").await?;
            writeln(class, "  enable  <x|y|z|e|all>").await?;
            writeln(class, "  disable <x|y|z|e|all>").await?;
            writeln(class, "  jog     <axis> <signed_hz>     0 stops; sign sets DIR").await?;
            writeln(class, "  move    <axis> <steps> [hz]    one-shot, duration-based").await?;
            writeln(class, "max hz 40000.").await?;
        }
        "status" => {
            let mut buf: String<160> = String::new();
            for i in 0..AXES {
                let _ = write!(
                    buf,
                    "  {}: {} vel={}\r\n",
                    AXIS_NAMES[i],
                    if SHADOW[i].enabled.load(Ordering::Relaxed) { "EN" } else { "--" },
                    SHADOW[i].velocity.load(Ordering::Relaxed),
                );
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
        _ => {
            writeln(class, "unknown command - try 'help'").await?;
        }
    }
    Ok(())
}

async fn cli_session(class: &mut Class) -> Result<(), Disconnected> {
    writeln(class, "").await?;
    writeln(class, "tmc-new-era - SKR Pico stepper CLI").await?;
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
        // a coil powered indefinitely.
        for i in 0..AXES {
            send_cmd(i, AxisCmd::Jog(0));
            send_cmd(i, AxisCmd::Enable(false));
        }
    }
}
