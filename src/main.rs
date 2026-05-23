// Bare-bones firmware for the BTT SKR Pico (RP2040).
//
// Boots as a USB CDC ACM device. Connect a terminal (picocom, screen, minicom)
// and type:
//
//     help
//     enable all
//     move x 800 1000     # 800 steps, 1000 steps/sec
//     move y -1600 2000
//     disable all
//
// Pin map mirrors the working Klipper config in
// /tmp/SKR-Pico/Klipper/SKR Pico klipper.cfg.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
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
// Stepper axis abstraction. EN is active-low (the Klipper config marks each
// enable pin as `!gpioNN`). DIR polarity here is arbitrary: positive `steps`
// means DIR=high. If a motor moves the wrong way for your printer, flip the
// sign on the move command or swap the coil wiring.
// --------------------------------------------------------------------------

struct Axis {
    name: &'static str,
    step: Output<'static>,
    dir: Output<'static>,
    en: Output<'static>,
    enabled: bool,
}

impl Axis {
    fn new(name: &'static str, step: Output<'static>, dir: Output<'static>, mut en: Output<'static>) -> Self {
        en.set_high(); // disabled (active-low)
        Self { name, step, dir, en, enabled: false }
    }

    fn enable(&mut self) {
        self.en.set_low();
        self.enabled = true;
    }

    fn disable(&mut self) {
        self.en.set_high();
        self.enabled = false;
    }

    async fn move_steps(&mut self, steps: i32, speed_hz: u32) {
        if steps == 0 {
            return;
        }
        if steps > 0 {
            self.dir.set_high();
        } else {
            self.dir.set_low();
        }
        // TMC2209 needs >=20ns DIR setup before STEP and a >=100ns pulse;
        // we use 2us / 2us for safe margins.
        Timer::after(Duration::from_micros(2)).await;

        let n = steps.unsigned_abs();
        let period_us: u64 = (1_000_000u64 / speed_hz.max(1) as u64).max(4);
        let high_us: u64 = 2;
        let low_us: u64 = period_us.saturating_sub(high_us).max(2);

        for _ in 0..n {
            self.step.set_high();
            Timer::after(Duration::from_micros(high_us)).await;
            self.step.set_low();
            Timer::after(Duration::from_micros(low_us)).await;
        }
    }
}

// --------------------------------------------------------------------------
// USB write helpers
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
// Command parsing + dispatch
// --------------------------------------------------------------------------

fn axis_index(s: &str) -> Option<usize> {
    match s {
        "x" | "X" => Some(0),
        "y" | "Y" => Some(1),
        "z" | "Z" => Some(2),
        "e" | "E" => Some(3),
        _ => None,
    }
}

async fn run_command(
    class: &mut Class,
    axes: &mut [Axis; 4],
    line: &str,
) -> Result<(), Disconnected> {
    let mut it = line.split_ascii_whitespace();
    let Some(cmd) = it.next() else {
        return Ok(());
    };
    match cmd {
        "help" | "?" => {
            writeln(class, "commands:").await?;
            writeln(class, "  help                        show this").await?;
            writeln(class, "  status                      show axis enable state").await?;
            writeln(class, "  enable  <x|y|z|e|all>       drive EN low (motor energized)").await?;
            writeln(class, "  disable <x|y|z|e|all>       drive EN high (motor free)").await?;
            writeln(class, "  move    <axis> <steps> [hz] steps signed, hz default 1000 (max 20000)").await?;
            writeln(class, "pins (from SKR-Pico klipper.cfg):").await?;
            writeln(class, "  X  step=GP11  dir=GP10  en=GP12").await?;
            writeln(class, "  Y  step=GP6   dir=GP5   en=GP7").await?;
            writeln(class, "  Z  step=GP19  dir=GP28  en=GP2").await?;
            writeln(class, "  E  step=GP14  dir=GP13  en=GP15").await?;
        }
        "status" => {
            let mut buf: String<96> = String::new();
            for ax in axes.iter() {
                let _ = write!(buf, "  {}: {}\r\n", ax.name, if ax.enabled { "enabled" } else { "disabled" });
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
                for ax in axes.iter_mut() {
                    if on { ax.enable() } else { ax.disable() }
                }
                writeln(class, if on { "all axes enabled" } else { "all axes disabled" }).await?;
            } else if let Some(i) = axis_index(arg) {
                if on { axes[i].enable() } else { axes[i].disable() }
                let mut buf: String<48> = String::new();
                let _ = write!(buf, "{} {}\r\n", axes[i].name, if on { "enabled" } else { "disabled" });
                write_all(class, buf.as_bytes()).await?;
            } else {
                writeln(class, "unknown axis (expected x|y|z|e|all)").await?;
            }
        }
        "move" => {
            let Some(axis_s) = it.next() else {
                writeln(class, "usage: move <axis> <steps> [hz]").await?;
                return Ok(());
            };
            let Some(steps_s) = it.next() else {
                writeln(class, "usage: move <axis> <steps> [hz]").await?;
                return Ok(());
            };
            let Some(idx) = axis_index(axis_s) else {
                writeln(class, "unknown axis").await?;
                return Ok(());
            };
            let Ok(steps) = steps_s.parse::<i32>() else {
                writeln(class, "bad steps (signed integer expected)").await?;
                return Ok(());
            };
            let speed: u32 = match it.next() {
                Some(s) => match s.parse::<u32>() {
                    Ok(v) if v > 0 => v,
                    _ => {
                        writeln(class, "bad hz (positive integer expected)").await?;
                        return Ok(());
                    }
                },
                None => 1000,
            };
            let speed = speed.min(20_000);

            if !axes[idx].enabled {
                axes[idx].enable();
            }

            let mut buf: String<96> = String::new();
            let _ = write!(buf, "moving {} {} steps @ {} hz\r\n", axes[idx].name, steps, speed);
            write_all(class, buf.as_bytes()).await?;

            axes[idx].move_steps(steps, speed).await;
            writeln(class, "done").await?;
        }
        _ => {
            writeln(class, "unknown command — try 'help'").await?;
        }
    }
    Ok(())
}

async fn cli_session(class: &mut Class, axes: &mut [Axis; 4]) -> Result<(), Disconnected> {
    writeln(class, "").await?;
    writeln(class, "tmc-new-era — SKR Pico stepper CLI").await?;
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
                        run_command(class, axes, line.as_str()).await?;
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
                    // Ctrl-C — abandon line
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

    // Stepper pins, copied verbatim from the SKR-Pico Klipper config.
    // X: step gp11, dir gp10, en gp12
    let x = Axis::new(
        "X",
        Output::new(p.PIN_11, Level::Low),
        Output::new(p.PIN_10, Level::Low),
        Output::new(p.PIN_12, Level::High),
    );
    // Y: step gp6, dir gp5, en gp7
    let y = Axis::new(
        "Y",
        Output::new(p.PIN_6, Level::Low),
        Output::new(p.PIN_5, Level::Low),
        Output::new(p.PIN_7, Level::High),
    );
    // Z: step gp19, dir gp28, en gp2
    let z = Axis::new(
        "Z",
        Output::new(p.PIN_19, Level::Low),
        Output::new(p.PIN_28, Level::Low),
        Output::new(p.PIN_2, Level::High),
    );
    // E: step gp14, dir gp13, en gp15
    let e = Axis::new(
        "E",
        Output::new(p.PIN_14, Level::Low),
        Output::new(p.PIN_13, Level::Low),
        Output::new(p.PIN_15, Level::High),
    );
    let mut axes = [x, y, z, e];

    // USB driver + CDC ACM class.
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
        let _ = cli_session(&mut class, &mut axes).await;
        // Disconnected — safe-stop: disable all motors so a yanked cable
        // doesn't leave a coil energized indefinitely.
        for ax in axes.iter_mut() {
            ax.disable();
        }
    }
}
