# tmc-new-era

Rust firmware for the **BigTreeTech SKR Pico** (RP2040) plus a Rust host
CLI that lets a USB joystick drive the X/Y/Z/E steppers.

Pin map is taken verbatim from the working Klipper config in
[bigtreetech/SKR-Pico](https://github.com/bigtreetech/SKR-Pico)
(`Klipper/SKR Pico klipper.cfg`).

| Axis | STEP    | DIR     | EN (active low) |
|------|---------|---------|-----------------|
| X    | GP11    | GP10    | GP12            |
| Y    | GP6     | GP5     | GP7             |
| Z    | GP19    | GP28    | GP2             |
| E    | GP14    | GP13    | GP15            |

## Repo layout

```
firmware/         RP2040 firmware (no_std, embassy). Builds to a UF2.
host/             Linux CLI: TOML mapping + /dev/input/jsN + USB CDC serial.
config.example.toml   Copy to config.toml and edit for your joystick.
```

The two crates are independent (no workspace) so each builds for its own
target without fighting `.cargo/config.toml`.

## Build

```sh
rustup target add thumbv6m-none-eabi      # one-time, for the firmware

# Firmware
cd firmware
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/tmc-new-era \
           target/thumbv6m-none-eabi/release/tmc-new-era.uf2

# Host CLI
cd ../host
cargo build --release        # -> ./target/release/tmc-new-era-host
```

(`cargo install elf2uf2-rs` if you don't already have it.)

## Flash the firmware

1. Hold **BOOTSEL** on the SKR Pico and plug it into USB. It mounts as
   `RPI-RP2`.
2. Copy `firmware/target/thumbv6m-none-eabi/release/tmc-new-era.uf2` onto
   it. The board reboots automatically.

After reboot the board enumerates as a USB CDC ACM serial device
(`/dev/ttyACM0` on Linux).

## Talk to it directly

```sh
picocom -b 115200 /dev/ttyACM0
```

Commands:

```
help                              show usage
status                            show enable state and last commanded velocity
enable  <x|y|z|e|all>             drive EN low (motor energized)
disable <x|y|z|e|all>             drive EN high (motor free)
jog     <axis> <signed_hz>        continuous motion. 0 stops. + = DIR high.
move    <axis> <steps> [hz]       one-shot, duration-based (approx N steps)
```

Jog updates take effect mid-pulse - sending a new value while motion is
underway preempts the old one immediately.

## Drive it with a joystick

```sh
cp config.example.toml config.toml
# edit paths and per-axis sensitivity
./host/target/release/tmc-new-era-host config.toml
```

The host:

- opens the TOML config (path defaults to `./config.toml`)
- opens the joystick at `/dev/input/jsN`
- opens the firmware's USB CDC serial
- at 40 Hz (configurable), turns each mapped stick axis into a signed
  steps/sec value, applies a deadzone, buckets to 25 steps/sec, and
  sends `jog x <hz>` etc. only when the value changes
- on each button press fires a one-shot `move`
- on Ctrl-C sends `jog 0` to every axis and `disable all` before exit

`log = true` in the config prints every command it sends - useful for
first bring-up.

## Caveats

The TMC2209 drivers on the SKR Pico are wired for single-wire UART control
(addresses 0, 2, 1, 3 for X/Y/Z/E on GP8/GP9). This firmware does **not**
talk UART - it only pulses STEP/DIR/EN. The drivers therefore run in
*standalone* mode, where MS1/MS2 (which also set the UART address) select
the microstep count:

| Axis | UART addr (MS2,MS1) | Standalone microsteps |
|------|---------------------|-----------------------|
| X    | 0 (0,0)             | 8                     |
| Y    | 2 (1,0)             | 64                    |
| Z    | 1 (0,1)             | 32                    |
| E    | 3 (1,1)             | 16                    |

So the same `steps` count produces different mm of motion per axis. See
[NOTES.md](NOTES.md) for what a UART-config pass would need to do.

Run current and hold current are also at the standalone defaults rather
than the values in the Klipper config. Fine for light testing; check
driver temperatures for sustained motion.
