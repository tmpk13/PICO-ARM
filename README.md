# tmc-new-era

Minimal Rust firmware for the **BigTreeTech SKR Pico** (RP2040). Boots as a
USB CDC ACM serial device. Connect with any terminal program and type
commands to drive the X/Y/Z/E steppers.

Pin map is taken verbatim from the working Klipper config in
[bigtreetech/SKR-Pico](https://github.com/bigtreetech/SKR-Pico)
(`Klipper/SKR Pico klipper.cfg`).

| Axis | STEP    | DIR     | EN (active low) |
|------|---------|---------|-----------------|
| X    | GP11    | GP10    | GP12            |
| Y    | GP6     | GP5     | GP7             |
| Z    | GP19    | GP28    | GP2             |
| E    | GP14    | GP13    | GP15            |

## Build

```sh
rustup target add thumbv6m-none-eabi   # one-time
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/tmc-new-era \
           target/thumbv6m-none-eabi/release/tmc-new-era.uf2
```

(`cargo install elf2uf2-rs` if you don't have it.)

## Flash

1. Hold **BOOTSEL** on the SKR Pico and plug it into USB. It mounts as a
   mass storage device called `RPI-RP2`.
2. Copy `target/thumbv6m-none-eabi/release/tmc-new-era.uf2` onto it. The
   board reboots automatically.

## Use

Once flashed, the SKR Pico re-enumerates as a USB CDC ACM serial port
(`/dev/ttyACM0` on Linux). Open it at any baud rate (CDC ACM ignores it):

```sh
picocom -b 115200 /dev/ttyACM0
```

Commands:

```
help                       show usage
status                     show enable state of each axis
enable  <x|y|z|e|all>      drive EN low (motor energized, holding torque)
disable <x|y|z|e|all>      drive EN high (motor free)
move    <axis> <steps> [hz]  steps signed, hz default 1000, capped at 20000
```

Example session:

```
> enable all
all axes enabled
> move x 800 1000
moving X 800 steps @ 1000 hz
done
> move x -800 1000
moving X -800 steps @ 1000 hz
done
> disable all
all axes disabled
```

## Caveats

The TMC2209 drivers on the SKR Pico are wired for single-wire UART control
(addresses 0, 2, 1, 3 for X/Y/Z/E on GP8/GP9). This firmware does **not**
talk UART -- it only pulses STEP/DIR/EN. The drivers therefore run in
*standalone* mode, where MS1/MS2 (also used to set the UART address) select
the microstep count:

| Axis | UART addr (MS2,MS1) | Standalone microsteps |
|------|---------------------|-----------------------|
| X    | 0 (0,0)             | 8                     |
| Y    | 2 (1,0)             | 64                    |
| Z    | 1 (0,1)             | 32                    |
| E    | 3 (1,1)             | 16                    |

So the same `steps` count produces different mm of motion per axis. Klipper
overrides this with `microsteps: 16` over UART; you'll need to do the same
if you want a uniform mm-per-step. See `NOTES.md` for details.

Run current and hold current are also at the standalone defaults (set by
the on-board sense resistor) rather than the values in the Klipper config.
For light testing this is fine; for sustained motion check that drivers
aren't overheating.
