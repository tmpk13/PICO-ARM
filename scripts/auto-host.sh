#!/usr/bin/env bash
# Decoupled host-launcher for the RPi 1.2 (or any Linux host).
#
# Polls every POLL_SECS seconds. When a joystick (/dev/input/jsN) or any
# tmc-new-era SKR Pico USB CDC device appears, it spawns the host binary
# and tracks the child. When the binary exits (clean or crash), the
# watcher returns to polling -- so unplugging hardware mid-run and
# plugging it back in transparently restarts the session.
#
# Knobs (all overridable via env):
#   TMC_HOST_BIN     path to tmc-new-era-host binary
#   TMC_HOST_CONFIG  path to config.toml passed to the binary
#   POLL_SECS        seconds between trigger probes (default 3)
#   RESTART_SECS     cool-down after the binary exits (default 2)
#   REQUIRE_BOTH     "1" -> require joystick AND at least one pico
#                    "0" (default) -> trigger on either, matching the
#                    "either the joystick or the picos" spec.
#   PICO_GLOB        glob that matches a Pico CDC device. The default
#                    matches the by-id strings the firmware advertises.
#   JOY_GLOB         glob that matches a joystick device.
#
# This script is intentionally standalone -- no part of the Rust code
# imports or knows about it. Run it directly, or under systemd via
# tmc-new-era-host.service.

set -u

: "${TMC_HOST_BIN:=/home/tmpk/tmc-new-era/host/target/release/tmc-new-era-host}"
: "${TMC_HOST_CONFIG:=/home/tmpk/tmc-new-era/config.toml}"
: "${POLL_SECS:=3}"
: "${RESTART_SECS:=2}"
: "${REQUIRE_BOTH:=0}"
: "${PICO_GLOB:=/dev/serial/by-id/usb-tmc-new-era_*}"
: "${JOY_GLOB:=/dev/input/js*}"

log() { printf '[auto-host %s] %s\n' "$(date +%H:%M:%S)" "$*"; }

# Returns 0 when any path matching $1 exists.
glob_present() {
    # nullglob behavior via compgen: -G expands the pattern and lists
    # nothing when there is no match, so we just check for any output.
    compgen -G "$1" > /dev/null
}

should_start() {
    local have_joy have_pico
    glob_present "$JOY_GLOB" && have_joy=1 || have_joy=0
    glob_present "$PICO_GLOB" && have_pico=1 || have_pico=0
    if [ "$REQUIRE_BOTH" = "1" ]; then
        [ "$have_joy" = "1" ] && [ "$have_pico" = "1" ]
    else
        [ "$have_joy" = "1" ] || [ "$have_pico" = "1" ]
    fi
}

child_pid=0
cleanup() {
    if [ "$child_pid" -ne 0 ] && kill -0 "$child_pid" 2>/dev/null; then
        log "stopping host (pid $child_pid)"
        kill -INT "$child_pid" 2>/dev/null || true
        # Give it a moment to honor safe_stop_on_exit, then force-kill.
        for _ in 1 2 3 4 5; do
            kill -0 "$child_pid" 2>/dev/null || break
            sleep 1
        done
        kill -KILL "$child_pid" 2>/dev/null || true
    fi
    exit 0
}
trap cleanup INT TERM

if [ ! -x "$TMC_HOST_BIN" ]; then
    log "binary not found or not executable: $TMC_HOST_BIN"
    log "build it first: (cd host && cargo build --release)"
    exit 1
fi

log "watching joystick='$JOY_GLOB' pico='$PICO_GLOB' (require_both=$REQUIRE_BOTH)"

while true; do
    until should_start; do
        sleep "$POLL_SECS"
    done

    log "trigger detected, launching $TMC_HOST_BIN"
    # Run the binary in its own process group so Ctrl-C inside the
    # binary stays scoped to it.
    setsid "$TMC_HOST_BIN" "$TMC_HOST_CONFIG" &
    child_pid=$!

    wait "$child_pid"
    status=$?
    child_pid=0
    log "host exited with status $status, cooling down ${RESTART_SECS}s"
    sleep "$RESTART_SECS"
done
