BLE remote for [Wahoo Headwind](https://eu.wahoofitness.com/devices/indoor-cycling/accessories/kickr-headwind) using [embassy-rs/nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice).

Cycles modes, speed setpoint in sync with front-panel/App control.

## Requires

* nRF52840 board (using nRF52840dk); can be ported to other nrf5XXXX but this is my target.
* `cargo install probe-rs -F=cli`

## Setup

1. Download SoftDevice S140 from Nordic's website [here](https://www.nordicsemi.com/Software-and-tools/Software/S140/Download). Supported versions are 7.x.x
2. Program SoftDevice with: `probe-rs download --chip nrf52840 --format hex s140_nrf52_7.X.X_softdevice.hex`
3. Run `cargo run`

By default the DEFMT_LOG level is 'info' (set in ./cargo/config). For debugging use `DEFMT_LOG=debug cargo run`.

To flash build: `cargo flash --release --chip nRF52840_xxAA`.

## Usage

The default build will scan for BLE devices and use the first device with advertisement data name (0x09) starting with 'HEADWIND' (default Wahoo Headwind name) - the Headwind service UUID is not in advertisement packet unfortunately.

To use a fixed address, change the `HEADWIND_ADDR` const in './src/main.rs' and use the `--features=fixed_addr`.

Once connected, the buttons control the Headwind mode:

* Button 1: Next mode = Off -> Heartrate -> Manual .. 0..100 step 25 -> Off.
* Button 2: Previous mode = Off <- Heartrate <- Manual .. 0..100 step -25 <- Off.

The current operating mode and speed setpoint are maintained via the notify characteristic so if changed by the front-panel or App, the buttons will cycle from that updated mode.
