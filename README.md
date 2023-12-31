BLE remote for [Wahoo Headwind](https://eu.wahoofitness.com/devices/indoor-cycling/accessories/kickr-headwind) using [embassy-rs/nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice).

Cycles modes and speed setpoint in sync with front-panel/App control. The system works from my reverse engineering the simple BLE protocol with WireShark. It's not perfect; the protocol state notify is cyclic rather than actual notify so can represent an old state rather than expected setpoint. The remote handles this well enough for this task though.

https://github.com/tuna-f1sh/tailwind/assets/1886746/1fe502b9-5f74-41d0-b3a3-664f1ef39549

## Requires

* nRF52840 board (using nRF52840dk) or Feather nRF52840 (will require JLink); can be ported to other nrf5XXXX but these are my targets.
* NeoKey Wing for final assembly and [case](https://learn.adafruit.com/deco-two-key-keypad-macropad-circuitpython-feather/build-the-deco-keypad): `--features=feather_neokey`.
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

* Button 1: Next mode = Off -> Heartrate -> Manual .. STEP,100,STEP -> Off.
* Button 2: Previous mode = Off <- Heartrate <- Manual .. 100,STEP,-STEP <- Off.

The current operating mode and speed setpoint are maintained via the notify characteristic so if changed by the front-panel or App, the buttons will cycle from that updated mode.
