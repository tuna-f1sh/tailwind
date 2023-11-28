BLE remote for [Wahoo Headwind](https://eu.wahoofitness.com/devices/indoor-cycling/accessories/kickr-headwind) using [embassy-rs/nrf-softdevice](https://github.com/embassy-rs/nrf-softdevice).

## Requires

* nrf52840 board; can be ported to other nrf5XXXX but this is my target.
* `cargo install probe-rs -F=cli`
* `cargo install probe-run`

## 

1. Download SoftDevice S140 from Nordic's website [here](https://www.nordicsemi.com/Software-and-tools/Software/S140/Download). Supported versions are 7.x.x
2. Program SoftDevice with: `probe-rs download --chip nrf52840 --format hex s140_nrf52_7.X.X_softdevice.hex`
3. Run `cargo run`
