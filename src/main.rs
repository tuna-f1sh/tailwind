#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![macro_use]

use defmt_rtt as _; // global logger
use embassy_nrf as _; // time driver
use panic_probe as _;

use core::mem;

use defmt::{info, *};
use embassy_executor::Spawner;
use nrf_softdevice::ble::{central, gatt_client, Address, AddressType};
use nrf_softdevice::{raw, Softdevice};

const HEADWIND_ADDR: [u8; 6] = [0xb9, 0xb1, 0x4f, 0x9d, 0x10, 0xe3];

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

#[nrf_softdevice::gatt_client(uuid = "a026ee0c-0a7d-4ab3-97fa-f1500f9feb8b")]
struct HeadwindServiceClient {
    #[characteristic(uuid = "a026e038-0a7d-4ab3-97fa-f1500f9feb8b", read, write, notify)]
    command: [u8; 4],
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Tailwind Remote");

    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 6,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 128 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t { attr_tab_size: 32768 }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"Tailwind" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    unwrap!(spawner.spawn(softdevice_task(sd)));

    let addrs = &[&Address::new(
        AddressType::RandomStatic,
        HEADWIND_ADDR,
    )];

    let mut config = central::ConnectConfig::default();
    config.scan_config.whitelist = Some(addrs);
    // TODO scan and test each for HardwareServiceClient
    let conn = unwrap!(central::connect(sd, &config).await);
    info!("connected");

    let client: HeadwindServiceClient = unwrap!(gatt_client::discover(&conn).await);

    // Read
    // let val = unwrap!(client.command_read().await);
    // info!("read command: {}", val);

    // // Write, set it to 42
    // unwrap!(client.command_write(&42).await);
    // info!("Wrote command!");

    // // Read to check it's changed
    // let val = unwrap!(client.command_read().await);
    // info!("read command: {}", val);

    // Enable command notifications from the peripheral
    client.command_cccd_write(true).await.unwrap();

    // Receive notifications
    gatt_client::run(&conn, &client, |event| match event {
        HeadwindServiceClientEvent::CommandNotification(val) => {
            let cmd_type = val[0];

            match cmd_type {
                // 0xfe seems to be confirm command
                0xfe => {
                    let cmd = val[1];
                    let speed = val[3];
                    info!("set: {:02x} {}", cmd, speed);
                },
                // 0xfd seems to be general state of the device
                0xfd => {
                    let mode = val[3];
                    let speed = val[2];
                    info!("state notification: {:02x} {}", mode, speed);
                }
                _ => {
                    info!("unknown notification: {:08x}", val);
                }
            }
        }
    })
    .await;
}
