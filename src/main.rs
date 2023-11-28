#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![macro_use]

use defmt_rtt as _; // global logger
use embassy_nrf as _;
// time driver
use panic_probe as _;

use core::mem;

use defmt::{info, debug, *};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Pin as _, AnyPin, Pull};
use nrf_softdevice::ble::{central, gatt_client, Address, AddressType, Connection};
use nrf_softdevice::{raw, Softdevice};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use futures::pin_mut;

const HEADWIND_ADDR: [u8; 6] = [0xb9, 0xb1, 0x4f, 0x9d, 0x10, 0xe3];

#[nrf_softdevice::gatt_client(uuid = "a026ee0c-0a7d-4ab3-97fa-f1500f9feb8b")]
struct HeadwindServiceClient {
    #[characteristic(uuid = "a026e038-0a7d-4ab3-97fa-f1500f9feb8b", read, write, notify)]
    command: [u8; 4],
}

#[derive(Clone, Copy, Debug, defmt::Format)]
#[repr(u8)]
enum HeadwindState {
    Off = 0x01,
    HeartRate = 0x02,
    Speed = 0x03,
    Manual(u8) = 0x04,
    Sleep = 0x05,
}

impl HeadwindState {
    fn to_bytes(self) -> [u8; 4] {
        self.into()
    }

    fn next(self) -> Self {
        match self {
            HeadwindState::Off => HeadwindState::HeartRate,
            // HeadwindState::HeartRate => HeadwindState::Speed,
            HeadwindState::Speed | HeadwindState::HeartRate => HeadwindState::Manual(0),
            HeadwindState::Manual(speed) => {
                if speed + 20 <= 100 {
                    HeadwindState::Manual(speed + 20)
                } else if speed < 100 {
                    HeadwindState::Manual(100)
                } else {
                    HeadwindState::Off
                }
            }
            HeadwindState::Sleep => HeadwindState::Manual(20),
        }
    }

    fn previous(self) -> Self {
        match self {
            // HeadwindState::Off => HeadwindState::Sleep,
            HeadwindState::Off => HeadwindState::Manual(100),
            HeadwindState::HeartRate => HeadwindState::Off,
            HeadwindState::Speed => HeadwindState::HeartRate,
            HeadwindState::Manual(speed) => {
                if speed == 0 {
                    HeadwindState::HeartRate
                } else if speed < 20 {
                    HeadwindState::Manual(0)
                } else {
                    HeadwindState::Manual(speed - 20)
                }
            }
            HeadwindState::Sleep => HeadwindState::Manual(100),
        }
    }
}

impl TryFrom<&[u8; 4]> for HeadwindState {
    type Error = &'static str;

    fn try_from(res: &[u8; 4]) -> Result<Self, Self::Error> {
        let cmd_type = res[0];

        match cmd_type {
            // 0xfe seems to be confirm command
            0xfe => {
                let cmd = res[1];

                match cmd {
                    0x01 => Ok(HeadwindState::Off),
                    0x02 => Ok(HeadwindState::HeartRate),
                    0x03 => Ok(HeadwindState::Speed),
                    0x04 => { 
                        info!("set: {:02x} {} %", cmd, res[3]);
                        Ok(HeadwindState::Manual(res[3]))
                    }
                    0x05 => Ok(HeadwindState::Sleep),
                    _ => Err("Unknown state")
                }
            },
            // 0xfd seems to be general state of the device
            0xfd => {
                let mode = res[3];
                let speed = res[2];
                info!("state notification: {:02x} {} %", mode, speed);

                match mode {
                    0x01 => Ok(HeadwindState::Off),
                    0x02 => Ok(HeadwindState::HeartRate),
                    0x03 => Ok(HeadwindState::Speed),
                    0x04 => Ok(HeadwindState::Manual(speed)),
                    0x05 => Ok(HeadwindState::Sleep),
                    _ => Err("Unknown state")
                }
            }
            _ => {
                info!("unknown notification: {:08x}", res);
                Err("Unknown state")
            }
        }
    }
}

impl From<HeadwindState> for [u8; 4] {
    fn from(state: HeadwindState) -> Self {
        match state {
            HeadwindState::Off => [0x04, 0x01, 0x00, 0x00],
            HeadwindState::HeartRate => [0x04, 0x02, 0x00, 0x00],
            HeadwindState::Speed => [0x04, 0x03, 0x00, 0x00],
            HeadwindState::Manual(speed) => [0x02, speed, 0x00, 0x00],
            HeadwindState::Sleep => [0x04, 0x05, 0x00, 0x00],
        }
    }
}

struct Headwind {
    conn: Connection,
    client: HeadwindServiceClient,
    state: Mutex<ThreadModeRawMutex, RefCell<HeadwindState>>,
}

impl Headwind {
    fn update_state(&self, bytes: &[u8; 4]) -> () {
        if let Some(new_state) = HeadwindState::try_from(bytes).ok() {
            self.state.lock(|state| {
                *state.borrow_mut() = new_state;
            });
        }
    }

    async fn read_state(&self) -> Result<HeadwindState, gatt_client::ReadError> {
        let val = unwrap!(self.client.command_read().await);
        self.update_state(&val);
        Ok(self.state.lock(|state| *state.borrow()))
    }

    // async fn write_state(&self) -> Result<(), gatt_client::WriteError> {
    //     let buf: [u8; 4] = self.state.lock(|state| state.borrow().to_bytes());
    //     self.client.command_write(&buf).await
    // }

    async fn cycle_state_up(&self) -> Result<(), gatt_client::WriteError> {
        let buf: [u8; 4] = self.state.lock(|state| state.borrow().next().to_bytes());
        self.client.command_write(&buf).await
    }

    async fn cycle_state_down(&self) -> Result<(), gatt_client::WriteError> {
        let buf: [u8; 4] = self.state.lock(|state| state.borrow().previous().to_bytes());
        self.client.command_write(&buf).await
    }
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

async fn try_connect_headwind(sd: &'static Softdevice, addr: &[u8; 6]) -> Result<Headwind, gatt_client::DiscoverError> {
    let addrs = &[&Address::new(
        AddressType::RandomStatic,
        *addr
    )];

    let mut config = central::ConnectConfig::default();
    config.scan_config.whitelist = Some(addrs);

    let conn = unwrap!(central::connect(sd, &config).await);
    info!("Connected to {:x}", addr);

    match gatt_client::discover(&conn).await {
        Ok(client) => {
            info!("Discovered");
            let ret = Headwind { conn, client, state: Mutex::new(RefCell::new(HeadwindState::Off)) };
            unwrap!(ret.read_state().await);
            Ok(ret)
        }
        Err(e) => {
            info!("Discover failed: {:?}", e);
            Err(e)
        }
    }

    // gatt_client::discover(&conn).await.map(|client| Headwind { conn, client, state: Mutex::new(RefCell::new(HeadwindState::Off)) })
}

async fn find_headwind(sd: &'static Softdevice) -> Headwind {
    // let addrs = &[&Address::new(
    //     AddressType::RandomStatic,
    //     HEADWIND_ADDR,
    // )];

    let config = central::ScanConfig::default();

    let addr: [u8; 6] = unwrap!(central::scan(sd, &config, |params| {
        if !params.type_.connectable() == 0 {
            return None;
        }

        if params.peer_addr.addr == HEADWIND_ADDR {
            info!("Found headwind!");
            Some(params.peer_addr.addr)
        } else {
            info!("Not headwind: {:x}", params.peer_addr.addr);
            None
        }
    }).await);

    unwrap!(try_connect_headwind(sd, &addr).await)
}

fn init_buttons() -> (Input<'static, AnyPin>, Input<'static, AnyPin>) {
    let config = embassy_nrf::config::Config::default();
    let p = embassy_nrf::init(config);

    let button1 = Input::new(p.P0_11.degrade(), Pull::Up);
    let button2 = Input::new(p.P0_12.degrade(), Pull::Up);

    (button1, button2)
}

async fn button_task<'a>(up_btn: &'a mut Input<'_, AnyPin>, down_btn: &'a mut Input<'_, AnyPin>, headwind: &'a Headwind) {
    loop {
        let btn1_fut = async { up_btn.wait_for_low().await };
        let btn2_fut = async { down_btn.wait_for_low().await };
        pin_mut!(btn1_fut, btn2_fut);

        match futures::future::select(btn1_fut, btn2_fut).await {
            futures::future::Either::Left(_) => {
                info!("Button 1 pressed");
                unwrap!(headwind.cycle_state_up().await);
            }
            futures::future::Either::Right(_) => {
                info!("Button 2 pressed");
                unwrap!(headwind.cycle_state_down().await);
            }
        }
    }
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

    let (mut up_btn, mut down_btn) = init_buttons();

    loop {
        let headwind = find_headwind(sd).await;

        // Enable command notifications from the peripheral
        headwind.client.command_cccd_write(true).await.unwrap();

        // Start button future
        let button_fut = button_task(&mut up_btn, &mut down_btn, &headwind);

        // Receive notifications
        let gatt_fut = gatt_client::run(&headwind.conn, &headwind.client, |event| match event {
            HeadwindServiceClientEvent::CommandNotification(val) => {
                trace!("notification: {}", val);
                headwind.state.lock(|state| {
                    *state.borrow_mut() = HeadwindState::try_from(&val).unwrap_or(HeadwindState::Off);
                });
            }
        });

        pin_mut!(button_fut, gatt_fut);

        // Wait for either future to exit 
        // - if the button future exits something went wrong
        // - if the gatt future exits the connection was lost
        let _ = match futures::future::select(button_fut, gatt_fut).await {
            futures::future::Either::Left((_, _)) => {
                info!("ADC encountered an error and stopped!")
            }
            futures::future::Either::Right((e, _)) => {
                info!("gatt_server run exited with error: {:?}", e);
            }
        };

        // Disconnected
        // Since we're in a loop, we'll just try to reconnect
        info!("Disconnected");
        headwind.state.lock(|state| {
            *state.borrow_mut() = HeadwindState::Off;
        });
    }
}
