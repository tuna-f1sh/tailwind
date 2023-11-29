#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![macro_use]

use defmt_rtt as _; // global logger
use embassy_nrf::{self as _, Peripherals};
// time driver
use panic_probe as _;

use core::mem;

use defmt::{info, debug, *};
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Input, Pin as _, AnyPin, Pull, Output, Level, OutputDrive};
use embassy_nrf::interrupt::Priority;
use nrf_softdevice::ble::{central, gatt_client, Address, AddressType, Connection};
use nrf_softdevice::{raw, Softdevice};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use futures::pin_mut;

#[cfg(feature = "fixed_addr")]
const HEADWIND_ADDR: [u8; 6] = [0xb9, 0xb1, 0x4f, 0x9d, 0x10, 0xe3];

#[derive(Debug, defmt::Format)]
enum Error {
    Connection(central::ConnectError),
    Discover(gatt_client::DiscoverError),
    Read(gatt_client::ReadError),
    Write(gatt_client::TryWriteError),
    UnknownState(u8),
    UnknownCommand(u8),
    UnknownNotification([u8; 4]),
}

#[nrf_softdevice::gatt_client(uuid = "a026ee0c-0a7d-4ab3-97fa-f1500f9feb8b")]
struct HeadwindServiceClient {
    #[characteristic(uuid = "a026e038-0a7d-4ab3-97fa-f1500f9feb8b", read, write, notify)]
    // Since the characteristic is used for differnt variables, we'll use a slice of u8
    command: [u8; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
#[repr(u8)]
enum HeadwindState {
    Off = 0x01,
    HeartRate = 0x02,
    Speed = 0x03,
    Manual(u8) = 0x04,
    Sleep = 0x05,
}

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
#[repr(u8)]
enum HeadwindCommand {
    SetState(HeadwindState) = 0x04,
    SetSpeed(u8) = 0x02,
}

impl HeadwindState {
    fn next(self) -> Self {
        match self {
            // off -> hr -> manual .. speed -> off ...
            HeadwindState::Off => HeadwindState::HeartRate,
            // HeadwindState::HeartRate => HeadwindState::Speed,
            HeadwindState::Speed | HeadwindState::HeartRate => HeadwindState::Manual(0xFF),
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
            // off <- hr <- manual .. speed <- off ...
            // HeadwindState::Off => HeadwindState::Sleep,
            HeadwindState::Off => HeadwindState::Manual(0xFF),
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
            HeadwindState::Sleep => HeadwindState::Manual(0xFF),
        }
    }
}

impl TryFrom<&[u8; 4]> for HeadwindState {
    type Error = Error;

    fn try_from(res: &[u8; 4]) -> Result<Self, Self::Error> {
        let cmd_type = res[0];

        match cmd_type {
            // 0xfe seems to be confirm command
            0xfe => {
                let cmd = res[1];

                match cmd {
                    0x02 => { 
                        Ok(HeadwindState::Manual(res[3]))
                    },
                    0x04 => {
                        match res[3] {
                            0x01 => Ok(HeadwindState::Off),
                            0x02 => Ok(HeadwindState::HeartRate),
                            0x03 => Ok(HeadwindState::Speed),
                            0x04 => Ok(HeadwindState::Manual(res[2])),
                            0x05 => Ok(HeadwindState::Sleep),
                            _ => Err(Error::UnknownState(res[3]))
                        }
                    }
                    _ => Err(Error::UnknownCommand(cmd))
                }
            },
            // 0xfd seems to be general state of the device
            0xfd => {
                let mode = res[3];
                let speed = res[2];

                match mode {
                    0x01 => Ok(HeadwindState::Off),
                    0x02 => Ok(HeadwindState::HeartRate),
                    0x03 => Ok(HeadwindState::Speed),
                    0x04 => Ok(HeadwindState::Manual(speed)),
                    0x05 => Ok(HeadwindState::Sleep),
                    _ => Err(Error::UnknownState(mode))
                }
            }
            _ => {
                warn!("Unknown notification: {:08x}", res);
                Err(Error::UnknownNotification(*res))
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
            HeadwindState::Manual(speed) => {
                // 0xff is internal flag to enter manual mode; will use last setpoint
                if speed == 0xFF {
                    [0x04, 0x04, 0x00, 0x00]
                } else {
                    [0x02, speed, 0x00, 0x00]
                }
            }
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
    fn set_state(&self, new_state: HeadwindState) -> () {
        self.state.lock(|state| {
            if *state.borrow() != new_state {
                *state.borrow_mut() = new_state;
                info!("Headwind state updated: {:?}", new_state);
            }
        });
    }

    /// Update the local state from read/notification command bytes
    fn update_state(&self, bytes: &[u8; 4]) -> () {
        match HeadwindState::try_from(bytes) {
            Ok(new_state) => self.set_state(new_state),
            Err(e) => warn!("Failed to parse command: {:?}", e)
        }
    }

    /// Read the current state from Headwind and update the local state
    async fn read_state(&self) -> Result<HeadwindState, Error> {
        let val = self.client.command_read().await.map_err(|e| Error::Read(e))?;
        self.update_state(&val);
        Ok(self.state.lock(|state| *state.borrow()))
    }

    /// Request the current state from Headwind by writing to the command characteristic
    async fn request_state(&self) -> Result<(), Error> {
        let buf: [u8; 4] = self.state.lock(|state| {
            let s = *state.borrow();
            info!("Request state: {:?}", s);
            s.into()
        });
        // Has to be try_write_without_response because gatt_client::run is attached to SoftDevice Portal
        self.client.command_try_write_without_response(&buf).map_err(|e| Error::Write(e))
    }

    async fn next_state(&self) -> Result<(), Error> {
        self.state.lock(|state| {
            let next_state = state.borrow().next();
            *state.borrow_mut() = next_state;
        });
        self.request_state().await
    }

    async fn previous_state(&self) -> Result<(), Error> {
        self.state.lock(|state| {
            let prev_state = state.borrow().previous();
            *state.borrow_mut() = prev_state;
        });
        self.request_state().await
    }
}

#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    sd.run().await
}

async fn try_connect_headwind(sd: &'static Softdevice, addr: &[u8; 6]) -> Result<Headwind, Error> {
    let addrs = &[&Address::new(
        AddressType::RandomStatic,
        *addr
    )];

    let mut config = central::ConnectConfig::default();
    config.scan_config.whitelist = Some(addrs);

    match central::connect(sd, &config).await {
        Ok(conn) => {
            info!("Connected to {:#02x}", addr);

            match gatt_client::discover(&conn).await {
                Ok(client) => {
                    let ret = Headwind { conn, client, state: Mutex::new(RefCell::new(HeadwindState::Off)) };
                    ret.read_state().await?;
                    info!("Headwind created in state: {:?}", ret.state.lock(|state| *state.borrow()));
                    Ok(ret)
                }
                Err(e) => {
                    error!("Discover Headwind failed: {:?}", e);
                    Err(Error::Discover(e))
                }
            }
        }
        Err(e) => {
            error!("Connect to Headwind failed: {:?}", e);
            Err(Error::Connection(e))
        }
    }
}

#[cfg(feature = "fixed_addr")]
async fn find_headwind(sd: &'static Softdevice) -> Result<Headwind, Error> {
    try_connect_headwind(sd, &HEADWIND_ADDR).await
}

#[cfg(not(feature = "fixed_addr"))]
async fn find_headwind(sd: &'static Softdevice) -> Result<Headwind, Error> {
    let config = central::ScanConfig::default();

    let addr: [u8; 6] = unwrap!(central::scan(sd, &config, |params| unsafe {
        if params.data.len < 10 {
            return None;
        }

        // Headwind doesn't advertise service UUID, so we have to look for local name starting with 'HEADWIND'
        let mut data = core::slice::from_raw_parts(params.data.p_data, params.data.len as usize);
        debug!("Found {:#02x} with ad data:", params.peer_addr.addr);

        while data.len() != 0 {
            let len = data[0] as usize;
            if data.len() < len + 1 {
                warn!("Advertisement data truncated?");
                break;
            }
            if len < 1 {
                warn!("Advertisement data malformed?");
                break;
            }
            let key = data[1];
            let value = &data[2..len + 1];
            debug!("{:#02x}: {:#02x}", key, value);
            data = &data[len + 1..];

            // If it's name and it's HEADWIND, we found it
            if key == 0x09 && value.len() >= 8 {
                if value[..8] == [0x48, 0x45, 0x41, 0x44, 0x57, 0x49, 0x4e, 0x44] {
                    info!("Found Headwind!");
                    return Some(params.peer_addr.addr)
                }
            }
        }

        None
    }).await);

    try_connect_headwind(sd, &addr).await
}

fn init_peripherals() -> Peripherals {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    embassy_nrf::init(config)
}

fn init_buttons() -> (Input<'static, AnyPin>, Input<'static, AnyPin>) {
    let p = init_peripherals();
    // nrf52540dk buttons - also p0.24 and p0.25
    let button1 = Input::new(p.P0_11.degrade(), Pull::Up);
    let button2 = Input::new(p.P0_12.degrade(), Pull::Up);

    (button1, button2)
}

fn init_leds() -> (Output<'static, AnyPin>, Output<'static, AnyPin>) {
    let p = init_peripherals();
    (Output::new(p.P0_13.degrade(), Level::High, OutputDrive::Standard),
    Output::new(p.P0_14.degrade(), Level::High, OutputDrive::Standard))
}

async fn button_task<'a>(up_btn: &'a mut Input<'_, AnyPin>, down_btn: &'a mut Input<'_, AnyPin>, headwind: &'a Headwind) {
    loop {
        let up_fut = async { up_btn.wait_for_falling_edge().await };
        let down_fut = async { down_btn.wait_for_falling_edge().await };
        pin_mut!(up_fut, down_fut);

        match futures::future::select(up_fut, down_fut).await {
            futures::future::Either::Left(_) => {
                debug!("Up pressed");
                unwrap!(headwind.previous_state().await);
            }
            futures::future::Either::Right(_) => {
                debug!("Down pressed");
                unwrap!(headwind.next_state().await);
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Tailwind Remote");

    let (mut up_btn, mut down_btn) = init_buttons();
    // let (mut led1, mut led2) = init_leds();
    // pin_mut!(up_btn, down_btn);

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
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 23 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t { attr_tab_size: 32768 }),
        // gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
        //     adv_set_count: 1,
        //     periph_role_count: 3,
        //     central_role_count: 3,
        //     central_sec_count: 0,
        //     _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        // }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    unwrap!(spawner.spawn(softdevice_task(sd)));

    loop {
        match find_headwind(sd).await {
            Ok(headwind) => {
                // Enable command notifications from the peripheral
                headwind.client.command_cccd_write(true).await.unwrap();

                // Start button future
                let button_fut = button_task(&mut up_btn, &mut down_btn, &headwind);

                // Receive notifications
                let gatt_fut = gatt_client::run(&headwind.conn, &headwind.client, |event| match event {
                    HeadwindServiceClientEvent::CommandNotification(val) => {
                        debug!("notification: {:#02x}", val);
                        headwind.update_state(&val);
                    }
                });

                pin_mut!(button_fut, gatt_fut);

                // Wait for either future to exit 
                // - if the button future exits something went wrong
                // - if the gatt future exits the connection was lost
                let _ = match futures::future::select(button_fut, gatt_fut).await {
                    futures::future::Either::Left((_, _)) => {
                        error!("Buttons encountered an error and stopped!")
                    }
                    futures::future::Either::Right((e, _)) => {
                        debug!("gatt_server run exited with: {:?}", e);
                    }
                };

                // Disconnected
                // Since we're in a loop, we'll just try to reconnect
                info!("Disconnected");
            }
            Err(e) => {
                error!("Failed to connect to headwind: {:?}", e);
            }
        }
    }
}
