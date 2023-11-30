#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![macro_use]

use defmt_rtt as _; // global logger
// time driver
use panic_probe as _;

use defmt::{info, debug, *};
use embassy_executor::Spawner;
use embassy_nrf::{self as _, Peripherals};
use embassy_nrf::gpio::{Input, Pin as _, AnyPin, Pull, Output, Level, OutputDrive};
use embassy_nrf::interrupt::Priority;
use nrf_softdevice::ble::{central, gatt_client, Address, AddressType, Connection};
use nrf_softdevice::{raw, Softdevice};
use core::cell::RefCell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use futures::pin_mut;

#[cfg(feature = "fixed_addr")]
const HEADWIND_ADDR: [u8; 6] = [0xb9, 0xb1, 0x4f, 0x9d, 0x10, 0xe3];
const SPEED_STEP: u8 = 25;

#[derive(Debug, defmt::Format)]
#[allow(dead_code)]
enum Error {
    Connection(central::ConnectError),
    Discover(gatt_client::DiscoverError),
    Read(gatt_client::ReadError),
    Write(gatt_client::WriteError),
    TryWrite(gatt_client::TryWriteError),
    UnknownState(u8),
    UnknownCommand(u8),
    UnknownNotification([u8; 4]),
    InvalidPayload([u8; 4]),
    InvalidSpeed(u8),
    Nak,
    Timeout,
}

#[nrf_softdevice::gatt_client(uuid = "a026ee0c-0a7d-4ab3-97fa-f1500f9feb8b")]
struct HeadwindServiceClient {
    #[characteristic(uuid = "a026e038-0a7d-4ab3-97fa-f1500f9feb8b", read, write, notify)]
    // Since the characteristic is used for differnt variables, we'll use a slice of u8
    command: [u8; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
enum HeadwindState {
    Off,
    HeartRate,
    Speed,
    Manual(u8),
    Sleep,
}

impl From<HeadwindState> for u8 {
    fn from(state: HeadwindState) -> Self {
        match state {
            HeadwindState::Off => 0x01,
            HeadwindState::HeartRate => 0x02,
            HeadwindState::Speed => 0x03,
            HeadwindState::Manual(_) => 0x04,
            HeadwindState::Sleep => 0x05,
        }
    }
}

impl HeadwindState {
    fn next(self) -> Self {
        match self {
            // off -> hr -> manual(speed step,100,step) -> hr ...
            HeadwindState::Off => HeadwindState::HeartRate,
            HeadwindState::Speed |
                HeadwindState::HeartRate |
                HeadwindState::Sleep |
                HeadwindState::Manual(0xFF) => HeadwindState::Manual(SPEED_STEP),
            HeadwindState::Manual(speed) => {
                if speed < 100 {
                    HeadwindState::Manual(speed + (SPEED_STEP - speed % SPEED_STEP))
                } else {
                    HeadwindState::HeartRate
                }
            }
        }
    }

    fn previous(self) -> Self {
        match self {
            // off <- hr <- manual(speed, 100,step,-step) <- off ...
            HeadwindState::Off |
                HeadwindState::Sleep |
                HeadwindState::HeartRate |
                HeadwindState::Manual(0xFF)  => HeadwindState::Manual(100),
            // HeadwindState::HeartRate => HeadwindState::Off,
            HeadwindState::Speed => HeadwindState::HeartRate,
            HeadwindState::Manual(speed) => {
                if speed < SPEED_STEP {
                    HeadwindState::HeartRate
                } else {
                    HeadwindState::Manual(speed - SPEED_STEP)
                }
            }
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
                warn!("Should not pass HeadwindState from confirm command {:#02x}, does not contain speed setpoint!", res);
                let cmd = HeadwindCommand::try_from(res)?;
                match cmd {
                    HeadwindCommand::SetState(state) => Ok(state),
                    HeadwindCommand::SetSpeed(speed) => Ok(HeadwindState::Manual(speed)),
                }
            },
            // 0xfd seems to be general state of the device
            // Annoyingly it's not really a notify but cyclic and the notified state can be outdated
            // and cause potentially a race condition if notify old state between button presses but this is good enough...
            // Alternatively would be not using notify at all and read state before each action
            // Best would be only using notify for ACK/read before action but that doesn't seem to be possible waiting on same characteristic Portal
            0xfd => {
                let mode = res[3];

                match mode {
                    0x01 => Ok(HeadwindState::Off),
                    0x02 => Ok(HeadwindState::HeartRate),
                    0x03 => Ok(HeadwindState::Speed),
                    0x04 => Ok(HeadwindState::Manual(res[2])),
                    0x05 => Ok(HeadwindState::Sleep),
                    _ => Err(Error::UnknownState(mode))
                }
            }
            _ => {
                warn!("Unknown notification: {:#02x}", res);
                Err(Error::UnknownNotification(*res))
            }
        }
    }
}

impl From<HeadwindState> for [u8; 4] {
    fn from(state: HeadwindState) -> Self {
        [0x04, u8::from(state), 0x00, 0x00]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, defmt::Format)]
enum HeadwindCommand {
    SetState(HeadwindState),
    SetSpeed(u8),
}

impl From<HeadwindCommand> for u8 {
    fn from(cmd: HeadwindCommand) -> Self {
        match cmd {
            HeadwindCommand::SetSpeed(_) => 0x02,
            HeadwindCommand::SetState(_) => 0x04,
        }
    }
}

impl TryFrom<&[u8; 4]> for HeadwindCommand {
    type Error = Error;

    fn try_from(res: &[u8; 4]) -> Result<Self, Self::Error> {
        let cmd_type = res[0];

        match cmd_type {
            // command confirmation
            0xfe => {
                let cmd = res[1];
                match cmd {
                    // set speed
                    0x02 => Ok(HeadwindCommand::SetSpeed(res[3])),
                    // set state
                    0x04 => {
                        match res[3] {
                            0x01 => Ok(HeadwindCommand::SetState(HeadwindState::Off)),
                            0x02 => Ok(HeadwindCommand::SetState(HeadwindState::HeartRate)),
                            0x03 => Ok(HeadwindCommand::SetState(HeadwindState::Speed)),
                            // State change confirm doesn't contain speed so put an invalid flag - could be Option u8 instead but...
                            0x04 => Ok(HeadwindCommand::SetState(HeadwindState::Manual(0xFF))),
                            0x05 => Ok(HeadwindCommand::SetState(HeadwindState::Sleep)),
                            _ => Err(Error::UnknownState(res[3]))
                        }
                    }
                    // have seen 0x03, 0x05, 0x06 with App but not need for our use - probably device info
                    _ => Err(Error::UnknownCommand(cmd))
                }
            },
            // state update is not a command confirmation
            0xfd => Err(Error::InvalidPayload(*res)),
            _ => {
                warn!("Unknown command: {:08x}", res);
                Err(Error::UnknownNotification(*res))
            }
        }
    }
}

impl From<HeadwindCommand> for [u8; 4] {
    fn from(cmd: HeadwindCommand) -> Self {
        match cmd {
            HeadwindCommand::SetState(state) => state.into(),
            HeadwindCommand::SetSpeed(speed) => [u8::from(cmd), speed, 0x00, 0x00],
        }
    }
}

struct Headwind {
    conn: Connection,
    client: HeadwindServiceClient,
    state: Mutex<ThreadModeRawMutex, RefCell<HeadwindState>>,
    command_signal: Signal<ThreadModeRawMutex, HeadwindCommand>,
}

impl Headwind {
    /// Set the local [`HeadwindState`]
    fn set_state(&self, new_state: HeadwindState) -> () {
        self.state.lock(|state| {
            let old_state = *state.borrow();
            match (old_state, new_state) {
                // HACK: If we're in manual mode and the new state is manual with invalid speed, keep the old speed
                (HeadwindState::Manual(old), HeadwindState::Manual(0xff)) => {
                    let new_state = HeadwindState::Manual(old);
                    *state.borrow_mut() = new_state;
                },
                _ => {
                    *state.borrow_mut() = new_state;
                }
            }
            info!("Headwind set state: {:?}", new_state);
        });
    }

    /// Update the local [`HeadwindState`] from read/notification command bytes
    fn update_state(&self, bytes: &[u8; 4]) -> () {
        match HeadwindState::try_from(bytes) {
            Ok(new_state) => self.set_state(new_state),
            Err(e) => warn!("Failed to parse command: {:?}", e)
        }
    }

    /// Read the current [`HeadwindState`] from Headwind and update the local state
    async fn read_state(&self) -> Result<HeadwindState, Error> {
        let val = self.client.command_read().await.map_err(|e| Error::Read(e))?;
        self.update_state(&val);
        Ok(self.state.lock(|state| *state.borrow()))
    }

    /// Request the Headwind enters the [`HeadwindState`] by writing to the command characteristic
    ///
    /// Note that it does not wait for a response, but the response will be received as a notification
    async fn request_state(&self, state: HeadwindState) -> Result<(), Error> {
        info!("Request state: {:?}", state);
        self.send_command(HeadwindCommand::SetState(state)).await?;
        // Set local since it was ACK'd
        self.set_state(state);
        Ok(())
    }

    /// Request the Manual mode `speed` by writing to the command
    /// characteristic
    ///
    /// Note that the Headwind will not enter Manual mode with the SetSpeed so
    /// should be in that state before request
    async fn request_speed(&self, speed: u8) -> Result<(), Error> {
        if speed <= 100 {
            info!("Request speed: {:?}", speed);
            self.send_command(HeadwindCommand::SetSpeed(speed)).await
        } else {
            Err(Error::InvalidSpeed(speed))
        }
    }

    /// Send a [`HeadwindCommand`] to the Headwind by writing to the command characteristic
    /// 
    /// Waits for a response ACK from the Headwind and returns an error if the response is not the same
    async fn send_command(&self, cmd: HeadwindCommand) -> Result<(), Error> {
        let buf: [u8; 4] = cmd.into();
        debug!("Sending command: {:?}/{:#02x}", cmd, buf);
        self.client.command_write_without_response(&buf).await.map_err(|e| Error::Write(e))?;

        // wait for ACK - Headwind doesn't seem to like set speed if not in manual mode
        match futures::future::select(embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)), self.command_signal.wait()).await {
            futures::future::Either::Left(_) => {
                error!("Command timeout: {:?}", cmd);
                Err(Error::Timeout)
            }
            futures::future::Either::Right((ack, _)) => {
                match (cmd, ack) {
                    // HACK: ack for state will not contain speed
                    (HeadwindCommand::SetState(HeadwindState::Manual(_)), HeadwindCommand::SetState(HeadwindState::Manual(_))) => {
                        Ok(())
                    },
                    _ => {
                        if ack == cmd {
                            Ok(())
                        } else {
                            error!("Command NAK: {:?} != {:?}", ack, cmd);
                            Err(Error::Nak)
                        }
                    }
                }
            }
        }
    }

    async fn change_state(&self, new_state: HeadwindState) -> Result<(), Error> {
        let current = self.state.lock(|state| *state.borrow());

        // For entering Manual mode with cycle, set the speed too
        match (current, new_state) {
            (HeadwindState::Manual(_), HeadwindState::Manual(speed)) => {
                self.request_speed(speed).await?;
                // Set the state since request speed will not but we know are in manual mode
                self.set_state(new_state);
                Ok(())
            }
            (_, HeadwindState::Manual(speed)) => {
                self.request_state(new_state).await?;
                self.request_speed(speed).await
            },
            (_, _) => {
                self.request_state(new_state).await
            }
        }
    }

    /// Request the Headwind enters the next [`HeadwindState`] by writing to the command characteristic
    async fn next_state(&self) -> Result<(), Error> {
        self.change_state(self.state.lock(|state| state.borrow().next())).await
    }

    /// Request the Headwind enters the previous [`HeadwindState`] by writing to the command characteristic
    async fn previous_state(&self) -> Result<(), Error> {
        self.change_state(self.state.lock(|state| state.borrow().previous())).await
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
                    let ret = Headwind { conn, client, state: Mutex::new(RefCell::new(HeadwindState::Off)), command_signal: Signal::new() };
                    // Read state updates internal state
                    let state = ret.read_state().await?;
                    info!("Headwind created in state: {:?}", state);
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
    info!("Attmpting connection to Headwind with fixed address {:02x}", HEADWIND_ADDR);
    try_connect_headwind(sd, &HEADWIND_ADDR).await
}

#[cfg(not(feature = "fixed_addr"))]
async fn find_headwind(sd: &'static Softdevice) -> Result<Headwind, Error> {
    info!("Scanning for devices with name starting: 'HEADWIND'");
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

async fn button_task<'a>(up_btn: &'a mut Input<'_, AnyPin>, down_btn: &'a mut Input<'_, AnyPin>, headwind: &'a Headwind) {
    loop {
        let up_fut = async { up_btn.wait_for_falling_edge().await };
        let down_fut = async { down_btn.wait_for_falling_edge().await };
        pin_mut!(up_fut, down_fut);

        match futures::future::select(up_fut, down_fut).await {
            futures::future::Either::Left(_) => {
                debug!("Up pressed");
                if let Err(e) = headwind.next_state().await {
                    error!("Failed to set next state: {:?}", e);
                }
            }
            futures::future::Either::Right(_) => {
                debug!("Down pressed");
                if let Err(e) = headwind.previous_state().await {
                    error!("Failed to set previous state: {:?}", e);
                }
            }
        }
    }
}

async fn blink_led(led: &mut Output<'_, AnyPin>, on_period: u64, off_period: Option<u64>) {
    loop {
        led.set_low();
        embassy_time::Timer::after(embassy_time::Duration::from_millis(on_period)).await;
        led.set_high();
        // If off_period keep looping forever, otherwise break
        if let Some(off_period) = off_period {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(off_period)).await;
        } else {
            break;
        }
    }
}

static LED_SIGNAL: Signal<ThreadModeRawMutex, u8> = Signal::new();

#[embassy_executor::task]
async fn led_task(rpin: AnyPin, gpin: AnyPin, bpin: AnyPin) {
    let rled = Output::new(rpin, Level::High, OutputDrive::Standard);
    let gled = Output::new(gpin, Level::High, OutputDrive::Standard);
    let bled = Output::new(bpin, Level::High, OutputDrive::Standard);
    pin_mut!(rled, gled, bled);

    loop {
        let blink_fut = blink_led(&mut bled, 500, Some(500));
        pin_mut!(blink_fut);

        // TODO LED commands
        match futures::future::select(LED_SIGNAL.wait(), blink_fut).await {
            futures::future::Either::Left((val, _)) => {
                match val {
                    1 => {
                        rled.set_low();
                    },
                    2 => {
                        gled.set_low();
                    },
                    _ => {
                        rled.set_high();
                        gled.set_high();
                    },
                }
            }
            _ => ()
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Tailwind Remote");

    // Initialize peripherals; GPIOs and SoftDevice
    let p = init_peripherals();
    // nrf52540dk buttons - also p0.24 and p0.25
    let mut down_btn = Input::new(p.P0_11.degrade(), Pull::Up); // 11 on Feather/Button 1 on DK
    let mut up_btn = Input::new(p.P0_12.degrade(), Pull::Up); // SCK on Feather/Button 2 on DK

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
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 0, // don't need advertising
            periph_role_count: 0, // don't need peripheral
            central_role_count: 1, // one central for connecting to headwind
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        ..Default::default()
    };

    let sd = Softdevice::enable(&config);
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // LED task
    let rpin = p.P0_13.degrade(); // MOSI on Feather/LED1 on DK
    let gpin = p.P0_14.degrade(); // MISO on Feather/LED2 on DK
    let bpin = p.P0_15.degrade(); // 15 on Feather/LED3 on DK
    unwrap!(spawner.spawn(led_task(rpin, gpin, bpin)));

    loop {
        match find_headwind(sd).await {
            Ok(headwind) => {
                // Enable command notifications from the peripheral
                headwind.client.command_cccd_write(true).await.unwrap();

                LED_SIGNAL.signal(1);

                // Start button future
                let button_fut = button_task(&mut up_btn, &mut down_btn, &headwind);

                // Receive notifications
                let gatt_fut = gatt_client::run(&headwind.conn, &headwind.client, |event| match event {
                    HeadwindServiceClientEvent::CommandNotification(val) => {
                        debug!("notification: {:#02x}", val);
                        // Only update state if it's a state update (0xfd)
                        if val[0] == 0xfd {
                            headwind.update_state(&val);
                        // Else signal the command ACK
                        } else {
                            match HeadwindCommand::try_from(&val) {
                                Ok(cmd) => {
                                    debug!("Confirmed command: {:?}", cmd);
                                    headwind.command_signal.signal(cmd);
                                }
                                Err(e) => error!("Failed to parse command: {:?}", e)
                            }
                        }
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

                LED_SIGNAL.signal(0);

                // Disconnected
                // Since we're in a loop, we'll just try to reconnect
                info!("Disconnected");
            }
            Err(e) => {
                error!("Failed to connect to Headwind: {:?}", e);
            }
        }
    }
}
