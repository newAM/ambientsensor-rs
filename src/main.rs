#![no_std]
#![no_main]
#![allow(clippy::type_complexity)]

mod mqtt;

use dhcp::{Dhcp, DhcpState, MsgType, DHCP_DESTINATION, DHCP_SOURCE_PORT};
use mqtt::{Connack, ConnectCode, CtrlPacket, Publish, CONNACK_LEN};

use bme280::{Address, Bme280, Sample};
use core::fmt::Write;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use embedded_hal::digital::v2::OutputPin;
use rtic::time::duration::Extensions;
use stm32f0xx_hal::{
    gpio::{
        self,
        gpioa::{PA4, PA5, PA6, PA7},
        gpiob::{PB6, PB7},
        AF0, AF1,
    },
    i2c::I2c,
    pac::{EXTI, I2C1, SPI1},
    prelude::*,
    serial::Serial,
    spi::{self, Spi},
    stm32f0::stm32f0x0::Peripherals,
};
use systick_monotonic::Systick;
use w5500_hl::ll::{
    blocking::vdm::W5500,
    net::{Eui48Addr, Ipv4Addr, SocketAddrV4},
    spi::MODE as W5500_MODE,
    LinkStatus, OperationMode, PhyCfg, Registers, Sn, SocketInterrupt, SocketInterruptMask,
    SOCKETS,
};
use w5500_hl::{Tcp, Udp};

const DHCP_SOCKET: Sn = Sn::Sn0;
const MQTT_SOCKET: Sn = Sn::Sn1;

const MQTT_SERVER: SocketAddrV4 = SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 4), 1883);
const HOSTNAME: &str = "ambient1";
const TEMPERATURE_TOPIC: &str = "/home/ambient1/temperature";
const HUMIDITY_TOPIC: &str = "/home/ambient1/humidity";
const PRESSURE_TOPIC: &str = "/home/ambient1/pressure";

const BME280_SETTINGS: bme280::Settings = bme280::Settings {
    config: bme280::Config::reset()
        .set_standby_time(bme280::Standby::Millis125)
        .set_filter(bme280::Filter::X8),
    ctrl_meas: bme280::CtrlMeas::reset()
        .set_osrs_t(bme280::Oversampling::X8)
        .set_osrs_p(bme280::Oversampling::X8)
        .set_mode(bme280::Mode::Normal),
    ctrl_hum: bme280::Oversampling::X8,
};

// This is very inefficient logging, but that is OK because this is only used
// for assertions in release mode (which should never occur).
pub fn log_byte(byte: u8) {
    let dp: Peripherals = unsafe { Peripherals::steal() };
    while dp.USART1.isr.read().txe().bit_is_clear() {}
    dp.USART1.tdr.write(|w| w.tdr().bits(byte as u16));
}

struct Logger;

impl ::core::fmt::Write for Logger {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes().iter().for_each(|x| log_byte(*x));
        Ok(())
    }
}

macro_rules! log {
    ($msg:expr) => {
        #[cfg(not(feature = "release"))]
        {
            log_always!($msg);
        }
    };
    ($msg:expr, $($arg:tt)*) => {
        #[cfg(not(feature = "release"))]
        {
            log_always!($msg, $($arg)*);
        }
    };
}

macro_rules! log_always {
    ($msg:expr) => {
        writeln!(&mut Logger, $msg).ok();
    };
    ($msg:expr, $($arg:tt)*) => {
        writeln!(&mut Logger, $msg, $($arg)*).ok();
    };
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    loop {
        log_always!("{}", info);
        compiler_fence(SeqCst);
    }
}

#[cortex_m_rt::exception]
#[allow(non_snake_case)]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::interrupt::disable();

    loop {
        log_always!("HARD FAULT {:#?}", ef);
        compiler_fence(SeqCst);
    }
}

/// Handles timeouts in the DHCP state machiene.
pub struct DhcpTimeout {
    /// State that we should transition from.
    state: DhcpState,
    /// Number of seconds left util timeout occurs.
    secs: u32,
}

impl DhcpTimeout {
    /// Create a new timeout structure.
    pub fn new(state: DhcpState) -> DhcpTimeout {
        DhcpTimeout { state, secs: 3 }
    }

    pub fn decrement(&mut self) {
        self.secs = self.secs.saturating_sub(1)
    }

    /// Returns `true` if the timeout occured.
    pub fn timeout_occured(&self) -> bool {
        self.secs == 0
    }
}

/// State handling for the MQTT client task.
///
/// This is internal code, and has nothing to do with the MQTT protocol.
///
/// The client simply publishes sensor samples to various topics.
/// Nothing fancy here.
#[derive(Debug)]
pub enum MqttState {
    /// Initial state.
    Init,
    /// Socket `CON` interrupt raised, TCP socket is established.
    ConInt,
    /// Socket `RECV` interrupt recieved, MQTT CONACK
    RecvInt,
    /// The MQTT server has accepted our connection and we can publish.
    Happy,
}

pub struct NopDelay;

impl Default for NopDelay {
    fn default() -> NopDelay {
        NopDelay
    }
}

impl embedded_hal::blocking::delay::DelayMs<u8> for NopDelay {
    fn delay_ms(&mut self, ms: u8) {
        nop_delay_ms(ms.into())
    }
}

/// Worlds worst delay function.
#[inline(always)]
pub fn nop_delay_ms(ms: usize) {
    for _ in 0..(727 * ms) {
        cortex_m::asm::nop();
    }
}

#[rtic::app(
    device = stm32f0xx_hal::stm32,
    dispatchers = [I2C2, USART1, USART2],
)]
mod app {
    use super::*;

    // RTIC manual says not to use this in production.
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1>; // 1 Hz / 1 s granularity

    #[shared]
    struct Shared {
        w5500: W5500<
            Spi<
                SPI1,
                PA5<gpio::Alternate<AF0>>,
                PA6<gpio::Alternate<AF0>>,
                PA7<gpio::Alternate<AF0>>,
                spi::EightBit,
            >,
            PA4<gpio::Output<gpio::PushPull>>,
        >,
        mac: Eui48Addr,
        dhcp_timeout: Option<DhcpTimeout>,
        dhcp_state: DhcpState,
        /// Timer for renewal.
        dhcp_t1: Option<u32>,
        /// Timer for rebinding.
        dhcp_t2: Option<u32>,
        /// Timer for the lease.
        dhcp_lease: Option<u32>,
        yiaddr: Ipv4Addr,
        mqtt_state: MqttState,
        xid: u32,
    }

    #[local]
    struct Local {
        exti: EXTI,
        bme280:
            Bme280<I2c<I2C1, PB6<gpio::Alternate<AF1>>, PB7<gpio::Alternate<AF1>>>, bme280::Init>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp = cx.device;
        let mut rcc = {
            let rcc = dp.RCC;
            rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());
            rcc.configure().sysclk(8.mhz()).freeze(&mut dp.FLASH)
        };
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let syscfg = dp.SYSCFG;
        let exti: EXTI = dp.EXTI;

        let systick = cx.core.SYST;
        let mono = Systick::new(systick, 8_000_000);

        // hardware declaration
        let ((eepom_cs, w5500_cs, mut w5500_rst), spi1_pins, spi2_pins, i2c1_pins, uart_pins) =
            cortex_m::interrupt::free(move |cs| {
                gpiob.pb0.into_pull_down_input(cs); // INT
                (
                    (
                        gpiob.pb12.into_push_pull_output(cs), // EEPROM CS
                        gpioa.pa4.into_push_pull_output(cs),  // W5500 CS
                        gpioa.pa3.into_push_pull_output(cs),  // W5500 RST
                    ),
                    (
                        gpioa.pa5.into_alternate_af0(cs), // W5500 SCK
                        gpioa.pa6.into_alternate_af0(cs), // W5500 MISO
                        gpioa.pa7.into_alternate_af0(cs), // W5500 MOSI
                    ),
                    (
                        gpiob.pb13.into_alternate_af0(cs), // EEPROM SCK
                        gpiob.pb14.into_alternate_af0(cs), // EEPROM MISO
                        gpiob.pb15.into_alternate_af0(cs), // EEPROM MOSI
                    ),
                    (
                        gpiob.pb6.into_alternate_af1(cs), // I2C SCL
                        gpiob.pb7.into_alternate_af1(cs), // I2C SDA
                    ),
                    (
                        gpioa.pa9.into_alternate_af1(cs),  // UART TX
                        gpioa.pa10.into_alternate_af1(cs), // UART RX
                    ),
                )
            });
        let spi1 = Spi::spi1(dp.SPI1, spi1_pins, W5500_MODE, 1.mhz(), &mut rcc);
        let spi2 = Spi::spi2(dp.SPI2, spi2_pins, W5500_MODE, 1.mhz(), &mut rcc);
        let i2c = I2c::i2c1(dp.I2C1, i2c1_pins, 100.khz(), &mut rcc);
        let mut serial = Serial::usart1(dp.USART1, uart_pins, 115_200.bps(), &mut rcc);
        let mut eeprom = eeprom25aa02e48::Eeprom25aa02e48::new(spi2, eepom_cs);
        let mut bme280 = Bme280::new(i2c, Address::SdoGnd);
        let mut w5500 = W5500::new(spi1, w5500_cs);

        writeln!(&mut serial, "Hello world!").ok();
        log!("Hello world!");

        // get the MAC address from the EEPROM
        // the Microchip 25aa02e48 EEPROM comes pre-programmed with a valid MAC
        // TODO: investigate why this fails
        const BAD_MAC: Eui48Addr = Eui48Addr::new(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
        let mac: Eui48Addr = loop {
            let mac: Eui48Addr = eeprom.read_eui48().unwrap().into();
            if mac == Eui48Addr::UNSPECIFIED || mac == BAD_MAC {
                log!("Failed to read the MAC address: {}", mac);
                nop_delay_ms(10);
            } else {
                break mac;
            }
        };
        debug_assert_eq!(mac.octets[0], 0x54);
        log!("MAC: {}", mac);

        w5500_hl::ll::reset(&mut w5500_rst, &mut NopDelay::default()).unwrap();

        // sanity check temperature sensor
        assert_eq!(bme280.chip_id().unwrap(), bme280::CHIP_ID);
        let bme280 = bme280.init(&BME280_SETTINGS).unwrap();

        // enable external interrupt for pb0 (W5500 interrupt)
        syscfg.exticr1.modify(|_, w| w.exti0().pb0());
        debug_assert!(syscfg.exticr1.read().exti0().is_pb0());

        // set interrupt request mask for line 1
        exti.imr.modify(|_, w| w.mr0().set_bit());

        // set interrupt falling trigger for line 1
        exti.rtsr.modify(|_, w| w.tr0().clear_bit());
        exti.ftsr.modify(|_, w| w.tr0().set_bit());

        // continually initialize the W5500 until we link up
        // since we are using power over Ethernet we know that if the device
        // has power it also has an Ethernet cable connected.
        let _phy_cfg: PhyCfg = 'outer: loop {
            // sanity check W5500 communications
            assert_eq!(w5500.version().unwrap(), w5500_hl::ll::VERSION);

            // load the MAC address we got from EEPROM
            w5500.set_shar(&mac).unwrap();
            debug_assert_eq!(w5500.shar().unwrap(), mac);

            // wait for the PHY to indicate the Ethernet link is up
            let mut attempts: usize = 0;
            log!("Polling for link up");
            const PHY_CFG: PhyCfg = PhyCfg::DEFAULT.set_opmdc(OperationMode::FullDuplex10bt);
            w5500.set_phycfgr(PHY_CFG).unwrap();

            const LINK_UP_POLL_PERIOD_MILLIS: usize = 100;
            const LINK_UP_POLL_ATTEMPTS: usize = 50;
            loop {
                let phy_cfg: PhyCfg = w5500.phycfgr().unwrap();
                if phy_cfg.lnk() == LinkStatus::Up {
                    break 'outer phy_cfg;
                }
                if attempts >= LINK_UP_POLL_ATTEMPTS {
                    log!(
                        "Failed to link up in {} ms",
                        attempts * LINK_UP_POLL_PERIOD_MILLIS,
                    );
                    break;
                }
                nop_delay_ms(LINK_UP_POLL_PERIOD_MILLIS);
                attempts += 1;
            }

            w5500_rst.set_low().unwrap();
            nop_delay_ms(1);
            w5500_rst.set_high().unwrap();
            nop_delay_ms(3);
        };
        log!("Done link up\n{}", _phy_cfg);

        // enable DHCP & MQTT socket interrupts
        w5500
            .set_simr(DHCP_SOCKET.bitmask() | MQTT_SOCKET.bitmask())
            .unwrap();

        // start the DHCP task
        dhcp_fsm::spawn().unwrap();

        // start the 1s ticker
        second_ticker::spawn().unwrap();

        (
            Shared {
                w5500,
                dhcp_timeout: None,
                dhcp_state: DhcpState::Init,
                dhcp_t1: None,
                dhcp_t2: None,
                dhcp_lease: None,
                mac,
                yiaddr: Ipv4Addr::UNSPECIFIED,
                mqtt_state: MqttState::Init,
                // TODO: this is a terrible way to get a random XID
                xid: cortex_m::peripheral::SYST::get_current() << 24
                    | cortex_m::peripheral::SYST::get_current(),
            },
            Local { bme280, exti },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        log!("[TASK] idle");
        loop {
            compiler_fence(SeqCst);
        }
    }

    /// This is the W5500 interrupt.
    ///
    /// The only interrupts we should get are for the DHCP & MQTT sockets.
    #[task(binds = EXTI0_1, local = [exti], shared = [w5500, mqtt_state])]
    fn exti0(cx: exti0::Context) {
        log!("[TASK] exti0");

        (cx.shared.w5500, cx.shared.mqtt_state).lock(|w5500, mqtt_state| {
            let mut sir: u8 = w5500.sir().unwrap();
            // may occur when there are power supply issues
            if sir == 0 {
                log_always!("[ERROR] spurious interrupt");
            }

            for sn in SOCKETS.iter() {
                let mask: u8 = 1 << (*sn as u8);
                if sir & mask != 0 {
                    let sn_ir: SocketInterrupt = w5500.sn_ir(*sn).unwrap();
                    w5500.set_sn_ir(*sn, sn_ir).unwrap();
                    match *sn {
                        DHCP_SOCKET => {
                            if sn_ir.con_raised() {
                                log!("[DHCP] INTERRUPT CON");
                            }
                            if sn_ir.discon_raised() {
                                log!("[DHCP] INTERRUPT DISCON");
                            }
                            if sn_ir.recv_raised() {
                                log!("[DHCP] INTERRUPT RECV");
                                dhcp_fsm::spawn().unwrap()
                            }
                            if sn_ir.timeout_raised() {
                                log!("[DHCP] INTERRUPT TIMEOUT");
                            }
                            if sn_ir.sendok_raised() {
                                log!("[DHCP] INTERRUPT SENDOK");
                            }
                        }
                        MQTT_SOCKET => {
                            if sn_ir.discon_raised() | sn_ir.timeout_raised() {
                                log!("[MQTT] INTERRUPT DISCON/TIMEOUT");
                                *mqtt_state = MqttState::Init;
                                if mqtt_client::spawn().is_err() {
                                    log!("[MQTT] already spawned");
                                }
                            } else if sn_ir.con_raised() {
                                log!("[MQTT] INTERRUPT CON");
                                *mqtt_state = MqttState::ConInt;
                                if mqtt_client::spawn().is_err() {
                                    log!("[MQTT] already spawned");
                                }
                            } else if sn_ir.recv_raised() {
                                log!("[MQTT] INTERRUPT RECV");
                                *mqtt_state = MqttState::RecvInt;
                                if mqtt_client::spawn().is_err() {
                                    log!("[MQTT] already spawned");
                                }
                            }
                        }
                        _ => {
                            // may occur when there are power supply issues
                            log_always!("[ERROR] unhandled socket {:?}", *sn);
                        }
                    }
                    sir &= !mask;
                }
                if sir == 0 {
                    break;
                }
            }

            cx.local.exti.pr.write(|w| w.pr0().set_bit());
        });
    }

    /// This handles long timers:
    ///
    /// * DHCP FSM timeouts
    /// * DHCP T1 (renewal)
    /// * DHCP T2 (rebinding)
    /// * DHCP Lease
    #[task(shared = [dhcp_state, dhcp_timeout, dhcp_t1, dhcp_t2, dhcp_lease])]
    fn second_ticker(cx: second_ticker::Context) {
        (
            cx.shared.dhcp_state,
            cx.shared.dhcp_timeout,
            cx.shared.dhcp_t1,
            cx.shared.dhcp_t2,
            cx.shared.dhcp_lease,
        )
            .lock(|dhcp_state, dhcp_timeout, dhcp_t1, dhcp_t2, dhcp_lease| {
                let mut spawn_dhcp_fsm: bool = false;

                if let Some(timeout) = dhcp_timeout {
                    if timeout.timeout_occured() {
                        log!("[DHCP] timeout");
                        *dhcp_state = DhcpState::Init;
                        *dhcp_timeout = None;
                        spawn_dhcp_fsm = true;
                    } else if timeout.state != *dhcp_state {
                        log!("[DHCP] clearing timeout");
                        *dhcp_timeout = None;
                    } else {
                        timeout.decrement();
                    }
                }

                let mut decrement_dhcp_timer = |t: &mut Option<u32>, init: bool| {
                    if let Some(remaining) = t {
                        *remaining = remaining.saturating_sub(1);
                        if *remaining == 0 {
                            *t = None;
                            spawn_dhcp_fsm = true;
                            if init {
                                *dhcp_state = DhcpState::Init;
                            }
                        }
                    }
                };

                decrement_dhcp_timer(dhcp_t1, false);
                decrement_dhcp_timer(dhcp_t2, false);
                decrement_dhcp_timer(dhcp_lease, true);

                if spawn_dhcp_fsm {
                    dhcp_fsm::spawn().unwrap();
                }

                second_ticker::spawn_after(1.seconds()).unwrap();
            });
    }

    #[task(
        shared = [
            w5500,
            dhcp_state,
            dhcp_timeout,
            mac,
            yiaddr,
            mqtt_state,
            xid,
            dhcp_t1,
            dhcp_t2,
            dhcp_lease
        ]
    )]
    fn dhcp_fsm(cx: dhcp_fsm::Context) {
        log!("[TASK] dhcp_fsm");

        (
            cx.shared.w5500,
            cx.shared.dhcp_state,
            cx.shared.dhcp_timeout,
            cx.shared.dhcp_t1,
            cx.shared.dhcp_t2,
            cx.shared.dhcp_lease,
            cx.shared.mac,
            cx.shared.yiaddr,
            cx.shared.mqtt_state,
            cx.shared.xid,
        )
            .lock(
                |w5500,
                 dhcp_state,
                 dhcp_timeout,
                 dhcp_t1,
                 dhcp_t2,
                 dhcp_lease,
                 mac,
                 yiaddr,
                 mqtt_state,
                 xid| {
                    let mut dhcp_buf = unsafe { Dhcp::steal() };

                    let mut dhcp_recv = || -> bool {
                        let (num_bytes, _) =
                            w5500.udp_recv_from(DHCP_SOCKET, dhcp_buf.recv()).unwrap();
                        log!("Read {} bytes", num_bytes);
                        assert!(
                            num_bytes < dhcp_buf.len(),
                            "Buffer was too small to receive all data"
                        );
                        // TODO: handle buffer underrun

                        #[allow(clippy::if_same_then_else, clippy::needless_bool)]
                        if !dhcp_buf.is_bootreply() {
                            log!("not a bootreply");
                            true
                        } else if dhcp_buf.xid() != *xid {
                            log!(
                                "xid does not match: 0x{:08X} != 0x{:08X}",
                                dhcp_buf.xid(),
                                xid
                            );
                            true
                        } else {
                            false
                        }
                    };

                    log!("[DHCP] {:?}", dhcp_state);
                    match dhcp_state {
                        DhcpState::Init => {
                            w5500.udp_bind(DHCP_SOCKET, DHCP_SOURCE_PORT).unwrap();

                            let discover: &[u8] = dhcp_buf.dhcp_discover(mac, HOSTNAME, xid);
                            log!("[DHCP] SENDING DISCOVER");
                            let tx_bytes: usize = w5500
                                .udp_send_to(DHCP_SOCKET, discover, &DHCP_DESTINATION)
                                .unwrap();
                            assert_eq!(tx_bytes, discover.len());

                            *dhcp_state = DhcpState::Selecting;

                            // start timeout tracker
                            *dhcp_timeout = Some(DhcpTimeout::new(*dhcp_state));
                        }
                        DhcpState::Selecting => {
                            if dhcp_recv() {
                                return;
                            }

                            *yiaddr = dhcp_buf.yiaddr();
                            log!("yiaddr: {}", yiaddr);

                            let request: &[u8] = dhcp_buf.dhcp_request(mac, yiaddr, HOSTNAME, xid);
                            log!("[DHCP] SENDING REQUEST");
                            let tx_bytes: usize = w5500.udp_send(DHCP_SOCKET, request).unwrap();
                            assert_eq!(tx_bytes, request.len());

                            *dhcp_state = DhcpState::Requesting;

                            // start timeout tracker
                            *dhcp_timeout = Some(DhcpTimeout::new(*dhcp_state));
                        }
                        DhcpState::Requesting | DhcpState::Renewing | DhcpState::Rebinding => {
                            if dhcp_recv() {
                                return;
                            }

                            match dhcp_buf.message_type().unwrap() {
                                MsgType::Ack => {
                                    let subnet_mask: Ipv4Addr = dhcp_buf.subnet_mask().unwrap();
                                    let gateway: Ipv4Addr = dhcp_buf.dhcp_server().unwrap();
                                    let renewal_time: u32 = dhcp_buf.renewal_time().unwrap();
                                    let rebinding_time: u32 = dhcp_buf.rebinding_time().unwrap();
                                    let lease_time: u32 = dhcp_buf.lease_time().unwrap();
                                    *dhcp_t1 = Some(renewal_time);
                                    *dhcp_t2 = Some(rebinding_time);
                                    *dhcp_lease = Some(lease_time);
                                    log!("Subnet Mask:    {}", subnet_mask);
                                    log!("Client IP:      {}", *yiaddr);
                                    log!("Gateway IP:     {}", gateway);
                                    log!("Renewal Time:   {}s", renewal_time);
                                    log!("Rebinding Time: {}s", rebinding_time);
                                    log!("Lease Time:     {}s", lease_time);
                                    debug_assert_eq!(dhcp_buf.yiaddr(), *yiaddr);
                                    w5500.set_subr(&subnet_mask).unwrap();
                                    w5500.set_sipr(yiaddr).unwrap();
                                    w5500.set_gar(&gateway).unwrap();
                                    if matches!(dhcp_state, DhcpState::Requesting) {
                                        *mqtt_state = MqttState::Init;
                                        if mqtt_client::spawn().is_err() {
                                            log!("[DHCP] MQTT already spawned");
                                        }
                                    }
                                    *dhcp_state = DhcpState::Bound;
                                }
                                MsgType::Nak => {
                                    log!("NAK");
                                    *dhcp_state = DhcpState::Init;
                                    dhcp_fsm::spawn().unwrap();
                                }
                                _x => {
                                    log!("Ignoring message: {:?}", _x);
                                }
                            }
                        }
                        DhcpState::Bound => {
                            let request: &[u8] = dhcp_buf.dhcp_request(mac, yiaddr, HOSTNAME, xid);
                            log!("[DHCP] SENDING REQUEST");
                            let tx_bytes: usize = w5500.udp_send(DHCP_SOCKET, request).unwrap();
                            assert_eq!(tx_bytes, request.len());

                            if dhcp_t2.is_none() {
                                *dhcp_state = DhcpState::Rebinding
                            } else if dhcp_t1.is_none() {
                                *dhcp_state = DhcpState::Renewing
                            }
                        }
                        x => todo!("[DHCP] {:?}", x),
                    }
                },
            );
    }

    #[task(shared = [w5500, dhcp_state, mqtt_state], local = [bme280])]
    fn mqtt_client(cx: mqtt_client::Context) {
        log!("[TASK] mqtt_client");

        let bme280: &mut Bme280<_, _> = cx.local.bme280;

        (cx.shared.w5500, cx.shared.dhcp_state, cx.shared.mqtt_state).lock(
            |w5500, dhcp_state, mqtt_state| {
                if !dhcp_state.has_lease() {
                    // we will be spawned when the DHCP client binds
                    return;
                }

                log!("[MQTT] {:?}", mqtt_state);
                match mqtt_state {
                    MqttState::Init => {
                        const SN_IMR: SocketInterruptMask =
                            SocketInterruptMask::DEFAULT.mask_sendok();
                        w5500.set_sn_imr(MQTT_SOCKET, SN_IMR).unwrap();
                        w5500.tcp_connect(MQTT_SOCKET, 33650, &MQTT_SERVER).unwrap();

                        // we will be spawned by socket interrupt
                    }
                    MqttState::ConInt => {
                        let tx_bytes: usize = w5500.tcp_write(MQTT_SOCKET, &mqtt::CONNECT).unwrap();
                        assert_eq!(tx_bytes, mqtt::CONNECT.len());

                        // we will be spawned by socket interrupt
                    }
                    MqttState::RecvInt => {
                        let mut connack: Connack = Connack::new();
                        let rx_bytes: usize =
                            w5500.tcp_read(MQTT_SOCKET, &mut connack.buf).unwrap();
                        // the server should only ever send us the CONNACK packet
                        #[allow(clippy::if_same_then_else)]
                        if rx_bytes != CONNACK_LEN {
                            log!(
                                "[MQTT] CONNACK buffer underrrun {}/{}",
                                rx_bytes,
                                CONNACK_LEN
                            );
                            *mqtt_state = MqttState::Init;
                        } else if connack.ctrl_pkt_type() != CtrlPacket::CONNACK.into() {
                            log!(
                                "[MQTT] Excepted a CONNACK byte (0x{:02X}), found byte 0x{:02X}",
                                u8::from(CtrlPacket::CONNACK),
                                connack.ctrl_pkt_type()
                            );
                            *mqtt_state = MqttState::Init;
                        } else if connack.remaining_len() != 2 {
                            log!(
                                "[MQTT] CONNACK remaining length is unexpected: {} (expected 2)",
                                connack.remaining_len()
                            );
                            *mqtt_state = MqttState::Init;
                        } else if connack.rc() == ConnectCode::ACCEPT.into() {
                            log!("[MQTT] CONNACK");
                            *mqtt_state = MqttState::Happy;
                        } else {
                            log!("[MQTT] Unexpected CONNACK code: 0x{:02X}", connack.rc());
                            *mqtt_state = MqttState::Init;
                        }

                        mqtt_client::spawn().unwrap();
                    }
                    MqttState::Happy => {
                        let sample: Sample = bme280.sample().unwrap();
                        log!("{:#?}", sample);

                        {
                            let mut publish: Publish = Publish::new();
                            publish.set_topic(TEMPERATURE_TOPIC);
                            publish
                                .write_fmt(format_args!("{:.1}", sample.temperature))
                                .unwrap();
                            let tx_bytes: usize =
                                w5500.tcp_write(MQTT_SOCKET, publish.as_slice()).unwrap();
                            assert_eq!(tx_bytes, publish.as_slice().len());
                        }
                        {
                            let mut publish: Publish = Publish::new();
                            publish.set_topic(PRESSURE_TOPIC);
                            publish
                                .write_fmt(format_args!("{}", sample.pressure as i32))
                                .unwrap();
                            let tx_bytes: usize =
                                w5500.tcp_write(MQTT_SOCKET, publish.as_slice()).unwrap();
                            assert_eq!(tx_bytes, publish.as_slice().len());
                        }
                        {
                            let mut publish: Publish = Publish::new();
                            publish.set_topic(HUMIDITY_TOPIC);
                            publish
                                .write_fmt(format_args!("{:.1}", sample.humidity))
                                .unwrap();
                            let tx_bytes: usize =
                                w5500.tcp_write(MQTT_SOCKET, publish.as_slice()).unwrap();
                            assert_eq!(tx_bytes, publish.as_slice().len());
                        }

                        mqtt_client::spawn_after(5.seconds()).unwrap();
                    }
                }
            },
        );
    }
}
