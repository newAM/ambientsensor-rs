#![no_std]
#![no_main]
#![allow(clippy::type_complexity)]

mod bme280;
mod monotonic;
mod mqtt;

use bme280::BME280;
use dhcp::{Dhcp, DhcpState, MsgType, DHCP_DESTINATION, DHCP_SOURCE_PORT};
use monotonic::{Instant, Tim6Monotonic, U16Ext};
use mqtt::{Connack, ConnectCode, CtrlPacket, Publish, CONNACK_LEN};

use core::fmt::Write;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use embedded_hal::digital::v2::OutputPin;
use rtic::app;
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
};

use w5500_hl::ll::{
    blocking::vdm::W5500,
    net::{Eui48Addr, Ipv4Addr, SocketAddrV4},
    spi::MODE as W5500_MODE,
    LinkStatus, OperationMode, PhyCfg, Registers, Socket, SocketInterrupt, SocketInterruptMask,
    SOCKETS,
};
use w5500_hl::{Tcp, Udp};

const DHCP_SOCKET: Socket = Socket::Socket0;
const MQTT_SOCKET: Socket = Socket::Socket1;

const MQTT_SERVER: SocketAddrV4 = SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 4), 1883);
const HOSTNAME: &str = "ambient1";
const TEMPERATURE_TOPIC: &str = "/home/ambient1/temperature";
const HUMIDITY_TOPIC: &str = "/home/ambient1/humidity";
const PRESSURE_TOPIC: &str = "/home/ambient1/pressure";

// this is very inefficient logging, but it is only used for assertions.
pub fn log_byte(byte: u8) {
    const USART1_BASE: u32 = 0x4001_3800;

    loop {
        let isr = unsafe { core::ptr::read_volatile((USART1_BASE + 0x1C) as *mut u32) };
        if (isr & (1 << 7)) != 0 {
            break;
        }
    }

    unsafe { core::ptr::write_volatile((USART1_BASE + 0x28) as *mut u32, u32::from(byte)) };
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
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
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
    /// Instant which the timeout was set.
    instant: Instant,
}

impl DhcpTimeout {
    /// Time to wait in milliseconds before resetting the DHCP FSM.
    const TIMEOUT_DURATION: u16 = 3000;

    /// Create a new timeout structure.
    pub fn new(state: DhcpState) -> DhcpTimeout {
        DhcpTimeout {
            state,
            instant: Instant::now(),
        }
    }

    /// Returns `true` if the timeout occured.
    pub fn timeout_occured(&self) -> bool {
        Instant::now() - self.instant > Self::TIMEOUT_DURATION.millis()
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

#[app(
    device = stm32f0xx_hal::stm32,
    peripherals = true,
    monotonic = crate::monotonic::Tim6Monotonic,
)]
const APP: () = {
    struct Resources {
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
        exti: EXTI,
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
        bme280: BME280<I2c<I2C1, PB6<gpio::Alternate<AF1>>, PB7<gpio::Alternate<AF1>>>>,
        xid: u32,
    }

    #[init(spawn = [dhcp_fsm, second_ticker], schedule = [mqtt_client])]
    fn init(cx: init::Context) -> init::LateResources {
        // let cp = cx.core;
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

        // this timer is for RTIC to schedule things
        Tim6Monotonic::initialize(dp.TIM6);

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
        let mut bme280 = BME280::new(i2c);
        let mut w5500 = W5500::new(spi1, w5500_cs);

        writeln!(&mut serial, "Hello world!").ok();
        log!("Hello world!");

        // get the MAC address from the EEPROM
        // the Microchip 25aa02e48 EEPROM comes pre-programmed with a valid MAC
        // TODO: investigate why this fails
        let mut mac: Eui48Addr = Eui48Addr::UNSPECIFIED;
        const BAD_MAC: Eui48Addr = Eui48Addr::new(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
        loop {
            eeprom.read_eui48(&mut mac.octets).unwrap();
            if mac == Eui48Addr::UNSPECIFIED || mac == BAD_MAC {
                log!("Failed to read the MAC address: {}", mac);
                nop_delay_ms(10);
            } else {
                break;
            }
        }
        debug_assert_eq!(mac.octets[0], 0x54);
        log!("MAC: {}", mac);

        w5500_hl::ll::reset(&mut w5500_rst, &mut NopDelay::default()).unwrap();

        // sanity check temperature sensor
        assert_eq!(bme280.chip_id().unwrap(), bme280::CHIP_ID);
        bme280.init().unwrap();

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
            let mut phy_cfg: PhyCfg = PhyCfg::default();
            phy_cfg.set_opmdc(OperationMode::FullDuplex10bt);
            w5500.set_phycfgr(phy_cfg).unwrap();

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
            .set_simr((1 << u8::from(DHCP_SOCKET)) | (1 << u8::from(MQTT_SOCKET)))
            .unwrap();

        // start the DHCP task
        cx.spawn.dhcp_fsm().unwrap();

        // start the 1s ticker
        cx.spawn.second_ticker().unwrap();

        #[allow(clippy::redundant_field_names)]
        init::LateResources {
            w5500: w5500,
            exti: exti,
            dhcp_timeout: None,
            dhcp_state: DhcpState::Init,
            dhcp_t1: None,
            dhcp_t2: None,
            dhcp_lease: None,
            mac: mac,
            yiaddr: Ipv4Addr::UNSPECIFIED,
            mqtt_state: MqttState::Init,
            bme280: bme280,
            // TODO: this is a terrible way to get a random XID
            xid: ((Instant::now().counts() as u32) << 16) | (Instant::now().counts() as u32),
        }
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
    #[task(binds = EXTI0_1, resources = [exti, w5500, mqtt_state], spawn = [dhcp_fsm, mqtt_client])]
    fn exti0(cx: exti0::Context) {
        log!("[TASK] exti0");
        let w5500: &mut W5500<_, _> = cx.resources.w5500;
        let exti: &mut EXTI = cx.resources.exti;
        let mqtt_state: &mut MqttState = cx.resources.mqtt_state;

        let mut sir: u8 = w5500.sir().unwrap();
        debug_assert_ne!(sir, 0x00);

        for socket in SOCKETS.iter() {
            let mask: u8 = 1 << (*socket as u8);
            if sir & mask != 0 {
                let sn_ir: SocketInterrupt = w5500.sn_ir(*socket).unwrap();
                w5500.set_sn_ir(*socket, sn_ir).unwrap();
                match *socket {
                    DHCP_SOCKET => {
                        if sn_ir.con_raised() {
                            log!("[DHCP] INTERRUPT CON");
                        }
                        if sn_ir.discon_raised() {
                            log!("[DHCP] INTERRUPT DISCON");
                        }
                        if sn_ir.recv_raised() {
                            log!("[DHCP] INTERRUPT RECV");
                            cx.spawn.dhcp_fsm().unwrap()
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
                            if cx.spawn.mqtt_client().is_err() {
                                log!("[MQTT] already spawned");
                            }
                        } else if sn_ir.con_raised() {
                            log!("[MQTT] INTERRUPT CON");
                            *mqtt_state = MqttState::ConInt;
                            if cx.spawn.mqtt_client().is_err() {
                                log!("[MQTT] already spawned");
                            }
                        } else if sn_ir.recv_raised() {
                            log!("[MQTT] INTERRUPT RECV");
                            *mqtt_state = MqttState::RecvInt;
                            if cx.spawn.mqtt_client().is_err() {
                                log!("[MQTT] already spawned");
                            }
                        }
                    }
                    _ => unreachable!("{:?}", socket),
                }
                sir &= !mask;
            }
            if sir == 0 {
                break;
            }
        }

        exti.pr.write(|w| w.pr0().set_bit());
    }

    /// This handles long timers:
    ///
    /// * DHCP FSM timeouts
    /// * DHCP T1 (renewal)
    /// * DHCP T2 (rebinding)
    /// * DHCP Lease
    #[task(schedule = [second_ticker], spawn = [dhcp_fsm], resources = [dhcp_state, dhcp_timeout, dhcp_t1, dhcp_t2, dhcp_lease])]
    fn second_ticker(cx: second_ticker::Context) {
        let dhcp_state: &mut DhcpState = cx.resources.dhcp_state;
        let dhcp_timeout: &mut Option<DhcpTimeout> = cx.resources.dhcp_timeout;
        let dhcp_t1: &mut Option<u32> = cx.resources.dhcp_t1;
        let dhcp_t2: &mut Option<u32> = cx.resources.dhcp_t2;
        let dhcp_lease: &mut Option<u32> = cx.resources.dhcp_lease;

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
            cx.spawn.dhcp_fsm().unwrap();
        }

        cx.schedule
            .second_ticker(cx.scheduled + 1000u16.millis())
            .unwrap();
    }

    #[task(
        spawn = [mqtt_client, dhcp_fsm],
        resources = [w5500, dhcp_state, dhcp_timeout, mac, yiaddr, mqtt_state, xid, dhcp_t1, dhcp_t2, dhcp_lease]
    )]
    fn dhcp_fsm(cx: dhcp_fsm::Context) {
        log!("[TASK] dhcp_fsm");
        let w5500: &mut W5500<_, _> = cx.resources.w5500;
        let dhcp_state: &mut DhcpState = cx.resources.dhcp_state;
        let dhcp_timeout: &mut Option<DhcpTimeout> = cx.resources.dhcp_timeout;
        let dhcp_t1: &mut Option<u32> = cx.resources.dhcp_t1;
        let dhcp_t2: &mut Option<u32> = cx.resources.dhcp_t2;
        let dhcp_lease: &mut Option<u32> = cx.resources.dhcp_lease;
        let mac: &mut Eui48Addr = cx.resources.mac;
        let yiaddr: &mut Ipv4Addr = cx.resources.yiaddr;
        let mqtt_state: &mut MqttState = cx.resources.mqtt_state;
        let xid: &mut u32 = cx.resources.xid;

        let mut dhcp_buf = unsafe { Dhcp::steal() };

        let mut dhcp_recv = || -> bool {
            let (num_bytes, _) = w5500.udp_recv_from(DHCP_SOCKET, dhcp_buf.recv()).unwrap();
            log!("Read {} bytes", num_bytes);
            assert!(
                num_bytes < dhcp_buf.len(),
                "Buffer was too small to receive all data"
            );
            // TODO: handle buffer underrun

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
                            cx.spawn.mqtt_client().unwrap();
                        }
                        *dhcp_state = DhcpState::Bound;
                    }
                    MsgType::Nak => {
                        log!("NAK");
                        *dhcp_state = DhcpState::Init;
                        cx.spawn.dhcp_fsm().unwrap();
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
    }

    #[task(schedule = [mqtt_client], resources = [w5500, dhcp_state, mqtt_state, bme280])]
    fn mqtt_client(cx: mqtt_client::Context) {
        log!("[TASK] mqtt_client");

        let w5500: &mut W5500<_, _> = cx.resources.w5500;
        let dhcp_state: &mut DhcpState = cx.resources.dhcp_state;
        let mqtt_state: &mut MqttState = cx.resources.mqtt_state;
        let bme280: &mut BME280<_> = cx.resources.bme280;

        if !dhcp_state.has_lease() {
            // we will be spawned when the DHCP client binds
            return;
        }

        log!("[MQTT] {:?}", mqtt_state);
        match mqtt_state {
            MqttState::Init => {
                let mut sn_imr: SocketInterruptMask = SocketInterruptMask::default();
                sn_imr.mask_sendok();
                w5500.set_sn_imr(MQTT_SOCKET, sn_imr).unwrap();
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
                let rx_bytes: usize = w5500.tcp_read(MQTT_SOCKET, &mut connack.buf).unwrap();
                // the server should only ever send us the CONNACK packet
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

                cx.schedule
                    .mqtt_client(cx.scheduled + 1u16.millis())
                    .unwrap();
            }
            MqttState::Happy => {
                let sample = bme280.sample().unwrap();
                log!("{:#?}", sample);

                {
                    let mut publish: Publish = Publish::new();
                    publish.set_topic(TEMPERATURE_TOPIC);
                    publish
                        .write_fmt(format_args!("{:.1}", sample.temperature))
                        .unwrap();
                    let tx_bytes: usize = w5500.tcp_write(MQTT_SOCKET, publish.as_slice()).unwrap();
                    assert_eq!(tx_bytes, publish.as_slice().len());
                }
                {
                    let mut publish: Publish = Publish::new();
                    publish.set_topic(PRESSURE_TOPIC);
                    publish
                        .write_fmt(format_args!("{}", sample.pressure as i32))
                        .unwrap();
                    let tx_bytes: usize = w5500.tcp_write(MQTT_SOCKET, publish.as_slice()).unwrap();
                    assert_eq!(tx_bytes, publish.as_slice().len());
                }
                {
                    let mut publish: Publish = Publish::new();
                    publish.set_topic(HUMIDITY_TOPIC);
                    publish
                        .write_fmt(format_args!("{:.1}", sample.humidity))
                        .unwrap();
                    let tx_bytes: usize = w5500.tcp_write(MQTT_SOCKET, publish.as_slice()).unwrap();
                    assert_eq!(tx_bytes, publish.as_slice().len());
                }

                cx.schedule
                    .mqtt_client(cx.scheduled + 5000u16.millis())
                    .unwrap();
            }
        }
    }

    extern "C" {
        fn I2C2();
        fn USART1();
        fn USART2();
        fn USART3_4();
    }
};
