#![no_std]
#![no_main]

mod bme280;
mod dhcp;
mod monotonic;
mod mqtt;
mod w5500;

use bme280::BME280;
use dhcp::{DhcpBuf, DhcpState, MsgType, DHCP_DESTINATION_PORT, DHCP_SOURCE_PORT};
use monotonic::{Instant, Tim6Monotonic, U16Ext};
use mqtt::{Connack, CtrlPacket, Publish, CONNACK_LEN};
use net::Eui48Addr;
use w5500::W5500;

use core::fmt::Write;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering::SeqCst;
use embedded_hal::digital::v2::OutputPin;
use rtic::app;
use rtt_target::{rprintln, rtt_init, ChannelMode, CriticalSectionFunc, UpChannel};
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
    spi::{self, Spi},
};
use w5500_ll::{self, OperationMode, SocketInterruptMask};
use w5500_ll::{
    net, LinkStatus, Protocol, Registers, Socket, SocketCommand, SocketInterrupt, SocketMode,
    SocketStatus,
};
use w5500_ll::{net::Ipv4Addr, PhyCfg};

const DHCP_SOCKET: Socket = Socket::Socket0;
const MQTT_SOCKET: Socket = Socket::Socket1;

const HOSTNAME: &str = "ambient1";
const TEMPERATURE_TOPIC: &str = "/home/ambient1/temperature";
const HUMIDITY_TOPIC: &str = "/home/ambient1/humidity";
const PRESSURE_TOPIC: &str = "/home/ambient1/pressure";

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
        channel.set_mode(ChannelMode::BlockIfFull);
        writeln!(channel, "{}", info).ok();
    }

    loop {
        compiler_fence(SeqCst);
    }
}

#[cortex_m_rt::exception]
#[allow(non_snake_case)]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::interrupt::disable();

    if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
        channel.set_mode(ChannelMode::BlockIfFull);
        writeln!(channel, "HARD FAULT {:#?}", ef).ok();
    }

    loop {
        compiler_fence(SeqCst);
    }
}

fn rtt_init() {
    let channels = rtt_init! {
        up: {
            0: {
                size: 1024
                mode: BlockIfFull // NoBlockSkip
                name: "Terminal"
            }
        }
    };
    unsafe {
        rtt_target::set_print_channel_cs(
            channels.up.0,
            &((|arg, f| cortex_m::interrupt::free(|_| f(arg))) as CriticalSectionFunc),
        );
    }
}

/// State handling for the MQTT client task.
///
/// This is internal code, and has nothing to do with the MQTT protocol.
///
/// The client simply publishes sensor samples to various topics.
/// Nothing fancy here.
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
        dhcp_state_for_timeout: DhcpState,
        dhcp_state: DhcpState,
        yiaddr: Ipv4Addr,
        mqtt_state: MqttState,
        bme280: BME280<I2c<I2C1, PB6<gpio::Alternate<AF1>>, PB7<gpio::Alternate<AF1>>>>,
        xid: u32,
    }

    #[init(spawn = [dhcp_fsm], schedule = [mqtt_client])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init();
        rprintln!("[TASK] init");

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
        let ((eepom_cs, w5500_cs, mut w5500_rst), spi1_pins, spi2_pins, i2c1_pins) =
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
                )
            });
        let spi1 = Spi::spi1(dp.SPI1, spi1_pins, w5500_ll::SPI_MODE, 1.mhz(), &mut rcc);
        let spi2 = Spi::spi2(dp.SPI2, spi2_pins, w5500_ll::SPI_MODE, 1.mhz(), &mut rcc);
        let i2c = I2c::i2c1(dp.I2C1, i2c1_pins, 100.khz(), &mut rcc);
        let mut eeprom = eeprom25aa02e48::Eeprom25aa02e48::new(spi2, eepom_cs);
        let mut bme280 = BME280::new(i2c);
        let mut w5500 = W5500::new(spi1, w5500_cs);

        // get the MAC address from the EEPROM
        // the Microchip 25aa02e48 EEPROM comes pre-programmed with a valid MAC
        // TODO: investigate why this fails
        let mut mac: Eui48Addr = net::Eui48Addr::UNSPECIFIED;
        const BAD_MAC: Eui48Addr = Eui48Addr::new(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
        loop {
            eeprom.read_eui48(&mut mac.octets).unwrap();
            if mac == Eui48Addr::UNSPECIFIED || mac == BAD_MAC {
                nop_delay_ms(10);
            } else {
                break;
            }
        }
        debug_assert_eq!(mac.octets[0], 0x54);
        rprintln!("MAC: {}", mac);

        // sanity check temperature sensor
        assert_eq!(bme280.chip_id().unwrap(), bme280::CHIP_ID);
        bme280.init().unwrap();

        // reset the W5500
        w5500_rst.set_high().unwrap();
        nop_delay_ms(1);
        w5500_rst.set_low().unwrap();
        nop_delay_ms(1);
        w5500_rst.set_high().unwrap();
        nop_delay_ms(3);

        // enable external interrupt for pb0 (W5500 interrupt)
        syscfg.exticr1.modify(|_, w| w.exti0().pb0());
        debug_assert!(syscfg.exticr1.read().exti0().is_pb0());

        // set interrupt request mask for line 1
        exti.imr.modify(|_, w| w.mr0().set_bit());

        // set interrupt falling trigger for line 1
        exti.rtsr.modify(|_, w| w.tr0().clear_bit());
        exti.ftsr.modify(|_, w| w.tr0().set_bit());

        // sanity check W5500 communications
        assert_eq!(w5500.version().unwrap(), w5500_ll::VERSION);

        // load the MAC address we got from EEPROM
        w5500.set_shar(&mac).unwrap();

        // wait for the PHY to indicate the Ethernet link is up
        let mut attempts: usize = 0;
        rprintln!("Polling for link up");
        let mut phy_cfg: PhyCfg = PhyCfg::default();
        phy_cfg.set_opmdc(OperationMode::FullDuplex10bt);
        w5500.set_phycfgr(phy_cfg).unwrap();
        let mut phy_cfg: PhyCfg = w5500.phycfgr().unwrap();
        while phy_cfg.lnk() != LinkStatus::Up {
            attempts += 1;
            assert!(attempts < 50);
            nop_delay_ms(100);
            phy_cfg = w5500.phycfgr().unwrap();
        }
        rprintln!("Done link up in about {} ms\n", attempts * 100);
        rprintln!("{}", phy_cfg);

        // enable all the socket interrupts
        w5500.set_simr(0xFF).unwrap();

        // start the DHCP task
        cx.spawn.dhcp_fsm().unwrap();
        // cx.schedule
        //     .mqtt_client(cx.start + 3000u16.millis())
        //     .unwrap();

        #[allow(clippy::redundant_field_names)]
        init::LateResources {
            w5500: w5500,
            exti: exti,
            dhcp_state_for_timeout: DhcpState::INIT,
            dhcp_state: DhcpState::INIT,
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
        rprintln!("[TASK] idle");
        loop {
            compiler_fence(SeqCst);
        }
    }

    /// This is the W5500 interrupt.
    ///
    /// The only interrupts we should get are for the DHCP & MQTT sockets.
    /// Other interrupts are left unmasked because I like to live dangerously.
    #[task(binds = EXTI0_1, resources = [exti, w5500, mqtt_state], spawn = [dhcp_fsm, mqtt_client])]
    fn exti0(cx: exti0::Context) {
        rprintln!("[TASK] exti0");
        let w5500: &mut W5500<_, _> = cx.resources.w5500;
        let exti: &mut EXTI = cx.resources.exti;
        let mqtt_state: &mut MqttState = cx.resources.mqtt_state;

        let mut sir: u8 = w5500.sir().unwrap();
        debug_assert_ne!(sir, 0x00);

        for socket in w5500_ll::SOCKETS.iter() {
            let mask: u8 = 1 << (*socket as u8);
            if sir & mask != 0 {
                let sn_ir: SocketInterrupt = w5500.sn_ir(*socket).unwrap();
                w5500.set_sn_ir(*socket, sn_ir).unwrap();
                match *socket {
                    DHCP_SOCKET => {
                        if sn_ir.con_raised() {
                            rprintln!("[DHCP] INTERRUPT CON")
                        }
                        if sn_ir.discon_raised() {
                            rprintln!("[DHCP] INTERRUPT DISCON")
                        }
                        if sn_ir.recv_raised() {
                            rprintln!("[DHCP] INTERRUPT RECV");
                            cx.spawn.dhcp_fsm().unwrap()
                        }
                        if sn_ir.timeout_raised() {
                            rprintln!("[DHCP] INTERRUPT TIMEOUT")
                        }
                        if sn_ir.sendok_raised() {
                            rprintln!("[DHCP] INTERRUPT SENDOK")
                        }
                    }
                    MQTT_SOCKET => {
                        if sn_ir.discon_raised() | sn_ir.timeout_raised() {
                            rprintln!("[MQTT] INTERRUPT DISCON/TIMEOUT");
                            *mqtt_state = MqttState::Init;
                            if cx.spawn.mqtt_client().is_err() {
                                rprintln!("[MQTT] already spawned");
                            }
                        } else if sn_ir.con_raised() {
                            rprintln!("[MQTT] INTERRUPT CON");
                            *mqtt_state = MqttState::ConInt;
                            if cx.spawn.mqtt_client().is_err() {
                                rprintln!("[MQTT] already spawned");
                            }
                        } else if sn_ir.recv_raised() {
                            rprintln!("[MQTT] INTERRUPT RECV");
                            *mqtt_state = MqttState::RecvInt;
                            if cx.spawn.mqtt_client().is_err() {
                                rprintln!("[MQTT] already spawned");
                            }
                        }
                    }
                    _ => unimplemented!("{:?}", socket),
                }
                sir &= !mask;
            }
            if sir == 0 {
                break;
            }
        }

        exti.pr.write(|w| w.pr0().set_bit());
    }

    #[task(spawn = [dhcp_fsm], resources = [dhcp_state, dhcp_state_for_timeout])]
    fn dhcp_timeout(cx: dhcp_timeout::Context) {
        rprintln!("[TASK] dhcp_timeout");
        let dhcp_state: &mut DhcpState = cx.resources.dhcp_state;
        let dhcp_state_for_timeout: &mut DhcpState = cx.resources.dhcp_state_for_timeout;

        // state transition should have occured
        if dhcp_state == dhcp_state_for_timeout {
            rprintln!("[DHCP] timeout");
            *dhcp_state = DhcpState::INIT;
            cx.spawn.dhcp_fsm().unwrap();
        }
    }

    #[task(
        schedule = [dhcp_timeout],
        spawn = [mqtt_client],
        resources = [w5500, dhcp_state, dhcp_state_for_timeout, mac, yiaddr, mqtt_state, xid]
    )]
    fn dhcp_fsm(cx: dhcp_fsm::Context) {
        rprintln!("[TASK] dhcp_fsm");
        let w5500: &mut W5500<_, _> = cx.resources.w5500;
        let dhcp_state: &mut DhcpState = cx.resources.dhcp_state;
        let dhcp_state_for_timeout: &mut DhcpState = cx.resources.dhcp_state_for_timeout;
        let mac: &mut Eui48Addr = cx.resources.mac;
        let yiaddr: &mut Ipv4Addr = cx.resources.yiaddr;
        let mqtt_state: &mut MqttState = cx.resources.mqtt_state;
        let xid: &mut u32 = cx.resources.xid;

        let mut dhcp_buf = unsafe { DhcpBuf::steal() };

        let mut dhcp_recv = || -> bool {
            let mut ptr: u16 = w5500.sn_rx_rd(DHCP_SOCKET).unwrap();
            let mut rsr: u16 = w5500.sn_rx_rsr(DHCP_SOCKET).unwrap();
            let mut header: [u8; 8] = [0xAA; 8];
            assert!(rsr >= header.len() as u16);
            w5500.sn_rx_buf(DHCP_SOCKET, ptr, &mut header).unwrap();
            ptr = ptr.wrapping_add(header.len() as u16);
            rsr -= header.len() as u16;
            dhcp_buf.zero();
            w5500
                .sn_rx_buf(DHCP_SOCKET, ptr, &mut dhcp_buf.buf[0..usize::from(rsr)])
                .unwrap();
            ptr = ptr.wrapping_add(rsr);
            w5500.set_sn_rx_rd(DHCP_SOCKET, ptr).unwrap();
            w5500.set_sn_cr(DHCP_SOCKET, SocketCommand::Recv).unwrap();

            if !dhcp_buf.is_bootreply() {
                rprintln!("not a bootreply");
                return true;
            }

            if dhcp_buf.xid() != *xid {
                rprintln!(
                    "xid does not match: 0x{:08X} != 0x{:08X}",
                    dhcp_buf.xid(),
                    xid
                );
                return true;
            }

            false
        };

        match dhcp_state {
            DhcpState::INIT => {
                rprintln!("DHCP INIT");
                w5500
                    .set_sn_dport(DHCP_SOCKET, DHCP_DESTINATION_PORT)
                    .unwrap();
                w5500
                    .set_sn_dipr(DHCP_SOCKET, &Ipv4Addr::BROADCAST)
                    .unwrap();
                w5500.socket_close(DHCP_SOCKET);

                let mut mode: SocketMode = SocketMode::default();
                mode.set_protocol(Protocol::Udp);
                w5500.set_sn_mr(DHCP_SOCKET, mode).unwrap();
                w5500.set_sn_port(DHCP_SOCKET, DHCP_SOURCE_PORT).unwrap();
                w5500.set_sn_cr(DHCP_SOCKET, SocketCommand::Open).unwrap();
                w5500.poll_sn_sr(DHCP_SOCKET, SocketStatus::Udp);

                dhcp_buf.dhcp_discover(mac, HOSTNAME);
                dhcp_buf.set_xid(*xid);
                debug_assert_eq!(dhcp_buf.xid(), *xid);
                rprintln!("[DHCP] SENDING DISCOVER");
                w5500.socket_send(DHCP_SOCKET, &dhcp_buf.buf[..dhcp_buf.buf_ptr]);

                *dhcp_state = DhcpState::SELECTING;

                // start timeout if not already started
                *dhcp_state_for_timeout = *dhcp_state;
                cx.schedule
                    .dhcp_timeout(Instant::now() + 4000u16.millis())
                    .ok();
            }
            DhcpState::SELECTING => {
                rprintln!("DHCP SELECTING");

                if dhcp_recv() {
                    return;
                }

                *yiaddr = dhcp_buf.yiaddr();
                rprintln!("yiaddr: {}", yiaddr);

                dhcp_buf.dhcp_request(mac, yiaddr, HOSTNAME);
                dhcp_buf.set_xid(*xid);
                rprintln!("[DHCP] SENDING REQUEST");
                w5500.socket_send(DHCP_SOCKET, &dhcp_buf.buf[..dhcp_buf.buf_ptr]);

                *dhcp_state = DhcpState::REQUESTING;

                // start timeout if not already started
                *dhcp_state_for_timeout = *dhcp_state;
                cx.schedule
                    .dhcp_timeout(Instant::now() + 4000u16.millis())
                    .ok();
            }
            DhcpState::REQUESTING => {
                if dhcp_recv() {
                    return;
                }

                match dhcp_buf.message_type().unwrap() {
                    MsgType::Ack => {
                        let subnet_mask: Ipv4Addr = dhcp_buf.subnet_mask().unwrap();
                        let gateway: Ipv4Addr = dhcp_buf.dhcp_server().unwrap();
                        let renewal_time: u32 = dhcp_buf.renewal_time().unwrap();
                        rprintln!("Subnet Mask: {}", subnet_mask);
                        rprintln!("Client IP:   {}", *yiaddr);
                        rprintln!("Gateway IP:  {}", gateway);
                        debug_assert_eq!(dhcp_buf.yiaddr(), *yiaddr);
                        w5500.set_subr(&subnet_mask).unwrap();
                        w5500.set_sipr(yiaddr).unwrap();
                        w5500.set_gar(&gateway).unwrap();
                        *dhcp_state = DhcpState::BOUND;
                        *mqtt_state = MqttState::Init;
                        cx.spawn.mqtt_client().unwrap();
                    }
                    MsgType::Nak => {
                        rprintln!("NAK");
                        *dhcp_state = DhcpState::INIT;
                    }
                    x => {
                        rprintln!("Ignoring message: {:?}", x);
                    }
                }
            }
            DhcpState::BOUND => {
                panic!("TODO: BOUND")
            }
            x => panic!("TODO: {:?}", x),
        }
    }

    #[task(schedule = [mqtt_client], resources = [w5500, dhcp_state, mqtt_state, bme280])]
    fn mqtt_client(cx: mqtt_client::Context) {
        rprintln!("[TASK] mqtt_client");

        let w5500: &mut W5500<_, _> = cx.resources.w5500;
        let dhcp_state: &mut DhcpState = cx.resources.dhcp_state;
        let mqtt_state: &mut MqttState = cx.resources.mqtt_state;
        let bme280: &mut BME280<_> = cx.resources.bme280;

        if *dhcp_state != DhcpState::BOUND {
            // we will be spawned when the DHCP client binds
            return;
        }

        match mqtt_state {
            MqttState::Init => {
                rprintln!("[MQTT] init");
                w5500.socket_close(MQTT_SOCKET);
                let mut sn_imr = SocketInterruptMask::default();
                sn_imr.mask_sendok();
                w5500.set_sn_imr(MQTT_SOCKET, sn_imr).unwrap();
                let mut mode: SocketMode = SocketMode::default();
                mode.set_protocol(Protocol::Tcp);
                w5500.set_sn_mr(MQTT_SOCKET, mode).unwrap();
                w5500.set_sn_port(MQTT_SOCKET, 33650).unwrap();
                w5500.set_sn_cr(MQTT_SOCKET, SocketCommand::Open).unwrap();
                w5500.set_sn_dport(MQTT_SOCKET, 1883).unwrap();
                w5500
                    .set_sn_dipr(MQTT_SOCKET, &Ipv4Addr::new(10, 0, 0, 4))
                    .unwrap();
                w5500.poll_sn_sr(MQTT_SOCKET, SocketStatus::Init);
                w5500
                    .set_sn_cr(MQTT_SOCKET, SocketCommand::Connect)
                    .unwrap();

                // we will be spawned by socket interrupt
            }
            MqttState::ConInt => {
                rprintln!("[MQTT] CONNECT");
                w5500.socket_send(MQTT_SOCKET, &mqtt::CONNECT);

                // we will be spawned by socket interrupt
            }
            MqttState::RecvInt => {
                rprintln!("[MQTT] recv");
                let mut connack: Connack = Connack::new();
                let rsr: u16 = w5500.sn_rx_rsr(MQTT_SOCKET).unwrap();
                // the server should only ever send us the CONNACK packet
                assert_eq!(usize::from(rsr), CONNACK_LEN);
                let ptr: u16 = w5500.sn_rx_rd(MQTT_SOCKET).unwrap();
                w5500.sn_rx_buf(MQTT_SOCKET, ptr, &mut connack.buf).unwrap();
                w5500
                    .set_sn_rx_rd(MQTT_SOCKET, ptr.wrapping_add(rsr))
                    .unwrap();
                w5500.set_sn_cr(MQTT_SOCKET, SocketCommand::Recv).unwrap();

                assert_eq!(connack.msg_type(), CtrlPacket::CONNACK.into());
                assert_eq!(connack.msg_len(), 2);

                match connack.rc() {
                    0 => {
                        rprintln!("[MQTT] CONNACK");
                        *mqtt_state = MqttState::Happy
                    }
                    x => {
                        rprintln!("[MQTT] BAD CONNACK: {}", x);
                        *mqtt_state = MqttState::Init;
                    }
                }

                cx.schedule
                    .mqtt_client(cx.scheduled + 1u16.millis())
                    .unwrap();
            }
            MqttState::Happy => {
                rprintln!("[MQTT] HAPPY");
                let sample = bme280.sample().unwrap();
                rprintln!("{:#?}", sample);

                {
                    let mut publish: Publish = Publish::new();
                    publish.set_topic(TEMPERATURE_TOPIC);
                    publish
                        .write_fmt(format_args!("{:.1}", sample.temperature))
                        .unwrap();
                    w5500.socket_send(MQTT_SOCKET, publish.as_slice());
                }
                {
                    let mut publish: Publish = Publish::new();
                    publish.set_topic(PRESSURE_TOPIC);
                    publish
                        .write_fmt(format_args!("{}", sample.pressure as i32))
                        .unwrap();
                    w5500.socket_send(MQTT_SOCKET, publish.as_slice());
                }
                {
                    let mut publish: Publish = Publish::new();
                    publish.set_topic(HUMIDITY_TOPIC);
                    publish
                        .write_fmt(format_args!("{:.1}", sample.humidity))
                        .unwrap();
                    w5500.socket_send(MQTT_SOCKET, publish.as_slice());
                }

                cx.schedule
                    .mqtt_client(cx.scheduled + 4000u16.millis())
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
