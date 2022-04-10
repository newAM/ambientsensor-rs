#![no_std]
#![no_main]
#![allow(clippy::type_complexity)]

mod logger;

use log::LevelFilter;
use mqtt::v3::{ConnackResult, Connect, ConnectCode, Publish, PublishBuilder, QoS, CONNACK_LEN};

use atomic_polyfill::{AtomicU32, Ordering};
use bme280_multibus::{Bme280, Sample};
use core::fmt::Write;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use embedded_hal::digital::v2::OutputPin;
use logger::Logger;
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
use systick_monotonic::{ExtU64, Systick};
use w5500_dhcp::{
    hl::{Hostname, Tcp},
    ll::{
        blocking::vdm_infallible_gpio::W5500,
        net::{Eui48Addr, Ipv4Addr, SocketAddrV4},
        spi::MODE as W5500_MODE,
        LinkStatus, OperationMode, PhyCfg, Registers, Sn, SocketInterrupt, SocketInterruptMask,
        SOCKETS,
    },
    Client as DhcpClient,
};

static LOGGER: Logger = Logger::new(LevelFilter::Trace);

const DHCP_SN: Sn = Sn::Sn0;
const MQTT_SN: Sn = Sn::Sn1;

const MQTT_SERVER: SocketAddrV4 = SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 4), 1883);
const HOSTNAME: Hostname<'static> = match Hostname::new("ambient1") {
    Some(hostname) => hostname,
    None => ::core::panic!("invalid hostname"),
};
const TEMPERATURE_PUBLISH: Publish<40> = PublishBuilder::new()
    .set_qos(QoS::AtMostOnce)
    .set_topic("/home/ambient1/temperature")
    .finalize();
const HUMIDITY_PUBLISH: Publish<40> = PublishBuilder::new()
    .set_qos(QoS::AtMostOnce)
    .set_topic("/home/ambient1/humidity")
    .finalize();
const PRESSURE_PUBLISH: Publish<40> = PublishBuilder::new()
    .set_qos(QoS::AtMostOnce)
    .set_topic("/home/ambient1/pressure")
    .finalize();

const BME280_SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
    config: bme280_multibus::Config::reset()
        .set_standby_time(bme280_multibus::Standby::Millis125)
        .set_filter(bme280_multibus::Filter::X8),
    ctrl_meas: bme280_multibus::CtrlMeas::reset()
        .set_osrs_t(bme280_multibus::Oversampling::X8)
        .set_osrs_p(bme280_multibus::Oversampling::X8)
        .set_mode(bme280_multibus::Mode::Normal),
    ctrl_hum: bme280_multibus::Oversampling::X8,
};

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    loop {
        log::error!("{}", info);
        delay_ms(500);
    }
}

#[cortex_m_rt::exception]
#[allow(non_snake_case)]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::interrupt::disable();

    loop {
        log::error!("HARD FAULT {:#?}", ef);
        delay_ms(500);
    }
}

static TIME: AtomicU32 = AtomicU32::new(0);

#[inline]
pub fn now() -> u32 {
    TIME.load(Ordering::Relaxed)
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

const SYSCLK_HZ: u32 = 8_000_000;

pub struct CycleDelay;

impl Default for CycleDelay {
    fn default() -> CycleDelay {
        CycleDelay
    }
}

impl embedded_hal::blocking::delay::DelayMs<u8> for CycleDelay {
    fn delay_ms(&mut self, ms: u8) {
        delay_ms(ms.into())
    }
}

/// Worlds worst delay function.
#[inline(always)]
pub fn delay_ms(ms: u32) {
    const CYCLES_PER_MILLIS: u32 = SYSCLK_HZ / 1000;
    cortex_m::asm::delay(CYCLES_PER_MILLIS.saturating_mul(ms));
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
        dhcp: DhcpClient<'static>,
        mqtt_state: MqttState,
    }

    #[local]
    struct Local {
        exti: EXTI,
        bme280: Bme280<
            bme280_multibus::i2c::Bme280Bus<
                I2c<I2C1, PB6<gpio::Alternate<AF1>>, PB7<gpio::Alternate<AF1>>>,
            >,
        >,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // symptom of a version mismatch when using the RTIC alpha
        // see: https://github.com/rust-embedded/cortex-m/pull/350
        // replace with `cx.cs` when cortex-m gets updated
        let cs = unsafe { &cortex_m::interrupt::CriticalSection::new() };

        unsafe { log::set_logger_racy(&LOGGER).unwrap() };
        log::set_max_level(LevelFilter::Trace);

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
        let mono = Systick::new(systick, SYSCLK_HZ);

        gpiob.pb0.into_pull_down_input(cs); // INT

        let eeprom_cs = gpiob.pb12.into_push_pull_output(cs);
        let w5500_cs = gpioa.pa4.into_push_pull_output(cs);
        let mut w5500_rst = gpioa.pa3.into_push_pull_output(cs);

        let spi1_pins = (
            gpioa.pa5.into_alternate_af0(cs), // W5500 SCK
            gpioa.pa6.into_alternate_af0(cs), // W5500 MISO
            gpioa.pa7.into_alternate_af0(cs), // W5500 MOSI
        );
        let spi2_pins = (
            gpiob.pb13.into_alternate_af0(cs), // EEPROM SCK
            gpiob.pb14.into_alternate_af0(cs), // EEPROM MISO
            gpiob.pb15.into_alternate_af0(cs), // EEPROM MOSI
        );
        let i2c1_pins = (
            gpiob.pb6.into_alternate_af1(cs), // I2C SCL
            gpiob.pb7.into_alternate_af1(cs), // I2C SDA
        );
        let uart_pins = (
            gpioa.pa9.into_alternate_af1(cs),  // UART TX
            gpioa.pa10.into_alternate_af1(cs), // UART RX
        );

        let spi1 = Spi::spi1(dp.SPI1, spi1_pins, W5500_MODE, 1.mhz(), &mut rcc);
        let spi2 = Spi::spi2(dp.SPI2, spi2_pins, W5500_MODE, 1.mhz(), &mut rcc);
        let i2c = I2c::i2c1(dp.I2C1, i2c1_pins, 100.khz(), &mut rcc);
        let mut serial = Serial::usart1(dp.USART1, uart_pins, 115_200.bps(), &mut rcc);
        let mut eeprom = eeprom25aa02e48::Eeprom25aa02e48::new(spi2, eeprom_cs);
        let mut bme280 = Bme280::from_i2c(i2c, bme280_multibus::i2c::Address::SdoGnd).unwrap();
        let mut w5500 = W5500::new(spi1, w5500_cs);

        writeln!(&mut serial, "Hello world!").ok();
        log::info!("Hello world!");

        // get the MAC address from the EEPROM
        // the Microchip 25aa02e48 EEPROM comes pre-programmed with a valid MAC
        // TODO: investigate why this fails
        const BAD_MAC: Eui48Addr = Eui48Addr::new(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF);
        let mac: Eui48Addr = loop {
            let mac: Eui48Addr = eeprom.read_eui48().unwrap().into();
            if mac == Eui48Addr::UNSPECIFIED || mac == BAD_MAC {
                log::info!("Failed to read the MAC address: {}", mac);
                delay_ms(10);
            } else {
                break mac;
            }
        };
        debug_assert_eq!(mac.octets[0], 0x54);
        log::info!("MAC: {}", mac);

        w5500_dhcp::ll::reset(&mut w5500_rst, &mut CycleDelay::default()).unwrap();

        // sanity check temperature sensor
        assert_eq!(bme280.chip_id().unwrap(), bme280_multibus::CHIP_ID);
        bme280.settings(&BME280_SETTINGS).unwrap();

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
            assert_eq!(w5500.version().unwrap(), w5500_dhcp::ll::VERSION);

            // load the MAC address we got from EEPROM
            w5500.set_shar(&mac).unwrap();
            debug_assert_eq!(w5500.shar().unwrap(), mac);

            // wait for the PHY to indicate the Ethernet link is up
            let mut attempts: u32 = 0;
            log::info!("Polling for link up");
            const PHY_CFG: PhyCfg = PhyCfg::DEFAULT.set_opmdc(OperationMode::FullDuplex10bt);
            w5500.set_phycfgr(PHY_CFG).unwrap();

            const LINK_UP_POLL_PERIOD_MILLIS: u32 = 100;
            const LINK_UP_POLL_ATTEMPTS: u32 = 50;
            loop {
                let phy_cfg: PhyCfg = w5500.phycfgr().unwrap();
                if phy_cfg.lnk() == LinkStatus::Up {
                    break 'outer phy_cfg;
                }
                if attempts >= LINK_UP_POLL_ATTEMPTS {
                    log::info!(
                        "Failed to link up in {} ms",
                        attempts * LINK_UP_POLL_PERIOD_MILLIS,
                    );
                    break;
                }
                delay_ms(LINK_UP_POLL_PERIOD_MILLIS);
                attempts += 1;
            }

            w5500_rst.set_low().unwrap();
            delay_ms(1);
            w5500_rst.set_high().unwrap();
            delay_ms(3);
        };
        log::info!("Done link up\n{}", _phy_cfg);

        let seed: u64 = u64::from(cortex_m::peripheral::SYST::get_current()) << 32
            | u64::from(cortex_m::peripheral::SYST::get_current());

        // enable DHCP & MQTT socket interrupts
        w5500
            .set_simr(DHCP_SN.bitmask() | MQTT_SN.bitmask())
            .unwrap();

        // start the 1s ticker
        second_ticker::spawn().unwrap();

        (
            Shared {
                w5500,
                dhcp: DhcpClient::new(DHCP_SN, seed, mac, HOSTNAME),
                mqtt_state: MqttState::Init,
            },
            Local { bme280, exti },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        log::info!("[TASK] idle");
        loop {
            compiler_fence(SeqCst);
        }
    }

    #[task(capacity = 5, shared = [mqtt_state])]
    fn mqtt_sn(mut cx: mqtt_sn::Context, sn_ir: SocketInterrupt) {
        cx.shared.mqtt_state.lock(|mqtt_state| {
            if sn_ir.discon_raised() | sn_ir.timeout_raised() {
                log::info!("[MQTT] INTERRUPT DISCON/TIMEOUT");
                *mqtt_state = MqttState::Init;
                if mqtt_client::spawn().is_err() {
                    log::info!("[MQTT] already spawned");
                }
            } else if sn_ir.con_raised() {
                log::info!("[MQTT] INTERRUPT CON");
                *mqtt_state = MqttState::ConInt;
                if mqtt_client::spawn().is_err() {
                    log::info!("[MQTT] already spawned");
                }
            } else if sn_ir.recv_raised() {
                log::info!("[MQTT] INTERRUPT RECV");
                *mqtt_state = MqttState::RecvInt;
                if mqtt_client::spawn().is_err() {
                    log::info!("[MQTT] already spawned");
                }
            }
        })
    }

    #[task(capacity = 5, shared = [w5500, dhcp, mqtt_state])]
    fn dhcp_sn(cx: dhcp_sn::Context, sn_ir: SocketInterrupt) {
        (cx.shared.w5500, cx.shared.dhcp, cx.shared.mqtt_state).lock(|w5500, dhcp, mqtt_state| {
            if sn_ir.con_raised() {
                log::info!("[DHCP] INTERRUPT CON");
            }
            if sn_ir.discon_raised() {
                log::info!("[DHCP] INTERRUPT DISCON");
            }
            if sn_ir.recv_raised() {
                log::info!("[DHCP] INTERRUPT RECV");
                let bound_before: bool = dhcp.is_bound();
                dhcp.process(w5500, now()).unwrap();
                if !bound_before && dhcp.is_bound() {
                    *mqtt_state = MqttState::Init;
                    if mqtt_client::spawn().is_err() {
                        log::info!("[DHCP] MQTT already spawned");
                    }
                }
            }
            if sn_ir.timeout_raised() {
                log::info!("[DHCP] INTERRUPT TIMEOUT");
            }
            if sn_ir.sendok_raised() {
                log::info!("[DHCP] INTERRUPT SENDOK");
            }
        })
    }

    /// This is the W5500 interrupt.
    ///
    /// The only interrupts we should get are for the DHCP & MQTT sockets.
    #[task(binds = EXTI0_1, local = [exti], shared = [w5500])]
    fn exti0(mut cx: exti0::Context) {
        log::info!("[TASK] exti0");

        cx.shared.w5500.lock(|w5500| {
            let sir: u8 = w5500.sir().unwrap();

            // may occur when there are power supply issues
            if sir == 0 {
                log::warn!("spurious interrupt");
                cx.local.exti.pr.write(|w| w.pr0().set_bit());
                return;
            }

            for sn in SOCKETS.iter() {
                let mask: u8 = sn.bitmask();
                if sir & mask != 0 {
                    let sn_ir: SocketInterrupt = w5500.sn_ir(*sn).unwrap();
                    w5500.set_sn_ir(*sn, sn_ir).unwrap();
                    match *sn {
                        DHCP_SN => dhcp_sn::spawn(sn_ir).unwrap(),
                        MQTT_SN => mqtt_sn::spawn(sn_ir).unwrap(),
                        _ => todo!(),
                    }
                }
            }

            cx.local.exti.pr.write(|w| w.pr0().set_bit());
        });
    }

    #[task(shared = [w5500, dhcp])]
    fn second_ticker(cx: second_ticker::Context) {
        second_ticker::spawn_after(1.secs()).unwrap();
        let monotonic_secs: u32 = TIME.fetch_add(1, Ordering::Relaxed);

        (cx.shared.w5500, cx.shared.dhcp).lock(|w5500, dhcp| {
            dhcp.process(w5500, monotonic_secs).unwrap();
        });
    }

    #[task(shared = [w5500, dhcp, mqtt_state], local = [bme280])]
    fn mqtt_client(cx: mqtt_client::Context) {
        log::info!("[TASK] mqtt_client");

        let bme280: &mut Bme280<_> = cx.local.bme280;

        (cx.shared.w5500, cx.shared.dhcp, cx.shared.mqtt_state).lock(|w5500, dhcp, mqtt_state| {
            if !dhcp.is_bound() {
                // we will be spawned when the DHCP client binds
                return;
            }

            log::info!("[MQTT] {:?}", mqtt_state);
            match mqtt_state {
                MqttState::Init => {
                    const SN_IMR: SocketInterruptMask = SocketInterruptMask::DEFAULT.mask_sendok();
                    w5500.set_sn_imr(MQTT_SN, SN_IMR).unwrap();
                    w5500.tcp_connect(MQTT_SN, 33650, &MQTT_SERVER).unwrap();

                    // we will be spawned by socket interrupt
                }
                MqttState::ConInt => {
                    let tx_bytes: usize = w5500
                        .tcp_write(MQTT_SN, &Connect::DEFAULT.into_array())
                        .unwrap()
                        .into();
                    assert_eq!(tx_bytes, Connect::LEN);

                    // we will be spawned by socket interrupt
                }
                MqttState::RecvInt => {
                    let mut buf: [u8; CONNACK_LEN] = [0; CONNACK_LEN];
                    let rx_bytes: usize = w5500.tcp_read(MQTT_SN, &mut buf).unwrap().into();

                    match ConnackResult::from_buf(&buf[..rx_bytes]) {
                        Ok(connack) => {
                            let code: ConnectCode = connack.code();
                            log::info!("[MQTT] CONNACK: {:?}", code);

                            if code == ConnectCode::Accept {
                                *mqtt_state = MqttState::Happy;
                            } else {
                                *mqtt_state = MqttState::Init;
                            }
                        }
                        Err(_e) => {
                            log::info!("[MQTT] {:?}", _e);
                            *mqtt_state = MqttState::Init;
                        }
                    }

                    mqtt_client::spawn().unwrap();
                }
                MqttState::Happy => {
                    let sample: Sample = match bme280.sample() {
                        Ok(s) => s,
                        Err(e) => {
                            log::warn!("Failed to sample BME280: {:?}", e);
                            mqtt_client::spawn_after(5.secs()).unwrap();
                            return;
                        }
                    };
                    log::info!("{:#?}", sample);

                    {
                        let mut publish = TEMPERATURE_PUBLISH;
                        write!(&mut publish, "{:.1}", sample.temperature).unwrap();
                        let tx_bytes: usize =
                            w5500.tcp_write(MQTT_SN, publish.as_slice()).unwrap().into();
                        assert_eq!(tx_bytes, publish.as_slice().len());
                    }
                    {
                        let mut publish = PRESSURE_PUBLISH;
                        write!(&mut publish, "{}", sample.pressure as i32).unwrap();
                        let tx_bytes: usize =
                            w5500.tcp_write(MQTT_SN, publish.as_slice()).unwrap().into();
                        assert_eq!(tx_bytes, publish.as_slice().len());
                    }
                    {
                        let mut publish = HUMIDITY_PUBLISH;
                        write!(&mut publish, "{:.1}", sample.humidity).unwrap();
                        let tx_bytes: usize =
                            w5500.tcp_write(MQTT_SN, publish.as_slice()).unwrap().into();
                        assert_eq!(tx_bytes, publish.as_slice().len());
                    }

                    mqtt_client::spawn_after(5.secs()).unwrap();
                }
            }
        });
    }
}
