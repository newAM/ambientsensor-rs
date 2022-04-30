#![no_std]
#![no_main]
#![allow(clippy::type_complexity)]

mod logger;

use log::LevelFilter;

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
    hl::Hostname,
    ll::{
        blocking::vdm_infallible_gpio::W5500,
        net::{Eui48Addr, Ipv4Addr, SocketAddrV4},
        spi::MODE as W5500_MODE,
        LinkStatus, OperationMode, PhyCfg, Registers, Sn,
    },
    Client as DhcpClient,
};
use w5500_mqtt::{Client as MqttClient, ClientId, Event as MqttEvent, SRC_PORT as MQTT_SRC_PORT};

static LOGGER: Logger = Logger::new(LevelFilter::Trace);

const DHCP_SN: Sn = Sn::Sn0;
const MQTT_SN: Sn = Sn::Sn1;

const MQTT_SERVER: SocketAddrV4 = SocketAddrV4::new(Ipv4Addr::new(10, 0, 0, 4), 1883);
const NAME: &str = "ambient1";
const HOSTNAME: Hostname<'static> = Hostname::new_unwrapped(NAME);
const CLIENT_ID: ClientId<'static> = ClientId::new_unwrapped(NAME);
const TEMPERATURE_TOPIC: &str = "/home/ambient1/temperature";
const HUMIDITY_TOPIC: &str = "/home/ambient1/humidity";
const PRESSURE_TOPIC: &str = "/home/ambient1/pressure";

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

fn monotonic_secs() -> u32 {
    app::monotonics::now()
        .duration_since_epoch()
        .to_secs()
        .try_into()
        .unwrap()
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
        mqtt: MqttClient<'static>,
        dhcp_spawn_at: Option<u32>,
        mqtt_spawn_at: Option<u32>,
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

        let mut mqtt: MqttClient = MqttClient::new(MQTT_SN, MQTT_SRC_PORT, MQTT_SERVER);
        mqtt.set_client_id(CLIENT_ID);

        let seed: u64 = u64::from(cortex_m::peripheral::SYST::get_current()) << 32
            | u64::from(cortex_m::peripheral::SYST::get_current());

        let dhcp = DhcpClient::new(DHCP_SN, seed, mac, HOSTNAME);
        dhcp.setup_socket(&mut w5500).unwrap();

        // start the DHCP client
        dhcp_sn::spawn().unwrap();

        // start the timeout tracker
        timeout_tracker::spawn().unwrap();

        (
            Shared {
                w5500,
                dhcp,
                mqtt,
                dhcp_spawn_at: None,
                mqtt_spawn_at: None,
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

    #[task(shared = [w5500, dhcp, dhcp_spawn_at])]
    fn dhcp_sn(cx: dhcp_sn::Context) {
        log::info!("[TASK] dhcp_sn");

        (cx.shared.w5500, cx.shared.dhcp, cx.shared.dhcp_spawn_at).lock(
            |w5500, dhcp, dhcp_spawn_at| {
                let leased_before: bool = dhcp.has_lease();
                let now: u32 = monotonic_secs();
                let spawn_after_secs: u32 = dhcp.process(w5500, now).unwrap();

                let spawn_at: u32 = now + spawn_after_secs;
                *dhcp_spawn_at = Some(spawn_at);
                log::info!("[DHCP] spawning after {spawn_after_secs} seconds, at {spawn_at}");

                // spawn MQTT task if bound
                if dhcp.has_lease() && !leased_before && mqtt_sn::spawn().is_err() {
                    log::error!("MQTT task is already spawned")
                }
            },
        )
    }

    /// This is the W5500 interrupt.
    ///
    /// The only interrupts we should get are for the DHCP & MQTT sockets.
    #[task(binds = EXTI0_1, local = [exti], shared = [w5500])]
    #[allow(clippy::collapsible_if)]
    fn exti0(mut cx: exti0::Context) {
        log::info!("[TASK] exti0");

        cx.shared.w5500.lock(|w5500| {
            let sir: u8 = w5500.sir().unwrap();

            cx.local.exti.pr.write(|w| w.pr0().set_bit());

            // may occur when there are power supply issues
            if sir == 0 {
                log::warn!("[W5500] spurious interrupt");
                return;
            }

            if sir & DHCP_SN.bitmask() != 0 {
                if dhcp_sn::spawn().is_err() {
                    log::error!("DHCP task already spawned")
                }
            }

            if sir & MQTT_SN.bitmask() != 0 {
                if mqtt_sn::spawn().is_err() {
                    log::error!("MQTT task already spawned")
                }
            }
        });
    }

    #[task(shared = [dhcp_spawn_at, mqtt_spawn_at])]
    fn timeout_tracker(mut cx: timeout_tracker::Context) {
        timeout_tracker::spawn_after(1.secs()).unwrap();

        let now: u32 = monotonic_secs();

        cx.shared.dhcp_spawn_at.lock(|dhcp_spawn_at| {
            if let Some(then) = dhcp_spawn_at {
                if now >= *then {
                    if dhcp_sn::spawn().is_err() {
                        log::error!("DHCP task is already spawned")
                    }
                    *dhcp_spawn_at = None;
                }
            }
        });

        cx.shared.mqtt_spawn_at.lock(|mqtt_spawn_at| {
            if let Some(then) = mqtt_spawn_at {
                if now >= *then {
                    if mqtt_sn::spawn().is_err() {
                        log::error!("MQTT task is already spawned")
                    }
                    *mqtt_spawn_at = None;
                }
            }
        });
    }

    #[task(shared = [w5500, mqtt, mqtt_spawn_at], local = [bme280])]
    fn mqtt_sn(cx: mqtt_sn::Context) {
        log::info!("[TASK] mqtt_sn");

        let bme280: &mut Bme280<_> = cx.local.bme280;

        (cx.shared.w5500, cx.shared.mqtt, cx.shared.mqtt_spawn_at).lock(
            |w5500, mqtt, mqtt_spawn_at| {
                loop {
                    let now: u32 = monotonic_secs();
                    match mqtt.process(w5500, now) {
                        Ok(MqttEvent::CallAfter(secs)) => {
                            *mqtt_spawn_at = Some(now + secs);
                            break;
                        }
                        Ok(MqttEvent::ConnAck) => {
                            log::info!("[MQTT] ConnAck");
                            // can subscribe to topics here
                            // not needed for this
                        }
                        Ok(MqttEvent::Publish(reader)) => {
                            log::warn!("should not get Publish never subscribed");
                            reader.done().unwrap();
                        }
                        Ok(MqttEvent::SubAck(_) | MqttEvent::UnSubAck(_)) => {
                            log::warn!("should not get (Un)SubAck, never (un)subscribed");
                        }
                        Ok(MqttEvent::None) => {
                            let sample: Sample = match bme280.sample() {
                                Ok(s) => s,
                                Err(e) => {
                                    log::warn!("Failed to sample BME280: {e:?}");
                                    *mqtt_spawn_at = Some(now + 5);
                                    return;
                                }
                            };
                            log::info!("{:#?}", sample);
                            {
                                let mut data: heapless::String<16> = heapless::String::new();
                                write!(&mut data, "{:.1}", sample.temperature).unwrap();
                                mqtt.publish(w5500, TEMPERATURE_TOPIC, data.as_bytes())
                                    .unwrap();
                            }
                            {
                                let mut data: heapless::String<16> = heapless::String::new();
                                write!(&mut data, "{:.0}", sample.pressure).unwrap();
                                mqtt.publish(w5500, PRESSURE_TOPIC, data.as_bytes())
                                    .unwrap();
                            }
                            {
                                let mut data: heapless::String<16> = heapless::String::new();
                                write!(&mut data, "{:.1}", sample.humidity).unwrap();
                                mqtt.publish(w5500, HUMIDITY_TOPIC, data.as_bytes())
                                    .unwrap();
                            }
                            *mqtt_spawn_at = Some(now + 5);
                            break;
                        }
                        Err(e) => {
                            log::error!("[MQTT] {e:?}");
                            *mqtt_spawn_at = Some(now + 10);
                            break;
                        }
                    }
                }
            },
        );
    }
}
