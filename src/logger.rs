use core::fmt::Write;
use log::{LevelFilter, Metadata, Record};
use stm32f0xx_hal::pac::Peripherals;

// This is very inefficient logging, but that is OK because this is only used
// for assertions in release mode (which should never occur).
pub fn log_byte(byte: u8) {
    let dp: Peripherals = unsafe { Peripherals::steal() };
    while dp.USART1.isr.read().txe().bit_is_clear() {}
    dp.USART1.tdr.write(|w| w.tdr().bits(byte as u16));
}

struct Inner;

impl Write for Inner {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.as_bytes().iter().for_each(|x| log_byte(*x));
        Ok(())
    }
}

pub struct Logger {
    level_filter: LevelFilter,
}

impl Logger {
    pub const fn new(level_filter: LevelFilter) -> Self {
        Self { level_filter }
    }
}

impl log::Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        self.level_filter.ge(&metadata.level())
    }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let mut inner = Inner;
            writeln!(&mut inner, "[{}] {}", record.level(), record.args()).ok();
        }
    }

    fn flush(&self) {}
}
