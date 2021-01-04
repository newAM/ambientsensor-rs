//! The W5500 is the TCP/IP offload chip.
//!
//! This module simply provides extensions to the [w5500-ll] crate I made.
//!
//! I would not recommend re-using this code, but the [w5500-ll] crate is
//! pretty good.
//!
//! [w5500-ll]: https://crates.io/crates/w5500-ll

use core::convert::TryFrom;
use embedded_hal::digital::v2::OutputPin;
use w5500_ll::{Registers, Socket, SocketCommand, SocketStatus};

/// SPI Access Modes.
#[repr(u8)]
enum AccessMode {
    /// Read access.
    Read = 0,
    /// Write access.
    Write = 1,
}
impl From<AccessMode> for u8 {
    fn from(val: AccessMode) -> Self {
        val as u8
    }
}

/// Helper to generate a SPI header.
#[inline(always)]
const fn spi_header(address: u16, block: u8, mode: AccessMode) -> [u8; 3] {
    [
        (address >> 8) as u8,
        address as u8,
        (block << 3) | ((mode as u8) << 2),
    ]
}

/// W5500 blocking implementation.
pub struct W5500<SPI, CS> {
    /// SPI bus.
    spi: SPI,
    /// GPIO for chip select.
    cs: CS,
}

/// W5500 blocking implementation error type.
#[derive(Debug)]
pub enum Error<SpiError, PinError> {
    /// SPI bus error wrapper.
    Spi(SpiError),
    /// GPIO pin error wrapper.
    Pin(PinError),
}

impl<SPI, CS, SpiError, PinError> Registers for W5500<SPI, CS>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = PinError>,
    SpiError: core::fmt::Debug,
    PinError: core::fmt::Debug,
    Error<SpiError, PinError>: core::fmt::Debug,
{
    /// SPI IO error type.
    type Error = Error<SpiError, PinError>;

    /// Read from the W5500.
    #[inline(always)]
    fn read(&mut self, address: u16, block: u8, data: &mut [u8]) -> Result<(), Self::Error> {
        let header = spi_header(address, block, AccessMode::Read);
        self.with_chip_enable(|spi| {
            spi.write(&header).map_err(Error::Spi)?;
            spi.transfer(data).map_err(Error::Spi)?;
            Ok(())
        })
    }

    /// Write to the W5500.
    #[inline(always)]
    fn write(&mut self, address: u16, block: u8, data: &[u8]) -> Result<(), Self::Error> {
        let header = spi_header(address, block, AccessMode::Write);
        self.with_chip_enable(|spi| {
            spi.write(&header).map_err(Error::Spi)?;
            spi.write(&data).map_err(Error::Spi)?;
            Ok(())
        })
    }
}

impl<SPI, CS, SpiError, PinError> W5500<SPI, CS>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8, Error = SpiError>
        + embedded_hal::blocking::spi::Write<u8, Error = SpiError>,
    CS: OutputPin<Error = PinError>,
    SpiError: core::fmt::Debug,
    PinError: core::fmt::Debug,
    Error<SpiError, PinError>: core::fmt::Debug,
{
    /// Creates a new `W5500` driver from a SPI peripheral and a chip select
    /// digital I/O pin.
    ///
    /// # Example
    ///
    /// ```
    /// # use embedded_hal_mock as hal;
    /// # let spi = hal::spi::Mock::new(&[]);
    /// # let pin = hal::pin::Mock::new(&[]);
    /// use w5500_ll::blocking::W5500;
    ///
    /// let mut w5500: W5500<_, _> = W5500::new(spi, pin);
    /// ```
    pub fn new(spi: SPI, cs: CS) -> Self {
        W5500 { spi, cs }
    }

    /// Free the SPI bus and CS pin from the W5500.
    ///
    ///
    /// # Example
    ///
    /// ```
    /// # use embedded_hal_mock as hal;
    /// # let spi = hal::spi::Mock::new(&[]);
    /// # let pin = hal::pin::Mock::new(&[]);
    /// use w5500_ll::blocking::W5500;
    ///
    /// let mut w5500 = W5500::new(spi, pin);
    /// let (spi, pin) = w5500.free();
    /// ```
    pub fn free(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }

    pub fn socket_send(&mut self, socket: Socket, data: &[u8]) {
        let data_len: u16 = u16::try_from(data.len()).unwrap();
        let free_size: u16 = self.sn_tx_fsr(socket).unwrap();
        assert!(free_size >= data_len, "{} >= {}", free_size, data_len);
        let ptr: u16 = self.sn_tx_wr(socket).unwrap();
        self.set_sn_tx_buf(socket, ptr, data).unwrap();
        self.set_sn_tx_wr(socket, ptr.wrapping_add(data_len))
            .unwrap();
        self.set_sn_cr(socket, SocketCommand::Send).unwrap();
    }

    /// Poll for the desired socket status.
    pub fn poll_sn_sr(&mut self, socket: Socket, status: SocketStatus) {
        loop {
            if SocketStatus::try_from(self.sn_sr(socket).unwrap()).unwrap() == status {
                break;
            }
        }
    }

    /// Close a socket and poll for the closed status.
    pub fn socket_close(&mut self, socket: Socket) {
        self.set_sn_cr(socket, SocketCommand::Close).unwrap();
        self.poll_sn_sr(socket, SocketStatus::Closed)
    }

    #[inline(always)]
    fn with_chip_enable<T, E, F>(&mut self, mut f: F) -> Result<T, E>
    where
        F: FnMut(&mut SPI) -> Result<T, E>,
        E: core::convert::From<Error<SpiError, PinError>>,
    {
        self.cs.set_low().map_err(Error::Pin)?;
        let result = f(&mut self.spi);
        self.cs.set_high().map_err(Error::Pin)?;
        result
    }
}
