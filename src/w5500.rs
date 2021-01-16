//! The W5500 is the TCP/IP offload chip.
//!
//! This module simply provides extensions to the [w5500-ll] crate I made.
//!
//! I would not recommend re-using this code, but the [w5500-ll] crate is
//! pretty good.
//!
//! [w5500-ll]: https://crates.io/crates/w5500-ll

use core::{cmp::min, convert::TryFrom};
use embedded_hal::digital::v2::OutputPin;
use w5500_ll::{
    net::Ipv4Addr,
    spi::{vdm_header, AccessMode},
    Protocol, Registers, Socket, SocketCommand, SocketMode, SocketStatus,
};

/// An IPv4 socket address.
///
/// This is mostly ripped directly from [`std::net::SocketAddrV4`].
///
/// IPv4 socket addresses consist of an [`IPv4` address] and a 16-bit port number, as
/// stated in [IETF RFC 793].
///
/// [IETF RFC 793]: https://tools.ietf.org/html/rfc793
/// [`IPv4` address]: Ipv4Addr
/// [`std::net::SocketAddrV4`]: https://doc.rust-lang.org/std/net/struct.SocketAddrV4.html
///
/// # Examples
///
/// ```
/// use w5500_hl::net::{Ipv4Addr, SocketAddrV4};
///
/// let socket = SocketAddrV4::new(Ipv4Addr::new(127, 0, 0, 1), 8080);
///
/// assert_eq!(socket.ip(), &Ipv4Addr::new(127, 0, 0, 1));
/// assert_eq!(socket.port(), 8080);
/// ```
pub struct SocketAddrV4 {
    ip: Ipv4Addr,
    port: u16,
}

impl SocketAddrV4 {
    /// Creates a new socket address from an [`IPv4` address] and a port number.
    ///
    /// [`IPv4` address]: Ipv4Addr
    ///
    /// # Examples
    ///
    /// ```
    /// use w5500_hl::net::{SocketAddrV4, Ipv4Addr};
    ///
    /// let socket = SocketAddrV4::new(Ipv4Addr::new(127, 0, 0, 1), 8080);
    /// ```
    pub const fn new(ip: Ipv4Addr, port: u16) -> SocketAddrV4 {
        SocketAddrV4 { ip, port }
    }

    /// Returns the IP address associated with this socket address.
    ///
    /// # Examples
    ///
    /// ```
    /// use w5500_hl::net::{SocketAddrV4, Ipv4Addr};
    ///
    /// let socket = SocketAddrV4::new(Ipv4Addr::new(127, 0, 0, 1), 8080);
    /// assert_eq!(socket.ip(), &Ipv4Addr::new(127, 0, 0, 1));
    /// ```
    pub const fn ip(&self) -> &Ipv4Addr {
        &self.ip
    }

    /// Changes the IP address associated with this socket address.
    ///
    /// # Examples
    ///
    /// ```
    /// use w5500_hl::net::{SocketAddrV4, Ipv4Addr};
    ///
    /// let mut socket = SocketAddrV4::new(Ipv4Addr::new(127, 0, 0, 1), 8080);
    /// socket.set_ip(Ipv4Addr::new(192, 168, 0, 1));
    /// assert_eq!(socket.ip(), &Ipv4Addr::new(192, 168, 0, 1));
    /// ```
    pub fn set_ip(&mut self, new_ip: Ipv4Addr) {
        self.ip = new_ip
    }

    /// Returns the port number associated with this socket address.
    ///
    /// # Examples
    ///
    /// ```
    /// use w5500_hl::net::{SocketAddrV4, Ipv4Addr};
    ///
    /// let socket = SocketAddrV4::new(Ipv4Addr::new(127, 0, 0, 1), 8080);
    /// assert_eq!(socket.port(), 8080);
    /// ```
    pub const fn port(&self) -> u16 {
        self.port
    }

    /// Changes the port number associated with this socket address.
    ///
    /// # Examples
    ///
    /// ```
    /// use w5500_hl::net::{SocketAddrV4, Ipv4Addr};
    ///
    /// let mut socket = SocketAddrV4::new(Ipv4Addr::new(127, 0, 0, 1), 8080);
    /// socket.set_port(4242);
    /// assert_eq!(socket.port(), 4242);
    /// ```
    pub fn set_port(&mut self, new_port: u16) {
        self.port = new_port
    }
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
    /// The operation needs to block to complete.
    WouldBlock,
    /// TX buffer overflow.
    ///
    /// The inner value is the free size in the TX socket buffer.
    TxOverflow(u16),
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
        let header = vdm_header(address, block, AccessMode::Read);
        self.with_chip_enable(|spi| {
            spi.write(&header).map_err(Error::Spi)?;
            spi.transfer(data).map_err(Error::Spi)?;
            Ok(())
        })
    }

    /// Write to the W5500.
    #[inline(always)]
    fn write(&mut self, address: u16, block: u8, data: &[u8]) -> Result<(), Self::Error> {
        let header = vdm_header(address, block, AccessMode::Write);
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

    pub fn socket_send(
        &mut self,
        socket: Socket,
        data: &[u8],
    ) -> Result<(), Error<SpiError, PinError>> {
        let data_len: u16 = u16::try_from(data.len()).unwrap();
        let free_size: u16 = self.sn_tx_fsr(socket)?;
        if data_len > free_size {
            return Err(Error::TxOverflow(free_size));
        }
        let ptr: u16 = self.sn_tx_wr(socket)?;
        self.set_sn_tx_buf(socket, ptr, data)?;
        self.set_sn_tx_wr(socket, ptr.wrapping_add(data_len))?;
        self.set_sn_cr(socket, SocketCommand::Send)
    }

    /// Poll for the desired socket status.
    pub fn poll_sn_sr(
        &mut self,
        socket: Socket,
        status: SocketStatus,
    ) -> Result<(), Error<SpiError, PinError>> {
        loop {
            if SocketStatus::try_from(self.sn_sr(socket)?) == Ok(status) {
                break;
            }
        }
        Ok(())
    }

    /// Close a socket and poll for the closed status.
    pub fn socket_close(&mut self, socket: Socket) -> Result<(), Error<SpiError, PinError>> {
        self.set_sn_cr(socket, SocketCommand::Close)?;
        self.poll_sn_sr(socket, SocketStatus::Closed)
    }

    /// Opens the socket as a UDP socket.
    ///
    /// The IP address is the global IPv4 shared by the device.
    pub fn bind_udp(&mut self, socket: Socket, port: u16) -> Result<(), Error<SpiError, PinError>> {
        self.set_sn_cr(socket, SocketCommand::Close)?;
        self.socket_close(socket)?;
        let mut mode: SocketMode = SocketMode::default();
        mode.set_protocol(Protocol::Udp);
        self.set_sn_mr(socket, mode)?;
        self.set_sn_port(socket, port)?;
        self.set_sn_cr(socket, SocketCommand::Open)?;
        self.poll_sn_sr(socket, SocketStatus::Udp)
    }

    /// Set the destination IPv4 and port.
    pub fn set_destination(
        &mut self,
        socket: Socket,
        addr: SocketAddrV4,
    ) -> Result<(), Error<SpiError, PinError>> {
        self.set_sn_dport(socket, addr.port())?;
        self.set_sn_dipr(socket, &addr.ip())
    }

    /// Receives a single datagram message on the socket.
    /// On success, returns the number of bytes read and the origin.
    ///
    /// The function must be called with valid byte array `buf` of sufficient
    /// size to hold the message bytes.
    /// If a message is too long to fit in the supplied buffer, excess bytes
    /// will be discarded.
    pub fn recv_from_udp(
        &mut self,
        socket: Socket,
        buf: &mut [u8],
    ) -> Result<(usize, SocketAddrV4), Error<SpiError, PinError>> {
        // note: not your standard UDP datagram header
        // For a UDP socket the W5500 UDP header contains:
        // * 4 bytes source IP
        // * 2 bytes source port
        // * 2 bytes size
        const UDP_HEADER_LEN: u16 = 8;
        const UDP_HEADER_LEN_USIZE: usize = UDP_HEADER_LEN as usize;

        let mut rsr: u16 = self.sn_rx_rsr(socket)?;

        // nothing to recieve
        if rsr < UDP_HEADER_LEN {
            return Err(Error::WouldBlock);
        }

        let mut ptr: u16 = self.sn_rx_rd(socket)?;
        let mut header: [u8; UDP_HEADER_LEN_USIZE] = [0; UDP_HEADER_LEN_USIZE];
        self.sn_rx_buf(socket, ptr, &mut header)?;
        ptr = ptr.wrapping_add(UDP_HEADER_LEN);
        rsr -= UDP_HEADER_LEN;
        let hdr_source_ip: Ipv4Addr = Ipv4Addr::new(header[0], header[1], header[2], header[3]);
        let hdr_source_port: u16 = u16::from_be_bytes([header[4], header[5]]);
        let hdr_size: u16 = u16::from_be_bytes([header[6], header[7]]);

        // not all data as indicated by the header has been buffered
        if rsr < hdr_size {
            return Err(Error::WouldBlock);
        }

        let read_size: usize = min(usize::from(hdr_size), buf.len());

        self.sn_rx_buf(socket, ptr, &mut buf[0..read_size])?;
        ptr = ptr.wrapping_add(hdr_size);
        self.set_sn_rx_rd(socket, ptr)?;
        self.set_sn_cr(socket, SocketCommand::Recv)?;

        Ok((read_size, SocketAddrV4::new(hdr_source_ip, hdr_source_port)))
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
