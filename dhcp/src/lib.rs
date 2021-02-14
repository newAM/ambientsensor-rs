#![no_std]

use core::convert::{TryFrom, TryInto};
use w5500_ll::net::{Eui48Addr, Ipv4Addr, SocketAddrV4};

/// DHCP state.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DhcpState {
    /// Initial DHCP state.
    Init,
    Selecting,
    Requesting,
    InitReboot,
    Rebooting,
    Bound,
    Renewing,
    Rebinding,
}

impl DhcpState {
    /// Returns `true` if the state is [`DhcpState::Init`].
    pub fn is_init(&self) -> bool {
        matches!(self, Self::Init)
    }

    /// Returns `true` if the state is [`DhcpState::Selecting`].
    pub fn is_selecting(&self) -> bool {
        matches!(self, Self::Selecting)
    }

    /// Returns `true` if the state is [`DhcpState::Requesting`].
    pub fn is_requesting(&self) -> bool {
        matches!(self, Self::Requesting)
    }

    /// Returns `true` if the state is [`DhcpState::InitReboot`].
    pub fn is_init_reboot(&self) -> bool {
        matches!(self, Self::InitReboot)
    }

    /// Returns `true` if the state is [`DhcpState::Rebooting`].
    pub fn is_rebooting(&self) -> bool {
        matches!(self, Self::Rebooting)
    }

    /// Returns `true` if the state is [`DhcpState::Bound`].
    pub fn is_bound(&self) -> bool {
        matches!(self, Self::Bound)
    }

    /// Returns `true` if the state is [`DhcpState::Renewing`].
    pub fn is_renewing(&self) -> bool {
        matches!(self, Self::Renewing)
    }

    /// Returns `true` if the state is [`DhcpState::Rebinding`].
    pub fn is_rebinding(&self) -> bool {
        matches!(self, Self::Rebinding)
    }

    /// Returns `true` if there is a valid IP lease in this state
    pub fn has_lease(&self) -> bool {
        matches!(self, Self::Bound | Self::Renewing | Self::Rebinding)
    }
}

/// DHCP message types.
///
/// From [RFC 2132 Section 9.6](https://tools.ietf.org/html/rfc2132#section-9.6)
#[repr(u8)]
#[derive(Debug, PartialEq, Eq)]
pub enum MsgType {
    Discover = 1,
    Offer = 2,
    Request = 3,
    Decline = 4,
    Ack = 5,
    Nak = 6,
    Release = 7,
    Inform = 8,
}
impl From<MsgType> for u8 {
    fn from(val: MsgType) -> u8 {
        val as u8
    }
}

/// DHCP options.
#[repr(u8)]
#[derive(Debug, PartialEq, Eq)]
#[allow(dead_code)]
enum DhcpOption {
    Pad = 0,
    SubnetMask = 1,
    TimeOffset = 2,
    Router = 3,
    TimeServer = 4,
    NameServer = 5,
    Dns = 6,
    Hostname = 12,
    /// Requested IP Address
    ///
    /// From [RFC 2132 Section 9.1](https://tools.ietf.org/html/rfc2132#section-9.1)
    RequestedIp = 50,
    LeaseTime = 51,
    MessageType = 53,
    ServerId = 54,
    ParameterRequest = 55,
    RenewalTime = 58,
    RebindingTime = 59,
    /// Client-identifier
    ///
    /// From [RFC 2132 Section 9.14](https://tools.ietf.org/html/rfc2132#section-9.14)
    ClientId = 61,
    End = 255,
}
impl From<DhcpOption> for u8 {
    fn from(val: DhcpOption) -> u8 {
        val as u8
    }
}

/// DHCP op code (message type)
///
/// From [RFC 2131 Section 2](https://tools.ietf.org/html/rfc2131#section-2)
#[repr(u8)]
pub enum OpCode {
    BOOTREQUEST = 1,
    BOOTREPLY = 2,
}
impl From<OpCode> for u8 {
    fn from(val: OpCode) -> u8 {
        val as u8
    }
}

/// DHCP hardware type.
///
/// See [RFC 1700](https://tools.ietf.org/html/rfc1700)
#[repr(u8)]
#[non_exhaustive]
pub enum HardwareType {
    Ethernet = 1,
    // lots of others that I do not care about
}
impl From<HardwareType> for u8 {
    fn from(val: HardwareType) -> u8 {
        val as u8
    }
}

pub const DHCP_DESTINATION_PORT: u16 = 67;
pub const DHCP_SOURCE_PORT: u16 = 68;
pub const DHCP_DESTINATION: SocketAddrV4 =
    SocketAddrV4::new(Ipv4Addr::BROADCAST, DHCP_DESTINATION_PORT);

const HW_ADDR_LEN: u8 = 6;

// Server name size.
// const SNAME_SIZE: usize = 64;
// Boot filename size
// const FILE_SIZE: usize = 128;
const MAGIC_COOKIE: [u8; 4] = [0x63, 0x82, 0x53, 0x63];

/// Fixed DHCP packet buffer size.
pub const BUF_SIZE: usize = 600;

/// This is a large buffer, so it is declared as a static global.
static mut DHCP_BUF: [u8; BUF_SIZE] = [0; BUF_SIZE];
/// Pointer for filling up the buffer.
static mut DHCP_BUF_PTR: usize = 0;

/// DHCP packet buffer.
pub struct Dhcp {
    buf: &'static mut [u8; BUF_SIZE],
    ptr: &'static mut usize,
}

#[allow(clippy::clippy::len_without_is_empty)]
impl Dhcp {
    /// Grab the statically allocated DHCP buffer.
    ///
    /// # Safety
    ///
    /// Ensure the consumer of the buffer has exclusive access.
    /// The buffer is global mutable state, which is a big no-no in rust.
    ///
    /// # Example
    ///
    /// ```
    /// use dhcp::Dhcp;
    ///
    /// let buf: Dhcp = unsafe { Dhcp::steal() };
    /// ```
    pub unsafe fn steal() -> Dhcp {
        Dhcp {
            buf: &mut DHCP_BUF,
            ptr: &mut DHCP_BUF_PTR,
        }
    }

    /// Zero out the buffer.
    ///
    /// # Example
    ///
    /// ```
    /// use dhcp::Dhcp;
    ///
    /// let mut buf: Dhcp = unsafe { Dhcp::steal() };
    /// buf.zero();
    /// ```
    pub fn zero(&mut self) {
        self.buf.iter_mut().for_each(|x| *x = 0);
        *self.ptr = 0;
    }

    /// Set the message type (op code)
    ///
    /// # Example
    ///
    /// ```
    /// use dhcp::{Dhcp, OpCode};
    ///
    /// let mut buf: Dhcp = unsafe { Dhcp::steal() };
    ///
    /// buf.set_op(OpCode::BOOTREQUEST);
    /// assert!(buf.is_bootrequest());
    /// assert!(!buf.is_bootreply());
    ///
    /// buf.set_op(OpCode::BOOTREPLY);
    /// assert!(!buf.is_bootrequest());
    /// assert!(buf.is_bootreply());
    /// ```
    pub fn set_op(&mut self, op: OpCode) {
        self.buf[0] = op.into()
    }

    /// Returns `true` if the message type is a bootrequest.
    pub fn is_bootrequest(&self) -> bool {
        self.buf[0] == OpCode::BOOTREQUEST.into()
    }

    /// Returns `true` if the message type is a bootreply.
    pub fn is_bootreply(&self) -> bool {
        self.buf[0] == OpCode::BOOTREPLY.into()
    }

    /// Set the hardware type to Ethernet.
    fn set_htype_ethernet(&mut self) {
        self.buf[1] = HardwareType::Ethernet.into()
    }

    /// Returns `true` if the hardware type is Ethernet.
    pub fn is_htype_ethernet(&self) -> bool {
        self.buf[1] == HardwareType::Ethernet.into()
    }

    /// Transaction ID.
    pub fn xid(&self) -> u32 {
        u32::from_be_bytes([self.buf[4], self.buf[5], self.buf[6], self.buf[7]])
    }

    /// Set the transaction ID.
    pub fn set_xid(&mut self, xid: &u32) {
        let bytes: [u8; 4] = xid.to_be_bytes();
        self.buf[4] = bytes[0];
        self.buf[5] = bytes[1];
        self.buf[6] = bytes[2];
        self.buf[7] = bytes[3];
    }

    /// Set the magic cookie.
    ///
    /// Sets the first four octets of the options field to 99, 138, 83, 99.
    ///
    /// From [RFC 2131 Section 3](https://tools.ietf.org/html/rfc2131#section-3)
    fn set_magic_cookie(&mut self) {
        MAGIC_COOKIE
            .iter()
            .enumerate()
            .for_each(|(idx, byte)| self.buf[idx + 236] = *byte)
    }

    /// client IP address;
    /// filled in by client in bootrequest if known.
    #[allow(dead_code)]
    pub fn ciaddr(&self) -> Ipv4Addr {
        Ipv4Addr::new(self.buf[12], self.buf[13], self.buf[14], self.buf[15])
    }

    /// 'your' (client) IP address;
    /// filled by server if client doesn't
    /// know its own address (ciaddr was 0).
    pub fn yiaddr(&self) -> Ipv4Addr {
        Ipv4Addr::new(self.buf[16], self.buf[17], self.buf[18], self.buf[19])
    }

    /// server IP address;
    /// returned in bootreply by server.
    #[allow(dead_code)]
    pub fn siaddr(&self) -> Ipv4Addr {
        Ipv4Addr::new(self.buf[20], self.buf[21], self.buf[22], self.buf[23])
    }

    /// gateway IP address,
    /// used in optional cross-gateway booting.
    #[allow(dead_code)]
    pub fn giaddr(&self) -> Ipv4Addr {
        Ipv4Addr::new(self.buf[24], self.buf[25], self.buf[26], self.buf[27])
    }

    fn find_option_index(&self, option: DhcpOption) -> Option<usize> {
        let option_u8: u8 = option.into();
        debug_assert_ne!(option_u8, 0xFF);
        let mut idx: usize = 240;
        loop {
            if idx >= BUF_SIZE - 1 {
                return None;
            }
            let byte: u8 = self.buf[idx];
            if byte == 0xFF {
                return None;
            } else if byte == 0x00 {
                idx += 1;
            } else if byte == option_u8 {
                return Some(idx);
            } else {
                idx += 2 + usize::from(self.buf[idx + 1]);
            }
        }
    }

    /// Returns the DHCP message type (option 53) if it exists.
    pub fn message_type(&self) -> Option<MsgType> {
        let idx: usize = self.find_option_index(DhcpOption::MessageType)?;
        let size: u8 = self.buf[idx + 1];
        debug_assert_eq!(size, 1);
        match self.buf[idx + 2] {
            1 => Some(MsgType::Discover),
            2 => Some(MsgType::Offer),
            3 => Some(MsgType::Request),
            4 => Some(MsgType::Decline),
            5 => Some(MsgType::Ack),
            6 => Some(MsgType::Nak),
            7 => Some(MsgType::Release),
            8 => Some(MsgType::Inform),
            x => panic!("invalid DHCP message type: {}", x),
        }
    }

    /// Returns the subnet mask (option 1) if it exists.
    pub fn subnet_mask(&self) -> Option<Ipv4Addr> {
        let idx: usize = self.find_option_index(DhcpOption::SubnetMask)?;
        let size: u8 = self.buf[idx + 1];
        debug_assert_eq!(size, 4);
        Some(Ipv4Addr::new(
            self.buf[idx + 2],
            self.buf[idx + 3],
            self.buf[idx + 4],
            self.buf[idx + 5],
        ))
    }

    /// Returns the rebinding time (option 59) if it exists.
    pub fn rebinding_time(&self) -> Option<u32> {
        let idx: usize = self.find_option_index(DhcpOption::RebindingTime)?;
        let size: u8 = self.buf[idx + 1];
        debug_assert_eq!(size, 4);
        Some(u32::from_be_bytes(
            self.buf[idx + 2..idx + 6].try_into().unwrap(),
        ))
    }

    /// Returns the renewal time (option 58) if it exists.
    pub fn renewal_time(&self) -> Option<u32> {
        let idx: usize = self.find_option_index(DhcpOption::RenewalTime)?;
        let size: u8 = self.buf[idx + 1];
        debug_assert_eq!(size, 4);
        Some(u32::from_be_bytes(
            self.buf[idx + 2..idx + 6].try_into().unwrap(),
        ))
    }

    /// Returns the lease time (option 51) if it exists.
    pub fn lease_time(&self) -> Option<u32> {
        let idx: usize = self.find_option_index(DhcpOption::LeaseTime)?;
        let size: u8 = self.buf[idx + 1];
        debug_assert_eq!(size, 4);
        Some(u32::from_be_bytes(
            self.buf[idx + 2..idx + 6].try_into().unwrap(),
        ))
    }

    /// Returns the DHCP server identifier (option 54) if it exists
    pub fn dhcp_server(&self) -> Option<Ipv4Addr> {
        let idx: usize = self.find_option_index(DhcpOption::ServerId)?;
        let size: u8 = self.buf[idx + 1];
        debug_assert_eq!(size, 4);
        Some(Ipv4Addr::new(
            self.buf[idx + 2],
            self.buf[idx + 3],
            self.buf[idx + 4],
            self.buf[idx + 5],
        ))
    }

    /// Prepares the buffer for a new DHCP message.
    ///
    /// From [RFC 2131 Section 2](https://tools.ietf.org/html/rfc2131#section-2)
    fn prepare_message(&mut self, mac: &Eui48Addr, xid: &u32) {
        self.zero();
        self.set_op(OpCode::BOOTREQUEST);
        self.set_htype_ethernet();
        self.buf[2] = 6; // hardware address (EUI-48 MAC) length
        self.buf[3] = 0; // hops
        self.set_xid(xid);
        self.buf[10] = 0x80; // broadcast flag
        self.buf[29] = mac.octets[0];
        self.buf[30] = mac.octets[1];
        self.buf[31] = mac.octets[2];
        self.buf[32] = mac.octets[3];
        self.buf[33] = mac.octets[4];
        self.buf[34] = mac.octets[5];
        self.set_magic_cookie();
        *self.ptr = 240;
    }

    #[inline(always)]
    fn write_buf(&mut self, data: u8) {
        self.buf[*self.ptr] = data;
        *self.ptr += 1
    }

    fn option_message_type(&mut self, msg_type: MsgType) {
        self.write_buf(DhcpOption::MessageType.into());
        self.write_buf(1);
        self.write_buf(msg_type.into())
    }

    fn option_client_identifier(&mut self, mac: &Eui48Addr) {
        self.write_buf(DhcpOption::ClientId.into());
        self.write_buf(HW_ADDR_LEN + 1);
        self.write_buf(HardwareType::Ethernet.into());
        mac.octets.iter().for_each(|o| self.write_buf(*o));
    }

    fn option_hostname(&mut self, hostname: &str) {
        self.write_buf(DhcpOption::Hostname.into());
        self.write_buf(u8::try_from(hostname.len()).unwrap());
        for byte in hostname.as_bytes().iter() {
            self.write_buf(*byte);
        }
    }

    fn option_parameter_request(&mut self) {
        self.write_buf(DhcpOption::ParameterRequest.into());
        self.write_buf(5);
        self.write_buf(DhcpOption::SubnetMask.into());
        self.write_buf(DhcpOption::Router.into());
        self.write_buf(DhcpOption::Dns.into());
        self.write_buf(DhcpOption::RenewalTime.into());
        self.write_buf(DhcpOption::RebindingTime.into());
    }

    fn option_requested_ip(&mut self, ip: &Ipv4Addr) {
        self.write_buf(DhcpOption::RequestedIp.into());
        self.write_buf(4);
        self.write_buf(ip.octets[0]);
        self.write_buf(ip.octets[1]);
        self.write_buf(ip.octets[2]);
        self.write_buf(ip.octets[3]);
    }

    fn option_end(&mut self) {
        self.write_buf(DhcpOption::End.into())
    }

    /// Create a DHCP discover.
    pub fn dhcp_discover(&mut self, mac: &Eui48Addr, hostname: &str, xid: &u32) -> &[u8] {
        self.prepare_message(mac, xid);
        self.option_message_type(MsgType::Discover);
        self.option_client_identifier(mac);
        self.option_hostname(hostname);
        self.option_end();
        &self.buf[..*self.ptr]
    }

    /// Create a DHCP request.
    pub fn dhcp_request(
        &mut self,
        mac: &Eui48Addr,
        ip: &Ipv4Addr,
        hostname: &str,
        xid: &u32,
    ) -> &[u8] {
        self.prepare_message(mac, xid);
        self.option_message_type(MsgType::Request);
        self.option_client_identifier(mac);
        self.option_hostname(hostname);
        self.option_parameter_request();
        self.option_requested_ip(ip);
        self.option_end();
        &self.buf[..*self.ptr]
    }

    pub fn recv(&mut self) -> &mut [u8] {
        self.buf
    }

    /// Maximum length of the DHCP buffer in bytes.
    ///
    /// # Example
    ///
    /// ```
    /// use dhcp::Dhcp;
    ///
    /// let buf: Dhcp = unsafe { Dhcp::steal() };
    /// assert_eq!(buf.len(), 600);
    /// ```
    pub const fn len(&self) -> usize {
        BUF_SIZE
    }
}
