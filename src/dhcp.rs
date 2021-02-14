//! Do as I say, not as I do.
//! Do not write your own DHCP code.
//!
//! I implemented my own DHCP client for reasons that are not applicable to
//! a production product:
//!
//! 1. Fun
//! 2. Learning
//!
//! This is pretty low-effort code here.
//! Minimum viable product sort of thing.
use core::convert::{TryFrom, TryInto};
use w5500_hl::net::{Eui48Addr, Ipv4Addr, SocketAddrV4};

/// DHCP state.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DhcpState {
    INIT,
    SELECTING,
    REQUESTING,
    INITREBOOT,
    REBOOTING,
    BOUND,
    RENEWING,
    REBINDING,
}

/// DHCP message types (RFC 2132)
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

/// DHCP options (RFC 2132)
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
    RequestedIp = 50,
    LeaseTime = 51,
    MessageType = 53,
    ServerId = 54,
    ParameterRequest = 55,
    RenewalTime = 58,
    RebindingTime = 59,
    ClientId = 61,
    End = 255,
}
impl From<DhcpOption> for u8 {
    fn from(val: DhcpOption) -> u8 {
        val as u8
    }
}

#[repr(u8)]
enum DhcpOp {
    BOOTREQUEST = 1,
    BOOTREPLY = 2,
}
impl From<DhcpOp> for u8 {
    fn from(val: DhcpOp) -> u8 {
        val as u8
    }
}

pub const DHCP_DESTINATION_PORT: u16 = 67;
pub const DHCP_SOURCE_PORT: u16 = 68;
pub const DHCP_DESTINATION: SocketAddrV4 =
    SocketAddrV4::new(Ipv4Addr::BROADCAST, DHCP_DESTINATION_PORT);

const HTYPE10MB: u8 = 1;
/// Server name size.
const SNAME_SIZE: usize = 64;
/// Boot filename size
const FILE_SIZE: usize = 128;
const MAGIC_COOKIE: [u8; 4] = [0x63, 0x82, 0x53, 0x63];

pub const BUF_SIZE: usize = 548;
static mut DHCP_BUF: [u8; BUF_SIZE] = [0; BUF_SIZE];

pub struct DhcpBuf {
    pub buf: [u8; BUF_SIZE],
    buf_ptr: usize,
}

impl DhcpBuf {
    /// Grab the statically allocated DHCP buffer.
    pub unsafe fn steal() -> DhcpBuf {
        DhcpBuf {
            buf: DHCP_BUF,
            buf_ptr: 0,
        }
    }

    /// Zero out the buffer.
    pub fn zero(&mut self) {
        for byte in self.buf.iter_mut() {
            *byte = 0;
        }
        self.buf_ptr = 0;
    }

    pub fn is_bootreply(&self) -> bool {
        self.buf[0] == DhcpOp::BOOTREPLY.into()
    }

    /// Transaction ID.
    pub fn xid(&self) -> u32 {
        u32::from_be_bytes(self.buf[4..8].try_into().unwrap())
    }

    /// Set the transaction ID
    pub fn set_xid(&mut self, xid: u32) {
        let bytes: [u8; 4] = xid.to_be_bytes();
        self.buf[4] = bytes[0];
        self.buf[5] = bytes[1];
        self.buf[6] = bytes[2];
        self.buf[7] = bytes[3];
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

    /// Writes boilerplate for a DHCP pakcet to the W5500.
    fn prepare_message(&mut self, mac: &Eui48Addr) {
        self.zero();
        self.buf[0] = DhcpOp::BOOTREQUEST.into();
        self.buf[1] = HTYPE10MB;
        self.buf[2] = 6; // hardware address (EUI-48 MAC) length
        self.buf[3] = 0; // hops
        self.buf[4] = 0x12; // xid
        self.buf[5] = 0x34; // xid
        self.buf[6] = 0x56; // xid
        self.buf[7] = 0x78; // xid
        self.buf[8] = 0x00; // sec
        self.buf[9] = 0x00; // sec
        self.buf[10] = 0x00; // flags
        self.buf[11] = 0x00; // flags
        for x in 12..29 {
            self.buf[x] = 0x00;
        }
        self.buf[29] = mac.octets[0];
        self.buf[30] = mac.octets[1];
        self.buf[31] = mac.octets[2];
        self.buf[32] = mac.octets[3];
        self.buf[33] = mac.octets[4];
        self.buf[34] = mac.octets[5];

        for x in 0..SNAME_SIZE {
            self.buf[35 + x] = 0x00;
        }
        for x in 0..FILE_SIZE {
            self.buf[108 + x] = 0x00;
        }
        for (idx, byte) in MAGIC_COOKIE.iter().enumerate() {
            self.buf[idx + 236] = *byte;
        }
        self.buf_ptr = 240;
    }

    #[inline(always)]
    fn write_buf(&mut self, data: u8) {
        self.buf[self.buf_ptr] = data;
        self.buf_ptr += 1
    }

    fn option_message_type(&mut self, msg_type: MsgType) {
        self.write_buf(DhcpOption::MessageType.into());
        self.write_buf(1);
        self.write_buf(msg_type.into())
    }

    fn option_client_identifier(&mut self, mac: &Eui48Addr) {
        self.write_buf(DhcpOption::ClientId.into());
        self.write_buf(7);
        self.write_buf(HTYPE10MB);
        self.write_buf(mac.octets[0]);
        self.write_buf(mac.octets[1]);
        self.write_buf(mac.octets[2]);
        self.write_buf(mac.octets[3]);
        self.write_buf(mac.octets[4]);
        self.write_buf(mac.octets[5])
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
    pub fn dhcp_discover(&mut self, mac: &Eui48Addr, hostname: &str) {
        self.prepare_message(mac);
        self.buf[10] = 0x80; // Broadcast
        self.option_message_type(MsgType::Discover);
        self.option_client_identifier(mac);
        self.option_hostname(hostname);
        // self.option_parameter_request();
        self.option_end();
    }

    /// Create a DHCP request.
    pub fn dhcp_request(&mut self, mac: &Eui48Addr, ip: &Ipv4Addr, hostname: &str) {
        self.prepare_message(mac);
        self.buf[10] = 0x80; // Broadcast
        self.option_message_type(MsgType::Request);
        self.option_client_identifier(mac);
        self.option_hostname(hostname);
        self.option_parameter_request();
        self.option_requested_ip(ip);
        self.option_end();
    }

    pub fn tx_data(&self) -> &[u8] {
        &self.buf[..self.buf_ptr]
    }

    pub fn tx_data_len(&self) -> usize {
        self.buf_ptr
    }
}
