//! Similar to the DHCP code this is low effort code.
//!
//! The same reasons apply for why I implemented my own MQTT client.
//! 1. Fun
//! 2. Learning

use core::fmt::Write;

/// Control packet types.
#[repr(u8)]
#[allow(dead_code)]
pub enum CtrlPacket {
    CONNECT = 1,
    CONNACK = 2,
    PUBLISH = 3,
    PUBACK = 4,
    PUBREC = 5,
    PUBREL = 6,
    PUBCOMP = 7,
    SUBSCRIBE = 8,
    SUBACK = 9,
    UNSUBSCRIBE = 10,
    UNSUBACK = 11,
    PINGREQ = 12,
    PINGRESP = 13,
    DISCONNECT = 14,
}

impl From<CtrlPacket> for u8 {
    fn from(val: CtrlPacket) -> Self {
        val as u8
    }
}

/// Connection return codes.
#[repr(u8)]
#[allow(non_camel_case_types, dead_code)]
pub enum ConnectCode {
    /// Connection Accepted
    ACCEPT = 0,
    /// Connection Refused, unacceptable protocol version
    BAD_PROTO = 1,
    /// Connection Refused, identifier rejected
    BAD_ID = 2,
    /// Connection Refused, Server unavailable
    UNAVALIABLE = 3,
    /// Connection Refused, bad user name or password
    BAD_CREDS = 4,
    /// Connection Refused, not authorized
    NOT_AUTH = 5,
}

impl From<ConnectCode> for u8 {
    fn from(val: ConnectCode) -> Self {
        val as u8
    }
}

const CONNECT_BUF_LEN: usize = 14;
const PROTO_LEN: u16 = 4;
/// MQTT v3.1.1
const PROTO_LEVEL: u8 = 4;
const KEEP_ALIVE: u16 = 3600;

/// Hard coded CONNECT packet.
///
/// Yes I am a monster, but I am an efficient monster.
pub const CONNECT: [u8; CONNECT_BUF_LEN] = [
    (CtrlPacket::CONNECT as u8) << 4,
    (CONNECT_BUF_LEN as u8) - 2, // length of packet after this field
    (PROTO_LEN >> 8) as u8,
    PROTO_LEN as u8,
    b'M',
    b'Q',
    b'T',
    b'T',
    PROTO_LEVEL,
    0x02, // clean session flag (required with no client ID)
    (KEEP_ALIVE >> 8) as u8,
    KEEP_ALIVE as u8,
    0, // no client ID
    0, // no client ID
];

pub const CONNACK_LEN: usize = 4;

/// CONNACK packet.
pub struct Connack {
    pub buf: [u8; CONNACK_LEN],
}

impl Connack {
    #[inline(always)]
    pub const fn new() -> Connack {
        Connack { buf: [0; 4] }
    }

    pub const fn ctrl_pkt_type(&self) -> u8 {
        self.buf[0] >> 4
    }

    pub const fn remaining_len(&self) -> u8 {
        self.buf[1]
    }

    pub const fn rc(&self) -> u8 {
        self.buf[3]
    }
}

// #[repr(u8)]
// #[derive(Debug)]
// pub enum QoS {
//     AtMostOnce = 0b00,
//     AtLeastOnce = 0b01,
//     ExactlyOnce = 0b10,
// }

/// Just a buffer length. Publish packets vary in length.
pub const PUBLISH_LEN: usize = 128;

/// A MQTT publish packet.
///
/// Not very safe, you can hurt yourself in many ways with this.
///
/// # Example
///
/// ```
/// // header auto-fills
/// let publish = Publish::new();
/// // must be done before writing the payload
/// publish.set_topic("/home/sensor_name/temperature");
/// // write payload with format strings
/// publish.write_fmt(format_args!("{.1f}", 24.454651f32));
///
/// // use buffer contents
/// publish.as_slice();
/// ```
pub struct Publish {
    buf: [u8; PUBLISH_LEN],
}

impl Publish {
    pub fn new() -> Publish {
        let mut buf: [u8; PUBLISH_LEN] = [0; PUBLISH_LEN];
        buf[0] = (CtrlPacket::PUBLISH as u8) << 4;
        buf[1] = 2;
        Publish { buf }
    }

    fn inc_len(&mut self) {
        self.buf[1] += 1;
    }

    /// Length of the buffer that we are using.
    ///
    /// Derived from the message length in the packet, plus 2 for the header.
    fn len(&self) -> usize {
        self.buf[1] as usize + 2
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.len()]
    }

    pub fn set_topic(&mut self, topic: &str) {
        self.buf[2] = (topic.len() >> 8) as u8;
        self.buf[3] = topic.len() as u8;
        self.write_str(topic).unwrap();
    }
}

impl core::fmt::Write for Publish {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            self.buf[self.len()] = byte;
            self.inc_len();
        }
        Ok(())
    }
}
