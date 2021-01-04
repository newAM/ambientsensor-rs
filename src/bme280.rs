//! Bosch BME280 driver.
//!
//! "Humidity sensor measuring relative humidity, barometric pressure, and
//! ambient temperature."
//!
//! This is hard-coded for the specific usecases here (continuous low frequency
//! sampling with the I2C bus).  The code is not very extensible.

use embedded_hal::blocking::i2c;

/// BME280 chip ID.
pub const CHIP_ID: u8 = 0x60;

const NUM_CALIB_REG: usize = 33;
const NUM_MEAS_REG: usize = 8;

#[derive(Debug)]
struct Calibration {
    t1: u16, // 0x88..0x89 buf[00:01]
    t2: i16, // 0x8A..0x8B buf[02:03]
    t3: i16, // 0x8C..0x8D buf[04:05]
    p1: u16, // 0x8E..0x8F buf[06:07]
    p2: i16, // 0x90..0x91 buf[08:09]
    p3: i16, // 0x92..0x93 buf[10:11]
    p4: i16, // 0x94..0x95 buf[12:13]
    p5: i16, // 0x96..0x97 buf[14:15]
    p6: i16, // 0x98..0x99 buf[16:17]
    p7: i16, // 0x9A..0x9B buf[18:19]
    p8: i16, // 0x9C..0x9D buf[20:21]
    p9: i16, // 0x9E..0x9F buf[22:23]
    // INTENTIONAL ONE BYTE GAP (see datasheet)
    h1: u8,  // 0xA1       buf[25]
    h2: i16, // 0xE1..0xE2 buf[26:27]
    h3: u8,  // 0xE3       buf[28]
    h4: i16, // 0xE4..0xE5[3:0] = H4 [11:4]..[3:0]
    h5: i16, // 0xE5[7:4]..0xE6 = H5 [3:0]..[11:4]
    h6: i8,  // 0xE7       buf[32]
}

impl From<[u8; NUM_CALIB_REG]> for Calibration {
    fn from(buf: [u8; NUM_CALIB_REG]) -> Self {
        Calibration {
            t1: u16::from_le_bytes([buf[0], buf[1]]),
            t2: i16::from_le_bytes([buf[2], buf[3]]),
            t3: i16::from_le_bytes([buf[4], buf[5]]),
            p1: u16::from_le_bytes([buf[6], buf[7]]),
            p2: i16::from_le_bytes([buf[8], buf[9]]),
            p3: i16::from_le_bytes([buf[10], buf[11]]),
            p4: i16::from_le_bytes([buf[12], buf[13]]),
            p5: i16::from_le_bytes([buf[14], buf[15]]),
            p6: i16::from_le_bytes([buf[16], buf[17]]),
            p7: i16::from_le_bytes([buf[18], buf[19]]),
            p8: i16::from_le_bytes([buf[20], buf[21]]),
            p9: i16::from_le_bytes([buf[22], buf[23]]),
            // INTENTIONAL ONE BYTE GAP (see datasheet)
            h1: buf[25],
            h2: i16::from_le_bytes([buf[26], buf[27]]),
            h3: buf[28],
            h4: ((buf[29] as i16) << 4 | (buf[30] as i16) & 0xF),
            h5: (((buf[30] as i16) & 0xF0) >> 4) | ((buf[31] as i16) << 4),
            h6: buf[32] as i8,
        }
    }
}

/// Register addresses.
///
/// from Table 18: Memory Map
#[allow(dead_code)]
mod reg {
    pub const HUM_LSB: u8 = 0xFE;
    pub const HUM_MSB: u8 = 0xFB;
    pub const TEMP_XLSB: u8 = 0xFC;
    pub const TEMP_LSB: u8 = 0xFB;
    pub const TEMP_MSB: u8 = 0xFA;
    pub const PRESS_XLSB: u8 = 0xF9;
    pub const PRESS_LSB: u8 = 0xF8;
    pub const PRESS_MSB: u8 = 0xF7;
    pub const CONFIG: u8 = 0xF5;
    pub const CTRL_MEAS: u8 = 0xF4;
    pub const STATUS: u8 = 0xF3;
    pub const CTRL_HUM: u8 = 0xF2;
    pub const CALIB_41: u8 = 0xF0;
    pub const CALIB_40: u8 = 0xEF;
    pub const CALIB_39: u8 = 0xEE;
    pub const CALIB_38: u8 = 0xED;
    pub const CALIB_37: u8 = 0xEC;
    pub const CALIB_36: u8 = 0xEB;
    pub const CALIB_35: u8 = 0xEA;
    pub const CALIB_34: u8 = 0xE9;
    pub const CALIB_33: u8 = 0xE8;
    pub const CALIB_32: u8 = 0xE7;
    pub const CALIB_31: u8 = 0xE6;
    pub const CALIB_30: u8 = 0xE5;
    pub const CALIB_29: u8 = 0xE4;
    pub const CALIB_28: u8 = 0xE3;
    pub const CALIB_27: u8 = 0xE2;
    pub const CALIB_26: u8 = 0xE1;
    pub const RESET: u8 = 0xE0;
    pub const ID: u8 = 0xD0;
    pub const CALIB_25: u8 = 0xA1;
    pub const CALIB_24: u8 = 0xA0;
    pub const CALIB_23: u8 = 0x9F;
    pub const CALIB_22: u8 = 0x9E;
    pub const CALIB_21: u8 = 0x9D;
    pub const CALIB_20: u8 = 0x9C;
    pub const CALIB_19: u8 = 0x9B;
    pub const CALIB_18: u8 = 0x9A;
    pub const CALIB_17: u8 = 0x99;
    pub const CALIB_16: u8 = 0x98;
    pub const CALIB_15: u8 = 0x97;
    pub const CALIB_14: u8 = 0x96;
    pub const CALIB_13: u8 = 0x95;
    pub const CALIB_12: u8 = 0x94;
    pub const CALIB_11: u8 = 0x93;
    pub const CALIB_10: u8 = 0x92;
    pub const CALIB_09: u8 = 0x91;
    pub const CALIB_08: u8 = 0x90;
    pub const CALIB_07: u8 = 0x8F;
    pub const CALIB_06: u8 = 0x8E;
    pub const CALIB_05: u8 = 0x8D;
    pub const CALIB_04: u8 = 0x8C;
    pub const CALIB_03: u8 = 0x8B;
    pub const CALIB_02: u8 = 0x8A;
    pub const CALIB_01: u8 = 0x89;
    pub const CALIB_00: u8 = 0x88;
}

#[derive(Debug)]
pub struct Sample {
    pub temperature: f32,
    pub pressure: f32,
    pub humidity: f32,
}

/// BME280
pub struct BME280<I2C> {
    /// I2C bus.
    i2c: I2C,
    /// I2C device address.
    address: u8,
    /// Calibration data.
    cal: Option<Calibration>,
}

impl<I2C, E> BME280<I2C>
where
    I2C: i2c::Read<Error = E> + i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        BME280 {
            i2c,
            address: 0x76,
            cal: None,
        }
    }

    /// Read from the BME280.
    ///
    /// ```ignore
    /// Read example (BME280 Datasheet Figure 10: I2C multiple byte read)
    /// +-------+---------------+----+------+------------------+------+
    /// | Start | Slave Address | RW | ACKS | Register Address | ACKS |
    /// +-------+---------------+----+------+------------------+------+
    /// | S     | 111011x       |  0 |      | xxxxxxxx         |      |
    /// +-------+---------------+----+------+------------------+------+
    ///
    /// +-------+---------------+----+------+---------------+------+---------------+--------+------+
    /// | Start | Slave Address | RW | ACKS | Register Data | ACKM | Register Data | NOACKM | Stop |
    /// +-------+---------------+----+------+---------------+------+---------------+--------+------+
    /// | S     | 111011x       |  1 |      |      76543210 |      |      76543210 |        | P    |
    /// +-------+---------------+----+------+---------------+------+---------------+--------+------+
    /// ```
    #[inline(always)]
    fn read_register(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), E> {
        self.i2c.write_read(self.address, &[reg], buf)
    }

    /// Write to the BME280.
    ///
    /// ```ignore
    /// Write example (BME280 Datasheet Figure 9: I2C multiple byte write)
    /// +-------+---------------+----+------+------------------+------+---------------+------+
    /// | Start | Slave Address | RW | ACKS | Register Address | ACKS | Register Data | ACKS |
    /// +-------+---------------+----+------+------------------+------+---------------+------+
    /// | S     | 111011x       |  0 |      | xxxxxxxx         |      |      76543210 |      |
    /// +-------+---------------+----+------+------------------+------+---------------+------+
    ///
    ///     +------------------+------+---------------+------+------+
    /// ... | Register Address | ACKS | Register Data | ACKS | Stop |
    ///     +------------------+------+---------------+------+------+
    /// ... | xxxxxxxx         |      |       7654321 |      | P    |
    ///     +------------------+------+---------------+------+------+
    /// ```
    #[inline(always)]
    fn write_register(&mut self, reg: u8, data: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg, data])
    }

    pub fn chip_id(&mut self) -> Result<u8, E> {
        let mut buf: [u8; 1] = [0];
        self.read_register(reg::ID, &mut buf)?;
        Ok(buf[0])
    }

    pub fn init(&mut self) -> Result<(), E> {
        const CONFIG: u8 = 0b1011000;
        const CTRL_HUM: u8 = 0b100;
        const CTRL_MEAS: u8 = 0b10010011;
        self.write_register(reg::CONFIG, CONFIG)?;
        self.write_register(reg::CTRL_HUM, CTRL_HUM)?;
        self.write_register(reg::CTRL_MEAS, CTRL_MEAS)?;
        self.calibrate()
    }

    fn calibrate(&mut self) -> Result<(), E> {
        let first = usize::from(reg::CALIB_25 - reg::CALIB_00 + 1);
        debug_assert_eq!(first, 26);
        let second = usize::from(reg::CALIB_32 - reg::CALIB_26 + 1);
        debug_assert_eq!((first + second), NUM_CALIB_REG);

        let mut buf: [u8; NUM_CALIB_REG] = [0; NUM_CALIB_REG];
        self.read_register(reg::CALIB_00, &mut buf[0..first])?;
        self.read_register(reg::CALIB_26, &mut buf[first..(first + second)])?;
        self.cal = Some(buf.into());
        Ok(())
    }

    /// Read a sample from the BME280.
    ///
    /// This function is magical.
    ///
    /// The magic comes from the datasheet, I am not to blame for this!
    pub fn sample(&mut self) -> Result<Sample, E> {
        let mut buf: [u8; NUM_MEAS_REG] = [0; NUM_MEAS_REG];
        self.read_register(reg::PRESS_MSB, &mut buf)?;

        // msb [7:0] = p[19:12]
        // lsb [7:0] = p[11:4]
        // xlsb[7:4] = p[3:0]
        let p: u32 = ((buf[0] as u32) << 12) | ((buf[1] as u32) << 4) | ((buf[2] as u32) >> 4);
        // msb [7:0] = t[19:12]
        // lsb [7:0] = t[11:4]
        // xlsb[7:4] = t[3:0]
        let t: u32 = ((buf[3] as u32) << 12) | ((buf[4] as u32) << 4) | ((buf[5] as u32) >> 4);
        // msb [7:0] = h[15:8]
        // lsb [7:0] = h[7:0]
        let h: u32 = ((buf[6] as u32) << 8) | (buf[7] as u32);

        // output is held in reset
        debug_assert_ne!(t, 0x80000000);
        debug_assert_ne!(p, 0x80000000);
        debug_assert_ne!(h, 0x8000);

        let p: i32 = p as i32;
        let t: i32 = t as i32;
        let h: i32 = h as i32;

        let cal = self.cal.as_ref().unwrap();

        let var1: i32 = (((t >> 3) - ((cal.t1 as i32) << 1)) * (cal.t2 as i32)) >> 11;
        let var2: i32 = (((((t >> 4) - (cal.t1 as i32)) * ((t >> 4) - (cal.t1 as i32))) >> 12)
            * (cal.t3 as i32))
            >> 14;

        let t_fine: i32 = var1 + var2;

        let temperatue: i32 = (t_fine * 5 + 128) >> 8;
        let temperature: f32 = (temperatue as f32) / 100.0;

        let var1: i64 = (t_fine as i64) - 128000;
        let var2: i64 = var1 * var1 * (cal.p6 as i64);
        let var2: i64 = var2 + ((var1 * (cal.p5 as i64)) << 17);
        let var2: i64 = var2 + ((cal.p4 as i64) << 35);
        let var1: i64 = ((var1 * var1 * (cal.p3 as i64)) >> 8) + ((var1 * (cal.p4 as i64)) << 12);
        let var1: i64 = ((((1i64) << 47) + var1) * (cal.p1 as i64)) >> 33;
        let pressure: f32 = if var1 == 0 {
            0.0
        } else {
            let var3: i64 = 1048576 - (p as i64);
            let var3: i64 = (((var3 << 31) - var2) * 3125) / var1;
            let var1: i64 = ((cal.p9 as i64) * (var3 >> 13) * (var3 >> 13)) >> 25;
            let var2: i64 = ((cal.p8 as i64) * var3) >> 19;

            let var3: i64 = ((var3 + var1 + var2) >> 8) + ((cal.p7 as i64) << 4);
            (var3 as f32) / 256.0
        };

        let var1: i32 = t_fine - 76800i32;
        let var1: i32 =
            ((((h << 14) - ((cal.h4 as i32) << 20) - ((cal.h5 as i32) * var1)) + 16384i32) >> 15)
                * (((((((var1 * (cal.h6 as i32)) >> 10)
                    * (((var1 * (cal.h3 as i32)) >> 11) + (32768i32)))
                    >> 10)
                    + (2097152i32))
                    * (cal.h2 as i32)
                    + 8192)
                    >> 14);
        let var1: i32 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * (cal.h1 as i32)) >> 4);
        let var1: i32 = if var1 < 0 { 0 } else { var1 };
        let var1: i32 = if var1 > 419430400 { 419430400 } else { var1 };
        let humidity: f32 = ((var1 >> 12) as f32) / 1024.0;

        Ok(Sample {
            temperature,
            pressure,
            humidity,
        })
    }
}
