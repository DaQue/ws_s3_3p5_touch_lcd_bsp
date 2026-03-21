//! BME280 environmental sensor driver (temperature, humidity, pressure).
//!
//! # Hardware
//!
//! The Bosch BME280 measures:
//! - Temperature: −40 to +85 °C, ±1.0 °C accuracy
//! - Relative humidity: 0–100 %RH, ±3 %RH accuracy
//! - Barometric pressure: 300–1100 hPa, ±1 hPa accuracy
//!
//! Datasheet: <https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf>
//!
//! # I2C addresses
//!
//! | SDO pin | Address |
//! |---------|---------|
//! | GND     | 0x76 (default) |
//! | VDD     | 0x77 (alternate) |
//!
//! [`Bme280::init`] tries both addresses automatically.
//!
//! # Calibration
//!
//! The BME280 stores factory calibration coefficients in two non-volatile register
//! blocks (0x88–0x9F and 0xE1–0xE7). These are read once at initialization and
//! stored in the [`Bme280`] struct. All subsequent measurements are compensated
//! using the formulas from datasheet Section 4.2.3.
//!
//! # Integer widths
//!
//! The official compensation formulas use `i32` throughout, but the **humidity**
//! formula requires intermediate `i64` values — using `i32` overflows silently and
//! produces wildly wrong readings. This driver uses `i64` for the humidity path.
//!
//! # Averaging
//!
//! [`Bme280::read`] takes 5 samples and returns a trimmed mean (drop the highest
//! and lowest, average the remaining 3). This reduces noise from individual
//! outlier readings without significantly increasing latency.
//!
//! # Operating mode
//!
//! Configured for normal (continuous) mode at 1 Hz standby time, with 1× oversampling
//! on all three channels. Suitable for low-power periodic polling.
//!
//! # Usage
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::bme280::Bme280;
//!
//! if let Some(sensor) = Bme280::init(&mut i2c) {
//!     if let Some(reading) = sensor.read(&mut i2c) {
//!         println!("Temp: {:.1}°F  Humidity: {:.0}%  Pressure: {:.1} hPa",
//!             reading.temperature_f, reading.humidity, reading.pressure_hpa);
//!     }
//! }
//! ```

use esp_idf_hal::i2c::I2cDriver;
use log::{info, warn};

// ── I2C addresses ────────────────────────────────────────────────────────────

/// Default I2C address (SDO pin tied to GND).
const BME280_ADDR: u8 = 0x76;
/// Alternate I2C address (SDO pin tied to VDD).
const BME280_ADDR_ALT: u8 = 0x77;

// ── Register map ─────────────────────────────────────────────────────────────
// Reference: BME280 datasheet Table 18 (Memory Map)

/// Chip ID register — returns 0x60 for BME280 (0x61 = BME680).
const REG_CHIP_ID: u8 = 0xD0;

/// ctrl_hum — humidity oversampling. Must write before ctrl_meas.
/// Bits [2:0]: osrs_h. 0x01 = 1× oversampling.
const REG_CTRL_HUM: u8 = 0xF2;

/// ctrl_meas — temp + pressure oversampling and mode.
/// Bits [7:5]: osrs_t, [4:2]: osrs_p, [1:0]: mode.
/// 0x27 = temp 1×, pressure 1×, normal mode.
const REG_CTRL_MEAS: u8 = 0xF4;

/// config — standby time, IIR filter, SPI 3-wire enable.
/// 0xA0 = 1000 ms standby, no IIR filter.
const REG_CONFIG: u8 = 0xF5;

/// First raw ADC output register. Burst-read 8 bytes from here:
/// press_msb, press_lsb, press_xlsb, temp_msb, temp_lsb, temp_xlsb, hum_msb, hum_lsb.
const REG_PRESS_MSB: u8 = 0xF7;

/// Start of temperature and pressure calibration data block (26 bytes, 0x88–0xA1).
const REG_CALIB_00: u8 = 0x88;

/// Start of humidity calibration data block (7 bytes, 0xE1–0xE7).
const REG_CALIB_26: u8 = 0xE1;

// ── Chip ID ──────────────────────────────────────────────────────────────────

/// Expected value of REG_CHIP_ID for a genuine BME280.
const BME280_CHIP_ID: u8 = 0x60;

// ── Public types ─────────────────────────────────────────────────────────────

/// One compensated reading from the BME280.
#[derive(Debug, Clone)]
pub struct Bme280Reading {
    /// Temperature in degrees Fahrenheit.
    pub temperature_f: f32,
    /// Relative humidity in percent (0.0–100.0).
    pub humidity: f32,
    /// Barometric pressure in hectopascals (hPa / mbar).
    pub pressure_hpa: f32,
}

/// BME280 driver instance. Holds factory calibration data read at init.
///
/// Create via [`Bme280::init`]; call [`read`](Bme280::read) to take measurements.
pub struct Bme280 {
    /// I2C address (0x76 or 0x77).
    addr: u8,

    // Temperature calibration (datasheet Section 4.2.2)
    dig_t1: u16,  // unsigned
    dig_t2: i16,
    dig_t3: i16,

    // Pressure calibration
    dig_p1: u16,  // unsigned
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,

    // Humidity calibration (split across two register blocks)
    dig_h1: u8,   // 0xA1, unsigned
    dig_h2: i16,
    dig_h3: u8,   // unsigned
    dig_h4: i16,  // 12-bit, split across E3/E4
    dig_h5: i16,  // 12-bit, split across E4/E5
    dig_h6: i8,
}

impl Bme280 {
    /// Detect and initialize the BME280.
    ///
    /// Tries I2C addresses 0x76 and 0x77 in order. For the first device found,
    /// reads both calibration blocks and configures normal-mode sampling.
    ///
    /// Returns `None` if no BME280 is found or calibration read fails.
    pub fn init(i2c: &mut I2cDriver<'_>) -> Option<Self> {
        for &addr in &[BME280_ADDR, BME280_ADDR_ALT] {
            let mut chip_id = [0u8];
            if i2c.write_read(addr, &[REG_CHIP_ID], &mut chip_id, 100).is_ok()
                && chip_id[0] == BME280_CHIP_ID
            {
                info!("BME280 found at I2C 0x{:02X}", addr);
                let mut sensor = Self::new_uncalibrated(addr);
                if sensor.read_calibration(i2c).is_ok() {
                    sensor.configure(i2c);
                    return Some(sensor);
                }
                warn!("BME280 at 0x{:02X}: calibration read failed", addr);
            }
        }
        warn!("BME280 not found on I2C bus (tried 0x{:02X} and 0x{:02X})", BME280_ADDR, BME280_ADDR_ALT);
        None
    }

    fn new_uncalibrated(addr: u8) -> Self {
        Self {
            addr,
            dig_t1: 0, dig_t2: 0, dig_t3: 0,
            dig_p1: 0, dig_p2: 0, dig_p3: 0, dig_p4: 0, dig_p5: 0,
            dig_p6: 0, dig_p7: 0, dig_p8: 0, dig_p9: 0,
            dig_h1: 0, dig_h2: 0, dig_h3: 0, dig_h4: 0, dig_h5: 0, dig_h6: 0,
        }
    }

    /// Read factory calibration data from the two NVM register blocks.
    ///
    /// Block 1 (0x88–0xA1): temperature (6 bytes) + pressure (18 bytes) + h1 (1 byte)
    /// Block 2 (0xE1–0xE7): humidity coefficients h2–h6 (7 bytes, non-trivially packed)
    fn read_calibration(&mut self, i2c: &mut I2cDriver<'_>) -> Result<(), esp_idf_sys::EspError> {
        // Block 1: 26 bytes starting at 0x88
        let mut cal1 = [0u8; 26];
        i2c.write_read(self.addr, &[REG_CALIB_00], &mut cal1, 100)?;

        self.dig_t1 = u16::from_le_bytes([cal1[0], cal1[1]]);
        self.dig_t2 = i16::from_le_bytes([cal1[2], cal1[3]]);
        self.dig_t3 = i16::from_le_bytes([cal1[4], cal1[5]]);
        self.dig_p1 = u16::from_le_bytes([cal1[6], cal1[7]]);
        self.dig_p2 = i16::from_le_bytes([cal1[8], cal1[9]]);
        self.dig_p3 = i16::from_le_bytes([cal1[10], cal1[11]]);
        self.dig_p4 = i16::from_le_bytes([cal1[12], cal1[13]]);
        self.dig_p5 = i16::from_le_bytes([cal1[14], cal1[15]]);
        self.dig_p6 = i16::from_le_bytes([cal1[16], cal1[17]]);
        self.dig_p7 = i16::from_le_bytes([cal1[18], cal1[19]]);
        self.dig_p8 = i16::from_le_bytes([cal1[20], cal1[21]]);
        self.dig_p9 = i16::from_le_bytes([cal1[22], cal1[23]]);
        // cal1[24] = unused, cal1[25] = unused; dig_h1 is at 0xA1

        let mut h1 = [0u8];
        i2c.write_read(self.addr, &[0xA1], &mut h1, 100)?;
        self.dig_h1 = h1[0];

        // Block 2: 7 bytes starting at 0xE1 (humidity H2–H6)
        // Packing is non-standard — H4 and H5 share nibbles of register 0xE4.
        let mut cal2 = [0u8; 7];
        i2c.write_read(self.addr, &[REG_CALIB_26], &mut cal2, 100)?;

        self.dig_h2 = i16::from_le_bytes([cal2[0], cal2[1]]);        // 0xE1–0xE2
        self.dig_h3 = cal2[2];                                         // 0xE3
        self.dig_h4 = ((cal2[3] as i16) << 4) | ((cal2[4] as i16) & 0x0F); // 0xE4[7:0] | 0xE4[3:0]
        self.dig_h5 = ((cal2[5] as i16) << 4) | ((cal2[4] as i16) >> 4);   // 0xE5 | 0xE4[7:4]
        self.dig_h6 = cal2[6] as i8;                                   // 0xE6

        info!("BME280 hum cal: H1={} H2={} H3={} H4={} H5={} H6={}",
            self.dig_h1, self.dig_h2, self.dig_h3, self.dig_h4, self.dig_h5, self.dig_h6);

        Ok(())
    }

    /// Configure the BME280 for normal (continuous) 1 Hz sampling.
    ///
    /// Write order matters per datasheet Section 5.4.3:
    /// 1. Sleep mode (ctrl_meas mode=00) before changing settings
    /// 2. config register (standby time, filter)
    /// 3. ctrl_hum (humidity oversampling)
    /// 4. ctrl_meas (temp + pressure oversampling + normal mode) — this activates ctrl_hum
    fn configure(&self, i2c: &mut I2cDriver<'_>) {
        let _ = i2c.write(self.addr, &[REG_CTRL_MEAS, 0x00], 100); // sleep mode
        let _ = i2c.write(self.addr, &[REG_CONFIG,    0xA0], 100); // 1000ms standby, no IIR filter
        let _ = i2c.write(self.addr, &[REG_CTRL_HUM,  0x01], 100); // humidity 1× oversampling
        let _ = i2c.write(self.addr, &[REG_CTRL_MEAS, 0x27], 100); // temp 1×, pressure 1×, normal mode

        // Verify (informational — non-fatal if mismatched)
        let mut check = [0u8];
        if i2c.write_read(self.addr, &[REG_CTRL_HUM], &mut check, 100).is_ok() {
            info!("BME280 ctrl_hum readback: 0x{:02X} (expect 0x01)", check[0]);
        }
        if i2c.write_read(self.addr, &[REG_CTRL_MEAS], &mut check, 100).is_ok() {
            info!("BME280 ctrl_meas readback: 0x{:02X} (expect 0x27)", check[0]);
        }
    }

    /// Take one raw measurement and compensate it.
    ///
    /// Reads 8 bytes starting at [`REG_PRESS_MSB`]:
    /// - Bytes 0–2: pressure ADC (20-bit, right-aligned)
    /// - Bytes 3–5: temperature ADC (20-bit, right-aligned)
    /// - Bytes 6–7: humidity ADC (16-bit)
    fn read_once(&self, i2c: &mut I2cDriver<'_>) -> Option<Bme280Reading> {
        let mut raw = [0u8; 8];
        i2c.write_read(self.addr, &[REG_PRESS_MSB], &mut raw, 100).ok()?;

        // Reconstruct 20-bit ADC values (MSB aligned, shift to get 20 bits)
        let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
        let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
        let adc_h = ((raw[6] as i32) << 8)  |  (raw[7] as i32);

        // Temperature compensation (datasheet Section 4.2.3, formula 1)
        // t_fine is a shared intermediate used by all three compensation formulas
        let var1 = (((adc_t >> 3) - ((self.dig_t1 as i32) << 1)) * (self.dig_t2 as i32)) >> 11;
        let var2 = (((((adc_t >> 4) - (self.dig_t1 as i32)) * ((adc_t >> 4) - (self.dig_t1 as i32))) >> 12) * (self.dig_t3 as i32)) >> 14;
        let t_fine = var1 + var2;
        let temp_c = ((t_fine * 5 + 128) >> 8) as f32 / 100.0;
        let temp_f = temp_c * 9.0 / 5.0 + 32.0;

        let pressure_hpa = self.compensate_pressure(adc_p, t_fine);
        let humidity      = self.compensate_humidity(adc_h, t_fine);

        Some(Bme280Reading { temperature_f: temp_f, humidity, pressure_hpa })
    }

    /// Read with trimmed mean: 5 samples, drop the highest and lowest, average the rest.
    ///
    /// Falls back to a single read if fewer than 3 samples succeed.
    pub fn read(&self, i2c: &mut I2cDriver<'_>) -> Option<Bme280Reading> {
        let mut temps = [0.0f32; 5];
        let mut hums  = [0.0f32; 5];
        let mut press = [0.0f32; 5];
        let mut count = 0u8;

        for _ in 0..5 {
            if let Some(r) = self.read_once(i2c) {
                temps[count as usize] = r.temperature_f;
                hums [count as usize] = r.humidity;
                press[count as usize] = r.pressure_hpa;
                count += 1;
            }
        }

        if count < 3 {
            return self.read_once(i2c); // not enough good reads, return single best-effort
        }

        let n = count as usize;
        temps[..n].sort_unstable_by(|a, b| a.total_cmp(b));
        hums [..n].sort_unstable_by(|a, b| a.total_cmp(b));
        press[..n].sort_unstable_by(|a, b| a.total_cmp(b));

        // Drop min (index 0) and max (index n-1), average the middle
        let avg = |s: &[f32]| s[1..n - 1].iter().sum::<f32>() / (n - 2) as f32;
        Some(Bme280Reading {
            temperature_f: avg(&temps),
            humidity:      avg(&hums),
            pressure_hpa:  avg(&press),
        })
    }

    // ── Compensation formulas ────────────────────────────────────────────────
    // Direct port of the integer formulas from BME280 datasheet Section 4.2.3.
    // The `i64` intermediates in `compensate_pressure` and `compensate_humidity`
    // are intentional — `i32` overflows on production hardware.

    #[allow(clippy::precedence)]
    fn compensate_pressure(&self, adc_p: i32, t_fine: i32) -> f32 {
        let mut var1 = (t_fine as i64) - 128000;
        let mut var2 = var1 * var1 * (self.dig_p6 as i64);
        var2 += (var1 * (self.dig_p5 as i64)) << 17;
        var2 += (self.dig_p4 as i64) << 35;
        var1 = ((var1 * var1 * (self.dig_p3 as i64)) >> 8) + ((var1 * (self.dig_p2 as i64)) << 12);
        var1 = (((1i64 << 47) + var1) * (self.dig_p1 as i64)) >> 33;
        if var1 == 0 {
            return 0.0; // avoid division by zero
        }
        let mut p: i64 = 1048576 - adc_p as i64;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = ((self.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        var2 = ((self.dig_p8 as i64) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + ((self.dig_p7 as i64) << 4);
        // Result is in units of Pa × 256; convert to hPa
        (p as f32) / 25600.0
    }

    #[allow(clippy::precedence)]
    fn compensate_humidity(&self, adc_h: i32, t_fine: i32) -> f32 {
        // BME280 datasheet Section 4.2.3 humidity compensation — requires i64
        let v = (t_fine - 76800) as i64;

        let x1 = ((adc_h as i64) << 14)
            - ((self.dig_h4 as i64) << 20)
            - ((self.dig_h5 as i64) * v);
        let x1 = (x1 + 16384) >> 15;

        let x2 = (v * (self.dig_h6 as i64)) >> 10;
        let x3 = (v * (self.dig_h3 as i64)) >> 11;
        let x2 = ((x2 * (x3 + 32768)) >> 10) + 2097152;
        let x2 = ((x2 * (self.dig_h2 as i64)) + 8192) >> 14;

        let mut var = x1 * x2;
        // Clamp to valid 0–100% RH range before final scaling
        var -= (((var >> 15) * (var >> 15)) >> 7) * (self.dig_h1 as i64) >> 4;
        var = var.clamp(0, 419430400);

        // Result is in units of %RH × 1024; convert to percent
        (var >> 12) as f32 / 1024.0
    }
}
