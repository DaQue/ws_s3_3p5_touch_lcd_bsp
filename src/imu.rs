//! QMI8658 6-axis IMU driver (accelerometer + gyroscope + temperature).
//!
//! # Hardware
//!
//! The QMI8658 from QST Corporation is a 6-axis inertial measurement unit
//! combining a 3-axis accelerometer and 3-axis gyroscope in a single package.
//! On this board it communicates over I2C at address **0x6B**.
//!
//! Datasheet: <https://www.qstcorp.com/upload/pdf/202202/QMI8658A%20datasheet%20rev%200.9.pdf>
//!
//! # Axis orientation on the Waveshare ESP32-S3-Touch-LCD-3.5B
//!
//! With the board in landscape orientation (display facing up, USB on the right):
//!
//! ```text
//!          USB
//!   ┌───────────┐
//!   │           │
//!   │  display  │
//!   │           │
//!   └───────────┘
//!
//!   X+  →  right (toward USB port)
//!   Y+  ↓  down  (toward bottom edge)
//!   Z+  ↑  out of screen back (positive = face-down on table)
//! ```
//!
//! Gravity vector in typical orientations:
//!
//! | Physical orientation    | Dominant gravity axis |
//! |-------------------------|-----------------------|
//! | Landscape face-up       | Y ≈ −1g               |
//! | Landscape face-down     | Z ≈ +1g               |
//! | Portrait (rotated CW)   | X ≈ +1g               |
//! | Portrait (rotated CCW)  | X ≈ −1g               |
//!
//! # Configuration
//!
//! [`init`] configures the device with:
//! - Accelerometer: **±8g** full scale, 125 Hz output data rate (CTRL2 = 0x24)
//! - Gyroscope: **±512 dps** full scale, 125 Hz output data rate (CTRL3 = 0x54)
//! - Both sensors enabled (CTRL7 = 0x03)
//! - Address auto-increment enabled for burst reads (CTRL1 = 0x60)
//!
//! # Usage
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::imu;
//!
//! if imu::init(&mut i2c) {
//!     if let Some(reading) = imu::read(&mut i2c) {
//!         let tilt_g = reading.accel_y; // ≈ −1.0 when landscape face-up
//!     }
//! }
//! ```

use esp_idf_hal::i2c::I2cDriver;
use log::info;

// ── I2C address ──────────────────────────────────────────────────────────────

/// QMI8658 I2C address. (Pin SDO/SA0 = high → 0x6B; low → 0x6A)
const QMI8658_ADDR: u8 = 0x6B;

// ── Register map ─────────────────────────────────────────────────────────────
// Reference: QMI8658A datasheet Table 3 (Register Map)

/// WHO_AM_I — device identification register. Returns 0x05 on QMI8658.
const REG_WHO_AM_I: u8 = 0x00;

/// CTRL1 — general configuration (SPI/I2C settings, address auto-increment).
const REG_CTRL1: u8 = 0x02;

/// CTRL2 — accelerometer configuration (full scale, output data rate).
const REG_CTRL2: u8 = 0x03;

/// CTRL3 — gyroscope configuration (full scale, output data rate).
const REG_CTRL3: u8 = 0x04;

/// CTRL7 — sensor enable register. Bit 0 = accel enable, bit 1 = gyro enable.
const REG_CTRL7: u8 = 0x08;

/// TEMP_L — temperature data low byte. Burst-read from here covers
/// temp(2) + accel(6) + gyro(6) = 14 bytes at registers 0x33–0x40.
const REG_TEMP_L: u8 = 0x33;

#[allow(dead_code)]
/// ACC_X_L — first accelerometer register (not used directly; burst read via TEMP_L).
const REG_ACC_X_L: u8 = 0x35;

// ── Expected device ID ───────────────────────────────────────────────────────

/// Expected value of WHO_AM_I for a QMI8658(A). Returns 0x05.
const WHO_AM_I_EXPECTED: u8 = 0x05;

// ── Scale factors ────────────────────────────────────────────────────────────

/// Accelerometer full-scale range in g.
/// CTRL2 = 0x24 → ±8g (bits [4:3] = 0b10 → 8g).
const ACCEL_SCALE_G: f32 = 8.0;

/// Gyroscope full-scale range in degrees-per-second.
/// CTRL3 = 0x54 → ±512 dps (bits [4:3] = 0b10 → 512 dps).
const GYRO_SCALE_DPS: f32 = 512.0;

// ── Public types ─────────────────────────────────────────────────────────────

/// One IMU sample containing accelerometer, gyroscope, and die temperature.
///
/// All values are converted to physical units:
/// - Accelerometer axes: g-force (1.0 = 9.81 m/s²)
/// - Gyroscope axes: degrees per second
/// - Temperature: degrees Celsius (die temperature, not ambient)
#[derive(Debug, Clone)]
pub struct ImuReading {
    /// X-axis acceleration in g.
    pub accel_x: f32,
    /// Y-axis acceleration in g.
    pub accel_y: f32,
    /// Z-axis acceleration in g.
    pub accel_z: f32,
    /// X-axis angular rate in degrees/second.
    pub gyro_x: f32,
    /// Y-axis angular rate in degrees/second.
    pub gyro_y: f32,
    /// Z-axis angular rate in degrees/second.
    pub gyro_z: f32,
    /// Die temperature in °C (typ. 25–50°C under load; not an ambient sensor).
    pub temp_c: f32,
}

// ── Public functions ─────────────────────────────────────────────────────────

/// Initialize the QMI8658 IMU.
///
/// Verifies the WHO_AM_I register, then configures the accelerometer (±8g,
/// 125 Hz), gyroscope (±512 dps, 125 Hz), and enables both sensors.
///
/// Returns `true` on success. Returns `false` if the device is not found or
/// any register write fails (check I2C address and wiring).
pub fn init(i2c: &mut I2cDriver<'_>) -> bool {
    // Verify device identity
    let who = match read_reg(i2c, REG_WHO_AM_I) {
        Some(v) => v,
        None => {
            info!("QMI8658: not found at I2C 0x{:02X}", QMI8658_ADDR);
            return false;
        }
    };
    if who != WHO_AM_I_EXPECTED {
        info!("QMI8658: unexpected WHO_AM_I=0x{:02X} (expected 0x{:02X})", who, WHO_AM_I_EXPECTED);
        return false;
    }

    // CTRL1: enable address auto-increment for burst reads (bit 6 = 1), SPI 4-wire (bit 5 = 1)
    if !write_reg(i2c, REG_CTRL1, 0x60) {
        info!("QMI8658: failed to write CTRL1");
        return false;
    }

    // CTRL2: accelerometer — ±8g full scale (bits[4:3]=0b10), 125 Hz ODR (bits[6:5]=0b01)
    if !write_reg(i2c, REG_CTRL2, 0x24) {
        info!("QMI8658: failed to write CTRL2 (accel config)");
        return false;
    }

    // CTRL3: gyroscope — ±512 dps full scale (bits[4:3]=0b10), 125 Hz ODR (bits[6:5]=0b10)
    if !write_reg(i2c, REG_CTRL3, 0x54) {
        info!("QMI8658: failed to write CTRL3 (gyro config)");
        return false;
    }

    // CTRL7: enable accelerometer (bit 0) and gyroscope (bit 1)
    if !write_reg(i2c, REG_CTRL7, 0x03) {
        info!("QMI8658: failed to write CTRL7 (sensor enable)");
        return false;
    }

    info!("QMI8658: initialized (±{}g, ±{}dps, 125Hz)", ACCEL_SCALE_G as i32, GYRO_SCALE_DPS as i32);
    true
}

/// Read one sample from the QMI8658.
///
/// Performs a 14-byte burst read starting at [`REG_TEMP_L`] (0x33), covering:
/// - Bytes 0–1: temperature (little-endian i16, units = 1/256 °C)
/// - Bytes 2–7: accelerometer XYZ (little-endian i16 per axis)
/// - Bytes 8–13: gyroscope XYZ (little-endian i16 per axis)
///
/// Returns `None` if the I2C read fails.
pub fn read(i2c: &mut I2cDriver<'_>) -> Option<ImuReading> {
    let mut buf = [0u8; 14];
    if !read_regs(i2c, REG_TEMP_L, &mut buf) {
        return None;
    }

    let temp_raw = raw_to_i16(buf[0], buf[1]);
    let temp_c = temp_raw as f32 / 256.0;

    let ax_raw = raw_to_i16(buf[2], buf[3]);
    let ay_raw = raw_to_i16(buf[4], buf[5]);
    let az_raw = raw_to_i16(buf[6], buf[7]);
    let gx_raw = raw_to_i16(buf[8], buf[9]);
    let gy_raw = raw_to_i16(buf[10], buf[11]);
    let gz_raw = raw_to_i16(buf[12], buf[13]);

    // Convert raw 16-bit values to physical units
    // Full-scale range maps ±32768 counts to ±SCALE units
    let accel_lsb = 32768.0 / ACCEL_SCALE_G;
    let gyro_lsb  = 32768.0 / GYRO_SCALE_DPS;

    Some(ImuReading {
        accel_x: ax_raw as f32 / accel_lsb,
        accel_y: ay_raw as f32 / accel_lsb,
        accel_z: az_raw as f32 / accel_lsb,
        gyro_x:  gx_raw as f32 / gyro_lsb,
        gyro_y:  gy_raw as f32 / gyro_lsb,
        gyro_z:  gz_raw as f32 / gyro_lsb,
        temp_c,
    })
}

// ── Private I2C helpers ──────────────────────────────────────────────────────

fn write_reg(i2c: &mut I2cDriver<'_>, reg: u8, val: u8) -> bool {
    i2c.write(QMI8658_ADDR, &[reg, val], 100).is_ok()
}

fn read_reg(i2c: &mut I2cDriver<'_>, reg: u8) -> Option<u8> {
    let mut buf = [0u8; 1];
    i2c.write_read(QMI8658_ADDR, &[reg], &mut buf, 100).ok()?;
    Some(buf[0])
}

fn read_regs(i2c: &mut I2cDriver<'_>, reg: u8, buf: &mut [u8]) -> bool {
    i2c.write_read(QMI8658_ADDR, &[reg], buf, 100).is_ok()
}

/// Reconstruct a signed 16-bit integer from two bytes (little-endian).
fn raw_to_i16(low: u8, high: u8) -> i16 {
    (high as i16) << 8 | low as i16
}
