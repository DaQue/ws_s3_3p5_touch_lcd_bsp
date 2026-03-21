//! Board Support Package for the **Waveshare ESP32-S3-Touch-LCD-3.5B**.
//!
//! This crate provides driver modules for every on-board peripheral:
//!
//! | Module          | Hardware                           | Purpose                        |
//! |-----------------|------------------------------------|--------------------------------|
//! | [`display`]     | AXS15231B QSPI                     | Framebuffer + panel flush      |
//! | [`touch`]       | CST3240 at I2C 0x3B                | Capacitive touch + gestures    |
//! | [`layout`]      | —                                  | Colors, constants, draw helpers|
//! | [`imu`]         | QMI8658 at I2C 0x6B                | 6-axis accelerometer/gyroscope |
//! | [`bme280`]      | BME280 at I2C 0x76/0x77            | Temp / humidity / pressure     |
//! | [`speaker`]     | ES8311 codec + TCA9554 at I2C 0x18/0x20 + I2S0 | Alert tones   |
//!
//! # Hardware notes
//!
//! - **I2C bus**: All I2C devices share one bus. Use **100 kHz** — the CST3240
//!   touch controller does not work reliably at 400 kHz.
//! - **Display**: 320 × 480 native portrait, operated in software-rotated landscape
//!   (480 × 320) by default. See [`display`] for the rotation details.
//! - **PSRAM**: The framebuffer is allocated in PSRAM (`MALLOC_CAP_SPIRAM`).
//!   PSRAM must be enabled in `sdkconfig` (`CONFIG_ESP32S3_SPIRAM_SUPPORT=y`,
//!   `CONFIG_SPIRAM_MODE_OCT=y`, `CONFIG_SPIRAM_USE_CAPS_ALLOC=y`).
//!
//! # Quick start
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::{Framebuffer, TouchState, Gesture, Orientation};
//! use ws_s3_3p5_bsp::display::{FB_WIDTH, FB_HEIGHT};
//! use ws_s3_3p5_bsp::layout::BG_NOW;
//! use ws_s3_3p5_bsp::bme280::Bme280;
//! use ws_s3_3p5_bsp::imu;
//! use ws_s3_3p5_bsp::speaker::{Speaker, AlertTone, init_audio_path};
//!
//! // Display
//! let mut fb = Framebuffer::new(FB_WIDTH, FB_HEIGHT);
//! fb.clear_color(BG_NOW);
//! // ... draw with embedded-graphics ...
//! fb.flush_to_panel(io, panel, Orientation::Landscape);
//!
//! // Touch
//! let mut touch = TouchState::new();
//! if let Gesture::Tap { x, y } = touch.poll(&mut i2c, now_ms, Orientation::Landscape) {
//!     // handle tap at (x, y)
//! }
//!
//! // Environmental sensor
//! if let Some(sensor) = Bme280::init(&mut i2c) {
//!     if let Some(r) = sensor.read(&mut i2c) {
//!         // r.temperature_f, r.humidity, r.pressure_hpa
//!     }
//! }
//!
//! // IMU
//! if imu::init(&mut i2c) {
//!     if let Some(r) = imu::read(&mut i2c) {
//!         // r.accel_y ≈ -1.0 in normal landscape orientation
//!     }
//! }
//!
//! // Speaker
//! init_audio_path(&mut i2c).unwrap();
//! let mut spk = Speaker::new(i2s0, bclk, dout, ws, Some(mclk)).unwrap();
//! spk.play(AlertTone::Warning, || false).ok();
//! ```

pub mod display;
pub mod touch;
pub mod layout;
pub mod imu;
pub mod bme280;
pub mod speaker;

// Convenient top-level re-exports for the most commonly used types
pub use display::Framebuffer;
pub use touch::{TouchState, Gesture};
pub use layout::Orientation;
pub use imu::ImuReading;
pub use bme280::{Bme280, Bme280Reading};
pub use speaker::{Speaker, AlertTone};
