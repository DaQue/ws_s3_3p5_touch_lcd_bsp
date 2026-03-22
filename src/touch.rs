//! CST3240 capacitive touch controller driver.
//!
//! # Hardware
//!
//! The Waveshare ESP32-S3-Touch-LCD-3.5B uses a **CST3240** touch IC integrated
//! into the AXS15231B display module. It communicates over I2C and reports up to
//! 2 simultaneous touch points.
//!
//! ## I2C address
//!
//! The CST3240 on this board uses I2C address **0x3B** — *not* the commonly
//! documented 0x5A. Using 0x5A will fail silently.
//!
//! ## I2C bus speed
//!
//! This controller requires **100 kHz** I2C. At 400 kHz it either doesn't respond
//! or returns garbage.
//!
//! ## Read protocol
//!
//! Touch data is retrieved with a combined write-then-read transaction
//! (`i2c_master_transmit_receive` in ESP-IDF terms):
//!
//! 1. Write the 11-byte [`TOUCH_READ_CMD`] magic sequence
//! 2. Read back 14 bytes of touch data
//!
//! The response format:
//! ```text
//! Byte 0:    status / flags
//! Byte 1:    number of active touch points (0, 1, or 2; 0xBC = idle/no touch)
//! Bytes 2–5: first touch point  — bits[11:0] of bytes 2–3 = X, bits[11:0] of 4–5 = Y
//! Bytes 6–9: second touch point (same layout)
//! Bytes 10–13: unused
//! ```
//!
//! When idle, the controller returns `0xBC` in every byte.
//!
//! # Ghost event filtering
//!
//! The CST3240 occasionally fires a single spurious "touch" event at a fixed
//! coordinate (e.g. (269, 80)) with no physical contact. These ghost events last
//! exactly **one poll cycle**.
//!
//! [`TouchState::poll`] filters them by requiring [`CONFIRM_COUNT`] consecutive
//! polls with the same contact before reporting a press. At a 20 ms poll rate this
//! means a minimum 60 ms of physical contact — well below any real tap.
//!
//! # Coordinate mapping
//!
//! The controller reports coordinates in the native **portrait** panel space
//! (0–319 on X, 0–479 on Y). [`TouchState::poll`] maps these to the current
//! logical orientation automatically.
//!
//! # Usage
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::touch::{TouchState, Gesture};
//! use ws_s3_3p5_bsp::layout::Orientation;
//!
//! let mut touch = TouchState::new();
//! let now_ms: u32 = /* milliseconds since boot */;
//!
//! match touch.poll(&mut i2c, now_ms, Orientation::Landscape) {
//!     Gesture::Tap { x, y } => { /* handle tap */ }
//!     Gesture::SwipeLeft    => { /* previous page */ }
//!     Gesture::SwipeRight   => { /* next page */ }
//!     Gesture::LongPress    => { /* context menu */ }
//!     _                     => {}
//! }
//! ```

use esp_idf_hal::i2c::I2cDriver;
use log::info;
use crate::layout::Orientation;

// ── I2C address and protocol ─────────────────────────────────────────────────

/// I2C address of the CST3240 touch controller on this board.
/// Note: many CST32xx datasheets list 0x5A — this board uses 0x3B.
const TOUCH_ADDR: u8 = 0x3B;

/// 11-byte "wake + request touch data" command sequence.
///
/// Derived from the Waveshare C factory BSP (`bsp_touch.c`). The exact meaning
/// of each byte is not publicly documented by CST, but the sequence must be
/// sent verbatim before each 14-byte read.
const TOUCH_READ_CMD: [u8; 11] = [
    0xb5, 0xab, 0xa5, 0x5a,  // wake/unlock sequence
    0x00, 0x00, 0x00, 0x0e,  // command: read touch data (length hint = 14)
    0x00, 0x00, 0x00,         // padding
];

/// Number of consecutive polls with contact required before confirming a press.
/// Filters ghost events from the CST3240. At 20ms poll rate, 5 polls = 100ms
/// of continuous contact required — ghosts typically last <80ms.
const CONFIRM_COUNT: u8 = 5;

// ── Gesture recognition thresholds ──────────────────────────────────────────

/// Minimum horizontal displacement (px) to classify a gesture as a swipe.
/// Raised from 64 to 96 to prevent ghost-drift false swipes.
const TOUCH_SWIPE_MIN_X_PX: i32 = 96;
/// Maximum vertical displacement (px) allowed during a horizontal swipe.
const TOUCH_SWIPE_MAX_Y_PX: i32 = 80;
/// Minimum vertical displacement (px) to classify a gesture as a vertical swipe.
const TOUCH_SWIPE_MIN_Y_PX: i32 = 48;
/// Minimum time (ms) between swipe events (debounce).
const TOUCH_SWIPE_COOLDOWN_MS: u32 = 300;
/// Minimum time (ms) between tap events (debounce).
const TOUCH_TAP_COOLDOWN_MS: u32 = 500;
/// Maximum finger travel (px) for a touch to be classified as a tap (not a swipe).
const TOUCH_TAP_MAX_MOVE_PX: i32 = 18;
/// Hold duration (ms) required to fire a long press.
const TOUCH_LONG_PRESS_MS: u32 = 600;

// ── Public types ─────────────────────────────────────────────────────────────

/// A gesture decoded from a complete touch press-and-release cycle.
///
/// Gestures are returned from [`TouchState::poll`]. `None` is returned on most
/// polls — a gesture fires at most once per finger contact.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Gesture {
    /// No gesture this poll (finger down, mid-swipe, or no contact).
    None,
    /// Finger tapped at the given logical screen coordinates.
    Tap { x: i16, y: i16 },
    /// Finger held in place for at least [`TOUCH_LONG_PRESS_MS`] ms.
    LongPress,
    /// Finger swiped left (decreasing X).
    SwipeLeft,
    /// Finger swiped right (increasing X).
    SwipeRight,
    /// Finger swiped up (decreasing Y).
    SwipeUp,
    /// Finger swiped down (increasing Y).
    SwipeDown,
}

/// State machine for touch gesture recognition.
///
/// Create one instance and call [`poll`](TouchState::poll) in a tight loop
/// (20–50 ms interval recommended).
pub struct TouchState {
    /// Whether a confirmed press is currently active.
    pressed: bool,
    /// Consecutive polls with a touch reading (for ghost-event filtering).
    confirm_count: u8,
    /// Logical X where the confirmed press started.
    start_x: i16,
    /// Logical Y where the confirmed press started.
    start_y: i16,
    /// Most recently read logical X (updated every poll while pressed).
    last_x: i16,
    /// Most recently read logical Y.
    last_y: i16,
    /// `now_ms` value when the press was confirmed (for long-press timing).
    press_start_ms: u32,
    /// Whether a long-press event has already been fired for this press cycle.
    long_press_fired: bool,
    /// `now_ms` of the last swipe event (for cooldown debounce).
    last_swipe_ms: u32,
    /// `now_ms` of the last tap event (for cooldown debounce).
    last_tap_ms: u32,
    /// Total polls since creation (for periodic stats logging).
    poll_count: u32,
    /// Total I2C read errors since creation.
    err_count: u32,
    /// Total confirmed touch contacts since creation.
    touch_count: u32,
}

impl TouchState {
    /// Create a new zero-initialized touch state.
    pub fn new() -> Self {
        Self {
            pressed: false,
            confirm_count: 0,
            start_x: 0, start_y: 0,
            last_x: 0, last_y: 0,
            press_start_ms: 0,
            long_press_fired: false,
            last_swipe_ms: 0,
            last_tap_ms: 0,
            poll_count: 0,
            err_count: 0,
            touch_count: 0,
        }
    }

    /// Read the touch controller and return any completed gesture.
    ///
    /// Call this once per poll cycle (recommended: every 20 ms).
    ///
    /// # Parameters
    ///
    /// - `i2c` — shared I2C bus driver (100 kHz required)
    /// - `now_ms` — current time in milliseconds since boot (for timing gestures)
    /// - `orientation` — current display orientation (used to map panel coordinates
    ///   to logical screen coordinates)
    ///
    /// # Return value
    ///
    /// Returns [`Gesture::None`] on most polls. A non-None gesture fires at most
    /// once per finger contact:
    /// - [`Gesture::LongPress`] fires while the finger is still held
    /// - All others fire on finger release
    pub fn poll(
        &mut self,
        i2c: &mut I2cDriver<'_>,
        now_ms: u32,
        orientation: Orientation,
    ) -> Gesture {
        self.poll_count += 1;

        // Periodic stats at debug level (every ~1 second at 20ms poll rate)
        #[allow(clippy::manual_is_multiple_of)]
        if self.poll_count % 50 == 0 {
            log::debug!(
                "TOUCH stats: polls={} errs={} touches={}",
                self.poll_count, self.err_count, self.touch_count
            );
        }

        let touch = read_touch(i2c, &mut self.err_count);

        if let Some((px, py)) = touch {
            // Map native portrait coordinates → current logical orientation
            let (x, y) = map_to_orientation(px, py, orientation);
            self.touch_count += 1;

            if !self.pressed {
                // Require CONFIRM_COUNT consecutive polls before accepting as real.
                // Ghost events always last exactly 1 poll.
                self.confirm_count = self.confirm_count.saturating_add(1);
                if self.confirm_count >= CONFIRM_COUNT {
                    self.pressed = true;
                    self.start_x = x;
                    self.start_y = y;
                    self.press_start_ms = now_ms;
                    self.long_press_fired = false;
                    log::debug!("TOUCH down at ({}, {}) (confirmed after {} polls)", x, y, self.confirm_count);
                } else {
                    log::debug!("TOUCH pending confirm ({}/{}) at ({}, {})", self.confirm_count, CONFIRM_COUNT, x, y);
                }
            } else if !self.long_press_fired {
                // Check for long press (finger held without significant movement)
                let dx = (x as i32 - self.start_x as i32).abs();
                let dy = (y as i32 - self.start_y as i32).abs();
                if dx <= TOUCH_TAP_MAX_MOVE_PX
                    && dy <= TOUCH_TAP_MAX_MOVE_PX
                    && now_ms.wrapping_sub(self.press_start_ms) >= TOUCH_LONG_PRESS_MS
                {
                    self.long_press_fired = true;
                    log::debug!("TOUCH -> LongPress");
                    return Gesture::LongPress;
                }
            }
            self.last_x = x;
            self.last_y = y;
            return Gesture::None;
        }

        // No contact this poll — reset confirm counter
        self.confirm_count = 0;

        if !self.pressed {
            return Gesture::None;
        }

        // Finger released — classify the completed gesture
        self.pressed = false;

        if self.long_press_fired {
            // Long press already consumed; suppress tap/swipe on release
            self.long_press_fired = false;
            return Gesture::None;
        }

        let dx = self.last_x as i32 - self.start_x as i32;
        let dy = self.last_y as i32 - self.start_y as i32;
        let abs_dx = dx.abs();
        let abs_dy = dy.abs();

        log::debug!(
            "TOUCH up: start=({},{}) end=({},{}) dx={} dy={}",
            self.start_x, self.start_y, self.last_x, self.last_y, dx, dy
        );

        // Tap: small total movement, subject to cooldown
        if abs_dx <= TOUCH_TAP_MAX_MOVE_PX && abs_dy <= TOUCH_TAP_MAX_MOVE_PX {
            if now_ms.wrapping_sub(self.last_tap_ms) < TOUCH_TAP_COOLDOWN_MS {
                log::debug!("TOUCH -> Tap suppressed (cooldown)");
                return Gesture::None;
            }
            self.last_tap_ms = now_ms;
            log::debug!("TOUCH -> Tap at ({}, {})", self.last_x, self.last_y);
            return Gesture::Tap { x: self.last_x, y: self.last_y };
        }

        // Swipe cooldown (shared across horizontal and vertical)
        if now_ms.wrapping_sub(self.last_swipe_ms) < TOUCH_SWIPE_COOLDOWN_MS {
            return Gesture::None;
        }

        // Vertical swipe: dominant vertical component
        if abs_dy >= TOUCH_SWIPE_MIN_Y_PX && abs_dy >= abs_dx {
            self.last_swipe_ms = now_ms;
            let g = if dy < 0 { Gesture::SwipeUp } else { Gesture::SwipeDown };
            let g = map_swipe_for_orientation(g, orientation);
            log::debug!("TOUCH -> {:?}", g);
            return g;
        }

        // Horizontal swipe: dominant horizontal component
        if abs_dx >= TOUCH_SWIPE_MIN_X_PX && abs_dy <= TOUCH_SWIPE_MAX_Y_PX && abs_dx > abs_dy {
            self.last_swipe_ms = now_ms;
            let g = if dx < 0 { Gesture::SwipeLeft } else { Gesture::SwipeRight };
            let g = map_swipe_for_orientation(g, orientation);
            log::debug!("TOUCH -> {:?}", g);
            return g;
        }

        log::debug!("TOUCH -> unclassified (abs_dx={}, abs_dy={})", abs_dx, abs_dy);
        Gesture::None
    }
}

impl Default for TouchState {
    fn default() -> Self {
        Self::new()
    }
}

// ── Private I2C helpers ──────────────────────────────────────────────────────

/// Read one touch sample from the CST3240.
///
/// Returns `Some((x, y))` in native portrait coordinates (0–319, 0–479),
/// or `None` if there is no active touch or the read failed.
fn read_touch(i2c: &mut I2cDriver<'_>, err_count: &mut u32) -> Option<(i16, i16)> {
    let mut data = [0u8; 14];

    // Combined write (command) + read (response) — matches C factory protocol
    match i2c.write_read(TOUCH_ADDR, &TOUCH_READ_CMD, &mut data, 100) {
        Ok(_) => {}
        Err(_) => {
            *err_count += 1;
            log::debug!("TOUCH read error (total errs={})", *err_count);
            return None;
        }
    }

    // Byte 1 = touch point count. 0xBC means idle (all bytes = 0xBC when no touch).
    let num_points = data[1];
    if num_points == 0 || num_points > 2 {
        return None;
    }

    // First touch point: X in bits[11:0] of bytes 2–3, Y in bits[11:0] of bytes 4–5
    let raw_x = (((data[2] & 0x0F) as u16) << 8) | data[3] as u16;
    let raw_y = (((data[4] & 0x0F) as u16) << 8) | data[5] as u16;

    log::debug!(
        "TOUCH raw: [{:02X} {:02X} {:02X} {:02X} {:02X} {:02X}] points={} x={} y={}",
        data[0], data[1], data[2], data[3], data[4], data[5],
        num_points, raw_x, raw_y
    );

    Some((raw_x as i16, raw_y as i16))
}

// ── Coordinate mapping ───────────────────────────────────────────────────────

/// Map native portrait panel coordinates to the current logical orientation.
///
/// The CST3240 always reports in portrait space (X: 0–319, Y: 0–479).
/// This function rotates/mirrors as needed for each orientation.
fn map_to_orientation(x: i16, y: i16, orientation: Orientation) -> (i16, i16) {
    use crate::layout::{SCREEN_W_PORTRAIT, SCREEN_H_PORTRAIT};
    let px = x.clamp(0, (SCREEN_W_PORTRAIT - 1) as i16);
    let py = y.clamp(0, (SCREEN_H_PORTRAIT - 1) as i16);
    match orientation {
        Orientation::Portrait => (px, py),
        Orientation::PortraitFlipped => (
            (SCREEN_W_PORTRAIT - 1) as i16 - px,
            (SCREEN_H_PORTRAIT - 1) as i16 - py,
        ),
        Orientation::Landscape => {
            // 90° CW: landscape_x = portrait_y, landscape_y = (portrait_w - 1) - portrait_x
            (py, (SCREEN_W_PORTRAIT - 1) as i16 - px)
        }
        Orientation::LandscapeFlipped => {
            // 90° CCW: landscape_x = (portrait_h - 1) - portrait_y, landscape_y = portrait_x
            ((SCREEN_H_PORTRAIT - 1) as i16 - py, px)
        }
    }
}

/// Invert swipe direction for flipped orientations.
///
/// In a flipped orientation, the user perceives left/right and up/down as
/// reversed relative to panel coordinates, so swipe directions must be flipped.
fn map_swipe_for_orientation(g: Gesture, orientation: Orientation) -> Gesture {
    match orientation {
        Orientation::LandscapeFlipped | Orientation::PortraitFlipped => match g {
            Gesture::SwipeLeft  => Gesture::SwipeRight,
            Gesture::SwipeRight => Gesture::SwipeLeft,
            Gesture::SwipeUp    => Gesture::SwipeDown,
            Gesture::SwipeDown  => Gesture::SwipeUp,
            _ => g,
        },
        _ => g,
    }
}

// ── Diagnostics ──────────────────────────────────────────────────────────────

/// One-shot I2C diagnostic: attempt several read strategies and log results.
///
/// Useful during bringup to verify the touch controller is responding.
/// Output goes to the `info` log level.
pub fn probe(i2c: &mut I2cDriver<'_>) {
    info!("=== TOUCH PROBE START ===");

    let mut data = [0u8; 14];
    match i2c.write_read(TOUCH_ADDR, &TOUCH_READ_CMD, &mut data, 100) {
        Ok(_) => info!(
            "Touch write_read OK: [{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}]",
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]
        ),
        Err(e) => info!("Touch write_read FAILED: {:?}", e),
    }

    let mut byte = [0u8; 1];
    match i2c.read(TOUCH_ADDR, &mut byte, 100) {
        Ok(_) => info!("Touch bare read OK: [{:02X}]", byte[0]),
        Err(e) => info!("Touch bare read FAILED: {:?}", e),
    }

    info!("=== TOUCH PROBE END ===");
}
