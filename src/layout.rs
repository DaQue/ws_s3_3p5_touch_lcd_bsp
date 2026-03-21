//! Screen layout constants, color palette, orientation type, and drawing helpers.
//!
//! # Physical vs. Logical Coordinates
//!
//! The AXS15231B panel is physically 320×480 (portrait). The firmware runs it in
//! landscape by rendering into a 480×320 framebuffer and rotating 90° CW on flush.
//! All layout constants here use **logical** coordinates — the numbers you draw at.
//!
//! | Mode      | Logical size | Physical panel |
//! |-----------|-------------|----------------|
//! | Landscape | 480 × 320   | 320 × 480 (rotated on flush) |
//! | Portrait  | 320 × 480   | 320 × 480 (direct flush)     |
//!
//! # Color Palette
//!
//! All colors were derived from the Waveshare C factory demo
//! (`drawing_screen_canvas.c`) and converted to Rgb565. The palette uses dark,
//! slightly blue-tinted backgrounds to match an OLED-style aesthetic on the
//! IPS panel.
//!
//! # Usage
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::layout::{self, Orientation, BG_NOW, CARD_FILL_NOW, CARD_BORDER_NOW};
//!
//! let orientation = Orientation::Landscape;
//! let (w, h) = layout::screen_size(orientation);
//! fb.clear_color(BG_NOW);
//! layout::draw_card(&mut fb, 8, 8, w - 16, h - 16, 12, CARD_FILL_NOW, CARD_BORDER_NOW, 1);
//! ```

use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

// ── Color helpers ────────────────────────────────────────────────────────────

/// Convert 8-bit RGB (0–255 per channel) to [`Rgb565`].
///
/// Rgb565 uses 5 bits for R/B and 6 bits for G. This helper shifts correctly:
/// R and B are shifted right 3 (keep top 5 bits), G is shifted right 2 (keep top 6 bits).
pub const fn rgb(r: u8, g: u8, b: u8) -> Rgb565 {
    Rgb565::new(r >> 3, g >> 2, b >> 3)
}

// ── Background colors ────────────────────────────────────────────────────────

/// NOW view background — dark blue-grey.
pub const BG_NOW: Rgb565 = rgb(27, 31, 39);
/// INDOOR sensor view background.
pub const BG_INDOOR: Rgb565 = rgb(22, 28, 38);
/// FORECAST view background (matches NOW).
pub const BG_FORECAST: Rgb565 = rgb(27, 31, 39);
/// I2C scan view background.
pub const BG_I2C: Rgb565 = rgb(27, 31, 39);
/// WiFi scan view background.
pub const BG_WIFI: Rgb565 = rgb(24, 30, 39);
/// About / system info background.
pub const BG_ABOUT: Rgb565 = rgb(22, 28, 38);
/// HVAC stats view background.
pub const BG_HVAC: Rgb565 = rgb(22, 28, 38);
/// Warning / alert full-screen background — dark red.
pub const BG_WARNING: Rgb565 = rgb(48, 8, 8);

// ── Separator and line colors ────────────────────────────────────────────────

/// Primary separator line (header rule, card dividers).
pub const LINE_COLOR_1: Rgb565 = rgb(56, 63, 76);
/// Secondary separator line (lighter, used in graph views).
pub const LINE_COLOR_3: Rgb565 = rgb(58, 70, 84);

// ── Card fill and border colors ──────────────────────────────────────────────

/// NOW view card — dark charcoal fill.
pub const CARD_FILL_NOW: Rgb565 = rgb(20, 25, 35);
/// NOW view card border — medium blue-grey.
pub const CARD_BORDER_NOW: Rgb565 = rgb(63, 75, 95);

/// Forecast preview strip card fill.
pub const CARD_FILL_FORECAST_PREVIEW: Rgb565 = rgb(23, 29, 40);
/// Forecast preview strip card border.
pub const CARD_BORDER_FORECAST_PREVIEW: Rgb565 = rgb(66, 86, 108);

/// Full forecast view card fill.
pub const CARD_FILL_FORECAST: Rgb565 = rgb(24, 29, 39);
/// Full forecast view card border.
pub const CARD_BORDER_FORECAST: Rgb565 = rgb(63, 75, 95);

/// Indoor sensor card fill.
pub const CARD_FILL_INDOOR: Rgb565 = rgb(20, 29, 40);
/// Indoor sensor card border.
pub const CARD_BORDER_INDOOR: Rgb565 = rgb(66, 86, 108);

/// I2C scan result card fill.
pub const CARD_FILL_I2C: Rgb565 = rgb(22, 27, 37);
/// I2C scan result card border.
pub const CARD_BORDER_I2C: Rgb565 = rgb(63, 75, 95);

/// WiFi scan result card fill.
pub const CARD_FILL_WIFI: Rgb565 = rgb(20, 29, 40);
/// WiFi scan result card border.
pub const CARD_BORDER_WIFI: Rgb565 = rgb(66, 86, 108);

// ── Alert / warning card colors ──────────────────────────────────────────────

/// "Silence alert" action button — red.
pub const CARD_FILL_SILENCE: Rgb565 = rgb(180, 40, 40);
pub const CARD_BORDER_SILENCE: Rgb565 = rgb(255, 80, 80);

/// "Alert silenced" confirmation state — green.
pub const CARD_FILL_SILENCED: Rgb565 = rgb(40, 60, 40);
pub const CARD_BORDER_SILENCED: Rgb565 = rgb(80, 140, 80);

// ── Text colors ──────────────────────────────────────────────────────────────

/// Alert title text — bright red for maximum urgency.
pub const TEXT_WARNING_TITLE: Rgb565 = rgb(255, 100, 100);
/// Alert body text — warm off-white for readability on dark red background.
pub const TEXT_WARNING_BODY: Rgb565 = rgb(240, 230, 220);
/// Silence button label — pure white.
pub const TEXT_SILENCE_BUTTON: Rgb565 = rgb(255, 255, 255);

/// Header / title text — near-white.
pub const TEXT_HEADER: Rgb565 = rgb(222, 225, 230);
/// Primary data text (temperature, main readings).
pub const TEXT_PRIMARY: Rgb565 = rgb(232, 235, 240);
/// Secondary data text (feels-like, secondary readings).
pub const TEXT_SECONDARY: Rgb565 = rgb(225, 228, 233);
/// Tertiary text (labels, sub-values).
pub const TEXT_TERTIARY: Rgb565 = rgb(188, 196, 208);
/// Detail text (small stats, list items).
pub const TEXT_DETAIL: Rgb565 = rgb(184, 189, 198);
/// Condition / sky description text — light blue.
pub const TEXT_CONDITION: Rgb565 = rgb(166, 208, 255);
/// Bottom hint / nav hint text — dimmed.
pub const TEXT_BOTTOM: Rgb565 = rgb(140, 148, 160);

// ── Orientation ──────────────────────────────────────────────────────────────

/// Display orientation from the application's logical perspective.
///
/// The physical panel is always 320×480 portrait. Landscape modes are achieved
/// by rotating the framebuffer contents 90° on flush (see [`crate::display`]).
///
/// # Axis orientation on hardware
///
/// In `Landscape` (normal use, USB port on the right side):
/// - X increases left → right (0 = left edge, 479 = right edge)
/// - Y increases top → bottom (0 = top edge, 319 = bottom edge)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    /// 480 × 320 landscape, USB port on right. Standard orientation.
    Landscape,
    /// 480 × 320 landscape, USB port on left (device flipped 180°).
    LandscapeFlipped,
    /// 320 × 480 portrait, USB port on bottom.
    Portrait,
    /// 320 × 480 portrait, USB port on top.
    PortraitFlipped,
}

impl Orientation {
    /// Returns `true` for [`Landscape`](Orientation::Landscape) and
    /// [`LandscapeFlipped`](Orientation::LandscapeFlipped).
    pub fn is_landscape(self) -> bool {
        matches!(self, Orientation::Landscape | Orientation::LandscapeFlipped)
    }

    /// Returns `true` for [`Portrait`](Orientation::Portrait) and
    /// [`PortraitFlipped`](Orientation::PortraitFlipped).
    pub fn is_portrait(self) -> bool {
        matches!(self, Orientation::Portrait | Orientation::PortraitFlipped)
    }
}

// ── Screen dimension constants ───────────────────────────────────────────────

/// Logical screen width in landscape orientation (pixels).
pub const SCREEN_W_LANDSCAPE: i32 = 480;
/// Logical screen height in landscape orientation (pixels).
pub const SCREEN_H_LANDSCAPE: i32 = 320;
/// Logical screen width in portrait orientation (pixels).
pub const SCREEN_W_PORTRAIT: i32 = 320;
/// Logical screen height in portrait orientation (pixels).
pub const SCREEN_H_PORTRAIT: i32 = 480;

/// Y coordinate of the header rule line (separates title bar from content area).
pub const HEADER_LINE_Y: i32 = 30;
/// Standard margin from screen edge to card edge (pixels).
pub const CARD_MARGIN: i32 = 8;
/// Standard card corner radius (pixels).
pub const CARD_RADIUS: i32 = 12;

/// Number of forecast day rows displayed on the Forecast view.
pub const FORECAST_ROWS: usize = 4;
/// Y coordinate of the top of info cards (I2C scan, WiFi scan, About).
pub const INFO_CARD_Y: i32 = 40;

// ── Screen dimension helpers ─────────────────────────────────────────────────

/// Returns the logical screen width for `orientation`.
pub fn screen_w(orientation: Orientation) -> i32 {
    if orientation.is_landscape() { SCREEN_W_LANDSCAPE } else { SCREEN_W_PORTRAIT }
}

/// Returns the logical screen height for `orientation`.
pub fn screen_h(orientation: Orientation) -> i32 {
    if orientation.is_landscape() { SCREEN_H_LANDSCAPE } else { SCREEN_H_PORTRAIT }
}

/// Returns `(width, height)` for `orientation`.
pub fn screen_size(orientation: Orientation) -> (i32, i32) {
    (screen_w(orientation), screen_h(orientation))
}

// ── Drawing helpers ──────────────────────────────────────────────────────────

use crate::display::Framebuffer;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle, RoundedRectangle};

/// Word-wrap `text` into lines of at most `max_chars` Unicode scalar values.
///
/// Wraps at word boundaries; words longer than `max_chars` are hard-split.
/// Newlines in the source text start a new paragraph.
///
/// # Returns
///
/// A `Vec<String>` where each element is one display line. Blank paragraphs
/// become empty strings.
pub fn word_wrap(text: &str, max_chars: usize) -> Vec<String> {
    let mut lines = Vec::new();
    for paragraph in text.split('\n') {
        let paragraph = paragraph.trim();
        if paragraph.is_empty() {
            lines.push(String::new());
            continue;
        }
        let mut line = String::new();
        for word in paragraph.split_whitespace() {
            let word_chars = word.chars().count();
            let line_chars = line.chars().count();
            if line.is_empty() {
                if word_chars > max_chars {
                    // Hard-split oversized words
                    let mut remaining = word;
                    while remaining.chars().count() > max_chars {
                        let end = remaining.char_indices().nth(max_chars).map(|(i, _)| i).unwrap_or(remaining.len());
                        lines.push(remaining[..end].to_string());
                        remaining = &remaining[end..];
                    }
                    line = remaining.to_string();
                } else {
                    line = word.to_string();
                }
            } else if line_chars + 1 + word_chars > max_chars {
                lines.push(line);
                if word_chars > max_chars {
                    let mut remaining = word;
                    while remaining.chars().count() > max_chars {
                        let end = remaining.char_indices().nth(max_chars).map(|(i, _)| i).unwrap_or(remaining.len());
                        lines.push(remaining[..end].to_string());
                        remaining = &remaining[end..];
                    }
                    line = remaining.to_string();
                } else {
                    line = word.to_string();
                }
            } else {
                line.push(' ');
                line.push_str(word);
            }
        }
        if !line.is_empty() {
            lines.push(line);
        }
    }
    lines
}

/// Draw a vertical 1-pixel-wide line from `(x, y1)` to `(x, y2)` inclusive.
pub fn draw_vline(fb: &mut Framebuffer, x: i32, y1: i32, y2: i32, color: Rgb565) {
    let style = PrimitiveStyleBuilder::new().fill_color(color).build();
    Rectangle::new(Point::new(x, y1), Size::new(1, (y2 - y1).max(0) as u32))
        .into_styled(style)
        .draw(fb)
        .ok();
}

/// Draw a horizontal 1-pixel-tall line across the full framebuffer width at `y`.
pub fn draw_hline(fb: &mut Framebuffer, y: i32, color: Rgb565) {
    let style = PrimitiveStyleBuilder::new().fill_color(color).build();
    Rectangle::new(Point::new(0, y), Size::new(fb.size().width, 1))
        .into_styled(style)
        .draw(fb)
        .ok();
}

/// Draw a filled rounded rectangle with an optional border.
///
/// The border is drawn first (as the outer rounded rect), then the inner fill
/// is drawn inset by `border_width` pixels on all sides. If `border_width` is 0
/// only the fill is drawn.
///
/// # Parameters
///
/// - `x`, `y` — top-left corner of the outer bounding box
/// - `w`, `h` — width and height of the outer bounding box
/// - `radius` — corner radius in pixels
/// - `fill` — interior fill color
/// - `border` — border color (unused when `border_width` == 0)
/// - `border_width` — border thickness in pixels
#[allow(clippy::too_many_arguments)]
pub fn draw_card(
    fb: &mut Framebuffer,
    x: i32, y: i32, w: i32, h: i32,
    radius: u32,
    fill: Rgb565,
    border: Rgb565,
    border_width: u32,
) {
    if border_width > 0 {
        // Outer rect in border color
        let style = PrimitiveStyleBuilder::new().fill_color(border).build();
        RoundedRectangle::with_equal_corners(
            Rectangle::new(Point::new(x, y), Size::new(w as u32, h as u32)),
            Size::new(radius, radius),
        )
        .into_styled(style)
        .draw(fb)
        .ok();

        // Inner rect in fill color, inset by border_width
        let bw = border_width as i32;
        let inner_style = PrimitiveStyleBuilder::new().fill_color(fill).build();
        RoundedRectangle::with_equal_corners(
            Rectangle::new(
                Point::new(x + bw, y + bw),
                Size::new((w - 2 * bw) as u32, (h - 2 * bw) as u32),
            ),
            Size::new(
                radius.saturating_sub(border_width),
                radius.saturating_sub(border_width),
            ),
        )
        .into_styled(inner_style)
        .draw(fb)
        .ok();
    } else {
        let style = PrimitiveStyleBuilder::new().fill_color(fill).build();
        RoundedRectangle::with_equal_corners(
            Rectangle::new(Point::new(x, y), Size::new(w as u32, h as u32)),
            Size::new(radius, radius),
        )
        .into_styled(style)
        .draw(fb)
        .ok();
    }
}
