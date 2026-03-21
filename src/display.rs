//! AXS15231B QSPI display driver — framebuffer allocation and panel flush.
//!
//! # Hardware
//!
//! The Waveshare ESP32-S3-Touch-LCD-3.5B uses an **AXS15231B** display controller
//! driving a 320×480 IPS panel over a 4-wire QSPI interface. The panel's native
//! orientation is **portrait** (320 wide × 480 tall).
//!
//! For landscape use, a 480×320 logical framebuffer is rendered into PSRAM and
//! then rotated 90° clockwise during the flush to panel — no hardware rotation
//! register is used.
//!
//! # Framebuffer
//!
//! [`Framebuffer`] allocates its pixel buffer from PSRAM (`MALLOC_CAP_SPIRAM`)
//! and a DMA-capable internal SRAM buffer (`MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL`)
//! for the per-chunk transfer staging area. Both are allocated once and reused.
//!
//! The pixel format is RGB565 (16-bit per pixel, big-endian on the wire).
//! [`Framebuffer`] implements [`embedded_graphics::draw_target::DrawTarget`],
//! so any `embedded-graphics` shape, text, or image can be drawn directly.
//!
//! # Panel flush
//!
//! [`Framebuffer::flush_to_panel`] transfers the framebuffer to the panel in
//! [`CHUNK_LINES`]-row DMA batches. A `vTaskDelay(1)` is inserted every 4 chunks
//! (~every 80 rows) to yield to the FreeRTOS idle task and feed the Task Watchdog.
//! Without this yield, back-to-back flushes can starve the WDT on the second core.
//!
//! # QSPI command encoding
//!
//! The AXS15231B uses a 32-bit SPI command word where:
//! - Bits 31–24 = opcode (`0x02` = write)
//! - Bits 15–8  = display command byte
//!
//! [`send_raset`] sets the row address window (RASET = `0x2B`) before each chunk.
//!
//! # Panel Initialization
//!
//! Getting the AXS15231B panel to display anything requires a non-trivial bringup
//! sequence that is not documented in any public datasheet. The notes below are
//! the result of considerable trial and error — preserve them carefully.
//!
//! ## Custom C component
//!
//! There is **no upstream ESP-IDF driver** for the AXS15231B. You must include a
//! custom C component that provides:
//!
//! ```c
//! esp_err_t esp_lcd_new_panel_axs15231b(
//!     esp_lcd_panel_io_handle_t io,
//!     const esp_lcd_panel_dev_config_t *panel_dev_config,
//!     esp_lcd_panel_handle_t *ret_panel);
//! ```
//!
//! Declare it in Rust via `extern "C"` and link it through your `build.rs`
//! or ESP-IDF component system.
//!
//! ## GPIO pin assignments
//!
//! | Signal | GPIO | Notes                                      |
//! |--------|------|--------------------------------------------|
//! | SCLK   |    5 | SPI clock                                  |
//! | D0     |    1 | MOSI / data line 0                         |
//! | D1     |    2 | MISO / data line 1 (QSPI)                  |
//! | D2     |    3 | QSPI data line 2                           |
//! | D3     |    4 | QSPI data line 3                           |
//! | CS     |   12 | Chip select (active low)                   |
//! | BL     |    6 | Backlight enable — drive **high** to turn on |
//!
//! The D/C (data/command) pin is **not used** (`dc_gpio_num = -1`). Command vs.
//! data selection is encoded in the 32-bit SPI command word instead.
//!
//! ## SPI bus configuration
//!
//! ```text
//! host:             SPI2_HOST
//! DMA channel:      SPI_DMA_CH_AUTO
//! max_transfer_sz:  PANEL_WIDTH × CHUNK_LINES × 2  (= 12 800 bytes)
//! ```
//!
//! ## Panel I/O configuration (`esp_lcd_panel_io_spi_config_t`)
//!
//! Several of these fields are **non-obvious** and the panel will produce garbage
//! or no output if they are wrong:
//!
//! | Field            | Value | Why                                                        |
//! |------------------|-------|------------------------------------------------------------|
//! | `cs_gpio_num`    | 12    | Same as PIN_LCD_CS above                                   |
//! | `dc_gpio_num`    | -1    | No D/C pin — command encoded in 32-bit word                |
//! | `spi_mode`       |  3    | AXS15231B requires CPOL=1, CPHA=1 (mode 3), **not mode 0**|
//! | `pclk_hz`        | 40 MHz| Maximum reliable clock; higher rates cause pixel corruption|
//! | `lcd_cmd_bits`   | 32    | Full 32-bit command word (opcode + cmd, see QSPI encoding) |
//! | `lcd_param_bits` |  8    | Parameter bytes are 8-bit                                  |
//! | `trans_queue_depth` | 10 | Allows pipelining DMA chunks                               |
//!
//! ## Vendor config (`Axs15231bVendorConfig`)
//!
//! ```rust,ignore
//! struct Axs15231bVendorConfig {
//!     init_cmds:      *const Axs15231bLcdInitCmd,
//!     init_cmds_size: u16,
//!     flags:          Axs15231bVendorFlags,   // use_qspi_interface: 1
//! }
//! ```
//!
//! `use_qspi_interface` **must be 1**. Without it the custom driver falls back
//! to a 1-wire SPI path and nothing is displayed.
//!
//! ## Init command table (32 entries)
//!
//! The 32 register-write commands below are byte-for-byte identical to the
//! Waveshare factory C demo. Their register meanings are undocumented — treat
//! this table as a magic constant. Do **not** reorder or omit entries.
//!
//! ```text
//! CMD    DATA (hex bytes)
//! ─────  ──────────────────────────────────────────────────────────────
//! 0xBB   00 00 00 00 00 00 5A A5
//! 0xA0   C0 10 00 02 00 00 04 3F 20 05 3F 3F 00 00 00 00 00
//! 0xA2   30 3C 24 14 D0 20 FF E0 40 19 80 80 80 20 F9 10 02
//!        FF FF F0 90 01 32 A0 91 E0 20 7F FF 00 5A
//! 0xD0   E0 40 51 24 08 05 10 01 20 15 42 C2 22 22 AA 03 10
//!        12 60 14 1E 51 15 00 8A 20 00 03 3A 12
//! 0xA3   A0 06 AA 00 08 02 0A 04 04 04 04 04 04 04 04 04 04
//!        04 04 00 55 55
//! 0xC1   31 04 02 02 71 05 24 55 02 00 41 00 53 FF FF FF 4F
//!        52 00 4F 52 00 45 3B 0B 02 0D 00 FF 40
//! 0xC3   00 00 00 50 03 00 00 00 01 80 01
//! 0xC4   00 24 33 80 00 EA 64 32 C8 64 C8 32 90 90 11 06 DC
//!        FA 00 00 80 FE 10 10 00 0A 0A 44 50
//! 0xC5   18 00 00 03 FE 3A 4A 20 30 10 88 DE 0D 08 0F 0F 01
//!        3A 4A 20 10 10 00
//! 0xC6   05 0A 05 0A 00 E0 2E 0B 12 22 12 22 01 03 00 3F 6A
//!        18 C8 22
//! 0xC7   50 32 28 00 A2 80 8F 00 80 FF 07 11 9C 67 FF 24 0C
//!        0D 0E 0F
//! 0xC9   33 44 44 01
//! 0xCF   2C 1E 88 58 13 18 56 18 1E 68 88 00 65 09 22 C4 0C
//!        77 22 44 AA 55 08 08 12 A0 08
//! 0xD5   40 8E 8D 01 35 04 92 74 04 92 74 04 08 6A 04 46 03
//!        03 03 03 82 01 03 00 E0 51 A1 00 00 00
//! 0xD6   10 32 54 76 98 BA DC FE 93 00 01 83 07 07 00 07 07
//!        00 03 03 03 03 03 03 00 84 00 20 01 00
//! 0xD7   03 01 0B 09 0F 0D 1E 1F 18 1D 1F 19 40 8E 04 00 20
//!        A0 1F
//! 0xD8   02 00 0A 08 0E 0C 1E 1F 18 1D 1F 19
//! 0xD9   1F 1F 1F 1F 1F 1F 1F 1F 1F 1F 1F 1F
//! 0xDD   1F 1F 1F 1F 1F 1F 1F 1F 1F 1F 1F 1F
//! 0xDF   44 73 4B 69 00 0A 02 90
//! 0xE0   3B 28 10 16 0C 06 11 28 5C 21 0D 35 13 2C 33 28 0D
//! 0xE1   37 28 10 16 0B 06 11 28 5C 21 0D 35 14 2C 33 28 0F
//! 0xE2   3B 07 12 18 0E 0D 17 35 44 32 0C 14 14 36 3A 2F 0D
//! 0xE3   37 07 12 18 0E 0D 17 35 44 32 0C 14 14 36 32 2F 0F
//! 0xE4   3B 07 12 18 0E 0D 17 39 44 2E 0C 14 14 36 3A 2F 0D
//! 0xE5   37 07 12 18 0E 0D 17 39 44 2E 0C 14 14 36 3A 2F 0F
//! 0xA4   85 85 95 82 AF AA AA 80 10 30 40 40 20 FF 60 30
//! 0xA4   85 85 95 85
//! 0xBB   00 00 00 00 00 00 00 00
//! 0x13   (no data)
//! 0x11   (no data, 120 ms delay after)
//! 0x2C   00 00 00 00
//! ```
//!
//! `0x13` = Normal Display Mode On (MIPI DCS).
//! `0x11` = Sleep Out (MIPI DCS) — **must be followed by ≥120 ms delay**.
//! `0x2C` = Memory Write (MIPI DCS) — primes the write pipeline.
//!
//! ## Init call sequence
//!
//! ```rust,ignore
//! // 1. Initialize SPI bus
//! spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
//!
//! // 2. Create panel I/O handle over SPI
//! esp_lcd_new_panel_io_spi(host, &io_cfg, &mut io);
//!
//! // 3. Create panel handle using the custom AXS15231B driver
//! //    (passes vendor_config with init_cmds + use_qspi_interface=1)
//! esp_lcd_new_panel_axs15231b(io, &panel_cfg, &mut panel);
//!
//! // 4. Reset and run the init command table (120 ms Sleep Out delay inside)
//! esp_lcd_panel_reset(panel);
//! esp_lcd_panel_init(panel);
//!
//! // 5. NAMING QUIRK: passing `false` here turns the display ON.
//! //    The AXS15231B driver inverts the boolean — `false` = display on.
//! esp_lcd_panel_disp_on_off(panel, false);
//!
//! // 6. Enable backlight (separate GPIO, independent of SPI)
//! gpio_set_level(PIN_LCD_BL /*GPIO 6*/, 1);
//! ```
//!
//! **`disp_on_off(false)` = ON**: The custom AXS15231B panel driver inverts the
//! polarity of this call relative to what the name implies. Passing `true`
//! (display off) will leave the panel dark. Passing `false` (display on) lights
//! it up. This is a driver quirk, not a hardware quirk.
//!
//! # Usage
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::display::{Framebuffer, FB_WIDTH, FB_HEIGHT};
//! use ws_s3_3p5_bsp::layout::{Orientation, BG_NOW};
//!
//! // Create a landscape framebuffer (call once at startup)
//! let mut fb = Framebuffer::new(FB_WIDTH, FB_HEIGHT);
//!
//! // Draw with embedded-graphics
//! fb.clear_color(BG_NOW);
//!
//! // Flush to panel (io and panel handles come from esp_lcd_new_panel_io_spi)
//! fb.flush_to_panel(io, panel, Orientation::Landscape);
//! ```

use embedded_graphics::{
    draw_target::DrawTarget,
    geometry::{Dimensions, OriginDimensions, Size},
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
    Pixel,
};

// ── Panel geometry ───────────────────────────────────────────────────────────

/// Logical framebuffer width in landscape orientation (pixels).
pub const FB_WIDTH: u32 = 480;
/// Logical framebuffer height in landscape orientation (pixels).
pub const FB_HEIGHT: u32 = 320;

/// Physical panel pixel width (native portrait).
pub const PANEL_WIDTH: u32 = 320;
/// Physical panel pixel height (native portrait).
pub const PANEL_HEIGHT: u32 = 480;

/// Number of panel rows transferred per DMA chunk.
///
/// Each chunk covers `PANEL_WIDTH × CHUNK_LINES × 2` bytes.
/// At 320 × 20 × 2 = 12 800 bytes per chunk, fitting comfortably in the
/// DMA-capable SRAM allocation.
pub const CHUNK_LINES: i32 = 20;

// ── AXS15231B / SPI command encoding ────────────────────────────────────────

/// SPI write opcode, placed in bits 31–24 of the 32-bit command word.
const LCD_OPCODE_WRITE_CMD: u32 = 0x02;

/// RASET (Row Address Set) — sets the start and end row for the next bitmap write.
/// Standard MIPI DCS command 0x2B.
const LCD_CMD_RASET: u32 = 0x2B;

// ── Framebuffer ──────────────────────────────────────────────────────────────

/// RGB565 framebuffer backed by PSRAM, with integrated panel flush.
///
/// # Memory layout
///
/// | Buffer      | Location                       | Size                        |
/// |-------------|--------------------------------|-----------------------------|
/// | Pixel data  | PSRAM (`MALLOC_CAP_SPIRAM`)    | `width × height × 2` bytes  |
/// | DMA staging | Internal SRAM (`MALLOC_CAP_DMA`) | `PANEL_WIDTH × CHUNK_LINES × 2` bytes |
///
/// Both buffers are allocated in [`Framebuffer::new`] and freed in [`Drop`].
///
/// # Thread safety
///
/// `Framebuffer` is `Send` but not `Sync`. Move it to the render thread at
/// startup; do not share across threads.
pub struct Framebuffer {
    /// Pointer to the PSRAM pixel buffer (width × height × 2 bytes, RGB565).
    buf: *mut u16,
    /// Total number of pixels (not bytes).
    len: usize,
    /// Logical width in pixels.
    width: u32,
    /// Logical height in pixels.
    height: u32,
    /// Pointer to the internal-SRAM DMA staging buffer.
    dma_buf: *mut u8,
    /// Size of `dma_buf` in bytes.
    dma_bytes: usize,
}

impl Framebuffer {
    /// Allocate a new framebuffer of `width × height` pixels.
    ///
    /// Panics if either PSRAM or DMA buffer allocation fails.
    ///
    /// Typical call:
    /// - Landscape: `Framebuffer::new(FB_WIDTH, FB_HEIGHT)` → 480 × 320
    /// - Portrait:  `Framebuffer::new(PANEL_WIDTH, PANEL_HEIGHT)` → 320 × 480
    pub fn new(width: u32, height: u32) -> Self {
        let pixels = (width * height) as usize;
        let bytes = pixels * core::mem::size_of::<u16>();

        let ptr = unsafe {
            esp_idf_sys::heap_caps_malloc(bytes, esp_idf_sys::MALLOC_CAP_SPIRAM) as *mut u16
        };
        assert!(!ptr.is_null(), "PSRAM framebuffer alloc failed ({} bytes)", bytes);
        // Zero-initialize (black screen)
        unsafe { core::ptr::write_bytes(ptr, 0, pixels); }

        // DMA staging buffer: one chunk at a time, internal SRAM
        let dma_pixels = (PANEL_WIDTH as usize) * (CHUNK_LINES as usize);
        let dma_bytes = dma_pixels * 2;
        let dma_buf = unsafe {
            esp_idf_sys::heap_caps_malloc(
                dma_bytes,
                esp_idf_sys::MALLOC_CAP_DMA
                    | esp_idf_sys::MALLOC_CAP_INTERNAL
                    | esp_idf_sys::MALLOC_CAP_8BIT,
            ) as *mut u8
        };
        assert!(!dma_buf.is_null(), "DMA buffer alloc failed ({} bytes)", dma_bytes);

        Self { buf: ptr, len: pixels, width, height, dma_buf, dma_bytes }
    }

    fn as_slice(&self) -> &[u16] {
        unsafe { core::slice::from_raw_parts(self.buf, self.len) }
    }

    fn as_mut_slice(&mut self) -> &mut [u16] {
        unsafe { core::slice::from_raw_parts_mut(self.buf, self.len) }
    }

    /// Fill the entire framebuffer with a single color.
    pub fn clear_color(&mut self, color: Rgb565) {
        let raw = RawU16::from(color).into_inner();
        self.as_mut_slice().fill(raw);
    }

    /// Flush the framebuffer contents to the physical panel.
    ///
    /// Selects the correct rotation/mirroring path based on `orientation`:
    ///
    /// | Orientation         | Transform applied on flush      |
    /// |---------------------|---------------------------------|
    /// | `Landscape`         | 90° CW rotation                 |
    /// | `LandscapeFlipped`  | 90° CCW rotation (= 270° CW)    |
    /// | `Portrait`          | Direct copy (no rotation)       |
    /// | `PortraitFlipped`   | 180° rotation                   |
    ///
    /// The framebuffer dimensions must match the orientation:
    /// - Landscape → 480 × 320
    /// - Portrait  → 320 × 480
    ///
    /// # Watchdog note
    ///
    /// A `vTaskDelay(1)` is called every 4 DMA chunks to yield to the idle task
    /// and prevent Task Watchdog timeouts during long flushes.
    pub fn flush_to_panel(
        &self,
        io: esp_idf_sys::esp_lcd_panel_io_handle_t,
        panel: esp_idf_sys::esp_lcd_panel_handle_t,
        orientation: crate::layout::Orientation,
    ) {
        let (need_w, need_h) = if orientation.is_landscape() { (480, 320) } else { (320, 480) };
        assert!(
            self.width == need_w && self.height == need_h,
            "framebuffer {}x{} does not match orientation {:?} (expected {}x{})",
            self.width, self.height, orientation, need_w, need_h
        );
        match orientation {
            crate::layout::Orientation::Landscape         => self.flush_landscape_rotated(io, panel),
            crate::layout::Orientation::LandscapeFlipped  => self.flush_landscape_rotated_flipped(io, panel),
            crate::layout::Orientation::Portrait          => self.flush_portrait_direct(io, panel),
            crate::layout::Orientation::PortraitFlipped   => self.flush_portrait_direct_flipped(io, panel),
        }
    }

    // ── Private flush paths ──────────────────────────────────────────────────

    /// Landscape flush: rotate 90° CW.
    ///
    /// Mapping: `panel(px, py) ← fb(py, fb_h - 1 - px)`
    fn flush_landscape_rotated(
        &self,
        io: esp_idf_sys::esp_lcd_panel_io_handle_t,
        panel: esp_idf_sys::esp_lcd_panel_handle_t,
    ) {
        let dma = unsafe { core::slice::from_raw_parts_mut(self.dma_buf, self.dma_bytes) };
        let fb = self.as_slice();
        let fb_w = self.width as usize;
        let fb_h = self.height as usize;
        let pw = PANEL_WIDTH as i32;
        let ph = PANEL_HEIGHT as i32;

        let mut py = 0i32;
        let mut chunk_n = 0i32;
        while py < ph {
            let py_end = (py + CHUNK_LINES).min(ph);
            let mut di = 0usize;
            for row in py..py_end {
                for px in 0..pw {
                    let lx = row as usize;           // landscape x = panel row
                    let ly = fb_h - 1 - px as usize; // landscape y = inverted panel col
                    let pixel = fb[ly * fb_w + lx];
                    // RGB565 is big-endian on the wire
                    dma[di]     = (pixel >> 8) as u8;
                    dma[di + 1] = (pixel & 0xFF) as u8;
                    di += 2;
                }
            }
            send_raset(io, py, py_end);
            unsafe {
                esp_idf_sys::esp_lcd_panel_draw_bitmap(panel, 0, py, pw, py_end, dma.as_ptr().cast());
                chunk_n += 1;
                if chunk_n % 4 == 0 {
                    esp_idf_sys::vTaskDelay(1); // yield to idle / WDT
                }
            }
            py = py_end;
        }
    }

    /// LandscapeFlipped flush: rotate 90° CCW (= 270° CW).
    ///
    /// Mapping: `panel(px, py) ← fb(fb_w - 1 - py, px)`
    fn flush_landscape_rotated_flipped(
        &self,
        io: esp_idf_sys::esp_lcd_panel_io_handle_t,
        panel: esp_idf_sys::esp_lcd_panel_handle_t,
    ) {
        let dma = unsafe { core::slice::from_raw_parts_mut(self.dma_buf, self.dma_bytes) };
        let fb = self.as_slice();
        let fb_w = self.width as usize;
        let pw = PANEL_WIDTH as i32;
        let ph = PANEL_HEIGHT as i32;

        let mut py = 0i32;
        let mut chunk_n = 0i32;
        while py < ph {
            let py_end = (py + CHUNK_LINES).min(ph);
            let mut di = 0usize;
            for row in py..py_end {
                for px in 0..pw {
                    let lx = fb_w - 1 - row as usize; // inverted landscape x
                    let ly = px as usize;              // landscape y = panel col
                    let pixel = fb[ly * fb_w + lx];
                    dma[di]     = (pixel >> 8) as u8;
                    dma[di + 1] = (pixel & 0xFF) as u8;
                    di += 2;
                }
            }
            send_raset(io, py, py_end);
            unsafe {
                esp_idf_sys::esp_lcd_panel_draw_bitmap(panel, 0, py, pw, py_end, dma.as_ptr().cast());
                chunk_n += 1;
                if chunk_n % 4 == 0 {
                    esp_idf_sys::vTaskDelay(1);
                }
            }
            py = py_end;
        }
    }

    /// Portrait flush: direct copy, no rotation.
    fn flush_portrait_direct(
        &self,
        io: esp_idf_sys::esp_lcd_panel_io_handle_t,
        panel: esp_idf_sys::esp_lcd_panel_handle_t,
    ) {
        let dma = unsafe { core::slice::from_raw_parts_mut(self.dma_buf, self.dma_bytes) };
        let fb = self.as_slice();
        let pw = PANEL_WIDTH as i32;
        let ph = PANEL_HEIGHT as i32;
        let fb_w = self.width as usize;

        let mut py = 0i32;
        let mut chunk_n = 0i32;
        while py < ph {
            let py_end = (py + CHUNK_LINES).min(ph);
            let mut di = 0usize;
            for row in py..py_end {
                let row_start = row as usize * fb_w;
                for px in 0..pw {
                    let pixel = fb[row_start + px as usize];
                    dma[di]     = (pixel >> 8) as u8;
                    dma[di + 1] = (pixel & 0xFF) as u8;
                    di += 2;
                }
            }
            send_raset(io, py, py_end);
            unsafe {
                esp_idf_sys::esp_lcd_panel_draw_bitmap(panel, 0, py, pw, py_end, dma.as_ptr().cast());
                chunk_n += 1;
                if chunk_n % 4 == 0 {
                    esp_idf_sys::vTaskDelay(1);
                }
            }
            py = py_end;
        }
    }

    /// PortraitFlipped flush: 180° rotation.
    fn flush_portrait_direct_flipped(
        &self,
        io: esp_idf_sys::esp_lcd_panel_io_handle_t,
        panel: esp_idf_sys::esp_lcd_panel_handle_t,
    ) {
        let dma = unsafe { core::slice::from_raw_parts_mut(self.dma_buf, self.dma_bytes) };
        let fb = self.as_slice();
        let pw = PANEL_WIDTH as i32;
        let ph = PANEL_HEIGHT as i32;
        let fb_w = self.width as usize;

        let mut py = 0i32;
        let mut chunk_n = 0i32;
        while py < ph {
            let py_end = (py + CHUNK_LINES).min(ph);
            let mut di = 0usize;
            for row in py..py_end {
                let src_y = (ph - 1 - row) as usize; // flip vertically
                let row_start = src_y * fb_w;
                for px in 0..pw {
                    let src_x = (pw - 1 - px) as usize; // flip horizontally
                    let pixel = fb[row_start + src_x];
                    dma[di]     = (pixel >> 8) as u8;
                    dma[di + 1] = (pixel & 0xFF) as u8;
                    di += 2;
                }
            }
            send_raset(io, py, py_end);
            unsafe {
                esp_idf_sys::esp_lcd_panel_draw_bitmap(panel, 0, py, pw, py_end, dma.as_ptr().cast());
                chunk_n += 1;
                if chunk_n % 4 == 0 {
                    esp_idf_sys::vTaskDelay(1);
                }
            }
            py = py_end;
        }
    }
}

// ── embedded-graphics trait impls ────────────────────────────────────────────

impl OriginDimensions for Framebuffer {
    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

impl DrawTarget for Framebuffer {
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let w = self.width;
        let h = self.height;
        let buf = self.as_mut_slice();
        for Pixel(point, color) in pixels {
            let x = point.x;
            let y = point.y;
            if x >= 0 && y >= 0 && (x as u32) < w && (y as u32) < h {
                buf[(y as u32 * w + x as u32) as usize] = RawU16::from(color).into_inner();
            }
        }
        Ok(())
    }

    /// Accelerated rectangle fill — bypasses the pixel iterator for solid fills.
    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let raw = RawU16::from(color).into_inner();
        let display = self.bounding_box();
        let area = area.intersection(&display);
        let w = self.width;
        let buf = self.as_mut_slice();
        for y in area.rows() {
            let row_start = (y as u32 * w) as usize;
            for x in area.columns() {
                buf[row_start + x as usize] = raw;
            }
        }
        Ok(())
    }
}

// Safety: the raw PSRAM and DMA pointers are owned exclusively by the render
// thread once `Framebuffer` is moved there. No other thread accesses them.
unsafe impl Send for Framebuffer {}

impl Drop for Framebuffer {
    fn drop(&mut self) {
        unsafe {
            esp_idf_sys::heap_caps_free(self.buf.cast());
            esp_idf_sys::heap_caps_free(self.dma_buf.cast());
        }
    }
}

// ── QSPI helpers ─────────────────────────────────────────────────────────────

/// Encode a display command byte into the 32-bit QSPI command word.
///
/// Format: `[opcode(8)] [0(8)] [cmd(8)] [0(8)]`
/// The AXS15231B expects the command in bits 15–8 with the write opcode in 31–24.
fn qspi_cmd(raw: u32) -> i32 {
    ((LCD_OPCODE_WRITE_CMD << 24) | ((raw & 0xFF) << 8)) as i32
}

/// Send RASET (row address set) to define the row window for the next bitmap.
///
/// `y_start` is inclusive, `y_end` is exclusive (end-1 is written to the panel).
fn send_raset(io: esp_idf_sys::esp_lcd_panel_io_handle_t, y_start: i32, y_end: i32) {
    let y_end_incl = y_end - 1;
    let params: [u8; 4] = [
        ((y_start >> 8) & 0xFF) as u8,
        (y_start & 0xFF) as u8,
        ((y_end_incl >> 8) & 0xFF) as u8,
        (y_end_incl & 0xFF) as u8,
    ];
    unsafe {
        esp_idf_sys::esp_lcd_panel_io_tx_param(
            io,
            qspi_cmd(LCD_CMD_RASET),
            params.as_ptr().cast(),
            params.len(),
        );
    }
}
