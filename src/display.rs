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
// ── AXS15231B panel initialisation ───────────────────────────────────────────
//
// GPIO pin assignments
const PIN_LCD_SCLK: i32 = 5;
const PIN_LCD_D0:   i32 = 1;
const PIN_LCD_D1:   i32 = 2;
const PIN_LCD_D2:   i32 = 3;
const PIN_LCD_D3:   i32 = 4;
const PIN_LCD_CS:   i32 = 12;
const PIN_LCD_BL:   i32 = 6;
const PCLK_HZ: u32 = 40_000_000;

#[repr(C)]
struct Axs15231bLcdInitCmd {
    cmd: i32,
    data: *const core::ffi::c_void,
    data_bytes: usize,
    delay_ms: u32,
}
unsafe impl Sync for Axs15231bLcdInitCmd {}

#[repr(C)]
struct Axs15231bVendorFlags { use_qspi_interface: u32 }

#[repr(C)]
struct Axs15231bVendorConfig {
    init_cmds: *const Axs15231bLcdInitCmd,
    init_cmds_size: u16,
    flags: Axs15231bVendorFlags,
}

extern "C" {
    fn esp_lcd_new_panel_axs15231b(
        io: esp_idf_sys::esp_lcd_panel_io_handle_t,
        panel_dev_config: *const esp_idf_sys::esp_lcd_panel_dev_config_t,
        ret_panel: *mut esp_idf_sys::esp_lcd_panel_handle_t,
    ) -> esp_idf_sys::esp_err_t;
    /// Board-level power sequencing — provided by the board_power C component.
    pub fn board_power_init() -> esp_idf_sys::esp_err_t;
}

// Init command data — byte-for-byte identical to C factory BSP. Do not reorder.
static LCD_INIT_CMD_00: [u8; 8]  = [0x00,0x00,0x00,0x00,0x00,0x00,0x5A,0xA5];
static LCD_INIT_CMD_01: [u8; 17] = [0xC0,0x10,0x00,0x02,0x00,0x00,0x04,0x3F,0x20,0x05,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00];
static LCD_INIT_CMD_02: [u8; 31] = [0x30,0x3C,0x24,0x14,0xD0,0x20,0xFF,0xE0,0x40,0x19,0x80,0x80,0x80,0x20,0xF9,0x10,0x02,0xFF,0xFF,0xF0,0x90,0x01,0x32,0xA0,0x91,0xE0,0x20,0x7F,0xFF,0x00,0x5A];
static LCD_INIT_CMD_03: [u8; 30] = [0xE0,0x40,0x51,0x24,0x08,0x05,0x10,0x01,0x20,0x15,0x42,0xC2,0x22,0x22,0xAA,0x03,0x10,0x12,0x60,0x14,0x1E,0x51,0x15,0x00,0x8A,0x20,0x00,0x03,0x3A,0x12];
static LCD_INIT_CMD_04: [u8; 22] = [0xA0,0x06,0xAA,0x00,0x08,0x02,0x0A,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,0x55,0x55];
static LCD_INIT_CMD_05: [u8; 30] = [0x31,0x04,0x02,0x02,0x71,0x05,0x24,0x55,0x02,0x00,0x41,0x00,0x53,0xFF,0xFF,0xFF,0x4F,0x52,0x00,0x4F,0x52,0x00,0x45,0x3B,0x0B,0x02,0x0D,0x00,0xFF,0x40];
static LCD_INIT_CMD_06: [u8; 11] = [0x00,0x00,0x00,0x50,0x03,0x00,0x00,0x00,0x01,0x80,0x01];
static LCD_INIT_CMD_07: [u8; 29] = [0x00,0x24,0x33,0x80,0x00,0xEA,0x64,0x32,0xC8,0x64,0xC8,0x32,0x90,0x90,0x11,0x06,0xDC,0xFA,0x00,0x00,0x80,0xFE,0x10,0x10,0x00,0x0A,0x0A,0x44,0x50];
static LCD_INIT_CMD_08: [u8; 23] = [0x18,0x00,0x00,0x03,0xFE,0x3A,0x4A,0x20,0x30,0x10,0x88,0xDE,0x0D,0x08,0x0F,0x0F,0x01,0x3A,0x4A,0x20,0x10,0x10,0x00];
static LCD_INIT_CMD_09: [u8; 20] = [0x05,0x0A,0x05,0x0A,0x00,0xE0,0x2E,0x0B,0x12,0x22,0x12,0x22,0x01,0x03,0x00,0x3F,0x6A,0x18,0xC8,0x22];
static LCD_INIT_CMD_10: [u8; 20] = [0x50,0x32,0x28,0x00,0xA2,0x80,0x8F,0x00,0x80,0xFF,0x07,0x11,0x9C,0x67,0xFF,0x24,0x0C,0x0D,0x0E,0x0F];
static LCD_INIT_CMD_11: [u8;  4] = [0x33,0x44,0x44,0x01];
static LCD_INIT_CMD_12: [u8; 27] = [0x2C,0x1E,0x88,0x58,0x13,0x18,0x56,0x18,0x1E,0x68,0x88,0x00,0x65,0x09,0x22,0xC4,0x0C,0x77,0x22,0x44,0xAA,0x55,0x08,0x08,0x12,0xA0,0x08];
static LCD_INIT_CMD_13: [u8; 30] = [0x40,0x8E,0x8D,0x01,0x35,0x04,0x92,0x74,0x04,0x92,0x74,0x04,0x08,0x6A,0x04,0x46,0x03,0x03,0x03,0x03,0x82,0x01,0x03,0x00,0xE0,0x51,0xA1,0x00,0x00,0x00];
static LCD_INIT_CMD_14: [u8; 30] = [0x10,0x32,0x54,0x76,0x98,0xBA,0xDC,0xFE,0x93,0x00,0x01,0x83,0x07,0x07,0x00,0x07,0x07,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x84,0x00,0x20,0x01,0x00];
static LCD_INIT_CMD_15: [u8; 19] = [0x03,0x01,0x0B,0x09,0x0F,0x0D,0x1E,0x1F,0x18,0x1D,0x1F,0x19,0x40,0x8E,0x04,0x00,0x20,0xA0,0x1F];
static LCD_INIT_CMD_16: [u8; 12] = [0x02,0x00,0x0A,0x08,0x0E,0x0C,0x1E,0x1F,0x18,0x1D,0x1F,0x19];
static LCD_INIT_CMD_17: [u8; 12] = [0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F];
static LCD_INIT_CMD_18: [u8; 12] = [0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F];
static LCD_INIT_CMD_19: [u8;  8] = [0x44,0x73,0x4B,0x69,0x00,0x0A,0x02,0x90];
static LCD_INIT_CMD_20: [u8; 17] = [0x3B,0x28,0x10,0x16,0x0C,0x06,0x11,0x28,0x5C,0x21,0x0D,0x35,0x13,0x2C,0x33,0x28,0x0D];
static LCD_INIT_CMD_21: [u8; 17] = [0x37,0x28,0x10,0x16,0x0B,0x06,0x11,0x28,0x5C,0x21,0x0D,0x35,0x14,0x2C,0x33,0x28,0x0F];
static LCD_INIT_CMD_22: [u8; 17] = [0x3B,0x07,0x12,0x18,0x0E,0x0D,0x17,0x35,0x44,0x32,0x0C,0x14,0x14,0x36,0x3A,0x2F,0x0D];
static LCD_INIT_CMD_23: [u8; 17] = [0x37,0x07,0x12,0x18,0x0E,0x0D,0x17,0x35,0x44,0x32,0x0C,0x14,0x14,0x36,0x32,0x2F,0x0F];
static LCD_INIT_CMD_24: [u8; 17] = [0x3B,0x07,0x12,0x18,0x0E,0x0D,0x17,0x39,0x44,0x2E,0x0C,0x14,0x14,0x36,0x3A,0x2F,0x0D];
static LCD_INIT_CMD_25: [u8; 17] = [0x37,0x07,0x12,0x18,0x0E,0x0D,0x17,0x39,0x44,0x2E,0x0C,0x14,0x14,0x36,0x3A,0x2F,0x0F];
static LCD_INIT_CMD_26: [u8; 16] = [0x85,0x85,0x95,0x82,0xAF,0xAA,0xAA,0x80,0x10,0x30,0x40,0x40,0x20,0xFF,0x60,0x30];
static LCD_INIT_CMD_27: [u8;  4] = [0x85,0x85,0x95,0x85];
static LCD_INIT_CMD_28: [u8;  8] = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00];
static LCD_INIT_CMD_29: [u8;  4] = [0x00,0x00,0x00,0x00];

static LCD_INIT_CMDS: [Axs15231bLcdInitCmd; 32] = [
    Axs15231bLcdInitCmd { cmd: 0xBB, data: LCD_INIT_CMD_00.as_ptr().cast(), data_bytes: LCD_INIT_CMD_00.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xA0, data: LCD_INIT_CMD_01.as_ptr().cast(), data_bytes: LCD_INIT_CMD_01.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xA2, data: LCD_INIT_CMD_02.as_ptr().cast(), data_bytes: LCD_INIT_CMD_02.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xD0, data: LCD_INIT_CMD_03.as_ptr().cast(), data_bytes: LCD_INIT_CMD_03.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xA3, data: LCD_INIT_CMD_04.as_ptr().cast(), data_bytes: LCD_INIT_CMD_04.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC1, data: LCD_INIT_CMD_05.as_ptr().cast(), data_bytes: LCD_INIT_CMD_05.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC3, data: LCD_INIT_CMD_06.as_ptr().cast(), data_bytes: LCD_INIT_CMD_06.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC4, data: LCD_INIT_CMD_07.as_ptr().cast(), data_bytes: LCD_INIT_CMD_07.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC5, data: LCD_INIT_CMD_08.as_ptr().cast(), data_bytes: LCD_INIT_CMD_08.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC6, data: LCD_INIT_CMD_09.as_ptr().cast(), data_bytes: LCD_INIT_CMD_09.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC7, data: LCD_INIT_CMD_10.as_ptr().cast(), data_bytes: LCD_INIT_CMD_10.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xC9, data: LCD_INIT_CMD_11.as_ptr().cast(), data_bytes: LCD_INIT_CMD_11.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xCF, data: LCD_INIT_CMD_12.as_ptr().cast(), data_bytes: LCD_INIT_CMD_12.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xD5, data: LCD_INIT_CMD_13.as_ptr().cast(), data_bytes: LCD_INIT_CMD_13.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xD6, data: LCD_INIT_CMD_14.as_ptr().cast(), data_bytes: LCD_INIT_CMD_14.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xD7, data: LCD_INIT_CMD_15.as_ptr().cast(), data_bytes: LCD_INIT_CMD_15.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xD8, data: LCD_INIT_CMD_16.as_ptr().cast(), data_bytes: LCD_INIT_CMD_16.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xD9, data: LCD_INIT_CMD_17.as_ptr().cast(), data_bytes: LCD_INIT_CMD_17.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xDD, data: LCD_INIT_CMD_18.as_ptr().cast(), data_bytes: LCD_INIT_CMD_18.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xDF, data: LCD_INIT_CMD_19.as_ptr().cast(), data_bytes: LCD_INIT_CMD_19.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xE0, data: LCD_INIT_CMD_20.as_ptr().cast(), data_bytes: LCD_INIT_CMD_20.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xE1, data: LCD_INIT_CMD_21.as_ptr().cast(), data_bytes: LCD_INIT_CMD_21.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xE2, data: LCD_INIT_CMD_22.as_ptr().cast(), data_bytes: LCD_INIT_CMD_22.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xE3, data: LCD_INIT_CMD_23.as_ptr().cast(), data_bytes: LCD_INIT_CMD_23.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xE4, data: LCD_INIT_CMD_24.as_ptr().cast(), data_bytes: LCD_INIT_CMD_24.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xE5, data: LCD_INIT_CMD_25.as_ptr().cast(), data_bytes: LCD_INIT_CMD_25.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xA4, data: LCD_INIT_CMD_26.as_ptr().cast(), data_bytes: LCD_INIT_CMD_26.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xA4, data: LCD_INIT_CMD_27.as_ptr().cast(), data_bytes: LCD_INIT_CMD_27.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0xBB, data: LCD_INIT_CMD_28.as_ptr().cast(), data_bytes: LCD_INIT_CMD_28.len(), delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0x13, data: core::ptr::null(), data_bytes: 0, delay_ms: 0 },
    Axs15231bLcdInitCmd { cmd: 0x11, data: core::ptr::null(), data_bytes: 0, delay_ms: 120 },
    Axs15231bLcdInitCmd { cmd: 0x2C, data: LCD_INIT_CMD_29.as_ptr().cast(), data_bytes: LCD_INIT_CMD_29.len(), delay_ms: 0 },
];

fn esp_check(err: esp_idf_sys::esp_err_t, label: &str) -> anyhow::Result<()> {
    if err == esp_idf_sys::ESP_OK as i32 {
        Ok(())
    } else {
        anyhow::bail!("{} failed: {:08X}", label, err)
    }
}

/// Owns the LCD panel IO and panel handles produced by [`init_display`].
pub struct LcdContext {
    pub io:    esp_idf_sys::esp_lcd_panel_io_handle_t,
    pub panel: esp_idf_sys::esp_lcd_panel_handle_t,
    _vendor_config: Box<Axs15231bVendorConfig>,
}

// Safety: C handles are only called from the render thread after LcdContext is moved there.
unsafe impl Send for LcdContext {}

impl LcdContext {
    /// Flush a framebuffer to the physical panel.
    pub fn flush_fb(&self, fb: &Framebuffer, orientation: crate::layout::Orientation) {
        fb.flush_to_panel(self.io, self.panel, orientation);
    }
}

/// Initialise the SPI bus and AXS15231B panel. Returns an [`LcdContext`] on success.
pub fn init_display() -> anyhow::Result<LcdContext> {
    let mut bus_cfg = esp_idf_sys::spi_bus_config_t::default();
    bus_cfg.__bindgen_anon_1.mosi_io_num = PIN_LCD_D0;
    bus_cfg.__bindgen_anon_2.miso_io_num = PIN_LCD_D1;
    bus_cfg.__bindgen_anon_3.quadwp_io_num = PIN_LCD_D2;
    bus_cfg.__bindgen_anon_4.quadhd_io_num = PIN_LCD_D3;
    bus_cfg.sclk_io_num = PIN_LCD_SCLK;
    bus_cfg.max_transfer_sz = PANEL_WIDTH as i32 * CHUNK_LINES * 2;

    let host = esp_idf_sys::spi_host_device_t_SPI2_HOST;
    esp_check(
        unsafe { esp_idf_sys::spi_bus_initialize(host, &bus_cfg, esp_idf_sys::spi_common_dma_t_SPI_DMA_CH_AUTO) },
        "spi_bus_initialize",
    )?;

    let mut io: esp_idf_sys::esp_lcd_panel_io_handle_t = core::ptr::null_mut();
    let io_cfg = esp_idf_sys::esp_lcd_panel_io_spi_config_t {
        cs_gpio_num: PIN_LCD_CS,
        dc_gpio_num: -1,
        spi_mode: 3,
        pclk_hz: PCLK_HZ,
        trans_queue_depth: 10,
        on_color_trans_done: None,
        user_ctx: core::ptr::null_mut(),
        lcd_cmd_bits: 32,
        lcd_param_bits: 8,
        flags: esp_idf_sys::esp_lcd_panel_io_spi_config_t__bindgen_ty_1 {
            _bitfield_align_1: [],
            _bitfield_1: esp_idf_sys::esp_lcd_panel_io_spi_config_t__bindgen_ty_1::new_bitfield_1(
                0, 0, 0, 0, 1, 0, 0, 0,
            ),
            __bindgen_padding_0: [0; 3],
        },
    };
    esp_check(
        unsafe { esp_idf_sys::esp_lcd_new_panel_io_spi(host as esp_idf_sys::esp_lcd_spi_bus_handle_t, &io_cfg, &mut io) },
        "esp_lcd_new_panel_io_spi",
    )?;

    let mut panel: esp_idf_sys::esp_lcd_panel_handle_t = core::ptr::null_mut();
    let vendor_config = Box::new(Axs15231bVendorConfig {
        init_cmds: LCD_INIT_CMDS.as_ptr(),
        init_cmds_size: LCD_INIT_CMDS.len() as u16,
        flags: Axs15231bVendorFlags { use_qspi_interface: 1 },
    });

    let panel_cfg = esp_idf_sys::esp_lcd_panel_dev_config_t {
        reset_gpio_num: -1,
        __bindgen_anon_1: esp_idf_sys::esp_lcd_panel_dev_config_t__bindgen_ty_1 {
            rgb_ele_order: esp_idf_sys::lcd_rgb_element_order_t_LCD_RGB_ELEMENT_ORDER_RGB,
        },
        data_endian: esp_idf_sys::lcd_rgb_data_endian_t_LCD_RGB_DATA_ENDIAN_BIG,
        bits_per_pixel: 16,
        flags: esp_idf_sys::esp_lcd_panel_dev_config_t__bindgen_ty_2 {
            _bitfield_align_1: [],
            _bitfield_1: esp_idf_sys::esp_lcd_panel_dev_config_t__bindgen_ty_2::new_bitfield_1(0),
            __bindgen_padding_0: [0; 3],
        },
        vendor_config: (&*vendor_config) as *const Axs15231bVendorConfig as *mut core::ffi::c_void,
    };

    esp_check(
        unsafe { esp_lcd_new_panel_axs15231b(io, &panel_cfg, &mut panel) },
        "esp_lcd_new_panel_axs15231b",
    )?;

    esp_check(unsafe { esp_idf_sys::esp_lcd_panel_reset(panel) }, "panel_reset")?;
    esp_check(unsafe { esp_idf_sys::esp_lcd_panel_init(panel) }, "panel_init")?;
    // Note: AXS15231B driver inverts polarity — false = display ON.
    esp_check(unsafe { esp_idf_sys::esp_lcd_panel_disp_on_off(panel, false) }, "disp_on")?;

    log::info!("Display initialized OK");
    Ok(LcdContext { io, panel, _vendor_config: vendor_config })
}

/// Drive the backlight GPIO high to turn the display on.
pub fn enable_backlight() {
    unsafe {
        let io_conf = esp_idf_sys::gpio_config_t {
            pin_bit_mask: 1u64 << (PIN_LCD_BL as u64),
            mode: esp_idf_sys::gpio_mode_t_GPIO_MODE_OUTPUT,
            pull_up_en: esp_idf_sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: esp_idf_sys::gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: esp_idf_sys::gpio_int_type_t_GPIO_INTR_DISABLE,
        };
        esp_idf_sys::gpio_config(&io_conf);
        esp_idf_sys::gpio_set_level(PIN_LCD_BL, 1);
    }
    log::info!("Backlight ON");
}

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
