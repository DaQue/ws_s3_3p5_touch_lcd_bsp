//! ES8311 audio codec driver with alert tone generation.
//!
//! # Hardware
//!
//! The Waveshare ESP32-S3-Touch-LCD-3.5B includes:
//!
//! - **ES8311** — Everest Semiconductor mono audio codec, connected via I2C (for
//!   register configuration) and I2S (for PCM audio data). I2C address **0x18**.
//! - **TCA9554** — Texas Instruments 8-bit I2C GPIO expander used to control
//!   the speaker power amplifier enable line. I2C address **0x20**.
//!
//! The signal path is: ESP32-S3 I2S0 → ES8311 DAC → amplifier (PA) → speaker.
//! The PA is controlled through the TCA9554's bit 7 on the output register.
//!
//! # ES8311 initialization
//!
//! The [`init_audio_path`] function configures the ES8311 in **I2S slave mode**
//! (the ESP32 is the I2S master). Audio data format: 16-bit, stereo, Philips
//! (standard I2S) framing at 48 kHz.
//!
//! The register sequence was derived from the Espressif `esp_codec_dev` ES8311
//! open/start flow and validated against hardware. Key steps:
//! 1. Enable PA via TCA9554 GPIO
//! 2. Set ES8311 to slave mode, external MCLK
//! 3. Configure DAC path (unmute, set volume to max)
//! 4. Start clock and data paths
//!
//! # Tone generation
//!
//! [`Speaker::play`] generates **square wave tones** in software and streams
//! them to the codec via I2S. Three urgency levels are defined as [`AlertTone`]:
//!
//! | Tone       | Pattern                          | Use case             |
//! |------------|----------------------------------|----------------------|
//! | Advisory   | Soft double chirp (820 + 980 Hz) | Informational alert  |
//! | Watch      | Triple pulse (1280/1460/1280 Hz) | Attention required   |
//! | Warning    | 3× rising triad (1250/1750/2450 Hz) | Immediate danger  |
//!
//! Tones include a short fade-in and fade-out to reduce click/pop at boundaries.
//! The `should_stop` callback is checked between chunks, allowing the caller to
//! interrupt playback at any time (e.g. if the alert was silenced).
//!
//! # Usage
//!
//! ```rust,ignore
//! use ws_s3_3p5_bsp::speaker::{Speaker, AlertTone, init_audio_path};
//!
//! // One-time init (call after I2C and I2S peripherals are set up)
//! init_audio_path(&mut i2c).expect("audio init failed");
//!
//! let mut speaker = Speaker::new(i2s0, bclk_pin, dout_pin, ws_pin, Some(mclk_pin))
//!     .expect("I2S init failed");
//!
//! // Play a warning tone (blocks until complete or should_stop returns true)
//! speaker.play(AlertTone::Warning, || false).ok();
//! ```

use std::thread;
use std::time::Duration;

use esp_idf_hal::gpio::{InputPin, OutputPin};
use esp_idf_hal::i2c::I2cDriver;
use esp_idf_hal::i2s::config::{
    Config as I2sChannelConfig, DataBitWidth, SlotMode, StdClkConfig, StdConfig, StdGpioConfig,
    StdSlotConfig,
};
use esp_idf_hal::i2s::{I2sDriver, I2sTx, I2S0};
use esp_idf_hal::peripheral::Peripheral;
use esp_idf_sys::{EspError, ESP_ERR_TIMEOUT};

// ── I2C addresses ────────────────────────────────────────────────────────────

/// ES8311 audio codec I2C address (fixed, not configurable via pins).
const ES8311_ADDR: u8 = 0x18;

/// TCA9554 I/O expander I2C address (A2=A1=A0=GND → 0x20).
const TCA9554_ADDR: u8 = 0x20;

// ── TCA9554 registers ────────────────────────────────────────────────────────
// Reference: TCA9554 datasheet Table 4

/// TCA9554 Output Port register — write to set output pin levels.
const TCA9554_REG_OUTPUT: u8 = 0x01;

/// TCA9554 Configuration register — bit=0 → output, bit=1 → input.
const TCA9554_REG_CONFIG: u8 = 0x03;

/// TCA9554 bit position for the speaker power amplifier enable.
/// Bit 7 = PA_CTRL. Writing 1 enables the amplifier.
const TCA9554_PA_CTRL_BIT: u8 = 1 << 7;

// ── ES8311 registers ─────────────────────────────────────────────────────────
// Reference: ES8311 datasheet register map (Everest Semiconductor, not publicly available;
// values derived from Espressif esp_codec_dev open-source driver)

/// ES8311 REG00 — Reset and misc control. Bit 6 = MCLK source select.
const ES8311_REG_RESET: u8 = 0x00;

/// ES8311 REG01 — Clock manager 1. Controls master clock dividers.
const ES8311_REG_CLK1: u8 = 0x01;

/// ES8311 REG02 — Clock manager 2 (MCLK frequency selection).
const ES8311_REG_CLK2: u8 = 0x02;

/// ES8311 REG03 — Clock manager 3 (LRCK/BCLK ratio).
const ES8311_REG_CLK3: u8 = 0x03;

/// ES8311 REG04 — Clock manager 4.
const ES8311_REG_CLK4: u8 = 0x04;

/// ES8311 REG05 — Clock manager 5.
const ES8311_REG_CLK5: u8 = 0x05;

/// ES8311 REG06 — Clock manager 6. Bit 5 = master clock invert.
const ES8311_REG_CLK6: u8 = 0x06;

/// ES8311 REG09 — Serial data port input (I2S RX) routing.
const ES8311_REG_SDPIN: u8 = 0x09;

/// ES8311 REG0A — Serial data port output (I2S TX) routing.
const ES8311_REG_SDPOUT: u8 = 0x0A;

/// ES8311 REG0B — System register 1.
const ES8311_REG_SYS1: u8 = 0x0B;

/// ES8311 REG0C — System register 2.
const ES8311_REG_SYS2: u8 = 0x0C;

/// ES8311 REG0D — System register 3 (power).
const ES8311_REG_SYS3: u8 = 0x0D;

/// ES8311 REG0E — DAC power management.
const ES8311_REG_DAC_PWR: u8 = 0x0E;

/// ES8311 REG10 — ADC control (gain).
const ES8311_REG_ADC_CTRL: u8 = 0x10;

/// ES8311 REG11 — ADC gain.
const ES8311_REG_ADC_GAIN: u8 = 0x11;

/// ES8311 REG12 — DAC control.
const ES8311_REG_DAC_CTRL: u8 = 0x12;

/// ES8311 REG13 — DAC EQ / tone control.
const ES8311_REG_DAC_EQ: u8 = 0x13;

/// ES8311 REG14 — ADC input selection and gain.
const ES8311_REG_ADC_INP: u8 = 0x14;

/// ES8311 REG15 — ADC ALC control.
const ES8311_REG_ADC_ALC: u8 = 0x15;

/// ES8311 REG16 — ADC output / mute control.
const ES8311_REG_ADC_OUT: u8 = 0x16;

/// ES8311 REG17 — DAC output / mute control. Bits [5:4] = mute.
const ES8311_REG_DAC_OUT: u8 = 0x17;

/// ES8311 REG1B — DAC left channel digital volume.
const ES8311_REG_DAC_VOL_L: u8 = 0x1B;

/// ES8311 REG1C — DAC right channel digital volume.
const ES8311_REG_DAC_VOL_R: u8 = 0x1C;

/// ES8311 REG31 — DAC analog mute control. Bits [5:4]: 0b00 = unmuted.
const ES8311_REG_ANA_MUTE: u8 = 0x31;

/// ES8311 REG32 — DAC analog gain (output level). 0xFF = maximum.
const ES8311_REG_ANA_GAIN: u8 = 0x32;

/// ES8311 REG37 — ALC / noise gate control.
const ES8311_REG_ALC: u8 = 0x37;

/// ES8311 REG44 — GPIO / jack detection control.
const ES8311_REG_GPIO: u8 = 0x44;

/// ES8311 REG45 — GPIO 2 control.
const ES8311_REG_GPIO2: u8 = 0x45;

// ── Public types ─────────────────────────────────────────────────────────────

/// Alert tone urgency level.
///
/// Each level uses a distinct frequency pattern and amplitude to convey urgency.
/// Advisory is the softest; Warning is the loudest and most attention-demanding.
#[derive(Clone, Copy, Debug)]
pub enum AlertTone {
    /// Soft double chirp (820 Hz + 980 Hz). Low urgency — informational.
    Advisory,
    /// Triple pulse (1280 / 1460 / 1280 Hz). Medium urgency — action recommended.
    Watch,
    /// Rising triad ×3 (1250 / 1750 / 2450 Hz). High urgency — immediate action required.
    Warning,
}

impl AlertTone {
    /// Decode a tone from a compact integer code (used for inter-thread signalling).
    /// Returns `None` for unknown codes.
    pub fn from_request(v: i8) -> Option<Self> {
        match v {
            0 => Some(Self::Advisory),
            1 => Some(Self::Watch),
            2 => Some(Self::Warning),
            _ => None,
        }
    }

    /// Human-readable name of the tone level.
    pub fn as_str(self) -> &'static str {
        match self {
            AlertTone::Advisory => "advisory",
            AlertTone::Watch    => "watch",
            AlertTone::Warning  => "warning",
        }
    }

    /// Compact integer code for this tone level (inverse of [`from_request`](Self::from_request)).
    pub fn request_code(self) -> i8 {
        match self {
            AlertTone::Advisory => 0,
            AlertTone::Watch    => 1,
            AlertTone::Warning  => 2,
        }
    }
}

/// I2S speaker driver. Streams square-wave PCM tones to the ES8311 codec.
pub struct Speaker<'d> {
    i2s: I2sDriver<'d, I2sTx>,
    /// Sample rate used for tone generation (must match I2S config).
    sample_rate_hz: u32,
}

// ── Public functions ─────────────────────────────────────────────────────────

/// Initialize the ES8311 audio path.
///
/// Must be called **once** after I2C is initialized and before creating a
/// [`Speaker`]. Enables the power amplifier via TCA9554 and configures the
/// ES8311 for 48 kHz 16-bit stereo slave-mode playback.
pub fn init_audio_path(i2c: &mut I2cDriver<'_>) -> Result<(), EspError> {
    enable_pa(i2c)?;
    init_es8311(i2c)?;
    Ok(())
}

// ── Private I2C helpers ──────────────────────────────────────────────────────

fn es_write(i2c: &mut I2cDriver<'_>, reg: u8, val: u8) -> Result<(), EspError> {
    i2c.write(ES8311_ADDR, &[reg, val], 100)
}

fn es_read(i2c: &mut I2cDriver<'_>, reg: u8) -> Result<u8, EspError> {
    let mut v = [0u8; 1];
    i2c.write_read(ES8311_ADDR, &[reg], &mut v, 100)?;
    Ok(v[0])
}

fn tca_read(i2c: &mut I2cDriver<'_>, reg: u8) -> Result<u8, EspError> {
    let mut v = [0u8; 1];
    i2c.write_read(TCA9554_ADDR, &[reg], &mut v, 100)?;
    Ok(v[0])
}

fn tca_write(i2c: &mut I2cDriver<'_>, reg: u8, val: u8) -> Result<(), EspError> {
    i2c.write(TCA9554_ADDR, &[reg, val], 100)
}

/// Enable the speaker power amplifier via TCA9554 bit 7.
///
/// 1. Configure PA_CTRL pin as output (clear bit 7 of CONFIG register)
/// 2. Set PA_CTRL output high (set bit 7 of OUTPUT register) to enable the PA
fn enable_pa(i2c: &mut I2cDriver<'_>) -> Result<(), EspError> {
    let mut config = tca_read(i2c, TCA9554_REG_CONFIG)?;
    config &= !TCA9554_PA_CTRL_BIT;          // set bit 7 as output
    tca_write(i2c, TCA9554_REG_CONFIG, config)?;

    let mut output = tca_read(i2c, TCA9554_REG_OUTPUT)?;
    output |= TCA9554_PA_CTRL_BIT;           // drive bit 7 high → PA enabled
    tca_write(i2c, TCA9554_REG_OUTPUT, output)?;
    Ok(())
}

/// Configure the ES8311 codec for 48 kHz 16-bit stereo I2S slave playback.
///
/// This sequence follows the Espressif `esp_codec_dev` ES8311 open/start flow.
/// The ESP32-S3 is the I2S master; the ES8311 derives its clocks from BCLK/LRCK.
fn init_es8311(i2c: &mut I2cDriver<'_>) -> Result<(), EspError> {
    // ── Power-on reset ───────────────────────────────────────────────────────
    es_write(i2c, ES8311_REG_GPIO,  0x08)?; // GPIO pin function: CLK_OUT
    es_write(i2c, ES8311_REG_GPIO,  0x08)?; // write twice to ensure stable
    es_write(i2c, ES8311_REG_CLK1,  0x30)?; // reset codec
    es_write(i2c, ES8311_REG_CLK2,  0x00)?; // MCLK divider = 1
    es_write(i2c, ES8311_REG_CLK3,  0x10)?; // ADC/DAC OSR = 16
    es_write(i2c, ES8311_REG_ADC_OUT, 0x24)?; // ADC output: normal, mute
    es_write(i2c, ES8311_REG_CLK4,  0x10)?; // LRCK divider high byte
    es_write(i2c, ES8311_REG_CLK5,  0x00)?; // LRCK divider low byte
    es_write(i2c, ES8311_REG_SYS1,  0x00)?; // system power normal
    es_write(i2c, ES8311_REG_SYS2,  0x00)?; // reference power normal
    es_write(i2c, ES8311_REG_ADC_CTRL, 0x1F)?; // ADC gain = 0 dB
    es_write(i2c, ES8311_REG_ADC_GAIN, 0x7F)?; // ADC digital gain = max
    es_write(i2c, ES8311_REG_RESET, 0x80)?; // release reset, enable clocks

    // ── Slave mode: use external MCLK from I2S master ────────────────────────
    let mut reg00 = es_read(i2c, ES8311_REG_RESET)?;
    reg00 &= !0x40;                              // bit 6 = 0 → use CLK_IN pin (external MCLK)
    es_write(i2c, ES8311_REG_RESET, reg00)?;
    es_write(i2c, ES8311_REG_CLK1,  0x3F)?;     // all clock dividers enabled

    let mut reg06 = es_read(i2c, ES8311_REG_CLK6)?;
    reg06 &= !0x20;                              // bit 5 = 0 → non-inverted MCLK
    es_write(i2c, ES8311_REG_CLK6, reg06)?;

    // ── DAC path configuration ───────────────────────────────────────────────
    es_write(i2c, ES8311_REG_DAC_EQ,   0x10)?;  // DAC tone control: flat
    es_write(i2c, ES8311_REG_DAC_VOL_L, 0x0A)?; // DAC left volume: moderate
    es_write(i2c, ES8311_REG_DAC_VOL_R, 0x6A)?; // DAC right volume: higher for L+R mix
    es_write(i2c, ES8311_REG_GPIO,      0x58)?;  // GPIO: enable CLK_OUT for MCLK

    // ── Start clock and data path ────────────────────────────────────────────
    es_write(i2c, ES8311_REG_RESET, 0x80)?;      // re-release reset (belt-and-suspenders)
    es_write(i2c, ES8311_REG_CLK1,  0x3F)?;      // enable all clocks

    let mut reg09 = es_read(i2c, ES8311_REG_SDPIN)?;
    reg09 &= !0x40;                              // bit 6 = 0 → I2S input normal (not loopback)
    es_write(i2c, ES8311_REG_SDPIN, reg09)?;

    let mut reg0a = es_read(i2c, ES8311_REG_SDPOUT)?;
    reg0a &= !0x40;                              // bit 6 = 0 → I2S output normal
    es_write(i2c, ES8311_REG_SDPOUT, reg0a)?;

    es_write(i2c, ES8311_REG_DAC_OUT,  0xBF)?;  // DAC output: enable, high-drive
    es_write(i2c, ES8311_REG_DAC_PWR,  0x02)?;  // DAC power: enable DAC
    es_write(i2c, ES8311_REG_DAC_CTRL, 0x00)?;  // DAC control: normal operation
    es_write(i2c, ES8311_REG_ADC_INP,  0x1A)?;  // ADC input: line-in, gain
    es_write(i2c, ES8311_REG_SYS3,     0x01)?;  // power on sequence complete
    es_write(i2c, ES8311_REG_ADC_ALC,  0x40)?;  // ALC: disabled
    es_write(i2c, ES8311_REG_ALC,      0x08)?;  // noise gate: disabled
    es_write(i2c, ES8311_REG_GPIO2,    0x00)?;  // GPIO2: default

    // ── Unmute and set output level ──────────────────────────────────────────
    // REG31 bits [5:4]: 0b00 = unmuted. Preserve other bits.
    let mut reg31 = es_read(i2c, ES8311_REG_ANA_MUTE)?;
    reg31 &= 0x9F;                               // clear bits [5:4]
    es_write(i2c, ES8311_REG_ANA_MUTE, reg31)?;
    es_write(i2c, ES8311_REG_ANA_GAIN, 0xFF)?;  // analog gain: maximum

    Ok(())
}

// ── Speaker impl ─────────────────────────────────────────────────────────────

impl<'d> Speaker<'d> {
    /// Create a new Speaker using I2S0.
    ///
    /// Configures I2S0 as a 48 kHz, 16-bit stereo Philips-mode TX channel.
    /// Call [`init_audio_path`] first to enable the codec and PA.
    ///
    /// # Parameters
    ///
    /// - `i2s` — I2S0 peripheral (consumed)
    /// - `bclk` — bit clock GPIO
    /// - `dout` — data output GPIO (ESP32 → codec)
    /// - `ws` — word select (LRCK) GPIO
    /// - `mclk` — optional master clock GPIO (pass `None` if codec is in slave mode)
    pub fn new(
        i2s: impl Peripheral<P = I2S0> + 'd,
        bclk: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        dout: impl Peripheral<P = impl OutputPin> + 'd,
        ws: impl Peripheral<P = impl InputPin + OutputPin> + 'd,
        mclk: Option<impl Peripheral<P = impl InputPin + OutputPin> + 'd>,
    ) -> Result<Self, EspError> {
        let sample_rate_hz = 48_000u32;
        let std_config = StdConfig::new(
            I2sChannelConfig::default().auto_clear(true),
            StdClkConfig::from_sample_rate_hz(sample_rate_hz),
            StdSlotConfig::philips_slot_default(DataBitWidth::Bits16, SlotMode::Stereo),
            StdGpioConfig::default(),
        );
        let mut i2s = I2sDriver::<I2sTx>::new_std_tx(i2s, &std_config, bclk, dout, mclk, ws)?;
        i2s.tx_enable()?;
        Ok(Self { i2s, sample_rate_hz })
    }

    /// Play an alert tone.
    ///
    /// Blocks until the tone completes or `should_stop()` returns `true`.
    /// `should_stop` is polled between each audio chunk (~5 ms resolution).
    ///
    /// After the tone (or on early stop), the I2S FIFO is flushed with silence
    /// to prevent audio artifacts.
    pub fn play<F: FnMut() -> bool>(&mut self, tone: AlertTone, mut should_stop: F) -> Result<(), EspError> {
        match tone {
            AlertTone::Advisory => {
                // Soft double chirp: low-urgency informational
                if !self.tone(820, 90, 9000, &mut should_stop)?  { return self.silence(); }
                if !Self::pause(55, &mut should_stop)             { return self.silence(); }
                if !self.tone(980, 90, 9000, &mut should_stop)?  { return self.silence(); }
            }
            AlertTone::Watch => {
                // Triple ascending pulse: clear attention-getter
                if !self.tone(1280, 85, 11500, &mut should_stop)? { return self.silence(); }
                if !Self::pause(45, &mut should_stop)              { return self.silence(); }
                if !self.tone(1460, 85, 11500, &mut should_stop)? { return self.silence(); }
                if !Self::pause(45, &mut should_stop)              { return self.silence(); }
                if !self.tone(1280, 95, 11500, &mut should_stop)? { return self.silence(); }
            }
            AlertTone::Warning => {
                // 3× rising triad: cannot be ignored
                for round in 0..3u8 {
                    if !self.tone(1250, 95,  12500, &mut should_stop)? { return self.silence(); }
                    if !Self::pause(35, &mut should_stop)               { return self.silence(); }
                    if !self.tone(1750, 105, 12500, &mut should_stop)? { return self.silence(); }
                    if !Self::pause(35, &mut should_stop)               { return self.silence(); }
                    if !self.tone(2450, 135, 12500, &mut should_stop)? { return self.silence(); }
                    if round < 2 && !Self::pause(250, &mut should_stop) { return self.silence(); }
                }
            }
        }
        self.silence()
    }

    // ── Private tone generation ──────────────────────────────────────────────

    /// Flush the I2S FIFO with silence to prevent audio artifacts at tone boundaries.
    fn silence(&mut self) -> Result<(), EspError> {
        let buf = [0u8; 512];
        for _ in 0..6 {
            let mut offset = 0usize;
            while offset < buf.len() {
                match self.i2s.write(&buf[offset..], 20) {
                    Ok(0) => break,
                    Ok(n) => offset += n,
                    Err(e) if e.code() == ESP_ERR_TIMEOUT => continue,
                    Err(e) => return Err(e),
                }
            }
        }
        Ok(())
    }

    /// Generate and stream a square wave tone.
    ///
    /// - `freq_hz` — tone frequency in Hz
    /// - `duration_ms` — tone duration in milliseconds
    /// - `amp` — peak amplitude (0–32767; values above ~15000 may clip on small speakers)
    /// - `should_stop` — callback polled between chunks to allow early cancellation
    ///
    /// Returns `true` if the tone completed, `false` if interrupted by `should_stop`.
    ///
    /// The waveform is a symmetric square wave with a short linear fade-in and
    /// fade-out (each 5 ms at 48 kHz) to reduce click/pop artifacts.
    fn tone<F: FnMut() -> bool>(
        &mut self,
        freq_hz: u32,
        duration_ms: u64,
        amp: i16,
        should_stop: &mut F,
    ) -> Result<bool, EspError> {
        const CHUNK_FRAMES: usize = 256; // frames per write (256 frames = ~5.3 ms at 48 kHz)
        let mut phase: u32 = 0;
        let period = (self.sample_rate_hz / freq_hz.max(1)).max(2); // samples per full cycle
        let total_frames = (self.sample_rate_hz as u64 * duration_ms) / 1000;
        let fade_frames = ((self.sample_rate_hz / 200) as usize).max(1); // 5 ms fade
        let mut sent_frames: u64 = 0;

        while sent_frames < total_frames {
            if should_stop() { return Ok(false); }

            let chunk = (total_frames - sent_frames).min(CHUNK_FRAMES as u64) as usize;
            let mut samples = [0i16; CHUNK_FRAMES * 2]; // stereo interleaved

            for i in 0..chunk {
                // Square wave: high for first half of period, low for second half
                let hi = (phase % period) < (period / 2);

                // Linear fade envelope to prevent clicks at tone start/end
                let frame_abs = sent_frames as usize + i;
                let start_gain = if frame_abs < fade_frames {
                    frame_abs as f32 / fade_frames as f32
                } else { 1.0 };
                let end_frames_left = total_frames.saturating_sub(sent_frames + i as u64) as usize;
                let end_gain = if end_frames_left < fade_frames {
                    end_frames_left as f32 / fade_frames as f32
                } else { 1.0 };

                let s = if hi { (amp as f32 * start_gain.min(end_gain)) as i16 }
                        else  { -(amp as f32 * start_gain.min(end_gain)) as i16 };
                samples[i * 2]     = s; // left channel
                samples[i * 2 + 1] = s; // right channel (mono signal on both)
                phase = phase.wrapping_add(1);
            }

            let bytes = unsafe {
                core::slice::from_raw_parts(
                    samples.as_ptr() as *const u8,
                    chunk * 2 * core::mem::size_of::<i16>(),
                )
            };
            let mut offset = 0usize;
            while offset < bytes.len() {
                if should_stop() { return Ok(false); }
                match self.i2s.write(&bytes[offset..], 20) {
                    Ok(0) => continue,
                    Ok(n) => offset += n,
                    Err(e) if e.code() == ESP_ERR_TIMEOUT => continue,
                    Err(e) => return Err(e),
                }
            }
            sent_frames += chunk as u64;
        }
        Ok(true)
    }

    /// Sleep for `duration_ms` milliseconds, checking `should_stop` every 10 ms.
    /// Returns `false` if `should_stop` fired before the duration elapsed.
    fn pause<F: FnMut() -> bool>(duration_ms: u64, should_stop: &mut F) -> bool {
        let mut remaining = duration_ms;
        while remaining > 0 {
            if should_stop() { return false; }
            let step = remaining.min(10);
            thread::sleep(Duration::from_millis(step));
            remaining -= step;
        }
        true
    }
}
