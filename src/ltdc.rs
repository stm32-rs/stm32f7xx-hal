use micromath::F32Ext;

use crate::{
    device::{ltdc::LAYER, LTDC, RCC},
    rcc::HSEClock,
};

/// Display configuration constants
pub struct DisplayConfig {
    pub active_width: u16,
    pub active_height: u16,
    pub h_back_porch: u16,
    pub h_front_porch: u16,
    pub v_back_porch: u16,
    pub v_front_porch: u16,
    pub h_sync: u16,
    pub v_sync: u16,
    pub frame_rate: u16,
    /// `false`: active low, `true`: active high
    pub h_sync_pol: bool,
    /// `false`: active low, `true`: active high
    pub v_sync_pol: bool,
    /// `false`: active low, `true`: active high
    pub no_data_enable_pol: bool,
    /// `false`: active low, `true`: active high
    pub pixel_clock_pol: bool,
}

/// Accessible layers
/// * `L1`: layer 1
/// * `L2`: layer 2
pub enum Layer {
    L1,
    L2,
}

pub struct DisplayController<T: 'static + SupportedWord> {
    // ltdc instance
    _ltdc: LTDC,
    config: DisplayConfig,
    // Layer 1 buffer
    buffer1: Option<&'static mut [T]>,
    // Layer 2 buffer
    buffer2: Option<&'static mut [T]>,
}

impl<T: SupportedWord> DisplayController<T> {
    /// Create and configure the DisplayController
    pub fn new(ltdc: LTDC, config: DisplayConfig, hse: Option<&HSEClock>) -> DisplayController<T> {
        // TODO : change it to something safe ...
        let rcc = unsafe { &(*RCC::ptr()) };

        // Screen constants
        let total_width: u16 =
            config.h_sync + config.h_back_porch + config.active_width + config.h_front_porch - 1;
        let total_height: u16 =
            config.v_sync + config.v_back_porch + config.active_height + config.v_front_porch - 1;
        let lcd_clk: u32 =
            (total_width as u32) * (total_height as u32) * (config.frame_rate as u32);

        // Enable LTDC peripheral's clock
        rcc.apb2enr.modify(|_, w| w.ltdcen().enabled());
        // Reset LTDC peripheral
        rcc.apb2rstr.modify(|_, w| w.ltdcrst().reset());
        rcc.apb2rstr.modify(|_, w| w.ltdcrst().clear_bit());

        // Get base clock and PLLM divisor
        let base_clk: u32;
        match &hse {
            Some(hse) => base_clk = hse.freq,
            // If no HSE is provided, we use the HSI clock at 16 MHz
            None => base_clk = 16_000_000,
        }
        let pllm: u8 = rcc.pllcfgr.read().pllm().bits();

        // There are 24 combinations possible for a divisor with PLLR and DIVR
        // We find the one that is the closest possible to the target value
        // while respecting all the conditions
        let vco_in_mhz: f32 = (base_clk as f32 / pllm as f32) / 1_000_000.0;
        let lcd_clk_mhz = (lcd_clk as f32) / 1_000_000.0;
        let allowed_pllr = [2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        let allowed_divr = [2.0, 4.0, 8.0, 16.0];
        let mut best_pllr: f32 = allowed_pllr[0];
        let mut best_divr: f32 = allowed_divr[0];
        let mut best_plln: f32 = 100.0;
        let mut best_error: f32 = (vco_in_mhz * best_plln) / (best_pllr * best_divr);
        let mut error: f32;
        let mut plln: f32;

        for pllr in &allowed_pllr {
            for divr in &allowed_divr {
                plln = ((lcd_clk_mhz * divr * pllr) / vco_in_mhz).floor();
                error = lcd_clk_mhz - (vco_in_mhz * plln) / (pllr * divr);

                // We have to make sure that the VCO_OUT is in range [100, 432]
                // MHz Because VCO_IN is in range [1, 2] Mhz, the condition
                // PLLN in range [50, 432] is automatically satisfied
                if 100.0 <= vco_in_mhz * plln
                    && vco_in_mhz * plln <= 432.0
                    && error >= 0.0
                    && error < best_error
                {
                    best_pllr = *pllr;
                    best_divr = *divr;
                    best_plln = plln;
                    best_error = error;
                }
            }
        }

        let pllsaidivr: u8 = match best_divr as u16 {
            2 => 0b00,
            4 => 0b01,
            8 => 0b10,
            16 => 0b11,
            _ => unreachable!(),
        };

        // // Write PPLSAI configuration
        rcc.pllsaicfgr.write(|w| unsafe {
            w.pllsain()
                .bits(best_plln as u16)
                .pllsair()
                .bits(best_pllr as u8)
        });
        rcc.dckcfgr1.modify(|_, w| w.pllsaidivr().bits(pllsaidivr));

        // Enable PLLSAI and wait for it
        rcc.cr.modify(|_, w| w.pllsaion().on());
        while rcc.cr.read().pllsairdy().is_not_ready() {}

        // Configure LTDC Timing registers
        ltdc.sscr.write(|w| unsafe {
            w.hsw()
                .bits((config.h_sync - 1) as u16)
                .vsh()
                .bits((config.v_sync - 1) as u16)
        });
        ltdc.bpcr.write(|w| unsafe {
            w.ahbp()
                .bits((config.h_sync + config.h_back_porch - 1) as u16)
                .avbp()
                .bits((config.v_sync + config.v_back_porch - 1) as u16)
        });
        ltdc.awcr.write(|w| unsafe {
            w.aaw()
                .bits((config.h_sync + config.h_back_porch + config.active_width - 1) as u16)
                .aah()
                .bits((config.v_sync + config.v_back_porch + config.active_height - 1) as u16)
        });
        ltdc.twcr.write(|w| unsafe {
            w.totalw()
                .bits(total_width as u16)
                .totalh()
                .bits(total_height as u16)
        });

        // Configure LTDC signals polarity
        ltdc.gcr.write(|w| {
            w.hspol()
                .bit(config.h_sync_pol)
                .vspol()
                .bit(config.v_sync_pol)
                .depol()
                .bit(config.no_data_enable_pol)
                .pcpol()
                .bit(config.pixel_clock_pol)
        });

        // Set blue background color
        ltdc.bccr.write(|w| unsafe { w.bits(0xAAAAAAAA) });

        // TODO: configure DMA2D hardware accelerator
        // TODO: configure interupts

        // Reload ltdc config immediatly
        ltdc.srcr.modify(|_, w| w.imr().set_bit());
        // Turn display ON
        ltdc.gcr.modify(|_, w| w.ltdcen().set_bit().den().set_bit());

        // Reload ltdc config immediatly
        ltdc.srcr.modify(|_, w| w.imr().set_bit());

        DisplayController {
            _ltdc: ltdc,
            config,
            buffer1: None,
            buffer2: None,
        }
    }

    /// Configure a layer (layer 1 or layer 2)
    ///
    /// Note : the choice is made (for the sake of simplicity) to make the layer
    /// as big as the screen
    ///
    /// Color Keying and CLUT are not yet supported
    pub fn config_layer(
        &mut self,
        layer: Layer,
        pixel_format: PixelFormat,
        buffer: &'static mut [T],
    ) {
        let config: &DisplayConfig = &self.config;

        let layer: &LAYER = match layer {
            Layer::L1 => &self._ltdc.layer1,
            Layer::L2 => &self._ltdc.layer2,
        };

        // Horizontal and vertical window (coordinates include porches): where
        // in the time frame the layer values should be sent
        let h_win_start = config.h_sync + config.h_back_porch - 1;
        let v_win_start = config.v_sync + config.v_back_porch - 1;

        layer.whpcr.write(|w| unsafe {
            w.whstpos()
                .bits(h_win_start + 1)
                .whsppos()
                .bits(h_win_start + config.active_width)
        });
        layer.wvpcr.write(|w| unsafe {
            w.wvstpos()
                .bits(v_win_start + 1)
                .wvsppos()
                .bits(v_win_start + config.active_height)
        });

        // Set pixel format
        layer.pfcr.write(|w| unsafe {
            w.pf().bits(match pixel_format {
                PixelFormat::ARGB8888 => 0b000,
                // PixelFormat::RGB888 => 0b001,
                PixelFormat::RGB565 => 0b010,
                PixelFormat::ARGB1555 => 0b011,
                PixelFormat::ARGB4444 => 0b100,
                PixelFormat::L8 => 0b101,
                PixelFormat::AL44 => 0b110,
                PixelFormat::AL88 => 0b111,
                _ => unimplemented!(),
            })
        });

        // Set global alpha value to 1 (255/255). Used for layer blending.
        layer.cacr.write(|w| unsafe { w.consta().bits(0xFF) });

        // Set default color to plain (not transparent) red (for debug
        // purposes). The default color is used outside the defined layer window
        // or when a layer is disabled.
        layer.dccr.write(|w| unsafe { w.bits(0xFFFF0000) });

        // Blending factor: how the layer is combined with the layer below it
        // (layer 2 with layer 1 or layer 1 with background). Here it is set so
        // that the blending factor does not take the pixel alpha value, just
        // the global value of the layer
        layer
            .bfcr
            .write(|w| unsafe { w.bf1().bits(0b100).bf2().bits(0b101) });

        // Color frame buffer start address
        layer
            .cfbar
            .write(|w| unsafe { w.cfbadd().bits(buffer.as_ptr() as u32) });

        // Color frame buffer line length (active*byte per pixel + 3), and pitch
        let byte_per_pixel: u16 = match pixel_format {
            PixelFormat::ARGB8888 => 4,
            // PixelFormat::RGB888 => 24, unsupported for now because u24 does not exist
            PixelFormat::RGB565 => 2,
            PixelFormat::ARGB1555 => 2,
            PixelFormat::ARGB4444 => 16,
            PixelFormat::L8 => 1,
            PixelFormat::AL44 => 1,
            PixelFormat::AL88 => 2,
            _ => unimplemented!(),
        };
        layer.cfblr.write(|w| unsafe {
            w.cfbp()
                .bits(config.active_width * byte_per_pixel)
                .cfbll()
                .bits(config.active_width * byte_per_pixel + 3)
        });

        // Frame buffer number of lines
        layer
            .cfblnr
            .write(|w| unsafe { w.cfblnbr().bits(config.active_height) });

        // No Color Lookup table (CLUT)
        layer.cr.modify(|_, w| w.cluten().clear_bit());

        self.buffer1 = Some(buffer);

        self.reload();
    }

    /// Enable a layer (layer 1 or layer 2)
    pub fn enable_layer(&mut self, layer: Layer) {
        let layer: &LAYER = match layer {
            Layer::L1 => &self._ltdc.layer1,
            Layer::L2 => &self._ltdc.layer2,
        };

        // Layer enable
        layer.cr.modify(|_, w| w.len().set_bit());
    }

    /// Reload display controller immediatly
    pub fn reload(&self) {
        // Reload ltdc config immediatly
        self._ltdc.srcr.modify(|_, w| w.imr().set_bit());
    }

    /// Draw a pixel at position (x,y) on the given layer
    pub fn draw_pixel(&mut self, layer: Layer, x: usize, y: usize, color: T) {
        if x >= self.config.active_width as usize || y >= self.config.active_height as usize {
            loop {}
        }

        match layer {
            Layer::L1 => {
                self.buffer1.as_mut().unwrap()[x + self.config.active_width as usize * y] = color
            }
            Layer::L2 => {
                self.buffer2.as_mut().unwrap()[x + self.config.active_width as usize * y] = color
            }
        }
    }
}

/// Available PixelFormats to work with
///
/// Notes :
/// * `L8`: 8-bit luminance or CLUT
/// * `AL44`: 4-bit alpha + 4-bit luminance
/// * `AL88`: 8-bit alpha + 8-bit luminance
pub enum PixelFormat {
    ARGB8888,
    // RGB888(u24) unsupported for now because u24 does not exist
    RGB565,
    ARGB1555,
    ARGB4444,
    L8,
    AL44,
    AL88,
}

pub trait SupportedWord {}
impl SupportedWord for u8 {}
impl SupportedWord for u16 {}
impl SupportedWord for u32 {}
