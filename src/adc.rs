//! Analog to Digital Converter (ADC)
//!
//! # Examples
//!#![deny(missing_docs)]

/*
    Currently unused but this is the formula for using temperature calibration:
    Temperature in °C = ( ( (TS_CAL2_TEMP-TS_CAL1_TEMP) / (TS_CAL2-TS_CAL1) ) * (TS_DATA-TS_CAL1) ) + 30°C
*/
use crate::hal::adc::{Channel, OneShot};
use crate::hal::blocking::delay::DelayUs;

use core::convert::Infallible;
use core::marker::PhantomData;

use nb::block;

use crate::stm32::{ADC12_COMMON};
use crate::stm32::{ADC1, ADC2};

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
use crate::stm32::{ADC345_COMMON};
#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
use crate::stm32::{ADC3};

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g491", feature = "stm32g4A1")))]
use crate::stm32::{ADC4, ADC5};

use crate::gpio::{self, Analog};
use crate::pwr::{current_vos, VoltageScale};
use crate::rcc::rec::AdcClkSelGetter;
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

trait NumberOfBits {
    fn number_of_bits(&self) -> u32;
}

impl NumberOfBits for Resolution {
    fn number_of_bits(&self) -> u32 {
        match *self {
            Resolution::EightBit => 8,
            Resolution::TenBit => 10,
            Resolution::TwelveBit => 12,
            _ => 12,
        }
    }
}

/// Enabled ADC (type state)
pub struct Enabled;
/// Disabled ADC (type state)
pub struct Disabled;

pub trait ED {}
impl ED for Enabled {}
impl ED for Disabled {}

pub struct Adc<ADC, ED> {
    rb: ADC,
    sample_time: AdcSampleTime,
    resolution: Resolution,
    lshift: AdcLshift,
    clock: Hertz,
    current_channel: Option<u8>,
    _enabled: PhantomData<ED>,
}

/// ADC DMA modes
///
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcDmaMode {
    OneShot,
    Circular,
}

/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
//
// Refer to RM0440 Rev 7 - Chapter 21.4.12
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(non_camel_case_types)]
pub enum AdcSampleTime {
    /// 2.5 cycles sampling time
    T_2,
    /// 6.5 cycles sampling time
    T_6,
    /// 12.5 cycles sampling time
    T_12,
    /// 24.5 cycles sampling time
    T_24,
    /// 47.5 cycles sampling time
    T_47,
    /// 92.5 cycles sampling time
    T_92,
    /// 247.5 cycles sampling time
    T_247,
    /// 640.5 cycles sampling time
    T_640,
}

impl Default for AdcSampleTime {
    fn default() -> Self {
        AdcSampleTime::T_32
    }
}
impl AdcSampleTime {
    /// Returns the number of half clock cycles represented by this sampling time
    fn clock_cycles_x2(&self) -> u32 {
        let x = match self {
            AdcSampleTime::T_2 => 2,
            AdcSampleTime::T_6 => 6,
            AdcSampleTime::T_12 => 12,
            AdcSampleTime::T_24 => 24,
            AdcSampleTime::T_47 => 47,
            AdcSampleTime::T_92 => 92,
            AdcSampleTime::T_247 => 247,
            AdcSampleTime::T_640 => 640,
        };
        (2 * x) + 1
    }
}

// Refer to RM0440 Rev 7 - Chapter 21.4.12
impl From<AdcSampleTime> for u8 {
    fn from(val: AdcSampleTime) -> u8 {
        match val {
            AdcSampleTime::T_2 => 0b000,
            AdcSampleTime::T_6 => 0b001,
            AdcSampleTime::T_12 => 0b010,
            AdcSampleTime::T_24 => 0b011,
            AdcSampleTime::T_47 => 0b100,
            AdcSampleTime::T_92 => 0b101,
            AdcSampleTime::T_247 => 0b110,
            AdcSampleTime::T_640 => 0b111,
        }
    }
}

/// ADC LSHIFT\[3:0\] of the converted value
///
/// Only values in range of 0..=15 are allowed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcLshift(u8);

impl AdcLshift {
    pub fn new(lshift: u8) -> Self {
        if lshift > 15 {
            panic!("LSHIFT[3:0] must be in range of 0..=15");
        }

        AdcLshift(lshift)
    }

    pub fn value(self) -> u8 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalOffset(u16);

impl AdcCalOffset {
    pub fn value(self) -> u16 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalLinear([u32; 6]);

impl AdcCalLinear {
    pub fn value(self) -> [u32; 6] {
        self.0
    }
}

macro_rules! adc_pins {
    ($ADC:ident, $($input:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $input {
                type ID = u8;

                fn channel() -> u8 {
                    $chan
                }
            }
        )+
    };
}

macro_rules! adc_internal {
    ([$INT_ADC:ident, $INT_ADC_COMMON:ident]; $($input:ty => ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            impl $input {
                pub fn new() -> Self {
                    Self {}
                }

                /// Enables the internal voltage/sensor
                /// ADC must be disabled.
                pub fn enable(&mut self, _adc: &Adc<$INT_ADC, Disabled>) {

                    let common = unsafe { &*$INT_ADC_COMMON::ptr() };

                    common.ccr.modify(|_, w| w.$en().enabled());
                }
                /// Disables the internal voltage/sdissor
                /// ADC must be disabled.
                pub fn disable(&mut self, _adc: &Adc<$INT_ADC, Disabled>) {

                    let common = unsafe { &*$INT_ADC_COMMON::ptr() };

                    common.ccr.modify(|_, w| w.$en().disabled());
                }
            }

            adc_pins!($INT_ADC, $input => $chan);
        )+
    };
}

/// Vref internal signal
#[derive(Default)]
pub struct Vrefint;
/// Vbat internal signal
#[derive(Default)]
pub struct Vbat;
/// Internal temperature sensor
#[derive(Default)]
pub struct Temperature;

// Just implmenting INPx pins (INNx defaulting to V_ref-)
//
adc_pins!(ADC1,
    // Channel 0 connected to Vssa internaly
    gpio::PA0<Analog> => 1,
    gpio::PA1<Analog> => 2,
    gpio::PA2<Analog> => 3,
    gpio::PA3<Analog> => 4,
    gpio::PB14<Analog> => 5,
    gpio::PC0<Analog> => 6,
    gpio::PC1<Analog> => 7,
    gpio::PC2<Analog> => 8,
    gpio::PC3<Analog> => 9,
    gpio::PF0<Analog> => 10,
    gpio::PB12<Analog> => 11,
    gpio::PB1<Analog> => 12,
    // Channel 13 can be connected to Vopamp1 internal
    gpio::PB11<Analog> => 14,
    gpio::PB0<Analog> => 15,
    // Channel 16 can be connected to VTS
    // Channel 17 can be connected to VBAT/3
    // Channel 18 can be connected to VREFINT
);

adc_internal!(
    [ADC1, ADC12_COMMON];

    Temperature => (16, vsenseen),
    Vbat => (17, vbaten),
    Vrefint => (18, vrefen)
);

adc_pins!(ADC2,
    // Channel 0 connected to Vssa internaly
    gpio::PA0<Analog> => 1,
    gpio::PA1<Analog> => 2,
    gpio::PA6<Analog> => 3,
    gpio::PA7<Analog> => 4,
    gpio::PC4<Analog> => 5,
    gpio::PC0<Analog> => 6,
    gpio::PC1<Analog> => 7,
    gpio::PC2<Analog> => 8,
    gpio::PC3<Analog> => 9,
    gpio::PF1<Analog> => 10,
    gpio::PC5<Analog> => 11,
    gpio::PB2<Analog> => 12,
    gpio::PA5<Analog> => 13,
    gpio::PB11<Analog> => 14,
    gpio::PB15<Analog> => 15,
    // Channel 16 can be connected to Vopamp2 internal
    gpio::PA4<Analog> => 17,
    // Channel 18 can be connected to Vopamp3 internal
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
adc_pins!(ADC3,
    // Channel 0 connected to Vssa internaly
    gpio::PB1<Analog> => 1,
    gpio::PE9<Analog> => 2,
    gpio::PE13<Analog> => 3,
    gpio::PE7<Analog> => 4,
    gpio::PB13<Analog> => 5,
    gpio::PE8<Analog> => 6,
    gpio::PD10<Analog> => 7,
    gpio::PD11<Analog> => 8,
    gpio::PD12<Analog> => 9,
    gpio::PD13<Analog> => 10,
    gpio::PD14<Analog> => 11,
    gpio::PB0<Analog> => 12,
    // Channel 13 can be connected to Vopamp3 internal
    gpio::PE10<Analog> => 14,
    gpio::PE11<Analog> => 15,
    gpio::PE12<Analog> => 16,
    // Channel 17 can be connected to VBAT/3 on cat 3 devices or Vopamp6 on cat 6 devices internally
    // Channel 18 can be connected to VREFINT
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
adc_internal!(
    [ADC3, ADC345_COMMON];

    Vrefint => (18, vrefen)
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g491", feature = "stm32g4A1")))]
adc_pins!(ADC4,
    // Channel 0 connected to Vssa internaly
    gpio::PE14<Analog> => 1,
    gpio::PE15<Analog> => 2,
    gpio::PB12<Analog> => 3,
    gpio::PB14<Analog> => 4,
    gpio::PB15<Analog> => 5,
    gpio::PE8<Analog> => 6,
    gpio::PD10<Analog> => 7,
    gpio::PD11<Analog> => 8,
    gpio::PD12<Analog> => 9,
    gpio::PD13<Analog> => 10,
    gpio::PD14<Analog> => 11,
    gpio::PD8<Analog> => 12,
    gpio::PD9<Analog> => 13,
    gpio::PE10<Analog> => 14,
    gpio::PE11<Analog> => 15,
    gpio::PE12<Analog> => 16,
    // Channel 17 can be connected to  Vopamp6
    // Channel 18 can be connected to VREFINT
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g491", feature = "stm32g4A1")))]
adc_internal!(
    [ADC4, ADC345_COMMON];

    Vrefint => (18, vrefen)
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g491", feature = "stm32g4A1")))]
adc_pins!(ADC5,
    // Channel 0 connected to Vssa internaly
    gpio::PA8<Analog> => 1,
    gpio::PA9<Analog> => 2,
    // Channel 3 can be connected to  Vopamp5
    // Channel 4 can be connected to VTS
    // Channel 5 can be connected to  Vopamp4
    gpio::PE8<Analog> => 6,
    gpio::PD10<Analog> => 7,
    gpio::PD11<Analog> => 8,
    gpio::PD12<Analog> => 9,
    gpio::PD13<Analog> => 10,
    gpio::PD14<Analog> => 11,
    gpio::PD8<Analog> => 12,
    gpio::PD9<Analog> => 13,
    gpio::PE10<Analog> => 14,
    gpio::PE11<Analog> => 15,
    gpio::PE12<Analog> => 16,
    // Channel 17 can be connected to VBAT/3
    // Channel 18 can be connected to VREFINT
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g491", feature = "stm32g4A1")))]
adc_internal!(
    [ADC5, ADC345_COMMON];

    Temperature => (4, vsenseen),
    Vbat => (17, vbaten),
    Vrefint => (18, vrefen)
);

pub trait AdcExt<ADC>: Sized {
    type Rec: ResetEnable;

    fn adc(
        self,
        f_adc: impl Into<Hertz>,
        delay: &mut impl DelayUs<u8>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Adc<ADC, Disabled>;
}

/// Stored ADC config can be restored using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredConfig(AdcSampleTime, Resolution, AdcLshift);

#[cfg(feature = "defmt")]
impl defmt::Format for StoredConfig {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "StoredConfig({:?}, {:?}, {:?})",
            self.0,
            defmt::Debug2Format(&self.1),
            self.2
        )
    }
}

/// Returns the frequency of the current adc_ker_ck
///
/// # Panics
///
/// Panics if the kernel clock is not running
fn kernel_clk_unwrap(
    prec: &impl AdcClkSelGetter,
    clocks: &CoreClocks,
) -> Hertz {
    match prec.get_kernel_clk_mux() {
        Some(rec::AdcClkSel::SYSCLK) => {
            clocks.sysckl().expect("ADC: SYSCLK must be enabled")
        }
        Some(rec::AdcClkSel::PllP) => {
            clocks.pll_p_ck().expect("ADC: PLL_P must be enabled")
        }
        _ => unreachable!(),
    }
}

#[allow(unused_macros)]
macro_rules! adc_hal {
    ($(
        $ADC:ident, $ADC_COMMON:ident: (
            $adcX: ident,
            $Rec:ident
        )
    ),+ $(,)*) => {
        $(
            impl AdcExt<$ADC> for $ADC {
                type Rec = rec::$Rec;

	            fn adc(self,
                       f_adc: impl Into<Hertz>,
                       delay: &mut impl DelayUs<u8>,
                       prec: rec::$Rec,
                       clocks: &CoreClocks) -> Adc<$ADC, Disabled>
	            {
	                Adc::$adcX(self, f_adc, delay, prec, clocks)
	            }
	        }

            impl Adc<$ADC, Disabled> {
                /// Initialise ADC
                ///
                /// Sets all configurable parameters to one-shot defaults,
                /// performs a boot-time calibration.
                pub fn $adcX(adc: $ADC, f_adc: impl Into<Hertz>, delay: &mut impl DelayUs<u8>,
                             prec: rec::$Rec, clocks: &CoreClocks
                ) -> Self {
                    // Consume ADC register block, produce Self with default
                    // settings
                    let mut adc = Self::default_from_rb(adc);

                    // Enable AHB clock
                    let prec = prec.enable();

                    // Power Down
                    adc.power_down();

                    // Reset peripheral
                    //let prec = prec.reset();

                    // Power Up, Preconfigure and Calibrate
                    adc.power_up(delay);
                    adc.configure_clock(f_adc.into(), prec, clocks);
                    adc.preconfigure();
                    adc.calibrate();

                    adc
                }
                /// Creates ADC with default settings
                fn default_from_rb(rb: $ADC) -> Self {
                    Self {
                        rb,
                        sample_time: AdcSampleTime::default(),
                        resolution: Resoыlution::SixteenBit,
                        lshift: AdcLshift::default(),
                        clock: Hertz::from_raw(0),
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
                /// Sets the clock configuration for this ADC. This is common
                /// between ADC1 and ADC2, so the prec block is used to ensure
                /// this method can only be called on one of the ADCs (or both,
                /// using the [adc12](#method.adc12) method).
                ///
                /// Only `CKMODE[1:0]` = 0 is supported
                fn configure_clock(&mut self, f_adc: Hertz, prec: rec::$Rec, clocks: &CoreClocks) -> Hertz {
                    let ker_ck = kernel_clk_unwrap(&prec, clocks);

                    let max_ker_ck = match current_vos() {
                        // See RM0468 Rev 3 Table 56.
                        #[cfg(feature = "rm0468")]
                        VoltageScale::Scale0 | VoltageScale::Scale1 => 160_000_000,
                        #[cfg(feature = "rm0468")]
                        VoltageScale::Scale2 => 60_000_000,
                        #[cfg(feature = "rm0468")]
                        VoltageScale::Scale3 => 40_000_000,

                        // See RM0433 Rev 7 Table 59.
                        #[cfg(not(feature = "rm0468"))]
                        VoltageScale::Scale0 | VoltageScale::Scale1 => 80_000_000,
                        #[cfg(not(feature = "rm0468"))]
                        VoltageScale::Scale2 | VoltageScale::Scale3 => 40_000_000
                    };
                    assert!(ker_ck.raw() <= max_ker_ck,
                            "Kernel clock violates maximum frequency defined in Reference Manual. \
                             Can result in erroneous ADC readings");

                    let f_adc = self.configure_clock_unchecked(f_adc, prec, clocks);

                    // Maximum ADC clock speed. With BOOST = 0 there is a no
                    // minimum frequency given in part datasheets
                    assert!(f_adc.raw() <= 50_000_000);

                    f_adc
                }

                /// No clock checks
                fn configure_clock_unchecked(&mut self, f_adc: Hertz, prec: rec::$Rec, clocks: &CoreClocks) -> Hertz {
                    let ker_ck = kernel_clk_unwrap(&prec, clocks);

                    // Target mux output. See RM0433 Rev 7 - Figure 136.
                    #[cfg(feature = "revision_v")]
                    let f_target = f_adc.raw() * 2;

                    #[cfg(not(feature = "revision_v"))]
                    let f_target = f_adc.raw();

                    let (divider, presc) = match (ker_ck.raw() + f_target - 1) / f_target {
                        1 => (1, PRESC_A::Div1),
                        2 => (2, PRESC_A::Div2),
                        3..=4 => (4, PRESC_A::Div4),
                        5..=6 => (6, PRESC_A::Div6),
                        7..=8 => (8, PRESC_A::Div8),
                        9..=10 => (10, PRESC_A::Div10),
                        11..=12 => (12, PRESC_A::Div12),
                        13..=16 => (16, PRESC_A::Div16),
                        17..=32 => (32, PRESC_A::Div32),
                        33..=64 => (64, PRESC_A::Div64),
                        65..=128 => (128, PRESC_A::Div128),
                        129..=256 => (256, PRESC_A::Div256),
                        _ => panic!("Selecting the ADC clock required a prescaler > 256, \
                                     which is not possible in hardware. Either increase the ADC \
                                     clock frequency or decrease the kernel clock frequency"),
                    };
                    unsafe { &*$ADC_COMMON::ptr() }.ccr.modify(|_, w| w.presc().variant(presc));

                    // Calculate actual value. See RM0433 Rev 7 - Figure 136.
                    #[cfg(feature = "revision_v")]
                    let f_adc = Hertz::from_raw(ker_ck.raw() / (divider * 2));

                    // Calculate actual value Revison Y. See RM0433 Rev 7 - Figure 137.
                    #[cfg(not(feature = "revision_v"))]
                    let f_adc = Hertz::from_raw(ker_ck.raw() / divider);

                    self.clock = f_adc;
                    f_adc
                }

                /// Disables Deeppowerdown-mode and enables voltage regulator
                ///
                /// Note: After power-up, a [`calibration`](#method.calibrate) shall be run
                pub fn power_up(&mut self, delay: &mut impl DelayUs<u8>) {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.6
                    self.rb.cr.modify(|_, w|
                        w.deeppwd().clear_bit()
                            .advregen().set_bit()
                    );
                    delay.delay_us(10_u8);

                    // check LDORDY bit if present
                    $(
                        #[cfg(feature = "revision_v")]
                        while {
                            let $ldordy = self.rb.isr.read().bits() & 0x1000;
                            $ldordy == 0
                        }{}
                    )*
                }

                /// Enables Deeppowerdown-mode and disables voltage regulator
                ///
                /// Note: This resets the [`calibration`](#method.calibrate) of the ADC
                pub fn power_down(&mut self) {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.6
                    self.rb.cr.modify(|_, w|
                        w.deeppwd().set_bit()
                            .advregen().clear_bit()
                    );
                }

                /// Calibrates the ADC in single channel mode
                ///
                /// Note: The ADC must be disabled
                pub fn calibrate(&mut self) {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.8
                    self.check_calibration_conditions();

                    // single channel (INNx equals to V_ref-)
                    self.rb.cr.modify(|_, w|
                        w.adcaldif().clear_bit()
                            .adcallin().set_bit()
                    );
                    // calibrate
                    self.rb.cr.modify(|_, w| w.adcal().set_bit());
                    while self.rb.cr.read().adcal().bit_is_set() {}
                }

                fn check_calibration_conditions(&self) {
                    let cr = self.rb.cr.read();
                    if cr.aden().bit_is_set() {
                        panic!("Cannot start calibration when the ADC is enabled");
                    }
                    if cr.deeppwd().bit_is_set() {
                        panic!("Cannot start calibration when the ADC is in deeppowerdown-mode");
                    }
                    if cr.advregen().bit_is_clear() {
                        panic!("Cannot start calibration when the ADC voltage regulator is disabled");
                    }
                }

                /// Configuration process prior to enabling the ADC
                ///
                /// Note: the ADC must be disabled
                fn preconfigure(&mut self) {
                    self.configure_channels_dif_mode();
                }

                /// Sets channels to single ended mode
                fn configure_channels_dif_mode(&mut self) {
                    self.rb.difsel.reset();
                }

                /// Configuration process immediately after enabling the ADC
                fn configure(&mut self) {
                    // Single conversion mode, Software trigger
                    // Refer to RM0433 Rev 7 - Chapters 25.4.15, 25.4.19
                    self.rb.cfgr.modify(|_, w|
                        w.cont().clear_bit()
                            .exten().disabled()
                            .discen().set_bit()
                    );

                    // Enables boost mode for highest possible clock frequency
                    //
                    // Refer to RM0433 Rev 7 - Chapter 25.4.3
                    #[cfg(not(feature = "revision_v"))]
                    self.rb.cr.modify(|_, w| w.boost().set_bit());
                    #[cfg(feature = "revision_v")]
                    self.rb.cr.modify(|_, w| {
                        if self.clock.raw() <= 6_250_000 {
                            w.boost().lt6_25()
                        } else if self.clock.raw() <= 12_500_000 {
                            w.boost().lt12_5()
                        } else if self.clock.raw() <= 25_000_000 {
                            w.boost().lt25()
                        } else {
                            w.boost().lt50()
                        }
                    });
                }

                /// Enable ADC
                pub fn enable(mut self) -> Adc<$ADC, Enabled> {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.9
                    self.rb.isr.modify(|_, w| w.adrdy().set_bit());
                    self.rb.cr.modify(|_, w| w.aden().set_bit());
                    while self.rb.isr.read().adrdy().bit_is_clear() {}
                    self.rb.isr.modify(|_, w| w.adrdy().set_bit());

                    self.configure();

                    Adc {
                        rb: self.rb,
                        sample_time: self.sample_time,
                        resolution: self.resolution,
                        lshift: self.lshift,
                        clock: self.clock,
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
            }

            impl Adc<$ADC, Enabled> {
                fn stop_regular_conversion(&mut self) {
                    self.rb.cr.modify(|_, w| w.adstp().set_bit());
                    while self.rb.cr.read().adstp().bit_is_set() {}
                }

                fn stop_injected_conversion(&mut self) {
                    self.rb.cr.modify(|_, w| w.jadstp().set_bit());
                    while self.rb.cr.read().jadstp().bit_is_set() {}
                }

                fn set_chan_smp(&mut self, chan: u8) {
                    let t = self.get_sample_time().into();
                    if chan <= 9 {
                        self.rb.smpr1.modify(|_, w| match chan {
                            0 => w.smp0().bits(t),
                            1 => w.smp1().bits(t),
                            2 => w.smp2().bits(t),
                            3 => w.smp3().bits(t),
                            4 => w.smp4().bits(t),
                            5 => w.smp5().bits(t),
                            6 => w.smp6().bits(t),
                            7 => w.smp7().bits(t),
                            8 => w.smp8().bits(t),
                            9 => w.smp9().bits(t),
                            _ => unreachable!(),
                        })
                    } else {
                        self.rb.smpr2.modify(|_, w| match chan {
                            10 => w.smp10().bits(t),
                            11 => w.smp11().bits(t),
                            12 => w.smp12().bits(t),
                            13 => w.smp13().bits(t),
                            14 => w.smp14().bits(t),
                            15 => w.smp15().bits(t),
                            16 => w.smp16().bits(t),
                            17 => w.smp17().bits(t),
                            18 => w.smp18().bits(t),
                            19 => w.smp19().bits(t),
                            _ => unreachable!(),
                        })
                    }
                }

                // This method starts a conversion sequence on the given channel
                fn start_conversion_common(&mut self, chan: u8) {
                    self.check_conversion_conditions();

                    // Set LSHIFT[3:0]
                    self.rb.cfgr2.modify(|_, w| w.lshift().bits(self.get_lshift().value()));

                    // Select channel (with preselection, refer to RM0433 Rev 7 - Chapter 25.4.12)
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() | (1 << chan)) });
                    self.set_chan_smp(chan);
                    self.rb.sqr1.modify(|_, w| unsafe {
                        w.sq1().bits(chan)
                            .l().bits(0)
                    });
                    self.current_channel = Some(chan);

                    // Perform conversion
                    self.rb.cr.modify(|_, w| w.adstart().set_bit());
                }

                /// Start conversion
                ///
                /// This method starts a conversion sequence on the given pin.
                /// The value can be then read through the `read_sample` method.
                // Refer to RM0433 Rev 7 - Chapter 25.4.16
                pub fn start_conversion<PIN>(&mut self, _pin: &mut PIN)
                    where PIN: Channel<$ADC, ID = u8>,
                {
                    let chan = PIN::channel();
                    assert!(chan <= 19);

                    // Set resolution
                    self.rb.cfgr.modify(|_, w| unsafe { w.res().bits(self.get_resolution().into()) });
                    // Set discontinuous mode
                    self.rb.cfgr.modify(|_, w| w.cont().clear_bit().discen().set_bit());

                    self.start_conversion_common(chan);
                }

                /// Start conversion in DMA mode
                ///
                /// This method starts a conversion sequence with DMA
                /// enabled. The DMA mode selected depends on the [`AdcDmaMode`] specified.
                pub fn start_conversion_dma<PIN>(&mut self, _pin: &mut PIN, mode: AdcDmaMode)
                    where PIN: Channel<$ADC, ID = u8>,
                {
                    let chan = PIN::channel();
                    assert!(chan <= 19);

                    // Set resolution
                    self.rb.cfgr.modify(|_, w| unsafe { w.res().bits(self.get_resolution().into()) });


                    self.rb.cfgr.modify(|_, w| w.dmngt().bits(match mode {
                        AdcDmaMode::OneShot => 0b01,
                        AdcDmaMode::Circular => 0b11,
                    }));

                    // Set continuous mode
                    self.rb.cfgr.modify(|_, w| w.cont().set_bit().discen().clear_bit() );

                    self.start_conversion_common(chan);
                }


                /// Read sample
                ///
                /// `nb::Error::WouldBlock` in case the conversion is still
                /// progressing.
                // Refer to RM0433 Rev 7 - Chapter 25.4.16
                pub fn read_sample(&mut self) -> nb::Result<u32, Infallible> {
                    let chan = self.current_channel.expect("No channel was selected, use start_conversion first");

                    // Check if the conversion is finished
                    if self.rb.isr.read().eoc().bit_is_clear() {
                        return Err(nb::Error::WouldBlock);
                    }

                    // Disable preselection of this channel, refer to RM0433 Rev 7 - Chapter 25.4.12
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() & !(1 << chan)) });
                    self.current_channel = None;

                    // Retrieve result
                    let result = self.rb.dr.read().bits();
                    nb::Result::Ok(result)
                }

                fn check_conversion_conditions(&self) {
                    let cr = self.rb.cr.read();
                    // Ensure that no conversions are ongoing
                    if cr.adstart().bit_is_set() {
                        panic!("Cannot start conversion because a regular conversion is ongoing");
                    }
                    if cr.jadstart().bit_is_set() {
                        panic!("Cannot start conversion because an injected conversion is ongoing");
                    }
                    // Ensure that the ADC is enabled
                    if cr.aden().bit_is_clear() {
                        panic!("Cannot start conversion because ADC is currently disabled");
                    }
                    if cr.addis().bit_is_set() {
                        panic!("Cannot start conversion because there is a pending request to disable the ADC");
                    }
                }

                /// Disable ADC
                pub fn disable(mut self) -> Adc<$ADC, Disabled> {
                    let cr = self.rb.cr.read();
                    // Refer to RM0433 Rev 7 - Chapter 25.4.9
                    if cr.adstart().bit_is_set() {
                        self.stop_regular_conversion();
                    }
                    if cr.jadstart().bit_is_set() {
                        self.stop_injected_conversion();
                    }

                    self.rb.cr.modify(|_, w| w.addis().set_bit());
                    while self.rb.cr.read().aden().bit_is_set() {}

                    Adc {
                        rb: self.rb,
                        sample_time: self.sample_time,
                        resolution: self.resolution,
                        lshift: self.lshift,
                        clock: self.clock,
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
            }

            impl<ED> Adc<$ADC, ED> {
                /// Save current ADC config
                pub fn save_cfg(&mut self) -> StoredConfig {
                    StoredConfig(self.get_sample_time(), self.get_resolution(), self.get_lshift())
                }

                /// Restore saved ADC config
                pub fn restore_cfg(&mut self, cfg: StoredConfig) {
                    self.set_sample_time(cfg.0);
                    self.set_resolution(cfg.1);
                    self.set_lshift(cfg.2);
                }

                /// Reset the ADC config to default, return existing config
                pub fn default_cfg(&mut self) -> StoredConfig {
                    let cfg = self.save_cfg();
                    self.set_sample_time(AdcSampleTime::default());
                    self.set_resolution(Resolution::SixteenBit);
                    self.set_lshift(AdcLshift::default());
                    cfg
                }

                /// The current ADC clock frequency. Defined as f_ADC in device datasheets
                ///
                /// The value returned by this method will always be equal or
                /// lower than the `f_adc` passed to [`init`](#method.init)
                pub fn clock_frequency(&self) -> Hertz {
                    self.clock
                }

                /// The current ADC sampling frequency. This is the reciprocal of Tconv
                pub fn sampling_frequency(&self) -> Hertz {
                    let sample_cycles_x2 = self.sample_time.clock_cycles_x2();

                    // TODO: Exception for RM0468 ADC3
                    // let sar_cycles_x2 = match self.resolution {
                    //     Resolution::SixBit => 13, // 6.5
                    //     Resolution::EightBit => 17, // 8.5
                    //     Resolution::TenBit => 21,   // 10.5
                    //     _ => 25,                    // 12.5
                    // };

                    let sar_cycles_x2 = match self.resolution {
                        Resolution::EightBit => 9, // 4.5
                        Resolution::TenBit => 11,  // 5.5
                        Resolution::TwelveBit => 13, // 6.5
                        Resolution::FourteenBit => 15, // 7.5
                        _ => 17,                       // 8.5
                    };

                    let cycles = (sample_cycles_x2 + sar_cycles_x2) / 2;
                    self.clock / cycles
                }

                /// Get ADC samping time
                pub fn get_sample_time(&self) -> AdcSampleTime {
                    self.sample_time
                }

                /// Get ADC sampling resolution
                pub fn get_resolution(&self) -> Resolution {
                    self.resolution
                }

                /// Get ADC lshift value
                pub fn get_lshift(&self) -> AdcLshift {
                    self.lshift
                }

                /// Set ADC sampling time
                ///
                /// Options can be found in [AdcSampleTime](crate::adc::AdcSampleTime).
                pub fn set_sample_time(&mut self, t_samp: AdcSampleTime) {
                    self.sample_time = t_samp;
                }

                /// Set ADC sampling resolution
                pub fn set_resolution(&mut self, res: Resolution) {
                    self.resolution = res;
                }

                /// Set ADC lshift
                ///
                /// LSHIFT\[3:0\] must be in range of 0..=15
                pub fn set_lshift(&mut self, lshift: AdcLshift) {
                    self.lshift = lshift;
                }

                /// Returns the largest possible sample value for the current ADC configuration
                ///
                /// Using this value as the denominator when calculating
                /// transfer functions results in a gain error, and thus should
                /// be avoided. Use the [slope](#method.slope) method instead.
                #[deprecated(since = "0.12.0", note = "See the slope() method instead")]
                pub fn max_sample(&self) -> u32 {
                    ((1 << self.get_resolution().number_of_bits() as u32) - 1) << self.get_lshift().value() as u32
                }

                /// Returns the slope for the current ADC configuration. 1 LSB = Vref / slope
                ///
                /// This value can be used in calcuations involving the transfer function of
                /// the ADC. For example, to calculate an estimate for the
                /// applied voltage of an ADC channel referenced to voltage
                /// `vref`
                ///
                /// ```
                /// let v = adc.read(&ch).unwrap() as f32 * vref / adc.slope() as f32;
                /// ```
                pub fn slope(&self) -> u32 {
                    1 << (self.get_resolution().number_of_bits() as u32 + self.get_lshift().value() as u32)
                }


                /// Returns the offset calibration value for single ended channel
                pub fn read_offset_calibration_value(&self) -> AdcCalOffset {
                    AdcCalOffset(self.rb.calfact.read().calfact_s().bits())
                }

                /// Returns the linear calibration values stored in an array in the following order:
                /// LINCALRDYW1 -> result\[0\]
                /// ...
                /// LINCALRDYW6 -> result\[5\]
                pub fn read_linear_calibration_values(&mut self) -> AdcCalLinear {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.8
                    self.check_linear_read_conditions();

                    // Read 1st block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw1().clear_bit());
                    while self.rb.cr.read().lincalrdyw1().bit_is_set() {}
                    let res_1 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 2nd block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw2().clear_bit());
                    while self.rb.cr.read().lincalrdyw2().bit_is_set() {}
                    let res_2 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 3rd block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw3().clear_bit());
                    while self.rb.cr.read().lincalrdyw3().bit_is_set() {}
                    let res_3 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 4th block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw4().clear_bit());
                    while self.rb.cr.read().lincalrdyw4().bit_is_set() {}
                    let res_4 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 5th block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw5().clear_bit());
                    while self.rb.cr.read().lincalrdyw5().bit_is_set() {}
                    let res_5 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 6th block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw6().clear_bit());
                    while self.rb.cr.read().lincalrdyw6().bit_is_set() {}
                    let res_6 = self.rb.calfact2.read().lincalfact().bits();

                    AdcCalLinear([res_1, res_2, res_3, res_4, res_5, res_6])
                }

                fn check_linear_read_conditions(&self) {
                    let cr = self.rb.cr.read();
                    // Ensure the ADC is enabled and is not in deeppowerdown-mode
                    if cr.deeppwd().bit_is_set() {
                        panic!("Cannot read linear calibration value when the ADC is in deeppowerdown-mode");
                    }
                    if cr.advregen().bit_is_clear() {
                        panic!("Cannot read linear calibration value when the voltage regulator is disabled");
                    }
                    if cr.aden().bit_is_clear() {
                        panic!("Cannot read linear calibration value when the ADC is disabled");
                    }
                }

                /// Returns a reference to the inner peripheral
                pub fn inner(&self) -> &$ADC {
                    &self.rb
                }

                /// Returns a mutable reference to the inner peripheral
                pub fn inner_mut(&mut self) -> &mut $ADC {
                    &mut self.rb
                }
            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC, Enabled>
            where
                WORD: From<u32>,
                PIN: Channel<$ADC, ID = u8>,
            {
                type Error = ();

                fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                    self.start_conversion(pin);
                    let res = block!(self.read_sample()).unwrap();
                    Ok(res.into())
                }
            }
        )+
    }
}

adc_hal!(
    ADC1,
    ADC12_COMMON: (adc1, Adc12),
    ADC2,
    ADC12_COMMON: (adc2, Adc12),
);

#[cfg(not(any(feature = "stm32g431", feature = "stm32g441")))]
adc_hal!(ADC3, ADC345_COMMON: (adc3, Adc345));
#[cfg(not(any(feature = "stm32g431", feature = "stm32g441", feature = "stm32g491", feature = "stm32g4A1")))]
adc_hal!(
    ADC4,
    ADC345_COMMON: (adc4, Adc345),
    ADC5,
    ADC345_COMMON: (adc5, Adc345),
);
