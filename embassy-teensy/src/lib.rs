#![no_std]
#![allow(async_fn_in_trait)]

/// Exported for RTIC. Do not use.
#[doc(hidden)]
pub use teensy4_bsp::ral::{Interrupt, NVIC_PRIO_BITS};

// Reexports
pub use teensy4_bsp::board;
pub use teensy4_bsp::ral as pac;
pub use teensy4_bsp::hal as hal;
pub use teensy4_panic;

pub mod time_driver;

embassy_hal_internal::interrupt_mod!(
    DMA0_DMA16,
    DMA1_DMA17,
    DMA2_DMA18,
    DMA3_DMA19,
    DMA4_DMA20,
    DMA5_DMA21,
    DMA6_DMA22,
    DMA7_DMA23,
    DMA8_DMA24,
    DMA9_DMA25,
    DMA10_DMA26,
    DMA11_DMA27,
    DMA12_DMA28,
    DMA13_DMA29,
    DMA14_DMA30,
    DMA15_DMA31,
    DMA_ERROR,
    LPUART1,
    LPUART2,
    LPUART3,
    LPUART4,
    LPUART5,
    LPUART6,
    LPUART7,
    LPUART8,
    LPI2C1,
    LPI2C2,
    LPI2C3,
    LPI2C4,
    LPSPI1,
    LPSPI2,
    LPSPI3,
    LPSPI4,
    CAN1,
    CAN2,
    FLEXRAM,
    KPP,
    TSC_DIG,
    GPR_IRQ,
    LCDIF,
    CSI,
    PXP,
    WDOG2,
    SNVS_HP_WRAPPER,
    SNVS_HP_WRAPPER_TZ,
    SNVS_LP_WRAPPER,
    CSU,
    DCP,
    DCP_VMI,
    TRNG,
    BEE,
    SAI1,
    SAI2,
    SAI3_RX,
    SAI3_TX,
    SPDIF,
    PMU_EVENT,
    TEMP_LOW_HIGH,
    TEMP_PANIC,
    USB_PHY1,
    USB_PHY2,
    ADC1,
    ADC2,
    DCDC,
    GPIO1_INT0,
    GPIO1_INT1,
    GPIO1_INT2,
    GPIO1_INT3,
    GPIO1_INT4,
    GPIO1_INT5,
    GPIO1_INT6,
    GPIO1_INT7,
    GPIO1_COMBINED_0_15,
    GPIO1_COMBINED_16_31,
    GPIO2_COMBINED_0_15,
    GPIO2_COMBINED_16_31,
    GPIO3_COMBINED_0_15,
    GPIO3_COMBINED_16_31,
    GPIO4_COMBINED_0_15,
    GPIO4_COMBINED_16_31,
    GPIO5_COMBINED_0_15,
    GPIO5_COMBINED_16_31,
    FLEXIO1,
    FLEXIO2,
    WDOG1,
    RTWDOG,
    EWM,
    CCM_1,
    CCM_2,
    GPC,
    SRC,
    GPT1,
    GPT2,
    PWM1_0,
    PWM1_1,
    PWM1_2,
    PWM1_3,
    PWM1_FAULT,
    FLEXSPI2,
    FLEXSPI,
    SEMC,
    USDHC1,
    USDHC2,
    USB_OTG2,
    USB_OTG1,
    ENET,
    ENET_1588_TIMER,
    XBAR1_IRQ_0_1,
    XBAR1_IRQ_2_3,
    ADC_ETC_IRQ0,
    ADC_ETC_IRQ1,
    ADC_ETC_IRQ2,
    ADC_ETC_ERROR_IRQ,
    PIT,
    ACMP1,
    ACMP2,
    ACMP3,
    ACMP4,
    ENC1,
    ENC2,
    ENC3,
    ENC4,
    TMR1,
    TMR2,
    TMR3,
    TMR4,
    PWM2_0,
    PWM2_1,
    PWM2_2,
    PWM2_3,
    PWM2_FAULT,
    PWM3_0,
    PWM3_1,
    PWM3_2,
    PWM3_3,
    PWM3_FAULT,
    PWM4_0,
    PWM4_1,
    PWM4_2,
    PWM4_3,
    PWM4_FAULT,
    ENET2,
    ENET2_1588_TIMER,
    CAN3,
    FLEXIO3,
    GPIO6_7_8_9,
);

// developer note: this macro can't be in `embassy-hal-internal` due to the use of `$crate`.
#[macro_export]
macro_rules! bind_interrupts {
    ($vis:vis struct $name:ident { $($irq:ident => $($handler:ty),*;)* }) => {
        #[derive(Copy, Clone)]
        $vis struct $name;

        $(
            #[allow(non_snake_case)]
            #[no_mangle]
            unsafe extern "C" fn $irq() {
                $(
                    <$handler as $crate::interrupt::typelevel::Handler<$crate::interrupt::typelevel::$irq>>::on_interrupt,
                )*
            }

            $(
                unsafe impl $crate::interrupt::typelevel::Binding<$crate::interrupt::typelevel::$irq, $handler> for $name {}
            )*
        )*
    };
}

/// Initialize the `embassy-teensy` HAL with the provided configuration.
///
/// This returns the peripheral singletons that can be used for creating drivers.
///
/// This should only be called once at startup, otherwise it panics.
pub fn init() -> board::Resources<teensy4_bsp::pins::t41::Pins> {
    critical_section::with(|cs| {
        let p = teensy4_bsp::board::t41(teensy4_bsp::board::instances());

        time_driver::init(cs);

        p
    })
}
