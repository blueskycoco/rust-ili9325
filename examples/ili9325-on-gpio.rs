#![no_std]
#![no_main]

use core::iter::{Cloned, Cycle};
use core::slice::Iter;

use cortex_m_rt::entry;
use panic_semihosting as _;

use display_interface_parallel_gpio::{Generic16BitBus, PGPIO16BitInterface};
use ili9325::{DisplaySize, Ili9325};
pub use ili9325::{DisplaySize240x320, DisplaySize320x480, Orientation, Scroller};
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals};
use stm32f4xx_hal::prelude::*;

#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    // Make HCLK faster to allow updating the display more quickly
    let clocks = rcc.cfgr.hclk(100.MHz()).freeze();

    let mut delay = cp.SYST.delay(&clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let bus = Generic16BitBus::new((
                gpiob.pb0.into_push_pull_output(), 
                gpiob.pb1.into_push_pull_output(), 
                gpiob.pb2.into_push_pull_output(), 
                gpiob.pb3.into_push_pull_output(), 
                gpiob.pb4.into_push_pull_output(), 
                gpiob.pb5.into_push_pull_output(), 
                gpiob.pb6.into_push_pull_output(), 
                gpiob.pb7.into_push_pull_output(), 
                gpiob.pb8.into_push_pull_output(), 
                gpiob.pb9.into_push_pull_output(), 
                gpiob.pb10.into_push_pull_output(), 
                gpiob.pb11.into_push_pull_output(), 
                gpiob.pb12.into_push_pull_output(), 
                gpiob.pb13.into_push_pull_output(), 
                gpiob.pb14.into_push_pull_output(), 
                gpiob.pb15.into_push_pull_output(),
            )).unwrap();
    let interface = PGPIO16BitInterface::new(bus, gpioc.pc8.into_push_pull_output(),
                                                gpioc.pc7.into_push_pull_output());

    let ili9325 = Ili9325::new(interface, gpioc.pc5.into_push_pull_output(),
                                gpioc.pc9.into_push_pull_output(),
                                &mut delay, Orientation::LandscapeFlipped,
                                DisplaySize240x320).unwrap();
    loop {
    }
}
