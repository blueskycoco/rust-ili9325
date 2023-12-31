#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_semihosting as _;

use display_interface_parallel_stm32_gpio::{PGPIO16BitInterface};
use ili9325::{Ili9325};
pub use ili9325::{DisplaySize240x320, DisplaySize320x480, Orientation};
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals};
use stm32f4xx_hal::prelude::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle};
use cortex_m_semihosting::hprintln;

#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    hprintln!("ahb1enr: {:#x}", dp.RCC.ahb1enr.read().bits());
    dp.RCC.ahb1enr.write(|w| w.gpioben().enabled());
    hprintln!("ahb1enr: {:#x}", dp.RCC.ahb1enr.read().bits());
    let rcc = dp.RCC.constrain();
    // Make HCLK faster to allow updating the display more quickly
    //let clocks = rcc.cfgr.hclk(180.MHz()).freeze();
    let clocks = rcc.cfgr                                                                     
        .hclk(180.MHz())                                                         
        .sysclk(180.MHz())                                                       
        .pclk1(45.MHz())                                                         
        .pclk2(90.MHz())                                                         
        .freeze();
    let mut delay = cp.SYST.delay(&clocks);
    //let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
//    gpioc.pc6.into_push_pull_output().set_high();
/*    let bus = Generic16BitBus::new((
                &dp,
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
    dp.GPIOD.odr.write(|w| unsafe { w.bits(0x1234) });*/
    let interface = PGPIO16BitInterface::new(
                                              dp.GPIOB,
                                              gpioc.pc8.into_push_pull_output(),
                                              gpioc.pc7.into_push_pull_output(),
                                              gpioc.pc9.into_push_pull_output(),
                                              gpioc.pc6.into_push_pull_output()
                                            );

   let mut ili9325 = Ili9325::new(
                                    interface,
                                    &mut delay,
                                    Orientation::LandscapeFlipped,
                                    DisplaySize240x320
                                ).unwrap();
    let yoffset = 24;
    let x_max = (ili9325.width() as i32) - 1;
    let y_max = (ili9325.height() as i32) - 1;
//    ili9325.draw_raw_slice(0, 0, 240, 320, &[0x8700, 0x1122, 0x3389, 0x7687, 0x1123, 0x5578]).unwrap();

    let red_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::RED)
        .stroke_width(2)
        .build();

    let green_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::GREEN)
        .stroke_width(2)
        .build();

    let blue_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLUE)
        .stroke_width(2)
        .build();

    // screen outline
    Rectangle::with_corners(Point::new(0, 0), Point::new(x_max, y_max))
        .into_styled(red_style)
        .draw(&mut ili9325)
        .unwrap();

    // triangle
    Triangle::new(
        Point::new(16, 16 + yoffset),
        Point::new(16 + 16, 16 + yoffset),
        Point::new(16 + 8, yoffset),
    )
    .into_styled(red_style)
    .draw(&mut ili9325)
    .unwrap();

    // square
    Rectangle::with_corners(Point::new(54, yoffset), Point::new(54 + 16, 16 + yoffset))
        .into_styled(green_style)
        .draw(&mut ili9325)
        .unwrap();

    // circle
    Circle::new(Point::new(100, 8 + yoffset), 36)
        .into_styled(blue_style)
        .draw(&mut ili9325)
        .unwrap();
        
    loop {
    }
}
