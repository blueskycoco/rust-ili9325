#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_semihosting as _;

use ili9325::{Ili9325};
pub use ili9325::{DisplaySize240x320, DisplaySize320x480};
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals, GPIOB};
use stm32f4xx_hal::prelude::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle};
use cortex_m_semihosting::hprintln;

use embedded_hal::digital::v2::OutputPin;
pub use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};

type Result<T = ()> = core::result::Result<T, DisplayError>;
pub struct ParallelStm32GpioIntf<DC, WR, CS, RD> {
    gpio: GPIOB,
    dc: DC,
    wr: WR,
    cs: CS,
    rd: RD,
}

impl<DC, WR, CS, RD> ParallelStm32GpioIntf<DC, WR, CS, RD>
where
    DC: OutputPin,
    WR: OutputPin,
    CS: OutputPin,
    RD: OutputPin,
{
    /// Create new parallel GPIO interface for communication with a display driver
    pub fn new(gpio: GPIOB, mut dc: DC, mut wr: WR, mut cs: CS, mut rd: RD) -> Self {
        //read id first
        let _ = gpio.moder.write(|w| unsafe { w.bits(0x55555555) });
        let _ = gpio.pupdr.write(|w| unsafe { w.bits(0x55555555) });
        let _ = gpio.ospeedr.write(|w| unsafe { w.bits(0xffffffff) });
        hprintln!("moder: {:#x}", gpio.moder.read().bits());
        let _ = cs.set_low().map_err(|_| DisplayError::DCError);
        let _ = dc.set_low().map_err(|_| DisplayError::DCError);
        let _ = rd.set_high().map_err(|_| DisplayError::DCError);
        let _ = wr.set_low().map_err(|_| DisplayError::BusWriteError);
        let _ = gpio.odr.write(|w| unsafe { w.bits(0x00 as u32) } );
        let _ = wr.set_high().map_err(|_| DisplayError::BusWriteError);
        let _ = cs.set_high().map_err(|_| DisplayError::DCError);
        
        let _ = gpio.moder.write(|w| unsafe { w.bits(0x00 as u32) });
        hprintln!("moder: {:#x}", gpio.moder.read().bits());
        let _ = cs.set_low().map_err(|_| DisplayError::DCError);
        let _ = dc.set_high().map_err(|_| DisplayError::DCError);
        let _ = wr.set_high().map_err(|_| DisplayError::BusWriteError);
        let _ = rd.set_low().map_err(|_| DisplayError::DCError);
        hprintln!("ili9325 id: {:#x}", gpio.idr.read().bits());
        let _ = gpio.moder.write(|w| unsafe { w.bits(0x55555555) });
        hprintln!("moder: {:#x}", gpio.moder.read().bits());
        let _ = rd.set_high().map_err(|_| DisplayError::DCError);
        let _ = cs.set_high().map_err(|_| DisplayError::DCError);
        Self { gpio, dc, wr, cs, rd }
    }

    /// Consume the display interface and return
    /// the bus and GPIO pins used by it
    pub fn release(self) -> (DC, WR, CS, RD) {
        (self.dc, self.wr, self.cs, self.rd)
    }

    fn write_iter(&mut self, iter: impl Iterator<Item = u16>) -> Result {
        for value in iter {
            let _ = self.cs.set_low().map_err(|_| DisplayError::DCError);
            let _ = self.wr.set_low().map_err(|_| DisplayError::BusWriteError)?;
//            hprintln!("value w is {:#x}", value);
            let _ = self.gpio.odr.write(|w| unsafe { w.bits(value as u32) } );
//            hprintln!("value r is {:#x}", self.gpio.odr.read().bits());
            let _ = self.wr.set_high().map_err(|_| DisplayError::BusWriteError)?;
            let _ = self.cs.set_high().map_err(|_| DisplayError::DCError);
        }

        Ok(())
    }

    fn write_data(&mut self, data: DataFormat<'_>) -> Result {
        match data {
            DataFormat::U8(slice) => self.write_iter(slice.iter().copied().map(u16::from)),
            DataFormat::U8Iter(iter) => self.write_iter(iter.map(u16::from)),
            DataFormat::U16(slice) => self.write_iter(slice.iter().copied()),
            DataFormat::U16BE(slice) => self.write_iter(slice.iter().copied()),
            DataFormat::U16LE(slice) => self.write_iter(slice.iter().copied()),
            DataFormat::U16BEIter(iter) => self.write_iter(iter),
            DataFormat::U16LEIter(iter) => self.write_iter(iter),
            _ => Err(DisplayError::DataFormatNotImplemented),
        }
    }
}

impl<DC, WR, CS, RD> WriteOnlyDataCommand for ParallelStm32GpioIntf<DC, WR, CS, RD>
where
    DC: OutputPin,
    WR: OutputPin,
    CS: OutputPin,
    RD: OutputPin,
{
    fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result {
        self.dc.set_low().map_err(|_| DisplayError::DCError)?;
        self.write_data(cmds)
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> Result {
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        self.write_data(buf)
    }
}

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
    let gpioc = dp.GPIOC.split();
    let interface = ParallelStm32GpioIntf::new(
                                              dp.GPIOB,
                                              gpioc.pc8.into_push_pull_output(),
                                              gpioc.pc7.into_push_pull_output(),
                                              gpioc.pc9.into_push_pull_output(),
                                              gpioc.pc6.into_push_pull_output()
                                            );

   let mut ili9325 = Ili9325::new(
                                    interface,
                                    &mut delay,
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
