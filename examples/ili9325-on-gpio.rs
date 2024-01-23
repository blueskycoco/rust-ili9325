#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_semihosting as _;

use ili9325::{Ili9325};
pub use ili9325::{DisplaySize240x320, DisplaySize320x240};
use stm32f4xx_hal::pac::{CorePeripherals, Peripherals, GPIOB, NVIC};
use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::gpio::{Edge, Input, Output, PC1, PA12};
use stm32f4xx_hal::prelude::*;
use core::cell::{Cell, RefCell};
use core::ops::DerefMut;
use cortex_m::interrupt::{free, Mutex};
use embedded_graphics::{
    text::{Text,},
    prelude::*,
    pixelcolor::Rgb565,
    primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle},
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyle},
};
use core::fmt::Write;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{OutputPin, InputPin};
pub use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};

type ResultPin<T = ()> = core::result::Result<T, DisplayError>;

pub struct TouchStm32GpioIntf<CS, CLK, DIN, DOUT> {
    cs: CS,
    clk: CLK,
    din: DIN,
    dout: DOUT,
}

impl<CS, CLK, DIN, DOUT> TouchStm32GpioIntf<CS, CLK, DIN, DOUT>
where
    CS: OutputPin,
    CLK: OutputPin,
    DIN: OutputPin,
    DOUT: InputPin,
{
    pub fn new(mut cs: CS, mut clk: CLK, mut din: DIN, dout: DOUT) -> Self {
        let _ = clk.set_low();
        let _ = cs.set_high();
        let _ = din.set_high();
        let _ = clk.set_high();
        let _ = cs.set_low();
        Self { cs, clk, din, dout}
    }

    pub fn release(self) -> (CS, CLK, DIN, DOUT) {
        (self.cs, self.clk, self.din, self.dout)
    }

    pub fn get_pixel(&mut self, delay: &mut dyn DelayUs<u16>) -> (u16, u16) {
        let (x1, y1) = self.touch_read(delay);
        let (x2, y2) = self.touch_read(delay);
        if (x1.abs_diff(x2) < 15) && (y1.abs_diff(y2) < 15) {
            let x = (x1 + x2) / 2;
            let y = (y1 + y2) / 2;
            if (x > 280) && (y > 340) {
                let mut xx = (240 * (x - 280) as u32 / (3800 - 280) as u32) as u16;
                let mut yy = (320 * (y - 340) as u32 / (3600 - 340) as u32) as u16;
                if xx > 240 { xx = 240; }
                if yy > 320 { yy = 320; }
                return (xx, yy);
            } else {
                return (0, 0);
            }
        }

        (0, 0)
    }

    fn touch_read(&mut self, delay: &mut dyn DelayUs<u16>) -> (u16, u16) {
        let _ = self.cs.set_low();
        
        self.reg_w(0xd0, delay);
        let _ = self.clk.set_high();
        delay.delay_us(1);
        let _ = self.clk.set_low();
        delay.delay_us(1);
        let x = self.reg_r(delay);
        
        self.reg_w(0x90, delay);
        let _ = self.clk.set_high();
        delay.delay_us(1);
        let _ = self.clk.set_low();
        delay.delay_us(1);
        let y = self.reg_r(delay);
        
        let _ = self.cs.set_high();
        (x, y)
    }

    fn reg_w(&mut self, reg: u8, delay: &mut dyn DelayUs<u16>) {
        let mut i_reg = reg;
        for _ in 0..8 {
            if i_reg & 0x80 == 0x80 {
                let _ = self.din.set_high();
            } else {
                let _ = self.din.set_low();
            }
            let _ = self.clk.set_low();
            delay.delay_us(1);
            let _ = self.clk.set_high();
            delay.delay_us(1);
            i_reg = i_reg << 1;
        }
    }

    fn reg_r(&mut self, delay: &mut dyn DelayUs<u16>) -> u16 {
        let mut data: u16 = 0;
        for _ in 0..12 {
            data = data << 1;
            let _ = self.clk.set_high();
            delay.delay_us(1);
            let _ = self.clk.set_low();
            delay.delay_us(1);
            match self.dout.is_high() {
                Ok(true) => data = data + 1,
                _ => {},
            }
        }
        data
    }

}

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
    pub fn new(delay: &mut dyn DelayMs<u16>, tx: &mut dyn Write, gpio: GPIOB,
               mut dc: DC, mut wr: WR, mut cs: CS, mut rd: RD) -> Self {
        // config gpiob pushpull output, high speed.
        //writeln!(tx, "in ParallelStm32GpioIntf\r\n").unwrap();
        let _ = gpio.moder.write(|w| unsafe { w.bits(0x55555555) });
        let _ = gpio.pupdr.write(|w| unsafe { w.bits(0x55555555) });
        let _ = gpio.ospeedr.write(|w| unsafe { w.bits(0xffffffff) });
        //read id first
        //writeln!(tx, "moder: {:#x}\r", gpio.moder.read().bits()).unwrap();
        let _ = cs.set_low().map_err(|_| DisplayError::DCError);
        let _ = dc.set_low().map_err(|_| DisplayError::DCError);
        let _ = rd.set_high().map_err(|_| DisplayError::DCError);
        let _ = wr.set_low().map_err(|_| DisplayError::BusWriteError);
        let _ = gpio.odr.write(|w| unsafe { w.bits(0x00 as u32) } );
        let _ = wr.set_high().map_err(|_| DisplayError::BusWriteError);
        let _ = cs.set_high().map_err(|_| DisplayError::DCError);
        
        let _ = gpio.moder.write(|w| unsafe { w.bits(0x00 as u32) });
        //writeln!(tx, "moder: {:#x}\r", gpio.moder.read().bits()).unwrap();
        let _ = cs.set_low().map_err(|_| DisplayError::DCError);
        let _ = dc.set_high().map_err(|_| DisplayError::DCError);
        let _ = wr.set_high().map_err(|_| DisplayError::BusWriteError);
        let _ = rd.set_low().map_err(|_| DisplayError::DCError);
        delay.delay_ms(1);
        writeln!(tx, "ili9325 id: {:#x}\r", gpio.idr.read().bits()).unwrap();
        let _ = gpio.moder.write(|w| unsafe { w.bits(0x55555555) });
        //writeln!(tx, "moder: {:#x}\r", gpio.moder.read().bits()).unwrap();
        let _ = rd.set_high().map_err(|_| DisplayError::DCError);
        let _ = cs.set_high().map_err(|_| DisplayError::DCError);
        Self { gpio, dc, wr, cs, rd }
    }

    /// Consume the display interface and return
    /// the bus and GPIO pins used by it
    pub fn release(self) -> (DC, WR, CS, RD) {
        (self.dc, self.wr, self.cs, self.rd)
    }

    fn write_iter(&mut self, iter: impl Iterator<Item = u16>) -> ResultPin {
        for value in iter {
            let _ = self.cs.set_low().map_err(|_| DisplayError::DCError);
            let _ = self.wr.set_low().map_err(|_| DisplayError::BusWriteError)?;
            let _ = self.gpio.odr.write(|w| unsafe { w.bits(value as u32) } );
            //writeln!(self.tx, "tx: {:#x}\r", value).unwrap();
            let _ = self.wr.set_high().map_err(|_| DisplayError::BusWriteError)?;
            let _ = self.cs.set_high().map_err(|_| DisplayError::DCError);
        }

        Ok(())
    }

    fn write_data(&mut self, data: DataFormat<'_>) -> ResultPin {
        match data {
            DataFormat::U8(slice) => self.write_iter(slice.iter().copied()
                                                     .map(u16::from)),
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

impl<DC, WR, CS, RD> WriteOnlyDataCommand for
    ParallelStm32GpioIntf<DC, WR, CS, RD>
where
    DC: OutputPin,
    WR: OutputPin,
    CS: OutputPin,
    RD: OutputPin,
{
    fn send_commands(&mut self, cmds: DataFormat<'_>) -> ResultPin {
        self.dc.set_low().map_err(|_| DisplayError::DCError)?;
        self.write_data(cmds)
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> ResultPin {
        self.dc.set_high().map_err(|_| DisplayError::DCError)?;
        self.write_data(buf)
    }
}

static STATE: Mutex<Cell<u8>> = Mutex::new(Cell::new(0u8));
static TOUCH: Mutex<RefCell<Option<PC1<Input>>>> = Mutex::new(RefCell::new(None));
static LED2: Mutex<RefCell<Option<PA12<Output>>>> = Mutex::new(RefCell::new(None));
#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();

    dp.RCC.ahb1enr.write(|w| w.gpioben().enabled());
    let rcc = dp.RCC.constrain();
    // Make HCLK faster to allow updating the display more quickly
    let clocks = rcc.cfgr                                                                     
        .hclk(180.MHz())                                                         
        .sysclk(180.MHz())                                                       
        .pclk1(45.MHz())                                                         
        .pclk2(90.MHz())                                                         
        .freeze();
    let mut syscfg = dp.SYSCFG.constrain();

    // Create a button input with an interrupt
    let gpioc = dp.GPIOC.split();
    let mut touch_int = gpioc.pc1.into_pull_up_input();
    touch_int.make_interrupt_source(&mut syscfg);
    touch_int.enable_interrupt(&mut dp.EXTI);
    touch_int.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    let btn_int_num = touch_int.interrupt(); // hal::pac::Interrupt::EXTI15_10

    free(|cs| {
         TOUCH.borrow(cs).replace(Some(touch_int));
    });

    // Enable interrupts
    NVIC::unpend(btn_int_num);
    unsafe {
         NVIC::unmask(btn_int_num);
    };
    let mut delay = cp.SYST.delay(&clocks);
    //let gpioc = dp.GPIOC.split();
    let gpioa = dp.GPIOA.split();
    let mut led1 = gpioa.pa11.into_push_pull_output();
    let led2 = gpioa.pa12.into_push_pull_output();
    free(|cs| {
        LED2.borrow(cs).replace(Some(led2));
    });
    let mut tx = dp.USART2.tx(gpioa.pa2, 115200.bps(), &clocks).unwrap();
    writeln!(tx, "ILI9325 Lcd\r").unwrap();
    let interface = ParallelStm32GpioIntf::new(
                                              &mut delay,
                                              &mut tx,
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
    let _ = ili9325.clear(Rgb565::BLACK);
    let yoffset = 24;
    let x_max = (ili9325.width() as i32) - 1;
    let y_max = (ili9325.height() as i32) - 1;

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
    Rectangle::with_corners(Point::new(54, yoffset), Point::new(54 + 16,
                                                                16 + yoffset))
        .into_styled(green_style)
        .draw(&mut ili9325)
        .unwrap();

    // circle
    Circle::new(Point::new(100, 8 + yoffset), 36)
        .into_styled(blue_style)
        .draw(&mut ili9325)
        .unwrap();
        
    Text::new("Hello Eva, I love Eva", Point::new(10, 200),
                MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::YELLOW))
        .draw(&mut ili9325)
        .unwrap();
    let touch_din = gpioc.pc3.into_push_pull_output();
    let touch_cs = gpioc.pc13.into_push_pull_output();
    let touch_clk = gpioc.pc0.into_push_pull_output();
    let touch_dout = gpioc.pc2.into_pull_down_input();
    let mut touch = TouchStm32GpioIntf::new(touch_cs, touch_clk,
                                            touch_din, touch_dout);
    loop {
        let state = free(|cs| STATE.borrow(cs).get());
        match state {
            1 => {
                free(|cs| STATE.borrow(cs).replace(0));
                led1.toggle();
                free(|cs| {
                let mut touch_ref = TOUCH.borrow(cs).borrow_mut();
                if let Some(ref mut touch_io) = touch_ref.deref_mut() {
                loop {
                match touch_io.is_low() {
                    true => {
                        let (x, y) =touch.get_pixel(&mut delay);
                        if !(x == 0 && y == 0) {
                        writeln!(tx, "ILI9325 touch {} {}\r", x, y).unwrap();
                        Pixel(Point::new(x as i32, y as i32), Rgb565::GREEN).draw(&mut ili9325).unwrap();
                        }
                    },
                    _ => { break; },
                }
                }
                free(|cs| STATE.borrow(cs).replace(0));
                }
                });
            },
            _ => {},
        };
    }
}

fn led2_set(high: bool) {
    free(|cs| {
    let mut led_ref = LED2.borrow(cs).borrow_mut();
    if let Some(ref mut led) = led_ref.deref_mut() {
        match high {
            true => led.set_high(),
            _ => led.set_low(),
        }
    }
    });
}

#[interrupt]
fn EXTI1() {
    free(|cs| {
        let mut touch_ref = TOUCH.borrow(cs).borrow_mut();
        if let Some(ref mut touch) = touch_ref.deref_mut() {
            touch.clear_interrupt_pending_bit();
            free(|cs| STATE.borrow(cs).replace(1));
//            let mut led_ref = LED2.borrow(cs).borrow_mut();
//            if let Some(ref mut led) = led_ref.deref_mut() {
//                led.toggle();
//            }

            }
    });
}
