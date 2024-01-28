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
    image::{ Image },
    pixelcolor::Rgb565,
    primitives::{Circle, PrimitiveStyleBuilder, Rectangle, Triangle},
    mono_font::{ascii::FONT_9X18_BOLD, MonoTextStyle},
};

use tinybmp::Bmp;
use core::fmt::Write;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::digital::v2::{OutputPin, InputPin};
pub use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};

/* for usart1/dma2 */
use stm32f4xx_hal::dma::{DmaFlag, PeripheralToMemory, Stream2};
use stm32f4xx_hal::dma::config::DmaConfig;
use stm32f4xx_hal::pac::{DMA2};
use stm32f4xx_hal::pac::Interrupt;
use stm32f4xx_hal::uart::config::StopBits;
use stm32f4xx_hal::uart::{Config, Rx, Serial};
use stm32f4xx_hal::{
    dma::{StreamsTuple, Transfer},
    pac,
    prelude::*,
    serial,
};
use md5_rs::Context;

const UART_BUFFER_SIZE: usize = 8*1024;

// Simple ring buffer
pub struct Buffer {
    buffer: [u8; UART_BUFFER_SIZE],
    write_idx: usize,
    read_idx: usize,
}

impl Buffer {
    pub(crate) const fn new() -> Buffer {
        Buffer {
            buffer: [0; UART_BUFFER_SIZE],
            write_idx: 0,
            read_idx: 0,
        }
    }

    pub fn push(&mut self, data: u8) {
        self.buffer[self.write_idx] = data;
        self.write_idx = (self.write_idx + 1) % UART_BUFFER_SIZE;
    }

    pub fn read(&mut self) -> Option<u8> {
        if self.write_idx != self.read_idx {
            let data = self.buffer[self.read_idx];
            self.read_idx = (self.read_idx + 1) % UART_BUFFER_SIZE;
            Some(data)
        } else {
            None
        }
    }
}

// dma type, needs to be adapted for uart and dma channel
type UartDma = Transfer<
    Stream2<pac::DMA2>,
    4,
    Rx<pac::USART1>,
    PeripheralToMemory,
    &'static mut [u8; UART_BUFFER_SIZE],
>;

// shared dma reference
pub static G_TRANSFER: Mutex<RefCell<Option<UartDma>>> = Mutex::new(RefCell::new(None));

// shared uart1 reference
pub static G_UART1_BUFFER: Mutex<RefCell<Option<Buffer>>> = Mutex::new(RefCell::new(None));

// shared TX reference
pub static G_UART1_TX: Mutex<RefCell<Option<serial::Tx<pac::USART1>>>> =
    Mutex::new(RefCell::new(None));

// dma buffer
pub static mut RX_UART1_BUFFER: [u8; UART_BUFFER_SIZE] = [0; UART_BUFFER_SIZE];

pub fn uart1_read() -> Option<[u8; UART_BUFFER_SIZE]> {
    let r = free(|cs| {
        if let Some(buffer) = G_UART1_BUFFER.borrow(cs).borrow_mut().as_mut() {
            let mut buf = [0; UART_BUFFER_SIZE];
            let mut i = 0;
            while let Some(byte) = buffer.read() {
                if i < UART_BUFFER_SIZE - 1 {
                    buf[i] = byte;
                } else {
                    break;
                }
                i += 1;
            }
            //if buf[0] == 0 {
            //    return None;
            //}
            Some(buf)
        } else {
            None
        }
    });
    r
}
// a wrapper function that reads out of the uart ring buffer
pub fn uart1_read_until(eol: u8) -> Option<[u8; UART_BUFFER_SIZE]> {
    let r = free(|cs| {
        if let Some(buffer) = G_UART1_BUFFER.borrow(cs).borrow_mut().as_mut() {
            let mut buf = [0; UART_BUFFER_SIZE];
            let mut i = 0;
            while let Some(byte) = buffer.read() {
                if byte == eol {
                    break;
                }
                if i < UART_BUFFER_SIZE - 1 {
                    buf[i] = byte;
                } else {
                    break;
                }
                i += 1;
            }
            if buf[0] == 0 {
                return None;
            }
            Some(buf)
        } else {
            None
        }
    });
    r
}

// a wrapper function for uart write
pub fn uart1_write(data: &[u8]) -> Result<(), serial::Error> {
    free(|cs| {
        let ret = if let Some(uart) = G_UART1_TX.borrow(cs).borrow_mut().as_mut() {
            let non_zero_len = data
                .iter()
                .rposition(|&x| x != 0)
                .map(|idx| idx + 1)
                .unwrap_or(0);
            // Create a custom slice with only non-zero elements
            let mut led_ref = LED2.borrow(cs).borrow_mut();
            if let Some(ref mut led) = led_ref.deref_mut() {
                led.toggle();
            }
            uart.bwrite_all(&data[0..non_zero_len])?;
            uart.bflush()
        } else {
            Err(serial::Error::Other)
        };
        ret
    })
}

/* for usart1/dma2 */

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

    /* for usart1/dma2 */
    let gpioa = dp.GPIOA.split();
    let dma2 = StreamsTuple::new(dp.DMA2);
    // configure UART, it is important to configure this to use DMA
    let rx_1 = gpioa.pa10.into_alternate();
    let tx_1 = gpioa.pa9.into_alternate();
    let uart1 = Serial::new(
         dp.USART1,
         (tx_1, rx_1),
         Config::default()
             .baudrate(115200.bps())
             .parity_none()
             .stopbits(StopBits::STOP1)
             .dma(serial::config::DmaConfig::Rx),
         &clocks,
    )
    .unwrap();

    // Note! It is better to use memory pools, such as heapless::pool::Pool. But it not work with embedded_dma yet.
    // See CHANGELOG of unreleased main branch and issue https://github.com/japaric/heapless/pull/362 for details.
    let rx_buffer1 =
            cortex_m::singleton!(: [u8; UART_BUFFER_SIZE] = [0; UART_BUFFER_SIZE]).unwrap();
    //let _rx_buffer2 =
    //        cortex_m::singleton!(: [u8; UART_BUFFER_SIZE] = [0; UART_BUFFER_SIZE]).unwrap();

    let (tx1, mut rx) = uart1.split();

    rx.listen_idle();

    free(|cs| *G_UART1_TX.borrow(cs).borrow_mut() = Some(tx1));

    free(|cs| {
            *G_UART1_BUFFER.borrow(cs).borrow_mut() = Some(Buffer::new());
    });
    // Initialize and start DMA stream
    let mut rx_transfer = Transfer::init_peripheral_to_memory(
            dma2.2,
            rx,
            rx_buffer1,
            None,
            DmaConfig::default()
                .memory_increment(true)
                .fifo_enable(true)
                .fifo_error_interrupt(true)
                .transfer_complete_interrupt(true),
        );

    rx_transfer.start(|_rx| {});

    cortex_m::interrupt::free(|cs| *G_TRANSFER.borrow(cs).borrow_mut() = Some(rx_transfer));

    // Enable interrupt
    unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART1);
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::DMA2_STREAM2);
    }
    /* for usart1/dma2 */
    
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
    //let gpioa = dp.GPIOA.split();
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
        
    let touch_din = gpioc.pc3.into_push_pull_output();
    let touch_cs = gpioc.pc13.into_push_pull_output();
    let touch_clk = gpioc.pc0.into_push_pull_output();
    let touch_dout = gpioc.pc2.into_pull_down_input();
    let mut touch = TouchStm32GpioIntf::new(touch_cs, touch_clk,
                                            touch_din, touch_dout);
    let bmp_data = include_bytes!("logo.bmp");
    let bmp = Bmp::from_slice(bmp_data);
    match bmp {
        Ok(bmp_raw) => {
        let im: Image<Bmp<Rgb565>> = Image::new(&bmp_raw, Point::new(0, 0));
        im.draw(&mut ili9325).unwrap();
        },
        Err(error) => {
            writeln!(tx, "display logo failed {:?}", error);
        },
    }
    Text::new("Hello Eva, I love Eva", Point::new(10, 200),
                MonoTextStyle::new(&FONT_9X18_BOLD, Rgb565::GREEN))
        .draw(&mut ili9325)
        .unwrap();
   usr_wifi232_t_init(&mut tx, &mut delay); 
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
                        //writeln!(tx, "ILI9325 touch {} {}\r", x, y).unwrap();
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
            2 => {
                free(|cs| STATE.borrow(cs).replace(0));
                led1.toggle();
                let bmp_data = usr_wifi232_rcv(&mut tx);
                match bmp_data {
                    Some(bmp_raw) => {
                    let bmp = Bmp::from_slice(&bmp_raw[22..]);
                    match bmp {
                        Ok(bmp_byte) => {
                        let x: i32 = (bmp_raw[18] as i32) << 8 | bmp_raw[19] as i32;
                        let y: i32 = (bmp_raw[20] as i32) << 8 | bmp_raw[21] as i32;
                        let im: Image<Bmp<Rgb565>> = Image::new(&bmp_byte, Point::new(x, y));
                        im.draw(&mut ili9325).unwrap();
                        uart1_write(b"send ok");
                        },
                        Err(error) => {
                            writeln!(tx, "display logo failed {:?}\r", error);
                            uart1_write(b"send failed");
                        },
                    }
                    },
                    _ => { writeln!(tx, "bmp xfer failed"); },
                };
            },
            _ => {},
        };
    }
}

fn usr_wifi232_cmd(tx: &mut dyn Write,
                   delay: &mut dyn DelayMs<u16>,
                   cmd: &[u8],
                   ts: u16,
                   dest: &str) -> bool {
   uart1_write(cmd).unwrap();
   delay.delay_ms(ts);
   let resp = uart1_read().unwrap();
   let str_resp = core::str::from_utf8(&resp).unwrap();
   writeln!(tx, "{}", str_resp);
   if str_resp.contains(dest) {
       true
   } else {
       false
   }
}

fn usr_wifi232_rcv(tx: &mut dyn Write) ->Option<[u8;UART_BUFFER_SIZE]> {
    /* layout
     *
     *      | 2 byte len | 16 byte md5 | 4 byte x/y | bmp slice |
     * ofs  0            2             18           22  
     */
    uart1_read()
    /*
    //loop {
        let resp = uart1_read().unwrap();
        //match resp_byte {
           // Some(resp) => {
                let file_len: usize = (resp[0] as usize) << 8 | resp[1] as usize;
                if file_len == 0 {
                    uart1_write(b"send failed 1");
                    return None;
                }
                let mut ctx = Context::new();
                ctx.read(&resp[22..file_len+22]);
                let digest = ctx.finish();
                let remote_dig = &resp[2..18];
                writeln!(tx, "{:?}|{:?}\r", digest, remote_dig);
                if digest != remote_dig {
                    uart1_write(b"send failed 2");
                    return None;
                } else {
                    uart1_write(b"send ok");
                }
                return Some(resp);
           // },
           // _ => { 
             //   writeln!(tx, "no data\r");
           //     return None;
         //   },
       // };
    //}*/
}

fn usr_wifi232_t_init(tx: &mut dyn Write, delay: &mut dyn DelayMs<u16>) {
    //check mode
    let response = usr_wifi232_cmd(tx, delay, b"at+ver\r", 100, "+ok");
    if !response {
        //switch at cmd mode
        usr_wifi232_cmd(tx, delay, b"+++", 100, "a");
        usr_wifi232_cmd(tx, delay, b"a", 100, "+ok");
    }
    //usr_wifi232_cmd(tx, delay, b"at+z\r", 100, "+ok");
    //delay.delay_ms(3000_u16);
    //usr_wifi232_cmd(tx, delay, b"+++", 100, "a");
    //usr_wifi232_cmd(tx, delay, b"a", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+wann\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+netp\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+netp=tcp,server,1234,192.168.1.2\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+netp\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+ping=192.168.1.6\r", 100, "Success");
    usr_wifi232_cmd(tx, delay, b"at+h\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+tcpdis=on\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+tcpdis\r", 100, "+ok");
    usr_wifi232_cmd(tx, delay, b"at+z\r", 2000, "+ok");
    //usr_wifi232_cmd(tx, delay, b"at+tmode=throughput\r", 1000, "+ok");
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
            }
    });
}

#[interrupt]
#[allow(non_snake_case)]
fn USART1() {
    free(|cs| {
        if let Some(transfer) = G_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            if transfer.is_idle() {
            let mut led_ref = LED2.borrow(cs).borrow_mut();
                // Calc received bytes count
                let bytes_count = UART_BUFFER_SIZE - transfer.number_of_transfers() as usize;
                unsafe {
                    let mut buffer = [0; UART_BUFFER_SIZE];
                    match transfer.next_transfer(&mut RX_UART1_BUFFER) {
                        Ok((b, _)) => buffer = *b,
                        Err(_err) => {}
                    }
                    if let Some(ring_buffer) = G_UART1_BUFFER.borrow(cs).borrow_mut().as_mut() {
                        for i in 0..bytes_count {
                            if let Some(ref mut led) = led_ref.deref_mut() {
                                led.toggle();
                            }
                            ring_buffer.push(buffer[i]);
                        }
                        free(|cs| STATE.borrow(cs).replace(2));
                    }
                }
            }
            transfer.clear_idle_interrupt();
        }
    });
}

#[interrupt]
#[allow(non_snake_case)]
fn DMA2_STREAM2() {
    free(|cs| {
            let mut led_ref = LED2.borrow(cs).borrow_mut();
            if let Some(ref mut led) = led_ref.deref_mut() {
                led.toggle();
            }
        if let Some(transfer) = G_TRANSFER.borrow(cs).borrow_mut().as_mut() {
            // Its important to clear fifo errors as the transfer is paused until it is cleared
            transfer.clear_flags(DmaFlag::FifoError | DmaFlag::TransferComplete);
        }
    });
}
