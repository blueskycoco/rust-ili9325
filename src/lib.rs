#![no_std]

use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;

use core::iter::once;
use display_interface::DataFormat::{U16BEIter, U8Iter};
use display_interface::WriteOnlyDataCommand;

#[cfg(feature = "graphics")]
mod graphics_core;

pub use embedded_hal::spi::MODE_0 as SPI_MODE;

pub use display_interface::DisplayError;

type Result<T = (), E = DisplayError> = core::result::Result<T, E>;

/// Trait that defines display size information
pub trait DisplaySize {
    /// Width in pixels
    const WIDTH: usize;
    /// Height in pixels
    const HEIGHT: usize;
}

/// Generic display size of 240x320 pixels
pub struct DisplaySize240x320;

impl DisplaySize for DisplaySize240x320 {
    const WIDTH: usize = 240;
    const HEIGHT: usize = 320;
}

/// Generic display size of 320x480 pixels
pub struct DisplaySize320x480;

impl DisplaySize for DisplaySize320x480 {
    const WIDTH: usize = 320;
    const HEIGHT: usize = 480;
}

/// The default orientation is Portrait
pub enum Orientation {
    Portrait,
    PortraitFlipped,
    Landscape,
    LandscapeFlipped,
}

/// There are two method for drawing to the screen:
/// [Ili9325::draw_raw_iter] and [Ili9325::draw_raw_slice]
///
/// In both cases the expected pixel format is rgb565.
///
/// The hardware makes it efficient to draw rectangles on the screen.
///
/// What happens is the following:
///
/// - A drawing window is prepared (with the 2 opposite corner coordinates)
/// - The starting point for drawint is the top left corner of this window
/// - Every pair of bytes received is intepreted as a pixel value in rgb565
/// - As soon as a pixel is received, an internal counter is incremented,
///   and the next word will fill the next pixel (the adjacent on the right, or
///   the first of the next row if the row ended)
pub struct Ili9325<IFACE, RESET, CS> {
    interface: IFACE,
    reset: RESET,
    cs: CS,
    width: usize,
    height: usize,
    mode: Orientation,
}

impl<IFACE, RESET, CS> Ili9325<IFACE, RESET, CS>
where
    IFACE: WriteOnlyDataCommand,
    RESET: OutputPin,
    CS: OutputPin,
{
    pub fn new<DELAY, SIZE>(
        interface: IFACE,
        reset: RESET,
        cs: CS,
        delay: &mut DELAY,
        mode: Orientation,
        _display_size: SIZE,
    ) -> Result<Self>
    where
        DELAY: DelayMs<u16>,
        SIZE: DisplaySize,
    {
        let mut ili9325 = Ili9325 {
            interface,
            reset,
            cs,
            width: SIZE::WIDTH,
            height: SIZE::HEIGHT,
            mode: Orientation::Portrait,
        };

        // Do hardware reset by holding reset low for at least 10us
        ili9325.reset.set_low().map_err(|_| DisplayError::RSError)?;
        delay.delay_ms(1);
        // Set high for normal operation
        ili9325
            .reset
            .set_high()
            .map_err(|_| DisplayError::RSError)?;

        // Wait 5ms after reset before sending commands
        // and 120ms before sending Sleep Out
        delay.delay_ms(5);
/*
        // Do software reset
        ili9325.command(Command::SoftwareReset, &[])?;

        // Wait 5ms after reset before sending commands
        // and 120ms before sending Sleep Out
        delay.delay_ms(120);

        ili9325.set_orientation(mode)?;

        // Set pixel format to 16 bits per pixel
        ili9325.command(Command::PixelFormatSet, &[0x55])?;

        ili9325.command(Command::SleepOut, &[])?;

        // Wait 5ms after Sleep Out before sending commands
        delay.delay_ms(5);

        ili9325.command(Command::DisplayOn, &[])?;
*/
        ili9325.command(Command::DriverOutputCtl, &[0x01, 0x00])?;
    	ili9325.command(Command::LcdDrvCtl, &[0x07, 0x00])?;
	    ili9325.command(Command::EntryMode, &[0x10, 0x30])?;
    	ili9325.command(Command::ResizeCtl, &[0x00, 0x00])?;
	    ili9325.command(Command::DispCtl2, &[0x02, 0x07])?;
    	ili9325.command(Command::DispCtl3, &[0x00, 0x00])?;
	    ili9325.command(Command::DispCtl4, &[0x00, 0x00])?;
    	ili9325.command(Command::RGBDispCtl1, &[0x00, 0x00])?;
	    ili9325.command(Command::FrameMarker, &[0x00, 0x00])?;
    	ili9325.command(Command::RGBDispCtl2, &[0x00, 0x00])?;
        delay.delay_ms(50);
    	ili9325.command(Command::DispCtl1, &[0x01, 0x01])?;
        delay.delay_ms(50);
	    ili9325.command(Command::PwrCtl1, &[0x16, 0xb0])?;
    	ili9325.command(Command::PwrCtl2, &[0x00, 0x01])?;
	    ili9325.command(Command::Reg17, &[0x00, 0x01])?;
    	ili9325.command(Command::PwrCtl3, &[0x01, 0x38])?;
	    ili9325.command(Command::PwrCtl4, &[0x08, 0x00])?;
    	ili9325.command(Command::PwrCtl7, &[0x00, 0x09])?;
	    ili9325.command(Command::PwrCtl8, &[0x00, 0x09])?;
    	ili9325.command(Command::Rega4, &[0x00, 0x00])?;
	    ili9325.command(Command::HorizontalAddrStart, &[0x00, 0x00])?;
    	ili9325.command(Command::HorizontalAddrEnd, &[0x00, 0xef])?;
	    ili9325.command(Command::VerticalAddrStart, &[0x00, 0x00])?;
    	ili9325.command(Command::VerticalAddrEnd, &[0x01, 0x3f])?;
	    ili9325.command(Command::DrvCtl2, &[0xa7, 0x00])?;
    	ili9325.command(Command::Bidc, &[0x00, 0x03])?;
	    ili9325.command(Command::Vsc, &[0x00, 0x00])?;
    	ili9325.command(Command::Reg80, &[0x00, 0x00])?;
	    ili9325.command(Command::Reg81, &[0x00, 0x00])?;
    	ili9325.command(Command::Reg82, &[0x00, 0x00])?;
	    ili9325.command(Command::Reg83, &[0x00, 0x00])?;
        ili9325.command(Command::Reg84, &[0x00, 0x00])?;
    	ili9325.command(Command::Reg85, &[0x00, 0x00])?;
	    ili9325.command(Command::Reg90, &[0x00, 0x13])?;
        ili9325.command(Command::Reg92, &[0x00, 0x00])?;
    	ili9325.command(Command::Reg93, &[0x00, 0x03])?;
	    ili9325.command(Command::Reg95, &[0x01, 0x10])?;
    	ili9325.command(Command::DispCtl1, &[0x01, 0x73])?;
        Ok(ili9325)
    }
}

impl<IFACE, RESET, CS> Ili9325<IFACE, RESET, CS>
where
    IFACE: WriteOnlyDataCommand,
    CS: OutputPin,
{
    fn command(&mut self, cmd: Command, args: &[u8]) -> Result {
        self.cs.set_low().map_err(|_| DisplayError::RSError)?;
        self.interface.send_commands(U8Iter(&mut once(cmd as u8)))?;
        self.interface.send_data(U8Iter(&mut args.iter().cloned()))?;
        self.cs.set_high().map_err(|_| DisplayError::RSError)
    }

    fn write_iter<I: IntoIterator<Item = u16>>(&mut self, data: I) -> Result {
        self.command(Command::WriteDataToGram, &[])?;
        //self.interface.send_commands(U8Iter(&mut once(Command::WriteDataToGram as u8)))?;
        self.cs.set_low().map_err(|_| DisplayError::RSError)?;
        self.interface.send_data(U16BEIter(&mut data.into_iter()))?;
        self.cs.set_high().map_err(|_| DisplayError::RSError)
    }

    fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) -> Result {
        self.command(Command::HorizontalAddrStart,
                     &[ (x0 >> 8) as u8, (x0 & 0xff) as u8])?;
        self.command(Command::HorizontalAddrEnd,
                     &[ (x1 >> 8) as u8, (x1 & 0xff) as u8])?;
        self.command(Command::VerticalAddrStart,
                     &[ (y0 >> 8) as u8, (y0 & 0xff) as u8])?;
        self.command(Command::VerticalAddrEnd,
                     &[ (y1 >> 8) as u8, (y1 & 0xff) as u8])?;
        self.command(Command::HorizontalGRAMAddrSet,
                     &[ (x0 >> 8) as u8, (x0 & 0xff) as u8])?;
        self.command(Command::VerticalGRAMAddrSet,
                     &[ (y0 >> 8) as u8, (y0 & 0xff) as u8])
    }

    /// Configures the screen for hardware-accelerated vertical scrolling.
    pub fn configure_vertical_scroll(
        &mut self,
        fixed_top_lines: u16,
        fixed_bottom_lines: u16,
    ) -> Result<Scroller> {
        let height = match self.mode {
            Orientation::Landscape | Orientation::LandscapeFlipped => self.width,
            Orientation::Portrait | Orientation::PortraitFlipped => self.height,
        } as u16;
        let scroll_lines = height as u16 - fixed_top_lines - fixed_bottom_lines;

        self.command(
            Command::VerticalScrollDefine,
            &[
                (fixed_top_lines >> 8) as u8,
                (fixed_top_lines & 0xff) as u8,
                (scroll_lines >> 8) as u8,
                (scroll_lines & 0xff) as u8,
                (fixed_bottom_lines >> 8) as u8,
                (fixed_bottom_lines & 0xff) as u8,
            ],
        )?;

        Ok(Scroller::new(fixed_top_lines, fixed_bottom_lines, height))
    }

    pub fn scroll_vertically(&mut self, scroller: &mut Scroller, num_lines: u16) -> Result {
        scroller.top_offset += num_lines;
        if scroller.top_offset > (scroller.height - scroller.fixed_bottom_lines) {
            scroller.top_offset = scroller.fixed_top_lines
                + (scroller.top_offset + scroller.fixed_bottom_lines - scroller.height)
        }

        self.command(
            Command::VerticalScrollAddr,
            &[
                (scroller.top_offset >> 8) as u8,
                (scroller.top_offset & 0xff) as u8,
            ],
        )
    }

    /// Draw a rectangle on the screen, represented by top-left corner (x0, y0)
    /// and bottom-right corner (x1, y1).
    ///
    /// The border is included.
    ///
    /// This method accepts an iterator of rgb565 pixel values.
    ///
    /// The iterator is useful to avoid wasting memory by holding a buffer for
    /// the whole screen when it is not necessary.
    pub fn draw_raw_iter<I: IntoIterator<Item = u16>>(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
        data: I,
    ) -> Result {
        self.set_window(x0, y0, x1, y1)?;
        self.write_iter(data)
    }

    /// Draw a rectangle on the screen, represented by top-left corner (x0, y0)
    /// and bottom-right corner (x1, y1).
    ///
    /// The border is included.
    ///
    /// This method accepts a raw buffer of words that will be copied to the screen
    /// video memory.
    ///
    /// The expected format is rgb565.
    pub fn draw_raw_slice(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, data: &[u16]) -> Result {
        self.draw_raw_iter(x0, y0, x1, y1, data.iter().copied())
    }

    /// Change the orientation of the screen
    pub fn set_orientation(&mut self, mode: Orientation) -> Result {
        let was_landscape = match self.mode {
            Orientation::Landscape | Orientation::LandscapeFlipped => true,
            Orientation::Portrait | Orientation::PortraitFlipped => false,
        };
        let is_landscape = match mode {
            Orientation::Portrait => {
                self.command(Command::MemoryAccessControl, &[0x40 | 0x08])?;
                false
            }
            Orientation::Landscape => {
                self.command(Command::MemoryAccessControl, &[0x20 | 0x08])?;
                true
            }
            Orientation::PortraitFlipped => {
                self.command(Command::MemoryAccessControl, &[0x80 | 0x08])?;
                false
            }
            Orientation::LandscapeFlipped => {
                self.command(Command::MemoryAccessControl, &[0x40 | 0x80 | 0x20 | 0x08])?;
                true
            }
        };
        if was_landscape ^ is_landscape {
            core::mem::swap(&mut self.height, &mut self.width);
        }
        self.mode = mode;
        Ok(())
    }
}

impl<IFACE, RESET, CS> Ili9325<IFACE, RESET, CS> {
    /// Get the current screen width. It can change based on the current orientation
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get the current screen heighth. It can change based on the current orientation
    pub fn height(&self) -> usize {
        self.height
    }
}

/// Scroller must be provided in order to scroll the screen. It can only be obtained
/// by configuring the screen for scrolling.
pub struct Scroller {
    top_offset: u16,
    fixed_bottom_lines: u16,
    fixed_top_lines: u16,
    height: u16,
}

impl Scroller {
    fn new(fixed_top_lines: u16, fixed_bottom_lines: u16, height: u16) -> Scroller {
        Scroller {
            top_offset: fixed_top_lines,
            fixed_top_lines,
            fixed_bottom_lines,
            height,
        }
    }
}

#[derive(Clone, Copy)]
enum Command {
//    SoftwareReset = 0x01,
//    PixelFormatSet = 0x3a,
//    SleepOut = 0x11,
//    DisplayOn = 0x29,
//    ColumnAddressSet = 0x2a,
//    PageAddressSet = 0x2b,
//    MemoryWrite = 0x2c,
    
    VerticalScrollDefine = 0x33,
    VerticalScrollAddr = 0x37,

    
    MemoryAccessControl = 0x36,
    DriverOutputCtl = 0x01,
    LcdDrvCtl = 0x02,
    EntryMode = 0x03,
    ResizeCtl = 0x04,
    DispCtl1 = 0x07,
    DispCtl2 = 0x08,
    DispCtl3 = 0x09,
    DispCtl4 = 0x0a,
    RGBDispCtl1 = 0x0c,
    FrameMarker = 0x0d,
    RGBDispCtl2 = 0x0f,
    PwrCtl1 = 0x10,
    PwrCtl2 = 0x11,
    PwrCtl3 = 0x12,
    PwrCtl4 = 0x13,
    Reg17   = 0x17,
    PwrCtl7 = 0x29,
    PwrCtl8 = 0x2a,
    DrvCtl2 = 0x60,
    Bidc    = 0x61,
    Vsc     = 0x6a,
    Reg80   = 0x80,
    Reg81   = 0x81,
    Reg82   = 0x82,
    Reg83   = 0x83,
    Reg84   = 0x84,
    Reg85   = 0x85,
    Reg90   = 0x90,
    Reg92   = 0x92,
    Reg93   = 0x93,
    Reg95   = 0x95,
    Rega4   = 0xa4,
    HorizontalGRAMAddrSet = 0x20,
    VerticalGRAMAddrSet = 0x21,
    WriteDataToGram = 0x22,
    HorizontalAddrStart = 0x50,
    HorizontalAddrEnd = 0x51,
    VerticalAddrStart = 0x52,
    VerticalAddrEnd = 0x53,
}
