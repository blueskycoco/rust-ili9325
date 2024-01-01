#![no_std]

use embedded_hal::blocking::delay::DelayMs;

use core::iter::once;
use display_interface::DataFormat::{U16BEIter, U8Iter, U16};
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
pub struct Ili9325<IFACE> {
    interface: IFACE,
    width: usize,
    height: usize,
}

impl<IFACE> Ili9325<IFACE>
where
    IFACE: WriteOnlyDataCommand,
{
    pub fn new<DELAY, SIZE>(
        interface: IFACE,
        delay: &mut DELAY,
        _display_size: SIZE,
    ) -> Result<Self>
    where
        DELAY: DelayMs<u16>,
        SIZE: DisplaySize,
    {
        let mut ili9325 = Ili9325 {
            interface,
            width: SIZE::WIDTH,
            height: SIZE::HEIGHT,
        };

        ili9325.command(Command::DriverOutputCtl,       &[0x0100])?;
        ili9325.command(Command::LcdDrvCtl,             &[0x0700])?;
        ili9325.command(Command::EntryMode,             &[0x1030])?;
        ili9325.command(Command::ResizeCtl,             &[0x0000])?;
        ili9325.command(Command::DispCtl2,              &[0x0207])?;
        ili9325.command(Command::DispCtl3,              &[0x0000])?;
        ili9325.command(Command::DispCtl4,              &[0x0000])?;
        ili9325.command(Command::RGBDispCtl1,           &[0x0000])?;
        ili9325.command(Command::FrameMarker,           &[0x0000])?;
        ili9325.command(Command::RGBDispCtl2,           &[0x0000])?;
        delay.delay_ms(50);
        ili9325.command(Command::DispCtl1,              &[0x0101])?;
        delay.delay_ms(50);
        ili9325.command(Command::PwrCtl1,               &[0x16b0])?;
        ili9325.command(Command::PwrCtl2,               &[0x0001])?;
        ili9325.command(Command::Reg17,                 &[0x0001])?;
        ili9325.command(Command::PwrCtl3,               &[0x0138])?;
        ili9325.command(Command::PwrCtl4,               &[0x0800])?;
        ili9325.command(Command::PwrCtl7,               &[0x0009])?;
        ili9325.command(Command::PwrCtl8,               &[0x0009])?;
        ili9325.command(Command::Rega4,                 &[0x0000])?;
        ili9325.command(Command::HorizontalAddrStart,   &[0x0000])?;
        ili9325.command(Command::HorizontalAddrEnd,     &[0x00ef])?;
        ili9325.command(Command::VerticalAddrStart,     &[0x0000])?;
        ili9325.command(Command::VerticalAddrEnd,       &[0x013f])?;
        ili9325.command(Command::DrvCtl2,               &[0xa700])?;
        ili9325.command(Command::Bidc,                  &[0x0003])?;
        ili9325.command(Command::Vsc,                   &[0x0000])?;
        ili9325.command(Command::Reg80,                 &[0x0000])?;
        ili9325.command(Command::Reg81,                 &[0x0000])?;
        ili9325.command(Command::Reg82,                 &[0x0000])?;
        ili9325.command(Command::Reg83,                 &[0x0000])?;
        ili9325.command(Command::Reg84,                 &[0x0000])?;
        ili9325.command(Command::Reg85,                 &[0x0000])?;
        ili9325.command(Command::Reg90,                 &[0x0013])?;
        ili9325.command(Command::Reg92,                 &[0x0000])?;
        ili9325.command(Command::Reg93,                 &[0x0003])?;
        ili9325.command(Command::Reg95,                 &[0x0110])?;
        ili9325.command(Command::DispCtl1,              &[0x0173])?;
        delay.delay_ms(50);
        ili9325.set_window(0, 0, 239, 319)?;
        for _ in 0..240*320 {
            ili9325.write_iter([0x0000].iter().copied())?;
        }
        Ok(ili9325)
    }
}

impl<IFACE> Ili9325<IFACE>
where
    IFACE: WriteOnlyDataCommand,
{
    fn command(&mut self, cmd: Command, args: &[u16]) -> Result {
        self.interface.send_commands(U8Iter(&mut once(cmd as u8)))?;
        self.interface.send_data(U16(args))
    }

    fn write_iter<I: IntoIterator<Item = u16>>(&mut self, data: I) -> Result {
        self.interface.send_commands(U8Iter(&mut once(Command::WriteDataToGram as u8)))?;
        self.interface.send_data(U16BEIter(&mut data.into_iter()))
    }

    fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) -> Result {
        self.command(Command::HorizontalAddrStart, &[x0])?;
        self.command(Command::HorizontalAddrEnd, &[x1])?; 
        self.command(Command::VerticalAddrStart, &[y0])?;
        self.command(Command::VerticalAddrEnd, &[y1])?;
        self.command(Command::HorizontalGRAMAddrSet, &[x0])?; 
        self.command(Command::VerticalGRAMAddrSet, &[y0])
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

}

impl<IFACE> Ili9325<IFACE> {
    /// Get the current screen width. It can change based on the current orientation
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get the current screen heighth. It can change based on the current orientation
    pub fn height(&self) -> usize {
        self.height
    }
}

#[derive(Clone, Copy)]
enum Command {
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
