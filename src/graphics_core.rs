use crate::Ili9325;

use embedded_graphics_core::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};
use embedded_hal::digital::v2::OutputPin;

impl<IFACE, RESET, CS> OriginDimensions for Ili9325<IFACE, RESET, CS> {
    fn size(&self) -> Size {
        Size::new(self.width() as u32, self.height() as u32)
    }
}

impl<IFACE, RESET, CS> DrawTarget for Ili9325<IFACE, RESET, CS>
where
    IFACE: display_interface::WriteOnlyDataCommand,
    CS: OutputPin,
{
    type Error = display_interface::DisplayError;

    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if self.bounding_box().contains(point) {
                let x = point.x as u16;
                let y = point.y as u16;

                self.draw_raw_iter(
                    x,
                    y,
                    x,
                    y,
                    core::iter::once(RawU16::from(color).into_inner()),
                )?;
            }
        }
        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let drawable_area = area.intersection(&self.bounding_box());

        if let Some(drawable_bottom_right) = drawable_area.bottom_right() {
            let x0 = drawable_area.top_left.x as u16;
            let y0 = drawable_area.top_left.y as u16;
            let x1 = drawable_bottom_right.x as u16;
            let y1 = drawable_bottom_right.y as u16;

            if area == &drawable_area {
                // All pixels are on screen
                self.draw_raw_iter(
                    x0,
                    y0,
                    x1,
                    y1,
                    area.points()
                        .zip(colors)
                        .map(|(_, color)| RawU16::from(color).into_inner()),
                )
            } else {
                // Some pixels are on screen
                self.draw_raw_iter(
                    x0,
                    y0,
                    x1,
                    y1,
                    area.points()
                        .zip(colors)
                        .filter(|(point, _)| drawable_area.contains(*point))
                        .map(|(_, color)| RawU16::from(color).into_inner()),
                )
            }
        } else {
            // No pixels are on screen
            Ok(())
        }
    }
}
