//! Connects the board-independent BMM350 driver to an RRIV board.

use bmm350_driver::Bmm350Bus;
use rriv_board::RRIVBoard;

/// A short-lived view of the RRIV board's I2C bus and delay functions.
///
/// The adapter does not own the board. The BMM350 driver borrows it only for
/// the duration of an operation such as setup or taking a measurement.
pub(crate) struct RrivBmm350Bus<'a> {
    board: &'a mut dyn RRIVBoard,
}

impl<'a> RrivBmm350Bus<'a> {
    pub(crate) fn new(board: &'a mut dyn RRIVBoard) -> Self {
        Self { board }
    }
}

impl Bmm350Bus for RrivBmm350Bus<'_> {
    type Error = ();

    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.board.ic2_write(address, bytes)
    }

    fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.board.ic2_write_read(address, write, read)
    }

    fn delay_us(&mut self, microseconds: u16) {
        self.board.delay_us(microseconds);
    }

    fn delay_ms(&mut self, milliseconds: u16) {
        self.board.delay_ms(milliseconds);
    }
}
