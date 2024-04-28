//! SPI interface commands for the radio.

use defmt::println;
use hal::{
    delay_us,
    dma::{ChannelCfg, DmaChannel, DmaPeriph},
    gpio::Pin,
    pac::SPI1,
    spi::Spi,
};

use crate::{
    shared,
    shared::{OpCode, RadioError, MAX_ITERS},
};

pub type Spi_ = Spi<SPI1>;

// Note: There's some ambiguity over whether this is 255, or 256, but it needs to fix as a u8 in
// the max payload len param.
pub const RADIO_BUF_SIZE: usize = 255;

const AHB_FREQ: u32 = 170_000_000; // todo: temp hard-coded
const DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1; // todo: temp hard-coded

use crate::shared::{RadioPins, Register};

pub struct Interface {
    pub spi: Spi_,
    pub pins: RadioPins,
    pub tx_ch: DmaChannel,
    pub rx_ch: DmaChannel,
    pub read_buf: [u8; RADIO_BUF_SIZE],
    pub write_buf: [u8; RADIO_BUF_SIZE],
    pub rx_payload_len: u8,
    pub rx_payload_start: u8,
}

impl Interface {
    pub fn reset(&mut self) {
        // todo: Should only need 100us.
        self.pins.reset.set_low();
        delay_us(700, AHB_FREQ);
        self.pins.reset.set_high();
    }

    /// Wait for the radio to be ready to accept commands, using the busy pin. If the busy pin is high,
    /// the radio is not ready for commands.
    pub fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        let mut i = 0;

        while self.pins.busy.is_high() {
            i += 1;
            if i >= MAX_ITERS {
                println!("Exceeded max iters on wait on busy.");
                return Err(RadioError::BusyTimeout);
            }
        }
        Ok(())
    }

    /// Perform a write to an opcode, with 1 byte of data.
    pub fn write_op_word(&mut self, code: OpCode, word: u8) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.write(&[code as u8, word]).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        Ok(())
    }

    /// Perform a read of an opcode, with 1 byte of data.
    pub fn read_op_word(&mut self, code: OpCode) -> Result<u8, RadioError> {
        let mut buf = [code as u8, 0, 0, 0, 0];

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.transfer(&mut buf).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        // todo: Status is buf[1]. Use it? How do we interpret it?
        Ok(buf[2])
    }

    /// Write a single word to a register.
    pub fn write_reg_word(&mut self, reg: Register, word: u8) -> Result<(), RadioError> {
        let addr_split = shared::split_addr(reg as u16);

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self
            .spi
            .write(&[
                OpCode::WriteRegister as u8,
                addr_split.0,
                addr_split.1,
                word,
            ])
            .is_err()
        {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        Ok(())
    }

    /// Read a single word from a register.
    pub fn read_reg_word(&mut self, reg: Register) -> Result<u8, RadioError> {
        let addr_split = shared::split_addr(reg as u16);

        let mut read_buf = [0; 5];

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self
            .spi
            .transfer_type2(
                &[OpCode::ReadRegister as u8, addr_split.0, addr_split.1, 0, 0],
                &mut read_buf,
            )
            .is_err()
        {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.pins.cs.set_high();

        Ok(read_buf[4])
    }

    /// Write a buffer to the radio.
    pub fn write(&mut self, write_buffer: &[u8]) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.write(write_buffer).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.pins.cs.set_high();

        Ok(())
    }

    /// Write with a payload; uses DMA. Must clean up the transaction in an ISR. See note on offsets in
    /// `read_with_payload`.
    pub fn write_with_payload(&mut self, payload: &[u8], offset: u8) -> Result<(), RadioError> {
        unsafe {
            self.write_buf[0] = OpCode::WriteBuffer as u8;
            self.write_buf[1] = offset;

            for (i, word) in payload.iter().enumerate() {
                self.write_buf[i + 2] = *word;
            }
        }

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        unsafe {
            self.spi.write_dma(
                &self.write_buf[..2 + payload.len()],
                self.tx_ch,
                Default::default(),
                DMA_PERIPH,
            );
        }

        Ok(())
    }

    /// Read a payload; uses DMA.
    /// "Before any read or write operation it is hence necessary to initialize this offset to the corresponding beginning of the buffer.
    /// Upon reading or writing to the data buffer the address pointer will then increment automatically."
    pub fn read_with_payload(&mut self, payload_len: u8, offset: u8) -> Result<(), RadioError> {
        unsafe {
            self.write_buf[0] = OpCode::ReadBuffer as u8;
            self.write_buf[1] = offset;
            // "Note that the NOP must be sent after sending the offset."
            self.write_buf[2] = 0;
        }
        // DS, Table 13-27: ReadBuffer SPI Transaction: Payload starts at byte 3.
        let buf_end = payload_len as usize + 3;
        // let buf_end = 4; // todo  t

        self.wait_on_busy()?;

        self.pins.cs.set_low();
        unsafe {
            self.spi.transfer_dma(
                &self.write_buf[0..buf_end],
                &mut self.read_buf[0..buf_end],
                self.tx_ch,
                self.rx_ch,
                ChannelCfg::default(),
                ChannelCfg::default(),
                DMA_PERIPH,
            );
        }

        Ok(())
    }

    /// Request a read, filling the provided buffer.
    pub fn read(&mut self, buffer: &mut [u8]) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        self.pins.cs.set_low();
        if self.spi.transfer(buffer).is_err() {
            self.pins.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.pins.cs.set_high();

        Ok(())
    }

    /// // DS, Table 13-27: ReadBuffer SPI Transaction: Payload starts at byte 3, using the radio's API.
    /// Mutable for use with encryption.
    /// TODO: move to radio mod.
    // fn rx_payload_from_buf(&mut self) -> &'static mut [u8] {
    pub fn rx_payload_from_buf(&mut self) -> &mut [u8] {
        // fn rx_payload_from_buf() -> &'static [u8] {
        // Note: This is the payload length as reported by the radio.
        let payload_len = self.rx_payload_len as usize;
        let payload_start = self.rx_payload_start as usize;
        let payload_start_i = 3 + payload_start;

        unsafe { &mut self.read_buf[payload_start_i..payload_start_i + payload_len] }
    }

    //
    // /// Request a read, filling the provided buffer.
    // pub fn read(&mut self, write_buffer: &[u8], read_buffer: &mut [u8]) -> Result<(), RadioError> {
    //     self.wait_on_busy()?;
    //
    //     self.pins.cs.set_low();
    //     if self.spi.transfer_type2(write_buffer, read_buffer).is_err() {
    //         self.pins.cs.set_high();
    //         return Err(RadioError::SPI);
    //     }
    //     self.pins.cs.set_high();
    //
    //     Ok(())
    // }
}
