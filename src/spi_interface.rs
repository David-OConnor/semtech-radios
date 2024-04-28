//! SPI interface commands for the radio.

use crate::shared;
use defmt::println;
use hal::dma::{DmaChannel, DmaPeriph};
use hal::{delay_us, dma::ChannelCfg, gpio::Pin};

const AHB_FREQ: u32 = 170_000_000; // todo: temp hard-coded
const DMA_PERIPH: DmaPeriph = DmaPeriph::Dma1; // todo: temp hard-coded

use super::sx126x::{OpCode, RadioError, Register, Spi_, MAX_ITERS};

pub struct Interface {
    pub spi: Spi_,
    pub reset: Pin,
    pub busy: Pin,
    pub cs: Pin,
    pub tx_ch: DmaChannel,
    pub rx_ch: DmaChannel,
    pub read_buf: [u8; 256],
    pub write_buf: [u8; 256],
}

impl Interface {
    pub fn reset(&mut self) {
        // todo: Should only need 100us.
        self.reset.set_low();
        delay_us(700, AHB_FREQ);
        self.reset.set_high();
    }

    /// Wait for the radio to be ready to accept commands, using the busy pin. If the busy pin is high,
    /// the radio is not ready for commands.
    pub fn wait_on_busy(&mut self) -> Result<(), RadioError> {
        let mut i = 0;

        while self.busy.is_high() {
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

        self.cs.set_low();
        if self.spi.write(&[code as u8, word]).is_err() {
            self.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.cs.set_high();

        Ok(())
    }

    /// Perform a read of an opcode, with 1 byte of data.
    pub fn read_op_word(&mut self, code: OpCode) -> Result<u8, RadioError> {
        let mut buf = [code as u8, 0, 0, 0, 0];

        self.wait_on_busy()?;

        self.cs.set_low();
        if self.spi.transfer(&mut buf).is_err() {
            self.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.cs.set_high();

        // todo: Status is buf[1]. Use it? How do we interpret it?
        Ok(buf[2])
    }

    /// Write a single word to a register.
    pub fn write_reg_word(&mut self, reg: Register, word: u8) -> Result<(), RadioError> {
        let addr_split = shared::split_addr(reg as u16);

        self.wait_on_busy()?;

        self.cs.set_low();
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
            self.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.cs.set_high();

        Ok(())
    }

    /// Read a single word from a register.
    pub fn read_reg_word(&mut self, reg: Register) -> Result<u8, RadioError> {
        let addr_split = shared::split_addr(reg as u16);

        let mut read_buf = [0; 5];

        self.wait_on_busy()?;

        self.cs.set_low();
        if self
            .spi
            .transfer_type2(
                &[OpCode::ReadRegister as u8, addr_split.0, addr_split.1, 0, 0],
                &mut read_buf,
            )
            .is_err()
        {
            self.cs.set_high();
            return Err(RadioError::Spi);
        };
        self.cs.set_high();

        Ok(read_buf[4])
    }

    /// Write a buffer to the radio.
    pub fn write(&mut self, write_buffer: &[u8]) -> Result<(), RadioError> {
        self.wait_on_busy()?;

        self.cs.set_low();
        if self.spi.write(write_buffer).is_err() {
            self.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.cs.set_high();

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

        self.cs.set_low();
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

        self.cs.set_low();
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

        self.cs.set_low();
        if self.spi.transfer(buffer).is_err() {
            self.cs.set_high();
            return Err(RadioError::Spi);
        }
        self.cs.set_high();

        Ok(())
    }
    //
    // /// Request a read, filling the provided buffer.
    // pub fn read(&mut self, write_buffer: &[u8], read_buffer: &mut [u8]) -> Result<(), RadioError> {
    //     self.wait_on_busy()?;
    //
    //     self.cs.set_low();
    //     if self.spi.transfer_type2(write_buffer, read_buffer).is_err() {
    //         self.cs.set_high();
    //         return Err(RadioError::SPI);
    //     }
    //     self.cs.set_high();
    //
    //     Ok(())
    // }
}
