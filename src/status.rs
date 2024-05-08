//! Contains code related to assessing status of the radio and operations.

use defmt::println;

use crate::{
    shared::{OpCode, RadioError, RadioError::UnexpectedStatus},
    CommandStatus, OperatingModeRead, Radio, RadioConfig, RxBufferStatus, RxPacketStatusLora,
    RxStatistics6x,
};

impl Radio {
    /// 6x only. DS, section 13.5.5
    /// todo: Impl reset as well.
    pub fn get_statistics(&mut self) -> Result<RxStatistics6x, RadioError> {
        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::GetStatistics as u8,
            RadioConfig::R8x(_) => panic!("GetStatistics unavailabe on sx128x"),
        };

        let mut buf = [op_code, 0, 0, 0, 0, 0, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(RxStatistics6x {
            status: buf[1],
            num_received: u16::from_be_bytes([buf[2], buf[3]]),
            num_crc_error: u16::from_be_bytes([buf[4], buf[5]]),
            num_length_error: u16::from_be_bytes([buf[6], buf[7]]),
        })
    }

    /// 6x DS, section 13.5.3. This contains useful link stats from a received message. (LoRa)
    /// 8x: DS, section 11.8.2.Differen, including more fields, eg for BLE, FLRC etc. LoRa uses
    /// status, rssiSync and snr only. I think we can use the same code for both.
    pub fn get_packet_status(&mut self) -> Result<RxPacketStatusLora, RadioError> {
        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::GetPacketStatus as u8,
            RadioConfig::R8x(_) => OpCode::GetPacketStatus.val_8x(),
        };

        let mut buf = [op_code, 0, 0, 0, 0];
        self.interface.read(&mut buf)?;

        // Raw data, to make passing over the wire/air more compact. Convert using scaler/signed types
        // prior to display or use.
        Ok(RxPacketStatusLora {
            // todo: Which RSSI should we use?
            status: buf[1],
            rssi: buf[2],
            snr: buf[3],
            signal_rssi: buf[4],
        })
    }

    /// DS, section 13.5.4. todo: When would we use this over packet status?
    pub fn get_rssi_inst(&mut self) -> Result<i8, RadioError> {
        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::GetRSSIInst as u8,
            RadioConfig::R8x(_) => OpCode::GetRSSIInst.val_8x(),
        };

        let mut buf = [op_code, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(-(buf[2] as i8) / 2)
    }

    /// 6x: 13.5.1
    /// 8x: 11.3. (Similar, but at different indices.
    pub fn get_status(&mut self) -> Result<(OperatingModeRead, CommandStatus), RadioError> {
        let mut buf = [OpCode::GetStatus as u8, 0]; // This is OK; same OpCode on 6x and 8x.
        self.interface.read(&mut buf)?;

        let (om, c_s) = match self.config {
            RadioConfig::R6x(_) => ((buf[1] >> 4) & 0b111, (buf[1] >> 1) & 0b111),
            RadioConfig::R8x(_) => ((buf[0] >> 5) & 0b111, (buf[0] >> 2) & 0b111),
        };

        let operating_mode = match om {
            2 => OperatingModeRead::StbyRc,
            3 => OperatingModeRead::StbyOsc,
            4 => OperatingModeRead::Fs,
            5 => OperatingModeRead::Rx,
            6 => OperatingModeRead::Tx,
            1 => {
                println!("1 returned for operating mode. Investigate (\"RFU\" in DS; the future is now.)");
                OperatingModeRead::Fs // bogus
            }
            _ => return Err(UnexpectedStatus(om)),
        };

        let command_status = match c_s {
            1 => {
                match self.config {
                    RadioConfig::R6x(_) => {
                        println!("1 returned for command status. Investigate (\"RFU\" in DS; the future is now.)");
                        CommandStatus::FailureToExecuteCommand // bogus
                                                               // todo: This should be removed in favof of unexpected status.
                    }
                    RadioConfig::R8x(_) => CommandStatus::CommandProcessSuccess8x,
                }
            }
            2 => CommandStatus::DataAvailable,
            3 => CommandStatus::CommandTimeout,
            4 => CommandStatus::CommandProcessingError,
            5 => CommandStatus::FailureToExecuteCommand,
            6 => CommandStatus::CommandTxDone,
            _ => return Err(UnexpectedStatus(c_s)),
        };

        Ok((operating_mode, command_status))
    }

    /// 6x DS, section 13.5.2. This loads information related to the received payload; it may be useful
    /// in decoding the buffer.
    /// 8x, section 11.8.1 (Same as 6x, other than opcode addr)
    pub fn get_rx_buffer_status(&mut self) -> Result<RxBufferStatus, RadioError> {
        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::GetRxBufferStatus as u8,
            RadioConfig::R8x(_) => OpCode::GetRxBufferStatus.val_8x(),
        };
        let mut buf = [op_code, 0, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(RxBufferStatus {
            status: buf[1],
            payload_len: buf[2],
            rx_start_buf_pointer: buf[3],
        })
    }

    /// 6x only.
    pub fn get_device_errors(&mut self) -> Result<u16, RadioError> {
        let mut buf = [OpCode::GetDeviceErrors as u8, 0, 0, 0];
        self.interface.read(&mut buf)?;

        // Status avail at byte 2.
        Ok(u16::from_be_bytes([buf[2], buf[3]]))
    }
}
