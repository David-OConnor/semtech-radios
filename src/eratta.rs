//! Eratta workarounds

use crate::{
    params::LoraBandwidthSX126x,
    shared::{RadioError, Register, Register6x},
    PacketType, Radio, RadioConfig,
};

impl Radio {
    /// (6x only) See DS, section 9.6: Receive (RX) Mode).
    pub fn set_rxgain_retention(&mut self) -> Result<(), RadioError> {
        self.interface
            .write_reg_word(Register::Reg6x(Register6x::RxGainRetention0), 0x01)?;
        self.interface
            .write_reg_word(Register::Reg6x(Register6x::RxGainRetention1), 0x08)?;
        self.interface
            .write_reg_word(Register::Reg6x(Register6x::RxGainRetention2), 0xac)
    }

    /// (6x only) See DS, section 15.2.2.
    pub fn tx_clamp_workaround(&mut self) -> Result<(), RadioError> {
        let val = self
            .interface
            .read_reg_word(Register::Reg6x(Register6x::TxClampConfig))?;
        self.interface
            .write_reg_word(Register::Reg6x(Register6x::TxClampConfig), val | 0x1e)
    }

    /// DS, section 16.1.2. Adapted from pseudocode there.
    /// (6x only)
    pub fn mod_quality_workaround(&mut self) -> Result<(), RadioError> {
        let mut value = self
            .interface
            .read_reg_word(Register::Reg6x(Register6x::TxModulation))?;

        match &self.config {
            RadioConfig::R6x(config) => {
                if config.packet_type == PacketType::Lora
                    && config.modulation_params.mod_bandwidth == LoraBandwidthSX126x::BW_500
                {
                    value &= 0xFB;
                } else {
                    value |= 0x04;
                }

                // todo: QC how this works with u8 vs u16.
                self.interface
                    .write_reg_word(Register::Reg6x(Register6x::TxModulation), value)
            }
            _ => unimplemented!(),
        }
    }

    /// (6x only) See DS, section 15.3.2
    /// "It is advised to add the following commands after ANY Rx with Timeout active sequence, which stop the RTC and clear the
    /// timeout event, if any."
    pub fn implicit_header_to_workaround(&mut self) -> Result<(), RadioError> {
        // todo DS typo: Shows 0920 which is a diff one in code snipped.
        self.interface
            .write_reg_word(Register::Reg6x(Register6x::RtcControl), 0x00)?;
        let val = self
            .interface
            .read_reg_word(Register::Reg6x(Register6x::EventMask))?;
        self.interface
            .write_reg_word(Register::Reg6x(Register6x::EventMask), val | 0x02)
    }
}
