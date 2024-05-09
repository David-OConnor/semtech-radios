//! Code relating to configuring the radio.

use defmt::println;

use crate::{
    shared::{OpCode, RadioError},
    time_bytes_6x, time_bytes_8x, OperatingMode, PacketType, Radio, RadioConfig, FREQ_CONST_6X,
    FREQ_CONST_8X,
};

impl Radio {
    /// 6x: See DS, section 13.4.1 for this computation.
    /// 8x: See DS, section 11.7.3.
    pub(crate) fn set_rf_freq(&mut self) -> Result<(), RadioError> {
        match &self.config {
            RadioConfig::R6x(config) => {
                // We convert to u64 to prevent an overflow.
                let rf_freq_raw = ((config.rf_freq as f32 / FREQ_CONST_6X) as u32).to_be_bytes();

                self.interface.write(&[
                    OpCode::SetRfFrequency as u8,
                    rf_freq_raw[0],
                    rf_freq_raw[1],
                    rf_freq_raw[2],
                    rf_freq_raw[3],
                ])
            }
            RadioConfig::R8x(config) => {
                // See section 4.3, and this from the table.
                // "The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz, where 52e6 is the crystal frequency in Hz. SetRfFrequency()
                // defines the Tx frequency. The Rx frequency is down-converted to the IF. The IF is set by default to 1.3 MHz. This
                // configuration is handled internally by the transceiver, there is no need for the user to take this offset into account when
                // configuring SetRfFrequency. This must be called after SetPacket type."
                let rf_freq_raw = ((config.rf_freq as f32 / FREQ_CONST_8X) as u32).to_be_bytes();
                // todo: Of note, the DS example for this seems wrong... Would like to close the loop on that.

                self.interface.write(&[
                    OpCode::SetRfFrequency.val_8x(),
                    rf_freq_raw[0],
                    rf_freq_raw[1],
                    rf_freq_raw[2],
                ])
            }
        }
    }

    /// Send modulation parameters found in the config, to the radio.
    /// 6x DS, section 13.4.5. Parameters depend on the packet type.
    /// 8x DS: Section 11.7.7
    pub fn set_mod_params(&mut self) -> Result<(), RadioError> {
        match &self.config {
            RadioConfig::R6x(config) => {
                let mut p1 = 0;
                let mut p2 = 0;
                let mut p3 = 0;
                let mut p4 = 0;
                let p5 = 0;
                let p6 = 0;
                let p7 = 0;
                let p8 = 0;

                match config.packet_type {
                    PacketType::Gfsk => {
                        unimplemented!()
                    }
                    PacketType::Lora => {
                        p1 = config.modulation_params.spreading_factor as u8;
                        p2 = config.modulation_params.mod_bandwidth as u8;
                        p3 = config.modulation_params.coding_rate as u8;
                        p4 = config.modulation_params.low_data_rate_optimization as u8;
                    }
                    PacketType::LrFhssFlrc => {
                        // todo: FHSS on 8x
                        unimplemented!()
                    }
                    _ => unimplemented!(),
                }

                // todo: Confirm we can ignore unused params.

                self.interface.write(&[
                    OpCode::SetModulationParams as u8,
                    p1,
                    p2,
                    p3,
                    p4,
                    p5,
                    p6,
                    p7,
                    p8,
                ])
            }
            RadioConfig::R8x(config) => {
                let mut p1 = 0;
                let mut p2 = 0;
                let mut p3 = 0;

                match config.packet_type {
                    PacketType::Gfsk => {
                        unimplemented!()
                    }
                    PacketType::Lora => {
                        p1 = config.modulation_params.spreading_factor as u8;
                        p2 = config.modulation_params.mod_bandwidth as u8;
                        p3 = config.modulation_params.coding_rate as u8;
                    }
                    PacketType::LrFhssFlrc => {
                        // todo: Implement for FLRC
                        unimplemented!()
                    }
                    _ => unimplemented!(),
                }

                self.interface
                    .write(&[OpCode::SetModulationParams.val_8x(), p1, p2, p3])
            }
        }
    }

    /// Send packet parameters found in the config, to the radio.
    /// 6x: DS, section 13.4.6.
    /// 8x: DS, section 11.7.8
    pub(crate) fn set_packet_params(&mut self) -> Result<(), RadioError> {
        let mut p1 = 0;
        let mut p2 = 0;
        let mut p3 = 0;
        let mut p4 = 0;
        let mut p5 = 0;
        let mut p6 = 0;
        let p7 = 0;
        let p8 = 0;
        let p9 = 0;

        match &self.config {
            RadioConfig::R6x(config) => {
                // The preamble is between 10 and 65,535 symbols.
                if config.packet_params.preamble_len < 10 {
                    return Err(RadioError::Config);
                }
                match config.packet_type {
                    PacketType::Gfsk => {
                        unimplemented!()
                    }
                    PacketType::Lora => {
                        let preamble_len = config.packet_params.preamble_len.to_be_bytes();

                        p1 = preamble_len[0];
                        p2 = preamble_len[1];
                        p3 = config.packet_params.header_type.val_sx126x();
                        p4 = config.packet_params.payload_len;
                        p5 = config.packet_params.crc_enabled.val_sx126x();
                        p6 = config.packet_params.invert_iq.val_sx126x();
                    }
                    PacketType::LrFhssFlrc => {
                        // todo: Implement for 8x: FHSS.
                        unimplemented!()
                    }
                    _ => unimplemented!(), // BLE and Ranging.
                }

                // todo: Confirm we can ignore unused params.

                self.interface.write(&[
                    OpCode::SetPacketParams as u8,
                    p1,
                    p2,
                    p3,
                    p4,
                    p5,
                    p6,
                    p7,
                    p8,
                    p9,
                ])
            }
            RadioConfig::R8x(config) => {
                // Check preamble. len. Recommended: 12.
                // if config.packet_params.preamble_len < 10 {
                //     return Err(RadioError::Config);
                // }

                let mut p1 = 0;
                let mut p2 = 0;
                let mut p3 = 0;
                let mut p4 = 0;
                let mut p5 = 0;
                let mut p6 = 0;
                let p7 = 0;

                match config.packet_type {
                    PacketType::Gfsk => {
                        unimplemented!()
                    }
                    PacketType::Lora => {
                        // Note: The preamble here is handled differently from SX126x, to fit in a single param.
                        // preamble length = LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
                        let pble_len_maint = (config.packet_params.preamble_len & 0xf) as u8;
                        // Hard-set this at 0 for now; use `maint` raw. Limited to up to 16 until this is changed.
                        let pble_len_exp: u8 = 0;

                        // todo: QC order.
                        let preamble_len = ((pble_len_exp & 0xf) << 4) | (pble_len_maint & 0xf);

                        p1 = preamble_len;
                        p2 = config.packet_params.header_type.val_sx128x();
                        p3 = config.packet_params.payload_len;
                        p4 = config.packet_params.crc_enabled.val_sx128x();
                        p5 = config.packet_params.invert_iq.val_sx128x();
                    }
                    PacketType::LrFhssFlrc => {
                        // todo: Implement this for sx1280: FLRC
                        unimplemented!()
                    }
                    _ => unimplemented!(), // BLE and ranging.
                }

                // todo: Confirm we can ignore unused params.

                self.interface.write(&[
                    OpCode::SetPacketParams.val_8x(),
                    p1,
                    p2,
                    p3,
                    p4,
                    p5,
                    p6,
                    p7,
                ])
            }
        }
    }

    /// 6x only. See DS, section 13.1.14. These settings should be hard-set to specific values.
    /// See Table 13-21: PA Operating Modes and Optimal Settings for how to set this.
    pub(crate) fn set_pa_config(&mut self) -> Result<(), RadioError> {
        match &self.config {
            RadioConfig::R6x(config) => {
                let (duty_cycle, hp_max) = config.output_power.dutycycle_hpmax();
                // Byte 3 is always 0 for sx1262 (1 for 1261). Byte 4 is always 1.
                self.interface
                    .write(&[OpCode::SetPAConfig as u8, duty_cycle, hp_max, 0, 1])
            }
            _ => unimplemented!(),
        }
    }

    /// 6x DS, section 13.4.4
    /// The output power is defined as power in dBm in a range of
    /// - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
    /// - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
    ///
    /// 8x: 13db is max: power = 31 (0x1f)
    pub(crate) fn set_tx_params(&mut self) -> Result<(), RadioError> {
        let (power, ramp_time) = match &self.config {
            RadioConfig::R6x(config) => {
                (config.output_power as u8, config.ramp_time as u8) // Max power.
            }
            RadioConfig::R8x(config) => {
                assert!(config.output_power >= -18 && config.output_power <= 13);
                ((config.output_power + 18) as u8, config.ramp_time as u8) // Max power.
            }
        };

        self.interface
            .write(&[OpCode::SetTxParams as u8, power, ramp_time])
    }

    /// Sets the device into sleep mode; the lowest current consumption possible. Wake up by setting CS low.
    pub fn set_op_mode(&mut self, mode: OperatingMode) -> Result<(), RadioError> {
        match mode {
            // todo: This behavior is 6x only. 8x is different (?)
            OperatingMode::Sleep(cfg) => {
                self.interface
                    // todo: Wake-up on RTC A/R.
                    .write_op_word(OpCode::SetSleep, (cfg as u8) << 2)
            }

            OperatingMode::StbyRc => self.interface.write_op_word(OpCode::SetStandby, 0),
            OperatingMode::StbyOsc => self.interface.write_op_word(OpCode::SetStandby, 1),
            OperatingMode::Fs => self.interface.write(&[OpCode::SetFS as u8]),
            OperatingMode::Tx(timeout) => {
                let (op_code, to_bytes) = match self.config {
                    RadioConfig::R6x(_) => (OpCode::SetTx as u8, time_bytes_6x(timeout)),
                    RadioConfig::R8x(_) => (OpCode::SetTx.val_8x(), time_bytes_8x(timeout)),
                };
                self.interface
                    .write(&[op_code, to_bytes[0], to_bytes[1], to_bytes[2]])
            }
            OperatingMode::Rx(timeout) => {
                let (op_code, to_bytes) = match self.config {
                    RadioConfig::R6x(_) => (OpCode::SetRx as u8, time_bytes_6x(timeout)),
                    RadioConfig::R8x(_) => (OpCode::SetRx.val_8x(), time_bytes_8x(timeout)),
                };
                self.interface
                    .write(&[op_code, to_bytes[0], to_bytes[1], to_bytes[2]])
            }
        }
    }
}
