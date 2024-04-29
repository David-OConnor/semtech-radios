//! Code to use the SX1262 LoRa radio.

use defmt::println;
use hal::{dma::DmaChannel, gpio::Pin};

use super::{
    params::{LoraBandwidthSX126x, ModulationParamsLoraSx126x, PacketParamsLora},
    spi_interface::Interface,
};
use crate::{
    shared::{OpCode, RadioError, RadioPins, Register126x},
    spi_interface::{Spi_, RADIO_BUF_SIZE},
};
use crate::params::ModulationParamsLoraSx128x;

/// The timing factor used to convert between 24-bit integer timing conversions used
/// by the radio, and ms. Eg: Sleep Duration = sleepPeriod * 15.625 µs
const TIMING_FACTOR_MS: f32 = 0.015_625;

const F_XTAL: u64 = 32_000_000; // Oscillator frequency in Mhz.
                                // The radio's maximum data buffer size.

/// DS, 13.4.2. Table 13-38.  The switch from one frame to another must be done in STDBY_RC mode.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[allow(dead_code)]
pub enum PacketType {
    /// (G)Fsk
    Gfsk = 0,
    Lora = 1,
    /// Long Range FHSS
    Fhss = 3,
}

#[repr(u16)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
/// DS, table 12-1. Differentiate the LoRa signal for Public or Private network.
/// set the `LoRa Sync word MSB and LSB values to this.
pub enum LoraNetwork {
    Public = 0x3444,  // corresponds to sx127x 0x34
    Private = 0x1424, // corresponds to sx127x 0x12
}

/// DS, Table 13-41. Power ramp time. Titles correspond to ramp time in µs.
/// todo: Figure out guidelines for setting this. The DS doesn't have much on it.
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum RampTime {
    R10 = 0,
    R20 = 1,
    R40 = 2,
    R80 = 3,
    R200 = 4,
    R800 = 5,
    R1700 = 6,
    R3400 = 7,
}

/// DS, Table 13-29. repr as u16, since we use the values to bit-mask a 16-bit integer.
#[repr(u16)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum Irq {
    TxDone = 0,
    RxDone = 1,
    PremableDetected = 2,
    SyncWordValid = 3,
    HeaderValid = 4,
    HeaderErr = 5,
    CrcErr = 6,
    CadDone = 7,
    CadDetected = 8,
    Timeout = 9,
    LrFhssHop = 14,
}

/// These don't take into account the external PA, if applicable.
#[repr(u8)] // For storing in configs.
#[derive(Clone, Copy)]
pub enum OutputPower {
    /// 25mW
    Db14,
    /// 50mW
    Db17,
    /// 100mW
    Db20,
    /// 158mW
    Db22,
}

impl Default for OutputPower {
    fn default() -> Self {
        Self::Db22 // full power
    }
}

impl OutputPower {
    /// See datasheet, table 13-21
    /// For HP Max: 0 - 7. Do not set above 7, or you could cause early aging of the device. 7 sets max power,
    ///  achieve +22dBm.
    pub fn dutycycle_hpmax(&self) -> (u8, u8) {
        match self {
            Self::Db14 => (0x02, 0x02),
            Self::Db17 => (0x02, 0x03),
            Self::Db20 => (0x03, 0x05),
            Self::Db22 => (0x04, 0x07),
        }
    }
}

/// DS, 13.1.15. This defines the mode the radio goes into after a successful Tx or Rx.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum FallbackMode {
    Fs = 0x40,
    StdbyXosc = 0x30,
    StdbyRc = 0x20,
}

/// DS, section 9.6: Receive (RX) Mode
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum RxMode {
    Continuous,
    Single,
    SingleWithTimeout,
    Listen,
}

/// DS, section 13.1.1
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum SleepConfig {
    ColdStart = 0,
    WarmStart = 1,
}

/// DS, section 9.
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum OperatingMode {
    /// In this mode, most of the radio internal blocks are powered down or in low power mode and optionally the RC64k clock
    /// and the timer are running.
    Sleep(SleepConfig),
    /// In standby mode the host should configure the chip before going to RX or TX modes. By default in this state, the system is
    /// clocked by the 13 MHz RC oscillator to reduce power consumption (in all other modes except SLEEP the XTAL is turned ON).
    /// However if the application is time critical, the XOSC block can be turned or left ON.
    StbyRc,
    StbyOsc,
    /// In FS mode, PLL and related regulators are switched ON. The BUSY goes low as soon as the PLL is locked or timed out.
    /// The command SetFs() is used to set the device in the frequency synthesis mode where the PLL is locked to the carrier
    /// frequency. This mode is used for test purposes of the PLL and can be considered as an intermediate mode. It is
    /// automatically reached when going from STDBY_RC mode to TX mode or RX mode.
    Fs,
    /// The inner value is the timeout, in ms.
    Tx(f32),
    Rx(f32),
}

/// DS, Table 13-76
/// Without the inner values, eg from read status, and without sleep, which can't be read.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, defmt::Format, Debug)]
pub enum OperatingModeRead {
    StbyRc = 2,
    StbyOsc = 3,
    Fs = 4,
    Tx = 6,
    Rx = 5,
}

/// DS, section 13.5.2
pub struct RxBufferStatus {
    pub status: u8,
    pub payload_len: u8,
    pub rx_start_buf_pointer: u8,
}

/// DS, section 13.5.3. Contains link statistics from the previously packet.
/// Note that this contains the raw data, for easy sending on the wire. Convert to appropriate
/// units at the consumer side.
/// todo: FSK A/R; same field count, but with different meanings.
#[derive(Default)]
pub struct RxPacketStatusLora {
    pub status: u8,
    /// Average over last packet received of RSSI
    /// Actual signal power is –RssiPkt/2 (dBm).
    pub rssi: u8,
    /// Estimation of SNR on last packet received in two’s compliment format.
    pub snr: u8,
    /// stimation of RSSI of the LoRa® signal (after despreading) on last packet received
    /// Actual signal power is –RssiPkt/2 (dBm).
    pub signal_rssi: u8,
}

/// DS, section 13.5.4.
pub struct RxStatistics {
    pub status: u8,
    pub num_received: u16,
    pub num_crc_error: u16,
    pub num_length_error: u16,
}

/// DS, section 13.5.1
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, defmt::Format, Debug)]
pub enum CommandStatus {
    /// A packet has been successfully received and data can be retrieved
    DataAvailable = 2,
    /// A transaction from host took too long to complete and triggered an internal watchdog. The watchdog mechanism can be disabled by host; it
    /// is meant to ensure all outcomes are flagged to the host MCU
    CommandTimeout = 3,
    /// Processor was unable to process command either because of an invalid opcode or because an incorrect number of parameters has been
    /// provided.
    CommandProcessingError = 4,
    /// The command was successfully processed, however the chip could not execute the command; for instance it was unable to enter the specified
    /// device mode or send the requested data
    FailureToExecuteCommand = 5,
    /// The transmission of the current packet has terminated
    CommandTxDone = 6,
}

/// See [this interactive tool for info on how these parameters effect OTA time etc](https://www.semtech.com/design-support/lora-calculator)
/// It also includes consumption, RFIO schematics etc. Use `Shared RFIO`, vice a switch; breaks the calculator.
#[derive(Clone)]
pub struct RadioConfig {
    pub packet_type: PacketType,
    /// RF frequency in Hz.
    pub rf_freq: u32,
    pub use_dio2_as_rfswitch: bool,
    pub dc_dc_enabled: bool,
    pub modulation_params: ModulationParamsLoraSx126x,
    // todo: Sort this out.
    pub modulation_params_sx128x: ModulationParamsLoraSx128x,
    pub packet_params: PacketParamsLora,
    // todo: FSK A/R.
    // todo: FHSS? May be appropriate for your use case. See DS, section 6.3.
    /// Raw register value. Timeout duration = timeout * 15.625µs.
    pub tx_timeout: f32,
    pub rx_timeout: f32,
    pub fallback_mode: FallbackMode,
    pub ramp_time: RampTime,
    pub lora_network: LoraNetwork,
    pub output_power: OutputPower,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            packet_type: PacketType::Lora,
            rf_freq: 915_000_000,
            use_dio2_as_rfswitch: true,
            dc_dc_enabled: true,
            modulation_params: Default::default(),
            packet_params: Default::default(),
            tx_timeout: 0., // todo: Calculate this based on packet and mod params?
            rx_timeout: 0.,
            fallback_mode: FallbackMode::StdbyRc,
            ramp_time: RampTime::R200, // todo: What should this be?
            lora_network: LoraNetwork::Private,
            output_power: OutputPower::Db22,
        }
    }
}

pub struct Radio {
    pub interface: Interface,
    pub config: RadioConfig,
}

impl Radio {
    /// Initialize the radio. See DS section 14.5: Issuing Commands in the Right Order.
    ///
    /// Most of the commands can be sent in any order except for the radio configuration commands which will set the radio in
    /// the proper operating mode.
    /// ... (See inline comments prior to the mandatory order of the first 3 steps)
    /// If this order is not respected, the behavior of the device could be unexpected.
    pub fn new(
        config: RadioConfig,
        spi: Spi_,
        pins: RadioPins,
        tx_ch: DmaChannel,
        rx_ch: DmaChannel,
    ) -> Result<Self, RadioError> {
        let mut result = Self {
            config,
            interface: Interface {
                spi,
                pins,
                tx_ch,
                rx_ch,
                read_buf: [0; RADIO_BUF_SIZE],
                write_buf: [0; RADIO_BUF_SIZE],
                rx_payload_len: 0,
                rx_payload_start: 0,
            },
            // operating_mode: OperatingMode::StbyOsc,
        };

        result.interface.reset();

        // DS, section 9.1:
        //"At power-up or after a reset, the chip goes into STARTUP state, the control of the chip being done by the sleep state
        // machine operating at the battery voltage. The BUSY pin is set to high indicating that the chip is busy and cannot accept a
        // command. When the digital voltage and RC clock become available, the chip can boot up and the CPU takes control. At this
        // stage the BUSY line goes down and the device is ready to accept commands.
        // "
        result.interface.wait_on_busy()?;

        // Note: This is required to change some settings, like packet type.
        result.set_op_mode(OperatingMode::StbyRc)?;

        // Make sure we're in STDBY_RC mode when setting packet type.
        // "it is mandatory to set the radio protocol using the command SetPacketType(...) as a first
        // step before issuing any other radio configuration commands."
        result
            .interface
            .write_op_word(OpCode::SetPacketType, result.config.packet_type as u8)?;

        // Note: In the Tx/Rx recipes in the DS, this is before setting mod parameters; but it's not listed
        // this way in the part on section 9.1.
        result.set_rf_freq()?;

        // "In a second step, the user should define the modulation
        // parameter according to the chosen protocol with the command SetModulationParams(...)."
        result.set_mod_params_sx126x()?;

        // Finally, the user should then
        // select the packet format with the command SetPacketParams(...).
        result.set_packet_params_sx126x()?;

        result.set_rxgain_retention()?;
        result.tx_clamp_workaround()?;

        // Use the LDO, or DC-DC setup as required, based on hardware config.
        result
            .interface
            .write_op_word(OpCode::SetRegulatorMode, result.config.dc_dc_enabled as u8)?;

        result.set_pa_config()?;

        result
            .interface
            .write_op_word(OpCode::SetTxFallbackMode, result.config.fallback_mode as u8)?;

        result.interface.write_op_word(
            OpCode::SetDIO2AsRfSwitchCtrl,
            result.config.use_dio2_as_rfswitch as u8,
        )?;

        result.set_tx_params()?;

        // Note: Not required if private due to the reset value.
        result.set_sync_word(result.config.lora_network)?;

        let tx_addr = 0;
        let rx_addr = 0;
        result
            .interface
            .write(&[OpCode::SetBufferBaseAddress as u8, tx_addr, rx_addr])?;

        Ok(result)
    }

    /// See DS, section 13.4.1 for this computation.
    fn set_rf_freq(&mut self) -> Result<(), RadioError> {
        // We convert to u64 to prevent an overflow.
        let rf_freq_raw =
            ((self.config.rf_freq as u64 * 2_u64.pow(25) / F_XTAL) as u32).to_be_bytes();

        self.interface.write(&[
            OpCode::SetRfFrequency as u8,
            rf_freq_raw[0],
            rf_freq_raw[1],
            rf_freq_raw[2],
            rf_freq_raw[3],
        ])
    }

    /// See DS, section 9.6: Receive (RX) Mode).
    fn set_rxgain_retention(&mut self) -> Result<(), RadioError> {
        self.interface
            .write_reg_word(Register126x::RxGainRetention0, 0x01)?;
        self.interface
            .write_reg_word(Register126x::RxGainRetention1, 0x08)?;
        self.interface
            .write_reg_word(Register126x::RxGainRetention2, 0xac)
    }

    /// See DS, section 15.2.2.
    fn tx_clamp_workaround(&mut self) -> Result<(), RadioError> {
        let val = self.interface.read_reg_word(Register126x::TxClampConfig)?;
        self.interface
            .write_reg_word(Register126x::TxClampConfig, val | 0x1e)
    }

    /// DS, section 16.1.2. Adapted from pseudocode there.
    /// todo: Sx126x only UFN.
    fn mod_quality_workaround(&mut self) -> Result<(), RadioError> {
        let mut value = self.interface.read_reg_word(Register126x::TxModulation)?;
        if self.config.packet_type == PacketType::Lora
            && self.config.modulation_params.mod_bandwidth == LoraBandwidthSX126x::BW_500
        {
            value &= 0xFB;
        } else {
            value |= 0x04;
        }

        // todo: QC how this works with u8 vs u16.
        self.interface
            .write_reg_word(Register126x::TxModulation, value)
    }

    /// See DS, section 15.3.2
    /// "It is advised to add the following commands after ANY Rx with Timeout active sequence, which stop the RTC and clear the
    /// timeout event, if any."
    pub fn implicit_header_to_workaround(&mut self) -> Result<(), RadioError> {
        // todo DS typo: Shows 0920 which is a diff one in code snipped.
        self.interface
            .write_reg_word(Register126x::RtcControl, 0x00)?;
        let val = self.interface.read_reg_word(Register126x::EventMask)?;
        self.interface
            .write_reg_word(Register126x::EventMask, val | 0x02)
    }

    /// Send modulation parameters found in the config, to the radio.
    /// DS, section 13.4.5. Parameters depend on the packet tyhpe.
    pub fn set_mod_params_sx126x(&mut self) -> Result<(), RadioError> {
        let mut p1 = 0;
        let mut p2 = 0;
        let mut p3 = 0;
        let mut p4 = 0;
        let p5 = 0;
        let p6 = 0;
        let p7 = 0;
        let p8 = 0;

        match self.config.packet_type {
            PacketType::Gfsk => {
                unimplemented!()
            }
            PacketType::Lora => {
                p1 = self.config.modulation_params.spreading_factor as u8;
                p2 = self.config.modulation_params.mod_bandwidth as u8;
                p3 = self.config.modulation_params.coding_rate as u8;
                p4 = self.config.modulation_params.low_data_rate_optimization as u8;
            }
            PacketType::Fhss => {
                unimplemented!()
            }
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

    /// Send modulation parameters found in the config, to the radio.
    /// DS: Section 11.7.7
    pub fn set_mod_params_sx128x(&mut self) -> Result<(), RadioError> {
        let mut p1 = 0;
        let mut p2 = 0;
        let mut p3 = 0;

        match self.config.packet_type {
            PacketType::Gfsk => {
                unimplemented!()
            }
            PacketType::Lora => {
                p1 = self.config.modulation_params_sx128x.spreading_factor as u8;
                p2 = self.config.modulation_params_sx128x.mod_bandwidth as u8;
                p3 = self.config.modulation_params_sx128x.coding_rate as u8;
            }
            PacketType::Fhss => {
                unimplemented!()
            }
        }

        // todo: Confirm we can ignore unused params.

        self.interface.write(&[
            OpCode::SetModulationParams.val_sx128x(),
            p1,
            p2,
            p3,
        ])
    }

    /// Send packet parameters found in the config, to the radio.
    /// DS, section 13.4.6.
    fn set_packet_params_sx126x(&mut self) -> Result<(), RadioError> {
        // The preamble is between 10 and 65,535 symbols.
        if self.config.packet_params.preamble_len < 10 {
            return Err(RadioError::Config);
        }

        let mut p1 = 0;
        let mut p2 = 0;
        let mut p3 = 0;
        let mut p4 = 0;
        let mut p5 = 0;
        let mut p6 = 0;
        let p7 = 0;
        let p8 = 0;
        let p9 = 0;

        match self.config.packet_type {
            PacketType::Gfsk => {
                unimplemented!()
            }
            PacketType::Lora => {
                let preamble_len = self.config.packet_params.preamble_len.to_be_bytes();

                p1 = preamble_len[0];
                p2 = preamble_len[1];
                p3 = self.config.packet_params.header_git addtype.val_sx126x();
                p4 = self.config.packet_params.payload_len;
                p5 = self.config.packet_params.crc_enabled.val_sx126x();
                p6 = self.config.packet_params.invert_iq.val_sx126x();
            }
            PacketType::Fhss => {
                unimplemented!()
            }
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

    /// Send packet parameters found in the config, to the radio.
    /// DS, section 11.7.8
    fn set_packet_params_sx128x(&mut self) -> Result<(), RadioError> {
        // Check preamble. len. Recommended: 12.
        // if self.config.packet_params.preamble_len < 10 {
        //     return Err(RadioError::Config);
        // }

        let mut p1 = 0;
        let mut p2 = 0;
        let mut p3 = 0;
        let mut p4 = 0;
        let mut p5 = 0;
        let mut p6 = 0;
        let p7 = 0;
        let p8 = 0;
        let p9 = 0;

        match self.config.packet_type {
            PacketType::Gfsk => {
                unimplemented!()
            }
            PacketType::Lora => {
                // Note: The preamble here is handled differently from SX126x, to fit in a single param.
                // preamble length = LORA_PBLE_LEN_MANT*2^(LORA_PBLE_LEN_EXP)
                let pble_len_maint = (self.config.packet_params.preamble_len & 0xf) as u8;
                // Hard-set this at 0 for now; use `maint` raw. Limited to up to 16 until this is changed.
                let pble_len_exp: u8 = 0;

                // todo: QC order.
                let preamble_len = ((pble_len_exp & 0xf) << 4) | (pble_len_maint & 0xf);

                p1 = preamble_len;
                p2 = self.config.packet_params.header_type.val_sx128x();
                p3 = self.config.packet_params.payload_len;
                p4 = self.config.packet_params.crc_enabled.val_sx128x();
                p5 = self.config.packet_params.invert_iq.val_sx128x();
            }
            PacketType::Fhss => {
                unimplemented!()
            }
        }

        // todo: Confirm we can ignore unused params.

        self.interface.write(&[
            OpCode::SetPacketParams.val_sx128x(),
            p1,
            p2,
            p3,
            p4,
            p5,
            p6,
            p7,
        ])
    }

    /// See DS, section 13.1.14. These settings should be hard-set to specific values.
    /// See Table 13-21: PA Operating Modes and Optimal Settings for how to set this.
    fn set_pa_config(&mut self) -> Result<(), RadioError> {
        let (duty_cycle, hp_max) = self.config.output_power.dutycycle_hpmax();
        // Byte 3 is always 0 for sx1262 (1 for 1261). Byte 4 is always 1.
        self.interface
            .write(&[OpCode::SetPAConfig as u8, duty_cycle, hp_max, 0, 1])
    }

    /// DS, section 13.4.4
    /// The output power is defined as power in dBm in a range of
    /// - 17 (0xEF) to +14 (0x0E) dBm by step of 1 dB if low power PA is selected
    /// - 9 (0xF7) to +22 (0x16) dBm by step of 1 dB if high power PA is selected
    fn set_tx_params(&mut self) -> Result<(), RadioError> {
        let power = 0x16; // Max power.
        self.interface.write(&[
            OpCode::SetTxParams as u8,
            power,
            self.config.ramp_time as u8,
        ])
    }

    /// Sets the device into sleep mode; the lowest current consumption possible. Wake up by setting CS low.
    pub fn set_op_mode(&mut self, mode: OperatingMode) -> Result<(), RadioError> {
        match mode {
            OperatingMode::Sleep(cfg) => self
                .interface
                .write_op_word(OpCode::SetSleep, (cfg as u8) << 2),
            OperatingMode::StbyRc => self.interface.write_op_word(OpCode::SetStandby, 0),
            OperatingMode::StbyOsc => self.interface.write_op_word(OpCode::SetStandby, 1),
            OperatingMode::Fs => self.interface.write(&[OpCode::SetFS as u8]),
            OperatingMode::Tx(timeout) => {
                let to_bytes = time_bytes(timeout);
                self.interface
                    .write(&[OpCode::SetTx as u8, to_bytes[0], to_bytes[1], to_bytes[2]])
            }
            OperatingMode::Rx(timeout) => {
                let to_bytes = time_bytes(timeout);
                self.interface
                    .write(&[OpCode::SetRx as u8, to_bytes[0], to_bytes[1], to_bytes[2]])
            }
        }
    }

    /// DS, section 14.2. Frequency is set here and in receive initiation, for use with frequency hopping.
    pub fn send_payload(&mut self, payload: &[u8], rf_freq: u32) -> Result<(), RadioError> {
        let payload_len = payload.len();
        if payload_len > 255 {
            return Err(RadioError::PayloadSize(payload_len));
        }

        // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
        self.set_op_mode(OperatingMode::StbyRc)?;

        // See DS, section 15.1.2: Work around this eratta before each transmission
        self.mod_quality_workaround()?;

        // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
        // (Set on init)
        // self.interface
        //     .write_op_word(OpCode::SetPacketType, self.config.packet_type as u8)?;

        // 3. Define the RF frequency with the command SetRfFrequency(...)
        self.config.rf_freq = rf_freq;
        self.set_rf_freq()?;

        // 4. Define the Power Amplifier configuration with the command SetPaConfig(...)
        // (Set on init)
        // self.set_pa_config()?;

        // 5. Define output power and ramping time with the command SetTxParams(...)
        // (Set on init)

        // 6. Define where the data payload will be stored with the command SetBufferBaseAddress(...)
        let tx_addr = 0;
        let rx_addr = 0;
        self.interface
            .write(&[OpCode::SetBufferBaseAddress as u8, tx_addr, rx_addr])?;

        // 7. Send the payload to the data buffer with the command WriteBuffer(...)
        let offset = 0;
        // todo: Put back when reading.
        // self.interface
        //     .write_with_payload(payload, offset)?;

        // todo: Having trouble with SPI DMA writes (but reads work) Skipping for now; blocking.
        {
            let mut write_buf = [0; RADIO_BUF_SIZE as usize];
            write_buf[0] = OpCode::WriteBuffer as u8;
            write_buf[1] = offset;
            for (i, word) in payload.iter().enumerate() {
                write_buf[i + 2] = *word;
            }
            self.interface.write(&write_buf[..2 + payload_len])?;
        }

        // 8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
        // (set on init)
        // self.set_mod_params()?;

        // 9. Define the frame format to be used with the command SetPacketParams(...)
        self.config.packet_params.payload_len = payload_len as u8;
        self.set_packet_params_sx126x()?;

        // 10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1,
        // DIO2 or DIO3)
        // (Currently without DMA, this is handled in `start_transmission`.
        // (See `handle_tx_post_payload_write()`).

        // 11. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access
        // (Set on init)
        // self.set_sync_word(self.config.lora_network)?;

        // 12. Set the circuit in transmitter mode to start transmission with the command SetTx(). Use the parameter to enable
        // Timeout
        self.start_transmission()?; // todo: Handle in ISR once using DMA.
                                    // (Handled in SPI Tx complete ISR)

        // 13. Wait for the IRQ TxDone or Timeout: once the packet has been sent the chip goes automatically to STDBY_RC mode
        // (Handled by waiting for a firmware GPIO ISR)

        // 14. Clear the IRQ TxDone flag. Ie: radio.clear_irq(&[Irq::TxDone, Irq::Timeout])?;
        // (Handled in firmware GPIO ISR)

        Ok(())
    }

    /// Run these from the SPI Tx complete ISR. This initiates transmission; run this once the
    /// payload write to the radio's buffer is complete.
    pub fn start_transmission(&mut self) -> Result<(), RadioError> {
        self.set_irq(&[Irq::TxDone, Irq::Timeout], &[])?; // DIO 1
        self.set_op_mode(OperatingMode::Tx(self.config.rx_timeout))?;

        Ok(())
    }

    /// Set the radio into receive mode. DS, section 14.3.
    pub fn receive(&mut self, max_payload_len: u8, rf_freq: u32) -> Result<(), RadioError> {
        // 1. If not in STDBY_RC mode, then set the circuit in this mode with the command SetStandby()
        self.set_op_mode(OperatingMode::StbyRc)?;

        // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
        // (Set on init)
        // self.interface
        //     .write_op_word(OpCode::SetPacketType, self.config.packet_type as u8)?;

        // 3.Define the RF frequency with the command SetRfFrequency(...)
        self.config.rf_freq = rf_freq;
        self.set_rf_freq()?;

        // 4. Define where the data will be stored inside the data buffer in Rx with the command SetBufferBaseAddress(...)
        // (Note: We may have to set this here, since I believe this setting auto-increments.)
        let tx_addr = 0;
        let rx_addr = 0;
        self.interface
            .write(&[OpCode::SetBufferBaseAddress as u8, tx_addr, rx_addr])?;

        // 5. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
        // (Set on init)
        self.set_mod_params_sx126x()?;

        // 6. Define the frame format to be used with the command SetPacketParams(...)
        // We must set this, as it may have been changed during a transmission to payload length.
        self.config.packet_params.payload_len = max_payload_len;
        self.set_packet_params_sx126x()?;

        // 7. Configure DIO and irq: use the command SetDioIrqParams(...) to select the IRQ RxDone and map this IRQ to a DIO (DIO1
        // or DIO2 or DIO3), set IRQ Timeout as well.
        self.set_irq(&[], &[Irq::RxDone, Irq::Timeout])?; // DIO3.

        // 8. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access.
        // (Set on init)
        // self.set_sync_word(self.config.lora_network)?;

        // 9. Set the circuit in reception mode: use the command SetRx(). Set the parameter to enable timeout or continuous mode
        self.set_op_mode(OperatingMode::Rx(self.config.rx_timeout))?;

        // 10. Wait for IRQ RxDone2 or Timeout: the chip will stay in Rx and look for a new packet if the continuous mode is selected
        // otherwise it will goes to STDBY_RC mode.
        // (The rest is handled in `cleanup_rx`, called from a firmware GPIO ISR).

        Ok(())
    }

    /// Run these after transmission is complete, eg in an ISR. Clears the IRQ, and reports errors.
    pub fn cleanup_tx(&mut self) -> Result<(), RadioError> {
        self.clear_irq(&[Irq::TxDone, Irq::Timeout])?;

        let status = self.get_status()?;
        if status.0 != OperatingModeRead::StbyRc || status.1 != CommandStatus::CommandTxDone {
            // Note: For Rx cleanup, we allow timeouts, because no message may be received. For Tx,
            // we don't, as this indicates a problem.
            println!(
                "\nProblem with Tx status post-write. Operating mode: {} Command status: {}",
                status.0, status.1
            );
            return Err(RadioError::Status((status.0, status.1)));
        }

        let device_errors = self.get_device_errors()?;
        if device_errors != 0 {
            println!("\nDevice error: {}", device_errors);
            return Err(RadioError::Device);
        }

        Ok(())
    }

    /// Run these after reception is complete, eg in an ISR. Returns buffer status (payload size and start index),
    /// and command status (Data available, timeout etc). Note: If we didn't receive a message, run `clear_irq` instead of this.
    pub fn cleanup_rx(&mut self) -> Result<(RxBufferStatus, CommandStatus), RadioError> {
        let (op_mode, cmd_status) = self.get_status()?;
        if op_mode != OperatingModeRead::StbyRc
            || (cmd_status != CommandStatus::DataAvailable
                && cmd_status != CommandStatus::CommandTimeout)
        {
            println!(
                "\nProblem with Rx status post-read. Operating mode: {} Command status: {}",
                op_mode, cmd_status
            );
            return Err(RadioError::Status((op_mode, cmd_status)));
        }

        // 11. In case of the IRQ RxDone, check the status to ensure CRC is correct: use the command GetIrqStatus()
        // Note:
        // The IRQ RxDone means that a packet has been received but the CRC could be wrong: the user must check the CRC before
        // validating the packet.
        if cmd_status == CommandStatus::DataAvailable {
            let mut irq_status_buf = [OpCode::GetIrqStatus as u8, 0, 0, 0];
            self.interface.read(&mut irq_status_buf)?;
            let irq_status = u16::from_be_bytes([irq_status_buf[2], irq_status_buf[3]]);
            if irq_status & (0b11 << 5) != 0 {
                // Mask for header CRC error or wrong CRC received.
                println!("Irq CRC error post-read: {}", irq_status); // ensure bit 6 isn't set to validate CRC.
                self.clear_irq(&[Irq::RxDone, Irq::Timeout])?; // Clear the IRQ even if we are returning early.
                return Err(RadioError::Crc);
            }
        }

        // 12. Clear IRQ flag RxDone or Timeout: use the command ClearIrqStatus(). In case of a valid packet (CRC OK), get the packet
        // length and address of the first byte of the received payload by using the command GetRxBufferStatus(...)
        // eg in firmware: radio.clear_irq(&[Irq::RxDone, Irq::Timeout])?;
        self.clear_irq(&[Irq::RxDone, Irq::Timeout])?;

        let buf_status = self.get_rx_buffer_status()?;
        println!(
            "Buffer status. Status: {} len: {} start buf: {} ",
            buf_status.status, buf_status.payload_len, buf_status.rx_start_buf_pointer
        );

        self.implicit_header_to_workaround()?; // See eratta, section 15.3.

        let device_errors = self.get_device_errors()?;
        if device_errors != 0 {
            println!("\nDevice error: {}", device_errors);
            return Err(RadioError::Device);
        }

        // return Ok((RxBufferStatus {payload_len: 0, rx_start_buf_pointer: 0, status: 0}, CommandStatus::DataAvailable)); // todo t

        // 13. In case of a valid packet (CRC OK), start reading the packet
        // Note that in the case of a timeout, we get CommandStatus::Timeout, and don't try to read the data.
        if cmd_status == CommandStatus::DataAvailable {
            // self.interface
            //     // .read_with_payload(buf_status.payload_len, buf_status.rx_start_buf_pointer)?;
            //     .read_with_payload(buf_status.payload_len, 0)?;

            // todo TS. It seems DMA may be at the core of your demons.
            // let mut test_buf = unsafe { &mut READ_BUF };
            // let mut test_buf = &mut self.interface.read_buf;
            //
            // test_buf[0] = OpCode::ReadBuffer as u8;
            // test_buf[1] = buf_status.rx_start_buf_pointer;
            // test_buf[2] = 0;
            //
            // if self
            //     .interface
            //     .read(&mut test_buf[..3 + buf_status.payload_len as usize])
            //     .is_err()
            // {
            //     println!("Error reading the buffer");
            // }
        }

        // (Process the payload in the SPI Rx complete ISR)

        Ok((buf_status, cmd_status))
    }

    /// DS, section 13.5.2. This loads information related to the received payload; it may be useful
    /// in decoding the buffer.
    pub fn get_rx_buffer_status(&mut self) -> Result<RxBufferStatus, RadioError> {
        let mut buf = [OpCode::GetRxBufferStatus as u8, 0, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(RxBufferStatus {
            status: buf[1],
            payload_len: buf[2],
            rx_start_buf_pointer: buf[3],
        })
    }

    /// DS, section 13.5.3. This contains useful link stats from a received message.
    pub fn get_packet_status(&mut self) -> Result<RxPacketStatusLora, RadioError> {
        let mut buf = [OpCode::GetPacketStatus as u8, 0, 0, 0, 0];
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
        let mut buf = [OpCode::GetRSSIInst as u8, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(-(buf[2] as i8) / 2)
    }

    /// DS, section 13.5.5
    /// todo: Impl reset as well.
    pub fn get_statistics(&mut self) -> Result<RxStatistics, RadioError> {
        let mut buf = [OpCode::GetStats as u8, 0, 0, 0, 0, 0, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(RxStatistics {
            status: buf[1],
            num_received: u16::from_be_bytes([buf[2], buf[3]]),
            num_crc_error: u16::from_be_bytes([buf[4], buf[5]]),
            num_length_error: u16::from_be_bytes([buf[6], buf[7]]),
        })
    }

    /// DS, section 13.3.1. Setup DIO1 and DIO3 IRQs, which can be used with the MCU's GPU interrupts.
    /// We assume DIO2 controls the Tx/Rx switch.
    fn set_irq(&mut self, dio1: &[Irq], dio3: &[Irq]) -> Result<(), RadioError> {
        let mut irq_word: u16 = 0;
        let mut dio1_word: u16 = 0;
        let mut dio3_word: u16 = 0;

        for irq in dio1 {
            irq_word |= 1 << (*irq as u16);
            dio1_word |= 1 << (*irq as u16);
        }
        for irq in dio3 {
            irq_word |= 1 << (*irq as u16);
            dio3_word |= 1 << (*irq as u16);
        }

        let irq_bytes = irq_word.to_be_bytes();
        let dio1_bytes = dio1_word.to_be_bytes();
        let dio3_bytes = dio3_word.to_be_bytes();

        self.interface.write(&[
            OpCode::SetDioIrqParams as u8,
            irq_bytes[0],
            irq_bytes[1],
            dio1_bytes[0],
            dio1_bytes[1],
            0, // We don't use DIO2; it's configured as Rx/Tx switch.
            0,
            dio3_bytes[0],
            dio3_bytes[1],
        ])
    }

    /// This is a bit confusing, as the register API takes u16, and the sync word is a u16, yet
    /// it's split into two registers.
    fn set_sync_word(&mut self, network: LoraNetwork) -> Result<(), RadioError> {
        let sync_word_bytes = (network as u16).to_be_bytes();

        self.interface
            .write_reg_word(Register126x::LoraSyncWordMsb, sync_word_bytes[0])?;
        self.interface
            .write_reg_word(Register126x::LoraSyncWordLsb, sync_word_bytes[1])?;

        Ok(())
    }

    /// DS, section 13.3.4
    pub fn clear_irq(&mut self, irqs: &[Irq]) -> Result<(), RadioError> {
        // We use a single 16-bit word, with bits at the various values.
        let mut irq_word: u16 = 0;
        for irq in irqs {
            irq_word |= 1 << (*irq as u16);
        }

        let bytes = irq_word.to_be_bytes();
        self.interface
            .write(&[OpCode::ClearIrqStatus as u8, bytes[0], bytes[1]])
    }

    pub fn get_status(&mut self) -> Result<(OperatingModeRead, CommandStatus), RadioError> {
        let mut buf = [OpCode::GetStatus as u8, 0];
        self.interface.read(&mut buf)?;

        let om = (buf[1] >> 4) & 0b111;
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
            _ => return Err(RadioError::UnexpectedStatus(om)),
        };

        let status = (buf[1] >> 1) & 0b111;
        let command_status = match status {
            2 => CommandStatus::DataAvailable,
            3 => CommandStatus::CommandTimeout,
            4 => CommandStatus::CommandProcessingError,
            5 => CommandStatus::FailureToExecuteCommand,
            6 => CommandStatus::CommandTxDone,
            1 => {
                // todo: This should be removed in favof of unexpected status.
                println!("1 returned for command status. Investigate (\"RFU\" in DS; the future is now.)");
                CommandStatus::FailureToExecuteCommand // bogus
            }
            _ => return Err(RadioError::UnexpectedStatus(status)),
        };

        Ok((operating_mode, command_status))
    }

    pub fn get_device_errors(&mut self) -> Result<u16, RadioError> {
        let mut buf = [OpCode::GetDeviceErrors as u8, 0, 0, 0];
        self.interface.read(&mut buf)?;

        // Status avail at byte 2.
        Ok(u16::from_be_bytes([buf[2], buf[3]]))
    }
}

/// Convert a f32 time in ms to 3 24-but unsigned integer bytes, used with the radio's system.
/// This is defined a few times in teh datasheet, including section 13.1.4.
fn time_bytes(time_ms: f32) -> [u8; 3] {
    let result = ((time_ms / TIMING_FACTOR_MS) as u32).to_be_bytes();
    [result[1], result[2], result[3]]
}
