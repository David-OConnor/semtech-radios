//! Code to use the SX126x and SX128X LoRa radios.

#![no_std]

mod eratta;
pub mod params;
pub mod shared;
pub mod spi_interface;
pub mod sx126x;
pub mod sx128x;

use defmt::println;
use hal::dma::DmaChannel;

use crate::{
    params::{
        LoraBandwidthSX126x, ModulationParamsLora6x, ModulationParamsLora8x, PacketParamsLora,
    },
    shared::{OpCode, RadioError, RadioError::UnexpectedStatus, RadioPins, Register, Register6x},
    spi_interface::{Interface, Spi_, RADIO_BUF_SIZE},
};

/// The timing factor used to convert between 24-bit integer timing conversions used
/// by the radio, and ms. Eg: Sleep Duration = sleepPeriod * 15.625 µs. Same for rx mode duration.
/// DS, section 13.1.7 (6x)
///
/// Note: On 8x, we can choose from four of these. We use the same one as 6x, always, for now.
const TIMING_FACTOR_MS_6X: f32 = 0.015_625;

const F_XTAL_6X: u64 = 32_000_000; // Oscillator frequency in Mhz.
const F_XTAL_8X: u64 = 52_000_000; // Oscillator frequency in Mhz.
                                   // The radio's maximum data buffer size.

// These constants are pre-computed
const FREQ_CONST_6X: u64 = F_XTAL_6X / (1 << 25);
const FREQ_CONST_8X: u64 = F_XTAL_8X / (1 << 18);

/// 6x DS, 13.4.2. Table 13-38.  The switch from one frame to another must be done in STDBY_RC mode.
/// 8x: Table 11-42.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[allow(dead_code)]
pub enum PacketType {
    /// (G)Fsk
    Gfsk = 0,
    Lora = 1,
    /// 8x only.
    Ranging = 2,
    /// Long Range FHSS (FLRC on 8x)
    LrFhss = 3,
    /// 8x only.
    Ble = 4,
}

#[repr(u16)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
/// (SX126x only(?) DS, table 12-1. Differentiate the LoRa signal for Public or Private network.
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
pub enum RampTime6x {
    R10 = 0,
    R20 = 1,
    R40 = 2,
    R80 = 3,
    R200 = 4,
    R800 = 5,
    R1700 = 6,
    R3400 = 7,
}

/// DS, Table 11-49. Power ramp time. Titles correspond to ramp time in µs.
/// todo: Figure out guidelines for setting this. The DS doesn't have much on it.
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum RampTime8x {
    R02 = 0x0,
    R04 = 0x20,
    R06 = 0x40,
    R08 = 0x60,
    R10 = 0x80,
    R12 = 0xa0,
    R16 = 0xc0,
    R20 = 0xe0,
}

/// 6x: DS, Table 13-29. repr as u16, since we use the values to bit-mask a 16-bit integer.
/// 8x: DS, Table 11-73. repr as u16, since we use the values to bit-mask a 16-bit integer.
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
    //8x only below.
    SyncWordError,
    // todo, if you impl ranging.
    //     RangingSlaveResponseDone = 7,
    //     RangingSlaveRequestDiscard = 8,
    //     RangingMasterResultValid = 9,
    //     RangingMasterTimeut = 10,
    //     RangingSlaveRequestValid = 11,
}

impl Irq {
    pub fn val_8x(&self) -> u16 {
        match self {
            Self::SyncWordValid => 2,
            Self::SyncWordError => 3,
            Self::CadDone => 12,
            Self::CadDetected => 13,
            Self::Timeout => 14,
            Self::PremableDetected => 15, // Also used for Advanced Ranging Done.
            _ => *self as u16,            // When the same as 6x.
        }
    }
}

/// Table 13-21
/// These don't take into account the external PA, if applicable.
#[repr(u8)] // For storing in configs.
#[derive(Clone, Copy)]
pub enum OutputPower6x {
    /// 25mW
    Db14,
    /// 50mW
    Db17,
    /// 100mW
    Db20,
    /// 158mW
    Db22,
}

impl Default for OutputPower6x {
    fn default() -> Self {
        Self::Db22 // full power
    }
}

impl OutputPower6x {
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

/// 6x only. DS, 13.1.15. This defines the mode the radio goes into after a successful Tx or Rx.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum FallbackMode {
    Fs = 0x40,
    StdbyXosc = 0x30,
    StdbyRc = 0x20,
}

// todo: 6x only? Can't tell
/// 6x: DS, section 9.6: Receive (RX) Mode
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum RxMode {
    Continuous,
    Single,
    SingleWithTimeout,
    Listen,
}

/// 6x: DS, section 13.1.1. Table 13-2. For bit 2.
/// 8x: DS, section 11.6.1. Table 11-17. For bit 0.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum SleepConfig {
    /// "Ram flushed" on 8x.
    ColdStart = 0,
    /// "Ram retained" on 8x.
    WarmStart = 1,
}

/// 6x DS, section 9. (And table 13-76) 8x: Table 11-5. (Called Circuit mode)
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum OperatingMode {
    /// In this mode, most of the radio internal blocks are powered down or in low power mode and optionally the RC64k clock
    /// and the timer are running.
    Sleep(SleepConfig),
    /// In standby mode the host should configure the chip before going to RX or TX modes. By default in this state, the system is
    /// clocked by the 13 MHz RC oscillator to reduce power consumption (in all other modes except SLEEP the XTAL is turned ON).
    /// However, if the application is time-critical, the XOSC block can be turned or left ON.
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

/// (6x): DS, Table 13-76. (6x): Table 11-5
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

/// (6x): DS, section 13.5.2. (8x): Table 11-61
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
pub struct RxPacketStatusLora6x {
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

/// 6x only: DS, section 13.5.4. Table 13-82
pub struct RxStatistics6x {
    pub status: u8,
    pub num_received: u16,
    pub num_crc_error: u16,
    pub num_length_error: u16,
}

/// (6x): DS, section 13.5.1. 8x: Table 11-5
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, defmt::Format, Debug)]
pub enum CommandStatus {
    /// Transceiver has successfully processed the command.
    /// 8x only.
    CommandProcessSuccess8x,
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
pub struct RadioConfig6x {
    pub packet_type: PacketType,
    /// RF frequency in Hz.
    pub rf_freq: u32,
    pub use_dio2_as_rfswitch: bool,
    pub dc_dc_enabled: bool,
    pub modulation_params: ModulationParamsLora6x,
    pub packet_params: PacketParamsLora,
    // todo: FSK A/R.
    // todo: FHSS? May be appropriate for your use case. See DS, section 6.3.
    /// Raw register value. Timeout duration = timeout * 15.625µs.
    pub tx_timeout: f32,
    pub rx_timeout: f32,
    pub fallback_mode: FallbackMode,
    pub ramp_time: RampTime6x,
    pub lora_network: LoraNetwork,
    pub output_power: OutputPower6x,
}

impl Default for RadioConfig6x {
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
            ramp_time: RampTime6x::R200, // todo: What should this be?
            lora_network: LoraNetwork::Private,
            output_power: OutputPower6x::Db22,
        }
    }
}

/// See [this interactive tool for info on how these parameters effect OTA time etc](https://www.semtech.com/design-support/lora-calculator)
/// It also includes consumption, RFIO schematics etc. Use `Shared RFIO`, vice a switch; breaks the calculator.
#[derive(Clone)]
pub struct RadioConfig8x {
    pub packet_type: PacketType,
    // RF frequency in Hz.
    pub rf_freq: u64,
    pub dc_dc_enabled: bool,
    pub modulation_params: ModulationParamsLora8x,
    pub packet_params: PacketParamsLora,
    // todo: FSK A/R.
    // todo: FHSS? May be appropriate for your use case. See DS, section 6.3.
    /// Raw register value. Timeout duration = timeout * 15.625µs.
    pub tx_timeout: f32,
    pub rx_timeout: f32,
    pub fallback_mode: FallbackMode,
    pub ramp_time: RampTime8x,
    // pub lora_network: LoraNetwork,
}

impl Default for RadioConfig8x {
    fn default() -> Self {
        Self {
            packet_type: PacketType::Lora,
            rf_freq: 4_200_000_000,
            dc_dc_enabled: true,
            modulation_params: Default::default(),
            packet_params: Default::default(),
            tx_timeout: 0., // todo: Calculate this based on packet and mod params?
            rx_timeout: 0.,
            fallback_mode: FallbackMode::StdbyRc,
            ramp_time: RampTime8x::R10, // todo: What should this be?
                                        // lora_network: LoraNetwork::Private,
        }
    }
}

pub enum RadioConfig {
    R6x(RadioConfig6x),
    R8x(RadioConfig8x),
}

pub struct Radio {
    pub interface: Interface,
    // todo: Wrapped enum?
    // pub config: RadioConfig6x,
    // pub config_8x: RadioConfig8x, // todo
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
        let r8x = if let RadioConfig::R8x(_) = config {
            true
        } else {
            false
        };

        let mut result = Self {
            config,
            // config_8x,
            interface: Interface {
                spi,
                pins,
                tx_ch,
                rx_ch,
                read_buf: [0; RADIO_BUF_SIZE],
                write_buf: [0; RADIO_BUF_SIZE],
                rx_payload_len: 0,
                rx_payload_start: 0,
                r8x,
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

        match result.config {
            RadioConfig::R6x(ref config) => {
                result
                    .interface
                    .write_op_word(OpCode::SetPacketType, config.packet_type as u8)?;
            }
            RadioConfig::R8x(ref config) => {
                result
                    .interface
                    .write_op_word(OpCode::SetPacketType, config.packet_type as u8)?;
            }
        }

        // Note: In the Tx/Rx recipes in the DS, this is before setting mod parameters; but it's not listed
        // this way in the part on section 9.1.
        result.set_rf_freq()?;

        // "In a second step, the user should define the modulation
        // parameter according to the chosen protocol with the command SetModulationParams(...)."
        result.set_mod_params()?;

        // Finally, the user should then
        // select the packet format with the command SetPacketParams(...).
        result.set_packet_params()?;

        if let RadioConfig::R6x(_) = result.config {
            result.set_rxgain_retention()?;
            result.tx_clamp_workaround()?;
        }

        match result.config {
            RadioConfig::R6x(ref config) => {
                // prevents borrow mut error
                let (dc_dc, fallback, dio, network) = (
                    config.dc_dc_enabled,
                    config.fallback_mode,
                    config.use_dio2_as_rfswitch,
                    config.lora_network,
                );

                // Use the LDO, or DC-DC setup as required, based on hardware config.
                result
                    .interface
                    .write_op_word(OpCode::SetRegulatorMode, dc_dc as u8)?;

                result.set_pa_config()?;

                result
                    .interface
                    .write_op_word(OpCode::SetTxFallbackMode, fallback as u8)?;

                result
                    .interface
                    .write_op_word(OpCode::SetDIO2AsRfSwitchCtrl, dio as u8)?;

                result.set_tx_params()?;

                // Note: Not required if private due to the reset value.
                result.set_sync_word(network)?;
            }
            RadioConfig::R8x(ref config) => {
                // prevents borrow mut error
                let (dc_dc, fallback) = (config.dc_dc_enabled, config.fallback_mode);

                // Use the LDO, or DC-DC setup as required, based on hardware config.
                result
                    .interface
                    .write_op_word(OpCode::SetRegulatorMode, dc_dc as u8)?;

                result.set_pa_config()?;

                result
                    .interface
                    .write_op_word(OpCode::SetTxFallbackMode, fallback as u8)?;

                result.set_tx_params()?;
            }
        }

        let tx_addr = 0;
        let rx_addr = 0;
        result
            .interface
            .write(&[OpCode::SetBufferBaseAddress as u8, tx_addr, rx_addr])?;

        Ok(result)
    }

    /// 6x: See DS, section 13.4.1 for this computation.
    /// 8xx: See DS, section 11.7.3.
    fn set_rf_freq(&mut self) -> Result<(), RadioError> {
        match &self.config {
            RadioConfig::R6x(config) => {
                // We convert to u64 to prevent an overflow.
                let rf_freq_raw =
                    ((config.rf_freq as f32 / FREQ_CONST_6X as f32) as u32).to_be_bytes();

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
                let rf_freq_raw =
                    ((config.rf_freq as f32 / FREQ_CONST_8X as f32) as u32).to_be_bytes();
                // todo: Of note, the DS example for this seems wrong... Would like to close the loop on that.

                self.interface.write(&[
                    OpCode::SetRfFrequency as u8,
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
        let mut p1 = 0;
        let mut p2 = 0;
        let mut p3 = 0;
        let mut p4 = 0;
        let p5 = 0;
        let p6 = 0;
        let p7 = 0;
        let p8 = 0;

        match &self.config {
            RadioConfig::R6x(config) => {
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
                    PacketType::LrFhss => {
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
                match config.packet_type {
                    PacketType::Gfsk => {
                        unimplemented!()
                    }
                    PacketType::Lora => {
                        p1 = config.modulation_params.spreading_factor as u8;
                        p2 = config.modulation_params.mod_bandwidth as u8;
                        p3 = config.modulation_params.coding_rate as u8;
                    }
                    PacketType::LrFhss => {
                        // todo: Implement for fhss
                        unimplemented!()
                    }
                    _ => unimplemented!(),
                }

                // todo: Confirm we can ignore unused params.

                self.interface
                    .write(&[OpCode::SetModulationParams.val_8x(), p1, p2, p3])
            }
        }
    }

    /// Send packet parameters found in the config, to the radio.
    /// 6x: DS, section 13.4.6.
    /// 8x: DS, section 11.7.8
    fn set_packet_params(&mut self) -> Result<(), RadioError> {
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
                    PacketType::LrFhss => {
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
                let p8 = 0;
                let p9 = 0;

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
                    PacketType::LrFhss => {
                        // todo: Implement this for sx1280: FHSS.
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
    fn set_pa_config(&mut self) -> Result<(), RadioError> {
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
    fn set_tx_params(&mut self) -> Result<(), RadioError> {
        let (power, ramp_time) = match &self.config {
            RadioConfig::R6x(config) => {
                (0x16, config.ramp_time as u8) // Max power.
            }
            RadioConfig::R8x(config) => {
                (0x31, config.ramp_time as u8) // Max power.
            }
        };

        self.interface
            .write(&[OpCode::SetTxParams as u8, power, ramp_time])
    }

    /// Sets the device into sleep mode; the lowest current consumption possible. Wake up by setting CS low.
    pub fn set_op_mode(&mut self, mode: OperatingMode) -> Result<(), RadioError> {
        match mode {
            // todo: This behavior is 6x only. 8x is different (?)
            OperatingMode::Sleep(cfg) => self
                .interface
                // todo: Wake-up on RTC A/R.
                .write_op_word(OpCode::SetSleep, (cfg as u8) << 2),

            OperatingMode::StbyRc => self.interface.write_op_word(OpCode::SetStandby, 0),
            OperatingMode::StbyOsc => self.interface.write_op_word(OpCode::SetStandby, 1),
            OperatingMode::Fs => self.interface.write(&[OpCode::SetFS as u8]),
            OperatingMode::Tx(timeout) => {
                let to_bytes = time_bytes_6x(timeout);
                self.interface
                    .write(&[OpCode::SetTx as u8, to_bytes[0], to_bytes[1], to_bytes[2]])
            }
            OperatingMode::Rx(timeout) => {
                let to_bytes = match self.config {
                    RadioConfig::R6x(_) => time_bytes_6x(timeout),
                    RadioConfig::R8x(_) => time_bytes_8x(timeout),
                };
                self.interface
                    .write(&[OpCode::SetRx as u8, to_bytes[0], to_bytes[1], to_bytes[2]])
            }
        }
    }

    /// (6x) DS, section 14.2. Frequency is set here and in receive initiation, for use with frequency hopping.
    /// (8x) DS, section 14.4.22.
    pub fn send_payload(&mut self, payload: &[u8], rf_freq: u32) -> Result<(), RadioError> {
        // todo: Confirm the same for 8x.
        let payload_len = payload.len();
        if payload_len > 255 {
            return Err(RadioError::PayloadSize(payload_len));
        }

        // Separate to prevent borrow errors.
        match &mut self.config {
            RadioConfig::R6x(ref mut config) => {
                config.rf_freq = rf_freq;
                config.packet_params.payload_len = payload_len as u8;
            }
            RadioConfig::R8x(ref mut config) => {
                config.rf_freq = rf_freq as u64;
                config.packet_params.payload_len = payload_len as u8;
            }
        }

        match &self.config {
            RadioConfig::R6x(config) => {
                // todo: 8x too?
                // 1. If not in STDBY_RC mode, then go to this mode with the command SetStandby(...)
                self.set_op_mode(OperatingMode::StbyRc)?;

                // See DS, section 15.1.2: Work around this eratta before each transmission
                self.mod_quality_workaround()?;

                // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
                // (Set on init)
                // self.interface
                //     .write_op_word(OpCode::SetPacketType, self.config.packet_type as u8)?;

                // 3. Define the RF frequency with the command SetRfFrequency(...)

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
                    let mut write_buf = [0; RADIO_BUF_SIZE];
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
                self.set_packet_params()?;

                // 10. Configure DIO and IRQ: use the command SetDioIrqParams(...) to select TxDone IRQ and map this IRQ to a DIO (DIO1,
                // DIO2 or DIO3)
                // (Currently without DMA, this is handled in `start_transmission`.)
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
            }
            RadioConfig::R8x(config) => {
                // todo: Where do we set the frequency? Adding here for now.
                self.set_rf_freq()?;

                // 1. Define the output power and ramp time by sending the command:
                // (Set on init)

                // 2. Send the payload to the data buffer by sending the command:
                // WriteBuffer(offset,*data)
                // where *data is a pointer to the payload and offset is the address at which the first byte of the payload will be located in the
                // buffer. Offset will correspond to txBaseAddress in normal operation.

                // todo: DRY with 6x.
                let offset = 0;
                // todo: Put back when reading.
                // self.interface
                //     .write_with_payload(payload, offset)?;

                // todo: Having trouble with SPI DMA writes (but reads work) Skipping for now; blocking.
                {
                    let mut write_buf = [0; RADIO_BUF_SIZE];
                    write_buf[0] = OpCode::WriteBuffer as u8;
                    write_buf[1] = offset;
                    for (i, word) in payload.iter().enumerate() {
                        write_buf[i + 2] = *word;
                    }
                    self.interface.write(&write_buf[..2 + payload_len])?;
                }

                // 3. Configure the DIOs and Interrupt sources (IRQs) by sending the command:
                // SetDioIrqParams(irqMask,dio1Mask,dio2Mask,dio3Mask)
                // In a typical Tx operation the user can select one or several IRQ sources:
                // •TxDone IRQ to indicate the end of packet transmission. The transceiver will be in STDBY_RC mode.
                // •RxTxTimeout (optional) to make sure no deadlock can happen. The transceiver will return automatically to STDBY_RC
                // mode if a timeout occurs.

                // (Currently without DMA, this is handled in `start_transmission`.)
                // (See `handle_tx_post_payload_write()`).

                // 4. Once configured, set the transceiver in transmitter mode to start transmission by sending the command:
                // SetTx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
                // If a timeout is desired, set periodBaseCount to a non-zero value. This timeout can be used to avoid deadlock.
                // Wait for IRQ TxDone or RxTxTimeout
                // Once a packet has been sent or a timeout has occurred, the transceiver goes automatically to STDBY_RC mode.

                // todo: Packet params?
                // self.set_packet_params()?;

                self.start_transmission()?; // todo: Handle in ISR once using DMA.
                                            // (Handled in SPI Tx complete ISR)

                // 5.Clear TxDone or RxTxTimeout IRQ by sending the command:
                // ClrIrqStatus(irqStatus)
                // (Handled in firmware GPIO ISR)
            }
        }

        Ok(())
    }

    /// Run these from the SPI Tx complete ISR. This initiates transmission; run this once the
    /// payload write to the radio's buffer is complete.
    pub fn start_transmission(&mut self) -> Result<(), RadioError> {
        // todo: Sort out for 8x.
        self.set_irq(&[Irq::TxDone, Irq::Timeout], &[])?; // DIO 1

        let timeout = match &self.config {
            RadioConfig::R6x(c) => c.tx_timeout,
            RadioConfig::R8x(c) => c.tx_timeout,
        };
        self.set_op_mode(OperatingMode::Tx(timeout))?;

        Ok(())
    }

    /// (6x) Set the radio into receive mode. DS, section 14.3.
    /// (8x) 14.4.3
    /// todo: COnsider also using the SetDutyCycle sniff mode.
    pub fn receive(&mut self, max_payload_len: u8, rf_freq: u32) -> Result<(), RadioError> {
        // Separate to prevent borrow errors.
        match &mut self.config {
            RadioConfig::R6x(ref mut config) => {
                config.rf_freq = rf_freq;
                config.packet_params.payload_len = max_payload_len;
            }
            RadioConfig::R8x(ref mut config) => {
                config.rf_freq = rf_freq as u64;
                config.packet_params.payload_len = max_payload_len;
            }
        }

        match &self.config {
            RadioConfig::R6x(config) => {
                let timeout = config.rx_timeout; // prevents borrow errors.

                // 1. If not in STDBY_RC mode, then set the circuit in this mode with the command SetStandby()
                self.set_op_mode(OperatingMode::StbyRc)?;

                // 2. Define the protocol (LoRa® or FSK) with the command SetPacketType(...)
                // (Set on init)
                // self.interface
                //     .write_op_word(OpCode::SetPacketType, config.packet_type as u8)?;

                // 3.Define the RF frequency with the command SetRfFrequency(...)

                self.set_rf_freq()?;

                // 4. Define where the data will be stored inside the data buffer in Rx with the command SetBufferBaseAddress(...)
                // (Note: We may have to set this here, since I believe this setting auto-increments.)
                let tx_addr = 0;
                let rx_addr = 0;
                self.interface
                    .write(&[OpCode::SetBufferBaseAddress as u8, tx_addr, rx_addr])?;

                // 5. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
                // (Set on init)
                // self.set_mod_params_sx126x()?;

                // 6. Define the frame format to be used with the command SetPacketParams(...)
                // We must set this, as it may have been changed during a transmission to payload length.

                self.set_packet_params()?;

                // 7. Configure DIO and irq: use the command SetDioIrqParams(...) to select the IRQ RxDone and map this IRQ to a DIO (DIO1
                // or DIO2 or DIO3), set IRQ Timeout as well.
                self.set_irq(&[], &[Irq::RxDone, Irq::Timeout])?; // DIO3.

                // 8. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access.
                // (Set on init)

                // 9. Set the circuit in reception mode: use the command SetRx(). Set the parameter to enable timeout or continuous mode
                self.set_op_mode(OperatingMode::Rx(timeout))?;

                // 10. Wait for IRQ RxDone2 or Timeout: the chip will stay in Rx and look for a new packet if the continuous mode is selected
                // otherwise it will goes to STDBY_RC mode.
                // (The rest is handled in `cleanup_rx`, called from a firmware GPIO ISR).
            }
            RadioConfig::R8x(config) => {
                let timeout = config.rx_timeout; // prevents borrow errors.

                // todo: set freq? Here for now.
                self.set_rf_freq()?;

                // 1. Configure the DIOs and Interrupt sources (IRQs) by using command:
                // SetDioIrqParams(irqMask,dio1Mask,dio2Mask,dio3Mask)
                // In a typical LoRa® Rx operation the user could select one or several of the following IRQ sources:
                // •RxDone to indicate a packet has been detected. This IRQ does not mean that the packet is valid (size or CRC correct).
                // The user must check the packet status to ensure that a valid packed has been received.
                // •PreambleValid is available to indicate a vaid loRa preamble has been detected.
                // •HeaderValid (and HeaderError) are available to indicate whether a valid packet header has been received.
                // •SyncWordValid to indicate that a Sync Word has been detected.
                // •CrcError to indicate that the received packet has a CRC error
                // •RxTxTimeout to indicate that no packet has been detected in a given time frame defined by timeout parameter in the
                // SetRx() command.

                self.set_irq(&[], &[Irq::RxDone, Irq::Timeout])?; // DIO3.

                // 2.Once configured, set the transceiver in receiver mode to start reception using command:
                // SetRx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
                // Depending on periodBaseCount, 3 possible Rx behaviors are possible:
                // •periodBaseCount is set 0, then no Timeout, Rx Single mode, the device will stay in Rx mode until a reception occurs and
                // the device returns to STDBY_RC mode upon completion.
                // •periodBaseCount is set 0xFFFF, Rx Continuous mode, the device remains in Rx mode until the host sends a command to
                // change the operation mode. The device can receive several packets. Each time a packet is received, a packet received
                // indication is given to the host and the device will continue to search for a new packet.
                // •periodBaseCount is set to another value, then Timeout is active. The device remains in Rx mode; it returns automatically
                // to STDBY_RC Mode on timer end-of-count or when a packet has been received. As soon as a packet is detected, the
                // timer is automatically disabled to allow complete reception of the packet.
                self.set_op_mode(OperatingMode::Rx(timeout))?;

                // todo: What about things in the 6x section above like setting packet params?

                // 3. In typical cases, use a timeout and wait for IRQ RxDone or RxTxTimeout.
                // If IRQ RxDone is asserted, the transceiver goes to STDBY_RC mode if single mode is used (timeout set to a value different
                // from 0xFFFF). If Continuous mode is used (timeout set to 0xFFFF) the transceiver stays in Rx and continues to listen for a
                // new packet.
                // 4. Check the packet status to make sure that the packet has been received properly, by sending the command:
                // GetPacketStatus()
                // The command returns the following parameters:
                // • SnrPkt Estimation of SNR on last packet received. In two’s compliment format multiplied by 4.
                // Actual SNR in dB =SnrPkt/4 , noting that only negative values should be used.

                // 5. Once all checks are complete, clear IRQs by sending the command:
                // ClrIrqStatus(irqMask)
                // This command will reset the flag for which the corresponding bit position in irqMask is set to 1.
                // Note:
                // A DIO can be mapped to several IRQ sources (ORed with IRQ sources). The DIO will go to zero once IRQ flag has been
                // set to zero.
                // SX1280/SX1281
                // Data Sheet
                // Rev 3.2
                // DS.SX1280-1.W.APP
                // March 2020
                // 135 of 158
                // Semtech
                // www.semtech.com

                // 6.
                // Get the packet length and start address of the received payload by sending the command:
                // GetRxBufferStatus()
                // This command returns the length of the last received packet (payloadLengthRx) and the address of the first byte received
                // (rxStartBufferPointer). It is applicable to all modems. The address is an offset relative to the first byte of the data buffer.

                // 7. Read the data buffer using the command:
                // ReadBuffer(offset, payloadLengthRx)
                // Where offset is equal to rxStartBufferPointer and payloadLengthRx is the size of buffer to read.

                // 8. Optionally, the frequency error indicator (FEI) can be read from register 0x0954 (MSB) 0x0955, 0x0956 (LSB). The FEI is
                // 2's complement (signed) 20 bit number: SignedFEIReading. This must be converted from two’s compliment to a signed
                // FEI reading then, in turn, can be converted to a frequency error in Hz using the following formula:
                // The resolution of the frequency error indicator measurement in LoRa mode is given by:
                // FEI_RES [Hz] = 1 / (64 * Ts)
                // Where Ts is the LoRa symbol time. The table below shows all possible FEI resolution combinations versus SF and bandwidth.
                // (All values in milliseconds).
                // (The rest is handled in `cleanup_rx`, called from a firmware GPIO ISR).
            }
        }

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

        // Device errors (at least as a standalone Opcode) is not present on 8x.
        if let RadioConfig::R6x(_) = self.config {
            let device_errors = self.get_device_errors()?;
            if device_errors != 0 {
                println!("\nDevice error: {}", device_errors);
                return Err(RadioError::Device);
            }
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

        if let RadioConfig::R6x(_) = self.config {
            self.implicit_header_to_workaround()?; // See eratta, section 15.3.

            let device_errors = self.get_device_errors()?;
            if device_errors != 0 {
                println!("\nDevice error: {}", device_errors);
                return Err(RadioError::Device);
            }
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

    /// 6x DS, section 13.5.2. This loads information related to the received payload; it may be useful
    /// in decoding the buffer.
    /// 8x, section 11.8.1 (Same as 6x, other than opcode addr)
    pub fn get_rx_buffer_status(&mut self) -> Result<RxBufferStatus, RadioError> {
        let mut buf = [OpCode::GetRxBufferStatus as u8, 0, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(RxBufferStatus {
            status: buf[1],
            payload_len: buf[2],
            rx_start_buf_pointer: buf[3],
        })
    }

    /// 6x DS, section 13.5.3. This contains useful link stats from a received message. (LoRa)
    /// 8x: DS, section 11.8.2.Differen, including more fields, eg for BLE, FLRC etc. LoRa uses
    /// status, rssiSync and snr only. I think we can use the same code for both.
    pub fn get_packet_status(&mut self) -> Result<RxPacketStatusLora6x, RadioError> {
        let mut buf = [OpCode::GetPacketStatus as u8, 0, 0, 0, 0];
        self.interface.read(&mut buf)?;

        // Raw data, to make passing over the wire/air more compact. Convert using scaler/signed types
        // prior to display or use.
        Ok(RxPacketStatusLora6x {
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

    /// 6x only. DS, section 13.5.5
    /// todo: Impl reset as well.
    pub fn get_statistics(&mut self) -> Result<RxStatistics6x, RadioError> {
        let mut buf = [OpCode::GetStats as u8, 0, 0, 0, 0, 0, 0, 0];
        self.interface.read(&mut buf)?;

        Ok(RxStatistics6x {
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
    /// todo: 8x?
    fn set_sync_word(&mut self, network: LoraNetwork) -> Result<(), RadioError> {
        let sync_word_bytes = (network as u16).to_be_bytes();

        // todo: 6x for now.
        self.interface.write_reg_word(
            Register::Reg6x(Register6x::LoraSyncWordMsb),
            sync_word_bytes[0],
        )?;
        self.interface.write_reg_word(
            Register::Reg6x(Register6x::LoraSyncWordLsb),
            sync_word_bytes[1],
        )?;

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

    /// 6x: 13.5.1
    /// 8x: 11.3. Note: (Is this applicable more broadly?) on 8x, it appears the return is shifted left by one vice 6x.
    /// todo: QC on 86...
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

    /// 6x only.
    pub fn get_device_errors(&mut self) -> Result<u16, RadioError> {
        let mut buf = [OpCode::GetDeviceErrors as u8, 0, 0, 0];
        self.interface.read(&mut buf)?;

        // Status avail at byte 2.
        Ok(u16::from_be_bytes([buf[2], buf[3]]))
    }
}

// todo: Both radios: Should we use SetRxDutyCycle? Come back to later. (A sniff mode.)

/// Convert a f32 time in ms to 3 24-but unsigned integer bytes, used with the radio's system. Used for
/// sleep, and Rx duration.
/// This is defined a few times in the datasheet, including section 13.1.4.
fn time_bytes_6x(time_ms: f32) -> [u8; 3] {
    // Sleep Duration = sleepPeriod * 15.625 µs
    let result = ((time_ms / TIMING_FACTOR_MS_6X) as u32).to_be_bytes();
    [result[1], result[2], result[3]]
}

/// Convert a f32 time in ms to 3 24-but unsigned integer bytes, used with the radio's system.
///
/// Note: This is more flexible than 6x's, but we, for now, hard set the period base to be the same
/// as 6x.
/// See DS Table 11-24, and section 11.6.5.
fn time_bytes_8x(time_ms: f32) -> [u8; 3] {
    // Sleep Duration = PeriodBase * sleepPeriodBaseCount. (PeriodBase is what we set in the register)
    // todo: QC order, etc
    let period_base = ((time_ms / TIMING_FACTOR_MS_6X) as u32).to_be_bytes();
    // 0 in the first position defines th ebase period to be the 15.625 us value hard-coded for 6x.
    [0x00, period_base[2], period_base[3]]
}
