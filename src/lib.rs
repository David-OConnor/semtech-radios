//! Code to use the SX126x and SX128X LoRa radios.

#![no_std]

mod configure;
mod eratta;
pub mod params;
pub mod shared;
pub mod spi_interface;
mod status;

use defmt::println;
use hal::dma::DmaChannel;

// todo: Calibration on 8x?
use crate::{
    params::{ModulationParams8x, ModulationParamsLora6x, PacketParams, PacketParamsLora},
    shared::{OpCode, RadioError, RadioPins, Register, Register::Reg8x, Register6x, Register8x},
    spi_interface::{Interface, Spi_, RADIO_BUF_SIZE},
};

/// The timing factor used to convert between 24-bit integer timing conversions used
/// by the radio, and ms. Eg: Sleep Duration = sleepPeriod * 15.625 µs. Same for rx mode duration.
/// DS, section 13.1.7 (6x)
///
/// Note: On 8x, we can choose from four of these. We use the same one as 6x, always, for now.
const TIMING_FACTOR_MS_6X: f32 = 0.015_625;

// Oscillator frequency in Mhz.
const F_XTAL_6X: f32 = 32_000_000.;
const F_XTAL_8X: f32 = 52_000_000.;

// These constants are pre-computed
const FREQ_CONST_6X: f32 = F_XTAL_6X / (1 << 25) as f32;
const FREQ_CONST_8X: f32 = F_XTAL_8X / (1 << 18) as f32;

// Error in the datasheet?
const FIRMWARE_VERSION_8X_A: u16 = 0xA9B5;
const FIRMWARE_VERSION_8X_B: u16 = 0xA9B7;

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
    LrFhssFlrc = 3,
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
#[derive(Clone, Copy, defmt::Format)]
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
/// Note that the values are listed for sx1262. They are hard-coded for high power PA selection.
#[repr(u8)] // For storing in configs.
#[derive(Clone, Copy)]
pub enum OutputPower6x {
    /// 25mW
    Db14 = 0x0e,
    /// 50mW
    Db17 = 0x11,
    /// 100mW
    Db20 = 0x14,
    /// 158mW
    Db22 = 0x16,
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
#[derive(defmt::Format)]
pub struct RxBufferStatus {
    pub status: u8,
    pub payload_len: u8,
    pub rx_start_buf_pointer: u8,
}

/// DS, section 13.5.3. Contains link statistics from the previously packet.
/// Note that this contains the raw data, for easy sending on the wire. Convert to appropriate
/// units at the consumer side.
/// todo: FSK A/R; same field count, but with different meanings.
/// todo: For now, coopting for 8x as well.
#[derive(defmt::Format, Default)]
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
    /// Timeouts, in ms.
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
    // todo: Integrate mod and packet params into this enum?
    pub packet_type: PacketType,
    // RF frequency in Hz.
    pub rf_freq: u32,
    pub dc_dc_enabled: bool,
    pub modulation_params: ModulationParams8x,
    pub packet_params: PacketParams,
    /// Timeouts, in ms.
    pub tx_timeout: f32,
    pub rx_timeout: f32,
    pub ramp_time: RampTime8x,
    /// In dBm. Ranges from -18 to +13. Defaults to max power.
    pub output_power: i8, // pub lora_network: LoraNetwork,
}

impl Default for RadioConfig8x {
    fn default() -> Self {
        Self {
            packet_type: PacketType::Lora,
            rf_freq: 2_400_000_000,
            dc_dc_enabled: true,
            modulation_params: Default::default(),
            packet_params: Default::default(),
            tx_timeout: 0., // todo: Calculate this based on packet and mod params?
            rx_timeout: 0.,
            ramp_time: RampTime8x::R10, // todo: What should this be?
            output_power: 13,
        }
    }
}

#[derive(Clone)]
pub enum RadioConfig {
    R6x(RadioConfig6x),
    R8x(RadioConfig8x),
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
        let tx_addr = 0;
        let rx_addr = 0;

        let r8x = matches!(config, RadioConfig::R8x(_));

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
                r8x,
            },
        };

        if !r8x {
            // Removed, due to using multiple radios on Meerkat.
            result.interface.reset();
        }

        // We use this firmware version as a sanity check.
        if r8x {
            let firmware_version = result
                .interface
                .read_reg_word_16(Reg8x(Register8x::FirmwareVersions))?;
            if ![FIRMWARE_VERSION_8X_A, FIRMWARE_VERSION_8X_B].contains(&firmware_version) {
                return Err(RadioError::FirmwareVersion);
            }
        }

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

        let packet_type = match result.config {
            RadioConfig::R6x(ref config) => config.packet_type,
            RadioConfig::R8x(ref config) => config.packet_type,
        };

        result
            .interface
            .write_op_word(OpCode::SetPacketType, packet_type as u8)?;

        // Note: In the Tx/Rx recipes in the DS, this is before setting mod parameters; but it's not listed
        // this way in the part on section 9.1.

        // todo: This is breaking things on 8x, but we set it when receiving and transmitting.
        // result.set_rf_freq()?;

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

        result.set_tx_params()?;

        result
            .interface
            .write(&[OpCode::SetBufferBaseAddress as u8, tx_addr, rx_addr])?;

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
                    .write_op_word(OpCode::SetRxTxFallbackMode, fallback as u8)?;

                result
                    .interface
                    .write_op_word(OpCode::SetDIO2AsRfSwitchCtrl, dio as u8)?;

                // Note: Not required if private due to the reset value.
                result.set_sync_word(network)?;
            }
            // See DS, section 14.4: LoRa Operation, and similar.
            RadioConfig::R8x(ref config) => {
                // todo: This is breakigngs things...
                // result
                //     .interface
                //     .write_op_word(OpCode::SetRegulatorMode, config.dc_dc_enabled as u8)?;

                // todo: A/R. There's a subltety to it (See note below table 14-54)
                // result.set_sync_word(network)?;
            }
        }

        Ok(result)
    }

    /// (6x) DS, section 14.2. Frequency is set here and in receive initiation, for use with frequency hopping.
    /// (8x) DS, section 14.4.22.
    pub fn send_payload(&mut self, payload: &[u8], rf_freq: u32) -> Result<(), RadioError> {
        let payload_len = payload.len();

        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::WriteBuffer as u8,
            RadioConfig::R8x(_) => OpCode::WriteBuffer.val_8x(),
        };

        let offset = 0;

        self.interface.write_buf[0] = op_code;
        self.interface.write_buf[1] = offset;
        for (i, word) in payload.iter().enumerate() {
            self.interface.write_buf[i + 2] = *word;
        }

        // todo: This duplicated buffer prevents borrowed errors. Remove when we change to DMA.
        let mut write_buf = [0; RADIO_BUF_SIZE];
        write_buf[..payload_len + 2].copy_from_slice(&self.interface.write_buf[..payload_len + 2]);

        if payload_len > RADIO_BUF_SIZE {
            return Err(RadioError::PayloadSize(payload_len));
        }

        // Separate to prevent borrow errors.
        match &mut self.config {
            RadioConfig::R6x(ref mut config) => {
                config.rf_freq = rf_freq;
                config.packet_params.payload_len = payload_len as u8;
            }
            RadioConfig::R8x(ref mut config) => {
                config.rf_freq = rf_freq;

                match &mut config.packet_params {
                    PacketParams::Lora(p) => p.payload_len = payload_len as u8,
                    PacketParams::Flrc(p) => p.payload_len = payload_len as u8,
                }
            }
        }

        match &self.config {
            RadioConfig::R6x(_config) => {
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
                    // let mut write_buf = [0; RADIO_BUF_SIZE];
                    // write_buf[0] = OpCode::WriteBuffer as u8;
                    // write_buf[1] = offset;
                    // for (i, word) in payload.iter().enumerate() {
                    //     write_buf[i + 2] = *word;
                    // }
                    self.interface.write(&write_buf[..2 + payload_len])?;
                }

                // 8. Define the modulation parameter according to the chosen protocol with the command SetModulationParams(...)1
                // (set on init)
                // self.set_mod_params()?;

                // 9. Define the frame format to be used with the command SetPacketParams(...)
                // (set on init)
                // self.set_packet_params()?;

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
            RadioConfig::R8x(_config) => {
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
                    // let mut write_buf = [0; RADIO_BUF_SIZE];
                    // write_buf[0] = OpCode::WriteBuffer as u8;
                    // write_buf[1] = offset;
                    // for (i, word) in payload.iter().enumerate() {
                    //     write_buf[i + 2] = *word;
                    // }
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
        // Config access is separate to prevent borrow errors.
        match &mut self.config {
            RadioConfig::R6x(ref mut config) => {
                config.rf_freq = rf_freq;
                config.packet_params.payload_len = max_payload_len;
            }
            RadioConfig::R8x(ref mut config) => {
                config.rf_freq = rf_freq;

                match &mut config.packet_params {
                    PacketParams::Lora(p) => p.payload_len = max_payload_len,
                    PacketParams::Flrc(p) => p.payload_len = max_payload_len,
                }
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
                // Set on init
                // self.set_packet_params()?;

                // 7. Configure DIO and irq: use the command SetDioIrqParams(...) to select the IRQ RxDone and map this IRQ to a DIO (DIO1
                // or DIO2 or DIO3), set IRQ Timeout as well.
                self.set_irq(&[], &[Irq::RxDone, Irq::Timeout])?; // DIO3.

                // 8. Define Sync Word value: use the command WriteReg(...) to write the value of the register via direct register access.
                // (Set on init)

                // todo: Allow continous mode (Both 6x and 8x): Setting timeout to 0xffffff allows
                // todo receiving multiple packets. (0xffff on 8x)

                // 9. Set the circuit in reception mode: use the command SetRx(). Set the parameter to enable timeout or continuous mode
                self.set_op_mode(OperatingMode::Rx(timeout))?;

                // 10. Wait for IRQ RxDone2 or Timeout: the chip will stay in Rx and look for a new packet if the continuous mode is selected
                // otherwise it will goes to STDBY_RC mode.
                // (The rest is handled in `cleanup_rx`, called from a firmware GPIO ISR).
            }
            RadioConfig::R8x(config) => {
                let timeout = config.rx_timeout; // prevents borrow errors.

                self.set_op_mode(OperatingMode::StbyRc)?;

                let tx_addr = 0;
                let rx_addr = 0;
                self.interface
                    .write(&[OpCode::SetBufferBaseAddress.val_8x(), tx_addr, rx_addr])?;

                // self.set_rf_freq()?;

                // 1. Configure the DIOs and Interrupt sources (IRQs) by using command:
                // SetDioIrqParams(irqMask,dio1Mask,dio2Mask,dio3Mask)

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
            // This executes if we didn't receive a message.
            self.clear_irq(&[Irq::RxDone, Irq::Timeout])?;
            return Err(RadioError::Status((op_mode, cmd_status)));
        }

        // 11. In case of the IRQ RxDone, check the status to ensure CRC is correct: use the command GetIrqStatus()
        // Note:
        // The IRQ RxDone means that a packet has been received but the CRC could be wrong: the user must check the CRC before
        // validating the packet.
        if cmd_status == CommandStatus::DataAvailable {
            let op_code = match self.config {
                RadioConfig::R6x(_) => OpCode::GetIrqStatus as u8,
                RadioConfig::R8x(_) => OpCode::GetIrqStatus.val_8x(),
            };

            let mut irq_status_buf = [op_code, 0, 0, 0];

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
        // println!(
        //     "Buffer status. Status: {} len: {} start buf: {} ",
        //     buf_status.status, buf_status.payload_len, buf_status.rx_start_buf_pointer
        // );

        self.interface.rx_payload_len = buf_status.payload_len;
        self.interface.rx_payload_start = buf_status.rx_start_buf_pointer;

        if let RadioConfig::R6x(_) = self.config {
            self.implicit_header_to_workaround()?; // See eratta, section 15.3.

            // No device errors opcode on 8x.
            let device_errors = self.get_device_errors()?;
            if device_errors != 0 {
                println!("\nDevice error: {}", device_errors);
                return Err(RadioError::Device);
            }
        }

        // 13. In case of a valid packet (CRC OK), start reading the packet
        // Note that in the case of a timeout, we get CommandStatus::Timeout, and don't try to read the data.
        if cmd_status == CommandStatus::DataAvailable {
            // self.interface
            //     // .read_with_payload(buf_status.payload_len, buf_status.rx_start_buf_pointer)?;
            //     .read_with_payload(buf_status.payload_len, 0)?;

            // todo TS. It seems DMA may be at the core of your demons.

            // todo: This duplicate buffer prevents a borrow mut erroro for now. Change once we use DMA.
            let mut buf = [0; RADIO_BUF_SIZE];
            //
            let op_code = match self.config {
                RadioConfig::R6x(_) => OpCode::ReadBuffer as u8,
                RadioConfig::R8x(_) => OpCode::ReadBuffer.val_8x(),
            };

            buf[0] = op_code;
            buf[1] = buf_status.rx_start_buf_pointer;
            buf[2] = 0;

            let payload_len = buf_status.payload_len as usize;
            let buf_len = payload_len + 3;

            if self.interface.read(&mut buf[..buf_len]).is_err() {
                println!("Error reading the buffer");
            }

            // Transfer data from our SPI read buffer to our internal buffer; this includes
            // only the payload.
            self.interface.read_buf[..payload_len].copy_from_slice(&buf[3..buf_len])
        }

        // (Process the payload in the SPI Rx complete ISR)

        Ok((buf_status, cmd_status))
    }

    /// DS, section 13.3.1. Setup DIO1 and DIO3 IRQs, which can be used with the MCU's GPU interrupts.
    /// We assume DIO2 controls the Tx/Rx switch.
    ///
    /// Sx128x DS:
    /// "In a typical LoRa® Rx operation the user could select one or several of the following IRQ sources:
    /// •RxDone to indicate a packet has been detected. This IRQ does not mean that the packet is valid (size or CRC correct).
    /// The user must check the packet status to ensure that a valid packed has been received.
    /// •PreambleValid is available to indicate a vaid loRa preamble has been detected.
    /// •HeaderValid (and HeaderError) are available to indicate whether a valid packet header has been received.
    /// •SyncWordValid to indicate that a Sync Word has been detected.
    /// •CrcError to indicate that the received packet has a CRC error
    /// •RxTxTimeout to indicate that no packet has been detected in a given time frame defined by timeout parameter in the
    /// SetRx() command."
    fn set_irq(&mut self, dio1: &[Irq], dio3: &[Irq]) -> Result<(), RadioError> {
        let mut irq_word: u16 = 0;
        let mut dio1_word: u16 = 0;
        let mut dio3_word: u16 = 0;

        match self.config {
            RadioConfig::R6x(_) => {
                for irq in dio1 {
                    irq_word |= 1 << (*irq as u16);
                    dio1_word |= 1 << (*irq as u16);
                }
                for irq in dio3 {
                    irq_word |= 1 << (*irq as u16);
                    dio3_word |= 1 << (*irq as u16);
                }
            }
            RadioConfig::R8x(_) => {
                for irq in dio1 {
                    irq_word |= 1 << (irq.val_8x());
                    dio1_word |= 1 << (irq.val_8x());
                }
                for irq in dio3 {
                    irq_word |= 1 << (irq.val_8x());
                    dio3_word |= 1 << (irq.val_8x());
                }
            }
        }

        let irq_bytes = irq_word.to_be_bytes();
        let dio1_bytes = dio1_word.to_be_bytes();
        let dio3_bytes = dio3_word.to_be_bytes();

        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::SetDioIrqParams as u8,
            RadioConfig::R8x(_) => OpCode::SetDioIrqParams.val_8x(),
        };

        self.interface.write(&[
            op_code,
            irq_bytes[0],
            irq_bytes[1],
            dio1_bytes[0],
            dio1_bytes[1],
            0, // We don't use DIO2; it's configured as Rx/Tx switch on SX126x.
            0,
            dio3_bytes[0],
            dio3_bytes[1],
        ])
    }

    /// This is a bit confusing, as the register API takes u16, and the sync word is a u16, yet
    /// it's split into two registers.
    /// 8x: See the note below table 14-54.
    fn set_sync_word(&mut self, network: LoraNetwork) -> Result<(), RadioError> {
        let sync_word_bytes = (network as u16).to_be_bytes();

        match self.config {
            RadioConfig::R6x(_) => {
                self.interface.write_reg_word(
                    Register::Reg6x(Register6x::LoraSyncWordMsb),
                    sync_word_bytes[0],
                )?;
                self.interface.write_reg_word(
                    Register::Reg6x(Register6x::LoraSyncWordLsb),
                    sync_word_bytes[1],
                )?;
            }
            RadioConfig::R8x(_) => {
                // todo: This isn't quite right: Must be Read/write/modify, leaving bytes 0:3.
                // self.interface.write_reg_word(
                //     Register::Reg8x(Register8x::LoraSynchWordA),
                //     sync_word_bytes[0],
                // )?;
                // self.interface.write_reg_word(
                //     Register::Reg8x(Register8x::LoraSynchWordB),
                //     sync_word_bytes[1],
                // )?;
            }
        };

        Ok(())
    }

    /// DS, section 13.3.4
    pub fn clear_irq(&mut self, irqs: &[Irq]) -> Result<(), RadioError> {
        // We use a single 16-bit word, with bits at the various values.
        let mut irq_word: u16 = 0;
        for irq in irqs {
            let irq_val = match self.config {
                RadioConfig::R6x(_) => *irq as u16,
                RadioConfig::R8x(_) => irq.val_8x(),
            };
            irq_word |= 1 << irq_val;
        }

        let bytes = irq_word.to_be_bytes();

        let op_code = match self.config {
            RadioConfig::R6x(_) => OpCode::ClearIrqStatus as u8,
            RadioConfig::R8x(_) => OpCode::ClearIrqStatus.val_8x(),
        };
        self.interface.write(&[op_code, bytes[0], bytes[1]])
    }

    pub fn get_irq_status(&mut self) -> Result<u8, RadioError> {
        self.interface.read_op_word(OpCode::GetIrqStatus)
    }

    pub fn set_high_rx_gain(&mut self) -> Result<(), RadioError> {
        if !self.interface.r8x {
            panic!("High gain is unavailable on SX126x.")
        }

        // Update the word using its default value.
        let word = 0x25 | (3 << 6);
        self.interface
            .write_reg_word(Reg8x(Register8x::RxGain), word)
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

    // println!("PERIOD BASE: {:?}", period_base);

    [0x00, period_base[2], period_base[3]]
}
