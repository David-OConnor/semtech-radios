//! Contains modulation and packet params for the SX126x radio.
//!
//! todo: Shared with SX1280, or not?

/// DS, Table 13-44. Mod param 4.
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum GfskPulseShape {
    NoFilter = 0x00,
    GaussianBt0_3 = 0x08,
    GaussianBt0_5 = 0x09,
    GaussianBt0_7 = 0x0A,
    GaussianBt1 = 0x0B,
}

/// DS, Table 13-45. Mod param 5.
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum GfskBandwidth {
    B48 = 0x1f,
    B58 = 0x17,
    B73 = 0x0f,
    B97 = 0x1e,
    // todo: Complete A/R
}

/// (SX126x) DS, Table 13-47. Mod param 1.
/// (SX128x) DS, Table 14-47. Mod param 1.
/// "A higher spreading factor provides better receiver sensitivity at the expense of longer
/// transmission times (time-on-air)."
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum LoraSpreadingFactor {
    SF5 = 0x05,
    SF6 = 0x06,
    SF7 = 0x07,
    SF8 = 0x08,
    SF9 = 0x09,
    SF10 = 0x0A,
    SF11 = 0x0B,
    SF12 = 0x0C,
}

impl LoraSpreadingFactor {
    /// Register value for sx128x. 1126x uses the u8 value for now, for backwards compat.
    /// The same spreading factors are available on both, but with reversed reg hex positions.
    ///
    /// "After SetModulationParams command:
    /// If the Spreading Factor selected is SF5 or SF6, it is required to use WriteRegister( 0x925, 0x1E )
    /// If the Spreading Factor is SF7 or SF-8 then the command WriteRegister( 0x925, 0x37 ) must be used
    /// If the Spreading Factor is SF9, SF10, SF11 or SF12, then the command WriteRegister( 0x925, 0x32 ) must be used
    /// In all cases 0x1 must be written to the Frequency Error Compensation mode register 0x093C"
    pub fn val_8x(&self) -> u8 {
        match self {
            Self::SF5 => 0x50,
            Self::SF6 => 0x60,
            Self::SF7 => 0x70,
            Self::SF8 => 0x80,
            Self::SF9 => 0x90,
            Self::SF10 => 0xA0,
            Self::SF11 => 0xB0,
            Self::SF12 => 0xC0,
        }
    }
}

/// DS, Table 13-47. Mod param 2.
/// "An increase in signal bandwidth permits the use of a higher effective data rate, thus reducing transmission time at the
/// expense of reduced sensitivity improvement."
/// Note that the lower settings here can result in 5s or higher OTA time! OTA seems to scale linearly (inversely)
/// with bandwidth.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[allow(non_camel_case_types, dead_code)]
pub enum LoraBandwidth6x {
    BW_7 = 0x00,
    BW_10 = 0x08,
    BW_15 = 0x01,
    BW_20 = 0x09,
    BW_31 = 0x02,
    BW_41 = 0x0A,
    BW_62 = 0x03,
    BW_125 = 0x04,
    /// May not be available below 400Mhz)
    BW_250 = 0x05,
    /// May not be available below 400Mhz)
    BW_500 = 0x06,
}

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[allow(non_camel_case_types, dead_code)]
/// Table 14-48. Mod param 2.
pub enum LoraBandwidth8x {
    BW_1600 = 0x0a,
    BW_800 = 0x18,
    BW_400 = 0x26,
    BW_200 = 0x34,
}

/// SX126x: DS, Table 13-49. Mod param 3.
/// SX128x: DS, Table 14-49. Mod param 3.
/// "A higher coding rate provides better noise immunity at the expense of longer transmission time. In normal conditions a
/// factor of 4/5 provides the best trade-off; in the presence of strong interfererence a higher coding rate may be used. Error
/// correction code does not have to be known in advance by the receiver since it is encoded in the header part of the packet."
#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
pub enum LoraCodingRate {
    /// raw/total bits: 4/5. Overhead ratio: 1.25
    CR_4_5 = 1,
    /// raw/total bits: 4/6. Overhead ratio: 1.5
    CR_4_6 = 2,
    /// raw/total bits: 4/7. Overhead ratio: 1.75
    CR_4_7 = 3,
    /// raw/total bits: 4/8. Overhead ratio: 2.0
    CR_4_8 = 4,
    /// These CR_LIs are sx128x only:
    /// "* A new interleaving scheme has been implemented to increase robustness to burst interference and/or strong Doppler
    /// events. The FEC has been kept the same to limit the impact on complexity."
    CR_LI_4_5 = 5,
    CR_LI_4_6 = 6,
    CR_LI_4_7 = 8,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
/// (SX126x only) Table 13-50. Mod param 4.
/// "For low data rates (typically for high SF or low BW) and very long payloads which may last several seconds in the air, the low
/// data rate optimization (LDRO) can be enabled. This reduces the number of bits per symbol to the given SF minus two (see
/// Section 6.1.4 "LoRa® Time-on-Air" on page 41) in order to allow the receiver to have a better tracking of the LoRa® signal.
/// Depending on the payload size, the low data rate optimization is usually recommended when a LoRa® symbol time is equal
/// or above 16.38 ms."
pub enum LoraLdrOptimization {
    Disabled = 0,
    Enabled = 1,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Table 14-31.
pub enum FlrcBitrate {
    /// 1.3Mb/s, 1.2Mhz
    BR_1_3 = 0x45,
    /// 1.04Mb/s, 1.2Mhz
    BR_1_0 = 0x69,
    /// 0.65Mb/s, 0.6Mhz
    BR_0_6 = 0x86,
    /// 0.52Mb/s, 0.6Mhz
    BR_0_5 = 0xaa,
    /// .325Mb/s, 0.3Mhz
    BR_0_3 = 0xc7,
    /// 0.26Mb/s, 0.3Mhz
    BR_0_2 = 0xeb,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Table 14-32.
pub enum FlrcCodingRate {
    /// 1/2
    CR_1_2 = 0x00,
    /// 3/4
    CR_3_4 = 0x02,
    /// 1
    CR1_1 = 0x04,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Table 14-33.
pub enum FlrcBt {
    Disabled = 0x00,
    BT_1 = 0x10,
    BT_0_5 = 0x20,
}

#[derive(Clone)]
pub enum ModulationParams8x {
    Lora(ModulationParamsLora8x),
    Flrc(ModulationParamsFlrc),
}

impl Default for ModulationParams8x {
    fn default() -> Self {
        Self::Lora(Default::default())
    }
}

/// (126x) See DS, section 6.1.1: Modulation Parameter.
#[derive(Clone)]
pub struct ModulationParamsLora6x {
    /// Param 1
    pub mod_bandwidth: LoraBandwidth6x,
    /// Param 2
    pub spreading_factor: LoraSpreadingFactor,
    /// Param 3
    pub coding_rate: LoraCodingRate,
    /// Param 4
    pub low_data_rate_optimization: LoraLdrOptimization,
}

impl Default for ModulationParamsLora6x {
    /// We set this up for a short airtime; modify these as default to make the transmission more robust to interference,
    /// and potentially increase range.
    /// https://www.semtech.com/design-support/lora-calculator
    /// With a 64-byte payload, this results in 3-60ms airtime., depending on payload len.
    fn default() -> Self {
        Self {
            mod_bandwidth: LoraBandwidth6x::BW_500,
            spreading_factor: LoraSpreadingFactor::SF5,
            // "In normal conditions a
            // factor of 4/5 provides the best trade-off; in the presence of strong interferers a higher
            // coding rate may be used."
            coding_rate: LoraCodingRate::CR_4_5,
            low_data_rate_optimization: LoraLdrOptimization::Disabled,
        }
    }
}

/// (126x) See DS, section 6.1.1: Modulation Parameter.
#[derive(Clone)]
pub struct ModulationParamsLora8x {
    pub mod_bandwidth: LoraBandwidth8x,
    pub spreading_factor: LoraSpreadingFactor,
    pub coding_rate: LoraCodingRate,
}

impl Default for ModulationParamsLora8x {
    fn default() -> Self {
        Self {
            mod_bandwidth: LoraBandwidth8x::BW_200,
            spreading_factor: LoraSpreadingFactor::SF5,
            coding_rate: LoraCodingRate::CR_4_5, // todo: LI coding rates?
        }
    }
}

/// 8x only: See table 14-31.
#[derive(Clone)]
pub struct ModulationParamsFlrc {
    /// Param 1
    pub bitrate: FlrcBitrate,
    /// Param 2
    pub coding_rate: FlrcCodingRate,
    /// Param 3
    /// todo: Actual name?
    pub bt: FlrcBt,
}

impl Default for ModulationParamsFlrc {
    fn default() -> Self {
        // todo: QC these.
        Self {
            bitrate: FlrcBitrate::BR_1_3,       // The highest
            coding_rate: FlrcCodingRate::CR1_1, // todo: What's a good default? Check DS
            bt: FlrcBt::Disabled,               // todo: Which?
        }
    }
}

/// SX126x DS, Table 13-67. Packet param 3.
/// SX128x DS, Table 14-51. Packet param 2.
/// Also, Section 6.1.3. "The LoRa® modem employs two types of packet formats: explicit and implicit. The explicit
/// packet includes a short header
/// that contains information about the number of bytes, coding rate and whether a CRC is used in the packet."
#[derive(Clone, Copy)]
pub enum LoraHeaderType {
    /// Explict header
    VariableLength,
    /// Implicit header
    FixedLength,
}

impl LoraHeaderType {
    pub fn val_6x(&self) -> u8 {
        match self {
            Self::VariableLength => 0x00,
            Self::FixedLength => 0x01,
        }
    }
    pub fn val_8x(&self) -> u8 {
        match self {
            Self::VariableLength => 0x00,
            Self::FixedLength => 0x80,
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Packet param 1. Table 14-34. 8 Bit is the min for 1Mb/s bit rate. Min 16 otherwise.
pub enum FlrcPreambleLen {
    /// 8 bits, etc
    P8 = 0x10,
    P12 = 0x20,
    P16 = 0x30,
    P20 = 0x40,
    P24 = 0x50,
    P28 = 0x60,
    P32 = 0x70,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Packet param 2 Table 14-35.
/// The number of bytes used for Sync Word is defined by packetParam2. The user can rely on the built-in 21-bit preamble
/// always required to detect start of packet, or add 4 additional Sync Word for address detection in case of multiple
/// devices.
pub enum FlrcSyncWordLen {
    /// 21 bit preamble
    NoSync = 0x00,
    /// 21 bit preamble + 32 bit Sync Word
    P32 = 0x04,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Packet param 3. Table 14-36.
/// "A configurable number of bit-errors can be tolerated in the Sync Word. The desired number of bit errors permissible is
/// written to Synch Address Control register 0x9CD: this is a direct binary mapping with 0 meaning no error is tolerated
/// and 15 meaning up to 15 bit errors will be tolerated.
/// With 3 correlators, the transceiver can search for up to 3 Sync Words at the time. The combination of Sync Word
/// detection is defined by parameters PacketParam3."
pub enum FlrcSyncWordCombo {
    Disabled = 0x00,
    SyncWord1 = 0x10,
    SyncWord2 = 0x20,
    SyncWord1Or2 = 0x30,
    SyncWord3 = 0x40,
    SyncWord1Or3 = 0x50,
    SyncWord2Or3 = 0x60,
    SyncWord1Or2Or3 = 0x70,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Packet param 4. Table 14-37.
pub enum FlrcPacketType {
    LenFixed = 0x00,
    LenVariable = 0x20,
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(non_camel_case_types, dead_code)]
/// (SX128x only). Packet param 6. Table 14-39.
pub enum FlrcCrc {
    CrcOff = 0x00,
    Crc2Byte = 0x10,
    Crc3Byte = 0x20,
    Crc4Byte = 0x30,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CrcEnabled {
    Disabled,
    Enabled,
}

impl CrcEnabled {
    pub fn val_6x(&self) -> u8 {
        match self {
            Self::Disabled => 0,
            Self::Enabled => 1,
        }
    }

    pub fn val_8x(&self) -> u8 {
        match self {
            Self::Disabled => 0,
            Self::Enabled => 0x20,
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum InvertIq {
    Standard,
    Inverted,
}

impl InvertIq {
    pub fn val_6x(&self) -> u8 {
        match self {
            Self::Standard => 0,
            Self::Inverted => 1,
        }
    }

    pub fn val_8x(&self) -> u8 {
        match self {
            Self::Standard => 0x40,
            Self::Inverted => 0,
        }
    }
}

#[derive(Clone)]
/// Only used on 8x; we force Lora for 6x at this time.
pub enum PacketParams {
    Lora(PacketParamsLora),
    Flrc(PacketParamsFlrc),
}

impl Default for PacketParams {
    fn default() -> Self {
        Self::Lora(Default::default())
    }
}

/// (sx126x)See DS, section 13.4.6.2.
/// (sx128x)See DS, see starting at table 14-51.
#[derive(Clone)]
pub struct PacketParamsLora {
    /// The LoRa® packet starts with a preamble sequence which is used to synchronize the receiver with the incoming signal. By
    /// default the packet is configured with a 12-symbol long sequence. This is a programmable variable so the preamble length
    /// may be extended; for example, in the interest of reducing the receiver duty cycle in receive intensive applications. The
    /// transmitted preamble length may vary from 10 to 65535 symbols, once the fixed overhead of the preamble data is
    /// considered. This permits the transmission of near arbitrarily long preamble sequences.
    ///
    /// sx1262: Premable len is packet params 1 and 2. Sx1282: param 1, using base + exp setup.
    pub preamble_len: u16,
    pub header_type: LoraHeaderType,
    /// Size of the payload (in bytes) to transmit or maximum size of the
    /// payload that the receiver can accept.
    /// Sx1280: Packet param 3.
    pub payload_len: u8,
    pub crc_enabled: CrcEnabled,
    /// Sx1280. Packet param 5.
    pub invert_iq: InvertIq,
}

impl Default for PacketParamsLora {
    fn default() -> Self {
        Self {
            preamble_len: 12, // Recommended in 8x. (Maybe 6x too)
            header_type: LoraHeaderType::VariableLength,
            payload_len: 0, // This is set during transmission.
            crc_enabled: CrcEnabled::Enabled,
            invert_iq: InvertIq::Standard,
        }
    }
}

/// 8x only. DS, Table 14-34
#[derive(Clone)]
pub struct PacketParamsFlrc {
    /// Param 1
    pub preamble_len: FlrcPreambleLen,
    /// Param 2
    pub sync_word_len: FlrcSyncWordLen,
    /// Param 3
    pub sync_word_combo: FlrcSyncWordCombo,
    /// Param 4
    pub packet_type: FlrcPacketType,
    /// Param 5
    pub payload_len: u8,
    /// Param 6
    pub crc: FlrcCrc,
}

impl Default for PacketParamsFlrc {
    fn default() -> Self {
        Self {
            preamble_len: FlrcPreambleLen::P16, // Minimum for bit rates other than 1Mb/s (8 there)
            sync_word_len: FlrcSyncWordLen::NoSync, // consider setting the sync if using multiple devices.
            sync_word_combo: FlrcSyncWordCombo::Disabled,
            packet_type: FlrcPacketType::LenVariable, // todo?
            payload_len: 0,
            crc: FlrcCrc::CrcOff,
        }
    }
}

// todo: CAD params A/R.
