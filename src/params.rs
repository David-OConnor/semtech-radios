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

/// DS, Table 13-47. Mod param 1.
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

/// DS, Table 13-47. Mod param 2.
/// "An increase in signal bandwidth permits the use of a higher effective data rate, thus reducing transmission time at the
/// expense of reduced sensitivity improvement."
/// Note that the lower settings here can result in 5s or higher OTA time! OTA seems to scale linearly (inversely)
/// with bandwidth.
#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
#[allow(non_camel_case_types, dead_code)]
pub enum LoraBandwidth {
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

/// DS, Table 13-49. Mod param 3.
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
}

#[repr(u8)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
/// Table 13-50. Mod param 4.
/// "For low data rates (typically for high SF or low BW) and very long payloads which may last several seconds in the air, the low
/// data rate optimization (LDRO) can be enabled. This reduces the number of bits per symbol to the given SF minus two (see
/// Section 6.1.4 "LoRa® Time-on-Air" on page 41) in order to allow the receiver to have a better tracking of the LoRa® signal.
/// Depending on the payload size, the low data rate optimization is usually recommended when a LoRa® symbol time is equal
/// or above 16.38 ms."
pub enum LoraLdrOptimization {
    Disabled = 0,
    Enabled = 1,
}

/// See DS, section 6.1.1: Modulation Parameter.
pub struct ModulationParamsLora {
    pub mod_bandwidth: LoraBandwidth,
    pub spreading_factor: LoraSpreadingFactor,
    pub coding_rate: LoraCodingRate,
    pub low_data_rate_optimization: LoraLdrOptimization,
}

impl Default for ModulationParamsLora {
    /// We set this up for a short airtime; modify these as default to make the transmission more robust to interference,
    /// and potentially increase range.
    /// https://www.semtech.com/design-support/lora-calculator
    /// With a 64-byte payload, this results in 3-60ms airtime., depending on payload len.
    fn default() -> Self {
        Self {
            mod_bandwidth: LoraBandwidth::BW_500,
            spreading_factor: LoraSpreadingFactor::SF5,
            /// "In normal conditions a
            /// factor of 4/5 provides the best trade-off; in the presence of strong interferers a higher
            /// coding rate may be used."
            coding_rate: LoraCodingRate::CR_4_5,
            low_data_rate_optimization: LoraLdrOptimization::Disabled,
        }
    }
}

/// DS, Table 13-67. Packet param 3.
/// Also, Section 6.1.3. "The LoRa® modem employs two types of packet formats: explicit and implicit. The explicit
/// packet includes a short header
/// that contains information about the number of bytes, coding rate and whether a CRC is used in the packet."
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum LoraHeaderType {
    /// Explict header
    VariableLength = 0x00,
    /// Implicit header
    FixedLength = 0x01,
}

/// See DS, section 13.4.6.2.
pub struct PacketParamsLora {
    /// The LoRa® packet starts with a preamble sequence which is used to synchronize the receiver with the incoming signal. By
    /// default the packet is configured with a 12-symbol long sequence. This is a programmable variable so the preamble length
    /// may be extended; for example, in the interest of reducing the receiver duty cycle in receive intensive applications. The
    /// transmitted preamble length may vary from 10 to 65535 symbols, once the fixed overhead of the preamble data is
    /// considered. This permits the transmission of near arbitrarily long preamble sequences.
    pub preamble_len: u16,
    pub header_type: LoraHeaderType,
    /// Size of the payload (in bytes) to transmit or maximum size of the
    /// payload that the receiver can accept.
    pub payload_len: u8,
    pub crc_enabled: bool,
    pub invert_iq: bool,
}

impl Default for PacketParamsLora {
    fn default() -> Self {
        Self {
            preamble_len: 12,
            header_type: LoraHeaderType::VariableLength,
            payload_len: 0, // This is set during transmission.
            crc_enabled: true,
            invert_iq: false,
        }
    }
}

// todo: CAD params A/R.
