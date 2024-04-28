/// Split a u16 address into two bytes.
pub fn split_addr(addr: u16) -> (u8, u8) {
    let result = addr.to_be_bytes();
    (result[0], result[1])
}
