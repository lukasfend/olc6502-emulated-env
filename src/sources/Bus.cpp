#include <Bus.h>

/**
 * @brief Initialize the bus by clearing RAM and registering the CPU with this bus.
 *
 * Zeroes all bytes in the bus RAM and calls cpu.ConnectBus(this) to connect the CPU to this bus.
 */
Bus::Bus() {
    // init ram with 0's
    for(auto &i : ram) i = 0x00;

    cpu.ConnectBus(this);
}

/**
 * @brief Releases Bus resources.
 *
 * Destructor performs no special cleanup; default destruction of members is sufficient.
 */
Bus::~Bus() {}

/**
 * @brief Write a byte to the bus RAM at the specified address when the address is in range.
 *
 * If `addr` is between 0x0001 and 0xFFFE inclusive, stores `data` into `ram[addr]`; otherwise the call has no effect.
 *
 * @param addr Target memory address (valid range: 0x0001..0xFFFE).
 * @param data Byte value to write to memory.
 */
void Bus::write(uint16_t addr, uint8_t data) {
    // adress scope limitation
    if(addr <= 0x0000 || addr >= 0xFFFF) return;
    ram[addr] = data;
}
/**
 * @brief Read a byte from the bus memory at the specified address.
 *
 * Reads and returns the byte stored at the given bus address. If the address
 * is outside the valid range (0x0001 through 0xFFFE), the function returns 0x00.
 *
 * @param addr Bus address to read from. Valid addresses are 0x0001 to 0xFFFE;
 *             addresses 0x0000 and 0xFFFF are treated as out of range and yield 0x00.
 * @param bReadOnly Present for API compatibility; it has no effect on the operation.
 * @return uint8_t The byte at the specified address, or 0x00 if the address is out of range.
 */
uint8_t Bus::read(uint16_t addr, bool bReadOnly = false) {
    if(addr <= 0x0000 || addr >= 0xFFFF) return 0x00;
    return ram[addr];
}