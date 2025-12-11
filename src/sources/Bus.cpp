#include <Bus.h>

Bus::Bus() {
    // init ram with 0's
    for(auto &i : ram) i = 0x00;

    cpu.ConnectBus(this);
}

Bus::~Bus() {}

void Bus::write(uint16_t addr, uint8_t data) {
    // adress scope limitation
    if(addr <= 0x0000 || addr >= 0xFFFF) return;
    ram[addr] = data;
}
uint8_t Bus::read(uint16_t addr, bool bReadOnly = false) {
    if(addr <= 0x0000 || addr >= 0xFFFF) return 0x00;
    return ram[addr];
}