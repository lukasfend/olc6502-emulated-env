#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <map>

class Bus;

/**
 *  MOS 6502 CPU core emulator representing registers, flags, and execution state.
 *
 *  Exposes public CPU registers and status flags, a bus connection point, and a disassembly
 *  helper. Provides internal addressing modes, opcode implementations, memory access,
 *  and control flow methods used to simulate the CPU cycle behavior.
 */

/**
 *  Connects the CPU to a memory/IO bus.
 *  @param n Pointer to a Bus instance that the CPU will use for reads and writes.
 */

/**
 *  Produce a disassembly map for a range of memory addresses.
 *  Returns a mapping from each address in the range [nStart, nStop] to the disassembled
 *  instruction string located at that address.
 *  @param nStart Starting address (inclusive) for disassembly.
 *  @param nStop  Ending address (inclusive) for disassembly.
 *  @returns std::map<uint16_t, std::string> mapping addresses to disassembled instruction text.
 */

/**
 *  Read a byte from the connected bus at the given address.
 *  @param addr 16-bit memory address to read from.
 *  @returns The byte value read from the bus.
 */

/**
 *  Write a byte to the connected bus at the given address.
 *  @param addr 16-bit memory address to write to.
 *  @param data Byte value to write.
 */

/**
 *  Retrieve the current value of a specified status flag.
 *  @param flags The FLAGS6502 flag to query.
 *  @returns `1` if the specified flag is set, `0` otherwise.
 */

/**
 *  Set or clear a specified status flag.
 *  @param flags The FLAGS6502 flag to modify.
 *  @param value `true` to set the flag, `false` to clear it.
 */

/**
 *  Fetch the next operand for the currently executing instruction.
 *  Performs the operand fetch based on the current addressing mode and returns
 *  the fetched byte.
 *  @returns The fetched byte value.
 */

/**
 *  Advance the CPU state by one clock cycle.
 *  Executes micro-operations for the current instruction and updates internal
 *  timing and state; may complete the current instruction when cycles reach zero.
 */

/**
 *  Reset the CPU to its initial power-on state.
 *  Initializes registers, flags, program counter, and internal timing to the
 *  standard reset values expected for a 6502 core.
 */

/**
 *  Handle a maskable interrupt request (IRQ).
 *  Pushes processor state and vectors execution to the IRQ handler as defined by
 *  the 6502 interrupt sequence.
 */

/**
 *  Handle a non-maskable interrupt (NMI).
 *  Pushes processor state and vectors execution to the NMI handler as defined by
 *  the 6502 interrupt sequence.
 */

/**
 *  Indicates whether the current instruction has finished executing.
 *  @returns `true` if the current instruction has completed, `false` otherwise.
 */
class r6502
{
public:
    r6502();
    ~r6502();

    // https://www.nesdev.org/wiki/Status_flags
    enum FLAGS6502
    {
        C = (1 << 0), // carry bit
        Z = (1 << 1), // zero
        I = (1 << 2), // disable interrupt
        D = (1 << 3), // decimal mode
        B = (1 << 4), // break
        U = (1 << 5), // --
        V = (1 << 6), // overflow
        N = (1 << 7)  // negative
    };

    uint8_t a = 0x00;      // Accumulator
    uint8_t x = 0x00;      // x reg
    uint8_t y = 0x00;      // y reg
    uint8_t stkp = 0x00;   // stackpointer
    uint16_t pc = 0x0000;  // programcounter
    uint8_t status = 0x00; // statusregister

    void ConnectBus(Bus *n) { bus = n; }

    std::map<uint16_t, std::string> disassemble(uint16_t nStart, uint16_t nStop);

private:
    // adressing mode
    uint8_t IMP();
    uint8_t IMM();
    uint8_t ZP0();
    uint8_t ZPX();
    uint8_t ZPY();
    uint8_t REL();
    uint8_t ABS();
    uint8_t ABX();
    uint8_t ABY();
    uint8_t IND();
    uint8_t IZX();
    uint8_t IZY();

private:
    // opcodes
    uint8_t ADC();
    uint8_t AND();
    uint8_t ASL();
    uint8_t BCC();
    uint8_t BCS();
    uint8_t BEQ();
    uint8_t BIT();
    uint8_t BMI();
    uint8_t BNE();
    uint8_t BPL();
    uint8_t BRK();
    uint8_t BVC();
    uint8_t BVS();
    uint8_t CLC();
    uint8_t CLD();
    uint8_t CLI();
    uint8_t CLV();
    uint8_t CMP();
    uint8_t CPX();
    uint8_t CPY();
    uint8_t DEC();
    uint8_t DEX();
    uint8_t DEY();
    uint8_t EOR();
    uint8_t INC();
    uint8_t INX();
    uint8_t INY();
    uint8_t JMP();
    uint8_t JSR();
    uint8_t LDA();
    uint8_t LDX();
    uint8_t LDY();
    uint8_t LSR();
    uint8_t NOP();
    uint8_t ORA();
    uint8_t PHA();
    uint8_t PHP();
    uint8_t PLA();
    uint8_t PLP();
    uint8_t ROL();
    uint8_t ROR();
    uint8_t RTI();
    uint8_t RTS();
    uint8_t SBC();
    uint8_t SEC();
    uint8_t SED();
    uint8_t SEI();
    uint8_t STA();
    uint8_t STX();
    uint8_t STY();
    uint8_t TAX();
    uint8_t TAY();
    uint8_t TSX();
    uint8_t TXA();
    uint8_t TXS();
    uint8_t TYA();

    uint8_t XXX(); // catch-all fallback

    bool complete();
    void clock();
    void reset();
    void irq(); // interrupt request
    void nmi(); // non maskable interrupt

    uint8_t fetch();
    uint8_t fetched = 0x00;
    uint16_t temp = 0x0000;
    uint16_t addr_abs = 0x0000;
    uint16_t addr_rel = 0x00;
    uint8_t opcode = 0x00;
    uint8_t cycles = 0;
    uint32_t clock_count = 0;

private:
    Bus *bus = nullptr;
    uint8_t read(uint16_t addr);
    void write(uint16_t addr, uint8_t data);

    uint8_t GetFlag(FLAGS6502 flags);
    void SetFlag(FLAGS6502 flags, bool value);

    struct INSTRUCTION {
        std::string name;
        uint8_t(r6502::*operate)(void) = nullptr;
        uint8_t(r6502::*addrmode)(void) = nullptr;
        uint8_t cycles = 0;
    };

    std::vector<INSTRUCTION> lookup;
};