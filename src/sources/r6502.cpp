#include "r6502.h"
#include "Bus.h"

/**
 * @brief Initialize the 6502 CPU opcode lookup table.
 *
 * Populates the emulator's opcode dispatch table with entries that map each
 * opcode to its mnemonic, operation implementation, addressing-mode resolver,
 * and base cycle count.
 */
r6502::r6502()
{
    using a = r6502;
    lookup =
        {
            {"BRK", &a::BRK, &a::IMM, 7},
            {"ORA", &a::ORA, &a::IZX, 6},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 3},
            {"ORA", &a::ORA, &a::ZP0, 3},
            {"ASL", &a::ASL, &a::ZP0, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"PHP", &a::PHP, &a::IMP, 3},
            {"ORA", &a::ORA, &a::IMM, 2},
            {"ASL", &a::ASL, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::NOP, &a::IMP, 4},
            {"ORA", &a::ORA, &a::ABS, 4},
            {"ASL", &a::ASL, &a::ABS, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"BPL", &a::BPL, &a::REL, 2},
            {"ORA", &a::ORA, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 4},
            {"ORA", &a::ORA, &a::ZPX, 4},
            {"ASL", &a::ASL, &a::ZPX, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"CLC", &a::CLC, &a::IMP, 2},
            {"ORA", &a::ORA, &a::ABY, 4},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 4},
            {"ORA", &a::ORA, &a::ABX, 4},
            {"ASL", &a::ASL, &a::ABX, 7},
            {"???", &a::XXX, &a::IMP, 7},
            {"JSR", &a::JSR, &a::ABS, 6},
            {"AND", &a::AND, &a::IZX, 6},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"BIT", &a::BIT, &a::ZP0, 3},
            {"AND", &a::AND, &a::ZP0, 3},
            {"ROL", &a::ROL, &a::ZP0, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"PLP", &a::PLP, &a::IMP, 4},
            {"AND", &a::AND, &a::IMM, 2},
            {"ROL", &a::ROL, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"BIT", &a::BIT, &a::ABS, 4},
            {"AND", &a::AND, &a::ABS, 4},
            {"ROL", &a::ROL, &a::ABS, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"BMI", &a::BMI, &a::REL, 2},
            {"AND", &a::AND, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 4},
            {"AND", &a::AND, &a::ZPX, 4},
            {"ROL", &a::ROL, &a::ZPX, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"SEC", &a::SEC, &a::IMP, 2},
            {"AND", &a::AND, &a::ABY, 4},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 4},
            {"AND", &a::AND, &a::ABX, 4},
            {"ROL", &a::ROL, &a::ABX, 7},
            {"???", &a::XXX, &a::IMP, 7},
            {"RTI", &a::RTI, &a::IMP, 6},
            {"EOR", &a::EOR, &a::IZX, 6},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 3},
            {"EOR", &a::EOR, &a::ZP0, 3},
            {"LSR", &a::LSR, &a::ZP0, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"PHA", &a::PHA, &a::IMP, 3},
            {"EOR", &a::EOR, &a::IMM, 2},
            {"LSR", &a::LSR, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"JMP", &a::JMP, &a::ABS, 3},
            {"EOR", &a::EOR, &a::ABS, 4},
            {"LSR", &a::LSR, &a::ABS, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"BVC", &a::BVC, &a::REL, 2},
            {"EOR", &a::EOR, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 4},
            {"EOR", &a::EOR, &a::ZPX, 4},
            {"LSR", &a::LSR, &a::ZPX, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"CLI", &a::CLI, &a::IMP, 2},
            {"EOR", &a::EOR, &a::ABY, 4},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 4},
            {"EOR", &a::EOR, &a::ABX, 4},
            {"LSR", &a::LSR, &a::ABX, 7},
            {"???", &a::XXX, &a::IMP, 7},
            {"RTS", &a::RTS, &a::IMP, 6},
            {"ADC", &a::ADC, &a::IZX, 6},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 3},
            {"ADC", &a::ADC, &a::ZP0, 3},
            {"ROR", &a::ROR, &a::ZP0, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"PLA", &a::PLA, &a::IMP, 4},
            {"ADC", &a::ADC, &a::IMM, 2},
            {"ROR", &a::ROR, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"JMP", &a::JMP, &a::IND, 5},
            {"ADC", &a::ADC, &a::ABS, 4},
            {"ROR", &a::ROR, &a::ABS, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"BVS", &a::BVS, &a::REL, 2},
            {"ADC", &a::ADC, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 4},
            {"ADC", &a::ADC, &a::ZPX, 4},
            {"ROR", &a::ROR, &a::ZPX, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"SEI", &a::SEI, &a::IMP, 2},
            {"ADC", &a::ADC, &a::ABY, 4},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 4},
            {"ADC", &a::ADC, &a::ABX, 4},
            {"ROR", &a::ROR, &a::ABX, 7},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 2},
            {"STA", &a::STA, &a::IZX, 6},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 6},
            {"STY", &a::STY, &a::ZP0, 3},
            {"STA", &a::STA, &a::ZP0, 3},
            {"STX", &a::STX, &a::ZP0, 3},
            {"???", &a::XXX, &a::IMP, 3},
            {"DEY", &a::DEY, &a::IMP, 2},
            {"???", &a::NOP, &a::IMP, 2},
            {"TXA", &a::TXA, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"STY", &a::STY, &a::ABS, 4},
            {"STA", &a::STA, &a::ABS, 4},
            {"STX", &a::STX, &a::ABS, 4},
            {"???", &a::XXX, &a::IMP, 4},
            {"BCC", &a::BCC, &a::REL, 2},
            {"STA", &a::STA, &a::IZY, 6},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 6},
            {"STY", &a::STY, &a::ZPX, 4},
            {"STA", &a::STA, &a::ZPX, 4},
            {"STX", &a::STX, &a::ZPY, 4},
            {"???", &a::XXX, &a::IMP, 4},
            {"TYA", &a::TYA, &a::IMP, 2},
            {"STA", &a::STA, &a::ABY, 5},
            {"TXS", &a::TXS, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 5},
            {"???", &a::NOP, &a::IMP, 5},
            {"STA", &a::STA, &a::ABX, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"LDY", &a::LDY, &a::IMM, 2},
            {"LDA", &a::LDA, &a::IZX, 6},
            {"LDX", &a::LDX, &a::IMM, 2},
            {"???", &a::XXX, &a::IMP, 6},
            {"LDY", &a::LDY, &a::ZP0, 3},
            {"LDA", &a::LDA, &a::ZP0, 3},
            {"LDX", &a::LDX, &a::ZP0, 3},
            {"???", &a::XXX, &a::IMP, 3},
            {"TAY", &a::TAY, &a::IMP, 2},
            {"LDA", &a::LDA, &a::IMM, 2},
            {"TAX", &a::TAX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"LDY", &a::LDY, &a::ABS, 4},
            {"LDA", &a::LDA, &a::ABS, 4},
            {"LDX", &a::LDX, &a::ABS, 4},
            {"???", &a::XXX, &a::IMP, 4},
            {"BCS", &a::BCS, &a::REL, 2},
            {"LDA", &a::LDA, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 5},
            {"LDY", &a::LDY, &a::ZPX, 4},
            {"LDA", &a::LDA, &a::ZPX, 4},
            {"LDX", &a::LDX, &a::ZPY, 4},
            {"???", &a::XXX, &a::IMP, 4},
            {"CLV", &a::CLV, &a::IMP, 2},
            {"LDA", &a::LDA, &a::ABY, 4},
            {"TSX", &a::TSX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 4},
            {"LDY", &a::LDY, &a::ABX, 4},
            {"LDA", &a::LDA, &a::ABX, 4},
            {"LDX", &a::LDX, &a::ABY, 4},
            {"???", &a::XXX, &a::IMP, 4},
            {"CPY", &a::CPY, &a::IMM, 2},
            {"CMP", &a::CMP, &a::IZX, 6},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"CPY", &a::CPY, &a::ZP0, 3},
            {"CMP", &a::CMP, &a::ZP0, 3},
            {"DEC", &a::DEC, &a::ZP0, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"INY", &a::INY, &a::IMP, 2},
            {"CMP", &a::CMP, &a::IMM, 2},
            {"DEX", &a::DEX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 2},
            {"CPY", &a::CPY, &a::ABS, 4},
            {"CMP", &a::CMP, &a::ABS, 4},
            {"DEC", &a::DEC, &a::ABS, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"BNE", &a::BNE, &a::REL, 2},
            {"CMP", &a::CMP, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 4},
            {"CMP", &a::CMP, &a::ZPX, 4},
            {"DEC", &a::DEC, &a::ZPX, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"CLD", &a::CLD, &a::IMP, 2},
            {"CMP", &a::CMP, &a::ABY, 4},
            {"NOP", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 4},
            {"CMP", &a::CMP, &a::ABX, 4},
            {"DEC", &a::DEC, &a::ABX, 7},
            {"???", &a::XXX, &a::IMP, 7},
            {"CPX", &a::CPX, &a::IMM, 2},
            {"SBC", &a::SBC, &a::IZX, 6},
            {"???", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"CPX", &a::CPX, &a::ZP0, 3},
            {"SBC", &a::SBC, &a::ZP0, 3},
            {"INC", &a::INC, &a::ZP0, 5},
            {"???", &a::XXX, &a::IMP, 5},
            {"INX", &a::INX, &a::IMP, 2},
            {"SBC", &a::SBC, &a::IMM, 2},
            {"NOP", &a::NOP, &a::IMP, 2},
            {"???", &a::SBC, &a::IMP, 2},
            {"CPX", &a::CPX, &a::ABS, 4},
            {"SBC", &a::SBC, &a::ABS, 4},
            {"INC", &a::INC, &a::ABS, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"BEQ", &a::BEQ, &a::REL, 2},
            {"SBC", &a::SBC, &a::IZY, 5},
            {"???", &a::XXX, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 8},
            {"???", &a::NOP, &a::IMP, 4},
            {"SBC", &a::SBC, &a::ZPX, 4},
            {"INC", &a::INC, &a::ZPX, 6},
            {"???", &a::XXX, &a::IMP, 6},
            {"SED", &a::SED, &a::IMP, 2},
            {"SBC", &a::SBC, &a::ABY, 4},
            {"NOP", &a::NOP, &a::IMP, 2},
            {"???", &a::XXX, &a::IMP, 7},
            {"???", &a::NOP, &a::IMP, 4},
            {"SBC", &a::SBC, &a::ABX, 4},
            {"INC", &a::INC, &a::ABX, 7},
            {"???", &a::XXX, &a::IMP, 7},
        };
}

/**
 * @brief Destroy the r6502 CPU emulator instance and release any owned resources.
 *
 * Performs any necessary cleanup when an r6502 object is destroyed.
 */
r6502::~r6502()
{
}

/**
 * @brief Read a byte from the system bus at the given 16-bit address using CPU read mode.
 *
 * @param a 16-bit memory address to read from.
 * @return uint8_t The byte value read from the bus at address `a`.
 */
uint8_t r6502::read(uint16_t a)
{

    return bus->read(a, false);
}

/**
 * @brief Write a byte to the connected bus at the specified address.
 *
 * @param a 16-bit address to write to.
 * @param d Byte value to write.
 */
void r6502::write(uint16_t a, uint8_t d)
{
    bus->write(a, d);
}

/**
 * @brief Initialize CPU state and set the program counter from the reset vector.
 *
 * Reads the 16-bit reset vector at 0xFFFC/0xFFFD into the program counter, clears A, X, Y,
 * sets the stack pointer to 0xFD, initializes the status register with the unused flag set,
 * clears temporary addressing/fetch state, and sets the cycle counter to 8.
 */
void r6502::reset()
{

    addr_abs = 0xFFFC;
    uint16_t lo = read(addr_abs + 0);
    uint16_t hi = read(addr_abs + 1);

    pc = (hi << 8) | lo;

    a = 0;
    x = 0;
    y = 0;
    stkp = 0xFD;
    status = 0x00 | U;

    addr_rel = 0x0000;
    addr_abs = 0x0000;
    fetched = 0x00;

    cycles = 8;
}

/**
 * @brief Handle a maskable IRQ (interrupt request) if interrupts are enabled.
 *
 * Pushes the current program counter and status onto the stack, sets the interrupt
 * and system flags appropriately, loads the IRQ vector from 0xFFFE/0xFFFF into the
 * program counter, and schedules the CPU for the IRQ service routine by setting
 * the cycle count.
 */
void r6502::irq()
{

    if (GetFlag(I) == 0)
    {

        write(0x0100 + stkp, (pc >> 8) & 0x00FF);
        stkp--;
        write(0x0100 + stkp, pc & 0x00FF);
        stkp--;

        SetFlag(B, 0);
        SetFlag(U, 1);
        SetFlag(I, 1);
        write(0x0100 + stkp, status);
        stkp--;

        addr_abs = 0xFFFE;
        uint16_t lo = read(addr_abs + 0);
        uint16_t hi = read(addr_abs + 1);
        pc = (hi << 8) | lo;

        cycles = 7;
    }
}

/**
 * @brief Trigger a non-maskable interrupt (NMI) on the CPU.
 *
 * Pushes the current program counter and status register onto the stack, sets the B flag to 0,
 * the U (unused) flag to 1, and the I (interrupt disable) flag to 1, then loads the NMI vector
 * from address 0xFFFA/0xFFFB into the program counter and sets the CPU cycle count to 8.
 */
void r6502::nmi()
{
    write(0x0100 + stkp, (pc >> 8) & 0x00FF);
    stkp--;
    write(0x0100 + stkp, pc & 0x00FF);
    stkp--;

    SetFlag(B, 0);
    SetFlag(U, 1);
    SetFlag(I, 1);
    write(0x0100 + stkp, status);
    stkp--;

    addr_abs = 0xFFFA;
    uint16_t lo = read(addr_abs + 0);
    uint16_t hi = read(addr_abs + 1);
    pc = (hi << 8) | lo;

    cycles = 8;
}

/**
 * @brief Advance the CPU by one clock cycle and execute the next instruction when ready.
 *
 * When the internal cycle counter reaches zero, this method fetches the next opcode
 * from memory, updates the program counter and status, invokes the instruction's
 * addressing mode and operation handlers, and adjusts the cycle counter based on
 * the instruction's base and any additional cycles from addressing/operation.
 * Always increments the internal clock counter and decrements the remaining cycle count.
 */
void r6502::clock()
{

    if (cycles == 0)
    {

        opcode = read(pc);

#ifdef LOGMODE
        uint16_t log_pc = pc;
#endif

        SetFlag(U, true);

        pc++;

        cycles = lookup[opcode].cycles;

        uint8_t additional_cycle1 = (this->*lookup[opcode].addrmode)();

        uint8_t additional_cycle2 = (this->*lookup[opcode].operate)();

        cycles += (additional_cycle1 & additional_cycle2);

        SetFlag(U, true);

#ifdef LOGMODE

        if (logfile == nullptr)
            logfile = fopen("r6502.txt", "wt");
        if (logfile != nullptr)
        {
            fprintf(logfile, "%10d:%02d PC:%04X %s A:%02X X:%02X Y:%02X %s%s%s%s%s%s%s%s STKP:%02X\n",
                    clock_count, 0, log_pc, "XXX", a, x, y,
                    GetFlag(N) ? "N" : ".", GetFlag(V) ? "V" : ".", GetFlag(U) ? "U" : ".",
                    GetFlag(B) ? "B" : ".", GetFlag(D) ? "D" : ".", GetFlag(I) ? "I" : ".",
                    GetFlag(Z) ? "Z" : ".", GetFlag(C) ? "C" : ".", stkp);
        }
#endif
    }

    clock_count++;

    cycles--;
}

/**
 * @brief Check whether a processor status flag is set.
 *
 * @param f The status flag to test (one of FLAGS6502).
 * @return uint8_t `1` if the specified status flag is set, `0` otherwise.
 */
uint8_t r6502::GetFlag(FLAGS6502 f)
{
    return ((status & f) > 0) ? 1 : 0;
}

/**
 * @brief Set or clear a CPU status flag.
 *
 * Modifies the CPU internal status register by setting the specified flag when v is true
 * or clearing it when v is false.
 *
 * @param f The status flag to modify (one of FLAGS6502).
 * @param v True to set the flag, false to clear it.
 */
void r6502::SetFlag(FLAGS6502 f, bool v)
{
    if (v)
        status |= f;
    else
        status &= ~f;
}

/**
 * @brief Implied addressing mode that selects the accumulator as the operand.
 *
 * Sets the fetched operand to the accumulator register (A) for instructions
 * that operate on the accumulator using implied addressing.
 *
 * @return uint8_t 0 indicating no additional clock cycles are required.
 */
uint8_t r6502::IMP()
{
    fetched = a;
    return 0;
}

/**
 * @brief Resolve immediate addressing mode by pointing to the next byte operand.
 *
 * Sets the absolute address to the current program counter and advances the program counter
 * so the CPU can fetch the immediate operand.
 *
 * @return uint8_t 0 (no additional clock cycle required).
 */
uint8_t r6502::IMM()
{
    addr_abs = pc++;
    return 0;
}

/**
 * @brief Resolve zero-page addressing by reading the next byte and storing it in addr_abs.
 *
 * Reads a single byte from memory at the program counter, increments the program counter,
 * and masks the result to the zero page (0x00FF) before storing it in addr_abs.
 *
 * @return uint8_t `0` — no additional CPU cycles required.
 */
uint8_t r6502::ZP0()
{
    addr_abs = read(pc);
    pc++;
    addr_abs &= 0x00FF;
    return 0;
}

/**
 * @brief Resolve zero page addressing with X register offset.
 *
 * Sets the effective address to the zero-page location formed by adding the X register
 * to the immediate zero-page operand and advances the program counter. The resulting
 * address is masked to the zero page (0x00FF).
 *
 * @return uint8_t `0` when no additional CPU cycles are required.
 */
uint8_t r6502::ZPX()
{
    addr_abs = (read(pc) + x);
    pc++;
    addr_abs &= 0x00FF;
    return 0;
}

/**
 * @brief Zero Page,Y addressing mode: fetches a zero-page base from the next byte, adds Y, and stores the wrapped 8-bit address.
 *
 * Reads the zero-page base operand at the program counter, increments PC, adds the Y register, and masks the result to 8 bits so the address wraps within the zero page (0x0000–0x00FF).
 *
 * @return uint8_t `0` indicating no additional clock cycles.
 */
uint8_t r6502::ZPY()
{
    addr_abs = (read(pc) + y);
    pc++;
    addr_abs &= 0x00FF;
    return 0;
}

/**
 * @brief Read a relative branch offset from memory, sign-extend it, and advance the program counter.
 *
 * Reads the byte at the current program counter into the CPU's `addr_rel`, increments `pc` by one,
 * and sign-extends the byte into a 16-bit offset stored in `addr_rel` (negative offsets have the high
 * byte set to 0xFF).
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::REL()
{
    addr_rel = read(pc);
    pc++;
    if (addr_rel & 0x80)
        addr_rel |= 0xFF00;
    return 0;
}

/**
 * @brief Resolve the absolute addressing mode.
 *
 * Reads a 16-bit address from memory at the program counter, advances the program
 * counter by two, and stores the resulting address in `addr_abs`.
 *
 * @return uint8_t `0` indicating no additional CPU cycle is required for this addressing mode.
 */
uint8_t r6502::ABS()
{
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;

    addr_abs = (hi << 8) | lo;

    return 0;
}

/**
 * @brief Resolve the Absolute,X addressing mode.
 *
 * Reads a 16-bit base address from the program counter, adds the X register offset,
 * and stores the resulting effective address in `addr_abs`. The program counter is
 * advanced past the two address bytes.
 *
 * @return uint8_t `1` if adding X crossed a 0xFF->0x00 page boundary (indicating an extra cycle is required), `0` otherwise.
 */
uint8_t r6502::ABX()
{
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;

    addr_abs = (hi << 8) | lo;
    addr_abs += x;

    if ((addr_abs & 0xFF00) != (hi << 8))
        return 1;
    else
        return 0;
}

/**
 * @brief Compute an absolute address from the next two memory bytes, add the Y register, and detect page crossing.
 *
 * Sets r6502::addr_abs to the formed address and advances the program counter past the two operand bytes; reads those bytes from memory.
 *
 * @return uint8_t `1` if adding Y crosses a 256-byte page boundary, `0` otherwise.
 */
uint8_t r6502::ABY()
{
    uint16_t lo = read(pc);
    pc++;
    uint16_t hi = read(pc);
    pc++;

    addr_abs = (hi << 8) | lo;
    addr_abs += y;

    if ((addr_abs & 0xFF00) != (hi << 8))
        return 1;
    else
        return 0;
}

/**
 * @brief Resolve the indirect addressing operand and store the effective address.
 *
 * Reads a 16-bit pointer from the program counter and loads the 16-bit effective
 * address from that pointer into `addr_abs`. If the pointer's low byte is 0xFF,
 * the high byte is fetched from the same page (emulating the 6502 page-wrapping bug).
 *
 * @return `0` (no additional cycles)
 */
uint8_t r6502::IND()
{
    uint16_t ptr_lo = read(pc);
    pc++;
    uint16_t ptr_hi = read(pc);
    pc++;

    uint16_t ptr = (ptr_hi << 8) | ptr_lo;

    if (ptr_lo == 0x00FF)
    {
        addr_abs = (read(ptr & 0xFF00) << 8) | read(ptr + 0);
    }
    else
    {
        addr_abs = (read(ptr + 1) << 8) | read(ptr + 0);
    }

    return 0;
}

/**
 * @brief Resolve the indexed-indirect (IZX / (d, X)) addressing mode and store the effective address.
 *
 * Computes a 16-bit effective address by taking a zero-page base from the program counter,
 * adding the X register (with zero-page wrap-around), reading the low/high bytes from zero page,
 * and combining them into addr_abs.
 *
 * @return uint8_t Always returns 0 (no additional CPU cycles).
 */
uint8_t r6502::IZX()
{
    uint16_t t = read(pc);
    pc++;

    uint16_t lo = read((uint16_t)(t + (uint16_t)x) & 0x00FF);
    uint16_t hi = read((uint16_t)(t + (uint16_t)x + 1) & 0x00FF);

    addr_abs = (hi << 8) | lo;

    return 0;
}

/**
 * @brief Compute the effective address for the indirect-indexed (IZY) addressing mode and store it in addr_abs.
 *
 * Reads a zero-page pointer from memory at PC (increments PC), dereferences the pointer to form a 16-bit base address, adds the Y register, and updates addr_abs.
 * @return uint8_t `1` if adding Y crosses a page boundary (requires an extra CPU cycle), `0` otherwise.
 */
uint8_t r6502::IZY()
{
    uint16_t t = read(pc);
    pc++;

    uint16_t lo = read(t & 0x00FF);
    uint16_t hi = read((t + 1) & 0x00FF);

    addr_abs = (hi << 8) | lo;
    addr_abs += y;

    if ((addr_abs & 0xFF00) != (hi << 8))
        return 1;
    else
        return 0;
}

/**
 * @brief Obtains the current instruction operand into the CPU's fetched field.
 *
 * Reads the operand from memory into the internal `fetched` field for the instruction
 * currently referenced by `opcode`. For implied addressing modes the `fetched`
 * value is left as preloaded and is not read from memory.
 *
 * @return uint8_t The operand value stored in `fetched`.
 */
uint8_t r6502::fetch()
{
    if (!(lookup[opcode].addrmode == &r6502::IMP))
        fetched = read(addr_abs);
    return fetched;
}

/**
 * @brief Adds the fetched operand and the current carry flag to the accumulator and stores the result in A.
 *
 * Updates the processor status flags according to the result:
 * - Carry (C): set if the unsigned result is greater than 255.
 * - Zero  (Z): set if the low 8 bits of the result are zero.
 * - Overflow (V): set if signed overflow occurred.
 * - Negative (N): set from bit 7 of the result.
 *
 * @return uint8_t `1` indicating this instruction may require an additional clock cycle. 
 */
uint8_t r6502::ADC()
{

    fetch();

    temp = (uint16_t)a + (uint16_t)fetched + (uint16_t)GetFlag(C);

    SetFlag(C, temp > 255);

    SetFlag(Z, (temp & 0x00FF) == 0);

    SetFlag(V, (~((uint16_t)a ^ (uint16_t)fetched) & ((uint16_t)a ^ (uint16_t)temp)) & 0x0080);

    SetFlag(N, temp & 0x80);

    a = temp & 0x00FF;

    return 1;
}

/**
 * @brief Subtracts the fetched operand from the accumulator using the carry flag.
 *
 * Performs subtraction via two's-complement addition (inverts the fetched byte and adds it
 * to the accumulator plus the carry flag), updates the C, Z, V, and N status flags, and
 * stores the low 8 bits of the result in the accumulator.
 *
 * @return uint8_t `1` if the instruction may require an additional cycle, `0` otherwise.
 */
uint8_t r6502::SBC()
{
    fetch();

    uint16_t value = ((uint16_t)fetched) ^ 0x00FF;

    temp = (uint16_t)a + value + (uint16_t)GetFlag(C);
    SetFlag(C, temp & 0xFF00);
    SetFlag(Z, ((temp & 0x00FF) == 0));
    SetFlag(V, (temp ^ (uint16_t)a) & (temp ^ value) & 0x0080);
    SetFlag(N, temp & 0x0080);
    a = temp & 0x00FF;
    return 1;
}

/**
 * @brief Performs a bitwise AND between the accumulator and the fetched operand, stores the result in the accumulator, and updates Zero and Negative flags.
 *
 * @return uint8_t 1 indicating this instruction contributes one additional cycle.
 */
uint8_t r6502::AND()
{
    fetch();
    a = a & fetched;
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 1;
}

/**
 * @brief Perform an arithmetic left shift on the accumulator or memory operand and update status flags.
 *
 * Shifts the fetched byte left by one bit, stores the low 8 bits back into the accumulator when using
 * implied addressing or writes them to memory at addr_abs for other addressing modes. Updates the
 * Carry flag with the bit shifted out, the Zero flag if the resulting byte is zero, and the Negative
 * flag from bit 7 of the resulting byte.
 *
 * @return uint8_t Extra cycles required (always 0 for this implementation).
 */
uint8_t r6502::ASL()
{
    fetch();
    temp = (uint16_t)fetched << 1;
    SetFlag(C, (temp & 0xFF00) > 0);
    SetFlag(Z, (temp & 0x00FF) == 0x00);
    SetFlag(N, temp & 0x80);
    if (lookup[opcode].addrmode == &r6502::IMP)
        a = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}

/**
 * @brief Branch on carry clear.
 *
 * If the Carry flag is clear, adds the signed relative offset to the program counter and
 * increments the CPU cycle count for the branch; an additional cycle is added if the branch
 * crosses a page boundary.
 *
 * @return uint8_t 0
 */
uint8_t r6502::BCC()
{
    if (GetFlag(C) == 0)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Branches to a relative address if the carry flag is set.
 *
 * If the carry flag is set, advances the program counter by the instruction's
 * relative offset and increments the cycle count for a taken branch. Adds an
 * additional cycle if the branch crosses a page boundary.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::BCS()
{
    if (GetFlag(C) == 1)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Branches to the relative address when the zero flag is set.
 *
 * If the Z flag is set, advances the program counter by the signed relative offset
 * stored in addr_rel. Increments the CPU cycle count by 1 when the branch is taken
 * and by an additional 1 if the branch crosses a page boundary.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::BEQ()
{
    if (GetFlag(Z) == 1)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Tests accumulator bits against the fetched operand and updates status flags.
 *
 * Performs a bitwise AND between the accumulator and the fetched value, sets the Zero flag if the result is zero, and copies bit 7 and bit 6 of the fetched value into the Negative and Overflow flags respectively.
 *
 * @return uint8_t Number of additional clock cycles required (always 0).
 */
uint8_t r6502::BIT()
{
    fetch();
    temp = a & fetched;
    SetFlag(Z, (temp & 0x00FF) == 0x00);
    SetFlag(N, fetched & (1 << 7));
    SetFlag(V, fetched & (1 << 6));
    return 0;
}

/**
 * @brief Branch to a relative address if the Negative flag is set.
 *
 * If the Negative flag is set, updates the program counter to the target
 * relative address and increments the CPU cycle count by 1. If the branch
 * crosses a page boundary, increments the cycle count by an additional 1.
 *
 * @return 0
 */
uint8_t r6502::BMI()
{
    if (GetFlag(N) == 1)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Branches to the relative address if the zero flag is clear.
 *
 * If the zero flag is clear, updates the program counter to the computed relative target and increments the cycle count;
 * an additional cycle is added if the branch crosses a page boundary.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::BNE()
{
    if (GetFlag(Z) == 0)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Branches to a relative address if the Negative flag is clear.
 *
 * If the Negative flag is clear, advances the program counter by the
 * signed relative offset and increments the internal cycle count by 1.
 * If the branch crosses a page boundary, increments the cycle count by an
 * additional 1.
 *
 * @return 0
 */
uint8_t r6502::BPL()
{
    if (GetFlag(N) == 0)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Execute the BRK (force interrupt) instruction.
 *
 * Pushes the program counter and status register onto the stack, sets the
 * interrupt disable flag and the break flag while saving status, then loads
 * the interrupt vector from 0xFFFE/0xFFFF into the program counter.
 *
 * @return uint8_t `0` — no extra cycles beyond the instruction's base cycles.
 */
uint8_t r6502::BRK()
{
    pc++;

    SetFlag(I, 1);
    write(0x0100 + stkp, (pc >> 8) & 0x00FF);
    stkp--;
    write(0x0100 + stkp, pc & 0x00FF);
    stkp--;

    SetFlag(B, 1);
    write(0x0100 + stkp, status);
    stkp--;
    SetFlag(B, 0);

    pc = (uint16_t)read(0xFFFE) | ((uint16_t)read(0xFFFF) << 8);
    return 0;
}

/**
 * @brief Branches to the relative address if the overflow flag is clear.
 *
 * If the V (overflow) flag is clear, advances the cycle count for the branch,
 * computes the absolute target from the relative offset and updates the
 * program counter. Adds an additional cycle if the branch crosses a page
 * boundary.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::BVC()
{
    if (GetFlag(V) == 0)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Branches to a relative address when the overflow flag is set.
 *
 * If the overflow flag is set, advances the program counter by the signed
 * relative offset, increments the cycle count by one for the taken branch
 * and increments by one additional cycle if the branch crosses a page
 * boundary.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::BVS()
{
    if (GetFlag(V) == 1)
    {
        cycles++;
        addr_abs = pc + addr_rel;

        if ((addr_abs & 0xFF00) != (pc & 0xFF00))
            cycles++;

        pc = addr_abs;
    }
    return 0;
}

/**
 * @brief Clears the carry flag.
 *
 * Clears the CPU carry status flag.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::CLC()
{
    SetFlag(C, false);
    return 0;
}

/**
 * @brief Clear the decimal mode processor flag.
 *
 * @return uint8_t `0` (no additional cycles).
 */
uint8_t r6502::CLD()
{
    SetFlag(D, false);
    return 0;
}

/**
 * @brief Clears the Interrupt Disable flag to allow maskable interrupts.
 *
 * Clears the CPU status Interrupt Disable (I) flag so maskable IRQs can be serviced.
 *
 * @return uint8_t 0
 */
uint8_t r6502::CLI()
{
    SetFlag(I, false);
    return 0;
}

/**
 * @brief Clear the overflow (V) flag in the status register.
 *
 * Clears the CPU's overflow flag.
 *
 * @return uint8_t 0 — no additional cycles. 
 */
uint8_t r6502::CLV()
{
    SetFlag(V, false);
    return 0;
}

/**
 * @brief Compare the accumulator with the fetched operand and update processor flags.
 *
 * Performs A - operand (without storing the result) and updates the carry, zero,
 * and negative flags to reflect the comparison:
 * - Carry (C) set if A is greater than or equal to the operand.
 * - Zero (Z) set if the low byte of the subtraction is zero (A equals operand).
 * - Negative (N) set according to bit 7 of the subtraction result.
 *
 * @return uint8_t Number of additional cycles consumed by the instruction (always 1).
 */
uint8_t r6502::CMP()
{
    fetch();
    temp = (uint16_t)a - (uint16_t)fetched;
    SetFlag(C, a >= fetched);
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    return 1;
}

/**
 * @brief Compares the X register with the fetched operand and updates processor flags.
 *
 * Performs X - operand (using 8-bit semantics) and sets:
 * - Carry (C) if X is greater than or equal to the operand.
 * - Zero (Z) if the low 8 bits of the result are zero (X == operand).
 * - Negative (N) if bit 7 of the 8-bit result is set.
 *
 * @return uint8_t 0 (no additional CPU cycle). 
 */
uint8_t r6502::CPX()
{
    fetch();
    temp = (uint16_t)x - (uint16_t)fetched;
    SetFlag(C, x >= fetched);
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    return 0;
}

/**
 * Compares the Y register with the fetched operand and updates CPU flags.
 *
 * Sets the Carry flag if Y is greater than or equal to the operand, the Zero
 * flag if Y equals the operand, and the Negative flag from bit 7 of the result
 * (Y - operand).
 *
 * @return uint8_t 0 indicating no additional clock cycles beyond the base. 
 */
uint8_t r6502::CPY()
{
    fetch();
    temp = (uint16_t)y - (uint16_t)fetched;
    SetFlag(C, y >= fetched);
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    return 0;
}

/**
 * @brief Decrements the value at the resolved effective address by one and updates flags.
 *
 * Decrements the byte located at the CPU's current absolute address (addr_abs), writes the low
 * 8-bit result back to memory, and updates the Zero and Negative status flags based on the result.
 *
 * @return uint8_t Always returns 0 (no additional cycle penalty from this operation itself).
 */
uint8_t r6502::DEC()
{
    fetch();
    temp = fetched - 1;
    write(addr_abs, temp & 0x00FF);
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    return 0;
}

/**
 * @brief Decrement the X register and update processor flags.
 *
 * Decrements the X register by one, sets the zero flag if X becomes 0,
 * and sets the negative flag if bit 7 of X is set.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::DEX()
{
    x--;
    SetFlag(Z, x == 0x00);
    SetFlag(N, x & 0x80);
    return 0;
}

/**
 * @brief Decrements the Y register and updates processor flags.
 *
 * Decrements the Y register by one, sets the Zero flag if Y equals 0, and sets the Negative flag based on bit 7 of Y.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::DEY()
{
    y--;
    SetFlag(Z, y == 0x00);
    SetFlag(N, y & 0x80);
    return 0;
}

/**
 * @brief Performs bitwise exclusive OR between the accumulator and the fetched operand, stores the result in the accumulator, and updates zero and negative flags.
 *
 * @return uint8_t `1` if the instruction requires an additional CPU cycle, `0` otherwise.
 */
uint8_t r6502::EOR()
{
    fetch();
    a = a ^ fetched;
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 1;
}

/**
 * @brief Increment the value in memory at the resolved address by one and update processor flags.
 *
 * Increments the byte at addr_abs, stores the low 8 bits back to memory, and updates the Zero and Negative flags
 * based on the resulting 8-bit value.
 *
 * @return uint8_t `0` (no additional clock cycles).
 */
uint8_t r6502::INC()
{
    fetch();
    temp = fetched + 1;
    write(addr_abs, temp & 0x00FF);
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    return 0;
}

/**
 * @brief Increment the X register by one and update processor flags.
 *
 * Increments register X, sets the Zero flag if X equals 0, and sets the Negative
 * flag if bit 7 of X is 1.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::INX()
{
    x++;
    SetFlag(Z, x == 0x00);
    SetFlag(N, x & 0x80);
    return 0;
}

/**
 * @brief Increment the Y register by one and update processor flags.
 *
 * Increments the Y register, sets the Zero flag if Y equals 0, and sets the Negative
 * flag based on the high bit of Y.
 *
 * @return uint8_t `0`
 */
uint8_t r6502::INY()
{
    y++;
    SetFlag(Z, y == 0x00);
    SetFlag(N, y & 0x80);
    return 0;
}

/**
 * @brief Set the program counter to the resolved absolute address.
 *
 * Updates the CPU's program counter to the address previously resolved by the addressing mode.
 *
 * @return 0 No additional cycles are required.
 */
uint8_t r6502::JMP()
{
    pc = addr_abs;
    return 0;
}

/**
 * @brief Pushes the return address onto the stack and transfers execution to the subroutine target.
 *
 * The address pushed is the program counter minus one, stored as high byte then low byte on the CPU stack.
 *
 * @return uint8_t Always returns `0`.
 */
uint8_t r6502::JSR()
{
    pc--;

    write(0x0100 + stkp, (pc >> 8) & 0x00FF);
    stkp--;
    write(0x0100 + stkp, pc & 0x00FF);
    stkp--;

    pc = addr_abs;
    return 0;
}

/**
 * @brief Load the accumulator with the fetched operand and update status flags.
 *
 * Loads the fetched operand into the accumulator register `A`, sets the zero flag if
 * `A` equals 0x00, and sets the negative flag if bit 7 of `A` is 1.
 *
 * @return uint8_t `1` if the instruction requires an additional cycle, `0` otherwise.
 */
uint8_t r6502::LDA()
{
    fetch();
    a = fetched;
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 1;
}

/**
 * @brief Load the X register from the fetched operand and update processor flags.
 *
 * Sets the Zero flag if X becomes 0x00 and the Negative flag according to bit 7 of X.
 *
 * @return uint8_t `1` indicating this instruction consumes an extra cycle. 
 */
uint8_t r6502::LDX()
{
    fetch();
    x = fetched;
    SetFlag(Z, x == 0x00);
    SetFlag(N, x & 0x80);
    return 1;
}

/**
 * @brief Load the fetched operand into the Y register and update status flags.
 *
 * Sets the Zero flag if Y equals 0x00 and the Negative flag if bit 7 of Y is set.
 *
 * @return uint8_t Always returns 1 to indicate this instruction requires one additional cycle.
 */
uint8_t r6502::LDY()
{
    fetch();
    y = fetched;
    SetFlag(Z, y == 0x00);
    SetFlag(N, y & 0x80);
    return 1;
}

/**
 * @brief Performs a logical shift right on the current operand and stores the result.
 *
 * Shifts the operand right by one bit, transfers the least-significant bit into the Carry flag,
 * and updates the Zero and Negative (bit 7) flags based on the result. If the instruction uses
 * implied addressing, the shifted result is stored in the accumulator; otherwise the result is
 * written back to memory at the resolved address.
 *
 * @return uint8_t 0 (no additional cycles).
 */
uint8_t r6502::LSR()
{
    fetch();
    SetFlag(C, fetched & 0x0001);
    temp = fetched >> 1;
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    if (lookup[opcode].addrmode == &r6502::IMP)
        a = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}

/**
 * @brief Execute the NOP (no operation) instruction and report extra cycle usage.
 *
 * For a standard NOP this performs no state changes; certain undocumented/multi-byte NOP opcodes
 * require one additional cycle and are reported as such.
 *
 * @return uint8_t `1` if the current opcode is a multi-cycle NOP (one extra cycle), `0` otherwise.
 */
uint8_t r6502::NOP()
{

    switch (opcode)
    {
    case 0x1C:
    case 0x3C:
    case 0x5C:
    case 0x7C:
    case 0xDC:
    case 0xFC:
        return 1;
        break;
    }
    return 0;
}

/**
 * @brief Performs a bitwise OR between the accumulator and the fetched operand.
 *
 * Updates the accumulator with the result and sets the Zero and Negative flags
 * based on the new accumulator value.
 *
 * @return uint8_t Value `1`, indicating this instruction reports one extra cycle. 
 */
uint8_t r6502::ORA()
{
    fetch();
    a = a | fetched;
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 1;
}

/**
 * @brief Pushes the accumulator onto the stack.
 *
 * Pushes the A register to the stack page (0x0100 + SP) and decrements the stack pointer.
 *
 * @return uint8_t `0`.
 */
uint8_t r6502::PHA()
{
    write(0x0100 + stkp, a);
    stkp--;
    return 0;
}

/**
 * @brief Pushes the processor status register onto the stack and updates stack/status state.
 *
 * The status byte written to the stack has the Break and Unused flags set; after pushing,
 * the internal Break and Unused flags are cleared and the stack pointer is decremented.
 *
 * @return uint8_t Always 0.
 */
uint8_t r6502::PHP()
{
    write(0x0100 + stkp, status | B | U);
    SetFlag(B, 0);
    SetFlag(U, 0);
    stkp--;
    return 0;
}

/**
 * @brief Pulls a byte from the stack into the accumulator and updates flags.
 *
 * Increments the stack pointer, loads the pulled value into register A,
 * and updates the Zero and Negative status flags based on the loaded value.
 *
 * @return uint8_t 0 (no additional cycles).
 */
uint8_t r6502::PLA()
{
    stkp++;
    a = read(0x0100 + stkp);
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 0;
}

/**
 * @brief Restores the processor status register from the stack.
 *
 * Pops a byte from the CPU stack into the status register and ensures the
 * unused/always-set flag (U) is set.
 *
 * @return uint8_t Returns 0 to indicate no extra clock cycles. 
 */
uint8_t r6502::PLP()
{
    stkp++;
    status = read(0x0100 + stkp);
    SetFlag(U, 1);
    return 0;
}

/**
 * @brief Rotate the operand left through the carry flag and store the result.
 *
 * Rotates the operand left by one bit with the current Carry flag shifted into bit 0 and the former bit 7 shifted into Carry.
 * Updates the Carry, Zero, and Negative flags to reflect the result.
 * When the implied addressing mode is used, the rotated result is written to the accumulator; otherwise it is written to the resolved memory address.
 *
 * @return uint8_t 0 (no additional cycles).
 */
uint8_t r6502::ROL()
{
    fetch();
    temp = (uint16_t)(fetched << 1) | GetFlag(C);
    SetFlag(C, temp & 0xFF00);
    SetFlag(Z, (temp & 0x00FF) == 0x0000);
    SetFlag(N, temp & 0x0080);
    if (lookup[opcode].addrmode == &r6502::IMP)
        a = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}

/**
 * @brief Rotate right the fetched operand through the carry and store the result.
 *
 * Performs a right rotate through the carry flag on the current operand, updates the Carry, Zero, and Negative flags,
 * and writes the rotated result to the accumulator for implied addressing or to memory for other addressing modes.
 *
 * @return uint8_t `0` (operation-specific extra-cycle indicator; always 0 for this instruction).
 */
uint8_t r6502::ROR()
{
    fetch();
    temp = (uint16_t)(GetFlag(C) << 7) | (fetched >> 1);
    SetFlag(C, fetched & 0x01);
    SetFlag(Z, (temp & 0x00FF) == 0x00);
    SetFlag(N, temp & 0x0080);
    if (lookup[opcode].addrmode == &r6502::IMP)
        a = temp & 0x00FF;
    else
        write(addr_abs, temp & 0x00FF);
    return 0;
}

/**
 * @brief Restore processor status and program counter from the stack after an interrupt.
 *
 * Restores the status register and program counter from the CPU stack, clears the
 * Break (B) and Unused (U) flags in the restored status, and advances the stack pointer
 * to reflect the popped bytes.
 *
 * @return uint8_t Always returns 0.
 */
uint8_t r6502::RTI()
{
    stkp++;
    status = read(0x0100 + stkp);
    status &= ~B;
    status &= ~U;

    stkp++;
    pc = (uint16_t)read(0x0100 + stkp);
    stkp++;
    pc |= (uint16_t)read(0x0100 + stkp) << 8;
    return 0;
}

/**
 * @brief Restore the return address from the stack and resume execution at the next instruction.
 *
 * Pops a 16-bit return address from the CPU stack into the program counter and advances the
 * program counter to the instruction following the original call (JSR).
 *
 * @return uint8_t Always 0.
 */
uint8_t r6502::RTS()
{
    stkp++;
    pc = (uint16_t)read(0x0100 + stkp);
    stkp++;
    pc |= (uint16_t)read(0x0100 + stkp) << 8;

    pc++;
    return 0;
}

/**
 * @brief Set the processor Carry flag.
 *
 * Sets the processor Carry (C) status flag to true.
 *
 * @return uint8_t 0
 */
uint8_t r6502::SEC()
{
    SetFlag(C, true);
    return 0;
}

/**
 * @brief Enable decimal (BCD) arithmetic mode by setting the Decimal flag.
 *
 * Sets the processor status Decimal flag so subsequent arithmetic operations use BCD behavior.
 *
 * @return uint8_t `0` (no additional cycles).
 */
uint8_t r6502::SED()
{
    SetFlag(D, true);
    return 0;
}

/**
 * @brief Set the processor interrupt disable flag.
 *
 * Sets the I (interrupt disable) flag in the status register, preventing IRQs from being serviced.
 *
 * @return uint8_t Always 0.
 */
uint8_t r6502::SEI()
{
    SetFlag(I, true);
    return 0;
}

/**
 * @brief Store the accumulator into memory using the previously resolved address.
 *
 * Writes the CPU accumulator register (A) to the resolved address stored in `addr_abs`.
 *
 * @return uint8_t `0` indicating no additional clock cycles. 
 */
uint8_t r6502::STA()
{
    write(addr_abs, a);
    return 0;
}

/**
 * @brief Store the X register to memory at the computed effective address.
 *
 * Writes the value of the X register to the CPU bus at addr_abs.
 *
 * @return uint8_t 0 — indicates no additional cycles are required.
 */
uint8_t r6502::STX()
{
    write(addr_abs, x);
    return 0;
}

/**
 * @brief Store the Y register to the effective memory address.
 *
 * Writes the current value of the Y register into memory at the previously
 * resolved effective address (addr_abs).
 *
 * @return uint8_t 0 (no additional cycles).
 */
uint8_t r6502::STY()
{
    write(addr_abs, y);
    return 0;
}

/**
 * @brief Transfer the accumulator into the X register and update status flags.
 *
 * Sets X to the current value of A and updates the Zero and Negative flags
 * based on the new X value.
 *
 * @return uint8_t 0 (no additional cycles).
 */
uint8_t r6502::TAX()
{
    x = a;
    SetFlag(Z, x == 0x00);
    SetFlag(N, x & 0x80);
    return 0;
}

/**
 * @brief Transfer the accumulator into the Y register and update flags.
 *
 * Updates the Zero flag if Y is zero and the Negative flag from Y's bit 7.
 *
 * @return uint8_t `0` (no additional clock cycles).
 */
uint8_t r6502::TAY()
{
    y = a;
    SetFlag(Z, y == 0x00);
    SetFlag(N, y & 0x80);
    return 0;
}

/**
 * @brief Transfer the stack pointer into the X register and update processor flags.
 *
 * Sets the Zero flag if X becomes 0x00 and the Negative flag based on bit 7 of X.
 *
 * @return uint8_t 0 — no additional cycles. 
 */
uint8_t r6502::TSX()
{
    x = stkp;
    SetFlag(Z, x == 0x00);
    SetFlag(N, x & 0x80);
    return 0;
}

/**
 * @brief Transfer the X register into the accumulator and update flags.
 *
 * Sets the accumulator (A) to the value of the X register and updates the
 * zero and negative status flags based on the new accumulator value.
 *
 * @return uint8_t 0 indicating no additional CPU cycles required.
 */
uint8_t r6502::TXA()
{
    a = x;
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 0;
}

/**
 * @brief Transfer the X register value into the stack pointer.
 *
 * The X register is copied to the CPU stack pointer (STKP). This operation does not modify processor status flags.
 *
 * @return uint8_t `0`
 */
uint8_t r6502::TXS()
{
    stkp = x;
    return 0;
}

/**
 * @brief Transfer the Y register into the accumulator and update processor flags.
 *
 * Copies the value of the Y register into A, sets the Zero flag if A equals 0x00,
 * and sets the Negative flag based on the high bit of A.
 *
 * @return uint8_t Always returns 0 to indicate no additional cycle penalty.
 */
uint8_t r6502::TYA()
{
    a = y;
    SetFlag(Z, a == 0x00);
    SetFlag(N, a & 0x80);
    return 0;
}

/**
 * @brief Handler for undefined or unimplemented 6502 instructions.
 *
 * Acts as a placeholder for opcodes that have no implemented behavior.
 *
 * @return uint8_t Number of additional clock cycles consumed (always 0).
 */
uint8_t r6502::XXX()
{
    return 0;
}

/**
 * @brief Indicates whether the CPU has finished the current instruction.
 *
 * @return `true` if the pending cycle count is zero, `false` otherwise.
 */
bool r6502::complete()
{
    return cycles == 0;
}

/**
 * Produce a textual disassembly for the memory range between two addresses.
 *
 * Reads instruction bytes from the connected Bus and formats each instruction
 * into a human-readable line containing the address, mnemonic, operand bytes,
 * and an addressing-mode tag.
 *
 * @param nStart Starting address (inclusive) of the range to disassemble.
 * @param nStop  Ending address (inclusive) of the range to disassemble.
 * @return std::map<uint16_t, std::string> Map from instruction address to the
 *         formatted disassembly line (address, mnemonic, operands, and
 *         addressing-mode annotation).
 */
std::map<uint16_t, std::string> r6502::disassemble(uint16_t nStart, uint16_t nStop)
{
    uint32_t addr = nStart;
    uint8_t value = 0x00, lo = 0x00, hi = 0x00;
    std::map<uint16_t, std::string> mapLines;
    uint16_t line_addr = 0;

    auto hex = [](uint32_t n, uint8_t d)
    {
        std::string s(d, '0');
        for (int i = d - 1; i >= 0; i--, n >>= 4)
            s[i] = "0123456789ABCDEF"[n & 0xF];
        return s;
    };

    while (addr <= (uint32_t)nStop)
    {
        line_addr = addr;

        std::string sInst = "$" + hex(addr, 4) + ": ";

        uint8_t opcode = bus->read(addr, true);
        addr++;
        sInst += lookup[opcode].name + " ";

        if (lookup[opcode].addrmode == &r6502::IMP)
        {
            sInst += " {IMP}";
        }
        else if (lookup[opcode].addrmode == &r6502::IMM)
        {
            value = bus->read(addr, true);
            addr++;
            sInst += "#$" + hex(value, 2) + " {IMM}";
        }
        else if (lookup[opcode].addrmode == &r6502::ZP0)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + " {ZP0}";
        }
        else if (lookup[opcode].addrmode == &r6502::ZPX)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + ", X {ZPX}";
        }
        else if (lookup[opcode].addrmode == &r6502::ZPY)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = 0x00;
            sInst += "$" + hex(lo, 2) + ", Y {ZPY}";
        }
        else if (lookup[opcode].addrmode == &r6502::IZX)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = 0x00;
            sInst += "($" + hex(lo, 2) + ", X) {IZX}";
        }
        else if (lookup[opcode].addrmode == &r6502::IZY)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = 0x00;
            sInst += "($" + hex(lo, 2) + "), Y {IZY}";
        }
        else if (lookup[opcode].addrmode == &r6502::ABS)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = bus->read(addr, true);
            addr++;
            sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + " {ABS}";
        }
        else if (lookup[opcode].addrmode == &r6502::ABX)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = bus->read(addr, true);
            addr++;
            sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", X {ABX}";
        }
        else if (lookup[opcode].addrmode == &r6502::ABY)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = bus->read(addr, true);
            addr++;
            sInst += "$" + hex((uint16_t)(hi << 8) | lo, 4) + ", Y {ABY}";
        }
        else if (lookup[opcode].addrmode == &r6502::IND)
        {
            lo = bus->read(addr, true);
            addr++;
            hi = bus->read(addr, true);
            addr++;
            sInst += "($" + hex((uint16_t)(hi << 8) | lo, 4) + ") {IND}";
        }
        else if (lookup[opcode].addrmode == &r6502::REL)
        {
            value = bus->read(addr, true);
            addr++;
            sInst += "$" + hex(value, 2) + " [$" + hex(addr + value, 4) + "] {REL}";
        }

        mapLines[line_addr] = sInst;
    }

    return mapLines;
}