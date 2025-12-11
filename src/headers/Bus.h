#pragma once
#include <cstdint>
#include "r6502.h"
#include <array>

/**
 * Bus: simple 16-bit address space with attached 6502 CPU and 64KB RAM.
 */

/**
 * r6502 CPU instance attached to the bus.
 */

/**
 * 64 KB addressable RAM backing the bus.
 */

/**
 * Construct a Bus and initialize its CPU and memory.
 */

/**
 * Destroy the Bus and release any resources it holds.
 */

/**
 * Write a byte to the bus at the specified 16-bit address.
 * @param addr 16-bit address to write to.
 * @param data Byte value to write to the given address.
 */

/**
 * Read a byte from the bus at the specified 16-bit address.
 * @param addr 16-bit address to read from.
 * @param bReadOnly If `true`, perform the read without causing side effects that a normal read might trigger.
 * @returns Byte value read from the given address.
 */
class Bus
{
public:
    Bus();
    ~Bus();

public:
    r6502 cpu;
    std::array<uint8_t, 64 * 1024> ram;

public:
    void write(uint16_t addr, uint8_t data);
    uint8_t read(uint16_t addr, bool bReadOnly = false);
};