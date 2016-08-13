/*
 * avr_emulator.hpp
 *
 *  Created on: Aug 1, 2016
 *      Author: danny
 */

#ifndef AVRSIM_AVR_EMULATOR_HPP_
#define AVRSIM_AVR_EMULATOR_HPP_
#include <cstdint>
#include "decoder.hpp"
#include "avr_core.hpp"
#include "avr_instruction_set.hpp"

namespace avrsim {

template< typename core = avr_core>
class avr_emulator : private core
{
public:

    using typename core::register_t;
    using typename core::pointer_t;
    using typename core::instruction_t;
    using typename core::break_exception;

    using original_instruction_storage_t
            = std::map< pointer_t, instruction_t>;
    using decoder = typename find_decoder<core,instructions::list>::type;

    using core::pc;
    using core::rom;
    using core::ram;

    avr_emulator( size_t ram_size)
    : core{ram_size}
    {

    }

    /**
     * reset flags, ram, etc. to a known state (zeros).
     */
    void initialize()
    {
        static_cast<avr_state &>(*this) = avr_state{core::ram.size()};
    }

    /**
     * Take a buch of instruction words and fill the rom with it.
     */
    void fillRom( const typename core::rom_t &newRom)
    {
        rom = newRom;
    }

    const avr_state getState() const
    {
        return *this;
    }

    static constexpr instruction_t break_instruction = 0b1001010110011000;

    void execute_instruction( instruction_t instruction)
    {
        decoder::decode_and_execute(*this, instruction);
        ++core::clock_ticks;
    }

    /**
     * Run the emulator for at least the given amount of clock ticks and at
     * least one instruction.
     *
     * This will run individual instructions, starting with the instruction at
     * the current program counter until the given amount of
     * clock ticks has passed.
     *
     * This will always at least run one instruction and if there is a break-
     * point at that first location, it will be ignored.
     *
     * This function will finish the instruction that is running when the clock
     * tick passes, so depending on the instruction it may run for 1 to 3
     * clock ticks more.
     *
     * If this function returns because of hitting a break point, it will
     * return true, otherwise it will return false.
     */
    bool run(unsigned int clocks = 1)
    {
        const auto clock_end = core::clock_ticks + clocks;
        bool result = false;

        // run the first instruction. If it is a break instruction, find
        // the original instruction and run that one.
        instruction_t first_instruction
            = core::fetch_instruction_word();

        if (first_instruction == break_instruction)
        {
            // this will do the right thing if there was no
            // original instruction: it returns 0, which happens to be
            // NOP.
            first_instruction = original_instructions[pc - 1];
        }

        execute_instruction( first_instruction);

        try
        {
            while (core::clock_ticks < clock_end)
            {
                execute_instruction( core::fetch_instruction_word());
            }
        }
        catch( const break_exception &e)
        {
            // break points are implemented by exceptions thrown from the core.
            --pc; // set the program counter back to the location of the break
            result = true;
        }

        return result;
    }

    register_t getRam( pointer_t address) const
    {
        return core::ram[address];
    }

    void setRam( pointer_t address, register_t value)
    {
        core::ram[address] = value;
    }


    register_t getRegister( int reg) const
    {
        return core::r[reg];
    }

    void setRegister( int reg, register_t val)
    {
        core::r[reg] = val;
    }

    void setBreakpoint( pointer_t address)
    {
        if (rom[address] != break_instruction)
        {
            original_instructions[address] = rom[address];
            rom[address] = break_instruction;
        }
    }

    void clearBreakpoint( pointer_t address)
    {
        if (rom[address] == break_instruction)
        {
            // this will move a NOP instruction if the instruction was not
            // in original_instructions;
            rom[address] = original_instructions[address];
        }
    }

    int getPc() const
    {
        return static_cast<int>(core::pc);
    }

    void setPc( pointer_t pc)
    {
        core::pc = pc;
    }

private:
    /// if a breakpoint is inserted in memory, we'll store the
    /// original instruction in this map.
    original_instruction_storage_t original_instructions;

};

}




#endif /* AVRSIM_AVR_EMULATOR_HPP_ */
