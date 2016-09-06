//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#ifndef AVRSIM_PRECOMPILED_EMULATOR_HPP_
#define AVRSIM_PRECOMPILED_EMULATOR_HPP_
#include <algorithm>

#include "avr_core.hpp"
#include "precompiler.hpp"

namespace avrsim {
    template< typename Core = avr_core>
    class precompiled_emulator : private Core
    {
    public:
        using typename Core::register_t;
        using typename Core::pointer_t;
        using typename Core::break_exception;
        using Core::pc;
        using Core::rom;
        using Core::ram;
        using decoder = typename find_decoder<Core,instructions::list>::type;

        // inherit constructors from Core.
        using Core::Core;


        bool run( unsigned int clocks = 1)
        {
            const auto clock_end = Core::clock_ticks + clocks;
            bool result = false;

            // the first instruction will be decoded from the original
            // rom, just in case we have a breakpoint in the compiled rom
            execute_instruction( precompile<Core>(Core::fetch_instruction_word()));

            try
            {
                while (Core::clock_ticks < clock_end)
                {
                    execute_instruction( precompiled_rom[pc++]);
                }
            }
            catch ( const break_exception &e)
            {
                // break points are implemented by exceptions thrown from the core.
                --pc; // set the program counter back to the location of the break
                result = true;
            }
            return result;
        }

        void setBreakpoint( pointer_t address)
        {
            constexpr typename Core::instruction_t break_instruction = 0b1001010110011000;
            precompiled_rom[address] = precompile<Core>( break_instruction);
        }

        void clearBreakpoint( pointer_t address)
        {
            precompiled_rom[address] = precompile<Core>( rom[address]);
        }

        // todo: move some of these member functions into a generic
        // emulator.

        /**
         * Take a buch of instruction words and fill the rom with it.
         */
        void fillRom( const typename Core::rom_t &newRom)
        {
            rom = newRom;
            precompile_rom();
        }

        void precompile_rom()
        {
            precompiled_rom.resize( rom.size());
            std::transform(
                    rom.begin(), rom.end(),
                    precompiled_rom.begin(),
                    &precompile<Core>);
        }

        /**
         * reset flags, ram, etc. to a known state (zeros).
         */
        void initialize()
        {
            static_cast<avr_state &>(*this) = avr_state{Core::ram.size()};
        }

        const avr_state getState() const
        {
            return *this;
        }

        register_t getRam( pointer_t address) const
        {
            return Core::ram(address);
        }

        void setRam( pointer_t address, register_t value)
        {
            Core::ram(address) = value;
        }


        register_t getRegister( int address) const
        {
            return Core::reg(address);
        }

        void setRegister( int address, register_t val)
        {
            Core::assign8( address, val);
        }

        int getPc() const
        {
            return static_cast<int>(Core::pc);
        }

        void setPc( pointer_t pc)
        {
            Core::pc = pc;
        }

        size_t getClockTicks() const
        {
            return Core::clock_ticks;
        }

    private:
        using precompiled = precompiled_instruction<Core>;
        using precompiled_rom_t = std::vector< precompiled>;
        precompiled_rom_t precompiled_rom;

        void execute_instruction( const precompiled &i)
        {
            i.f(  *this, i.arg1, i.arg2);
            ++Core::clock_ticks;
        }


    };
}

#endif /* AVRSIM_PRECOMPILED_EMULATOR_HPP_ */
