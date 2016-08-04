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
        core::rom = newRom;
    }

    const avr_state &getState() const
    {
        return *this;
    }

    avr_state &getState()
    {
        return *this;
    }

    void run(int clocks = 1)
    {
        using decoder = typename find_decoder<core,instructions::list>::type;
        while (clocks--)
        {
            decoder::decode_and_execute(*this, core::fetch_instruction_word());
        }
    }

    int getPc() const
    {
        return static_cast<int>(core::pc);
    }


};

}




#endif /* AVRSIM_AVR_EMULATOR_HPP_ */
