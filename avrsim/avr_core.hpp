
//          Copyright Danny Havenith Aug 2, 2011.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

/*
 * avr_core.hpp
 *
 *  Created on: Aug 2, 2011
 *      Author: danny
 */

#ifndef AVR_CORE_HPP_
#define AVR_CORE_HPP_
#include <vector>
#include <boost/cstdint.hpp>
#include "avr_instruction_set.hpp"

namespace avrsim
{

using namespace instructions;

struct flags_t
{
    bool I;
    bool T;
    bool H;
    bool S;
    bool V;
    bool N;
    bool Z;
    bool C;
};

struct avr_state
{
    avr_state( size_t ram_size)
        : flags(0),sp( ram_size -1), ram(ram_size)
    {}

    typedef boost::uint16_t instruction_t;
    typedef boost::uint8_t register_t;
    typedef boost::uint32_t pointer_t;

    flags_t flags;
    register_t r[32];
    boost::uint64_t clock_ticks;
    pointer_t pc;
    pointer_t sp;
    std::vector<register_t> ram;
};

struct avr_core_utils
{
    static boost::int16_t signed_( boost::uint8_t input)
    {
        using namespace boost;
        return static_cast<int16_t>(static_cast<int8_t>( input));
    }

    static void assign16(boost::uint8_t &reg, boost::int16_t value)
    {
        using namespace boost;
        reg = static_cast<uint8_t>( value >> 8);
        *((&reg)+1) = static_cast<uint8_t>( value);
    }
};


/*!
 * Class that implements the core instruction set of the AVR microcontroller.
 *
 * This class implements each instruction as an overload of the 'execute' member function.
 * Each overload takes the instruction tag as its first argument and then, depending on the
 * instruction, takes zero, one or two extra arguments.
 */
class avr_core : avr_state, avr_core_utils
{
public:

    typedef boost::int16_t  signed16;
    typedef boost::uint16_t unsigned16;
    typedef boost::int8_t   signed8;
    typedef boost::uint8_t  unsigned8;
    typedef boost::uint16_t operand;

    void execute( NOP)
    {
        // do nothing
    }

    void execute( MOVW, operand dest, operand source)
    {
        // move word
        r[2*dest] = r[2*source];
        r[2*dest + 1] = r[2*source + 1];
    }

    void set_flags_after_multiply( signed16 result)
    {
        flags.Z = result == 0;
        flags.C = result & 0x8000;
    }

    unsigned16 unsigned_multiply( operand dest, operand source)
    {
        unsigned16 result = r[dest+16] * r[source+16];
        set_flags_after_multiply( result);
        extra_clocktick();
        return result;
    }

    signed16 signed_multiply( operand dest, operand source)
    {
        signed16 result = signed_(r[dest+16]) * signed_(r[source+16]);
        set_flags_after_multiply( result);
        extra_clocktick();
        return result;
    }

    signed16 signed_unsigned_multiply( operand dest, operand source)
    {
        boost::int16_t result = signed_( r[dest+16]) * r[source+16];
        set_flags_after_multiply( result);
        extra_clocktick();
        return result;
    }

    void execute( MULS, operand dest, operand source)
    {
        // multiply signed
        assign16( r[0], signed_multiply( dest, source));
    }

    void execute( MULSU, operand dest, operand source)
    {
        // multiply signed with unsigned
        assign16( r[0], signed_unsigned_multiply(dest, source));
    }

    void execute( FMUL, operand dest, operand source)
    {
        assign16( r[0], unsigned_multiply( dest, source) << 1);
    }

    void execute( FMULS, operand dest, operand source)
    {
        assign16( r[0], signed_multiply( dest, source) << 1);
    }

    void execute( FMULSU, operand dest, operand source)
    {
        assign16( r[0], signed_unsigned_multiply( dest, source) << 1);
    }

    unsigned8 subtract( unsigned8 dest, unsigned8 source)
    {
        unsigned8 result = dest - source;
        // calculate carry and half-carry
        // we're doing the common subexpressin elimination, just
        // in case our compiler doesn't
        unsigned8 carries =
                    (~dest & source)
                |   (source & result)
                |   (result & ~dest);
        flags.C = carries & 0x80;
        flags.H = carries & 0x08;
        flags.V =
                    ((dest & ~source & ~result)
                |   (~dest & source & result))
                & 0x80;
        flags.Z = result == 0;
        flags.N = result & 0x80;

        return result;
    }

    void execute( CPC)
    {
    }

    void execute( SBC)
    {
    }

    void execute( ADD)
    {
    }

    void execute( CPSE)
    {
    }

    void execute( CP)
    {
    }

    void execute( SUB)
    {
    }

    void execute( ADC)
    {
    }

    void execute( AND)
    {
    }

    void execute( EOR)
    {
    }

    void execute( OR)
    {
    }

    void execute( MOV)
    {
    }

    void execute( CPI)
    {
    }

    void execute( SBCI)
    {
    }

    void execute( SUBI)
    {
    }

    void execute( ORI)
    {
    }

    void execute( ANDI)
    {
    }

    void execute( LDD_Y)
    {
    }

    void execute( LDD_Z)
    {
    }

    void execute( STD_Z)
    {
    }

    void execute( STD_Y)
    {
    }

    void execute( LDS)
    {
    }

    void execute( LD_Z_inc)
    {
    }

    void execute( LD_Z_dec)
    {
    }

    void execute( LD_Z_min)
    {
    }

    void execute( LPM_Z)
    {
    }

    void execute( LPM_Z_inc)
    {
    }

    void execute( ELPM_Z)
    {
    }

    void execute( ELPM_Z_inc)
    {
    }

    void execute( LD_Y_inc)
    {
    }

    void execute( LD_Y_dec)
    {
    }

    void execute( LD_Y_min)
    {
    }

    void execute( LD_Y_2)
    {
    }

    void execute( LD_X_inc)
    {
    }

    void execute( LD_X_dec)
    {
    }

    void execute( LD_X_min)
    {
    }

    void execute( LD_X)
    {
    }

    void execute( POP)
    {
    }

    void execute( STS)
    {
    }

    void execute( ST_Z_inc)
    {
    }

    void execute( ST_Z_dec)
    {
    }

    void execute( ST_Z_min)
    {
    }

    void execute( unk1)
    {
    }

    void execute( ST_Y_inc)
    {
    }

    void execute( ST_Y_dec)
    {
    }

    void execute( ST_Y_min)
    {
    }

    void execute( ST_Y)
    {
    }

    void execute( ST_X_inc)
    {
    }

    void execute( ST_X_dec)
    {
    }

    void execute( ST_X_min)
    {
    }

    void execute( ST_X)
    {
    }

    void execute( PUSH)
    {
    }

    void execute( COM)
    {
    }

    void execute( NEG)
    {
    }

    void execute( SWAP)
    {
    }

    void execute( INC)
    {
    }

    void execute( unk2)
    {
    }

    void execute( ASR)
    {
    }

    void execute( LSR)
    {
    }

    void execute( ROR)
    {
    }

    void execute( DEC)
    {
    }

    void execute( BSET)
    {
    }

    void execute( BCLR)
    {
    }

    void execute( IJMP)
    {
    }

    void execute( EIJMP)
    {
    }

    void execute( RET)
    {
    }

    void execute( ICALL)
    {
    }

    void execute( RETI)
    {
    }

    void execute( EICALL)
    {
    }

    void execute( SLEEP)
    {
    }

    void execute( BREAK)
    {
    }

    void execute( WDR)
    {
    }

    void execute( LPM)
    {
    }

    void execute( ELPM)
    {
    }

    void execute( SPM)
    {
    }

    void execute( ESPM)
    {
    }

    void execute( JMP)
    {
    }

    void execute( CALL)
    {
    }

    void execute( ADIW)
    {
    }

    void execute( SBIW)
    {
    }

    void execute( CBI)
    {
    }

    void execute( SBIC)
    {
    }

    void execute( SBI)
    {
    }

    void execute( SBIS)
    {
    }

    void execute( MUL)
    {
    }

    void execute( IN)
    {
    }

    void execute( OUT)
    {
    }

    void execute( RJMP)
    {
    }

    void execute( RCALL)
    {
    }

    void execute( LDI)
    {
    }

    void execute( BRBS)
    {
    }

    void execute( BRBC)
    {
    }

    void execute( BLD)
    {
    }

    void execute( BST)
    {
    }

    void execute( SBRC)
    {
    }

    void execute( SBRS)
    {
    }
private:
    void extra_clocktick()
    {
        ++clock_ticks;
    }
};

}
#endif /* AVR_CORE_HPP_ */
