
//          Copyright Danny Havenith Aug 2, 2011.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

/**
 * @file avr_core.hpp
 * @author Danny Havenith
 *
 * This file contains the implementation of the AVR instructions. These are implemented in the avr_core class.
 *
 */
#ifndef AVR_CORE_HPP_
#define AVR_CORE_HPP_
#include <vector>
#include <boost/cstdint.hpp>
#include <array>
#include <exception>
#include <tuple>

#include "avr_instruction_set.hpp"

namespace avrsim
{

using namespace instructions;

struct flags_t
{
    typedef boost::uint8_t flag_t;
    flag_t I;
    flag_t T;
    flag_t H;
    flag_t S;
    flag_t V;
    flag_t N;
    flag_t Z;
    flag_t C;

    bool operator==( const flags_t &other) const
    {
        return std::tie( I, T, H, S, V, N, Z, C) ==
                std::tie( other.I, other.T, other.H, other.S, other.V, other.N, other.Z, other.C);
    }
};

struct avr_state
{
    avr_state( size_t ram_size)
        : sp( ram_size -1), ram(ram_size), r(32)
    {}

    avr_state()
        :avr_state(512){}

    typedef boost::uint16_t instruction_t;
    typedef boost::uint8_t  register_t;
    typedef boost::uint32_t pointer_t;
    typedef std::vector<instruction_t> rom_t;
    typedef std::vector<register_t>    ram_t;


    uint8_t         eind{}; // only available on 22-bit pc cores
    bool            is_sleeping{};
    flags_t         flags{};
    std::vector<register_t>      r;
    unsigned int    clock_ticks{};
    pointer_t       pc{};
    pointer_t       sp{};
    ram_t           ram;
    rom_t           rom;
};

struct avr_core_utils
{
    static boost::int16_t signed_( boost::uint8_t input)
    {
        using namespace boost;
        return static_cast<int16_t>(static_cast<int8_t>( input));
    }
};


/**
 * Class that implements the core instruction set of the AVR microcontroller.
 *
 * This class implements each instruction as an overload of the 'execute' member function.
 * Each overload takes the instruction tag as its first argument and then, depending on the
 * instruction, takes zero, one or two extra arguments.
 */
class avr_core : public avr_state, avr_core_utils
{
public:

    using avr_state::avr_state; // delegate constructors to the avr_state
    typedef boost::int16_t  signed16;
    typedef boost::uint16_t unsigned16;
    typedef boost::int8_t   signed8;
    typedef boost::uint8_t  unsigned8;
    typedef boost::uint16_t operand;
    static constexpr int register_X = 26;
    static constexpr int register_Y = 28;
    static constexpr int register_Z = 30;
    static constexpr int io_offset = 0x20;
    static constexpr bool is_xmega = false;
    static constexpr bool is_22bit_pc = false;
    static constexpr int rampz_io_address = 0x3b;

    // the following functions should become part of
    // a ram-model aspect.

    pointer_t get_rampd_offset() const
    {
        return 0;
    }

    // fetch the contents of the RAMPZ register in io space.
    pointer_t get_rampz_offset() const
    {
        return get_io( rampz_io_address);
    }

    // todo: implement
    pointer_t get_rampx_offset() const
    {
        return 0;
    }

    // todo: implement
    pointer_t get_rampy_offset() const
    {
        return 0;
    }

    register_t &get_io_address( uint16_t address)
    {
        return ram[address + io_offset];
    }

    const register_t &get_io_address( uint16_t address) const
    {
        return ram[address + io_offset];
    }

    void set_io( operand address, register_t value)
    {
        get_io_address( address) = value;
    }

    register_t get_io( operand address) const
    {
        return get_io_address( address);
    }

    void assign16( operand reg, boost::int16_t value)
    {
        r[reg] = value;
        r[reg+1] = value >> 8;
    }

    unsigned16 get16( operand reg)
    {
        return r[reg] + (static_cast<unsigned16>(r[reg+1]) << 8);
    }

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

    void set_z_and_n( register_t result)
    {
        flags.Z = result == 0;
        flags.N = result & 0x80;
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
        assign16( 0, signed_multiply( dest, source));
    }

    void execute( MULSU, operand dest, operand source)
    {
        // multiply signed with unsigned
        assign16( 0, signed_unsigned_multiply(dest, source));
    }

    void execute( FMUL, operand dest, operand source)
    {
        assign16( 0, unsigned_multiply( dest, source) << 1);
    }

    void execute( FMULS, operand dest, operand source)
    {
        assign16( 0, signed_multiply( dest, source) << 1);
    }

    void execute( FMULSU, operand dest, operand source)
    {
        assign16( 0, signed_unsigned_multiply( dest, source) << 1);
    }

    unsigned8 subtract( unsigned16 dest, unsigned16 source)
    {
        unsigned8 result = dest - source;

        // calculate carry and half-carry
        // we're doing the common subexpression elimination, just
        // in case our compiler doesn't
        unsigned16 carries =
                    (~dest & source)
                |   (source & result)
                |   (result & ~dest);
        flags.C = carries & 0x80;
        flags.H = carries & 0x08;
        flags.V =  (( dest & ~source & ~result)
                |   (~dest &  source &  result))
                & 0x80;
        flags.S = flags.V ^ flags.N;
        set_z_and_n( result);

        return result;
    }

    signed16 subtract16( signed16 left, signed16 right)
    {
        signed16 result = left - right;
        flags.C = (left >= 0) and (result < 0);
        flags.N = result < 0;
        flags.V = (left < 0) and (result >= 0); // check
        flags.Z = result == 0;
        flags.S = !flags.N != !flags.V;
        return result;
    }

    signed16 add16( signed16 left, signed16 right)
    {
        signed16 result = left + right;
        flags.C = result >= 0 and left < 0;
        flags.Z = result == 0;
        flags.N = result < 0;
        flags.V = result < 0 and left >= 0;
        flags.S = !flags.N != !flags.V;
        return result;
    }

    unsigned8 add( unsigned16 dest, unsigned16 source)
    {
        unsigned8 result = dest + source;
        unsigned16 carries =
                    (source & dest)
                |   (source & ~result)
                |   (dest & ~result);

        set_z_and_n( result);
        flags.C = carries & 0x80;
        flags.H = carries & 0x08;
        flags.V =   (dest & source & ~result)
                |   (~dest & ~source & result);
        flags.S = flags.V ^ flags.N;

        return result;
    }

    void execute( CPC, operand dest, operand source)
    {
        // ignore subtract results, change flags only
        auto previousZ = flags.Z;
        subtract( r[dest], static_cast<unsigned16>(r[source]) + (flags.C?1:0));
        flags.Z = flags.Z and previousZ;
    }

    void execute( SBC, operand dest, operand source)
    {
        auto previousZ = flags.Z;
        r[dest] = subtract( r[dest], static_cast<unsigned16>(r[source]) + (flags.C?1:0));
        flags.Z = flags.Z and previousZ;
    }

    void execute( ADD, operand dest, operand source)
    {
        r[dest] = add( r[dest], r[source]);
    }

    void skip()
    {
        auto instruction = fetch_instruction_word();
        // todo skip 2-word instructions
    }

    void execute( CPSE, operand dest, operand source)
    {
        if (r[dest] == r[source])
        {
            skip();
        }
    }

    void execute( CP, operand dest, operand source)
    {
        subtract( r[dest], r[source]);
    }

    void execute( SUB, operand dest, operand source)
    {
        r[dest] = subtract( r[dest], r[source]);
    }

    void execute( ADC, operand dest, operand source)
    {
        r[dest] = add( r[dest], flags.C?((unsigned16)(r[source]+1)):r[source]);
    }

    unsigned8 set_logical_flags( unsigned8 result)
    {
        set_z_and_n( result);
        flags.V = 0;
        flags.S = flags.N;

        return result;
    }

    void execute( AND, operand dest, operand source)
    {
        r[dest] = set_logical_flags(r[dest] & r[source]);
    }

    void execute( EOR, operand dest, operand source)
    {
        r[dest] = set_logical_flags( r[dest] ^ r[source]);
    }

    void execute( OR, operand dest, operand source)
    {
        r[dest] = set_logical_flags( r[dest] | r[source]);
    }

    void execute( MOV, operand dest, operand source)
    {
        r[dest] = r[source];
    }

    void execute( CPI, operand dest, operand constant)
    {
        subtract( r[dest + 16], constant);
    }

    void execute( SBCI, operand dest, operand constant)
    {
        r[dest+16] = subtract( r[dest+16], flags.C?((unsigned16)(constant+1)):constant);
    }

    void execute( SUBI, operand dest, operand constant)
    {
        r[dest+16] = subtract( r[dest+16], constant);
    }

    void execute( ORI, operand dest, operand constant)
    {
        r[dest] = set_logical_flags( r[dest] | constant);
    }

    void execute( ANDI, operand dest, operand constant)
    {
        r[dest] = set_logical_flags( r[dest] & constant);
    }

    void execute( LDD_Y, operand dest, operand offset)
    {
        r[dest] = ram[ get16( register_Y) + offset];
    }

    void execute( LDD_Z, operand dest, operand offset)
    {
        r[dest] = ram[ get16( register_Z) + offset];
    }

    void execute( STD_Z, operand source, operand offset)
    {
        ram[ get16( register_Z) + get_rampz_offset() + offset] = r[source];
    }

    void execute( STD_Y, operand source, operand offset)
    {
        ram[ get16( register_Y) + offset] = r[source];
    }

    unsigned16 fetch_instruction_word()
    {
        return rom[pc++];
    }

    void execute( LDS, operand destination)
    {
        r[destination] = ram[fetch_instruction_word()];
    }

    void increase16( operand reg)
    {
        assign16( reg, get16( reg) + 1);
    }

    void decrease16( operand reg)
    {
        assign16( reg, get16( reg) - 1);
    }

    void execute( LD_Z_inc, operand dest)
    {
        execute( LDD_Z(), dest, 0);
        increase16( register_Z);
    }

    void execute( LD_Z_dec, operand dest)
    {
        decrease16( register_Z);
        execute( LDD_Z(), dest, 0);
    }

    void execute( LD_Z_min, operand dest)
    {
        decrease16( register_Z);
        execute( LDD_Z(), dest, 0);
        increase16( register_Z);
    }

    unsigned8 fetch_rom_byte( pointer_t address) const
    {
        // ROM byte addressing emulates little endian
        // (low byte is address.0 == 0)
        return (address& 0x01)?
                        (rom[address/2] >> 8)
                    :   (rom[address/2]);
    }

    void execute( LPM_Z, operand dest)
    {
        r[dest] = fetch_rom_byte( get16( register_Z));
        extra_clockticks(2);
    }

    void execute( LPM_Z_inc, operand dest)
    {
        execute( LPM_Z(), dest);
        increase16( register_Z);
    }

    void execute( ELPM_Z, operand dest)
    {
        r[dest] = fetch_rom_byte( get16( register_Z) + get_rampz_offset());
        extra_clockticks(2);
    }

    void execute( ELPM_Z_inc, operand dest)
    {
        execute( ELPM_Z(), dest);
        increase16( register_Z);
    }

    void execute( LD_Y_inc, operand dest)
    {
        execute( LDD_Y(), dest, 0);
        increase16( register_Y);
    }

    void execute( LD_Y_dec, operand dest)
    {
        decrease16( register_Y);
        execute( LDD_Y(), dest, 0);
    }

    void execute( LD_X, operand dest)
    {
        r[dest] = ram[ get16( register_X)];
    }

    void execute( LD_X_inc, operand dest)
    {
        execute( LD_X(), dest);
        increase16( register_X);
    }

    void execute( LD_X_dec, operand dest)
    {
        decrease16( register_X);
        execute( LD_X(), dest);
    }

    void execute( POP, operand dest)
    {
        r[dest] = ram[++sp];
    }

    void execute( STS, operand source)
    {
        execute( STS_direct(), source, fetch_instruction_word());
    }

    void execute( STS_direct, operand source, operand address)
    {
        ram[ address + get_rampd_offset()] = r[source];
    }

    void execute( ST_Z_inc, operand source)
    {
        execute( STD_Z(), source, 0);
        increase16( register_Z);
    }

    void execute( ST_Z_dec, operand source)
    {
        decrease16( register_Z);
        execute( STD_Z(), source, 0);
    }

//    void execute( ST_Z_min, operand source)
//    {
//        decrease16( register_Z);
//        execute( STD_Z(), source, 0);
//        increase16( register_Z);
//    }

    void execute( unk1)
    {
    }

    void execute( ST_Y_inc, operand source)
    {
        execute( STD_Y(), source, 0);
        increase16( register_Y);
    }

    void execute( ST_Y_dec, operand source)
    {
        decrease16(register_Y);
        execute( STD_Y(), source, 0);
    }

//    void execute( ST_Y_min, operand source)
//    {
//        decrease16(register_Y);
//        execute( STD_Y(), source, 0);
//        increase16( register_Y);
//    }

    void execute( ST_Y, operand source)
    {
        execute( STD_Y(), source, 0);
    }

    void execute( ST_X_inc, operand source)
    {
        execute( ST_X(), source);
        increase16( register_X);
    }

    void execute( ST_X, operand source)
    {
        ram[ get16( register_X) + get_rampx_offset()] = r[source];
    }

    void execute( ST_X_dec, operand source)
    {
        decrease16( register_X);
        execute( ST_X(), source);
    }

//    void execute( ST_X_min, operand source)
//    {
//        decrease16( register_X);
//        execute( ST_X(), source);
//        increase16( register_X);
//    }


    void execute( PUSH, operand dest)
    {
        ram[ sp--] = r[dest];
    }

    void execute( COM, operand dest)
    {
        r[dest] = ~r[dest];
        flags.V = 0;
        flags.C = 0x80;
        set_z_and_n( r[dest]);
    }

    void execute( NEG, operand dest)
    {
        register_t result = -r[dest];
        set_z_and_n( result);
        flags.V = r[dest] == 0x80;
        flags.C = result != 0;
        flags.H = (result | r[dest]) & 0x80;
        r[dest] = result;
    }

    void execute( SWAP, operand dest)
    {
        r[dest] = (r[dest] >> 4) | (r[dest] << 4);
    }

    void execute( INC, operand dest)
    {
        ++r[dest];
        flags.V = r[dest] == 0x80;
        set_z_and_n( r[dest]);
    }

    void execute( unk2)
    {
    }

    void execute( ASR, operand dest)
    {
        flags.C = (r[dest] & 0x01) << 7;
        r[dest] = ((signed8)r[dest])/2;
        set_z_and_n( r[dest]);
        flags.V = flags.N ^ flags.C;
    }

    void execute( LSR, operand dest)
    {
        flags.C = (r[dest] & 0x01) << 7;
        r[dest] >>= 1;
        set_z_and_n( r[dest]);
        flags.V = flags.N ^ flags.C;
    }

    void execute( ROR, operand dest)
    {

        unsigned8 result = r[dest] >> 1;
        if (flags.C)
        {
            result |= 0x80;
        }
        flags.C = (r[dest] & 0x01) << 7;
        set_z_and_n( result);
        flags.V = flags.N ^ flags.C;
        r[dest] = result;
    }

    void execute( DEC, operand dest)
    {
        --r[dest];
        set_z_and_n( r[dest]);
        flags.V = r[dest] == 0x7f;
    }

    flags_t::flag_t &number_to_flag( operand bit)
    {
        switch (bit)
        {
        case 0: return flags.C;
        case 1: return flags.Z;
        case 2: return flags.N;
        case 3: return flags.V;
        case 4: return flags.S;
        case 5: return flags.H;
        case 6: return flags.T;
        default: return flags.I;
        }

    }

    void execute( BSET, operand bit)
    {
        number_to_flag( bit) = 0x80;
    }

    void execute( BCLR, operand bit)
    {
        number_to_flag( bit) = 0;
    }

    void execute( IJMP)
    {
        pc = get16( register_Z);
        extra_clocktick();
    }

    /// TODO
    pointer_t get_eind_offset()
    {
        return 0;
    }

    // 16-bit addressing scheme.
    pointer_t pop_address()
    {
        pointer_t result = ram[sp+1] + (ram[sp+2] << 8);
        sp += 2;
        extra_clocktick();
        return result;
    }

    void push_address( pointer_t address)
    {
        if (sp > 1)
        {
            ram[sp] = address >> 8;
            ram[sp-1] = address;
            sp -= 2;
            extra_clockticks(2);
        }
    }

    void execute( EIJMP)
    {
        pc = get16( register_Z) + get_eind_offset();
        extra_clocktick();
    }

    void execute( RET)
    {
        pc = pop_address();
    }

    void execute( ICALL)
    {
        push_address( pc);
        pc = get16( register_Z);
    }

    void execute( RETI)
    {
        pc = pop_address();
        if (!is_xmega) flags.I = 1;
        extra_clockticks( is_22bit_pc?4:3);
    }

    void execute( EICALL)
    {
        push_address(pc);
        pc = (static_cast<pointer_t>(eind) << 16) + get16( register_Z );
        extra_clockticks( is_xmega?2:3);
    }

    bool is_sleep_enabled()
    {
        // todo: implement MCUCR
        return true;
    }

    void execute( SLEEP)
    {
        if (is_sleep_enabled())
        {
            is_sleeping = true;
        }
    }

    class break_exception : std::exception
    {
    public:
        break_exception( pointer_t pc, uint64_t clock)
        :pc{pc},clock{clock}
        {}

        virtual const char *what() const noexcept
        {
            return "break";
        }
        pointer_t pc;
        uint64_t  clock;

    };

    void execute( BREAK)
    {
        throw break_exception{pc-1, clock_ticks};
    }

    void execute( WDR)
    {
        // todo: implement
    }

    void execute( LPM)
    {
        execute( LPM_Z{}, 0);
    }

    void execute( ELPM)
    {
        execute( ELPM_Z{}, 0);
    }

    void execute( SPM)
    {
        // todo: implement
    }

    void execute( ESPM)
    {
        // todo: implement
    }

    int16_t sign_extend12( operand input)
    {
        return (( static_cast<int16_t>(input) + 0x0800) & 0x0FFF) - 0x0800;
    }

    int8_t sign_extend7( operand input)
    {
        return ((static_cast<int8_t>(input) + 0x40) & 0x7F) - 0x40;
    }

    // not used yet. should replace the two functions above.
    template< typename Integer, int sourceBits>
    Integer sign_extend( Integer source)
    {
        constexpr Integer upperBit = (1 << (sourceBits - 1));
        return ((source ^ upperBit) & ((1<< sourceBits)-1)) - upperBit;
    }

    void execute( JMP, operand upper_bits)
    {
        pc = fetch_instruction_word() + (static_cast<pointer_t>( upper_bits) << 16);
        extra_clockticks(2);
    }

    void execute( CALL, operand upper_bits)
    {
        push_address( pc);
        pc = fetch_instruction_word() + (static_cast<pointer_t>( upper_bits) << 16);
        extra_clockticks(3);
    }

    void execute( ADIW, operand reg_pair, operand value)
    {
        assign16( 2*reg_pair + 24, add16( r[2*reg_pair + 24], value));
        extra_clocktick();
    }


    void execute( SBIW, operand reg_pair, operand value)
    {
        assign16( 2*reg_pair+24, subtract16( get16(2*reg_pair+24), value));
        extra_clocktick();
    }

    void execute( CBI, operand ioaddr, operand bit)
    {
        set_io( ioaddr, get_io( ioaddr) & ~ (1<<bit));
        extra_clocktick();
    }

    void execute( SBIC, operand ioaddr, operand bit)
    {
        if (!(get_io( ioaddr) & (1<<bit)))
        {
            skip();
        }
    }

    void execute( SBI, operand ioaddr, operand bit)
    {
        set_io( ioaddr, get_io( ioaddr) | (1<<bit));
    }

    void execute( SBIS, operand ioaddr, operand bit)
    {
        if (get_io( ioaddr) & (1<<bit))
        {
            skip();
        }
    }

    void execute( MUL, operand dest, operand source)
    {
       assign16( 0, static_cast<uint16_t>( r[dest]) * r[source]);
    }

    void execute( IN, operand dest, operand io_address)
    {
        r[dest] = get_io( io_address);
    }

    void execute( OUT, operand ioaddr, operand source)
    {
        set_io( ioaddr, r[source]);
    }

    void execute( RJMP, operand offset)
    {
        pc += sign_extend12( offset);
        extra_clocktick();
    }

    void execute( RCALL, operand offset)
    {
        push_address( pc);
        pc += sign_extend12( offset);
    }

    void execute( LDI, operand dest, operand constant)
    {
        r[0x10 | dest] = constant;
    }

    void execute( BRBS, operand offset, operand bit)
    {
        if (number_to_flag( bit))
        {
            pc += sign_extend7( offset);
            extra_clocktick();
        }
    }

    void execute( BRBC, operand offset, operand bit)
    {
        if (!number_to_flag(bit))
        {
            pc += sign_extend7( offset);
            extra_clocktick();
        }
    }

    void execute( BLD, operand, operand)
    {
    }

    void execute( BST, operand, operand)
    {
    }

    void execute( SBRC, operand, operand)
    {
    }

    void execute( SBRS, operand, operand)
    {
    }
private:
    void extra_clocktick()
    {
        ++clock_ticks;
    }

    void extra_clockticks( int ticks)
    {
        clock_ticks += ticks;
    }
};

}
#endif /* AVR_CORE_HPP_ */
