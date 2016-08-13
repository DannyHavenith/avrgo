/*
 * instruction.hpp
 *
 *  Created on: Jul 28, 2011
 *      Author: danny
 */

#ifndef INSTRUCTION_HPP_
#define INSTRUCTION_HPP_
#include "mpl_with_large_vectors.h"

namespace avrsim
{
namespace mpl = boost::mpl;

/// The instruction template encodes the meaning of the individual bits of an
/// AVR instruction.
/// The instruction template maps directly onto a 16-element vector_c of ints, with position
/// 0 representing the least significant bit.
/// instruction bits represent the literal values 1 or 0, but can also specify certain types of
/// operands, encoded as numbers > 1.
template <
    int b15, int b14, int b13, int b12,
    int b11, int b10, int b9, int b8,
    int b7, int b6, int b5, int b4,
    int b3, int b2, int b1, int b0>
struct instruction : boost::mpl::vector_c<
        int,
        b0, b1, b2,  b3,  b4,  b5,  b6,  b7,
        b8, b9, b10, b11, b12, b13, b14, b15
        >
{
};

/// Boolean second order meta function that determines whether the given instruction has
/// the given value at bit position 'bit'.
template< int bit, int value>
struct has_at
{
    template< typename instruction>
    struct apply : boost::mpl::bool_<
        mpl::at_c< instruction, bit>::type::value == value
        >
    {};
};

/// Boolean metafunction that determines whether the given instruction has
/// an operand bit at bit position 'bit'.
template< int bit>
struct has_operand_at
{
    template< typename instruction>
    struct apply : mpl::greater<
        typename mpl::at_c< instruction, bit>::type,
        mpl::int_<1>
        >
    {};
};

template<typename instruction, typename sorted_operands>
struct operands : mpl::copy_if<
    sorted_operands,
    mpl::contains< instruction, mpl::_>,
    mpl::back_inserter< mpl::vector<> >
    >
{
};

namespace detail
{
    unsigned int opcode( unsigned int value)
    {
        return (value <= 1?1:0);
    }

    unsigned int if_opcode( unsigned int value)
    {
        return (value <= 1?value:0);
    }
}

template< typename Instruction>
unsigned int opcode_pattern_mask()
{
    using detail::opcode;
    unsigned int value = 0;
    value |= opcode( mpl::at_c<Instruction, 15>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 14>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 13>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 12>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 11>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 10>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 9>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 8>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 7>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 6>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 5>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 4>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 3>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 2>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 1>::type::value);
    value <<= 1;
    value |= opcode( mpl::at_c<Instruction, 0>::type::value);

    return value;
}

template< typename Instruction>
unsigned int opcode_pattern()
{
    unsigned int value = 0;
    using detail::if_opcode;

    value |= if_opcode( mpl::at_c<Instruction, 15>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 14>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 13>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 12>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 11>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 10>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 9>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 8>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 7>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 6>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 5>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 4>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 3>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 2>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 1>::type::value);
    value <<= 1;
    value |= if_opcode( mpl::at_c<Instruction, 0>::type::value);

    return value;
}

template< typename Instruction>
bool is_instruction( uint16_t value)
{
    return (value & opcode_pattern_mask<Instruction>()) == opcode_pattern<Instruction>();
}

}



#endif /* INSTRUCTION_HPP_ */
