/*
 * unpack_operand.hpp
 *
 *  Created on: Jul 31, 2011
 *      Author: danny
 */

/**
 * This file contains meta functions that create code to extract an operand from an instruction word.
 * AVR operands can be quite scattered around in the instruction word, so sometimes several bits must be
 * gathered and compacted into a single integer value.
 *
 * The meta-functions create a list of unpacking instructions. Each instruction is either a 'shift' (of
 * the original instruction word), or a 'mask', which means masking a number of bits from the shifted
 * instruction word and transferring them to an operand accumulator.
 *
 * See the unpack-function for more information on how operand bits are extracted from instruction words.
 */

#ifndef UNPACK_OPERAND_HPP_
#define UNPACK_OPERAND_HPP_
#include <utility> // for std::pair and std::make_pair
#include <boost/mpl/vector.hpp>
#include <boost/mpl/int.hpp>
#include <boost/mpl/pop_front.hpp>
#include <boost/mpl/push_front.hpp>
#include <boost/mpl/front.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/back_inserter.hpp>
#include <boost/mpl/copy.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/cstdint.hpp>

namespace avrsim {

/**
 * This namespace contains types that represent unpacking instructions.
 * Unpacking an operand consists of a series of shifts and applying masks.
 */
namespace unpacking {

/// unpack state consists of two uint16 values.
/// the first contains the instruction word, which will be shifted during unpacking
/// the second contains the operand accumulator, which will receive the operand bits.
typedef std::pair<uint16_t,uint16_t> unpack_state;

/// This type represents a sequence of bits in an instruction word.
/// The boolean template argument use_bits is true if it is a sequence of bits that we want.
/// This type is used to determine which bits should be combined to get a specific operand from
/// an instruction word.
template< int bitcount, bool use_bits>
struct bits {};


/// This class is given as a lambda function to the mpl::fold meta function. Given an
/// operand code, when fed instruction code bits one-by-one, this class will build up state
/// which consists of a list of alternating bits< a, false>, bits<b, true>, bits<c, false>, etc...
/// These bits<x, b> elements represent sequences of x bits that are either part of the given operand (b=true)
/// or not (b=false).
/// the list of bits<x,b> instances will be in reverse order (describing the bits "from right to left").
///
/// For example, for an operand code q, if fed the bit-sequence 0,1,q,q,q,q,q,1,0,q,q the final state will consist of
/// bits<2,true>, bits<2, false>, bits<5, true>, bits<2, false>
///
template< int operand_code>
struct run_length_encoder
{
    /**
     * The state to maintain while folding the instruction mask.
     * current_list is the current list of unpacking instructions
     * current_instruction is the current instruction, which could be expanded.
     * offset is the offset in the operand accumulator where the next operand bits should be copied to.
     */
    template<
        typename current_list = boost::mpl::vector<>,
        typename current_bits = bits< 0, false>
        >
    struct state
    {
        typedef current_list bits_list;
        typedef current_bits bits;
    };

    /**
     * The apply meta function accepts a current state and a type representing the next bit. It will return the
     * new state.
     */
    template< typename state, typename next_bit, typename enable = void>
    struct apply { };

    /**
     * If we're in the masking state and we encounter a bit that is not part of the operand, we finish
     * our masking instruction (push it to the list of instructions) and start a shift instruction.
     */
    template< typename list, int bit_count, typename next_bit>
    struct apply<
        state< list, bits<bit_count, true> >,
        next_bit,
        typename boost::enable_if_c<next_bit::value != operand_code>::type >
    {
        typedef state<
                typename boost::mpl::push_front< list, bits<bit_count, true> >::type,
                bits<1, false>
                > type;
    };

    /**
     * If we're in the masking state and we encounter another operand bit, we extend the mask to include this bit.
     */
    template< typename list, int bit_count, typename next_bit>
    struct apply<
        state< list, bits<bit_count, true> >,
        next_bit,
        typename boost::enable_if_c< next_bit::value == operand_code >::type
    >
    {
        typedef state<
                list,
                bits<bit_count+1, true>
        > type;
    };

    /**
     * If we're in the shifting state and we encounter another bit that is not part of the operand,
     * we increase the shift amount.
     */
    template< typename list, int bit_count, typename next_bit>
    struct apply<
        state< list, bits< bit_count, false> >,
        next_bit,
        typename boost::enable_if_c<
            next_bit::value != operand_code
            >::type
    >
    {
        typedef state<
                list,
                bits< bit_count + 1, false>
        > type;
    };

    /**
     * If we encounter an operand bit while we where in the shifting state, we stop shifting and start
     * a masking instruction.
     */
    template< typename list, int bit_count, typename next_bit>
    struct apply<
        state< list, bits< bit_count, false> >,
        next_bit,
        typename boost::enable_if_c< next_bit::value == operand_code>::type
    >
    {
        typedef state<
                typename boost::mpl::push_front< list, bits< bit_count, false> >::type,
                bits< 1, true>
        > type;
    };

};

/**
 * Meta-function that constructs an mpl::vector of unpacking instructions for a given
 * instruction and a given operand code.
 */
template< typename instruction, int operand_code>
struct unpack_instructions
{
    typedef typename mpl::fold< instruction, typename run_length_encoder<operand_code>::template state<>, run_length_encoder<operand_code> >::type fold_result;
    typedef typename mpl::push_front< typename fold_result::bits_list, typename fold_result::bits>::type type;
};


template< typename instructions, typename enable = void>
struct unpack
{
    static unpack_state execute( unpack_state input)
    {
        typedef typename boost::mpl::copy< instructions, boost::mpl::back_inserter< boost::mpl::vector<> > >::type
                instruction_vector;
        typedef typename boost::mpl::front< instruction_vector>::type last_instruction;
        return last_instruction::execute(
                    unpack< typename boost::mpl::pop_front<instruction_vector>::type>::execute( input)
                );
    }
};

template< typename instructions>
struct unpack< instructions, typename boost::enable_if< typename boost::mpl::empty<instructions>::type>::type >
{
    static unpack_state execute( unpack_state input)
    {
        return input;
    }
};
}

/**
 * Unpack an operand from an instruction word.
 * The instruction type is a sequence of 0s, 1s and higher integer values. Each higher integer value
 * means that at that position an operand is encoded in the instruction word. Operands can be scattered throughout
 * the instruction word. Notice for instance how in the following instruction the offset bits ('q') are placed:
 *     struct LDD_Y    : instruction< 1,0,q,0,q,q,0,d,d,d,d,d,1,q,q,q> {};
 * 'q' is here an integer number higher than 1 and different from the integer for 'd'.
 * In this example, the unpack function, if given the LDD_Y instruction and the operand_code q, will
 * return the operand in the form 0000000000qqqqqq, i.e. a 6-bit (unsigned) integer.
 */
template< typename instruction, int operand_code>
uint16_t unpack( uint16_t word)
{
    typedef typename unpacking::unpack_instructions< instruction, operand_code>::type instructions;
    //return unpacking::unpack< instructions>::execute( std::make_pair( word, 0)).second;
    return word;
}

}
#endif /* UNPACK_OPERAND_HPP_ */
