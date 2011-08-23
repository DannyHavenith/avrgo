/*
 * unpack_operand.hpp
 *
 *  Created on: Jul 31, 2011
 *      Author: danny
 */

/*!
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

/*!
 * This namespace contains types that represent unpacking instructions.
 * Unpacking an operand consists of a series of shifts and applying masks.
 */
namespace unpacking {

/// unpack state consists of two uint16 values.
/// the first contains the instruction word, which will be shifted during unpacking
/// the second contains the operand accumulator, which will receive the operand bits.
typedef std::pair<uint16_t,uint16_t> unpack_state;

/// quick-and-dirty implementation of a power-of-two template.
template<unsigned int power>
struct power2
{
    static const unsigned int value = power2<power - 1>::value * 2;
};

template<>
struct power2<0>
{
    static const unsigned int value = 1;
};

/// this is the instruction to shift-right the instruction word a number of bits
template<int times>
struct shift
{
    static unpack_state execute( unpack_state state)
    {
        return std::make_pair( state.first >> times, state.second);
    }

    static const bool is_shift = true;
};

/// this is the instruction to transfer a number of bits at a given offset
/// from the instruction word to the operand accumulator.
template<int mask_bits, int mask_offset>
struct mask
{
    static const int bits = mask_bits;
    static const int offset = mask_offset;
    static const unsigned int value = (power2<bits>::value - 1) << offset;
    static unpack_state execute( unpack_state state)
    {
        return std::make_pair( state.first, state.second |(state.first & value));
    }
    static const bool is_shift = false;
};

template< typename T>
struct increase
{

};

template< int amount>
struct increase< shift<amount> >
{
    typedef shift<amount + 1> type;
};

template< int bits, int offset>
struct increase< mask<bits, offset> >
{
    typedef mask< bits + 1, offset> type;
};

/// unpacking builder creates unpacking instructions.
/// This class is given as an operator to the mpl::fold meta function
template< int operand_code>
struct unpacking_builder
{
    /*!
     * The state to maintain while folding the instruction mask.
     * current_list is the current list of unpacking instructions
     * current_instruction is the current instruction, which could be expanded.
     * offset is the offset in the operand accumulator where the next operand bits should be copied to.
     */
    template<
        typename current_list = boost::mpl::vector<>,
        typename current_instruction = shift<0>,
        unsigned int current_offset = 0 >
    struct state
    {
        typedef current_list instructions;
        typedef current_instruction instruction;
        static const int offset = current_offset;
    };

    /*!
     * The apply meta function accepts a current state and a type representing the next bit. It will return the
     * new state.
     */
    template< typename state, typename next_bit, typename enable = void>
    struct apply { };

    /*!
     * If we're in the masking state and we encounter a bit that is not part of the operand, we finish
     * our masking instruction (push it to the list of instructions) and start a shift instruction.
     */
    template< typename list, typename next_bit, int mask_bits, int mask_offset, int offset>
    struct apply<
        state< list, mask<mask_bits, mask_offset>, offset >,
        next_bit,
        typename boost::disable_if_c<next_bit::value == operand_code>::type >
    {
        typedef state< boost::mpl::push_front< list, mask< mask_bits, mask_offset> >, shift<1>, mask_offset + mask_bits> type;
    };

    /*!
     * If we're in the masking state and we encounter another operand bit, we extend the mask to include this bit.
     */
    template< typename current_state, typename next_bit>
    struct apply<
        current_state,
        next_bit,
        typename boost::enable_if_c<
            next_bit::value != operand_code && !current_state::instruction::is_shift>::type
    >
    {
        typedef state<
                typename boost::mpl::push_front< typename current_state::instructions, typename current_state::instruction>::type,
                shift<1>,
                current_state::instruction::bits + current_state::instruction::offset> type;
    };

    /*!
     * If we're in the shifting state and we encounter another bit that is not part of the operand,
     * we increase the shift amount.
     */
    template< typename current_state, typename next_bit>
    struct apply<
        current_state,
        next_bit,
        typename boost::enable_if_c<
            (next_bit::value != operand_code && current_state::instruction::is_shift)
            || (next_bit::value == operand_code && !current_state::instruction::is_shift)>::type
    >
    {
        typedef state<
                typename current_state::instructions,
                typename increase< typename current_state::instruction>::type,
                current_state::offset> type;
    };

    /*!
     * If we encounter an operand bit while we where in the shifting state, we stop shifting and start
     * a masking instruction.
     */
    template< typename current_state, typename next_bit>
    struct apply<
        current_state,
        next_bit,
        typename boost::enable_if_c< next_bit::value == operand_code && current_state::instruction::is_shift>::type
    >
    {
        typedef state<
                typename boost::mpl::push_front< typename current_state::instructions, typename current_state::instruction >::type,
                mask< 1, current_state::offset>,
                current_state::offset> type;
    };

};

/*!
 * Meta-function that constructs an mpl::vector of unpacking instructions for a given
 * instruction and a given operand code.
 */
template< typename instruction, int operand_code>
struct unpack_instructions
{
    typedef typename mpl::fold< instruction, typename unpacking_builder<operand_code>::template state<>, unpacking_builder<operand_code> >::type fold_result;
    typedef typename mpl::push_front< typename fold_result::instructions, typename fold_result::instruction>::type type;
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

/*!
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
    return unpacking::unpack< instructions>::execute( std::make_pair( word, 0)).second;
}

}
#endif /* UNPACK_OPERAND_HPP_ */
