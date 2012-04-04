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

/// a consecutive string of bits in an instruction word.
/// This class represents a step that needs to be taken during
/// operand extraction. This step consists of 1) shifting the instruction bit
/// right a number of bits and then 2) copying a sequence of bits from the instruction
/// word to an accumulator that will hold the operand.
/// A sequence of these instructions form a 'program' for an operand extractor to
/// extract a single operand, consisting of islands of consecutive bits, to an accumulator.
template< int bitposition, int bits, int shift>
struct island
{

};

template< int bits, int shift, int position>
struct shift_and_mask
{
    static unpack_state execute( const unpack_state &input)
    {
        const unpack_state::first_type shifted = input.first >> shift;
        const uint16_t mask = (power2<bits>::value -1) << position;
        return std::make_pair( shifted, (shifted & mask) | input.second);
    }
};

/// unpacking builder creates unpacking instructions.
/// This class is given as an operator to the mpl::fold meta function
template< int operand_code>
struct unpacking_builder
{

    template< typename list_so_far, int shift_count, int position>
    struct searching_state
    {
        static const int shifts = shift_count;
        typedef list_so_far instructions;
    };

    typedef searching_state< boost::mpl::vector0<>, 0, 0> start_state;

    template< typename list_so_far, int shift_count, int bit_count, int position>
    struct island_state
    {
        typedef list_so_far instructions;
    };

    /*!
     * The apply meta function accepts a current state and a type representing the next bit. It will return the
     * new state.
     */
    template< typename state, typename next_bit, typename enable = void>
    struct apply { };

    /*!
     * We're searching for operand bits and encounter another non-operand bit.
     * Increase the shift count and continue.
     */
    template< typename list, int shift_count, int position, typename next_bit>
    struct apply<
        searching_state< list, shift_count, position>,
        next_bit,
        typename boost::disable_if_c<next_bit::value == operand_code>::type >
    {
        typedef searching_state< list, shift_count + 1, position> type;
    };

    /*!
     * We're searching for operand bits and encounter our first operand bit.
     * switch to island state (we've found an island of operand bits).
     */
    template< typename list, int shift_count, int position, typename next_bit>
    struct apply<
        searching_state< list, shift_count, position>,
        next_bit,
        typename boost::enable_if_c<next_bit::value == operand_code>::type >
    {
        typedef island_state< list, shift_count, 1, position> type;
    };

    /*!
     * We're counting the bits of an island and encounter yet another operand bit.
     * increase the bit count for this island.
     */
    template< typename list, int shift_count, int bit_count, int position, typename next_bit>
    struct apply<
        island_state< list, shift_count, bit_count, position>,
        next_bit,
        typename boost::enable_if_c< next_bit::value == operand_code>::type >
    {
        typedef island_state< list, shift_count, bit_count + 1, position> type;
    };

    /*!
     * We're counting the bits of an island and encounter a non-operand bit.
     * switch to searching state and add a shift_and_mask instruction to the list.
     */
    template< typename list, int shift_count, int bit_count, int position, typename next_bit>
    struct apply<
        island_state< list, shift_count, bit_count, position>,
        next_bit,
        typename boost::disable_if_c< next_bit::value == operand_code>::type >
    {
        typedef typename mpl::push_back< list, shift_and_mask< shift_count, bit_count, position> >::type new_list;
        typedef searching_state< new_list, 1, position + bit_count + 1> type;
    };

    template<typename T>
    struct get_instructions
    {
        typedef typename T::instructions type;
    };

    template< typename list, int shift_count, int bit_count, int position>
    struct get_instructions< island_state< list, shift_count, bit_count, position> >
    {
        typedef mpl::push_back< list, shift_and_mask< shift_count, bit_count, position> > type;
    };
};

/*!
 * Meta-function that constructs an mpl::vector of unpacking instructions for a given
 * instruction and a given operand code.
 */
template< typename instruction, int operand_code>
struct unpack_instructions
{
    typedef typename mpl::fold< instruction, typename unpacking_builder<operand_code>::start_state, unpacking_builder<operand_code> >::type fold_result;
    typedef typename unpacking_builder<operand_code>:: template get_instructions< fold_result>::type type;
};


template< typename instructions, typename enable = void>
struct unpack
{
    typedef typename boost::mpl::front< instructions>::type first_instruction;
    typedef typename boost::mpl::pop_front<instructions>::type remaining_instructions;

    static unpack_state execute( unpack_state input)
    {
        return unpack< remaining_instructions>::execute(
                    first_instruction::execute( input)
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
