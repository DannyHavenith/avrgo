/*
 * decoder.hpp
 *
 *  Created on: Jul 27, 2011
 *      Author: danny
 */

#ifndef DECODER_HPP_
#define DECODER_HPP_
#include <boost/mpl/size.hpp>
#include <boost/mpl/front.hpp>
#include <boost/mpl/copy_if.hpp>
#include <boost/mpl/front_inserter.hpp>
#include <boost/mpl/list.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/empty.hpp>
#include <boost/mpl/or.hpp>
#include <boost/mpl/count_if.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/cstdint.hpp>

#include "instruction.hpp"

namespace avrsim
{
namespace mpl = boost::mpl;
typedef boost::uint16_t uint16_t;

struct UNKNOWN_INSTRUCTION
{

};

template<typename instruction_tag>
struct operand_harvester
{
    template< typename implementation>
    static void decode_and_execute( implementation &imp, uint16_t instruction)
    {
        // todo: implement
        imp.execute( instruction_tag(), instruction);
    }
};

template<>
struct operand_harvester<UNKNOWN_INSTRUCTION>
{
    template< typename implementation>
    void decode_and_execute( implementation &imp, uint16_t instruction)
    {
        imp.execute( UNKNOWN_INSTRUCTION(), instruction);
    }
};

template<
    typename implementation,
    typename instructions_with_zero_at_position,
    typename instructions_with_one_at_position,
    int bit
    >
struct decoder;

/// Meta function that finds a decoder given an instruction set (or subset)
template<
    typename implementation,
    typename instruction_set,
    int instruction_set_size = boost::mpl::size< instruction_set>::value
    >
struct find_decoder
{

    /// This meta function determines if a bit position, given the instruction set, is a discriminating
    /// bit position.
    /// A discriminating bit position is one where all the instructions in the instruction set have a fixed
    /// zero or one value
    /// and where none of the instructions has an operand. Furthermore, it should not be the case that all
    /// the instructions have the same value (zero or one) at the given position.
    /// If a bit position is a discriminating position, then we can look at that bit in an instruction word
    /// and reduce the number of candidate instructions.
    template <int bit>
    struct is_discriminator
    {
        typedef typename mpl::count_if< instruction_set, has_at<bit, 0> >::type zero_count;
        typedef typename mpl::count_if< instruction_set, has_at<bit, 1> >::type one_count;
        typedef typename mpl::count_if< instruction_set, has_operand_at<bit> >::type operand_count;

        typedef mpl::bool_< zero_count::value != 0 && one_count::value != 0 && operand_count::value == 0> type;
    };

    /// meta function to find a discrimination bit position in the given instruction set.
    /// the dummy template argument is there to satisfy g++s thing about fully specializing
    /// a template at 'non-namespace scope'.
    template< int start_bit, int dummy = 0>
    struct find_discriminator : mpl::if_<
        typename is_discriminator< start_bit>::type,
        mpl::int_< start_bit>,
        typename find_discriminator< start_bit - 1>::type
        >
    {};

    /// we should never reach this point, but we need it to end recursion.
    template< int dummy>
    struct find_discriminator<-1, dummy> : mpl::identity< mpl::int_<-1> >
    {};

    typedef typename find_discriminator<15>::type discriminator_bit;

    typedef typename mpl::copy_if< instruction_set, has_at< discriminator_bit::value, 0>, mpl::front_inserter< mpl::list<> > >::type zeros;
    typedef typename mpl::copy_if< instruction_set, has_at< discriminator_bit::value, 1>, mpl::front_inserter< mpl::list<> > >::type ones;

    /// the final result of this metafunction, the decoder that examines the discriminating bit
    /// and then decides to look further in either the instructions that have a value of '1' at that
    /// position or in those that have a '0' there.
    typedef decoder< implementation, zeros, ones, discriminator_bit::value> type;
};

/// When the instruction set contains only one item, we don't really need to decode
/// anymore, so we return a class that will harvest the operands from the instruction word and
/// that calls the implementation.
template<
    typename implementation,
    typename instruction_set
    >
struct find_decoder<implementation, instruction_set, 1>
{
    typedef typename boost::mpl::front<instruction_set>::type instruction_tag;
    typedef operand_harvester<instruction_tag> type;
};

/// Something's gone wrong if we end up with an empty instruction set, because now we don't know what function to call.
template<
    typename implementation,
    typename instruction_set
    >
struct find_decoder<implementation, instruction_set, 0>
{
    typedef operand_harvester<UNKNOWN_INSTRUCTION> type;
};

/// A decoder examines a single bit of an instruction and delegates further processing of the instruction
/// to the next decoder.
template<
    typename implementation,
    typename instructions_with_zero_at_position,
    typename instructions_with_one_at_position,
    int bit
    >
struct decoder
{
    static void decode_and_execute( implementation &imp, uint16_t instruction)
    {
        if ( instruction & (1 << bit))
        {
           typedef typename find_decoder< implementation, instructions_with_one_at_position>::type next_decoder;
           next_decoder::decode_and_execute( imp, instruction);
        }
        else
        {
            typedef typename find_decoder< implementation, instructions_with_zero_at_position>::type next_decoder;
            next_decoder::decode_and_execute( imp, instruction);
        }
    }
};

} // end namespace avrsim
#endif /* DECODER_HPP_ */
