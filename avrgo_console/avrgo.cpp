/*
 * avrgo.cpp
 *
 *  Created on: Jul 29, 2011
 *      Author: danny
 */
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50

#include <iostream>
#include <typeinfo>
#include <boost/cstdint.hpp>
#include "avrsim/avr_instruction_set.hpp"
#include "avrsim/decoder.hpp"
#include "avrsim/instruction_names.hpp"
using boost::uint16_t;
//#include <boost/fusion/algorithm/iteration/for_each.hpp>
//#include <boost/fusion/include/for_each.hpp>
//#include <boost/fusion/adapted/mpl.hpp>
//#include <boost/fusion/include/mpl.hpp>

/**
 * Simple disassembler class.
 *
 * This class, when given an instruction word to 'execute', will print the instruction mnemonic and operands to std::cout.
 */
struct simple_disassembler
{
    template<typename tag>
    void execute( const tag &)
    {
        using avrsim::instructions::instruction_name;
        std::cout << instruction_name<tag>::get() << "\n";
    }
    template<typename tag>
    void execute( const tag &, uint16_t op)
    {
        using avrsim::instructions::instruction_name;
        std::cout << instruction_name<tag>::get() << " " << op << "\n";
    }
    template<typename tag>
    void execute( const tag &, uint16_t op1, uint16_t op2)
    {
        using avrsim::instructions::instruction_name;
        std::cout << instruction_name<tag>::get() << " " << op1 << "," << op2 <<"\n";
    }
};

void decode_and_execute( boost::uint16_t word)
{
    using namespace avrsim;
    simple_disassembler impl;
    typedef typename find_decoder<simple_disassembler, avrsim::instructions::list>::type decoder;
    decoder::decode_and_execute( impl, word);
}

/**
 * This class prints a single unpacking instruction and can be used with fusions for_each to print a list of
 * unpacking isntructions.
 */
struct unpack_printer
{
    template<int times>
    void operator()( const avrsim::unpacking::shift<times> &) const
    {
        std::cout << "shift " << times << "\n";
    }

    template<int bits, int offset>
    void operator()(const avrsim::unpacking::mask<bits, offset> &) const
    {
        std::cout << "mask " << bits << " " << offset << "\n";
    }
};

int main(int argc, char *argv[])
{

   decode_and_execute( 256 + 16 + 2);
   decode_and_execute( 0);
   decode_and_execute( 43356);

//    typedef typename avrsim::unpacking::unpack_instructions<avrsim::instructions::LDD_Y, avrsim::q>::type instructions;
//
//    boost::fusion::for_each( instructions(), unpack_printer());

    return 0;
}



