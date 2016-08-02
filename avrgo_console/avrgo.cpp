#include <iosfwd>

/*
 * avrgo.cpp
 *
 *  Created on: Jul 29, 2011
 *      Author: danny
 */
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50

#include <iostream>
#include <fstream>
#include <typeinfo>
#include <boost/cstdint.hpp>
#include "avrsim/avr_instruction_set.hpp"
#include "avrsim/decoder.hpp"
#include "avrsim/avr_core.hpp"
#include "avrsim/instruction_names.hpp"
#include "avrsim/list_parser.hpp"

using boost::uint16_t;
#include <boost/fusion/algorithm/iteration/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/adapted/mpl.hpp>
#include <boost/fusion/include/mpl.hpp>

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
    typedef  find_decoder<simple_disassembler, avrsim::instructions::list>::type decoder;
    decoder::decode_and_execute( impl, word);
}

/**
 * This class prints a single unpacking instruction and can be used with fusions for_each to print a list of
 * unpacking instructions.
 */
struct unpack_printer
{
    template<int times>
    void operator()( const avrsim::unpacking::bits<times, false> &) const
    {
        std::cout << "shift " << times << "\n";
    }

    template<int times>
    void operator()(const avrsim::unpacking::bits<times, true> &) const
    {
        std::cout << "mask " << times << "\n";
    }
};

int main(int argc, char *argv[])
{
    using namespace avrsim;

    if (argc ==2)
    {
        std::ifstream assemblyfile{ argv[1]};
        auto listing = parse_listing( assemblyfile);


        avr_core avr{512};
        avr.rom = listing.rom;
        using decoder = typename find_decoder< avr_core, instructions::list>::type;
        while (true)
        {
            decoder::decode_and_execute( avr, avr.fetch_instruction_word());
        }
    }
    return 0;
}



