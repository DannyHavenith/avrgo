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


struct simple_implementation
{
    template<typename tag>
    void execute( const tag &, boost::uint16_t word)
    {
        std::cout << typeid( tag).name() << " " << word << "\n";
    }
};

void decode_and_execute( boost::uint16_t word)
{
    using namespace avrsim;
    using namespace avrsim::instructions;
    simple_implementation impl;
    typedef typename find_decoder<simple_implementation, avrsim::instructions::list>::type decoder;
    decoder::decode_and_execute( impl, word);
}

int main(int argc, char *argv[])
{

    decode_and_execute( 42);
    return 0;
}



