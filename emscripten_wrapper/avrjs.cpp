//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_VECTOR_SIZE 50

#include "emscripten/bind.h"
#include "avrsim/avr_emulator.hpp"
#include "avrsim/list_parser.hpp"
#include <sstream>
#include <memory>


using namespace emscripten;
using namespace avrsim;
using emulator = avr_emulator<avr_core>;

listing parseString( const std::string &assembly)
{
    std::stringstream stream{assembly};
    return parse_listing( stream);
}


emulator *makeEmulator( const std::string &assembly)
{
    auto listing = parseString( assembly);
    auto e = new emulator{ 512};
    e->fillRom( listing.rom);
    return e;
}

int main()
{
    return 0;
}

EMSCRIPTEN_BINDINGS( avrjs) {

    value_object<listing::line_info>("LineInfo")
        .field("file", &listing::line_info::file)
        .field("line", &listing::line_info::line)
        ;

    register_vector<uint16_t>("VectorInt16");
    register_vector<std::string>("VectorString");
    register_map<int,std::string>("MapAssembly");
    register_map<int,listing::line_info>("MapLineInfo");

    value_object<listing>("Listing")
        .field("rom",            &listing::rom)
        .field("assembly",       &listing::assembly)
        .field("filenames",      &listing::filenames)
        .field("memory_to_line", &listing::memory_to_source_line)
        ;

    function("parseString", &parseString);

    value_object<flags_t>("Flags")
        .field("I", &flags_t::I)
        .field("T", &flags_t::T)
        .field("H", &flags_t::H)
        .field("S", &flags_t::S)
        .field("V", &flags_t::V)
        .field("N", &flags_t::N)
        .field("Z", &flags_t::Z)
        .field("C", &flags_t::C)
         ;

    value_object<avr_state>("AvrState")
        .field("eind",          &avr_state::eind)
        .field("pc",            &avr_state::pc)
        .field("clock_ticks",   &avr_state::clock_ticks)
        .field("ram",           &avr_state::ram)
        .field("r",             &avr_state::r)
        ;

    class_<emulator>("Emulator")
//        .constructor<size_t>()
        .constructor(&makeEmulator, allow_raw_pointers())
        .function("initialize", &emulator::initialize)
        .function("fillRom",    &emulator::fillRom)
        .function("run",        &emulator::run)
        .function("getPc",      &emulator::getPc)
        //    .property("x", &MyClass::getX, &MyClass::setX)
        //    .class_function("getStringFromInstance", &MyClass::getStringFromInstance)
        ;
}
