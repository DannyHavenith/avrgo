//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//

/**
 * This file defines the emscripten bindings for an emulator object.
 */

#include "emscripten/bind.h"
#include "avrsim/avr_emulator.hpp"
#include "avrsim/precompiled_emulator.hpp"
#include "avrsim/list_parser.hpp"
#include <sstream>
#include <memory>


using namespace emscripten;
using namespace avrsim;
//using emulator = avr_emulator<avr_core>;
using emulator = precompiled_emulator<avr_core>;

listing parseString( const std::string &assembly)
{
    std::stringstream stream{assembly};
    return parse_listing( stream);
}


std::vector<uint8_t> testReturnVector()
{
    return {1,2,3,4,5};
}

emulator *makeEmulator( const std::string &assembly)
{
    auto listing = parseString( assembly);
    auto e = new emulator{ 16*1024};
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
    register_vector<uint8_t>("VectorRegister");
    register_map<int,std::string>("MapAssembly");
    register_map<int,listing::line_info>("MapLineInfo");
    register_map<int,int>("AddressToAssemblyLine");

    value_object<listing>("Listing")
        .field("rom",            &listing::rom)
        .field("assembly",       &listing::assembly)
        .field("filenames",      &listing::filenames)
        .field("addressToAssembly", &listing::address_to_assembly)
        .field("memoryToLine", &listing::memory_to_source_line)
        ;

    function("parseString", &parseString);
    function("testReturnVector", &testReturnVector);

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
        .field("clockTicks",    &avr_state::clock_ticks)
        .field("ram",           &avr_state::ram)
        .field("isSleeping",    &avr_state::is_sleeping)
        .field("flags",         &avr_state::flags)
        ;

    class_<emulator>("Emulator")
        .constructor(&makeEmulator, allow_raw_pointers())
        .function("fillRom",    &emulator::fillRom)
        .function("run",        &emulator::run)
        .function("getPc",      &emulator::getPc)
        .function("setPc",      &emulator::setPc)
        .function("getState",   &emulator::getState)
        .function("getRam",     &emulator::getRam)
        .function("setRam",     &emulator::setRam)
        .function("getRegister",&emulator::getRegister)
        .function("setRegister",&emulator::setRegister)
        .function("setBreakpoint",  &emulator::setBreakpoint)
        .function("clearBreakpoint",&emulator::clearBreakpoint)
        .function("getClockTicks",  &emulator::getClockTicks)
        ;
}
