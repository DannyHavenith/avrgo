/*
 * list_parser.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: danny
 */

#include "list_parser.hpp"

#include <string>
#include <regex>
#include <iostream>

namespace avrsim
{

listing parse_listing(std::istream& stream)
{
    using namespace std;

    listing result;
    string line;
    int file_counter = 0;
    const std::regex assembly_line{R"(^\s*([0-9a-fA-F]+):\s+([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+(?:([0-9a-fA-F]{2})\s+([0-9a-fA-F]{2})\s+)?(.*)$)"};

    while (std::getline( stream, line))
    {
        std::smatch match;
        if (regex_match(line, match, assembly_line))
        {
            auto address = stoul( match[1], 0, 16)/2;
            result.assembly[address] = match[6];
            if (result.rom.size() < address + 1) result.rom.resize( address + 1);
            result.rom[address]   = stoul( match[2], 0, 16) + 256 * stoul( match[3], 0, 16);
            if (match[4].matched)
            {
                if (result.rom.size() < address + 2) result.rom.resize( address + 2);
                result.rom[address+1] = stoul( match[4], 0, 16) + 256 * stoul( match[5], 0, 16);
            }
        }
    }
    return result;
}

} /* namespace avrsim */

