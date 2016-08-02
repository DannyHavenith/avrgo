/*
 * list_parser.h
 *
 *  Created on: Aug 1, 2016
 *      Author: danny
 */
#ifndef AVRSIM_LIST_PARSER_HPP_
#define AVRSIM_LIST_PARSER_HPP_
#include <cstdint>
#include <vector>
#include <map>
#include <string>

namespace avrsim
{

struct listing
{
    /// rom is implemented in the emulator as 16-bits words instead
    /// of addressable bytes.
    std::vector< uint16_t>       rom;
    std::map< int, std::string>  assembly;
    std::vector<std::string>     filenames;

    struct line_info
    {
        int file;
        int line;
    };

    std::map< int, line_info>   memory_to_line_info;
};

listing parse_listing( std::istream &stream);

} /* namespace avrsim */

#endif /* AVRSIM_LIST_PARSER_HPP_ */
