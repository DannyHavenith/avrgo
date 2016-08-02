
##          Copyright Danny Havenith 2011.
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)

add_library(
	avrsim 
	
	avrsim.cpp
    list_parser.cpp

	avr_instruction_set.hpp
	decoder.hpp
	instruction.hpp
	)

# when building for 64bit processors on linux we need to 
# add -fPIC to the compiler options to be able to use
# this lib in shared libraries
IF( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )
  SET_TARGET_PROPERTIES(avrsim PROPERTIES COMPILE_FLAGS "-fPIC")
ENDIF( CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" )

