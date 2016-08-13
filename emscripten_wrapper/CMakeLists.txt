
##          Copyright Danny Havenith 2011.
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)

add_executable(
	avrjs 

    avrjs.cpp
	)

target_link_libraries( 
    avrjs
    
    avrsim
    )
    
SET_TARGET_PROPERTIES( 
    avrjs 
    PROPERTIES LINK_FLAGS 
    "--bind -s WARN_ON_UNDEFINED_SYMBOLS=1 -s DISABLE_EXCEPTION_CATCHING=0 -s ASSERTIONS=1 -s NO_EXIT_RUNTIME=1")
