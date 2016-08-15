
##          Copyright Danny Havenith 2011.
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)

add_executable(
	avrgo 

	avrgo.cpp	
	)

target_link_libraries( 
    avrgo
    
    avrsim
    ${Boost_LIBRARIES}
    )


