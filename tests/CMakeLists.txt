
##          Copyright Danny Havenith 2011.
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)

enable_testing()
find_package( GTest REQUIRED)
find_package (Threads)

include_directories(${GTEST_INCLUDE_DIRS})
add_executable(
	unittests 

	test_core.cpp	
	)

target_link_libraries( 
    unittests
    
    avrsim
    ${GTEST_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )
    

