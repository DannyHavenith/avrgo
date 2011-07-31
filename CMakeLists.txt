
##          Copyright Danny Havenith 2011.
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)


cmake_minimum_required(VERSION 2.6)

project(avrgo)
add_definitions(-D_SCL_SECURE_NO_WARNINGS)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE( Boost COMPONENTS python serialization system)

include_directories( ${avrgo_SOURCE_DIR})

add_subdirectory (avrsim)
add_subdirectory (avrgo_console)


set(CPACK_PACKAGE_NAME "avrgo")
set(CPACK_PACKAGE_VENDOR "havenith.net")
set(CPACK_PACKAGE_CONTACT "dh.avrgo@havenith.net")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "avrgo, Atmel AVR emulator")
set(CPACK_PACKAGE_VERSION "0.1.0")
set(CPACK_PACKAGE_VERSION_MAJOR "0")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "avrgo")
set(CPACK_PACKAGE_EXECUTABLES "avrgoui;avrgo emulator")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE_1_0.txt")
include(InstallRequiredSystemLibraries)
include(CPack)
