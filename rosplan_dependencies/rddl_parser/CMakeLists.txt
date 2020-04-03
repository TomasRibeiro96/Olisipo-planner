cmake_minimum_required (VERSION 2.8.11)
project (RDDL_PARSER)

# Set flags
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE LBITS)
if (LBITS EQUAL 64)
    message(STATUS "LBITS: Using the -m64 flag")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64")
else()
    message(STATUS "LBITS: Using the -m32 flag")
    SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32")
endif(LBITS EQUAL 64)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -W -Wno-sign-compare -Wno-deprecated -ansi -pedantic -Werror -std=c++11")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -fomit-frame-pointer -DNDEBUG")

set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -O3 -g")

# Lexer and parser
find_package(BISON)
find_package(FLEX)

BISON_TARGET(RDDL_PARSER parser.ypp ${CMAKE_CURRENT_BINARY_DIR}/parser.tab.cc COMPILE_FLAGS -d)
FLEX_TARGET(RDDL_SCANNER lexer.l ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc)
ADD_FLEX_BISON_DEPENDENCY(RDDL_SCANNER RDDL_PARSER)

# Sources
set(RDDL_PARSER_SOURCES
        rddl.cc
        instantiator.cc
        preprocessor.cc
        task_analyzer.cc
        evaluatables.cc
        logical_expressions.cc
        states.cc
        probability_distribution.cc
        utils/timer.cc
        utils/string_utils.cc
        utils/system_utils.cc
        utils/math_utils.cc
        RDDLParser.cpp)

set(RDDL_PARSER_HEADERS
        rddl.h
        instantiator.h
        preprocessor.h
        task_analyzer.h
        evaluatables.h
        logical_expressions.h
        states.h
        probability_distribution.h
        utils/timer.h
        utils/string_utils.h
        utils/system_utils.h
        utils/math_utils.h
        RDDLParser.h)

# Compile
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}) # To find generated header files from BISON

add_library(rddl_parser SHARED
        ${BISON_RDDL_PARSER_OUTPUTS}
        ${FLEX_RDDL_SCANNER_OUTPUTS}
        ${RDDL_PARSER_SOURCES}
        ${RDDL_PARSER_HEADERS})

add_executable(parse_rddl
        parser_main.cpp)
target_link_libraries(parse_rddl rddl_parser)
