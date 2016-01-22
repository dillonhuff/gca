cmake_minimum_required (VERSION 2.6)
project (gca)

SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++98 -pedantic -I./src -I./test -Wall -Werror -g")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

SET(SRCS test/parse_tests.cpp test/point_tests.cpp test/checker_tests.cpp test/cut_tests.cpp test/line_tests.cpp test/mill_simulator_tests.cpp test/transformer_tests.cpp src/core/instr.cpp src/core/parser.cpp src/core/gprog.cpp src/core/move_instr.h src/core/value.cpp src/geometry/point.cpp src/synthesis/cut.h src/synthesis/output.cpp src/checkers/bounds_checker.h src/core/pass.cpp src/simulators/sim_mill.h src/simulators/sim_mill.cpp src/simulators/mill_tool.cpp)

add_executable(all-tests ${SRCS})
