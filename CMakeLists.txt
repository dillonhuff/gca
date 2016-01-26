cmake_minimum_required (VERSION 2.6)
project (gca)

SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++11 -I./src -I./test -Wall -Werror -g")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

SET(TEST_FILES test/parse_tests.cpp test/point_tests.cpp test/checker_tests.cpp test/cut_tests.cpp test/line_tests.cpp test/mill_simulator_tests.cpp test/transformer_tests.cpp)

SET(SRCS  src/core/instrs/all.h src/core/instrs/instr.cpp src/core/parser.cpp src/core/gprog.cpp src/core/value.cpp src/geometry/point.cpp src/synthesis/cut.h src/synthesis/output.cpp src/checkers/bounds_checker.h src/core/pass.cpp src/simulators/sim_mill.h src/simulators/sim_mill.cpp src/simulators/mill_tool.cpp src/system/bmp_output.cpp)

add_executable(all-tests ${SRCS} ${TEST_FILES})
add_executable(bmp-prog examples/bmp_prog.cpp ${SRCS})
add_executable(print-strap examples/print_strap.cpp ${SRCS})
add_executable(gdiff examples/gdiff.cpp ${SRCS})
add_executable(z-abstract examples/z_abstract.cpp ${SRCS})
add_executable(hole-optimizer examples/hole_optimizer.cpp ${SRCS})
add_executable(simple-deepen examples/simple_deepen.cpp ${SRCS})
add_executable(hello-world examples/hello_world.cpp ${SRCS})

######### VTK Example ##########

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(TriangulateTerrainMap MACOSX_BUNDLE examples/TriangulateTerrainMap.cxx ${SRCS})
 
if(VTK_LIBRARIES)
  target_link_libraries(TriangulateTerrainMap ${VTK_LIBRARIES})
else()
  target_link_libraries(TriangulateTerrainMap vtkHybrid vtkWidgets)
endif()
