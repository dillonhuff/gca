cmake_minimum_required(VERSION 2.6)

IF (APPLE)
ELSE()

SET(CMAKE_C_COMPILER "/usr/bin/gcc")
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

ENDIF()

project(gca)

#Add -Wextra ?
SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++11 -I./src -I./test -I/opt/local/include -O2")
# REINTRODUCE WHEN NOT USING VTK ? -pedantic -Werror -Wall")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

SET(TEST_FILES test/system_algorithm_tests.cpp
	       test/polygon_tests.cpp
	       test/polyline_tests.cpp
	       test/triangle_tests.cpp
	       test/retargeting_tests.cpp
	       test/arc_tests.cpp
	       test/lexer_tests.cpp
	       test/position_table_tests.cpp
	       test/point_tests.cpp
	       test/line_tests.cpp
	       test/unfold_tests.cpp
	       test/circular_arc_tests.cpp
	       test/mesh_tests.cpp
	       test/checker_tests.cpp
	       test/cut_tests.cpp
	       test/toolpath_generation_tests.cpp
	       test/mill_simulator_tests.cpp
	       test/transformer_tests.cpp
	       test/analysis_tests.cpp
	       test/spline_tests.cpp
	       test/dxf_to_gcode_tests.cpp
	       test/shapes_to_toolpaths_tests.cpp
	       test/toolpath_tests.cpp
	       test/cut_scheduling_tests.cpp
	       test/gcode_to_cuts_tests.cpp
	       test/machine_state_tests.cpp
	       test/axis_3_tests.cpp)

SET(HEADERS ./src/analysis/extract_cuts.cpp
	    ./src/checkers/bounds_checker.h
	    ./src/core/value.h
	    ./src/geometry/line.h
	    ./src/geometry/point.h
	    ./src/geometry/b_spline.h
	    ./src/geometry/triangle.h
	    ./src/geometry/triangular_mesh.h
	    ./src/simulators/mill_tool.h
	    ./src/simulators/region.h
	    ./src/simulators/sim_mill.h
	    ./src/simulators/sim_res.h
	    ./src/synthesis/cut.h
	    ./src/synthesis/linear_cut.h
	    ./src/synthesis/circular_arc.h
	    ./src/synthesis/output.h
	    ./src/synthesis/shapes_to_gcode.h
	    ./src/synthesis/toolpath_generation.h
	    ./src/synthesis/axis_3.h
	    ./src/system/arena_allocator.h
	    ./src/system/parse_stl.h
	    ./src/transformers/feed_changer.h
	    ./src/transformers/retarget.h)

SET(CPPS ./src/analysis/gcode_to_cuts.cpp
	 ./src/analysis/position_table.cpp
	 ./src/analysis/utils.cpp
	 ./src/analysis/unfold.cpp
	 ./src/analysis/machine_state.cpp
	 ./src/analysis/profiler.cpp
	 ./src/core/value.cpp
	 ./src/core/lexer.cpp
	 ./src/core/parse_stream.cpp
	 ./src/checkers/bounds_checker.cpp
	 ./src/checkers/forbidden_tool_checker.cpp
	 ./src/checkers/unsafe_spindle_checker.cpp
	 ./src/checkers/block_rate_checker.cpp
	 ./src/geometry/point.cpp
	 ./src/geometry/arc.cpp
	 ./src/geometry/box.cpp
	 ./src/geometry/line.cpp
	 ./src/geometry/polygon.cpp
	 ./src/geometry/polyline.cpp
	 ./src/geometry/triangle.cpp
	 ./src/geometry/triangular_mesh.cpp
	 ./src/simulators/mill_tool.cpp
	 ./src/simulators/sim_mill.cpp
	 ./src/synthesis/output.cpp
	 ./src/synthesis/align_blade.cpp
	 ./src/synthesis/dxf_reader.cpp
	 ./src/synthesis/shapes_to_gcode.cpp
	 ./src/synthesis/toolpath.cpp
	 ./src/synthesis/spline_sampling.cpp
	 ./src/synthesis/shapes_to_toolpaths.cpp
	 ./src/synthesis/cut_to_gcode.cpp
	 ./src/synthesis/machine.cpp
	 ./src/synthesis/schedule_cuts.cpp
	 ./src/synthesis/cut.cpp
	 ./src/synthesis/linear_cut.cpp
	 ./src/synthesis/toolpath_generation.cpp
	 ./src/synthesis/axis_3.cpp
	 ./src/system/settings.cpp
	 ./src/system/file.cpp
	 ./src/system/arena_allocator.cpp
	 ./src/system/parse_stl.cpp
	 ./src/transformers/feed_changer.cpp
	 ./src/transformers/retarget.cpp
	 ./src/dxflib/dl_dxf.cpp
	 ./src/dxflib/dl_writer_ascii.cpp)

SET(SRCS ${CPPS} ${HEADERS})

add_executable(all-tests ${SRCS} ${TEST_FILES})
# add_executable(hello-world examples/hello_world.cpp ${SRCS})
# add_executable(print-dxf examples/print_dxf.cpp ${SRCS})
# add_executable(misc examples/misc.cpp ${SRCS})
# add_executable(analyze-gcodes examples/analyze_gcodes.cpp ${SRCS})
add_executable(stl-parse examples/stl_parse.cpp ${SRCS})
#add_executable(face-graph examples/face_graph.cpp ${SRCS})

######### VTK Example ##########

# Make command:
# /opt/local/bin/cmake -DVTK_DIR:PATH=/Users/dillon/Downloads/VTK-build/ .

cmake_minimum_required(VERSION 2.8)
 
PROJECT(AddCell)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(AddCell MACOSX_BUNDLE examples/AddCell ${SRCS})
 
if(VTK_LIBRARIES)
  target_link_libraries(AddCell ${VTK_LIBRARIES})
else()
  target_link_libraries(AddCell vtkHybrid vtkWidgets)
endif()

# find_package(VTK REQUIRED)
# include(${VTK_USE_FILE})
 
# #add_executable(TriangulateTerrainMap MACOSX_BUNDLE examples/TriangulateTerrainMap.cxx ${SRCS})
# #add_executable(DataAnimationSubclass MACOSX_BUNDLE examples/DataAnimationSubclass ${SRCS})

# add_executable(tool-simulator MACOSX_BUNDLE examples/tool_simulator ${SRCS})
 
# if(VTK_LIBRARIES)
#   target_link_libraries(tool-simulator ${VTK_LIBRARIES})
# else()
#   target_link_libraries(tool-simulator vtkHybrid vtkWidgets)
# endif()
