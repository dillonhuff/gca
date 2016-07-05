cmake_minimum_required(VERSION 2.6)

IF (APPLE)
ELSE()

SET(CMAKE_C_COMPILER "/usr/bin/gcc")
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

ENDIF()

project(gca)

SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++11 -I./src -I./test -I/opt/local/include -O2 -Werror -Wall")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

SET(UTILS_CPPS ./src/utils/arena_allocator.cpp)

SET(UTILS_HEADERS ./src/utils/algorithm.h
		  ./src/utils/arena_allocator.h)

add_library(utils ${UTILS_CPPS} ${UTILS_HEADERS})

SET(GEOMETRY_HEADERS ./src/geometry/line.h
		     ./src/geometry/surface.h
		     ./src/geometry/point.h
	    ./src/geometry/b_spline.h
	    ./src/geometry/spline_sampling.h
	    ./src/geometry/matrix.h
	    ./src/geometry/triangle.h
	    ./src/geometry/triangular_mesh.h)

SET(GEOMETRY_CPPS ./src/geometry/point.cpp
		  ./src/geometry/surface.cpp
	 ./src/geometry/arc.cpp
	 ./src/geometry/box.cpp
	 ./src/geometry/line.cpp
	 ./src/geometry/matrix.cpp
	 ./src/geometry/polygon.cpp
	 ./src/geometry/polyline.cpp
	 ./src/geometry/spline_sampling.cpp
	 ./src/geometry/triangle.cpp
	 ./src/geometry/triangular_mesh.cpp
	 ./src/geometry/trimesh.cpp)

add_library(geometry ${GEOMETRY_CPPS} ${GEOMETRY_HEADERS})
target_link_libraries(geometry utils)

SET(GCODE_HEADERS ./src/gcode/value.h
	    	  ./src/gcode/gcode_program.h
	    	  ./src/gcode/circular_arc.h
	    	  ./src/gcode/linear_cut.h
	    	  ./src/gcode/safe_move.h
	    	  ./src/gcode/hole_punch.h)

SET(GCODE_CPPS 	 ./src/gcode/cut.cpp
	 ./src/gcode/value.cpp
	 ./src/gcode/lexer.cpp
	 ./src/gcode/parse_stream.cpp
	 ./src/gcode/linear_cut.cpp)

add_library(gcode ${GCODE_HEADERS} ${GCODE_CPPS})
target_link_libraries(gcode geometry utils)

SET(GPROCESS_HEADERS 	    ./src/checkers/bounds_checker.h
	    ./src/simulators/mill_tool.h
	    ./src/simulators/region.h
	    ./src/simulators/sim_mill.h
	    ./src/simulators/sim_res.h
	    ./src/transformers/feed_changer.h
	    ./src/transformers/retarget.h)

SET(GPROCESS_CPPS ./src/analysis/extract_cuts.cpp
	 ./src/analysis/gcode_to_cuts.cpp
	 ./src/analysis/position_table.cpp
	 ./src/analysis/utils.cpp
	 ./src/analysis/unfold.cpp
	 ./src/analysis/machine_state.cpp
	 ./src/analysis/profiler.cpp
	 ./src/checkers/bounds_checker.cpp
	 ./src/checkers/forbidden_tool_checker.cpp
	 ./src/checkers/unsafe_spindle_checker.cpp
	 ./src/checkers/block_rate_checker.cpp
	 ./src/simulators/mill_tool.cpp
	 ./src/simulators/sim_mill.cpp
	 ./src/transformers/feed_changer.cpp
	 ./src/transformers/retarget.cpp)

add_library(gprocess ${GPROCESS_HEADERS} ${GPROCESS_CPPS})
target_link_libraries(gprocess gcode geometry utils)

SET(HEADERS 
	    ./src/synthesis/mesh_to_gcode.h
	    ./src/synthesis/output.h
	    ./src/synthesis/shapes_to_gcode.h
	    ./src/synthesis/toolpath_generation.h
	    ./src/synthesis/axis_3.h
	    ./src/synthesis/millability.h
	    ./src/synthesis/vice.h
	    ./src/synthesis/tool.h
	    ./src/synthesis/workpiece.h
	    ./src/synthesis/fixture_analysis.h
	    ./src/synthesis/workpiece_clipping.h
	    ./src/system/json.h
	    ./src/system/parse_stl.h)

SET(CPPS 
	 ./src/synthesis/mesh_to_gcode.cpp
	 ./src/synthesis/output.cpp
	 ./src/synthesis/align_blade.cpp
	 ./src/synthesis/dxf_reader.cpp
	 ./src/synthesis/shapes_to_gcode.cpp
	 ./src/synthesis/toolpath.cpp
	 ./src/synthesis/shapes_to_toolpaths.cpp
	 ./src/synthesis/cut_to_gcode.cpp
	 ./src/synthesis/machine.cpp
	 ./src/synthesis/workpiece_clipping.cpp
	 ./src/synthesis/schedule_cuts.cpp
	 ./src/synthesis/toolpath_generation.cpp
	 ./src/synthesis/axis_3.cpp
	 ./src/synthesis/fixture_analysis.cpp
	 ./src/synthesis/millability.cpp
	 ./src/synthesis/vice.cpp
	 ./src/system/settings.cpp
	 ./src/system/file.cpp
	 ./src/system/json.cpp
	 ./src/system/parse_stl.cpp
	 ./src/dxflib/dl_dxf.cpp
	 ./src/dxflib/dl_writer_ascii.cpp)

SET(SRCS ${CPPS} ${HEADERS})

SET(TEST_FILES test/system_algorithm_tests.cpp
	       test/fixture_analysis_tests.cpp
	       test/millability_tests.cpp
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
	       test/mesh_to_gcode_tests.cpp
	       test/axis_3_tests.cpp)

add_executable(all-tests ${SRCS} ${TEST_FILES})
target_link_libraries(all-tests geometry utils gcode gprocess)

add_executable(json-mesh examples/json_mesh.cpp ${SRCS})
target_link_libraries(json-mesh geometry utils gcode gprocess)

add_executable(json-plan examples/json_plan.cpp ${SRCS})
target_link_libraries(json-plan geometry utils gcode gprocess)

add_executable(stl-parse examples/stl_parse.cpp ${SRCS})
target_link_libraries(stl-parse geometry utils gcode gprocess)

######### VTK Example ##########

# Make command:
# /opt/local/bin/cmake -DVTK_DIR:PATH=/Users/dillon/Downloads/VTK-build/ .

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(AddCell MACOSX_BUNDLE examples/AddCell ${SRCS})
target_link_libraries(AddCell geometry utils gcode gprocess)

add_executable(surface-test MACOSX_BUNDLE examples/surface_test.cpp ${SRCS})
target_link_libraries(surface-test geometry utils gcode gprocess)

if(VTK_LIBRARIES)
  target_link_libraries(AddCell ${VTK_LIBRARIES})
  target_link_libraries(surface-test ${VTK_LIBRARIES})
else()
  target_link_libraries(AddCell vtkHybrid vtkWidgets)
  target_link_libraries(surface-test vtkHybrid vtkWidgets)
endif()
