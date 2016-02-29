cmake_minimum_required(VERSION 2.6)

IF (APPLE)
ELSE()

SET(CMAKE_C_COMPILER "/usr/bin/gcc")
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")

ENDIF()

project(gca)

SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++98 -pedantic -I./src -I./test -Werror")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

SET(TEST_FILES test/parse_tests.cpp
	       test/point_tests.cpp
	       test/circular_arc_tests.cpp
	       test/checker_tests.cpp
	       test/cut_tests.cpp
	       test/line_tests.cpp
	       test/mill_simulator_tests.cpp
	       test/transformer_tests.cpp
	       test/analysis_tests.cpp
	       test/spline_tests.cpp
	       test/dxf_to_gcode_tests.cpp
	       test/shapes_to_toolpaths_tests.cpp
	       test/toolpath_tests.cpp
	       test/cut_scheduling_tests.cpp)

SET(HEADERS ./src/analysis/extract_cuts.cpp
	    ./src/checkers/bounds_checker.h
	    ./src/checkers/extra_instruction_checker.h
	    ./src/checkers/g0_move_checker.h
	    ./src/core/arena_allocator.h
	    ./src/core/basic_states.h
	    ./src/core/callback.h
	    ./src/core/context.h
	    ./src/core/gprog.h
	    ./src/core/instrs/all.h
	    ./src/core/instrs/assign_instr.h
	    ./src/core/instrs/comment.h
	    ./src/core/instrs/f_instr.h
	    ./src/core/instrs/g2_instr.h
	    ./src/core/instrs/g3_instr.h
	    ./src/core/instrs/instr.h
	    ./src/core/instrs/misc.h
	    ./src/core/instrs/circular_arc_instr.h
	    ./src/core/instrs/move_instr.h
	    ./src/core/parser.h
	    ./src/core/pass.h
	    ./src/core/state_names.h
	    ./src/core/value.h
	    ./src/diffs/all.h
	    ./src/geometry/line.h
	    ./src/geometry/point.h
	    ./src/geometry/b_spline.h
	    ./src/simulators/mill_tool.h
	    ./src/simulators/region.h
	    ./src/simulators/sim_mill.h
	    ./src/simulators/sim_res.h
	    ./src/synthesis/cut.h
	    ./src/synthesis/linear_cut.h
	    ./src/synthesis/circular_arc.h
	    ./src/synthesis/output.h
	    ./src/synthesis/shapes_to_gcode.h
	    ./src/system/bmp_output.h
	    ./src/transformers/abs_to_rel.h
	    ./src/transformers/feed_changer.h
	    ./src/transformers/g0_filter.h
	    ./src/transformers/rel_to_abs.h
	    ./src/transformers/tiler.h)

SET(CPPS ./src/core/arena_allocator.cpp
	 ./src/core/context.cpp
	 ./src/core/gprog.cpp
	 ./src/core/instrs/instr.cpp
	 ./src/core/parser.cpp
	 ./src/core/pass.cpp
	 ./src/core/value.cpp
	 ./src/checkers/bounds_checker.cpp
	 ./src/checkers/forbidden_tool_checker.cpp
	 ./src/checkers/unsafe_spindle_checker.cpp
	 ./src/geometry/point.cpp
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
	 ./src/synthesis/schedule_cuts.cpp
	 ./src/synthesis/cut.cpp
	 ./src/system/bmp_output.cpp
	 ./src/system/settings.cpp
	 ./src/transformers/scale_xyz.cpp
	 ./src/transformers/scale_xyz.cpp
	 ./src/transformers/shift_xyz.cpp
	 ./src/dxflib/dl_dxf.cpp
	 ./src/dxflib/dl_writer_ascii.cpp)

SET(SRCS ${CPPS} ${HEADERS})

add_executable(all-tests ${SRCS} ${TEST_FILES})
add_executable(gdiff examples/gdiff.cpp ${SRCS})
add_executable(simple-deepen examples/simple_deepen.cpp ${SRCS})
add_executable(hello-world examples/hello_world.cpp ${SRCS})
add_executable(pen-draw-lines examples/pen_draw_lines.cpp ${SRCS})
add_executable(print-dxf examples/print_dxf.cpp ${SRCS})
add_executable(cut-rect examples/cut_rect.cpp ${SRCS})

#add_executable(extract-g1-paths examples/extract_g1_paths.cpp ${SRCS})
#add_executable(bmp-prog examples/bmp_prog.cpp ${SRCS})
#add_executable(print-strap examples/print_strap.cpp ${SRCS})
#add_executable(z-abstract examples/z_abstract.cpp ${SRCS})
#add_executable(hole-optimizer examples/hole_optimizer.cpp ${SRCS})
#add_executable(print-knife-hacks examples/print_knife_hacks.cpp ${SRCS})

######### VTK Example ##########

# find_package(VTK REQUIRED)
# include(${VTK_USE_FILE})
 
# #add_executable(TriangulateTerrainMap MACOSX_BUNDLE examples/TriangulateTerrainMap.cxx ${SRCS})
# add_executable(DataAnimationSubclass MACOSX_BUNDLE examples/DataAnimationSubclass ${SRCS})
 
# if(VTK_LIBRARIES)
#   target_link_libraries(DataAnimationSubclass ${VTK_LIBRARIES})
# else()
#   target_link_libraries(DataAnimationSubclass vtkHybrid vtkWidgets)
# endif()
