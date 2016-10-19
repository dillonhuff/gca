cmake_minimum_required(VERSION 2.6)

project(gca)

set(VTK_DIR /Users/dillon/Downloads/VTK-build/)

find_package(CGAL REQUIRED COMPONENTS Core)
include( ${CGAL_USE_FILE} )

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++11 -I./src -I./test -I/opt/local/include -O2")
#-Werror -Wall")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")

INCLUDE_DIRECTORIES(./src/)

SET(UTILS_CPPS ./src/utils/arena_allocator.cpp)

SET(UTILS_HEADERS ./src/utils/algorithm.h
		  ./src/utils/arena_allocator.h)

add_library(utils ${UTILS_CPPS} ${UTILS_HEADERS})

SET(GEOMETRY_HEADERS ./src/geometry/line.h
		     ./src/geometry/surface.h
		     ./src/geometry/point.h
		     ./src/geometry/plane.h
		     ./src/geometry/offset.h
		     ./src/geometry/polygon_3.h
	    ./src/geometry/b_spline.h
	    ./src/geometry/extrusion.h
	    ./src/geometry/spline_sampling.h
	    ./src/geometry/matrix.h
	    ./src/geometry/rotation.h
	    ./src/geometry/triangle.h
	    ./src/geometry/triangular_mesh.h
	    ./src/geometry/homogeneous_transformation.h
	    ./src/geometry/mesh_operations.h
	    ./src/geometry/vtk_debug.h
	    ./src/geometry/vtk_utils.h)

SET(GEOMETRY_CPPS ./src/geometry/point.cpp
		  ./src/geometry/surface.cpp
		  ./src/geometry/offset.cpp
		  ./src/geometry/polygon_3.cpp
		  ./src/geometry/extern_triangulate.cpp
	 ./src/geometry/arc.cpp
	 ./src/geometry/box.cpp
	 ./src/geometry/extrusion.cpp
	 ./src/geometry/plane.cpp
	 ./src/geometry/line.cpp
	 ./src/geometry/matrix.cpp
	 ./src/geometry/polygon.cpp
	 ./src/geometry/polyline.cpp
	 ./src/geometry/spline_sampling.cpp
	 ./src/geometry/rotation.cpp
	 ./src/geometry/triangle.cpp
	 ./src/geometry/triangular_mesh.cpp
	 ./src/geometry/trimesh.cpp
	 ./src/geometry/homogeneous_transformation.cpp
	 ./src/geometry/mesh_operations.cpp
	 ./src/geometry/vtk_debug.cpp
	 ./src/geometry/vtk_utils.cpp)

add_library(geometry ${GEOMETRY_CPPS} ${GEOMETRY_HEADERS})
target_link_libraries(geometry utils ${CGAL_LIBRARIES} /Users/dillon/CppWorkspace/gca/src/triangle_lib/triangle.o)

SET(GCODE_HEADERS ./src/gcode/value.h
	    	  ./src/gcode/gcode_program.h
	    	  ./src/gcode/circular_arc.h
	    	  ./src/gcode/linear_cut.h
		  ./src/gcode/machine.h
	    	  ./src/gcode/safe_move.h
	    	  ./src/gcode/hole_punch.h)

SET(GCODE_CPPS 	 ./src/gcode/cut.cpp
	 ./src/gcode/value.cpp
	 ./src/gcode/lexer.cpp
	 ./src/gcode/machine.cpp
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

SET(GCA_HEADERS
	./src/process_planning/feature_to_pocket.h
	./src/process_planning/feature_selection.h
	./src/process_planning/axis_location.h
	./src/process_planning/job_planning.h
	./src/feature_recognition/feature_decomposition.h
	./src/feature_recognition/visual_debug.h
	./src/synthesis/operation.h
	./src/synthesis/clamp_orientation.h
	./src/synthesis/visual_debug.h
	./src/synthesis/jaw_cutout.h
	./src/synthesis/clipping_plan.h
	./src/synthesis/gcode_generation.h
	./src/synthesis/operation_name.h
	    ./src/synthesis/mesh_to_gcode.h
	    ./src/synthesis/contour_planning.h
	    ./src/synthesis/face_clipping.h
	    ./src/synthesis/output.h
	    ./src/synthesis/shapes_to_gcode.h
	    ./src/synthesis/toolpath_generation.h
	    ./src/synthesis/millability.h
	    ./src/synthesis/tool.h
	    ./src/synthesis/vice.h
	    ./src/synthesis/tool.h
	    ./src/synthesis/workpiece.h
	    ./src/synthesis/fixture_analysis.h
	    ./src/synthesis/workpiece_clipping.h
	    ./src/system/json.h
	    ./src/system/parse_stl.h)

SET(GCA_CPPS
	./src/feature_recognition/feature_decomposition.cpp
	./src/feature_recognition/visual_debug.cpp
	./src/process_planning/axis_location.cpp
	./src/process_planning/feature_to_pocket.cpp
	./src/process_planning/feature_selection.cpp
	./src/process_planning/job_planning.cpp
	./src/process_planning/tool_access.cpp
	./src/synthesis/clamp_orientation.cpp
	./src/synthesis/jaw_cutout.cpp
	./src/synthesis/face_clipping.cpp
	./src/synthesis/gcode_generation.cpp
	./src/synthesis/mesh_to_gcode.cpp
	./src/synthesis/operation_name.cpp
	./src/synthesis/operation.cpp
	 ./src/synthesis/output.cpp
	 ./src/synthesis/align_blade.cpp
	 ./src/synthesis/dxf_reader.cpp
	 ./src/synthesis/shapes_to_gcode.cpp
	 ./src/synthesis/shapes_to_toolpaths.cpp
	 ./src/synthesis/cut_to_gcode.cpp
	 ./src/synthesis/contour_planning.cpp
	 ./src/synthesis/workpiece_clipping.cpp
	 ./src/synthesis/schedule_cuts.cpp
	 ./src/synthesis/toolpath_generation.cpp
	 ./src/synthesis/fixture_analysis.cpp
	 ./src/synthesis/millability.cpp
	 ./src/synthesis/tool.cpp
	 ./src/synthesis/vice.cpp
	 ./src/synthesis/visual_debug.cpp
	 ./src/system/settings.cpp
	 ./src/system/file.cpp
	 ./src/system/json.cpp
	 ./src/system/parse_stl.cpp
	 ./src/dxflib/dl_dxf.cpp
	 ./src/dxflib/dl_writer_ascii.cpp)

add_library(gca ${GCA_HEADERS} ${GCA_CPPS})
target_link_libraries(gca gcode geometry utils gprocess)

SET(TEST_FILES test/main.cpp
	       test/tool_access_tests.cpp
	       test/feature_selection_tests.cpp
	       test/rotation_tests.cpp
	       test/feature_recognition_tests.cpp
	       test/rigid_arrangement_tests.cpp
	       test/jaw_cutout_tests.cpp
	       test/extrusion_tests.cpp
	       test/plane_tests.cpp
	       test/system_algorithm_tests.cpp
	       test/contour_planning_tests.cpp
	       test/surface_tests.cpp
	       test/matrix_tests.cpp
	       test/workpiece_alignment_tests.cpp
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

add_executable(all-tests ${TEST_FILES})
target_link_libraries(all-tests geometry utils gcode gprocess gca)

add_executable(json-mesh examples/json_mesh.cpp)
target_link_libraries(json-mesh geometry utils gcode gprocess gca)

add_executable(json-plan examples/json_plan.cpp)
target_link_libraries(json-plan geometry utils gcode gprocess gca)

add_executable(stl-parse examples/stl_parse.cpp)
target_link_libraries(stl-parse geometry utils gcode gprocess gca)

add_executable(plate-cutter examples/plate_cutter.cpp)
target_link_libraries(plate-cutter geometry utils gcode gprocess gca)

add_executable(feature-decomp examples/feature_decomp.cpp)
target_link_libraries(feature-decomp geometry utils gcode gprocess gca)

add_executable(ublas examples/ublas.cpp)
target_link_libraries(ublas geometry utils gcode gprocess gca)

add_executable(check-contours examples/check_contours.cpp)
target_link_libraries(check-contours geometry utils gcode gprocess gca)

add_executable(vtk-mass examples/vtk_mass_test.cpp)
target_link_libraries(vtk-mass geometry utils gcode gprocess gca)

add_executable(scale-down-mesh examples/scale_down_mesh.cpp)
target_link_libraries(scale-down-mesh geometry utils gcode gprocess gca)

add_executable(debug-test examples/debug_test.cpp)
target_link_libraries(debug-test geometry utils gcode gprocess gca)

add_executable(nef-test examples/nef_test.cpp)
target_link_libraries(nef-test geometry utils gcode gprocess gca)

add_executable(tri-test examples/tri_test.cpp)
target_link_libraries(tri-test geometry utils gcode gprocess gca)

add_executable(manufacture examples/manufacture.cpp)
target_link_libraries(manufacture geometry utils gcode gprocess gca)

#/Users/dillon/CppWorkspace/gca/src/triangle_lib/triangle.o)

find_package( OpenCV REQUIRED )

add_executable(engraving examples/engraving.cpp)
target_link_libraries(engraving geometry utils gcode gprocess gca ${OpenCV_LIBS})

add_executable(camaster-test examples/camaster_test.cpp)
target_link_libraries(camaster-test geometry utils gcode gprocess gca ${OpenCV_LIBS})

######### VTK Example ##########

# Make command:
# /opt/local/bin/cmake -DVTK_DIR:PATH=/Users/dillon/Downloads/VTK-build/ -DCMAKE_BUILD_TYPE=Debug . 

add_executable(AddCell MACOSX_BUNDLE examples/AddCell)
target_link_libraries(AddCell geometry utils gcode gprocess gca)

add_executable(surface-test MACOSX_BUNDLE examples/surface_test.cpp)
target_link_libraries(surface-test geometry utils gcode gprocess gca)

add_executable(interact-test MACOSX_BUNDLE examples/DataAnimationSubclass.cpp)
target_link_libraries(interact-test geometry utils gcode gprocess gca)

if(VTK_LIBRARIES)
  target_link_libraries(AddCell ${VTK_LIBRARIES})
  target_link_libraries(surface-test ${VTK_LIBRARIES})
  target_link_libraries(geometry ${VTK_LIBRARIES})
  target_link_libraries(interact-test ${VTK_LIBRARIES})
else()
  target_link_libraries(AddCell vtkHybrid vtkWidgets)
  target_link_libraries(surface-test vtkHybrid vtkWidgets)
  target_link_libraries(interact-test vtkHybrid vtkWidgets)
endif()
