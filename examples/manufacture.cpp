#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/vtk_debug.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/visual_debug.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Manufacturable parts") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_v =
      custom_jaw_vice_with_clamp_dir(4.0, 1.0, 8.0, point(0.0, 0.0, 0.0), point(1, 0, 0));
    vice test_vice = top_jaw_origin_vice(test_v);
    
    std::vector<plate_height> plates{0.1, 0.3, 0.7};
    fixtures fixes(test_vice, plates);

    workpiece workpiece_dims(3.5, 3.5, 3.8, ALUMINUM);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    
    tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.5);
    t2.set_cut_length(0.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    tool t3{1.0 / 8.0, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(1.0 / 8.0);
    t3.set_cut_length(1.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);

    tool t4{1.5, 3.94, 4, HSS, FLAT_NOSE};
    t4.set_cut_diameter(1.5);
    t4.set_cut_length(2.2);

    t4.set_shank_diameter(0.5);
    t4.set_shank_length(0.05);

    t4.set_holder_diameter(2.5);
    t4.set_holder_length(3.5);
    
    vector<tool> tools{t1, t2, t3, t4};

    vector<string> fails{
      // Winding issue?
      "test/stl-files/onshape_parts//Part Studio 1 - Part 1(13).stl",

	// Clamp orientation failure?
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(33).stl",

	// Another clamp orientation failure?
	"test/stl-files/onshape_parts//Part Studio 1 - Part 2.stl",

	};
    
    vector<string> too_large{
      "test/stl-files/onshape_parts//Part Studio 1 - Part 1(17).stl",
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(37).stl"};

    vector<string> passes{
      "test/stl-files/OctagonWithHoles.stl",
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(24).stl",


	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(29).stl",      

	"test/stl-files/onshape_parts//PSU Mount - PSU Mount.stl",
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(2).stl",
	  
      	"test/stl-files/onshape_parts//Part Studio 1 - Falcon Prarie .177 single shot tray.stl",

	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(3).stl",
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(20).stl",
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1.stl",

	"test/stl-files/onshape_parts//Part Studio 1 - ESC spacer.stl",
	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(23).stl"};

    vector<string> part_paths{};

    for (auto part_path : passes) {
      cout << "Part path: " << part_path << endl;

      auto mesh = parse_stl(part_path, 0.001);

      box bounding = mesh.bounding_box();

      cout << "X length = " << bounding.x_len() << endl;
      cout << "Y length = " << bounding.y_len() << endl;
      cout << "Z length = " << bounding.z_len() << endl;

      //vtk_debug_mesh(mesh);
    }

    double total_toolpath_execution_time = 0.0;
    double total_air_time = 0.0;

    double rapid_feed = 24.0;

    for (auto part_path : passes) {
      cout << "Part path: " << part_path << endl;

      auto mesh = parse_stl(part_path, 0.001);

      box bounding = mesh.bounding_box();

      cout << "Bounding box = " << endl;
      cout << bounding << endl;

      // vtk_debug_mesh(mesh);

      fabrication_plan p =
	make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

      cout << "Number of steps = " << p.steps().size() << endl;
      for (auto& step : p.steps()) {
	cout << "STEP" << endl;
	for (auto& tp : step.toolpaths()) {
	  double exec_time = execution_time_seconds(tp, rapid_feed);
	  double air_time = air_time_seconds(tp, rapid_feed);
	  double air_pct = (air_time / exec_time) * 100.0;

	  cout << "Toolpath execution time = " << exec_time << " seconds " << endl;
	  cout << "Toolpath air time = " << exec_time << " seconds " << endl;
	  cout << "Fraction of toolpath in air = " << air_pct << " % " << endl;

	  total_toolpath_execution_time += exec_time;
	  total_air_time += air_time;
	}
      }

      cout << "Total execution time so far = " << total_toolpath_execution_time << " seconds" << endl;
      cout << "Total air time so far = " << total_air_time / 60.0 << " minutes" << endl;

      double total_air_pct =
	(total_air_time / total_toolpath_execution_time) * 100.0;

      cout << "Total fraction of air time = " << total_air_pct << " % " << endl;

      for (auto step : p.steps()) {
      	visual_debug(step);
      }

    }

    cout << "Total toolpath time for all parts = " << total_toolpath_execution_time << " seconds" << endl;
    cout << "Total air time for all parts = " << total_air_time << " seconds" << endl;    

    double total_air_pct =
      (total_air_time / total_toolpath_execution_time) * 100.0;

    cout << "Percent of time spent in the air = " << total_air_pct << " % " << endl;

  }

}
