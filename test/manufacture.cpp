#include "catch.hpp"

#include "backend/timing.h"
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

  fabrication_inputs current_fab_inputs(const workpiece& workpiece_dims) { 
    vice test_v = current_setup();
    vice test_vice = top_jaw_origin_vice(test_v);

    std::vector<plate_height> plates{0.48, 0.625, 0.7};
    fixtures fixes(test_vice, plates);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.14);
    t1.set_cut_length(0.5);

    t1.set_shank_diameter(.375);
    t1.set_shank_length(0.18);

    t1.set_holder_diameter(1.8);
    t1.set_holder_length(3.0);

    tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.335);
    t2.set_cut_length(0.72);

    t2.set_shank_diameter(0.336);
    t2.set_shank_length(0.01);

    t2.set_holder_diameter(1.8);
    t2.set_holder_length(3.0);

    tool t3(0.125, 3.0, 4, HSS, FLAT_NOSE);
    t3.set_cut_diameter(0.125);
    t3.set_cut_length(1.0);

    t3.set_shank_diameter(.375);
    t3.set_shank_length(0.18);

    t3.set_holder_diameter(1.8);
    t3.set_holder_length(3.0);

    tool t4{1.0, 3.94, 4, HSS, FLAT_NOSE};
    t4.set_cut_diameter(1.0);
    t4.set_cut_length(1.0);

    t4.set_shank_diameter(1.1);
    t4.set_shank_length(0.05);

    t4.set_holder_diameter(2.5);
    t4.set_holder_length(3.5);

    tool t5(0.0625, 3.0, 4, HSS, FLAT_NOSE);
    t5.set_cut_diameter(0.0625);
    t5.set_cut_length(0.4);

    t5.set_shank_diameter(.375);
    t5.set_shank_length(0.18);

    t5.set_holder_diameter(1.8);
    t5.set_holder_length(3.0);
    
    vector<tool> tools{t1, t2, t3, t4, t5};

    return fabrication_inputs(fixes, tools, workpiece_dims);
  }

  fabrication_inputs extended_fab_inputs() {
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

    return fabrication_inputs(fixes, tools, workpiece_dims);
  }

  // NOTE: FAILING INPUTS
  // "test/stl-files/onshape_parts//Part Studio 1 - Part 1(13).stl"

  // Clamp orientation failure?
  // "test/stl-files/onshape_parts//Part Studio 1 - Part 1(33).stl"

  // Another clamp orientation failure?
  // "test/stl-files/onshape_parts//Part Studio 1 - Part 2.stl"

  // NOTE: Inputs that are too large
  //   "test/stl-files/onshape_parts//Part Studio 1 - Part 1(17).stl"
  // 	"test/stl-files/onshape_parts//Part Studio 1 - Part 1(37).stl"
  //    


  triangular_mesh parse_and_scale_box_stl(const std::string& part_path,
					  const double max_dim,
					  const double tol) {
    auto mesh = parse_stl(part_path, tol);

    box b = mesh.bounding_box();

    vector<double> dims{b.x_len(), b.y_len(), b.z_len()};

    double max_mesh_dim = max_e(dims, [](const double d) { return d; });
    if (max_mesh_dim > max_dim) {
      double scale_factor = max_dim / max_mesh_dim;

      auto scale_func = [scale_factor](const point p) {
	return scale_factor*p;
      };

      mesh =
	mesh.apply_to_vertices(scale_func);

    }

    box scaled_bounds = mesh.bounding_box();

    cout << "Scaled bounds " << endl;
    cout << "X length = " << scaled_bounds.x_len() << endl;
    cout << "Y length = " << scaled_bounds.y_len() << endl;
    cout << "Z length = " << scaled_bounds.z_len() << endl;

    return mesh;
  }

  struct part_info {
    string path;
    double scale_factor;
    workpiece stock;
  };

  TEST_CASE("Manufacturable parts") {
    arena_allocator a;
    set_system_allocator(&a);

    //fabrication_inputs extended_inputs = extended_fab_inputs();

    workpiece wp(1.75, 1.75, 2.5, ALUMINUM);

    vector<part_info> some_scaling{
      //      {"test/stl-files/onshape_parts/Part Studio 4 - Part 1.stl", 0.2, wp},
      {"test/stl-files/OctagonWithHolesShort.stl", 1.0, wp},
	{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 0.5, wp},
	  {"test/stl-files/onshape_parts/Part Studio 1 - ESC spacer.stl", 0.65, wp},
	    {"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 1.0, wp},
	      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 0.5, wp},

		{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(20).stl", 0.7, wp},
		  {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(23).stl", 0.5, wp},
	
		    {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(3).stl", 0.5, wp},

		      {"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", 1.0, wp},
			{"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.5, wp},
			  {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", 0.4, wp},
			    };
    
    vector<part_info> all_paths = some_scaling;
    for (auto part_path : all_paths) {
      cout << "Part path: " << part_path.path << endl;

      auto mesh = parse_stl(part_path.path, 0.001);

      box bounding = mesh.bounding_box();

      cout << "X length = " << bounding.x_len() << endl;
      cout << "Y length = " << bounding.y_len() << endl;
      cout << "Z length = " << bounding.z_len() << endl;

      //vtk_debug_mesh(mesh);
    }

    double rapid_feed = 24.0;
    fab_plan_timing_info total_time;

    for (auto info : all_paths) {
      cout << "Part path: " << info.path << endl;

      fabrication_inputs inputs = current_fab_inputs(info.stock);

      auto mesh = parse_and_scale_stl(info.path, info.scale_factor, 0.001);

      //vtk_debug_mesh(mesh);

      fabrication_plan p =
	make_fabrication_plan(mesh, inputs);

      cout << "Number of steps = " << p.steps().size() << endl;
      for (auto& step : p.steps()) {
	cout << "STEP" << endl;
	auto step_time = make_timing_info(step, rapid_feed);
	increment(total_time, step_time);
      }

      cout << "TOTAL TIME" << endl;
      print_time_info(cout, total_time);

      // for (auto step : p.steps()) {
      // 	visual_debug(step);
      // }

    }

    cout << "TOTAL TIME FOR ALL PARTS" << endl;
    print_time_info(cout, total_time);

  }

}
