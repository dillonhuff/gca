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
      // Failing due to a mesh with no edges?
      //{"test/stl-files/onshape_parts/Japanese_Two_Contours_Part.stl", 0.45, wp},
      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.5, wp},
      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(20).stl", 0.7, wp},
      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(3).stl", 0.5, wp},


      {"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", 1.0, wp},
		  
	{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", 0.4, wp},

	  {"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 1.0, wp},
	    {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 0.5, wp},
	      {"test/stl-files/OctagonWithHolesShort.stl", 1.0, wp},
		{"test/stl-files/onshape_parts/Part Studio 1 - ESC spacer.stl", 0.65, wp},
		  {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 0.5, wp},

		  //Passing		  
		    {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(23).stl", 0.5, wp},
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
