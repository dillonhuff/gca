#include "catch.hpp"

#include "synthesis/timing.h"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/triangular_mesh_utils.h"
#include "geometry/vtk_debug.h"
#include "process_planning/major_axis_fixturing.h"
#include "process_planning/surface_planning.h"
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

  struct surface_plan_case {
    std::string part_path;
    std::vector<int> acceptable_num_setups;
  };

  double angle_between_normals(const shared_edge e,
			       const triangular_mesh& m) {
    point n1 = m.face_orientation(e.triangle_1);
    point n2 = m.face_orientation(e.triangle_2);
    return angle_between(n1, n2);
  }
  
  bool share_non_fully_concave_edge(const surface& l, const surface& r) {
    vector<shared_edge> shared =
      all_shared_edges(l.index_list(), r.index_list(), l.get_parent_mesh());

    for (auto s : shared) {
      if (is_valley_edge(s, l.get_parent_mesh())) {
	return true;
      } else if (angle_between_normals(s, l.get_parent_mesh()) < 70.0) {
	return true;
      }
    }

    return false;
  }

  color random_color_non_pastel(const color mix) {
    unsigned red = rand() % 256;
    unsigned green = rand() % 256;
    unsigned blue = rand() % 256;

    return color(red, green, blue);

  }
  
  void
  visualize_surface_decomp(const std::vector<std::vector<surface> >& surf_complexes)
  {

    cout << "# of surfaces = " << surf_complexes.size() << endl;
    if (surf_complexes.size() == 0) { return; }

    const auto& part = surf_complexes.front().front().get_parent_mesh();

    auto part_polydata = polydata_for_trimesh(part);

    color white(255, 255, 255);

    vector<pair<vector<index_t>,  color> > colors;
    for (auto& sc : surf_complexes) {

      vector<index_t> inds;
      for (auto& s : sc) {
	concat(inds, s.index_list());
      }

      color tp_color = random_color(white);
      colors.push_back(std::make_pair(inds, tp_color));

    }

    highlight_cells(part_polydata, colors);    
    visualize_actors({polydata_actor(part_polydata)});

  }

  void visualize_non_concave_decomp(const triangular_mesh& part) {
    auto regions = const_orientation_regions(part);
    vector<surface> const_surfs = inds_to_surfaces(regions, part);
    vector<vector<surface> > surf_complexes =
      connected_components_by_elems(const_surfs,
				    [](const surface& l, const surface& r) {
				      return share_non_fully_concave_edge(l, r);
				    });

    visualize_surface_decomp(surf_complexes);
  }

  struct two_setup_plan_case {
    std::string part_path;
    point expected_axis;
    double scale_factor;
  };

      // {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", 0.4, wp},      
      // {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 0.5, wp},


      // {"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 1.0, wp},
      // 	{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 0.5, wp},

	
      // 	  {"test/stl-files/OctagonWithHolesShort.stl", 1.0, wp},
      // 	    {"test/stl-files/CircleWithFilletAndSide.stl", 1.0, wp},
      // 	      {"test/stl-files/onshape_parts/100-013 - Part 1.stl", 0.7, wp},

      // 		{"test/stl-files/onshape_parts/Part Studio 1 - ESC spacer.stl", 0.65, wp},

      // 		  {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(23).stl", 0.5, wp},
      // 		    {"test/stl-files/onshape_parts/Japanese_Two_Contours_Part.stl", 0.45, wp},
      // 		      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.5, wp},
      // 			{"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", 1.0, wp},

			      
      // 			  };

  vector<gca::two_setup_plan_case> two_setup_cases() {
    vector<gca::two_setup_plan_case> planning_cases;

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 1(37).stl", point(1, 0, 0), 0.02});

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 1(17).stl", point(0, 1, 0), 0.05});

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 1(33).stl", point(0, 0, 1), 0.45});

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 2.stl", point(0, 1, 0), 0.5});

    planning_cases.push_back({"test/stl-files/onshape_parts/100-009 - Part 1.stl", point(0, 0, 1), 1.0});

    planning_cases.push_back({"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", point(0, 0, 1), 0.4});

    planning_cases.push_back({"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", point(0, 0, 1), 1.0});
		    
    planning_cases.push_back({"test/stl-files/onshape_parts/100-013 - Part 1.stl", point(0, 0, 1), 1.0});

    planning_cases.push_back({"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", point(1, 0, 0), 0.5});

    planning_cases.push_back({"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", point(0, 1, 0), 1.0});

    return planning_cases;
  }

  TEST_CASE("Surface based plans") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<two_setup_plan_case> planning_cases = two_setup_cases();

    for (auto& test_case : planning_cases) {

      auto mesh =
	parse_and_scale_stl(test_case.part_path, test_case.scale_factor, 0.001);
      
      auto cut_axis = find_cut_axis(mesh);

      REQUIRE(cut_axis);

      const auto& sfs = cut_axis->decomp;

      cout << "Expected axis = " << test_case.expected_axis << endl;
      cout << "Actual axis   = " << cut_axis->major_axis << endl;

      //visualize_surface_decomp({sfs.positive, sfs.negative, sfs.mixed});

      bool correct_axis =
	angle_eps(cut_axis->major_axis, test_case.expected_axis, 0.0, 0.05) ||
	angle_eps(cut_axis->major_axis, -1*(test_case.expected_axis), 0.0, 0.05);

      REQUIRE(correct_axis);

      fixtures fixes = current_fixtures();
      axis_fixture axis_fix =
	build_axis_fixture(fixes, *cut_axis);

      REQUIRE(axis_fix.positive);
      REQUIRE(axis_fix.negative);

      vector<tool> tools = current_tools();
      workpiece wp(1.75, 1.75, 2.5, ALUMINUM);
      fixture_plan fs =
      	axis_fixture_plan(*cut_axis, axis_fix, fixes, wp, tools);

      //REQUIRE(fs.fixtures().size() == 2);

      fabrication_plan fp =
      	fabrication_plan_for_fixture_plan(fs, mesh, tools, wp);
    }

  }

  // TEST_CASE("Surface based plans") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   vector<surface_plan_case> planning_cases{
  //     {"test/stl-files/onshape_parts/100-009 - Part 1.stl", {3}},
  // 	{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", {2}},
  // 	  {"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", {2}},
  // 	    {"test/stl-files/OctagonWithHolesShort.stl", {8}},
  // 	      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", {2}},
  // 		{"test/stl-files/CircleWithFilletAndSide.stl", {3}},
  // 		  {"test/stl-files/onshape_parts/100-013 - Part 1.stl", {3}},
  // 		    {"test/stl-files/onshape_parts/Part Studio 1 - ESC spacer.stl", {2}},
  // 		      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(23).stl", {6}},
  // 			{"test/stl-files/onshape_parts/Japanese_Two_Contours_Part.stl", {2}},
  // 			  {"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", {2}},
  // 			    {"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", {2}},
  // 			      {"test/stl-files/onshape_parts/IL70 - Case - Case.stl", {2}},
  // 				};

  //   for (auto& test_case : planning_cases) {

  //     auto mesh = parse_stl(test_case.part_path, 0.001);

  //     vector<surface> surfs = select_profile(mesh);
  //     vtk_debug_highlight_inds(surfs);

  //     //visualize_non_concave_decomp(mesh);

  //     // boost::optional<std::vector<proto_setup> > setups =
  //     // 	surface_plans(mesh);

  //     //REQUIRE(setups);

  //     //cout << "# of setups in plan = " << (*setups).size() << endl;

  // 	// REQUIRE(elem(setups->size(), test_case.acceptable_num_setups));
  //   }

  // }

  TEST_CASE("Manufacturable parts") {
    arena_allocator a;
    set_system_allocator(&a);

    //fabrication_inputs extended_inputs = extended_fab_inputs();

    workpiece wp(1.75, 1.75, 2.5, ALUMINUM);

    vector<part_info> some_scaling{
      // This part is idiotic, should really replace it with a
      // more interesting and realistic part
      // {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(3).stl", 0.5, wp}, 

      // Failing due to non-manifold triangle in result
      //{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(20).stl", 0.7, wp},


	// Passing
      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", 0.4, wp},      
      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 0.5, wp},


      {"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 1.0, wp},
	{"test/stl-files/onshape_parts/Part Studio 1 - Part 1(29).stl", 0.5, wp},

	
	  {"test/stl-files/OctagonWithHolesShort.stl", 1.0, wp},
	    {"test/stl-files/CircleWithFilletAndSide.stl", 1.0, wp},
	      {"test/stl-files/onshape_parts/100-013 - Part 1.stl", 0.7, wp},

		{"test/stl-files/onshape_parts/Part Studio 1 - ESC spacer.stl", 0.65, wp},

		  {"test/stl-files/onshape_parts/Part Studio 1 - Part 1(23).stl", 0.5, wp},
		    {"test/stl-files/onshape_parts/Japanese_Two_Contours_Part.stl", 0.45, wp},
		      {"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.5, wp},
			{"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", 1.0, wp},

			      
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
