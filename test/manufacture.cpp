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
#include "synthesis/millability.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/visual_debug.h"
#include "utils/arena_allocator.h"
#include "utils/relation.h"
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

  struct part_info {
    string path;
    double scale_factor;
    workpiece stock;
  };

  struct surface_plan_case {
    std::string part_path;
    std::vector<int> acceptable_num_setups;
  };

  std::vector<surface>
  non_concave_decomp(const triangular_mesh& part) {
    auto regions = const_orientation_regions(part);
    vector<surface> const_surfs = inds_to_surfaces(regions, part);
    vector<vector<surface> > surf_complexes =
      connected_components_by_elems(const_surfs,
				    [](const surface& l, const surface& r) {
				      return share_non_fully_concave_edge(l, r);
				    });

    vector<surface> surfs;
    for (auto& sg : surf_complexes) {
      surfs.push_back(merge_surfaces(sg));
    }
    return surfs;
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

  struct surface_setup {
    point access_direction;
    std::vector<surface> surfs;
  };

  struct surface_plan {
    std::vector<surface_setup> setups;
  };

  struct partial_plan {
    const relation<surface, point>& access_rel;
    std::vector<unsigned> surfs_left;
    std::vector<unsigned> dirs_left;

    surface_plan plan;
  };

  std::vector<surface_plan>
  ranked_surface_plans(const std::vector<surface>& surfs) {
    if (surfs.size() == 0) { return {}; }

    const auto& mesh = surfs.front().get_parent_mesh();

    vector<surface> out_surfs = outer_surfaces(mesh);
    vector<point> access_dirs;
    for (auto& s : out_surfs) {
      access_dirs.push_back(normal(s));
    }

    relation<surface, point> access_rel(surfs, access_dirs);
    for (unsigned i = 0; i < access_dirs.size(); i++) {
      vector<index_t> inds = millable_faces(access_dirs[i], mesh);
      sort(begin(inds), end(inds));

      for (unsigned surf_ind = 0; surf_ind < surfs.size(); surf_ind++) {
	if ( surfs[surf_ind].contained_by_sorted(inds) ) {
	  access_rel.insert(surf_ind, i);
	}
      }
    }

    vector<surface_setup> setups;
    vector<unsigned> assigned;
    for (unsigned i = 0; i < access_dirs.size(); i++) {
      vector<surface> sfs;
      point p = access_dirs[i];

      surface_setup ss;
      ss.access_direction = p;

      for (unsigned j = 0; j < surfs.size(); j++) {
	if (!elem(j, assigned) && access_rel.connected(j, i)) {
	  assigned.push_back(j);
	  ss.surfs.push_back(surfs[j]);
	}
      }

      if (ss.surfs.size() > 0) {
	setups.push_back(ss);
      }
    }

    vector<surface_plan> plans{{setups}};

    return plans;
  }

  struct two_setup_plan_case {
    std::string part_path;
    point expected_axis;
    double scale_factor;
    workpiece wp;
  };

  vector<gca::two_setup_plan_case> two_setup_cases() {
    vector<gca::two_setup_plan_case> planning_cases;

    workpiece wp(1.75, 1.75, 2.5, ALUMINUM);

    wp = workpiece(1.0, 1.0, 1.0, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 2.stl", point(0, 1, 0), 0.5, wp});

    wp = workpiece(1.75, 1.75, 0.75, ALUMINUM);
    
    planning_cases.push_back({"test/stl-files/onshape_parts/Part Studio 1 - Part 1(24).stl", point(0, 0, 1), 0.4, wp});

    wp = workpiece(1.5, 1.5, 1.0, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 1(17).stl", point(0, 1, 0), 0.035, wp});

    wp = workpiece(1.5, 1.5, 1.5, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 1(37).stl", point(1, 0, 0), 0.02, wp});

    wp = workpiece(1.5, 1.5, 1.0, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", point(1, 0, 0), 0.5, wp});

    wp = workpiece(1.75, 1.75, 1.0, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts//Part Studio 1 - Part 1(33).stl", point(0, 0, 1), 0.45, wp});

    wp = workpiece(1.0, 1.0, 2.5, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts/100-013 - Part 1.stl", point(0, 0, 1), 1.0, wp});

    wp = workpiece(1.0, 1.0, 2.5, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", point(0, 0, 1), 1.0, wp});
		    
    wp = workpiece(1.75, 1.75, 1.0, ALUMINUM);

    planning_cases.push_back({"test/stl-files/onshape_parts/Part Studio 1 - Falcon Prarie .177 single shot tray.stl", point(0, 1, 0), 1.0, wp});

    wp = workpiece(1.75, 1.75, 2.25, ALUMINUM);
    
    // Was failing, but that seems to have been a problem with boolean operations
    planning_cases.push_back({"test/stl-files/onshape_parts/100-009 - Part 1.stl", point(0, 0, 1), 1.0, wp});

    return planning_cases;
  }

  std::vector<tool> long_tools() {
    auto tools = current_tools();

    tool t6{1.0 / 8.0, 3.94, 4, HSS, BALL_NOSE};
    t6.set_cut_diameter(1.0 / 16.0);
    t6.set_cut_length(3.25);

    t6.set_shank_diameter(0.5);
    t6.set_shank_length(0.05);

    t6.set_holder_diameter(2.5);
    t6.set_holder_length(3.5);
    t6.set_tool_number(4);
    
    tool long_tool(0.125, 3.0, 4, HSS, FLAT_NOSE);
    long_tool.set_cut_diameter(0.5);
    long_tool.set_cut_length(1.25);

    long_tool.set_shank_diameter(0.6);
    long_tool.set_shank_length(0.18);

    long_tool.set_holder_diameter(1.8);
    long_tool.set_holder_length(3.0);

    tool drill(0.1, 3.0, 4, HSS, TWIST_DRILL);
    drill.set_cut_diameter(0.06717);
    drill.set_cut_length(2.0);

    drill.set_shank_diameter(3.0 / 8.0);
    drill.set_shank_length(0.1);

    drill.set_holder_diameter(2.0);
    drill.set_holder_length(2.5);
    drill.set_tool_number(3);
    
    tools.push_back(t6);
    tools.push_back(long_tool);
    tools.push_back(drill);
    
    return tools;
  }

  TEST_CASE("Viz based plans") {
    arena_allocator a;
    set_system_allocator(&a);

    vector<two_setup_plan_case> planning_cases = two_setup_cases();

    for (auto& test_case : planning_cases) {

      auto mesh =
	parse_and_scale_stl(test_case.part_path, test_case.scale_factor, 0.001);

      vector<surface> surfs = non_concave_decomp(mesh);
      vector<surface_plan> plans
	= ranked_surface_plans(surfs);

      for (auto& plan : plans) {
	vector<vector<surface> > decomp;
	for (auto& step : plan.setups) {
	  //vtk_debug_highlight_inds(step.surfs);
	  cout << step.access_direction << endl;
	  decomp.push_back(step.surfs);
	}

	cout << "# of setups in plan = " << plan.setups.size() << endl;
	visualize_surface_decomp(decomp);
      }
    }

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

      vector<tool> tools = long_tools();
      workpiece wp = test_case.wp;
      fabrication_plan fp =
	axis_fabrication_plan(*cut_axis, axis_fix, fixes, wp, tools);

      REQUIRE(fp.steps().size() == 2);

      // for (auto& fs : fp.steps()) {
      // 	visual_debug(fs);
      // }
    }

  }

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

      print_programs_no_TLC(p);

      // cout << "TOTAL TIME" << endl;
      // print_time_info(cout, total_time);

      // for (auto step : p.steps()) {
      // 	visual_debug(step);
      // }

    }

    cout << "TOTAL TIME FOR ALL PARTS" << endl;
    print_time_info(cout, total_time);

  }

}
