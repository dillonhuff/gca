#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/workpiece_clipping.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  void sanity_check_toolpaths(const fabrication_plan& plan) {
    for (auto step : plan.steps()) {
      REQUIRE(step.toolpaths().size() > 0);
    }
  }

  TEST_CASE("Mesh to gcode") {
    arena_allocator a;
    set_system_allocator(&a);

    // TODO: Make emco_vice again
    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.1, 0.3};
    fixtures fixes(test_vice, parallel_plates);

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

    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL); //1.5, 1.2, 1.5, ACETAL);

    SECTION("Simple box") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl", 0.001);
      auto result_plan = make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

      SECTION("Produces only workpiece clipping programs") {
	REQUIRE(result_plan.steps().size() == 2);
      }

      SECTION("Workpiece clipping programs actually contain code") {
	sanity_check_toolpaths(result_plan);
      }
    }

    SECTION("Box with hole has 2 clippings") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 2);
    }

    SECTION("Box with two holes has a contour and 1 pocketing") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 3);
    }

    // TODO: When part axis selection can break ties, add this part back. For now
    // it seems like the surface area heuristic is good enough
    // SECTION("Box with protrusion") {
    //   // TODO: Restore old dims
    //   workpiece workpiece_dims(3.0, 3.0, 3.0, ALUMINUM); //1.5, 1.2, 2.0, ALUMINUM);
    //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithProtrusion.stl", 0.001);
    //   auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
    //   REQUIRE(result_programs.size() == 2);
    // }

    SECTION("Box with thru hole") {
      workpiece workpiece_dims(1.51, 1.51, 2.0, ACETAL);
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithThruHole.stl", 0.001);
      auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(result_programs.size() == 2);
    }
    
  }

  TEST_CASE("Mesh to gcode with parallel plates") {
    arena_allocator a;
    set_system_allocator(&a);

    // Restore emco_vice
    vice test_vice = large_jaw_vice(6, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
      
    vector<tool> tools{t1};
    workpiece workpiece_dims(3.0, 5.0, 3.0, ACETAL);

    SECTION("Clipped Pill") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ClippedPill.stl", 0.001);

      auto programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);
      REQUIRE(programs.size() == 2);

      for (auto p : programs) {
	REQUIRE(p.blocks.size() > 0);
      }
    }
  }

  TEST_CASE("Part with hole unreachable from the top") {
    arena_allocator a;
    set_system_allocator(&a);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.25);
    t1.set_cut_length(0.6);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.3);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    
    tool t2(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.1);
    t2.set_cut_length(1.0);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.4);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    std::vector<tool> tools{t1, t2};

    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.1, 0.3, 0.5};
    fixtures fixes(test_vice, parallel_plates);

    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.0001);

    auto result_plan = make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

    SECTION("Produces only workpiece clipping programs") {
      REQUIRE(result_plan.steps().size() == 2);
    }

    SECTION("Workpiece clipping programs actually contain code") {
      sanity_check_toolpaths(result_plan);
    }
    
  }

  // TODO: Currently only handling prismatic parts
  // TEST_CASE("Complex part pocket ordering") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
  //   fixtures fixes(test_vice);
  //   workpiece workpiece_dims(2.0, 1.6, 3.98, BRASS);
  //   tool t1(0.25, 3.0, 4, CARBIDE, FLAT_NOSE);
  //   vector<tool> tools{t1};

  //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);

  //   auto outer_surfs = outer_surfaces(mesh);

  //   fixture_plan plan =
  //     make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

  //   for (auto setup : plan.fixtures()) {
  //     double last_depth = max_distance_along(setup.part_mesh().vertex_list(), point(0, 0, 1));
  //     for (auto p : setup.pockets) {
  // 	REQUIRE(p.get_end_depth() <= last_depth);
  // 	last_depth = p.get_end_depth();
  //     }
  //   }
  // }

  TEST_CASE("Outer surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Simple box") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Cube0p5.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);

      

      SECTION("Simple box has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }

      SECTION("All simple box surfaces are part of a SA faces") {
	vector<index_t> fis = mesh.face_indexes();
	workpiece workpiece_dims(1.5, 1.2, 1.5, ALUMINUM);
	auto workpiece_mesh = align_workpiece(surfaces, workpiece_dims);
	auto clipped_surfs =
	  stable_surfaces_after_clipping(mesh, workpiece_mesh);
	remove_contained_surfaces(clipped_surfs, surfaces);
	
	REQUIRE(surfaces.size() == 0);
      }
    }

    SECTION("Box with 1 hole") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithTopHole.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);
      SECTION("A box with 1 hole has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }
    }

    SECTION("Box with 2 holes") {
      auto box_triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl").triangles;
      auto mesh = make_mesh(box_triangles, 0.001);
      auto surfaces = outer_surfaces(mesh);

      SECTION("A box with 2 holes has 6 outer surfaces") {
	REQUIRE(surfaces.size() == 6);
      }
    }
  }

  void test_jaw_alignment(const rigid_arrangement& a,
			  const vice& v) {
    REQUIRE(a.mesh_names().size() == 5);

    plane a_jaw_base = surface_plane(a.labeled_surface("a_jaw", "base"));

    REQUIRE(within_eps(angle_between(a_jaw_base.normal(), point(0, 0, -1)),  0, 1.0));

    double vice_z = v.base_z();

    REQUIRE(within_eps(a_jaw_base.pt().z, vice_z, 0.001));

    point z_axis(0, 0, 1);
    double part_top = max_point_in_dir(a.mesh("part"), z_axis).z;
    double jaw_top = max_point_in_dir(a.mesh("an_jaw"), z_axis).z;

    REQUIRE(part_top > jaw_top);
    
  }

  template<typename A, typename B>
  void print_map_info(const std::map<A, B>& m) {
    for (auto p : m) {
      cout << p.first << " -> " << p.second << endl;
    }
  }
  
  void test_no_empty_toolpaths(const fabrication_plan& plan) {
    int num_empty_toolpaths = 0;

    std::map<pocket_name, int> empty_map;
    std::map<pocket_name, int> total_map;
    
    for (auto s : plan.steps()) {
      for (auto& tp : s.toolpaths()) {
	if (total_map.find(tp.pocket_type()) != end(total_map)) {
	  total_map[tp.pocket_type()] = total_map[tp.pocket_type()] + 1;
	} else {
	  total_map[tp.pocket_type()] = 1;
	}

	if (tp.lines.size() == 0) {

	  if (empty_map.find(tp.pocket_type()) != end(empty_map)) {
	    empty_map[tp.pocket_type()] = empty_map[tp.pocket_type()] + 1;
	  } else {
	    empty_map[tp.pocket_type()] = 1;
	  }

	  num_empty_toolpaths++;
	}
      }
    }

    cout << "Total toolpaths" << endl;
    print_map_info(total_map);

    cout << "Empty toolpaths" << endl;
    print_map_info(empty_map);
    REQUIRE(num_empty_toolpaths == 0);
  }

  void test_no_freeform_pockets(const fixture_plan& fix_plan) {
    int num_freeform_pockets = 0;
    for (auto f : fix_plan.fixtures()) {
      for (auto pocket : f.pockets) {
	if (pocket.pocket_type() == FREEFORM_POCKET) { num_freeform_pockets++; }
      }
    }
    REQUIRE(num_freeform_pockets == 0);
  }

  // TEST_CASE("Pendulum Arm Joint Top") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   cout << "Arm joint top" << endl;

  //   vice test_vice = large_jaw_vice(5.0, point(1.2, -4.4, 3.3));
  //   std::vector<plate_height> parallel_plates{0.5}; //0.1, 0.3};
  //   fixtures fixes(test_vice, parallel_plates);
  //   tool t2(0.05, 3.0, 4, HSS, FLAT_NOSE);
  //   tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
  //   vector<tool> tools{t1, t2};
  //   workpiece workpiece_dims(4.0, 4.0, 4.0, ALUMINUM);

  //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/Arm_Joint_Top.stl", 0.001);

  //   auto fix_plan = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

  //   test_no_freeform_pockets(fix_plan);

  //   auto plan =
  //     fabrication_plan_for_fixture_plan(fix_plan, mesh, tools, workpiece_dims);

  //   REQUIRE(plan.steps().size() == 3);

  //   REQUIRE(plan.steps()[0].arrangement().mesh_names().size() == 3);

  //   test_jaw_alignment(plan.steps()[1].arrangement(), plan.steps()[1].v);
  //   test_jaw_alignment(plan.steps()[2].arrangement(), plan.steps()[2].v);

  //   test_no_empty_toolpaths(plan);
    
  //   REQUIRE(plan.custom_fixtures().size() == 2);

  //   for (auto f : plan.custom_fixtures()) {
  //     REQUIRE(f != nullptr);
  //     test_no_empty_toolpaths(*f);
  //   }

  // }

  // TEST_CASE("Small Arm Joint Dummy Part") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   cout << "Arm joint dummy" << endl;

  //   vice test_vice = current_setup();
  //   std::vector<plate_height> parallel_plates{0.5};
  //   fixtures fixes(test_vice, parallel_plates);

  //   tool t1(0.30, 3.0, 2, HSS, FLAT_NOSE);
  //   tool t2(0.14, 3.15, 2, HSS, FLAT_NOSE);
  //   tool t3(0.10, 3.15, 2, HSS, FLAT_NOSE);
    
  //   vector<tool> tools{t1, t2, t3};
    
  //   workpiece workpiece_dims(1.5, 1.5, 1.5, ALUMINUM);

  //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ArmJointDummy.stl", 0.001);

  //   auto fix_plan = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

  //   test_no_freeform_pockets(fix_plan);

  //   auto plan =
  //     fabrication_plan_for_fixture_plan(fix_plan, mesh, tools, workpiece_dims);

  //   REQUIRE(plan.steps().size() == 3);

  //   REQUIRE(plan.steps()[0].arrangement().mesh_names().size() == 3);

  //   test_jaw_alignment(plan.steps()[1].arrangement(), plan.steps()[1].v);
  //   test_jaw_alignment(plan.steps()[2].arrangement(), plan.steps()[2].v);

  //   test_no_empty_toolpaths(plan);
    
  //   REQUIRE(plan.custom_fixtures().size() == 2);

  //   for (auto f : plan.custom_fixtures()) {
  //     REQUIRE(f != nullptr);
  //     test_no_empty_toolpaths(*f);
  //   }

  // }
  
  bool all_z_coords_above(const std::vector<block>& blocks, double z) {
    vector<vector<cut*>> cuts;
    auto r = gcode_to_cuts(blocks, cuts);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    for (auto cb : cuts) {
      for (auto c : cb) {
	if (c->get_start().z <= z || c->get_end().z <= z) {
	  cout << "Cut above " << z << " is " << endl << *c << endl;
	  return false;
	}
      }
    }
    return true;
  }


  bool all_safe_moves_above(const std::vector<block>& blocks, double z) {
    vector<vector<cut*>> cuts;
    auto r = gcode_to_cuts(blocks, cuts);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    for (auto cb : cuts) {
      for (auto c : cb) {
	if (c->is_safe_move() && (c->get_end().z <= z)) {
	  cout << "Cut above " << z << " is " << endl << *c << endl;
	  return false;
	}
      }
    }
    return true;
  }

  bool no_gouging_within(const std::vector<block>& blocks,
			 const triangular_mesh& m,
			 const double tolerance) {
    vector<vector<cut*>> cuts;
    auto r = gcode_to_cuts(blocks, cuts);
    assert(r == GCODE_TO_CUTS_SUCCESS);
    for (auto cb : cuts) {
      for (auto c : cb) {
	for (double t = 0.0; t < 1.0; t += 0.2) {
	  point v = c->value_at(t);
	  // Reverse the y negation in emco_f1_code
	  maybe<double> mv = m.z_at(v.x, -v.y);
	  double tool_head_z = v.z; // - 3.15;
	  double mesh_z = mv.t - tolerance;
	  if (mv.just && tool_head_z < mesh_z) {
	    cout << "Cut is \n" << *c << endl;
	    cout << "Mesh z = " << mv.t << endl;
	    cout << "Tool head position = " << v << endl;
	    cout << "tool head z - mesh z = " << tool_head_z - mesh_z << endl;
	    return false;
	  }
	}
      }
    }
    return true;
  }
  
  // TEST_CASE("Toolpath bounds") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   vice test_vice = emco_vice(point(-1.8, -0.4, 3.3));
  //   std::vector<plate_height> plates{0.15, 0.03, 0.5};
  //   fixtures fixes(test_vice, plates);

  //   tool t1(0.35, 3.0, 4, HSS, FLAT_NOSE);
  //   t1.set_cut_diameter(0.25);
  //   t1.set_cut_length(0.6);

  //   t1.set_shank_diameter(3.0 / 8.0);
  //   t1.set_shank_length(0.3);

  //   t1.set_holder_diameter(2.5);
  //   t1.set_holder_length(3.5);
    
  //   tool t2(0.5, 3.0, 4, HSS, FLAT_NOSE);
  //   t2.set_cut_diameter(0.5);
  //   t2.set_cut_length(0.3);

  //   t2.set_shank_diameter(0.5);
  //   t2.set_shank_length(0.5);

  //   t2.set_holder_diameter(2.5);
  //   t2.set_holder_length(3.5);

  //   vector<tool> tools{t1, t2};
  //   workpiece workpiece_dims(1.7, 2.1, 1.65, ACETAL);

  //   SECTION("Box with 2 holes") {
  //     auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWith2Holes.stl", 0.001);
  //     auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);

  //     SECTION("Never cut below vice") {
  //   	for (auto program : result_programs) {
  //   	  REQUIRE(all_z_coords_above(program.blocks, test_vice.base_z()));
  //   	}
  //     }
  //   }

  // }
}
