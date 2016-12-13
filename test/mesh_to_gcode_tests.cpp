#include "analysis/gcode_to_cuts.h"
#include "catch.hpp"
#include "geometry/triangular_mesh.h"
#include "synthesis/fixture_analysis.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "synthesis/visual_debug.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  void sanity_check_toolpaths(const fabrication_plan& plan) {
    for (auto step : plan.steps()) {
      REQUIRE(step.toolpaths().size() > 0);
    }
  }

  TEST_CASE("Octagon with holes") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_v = custom_jaw_vice(6.0, 1.5, 10.0, point(0.0, 0.0, 0.0));
    vice test_vice = top_jaw_origin_vice(test_v);
    
    std::vector<plate_height> plates{0.1, 0.3, 0.7};
    fixtures fixes(test_vice, plates);

    workpiece workpiece_dims(5.0, 5.0, 5.0, ALUMINUM);

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

    tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(0.12);
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

    string part_path = "test/stl-files/OctagonWithHoles.stl";

    cout << "Part path: " << part_path << endl;

    auto mesh = parse_stl(part_path, 0.001);

    box bounding = mesh.bounding_box();

    cout << "Bounding box = " << endl;
    cout << bounding << endl;

    SECTION("Full fabrication plan") {
      fabrication_plan p =
	make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

      REQUIRE(p.steps().size() == 8);
    }
  }

  TEST_CASE("Mesh to gcode") {
    arena_allocator a;
    set_system_allocator(&a);

    // TODO: Make emco_vice again
    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0)); //large_jaw_vice(5, point(-0.8, -4.4, -3.3));
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
    t2.set_cut_length(0.5);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL);

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
  }


  TEST_CASE("Part 1(29)") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = custom_jaw_vice(6.0, 1.5, 10.0, point(0.0, 0.0, 0.0));
    std::vector<plate_height> plates{0.1, 0.3, 0.7};
    fixtures fixes(test_vice, plates);

    workpiece workpiece_dims(5.0, 5.0, 5.0, ALUMINUM);

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

    tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(0.05);
    t3.set_cut_length(1.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);

    // Ridiculous tool used to test feasability
    tool t4{1.5, 3.94, 4, HSS, FLAT_NOSE};
    t4.set_cut_diameter(1.5);
    t4.set_cut_length(2.2);

    t4.set_shank_diameter(0.5);
    t4.set_shank_length(0.05);

    t4.set_holder_diameter(2.5);
    t4.set_holder_length(3.5);
    
    vector<tool> tools{t1, t2, t3, t4};

    string part_path = "test/stl-files/onshape_parts//Part Studio 1 - Part 1(29).stl";
    auto mesh = parse_stl(part_path, 0.001);

    box bounding = mesh.bounding_box();

    cout << "Bounding box = " << endl;
    cout << bounding << endl;

    fabrication_plan p = make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

    cout << "Number of steps = " << p.steps().size() << endl;

    REQUIRE(p.steps().size() == 3);

  }

  TEST_CASE("Part 1 (24)") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_v = custom_jaw_vice(6.0, 1.5, 10.0, point(0.0, 0.0, 0.0));
    vice test_vice = top_jaw_origin_vice(test_v);
    
    std::vector<plate_height> plates{0.1, 0.3, 0.7};
    fixtures fixes(test_vice, plates);

    workpiece workpiece_dims(5.0, 5.0, 5.0, ALUMINUM);

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

    tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(0.12);
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

    string part_path =
      "test/stl-files/onshape_parts//Part Studio 1 - Part 1(24).stl";

    cout << "Part path: " << part_path << endl;

    auto mesh = parse_stl(part_path, 0.001);

    box bounding = mesh.bounding_box();

    cout << "Bounding box = " << endl;
    cout << bounding << endl;

    fabrication_plan p =
      make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

    REQUIRE(p.steps().size() == 2);
  }
  
  TEST_CASE("Box with hole") {
    arena_allocator a;
    set_system_allocator(&a);

    // TODO: Make emco_vice again
    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0)); //large_jaw_vice(5, point(-0.8, -4.4, -3.3));
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
    t2.set_cut_length(1.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    vector<tool> tools{t1, t2};
    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL); //1.5, 1.2, 1.5, ACETAL);

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
    
  }

  TEST_CASE("Code generation for box with thru hole") {
    arena_allocator a;
    set_system_allocator(&a);

    // TODO: Make emco_vice again
    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0)); //large_jaw_vice(5, point(-0.8, -4.4, -3.3));
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
    t2.set_cut_length(1.3);

    t2.set_shank_diameter(0.5);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    // NOTE: Totally unrealistic tool, should remove once open pocket analysis
    // is in place
    tool t3(0.5, 3.0, 4, HSS, FLAT_NOSE);
    t3.set_cut_diameter(0.1);
    t3.set_cut_length(1.5);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.5);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);
    
    vector<tool> tools{t1, t2, t3};
    workpiece workpiece_dims(1.75, 2.0, 2.0, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BoxWithThruHole.stl", 0.001);
    auto result_programs = mesh_to_gcode(mesh, fixes, tools, workpiece_dims);

    REQUIRE(result_programs.size() == 2);
  }  

  TEST_CASE("Mesh to gcode with parallel plates") {
    arena_allocator a;
    set_system_allocator(&a);

    // Change back to emco_vice
    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.1);
    t1.set_cut_length(0.4);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.1);

    t1.set_holder_diameter(2.0);
    t1.set_holder_length(2.5);

    tool t2(0.12, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(0.12);
    t2.set_cut_length(1.3);

    t2.set_shank_diameter(0.1);
    t2.set_shank_length(0.5);

    t2.set_holder_diameter(2.0);
    t2.set_holder_length(2.5);

    vector<tool> tools{t1, t2};

    workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);
    
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

    tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t3.set_cut_diameter(0.2334);
    t3.set_cut_length(2.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);
    
    std::vector<tool> tools{t1, t2, t3};

    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0)); //large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.1, 0.3, 0.5};
    fixtures fixes(test_vice, parallel_plates);

    workpiece workpiece_dims(3.0, 3.0, 3.0, ACETAL);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1.stl", 0.0001);

    auto result_plan = make_fabrication_plan(mesh, fixes, tools, {workpiece_dims});

    SECTION("2 setups") {
      REQUIRE(result_plan.steps().size() == 2);

      sanity_check_toolpaths(result_plan);
    }

  }

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

	if (tp.lines().size() == 0) {

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

  TEST_CASE("Three chamfers code") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    // NOTE: Ridiculous tool dimensions but that isnt the point
    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.1);
    t1.set_cut_length(3.0);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.1);

    t1.set_holder_diameter(2.0);
    t1.set_holder_length(2.5);
    t1.set_tool_number(1);

    vector<double> chamfer_angles{45.0, 53.0};

    tool t2(0.1, 3.0, 4, HSS, CHAMFER);
    t2.set_chamfer_included_angle(45.0*2.0);
    
    t2.set_cut_diameter(0.5);
    t2.set_cut_length(1.0);

    t2.set_shank_diameter(3.0 / 8.0);
    t2.set_shank_length(0.1);

    t2.set_holder_diameter(2.0);
    t2.set_holder_length(2.5);
    t2.set_tool_number(2);

    tool t3(0.1, 3.0, 4, HSS, CHAMFER);
    t3.set_chamfer_included_angle(36.0*2.0);
    
    t3.set_cut_diameter(0.5);
    t3.set_cut_length(1.0);

    t3.set_shank_diameter(3.0 / 8.0);
    t3.set_shank_length(0.1);

    t3.set_holder_diameter(2.0);
    t3.set_holder_length(2.5);
    t3.set_tool_number(3);

    vector<tool> tools{t1, t2, t3};

    workpiece workpiece_dims(4.0, 4.0, 4.0, BRASS);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ThreeChamfers.stl", 0.001);

    fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

    REQUIRE(p.fixtures().size() == 2);

    const fixture_setup& fix_0 = p.fixtures()[0];

    int num_chamfers = 0;
    for (auto& p : fix_0.pockets) {
      if (p.pocket_type() == CHAMFER_POCKET) {
	num_chamfers++;
      }
    }

    REQUIRE(num_chamfers == 3);

    fabrication_plan fab_plan =
      fabrication_plan_for_fixture_plan(p, mesh, tools, workpiece_dims);

    REQUIRE(fab_plan.steps().size() == 2);

    for (auto& tp : fab_plan.steps()[0].toolpaths()) {
      if (tp.pocket_type() == CHAMFER_POCKET) {
	REQUIRE((tp.get_tool().type() == CHAMFER));
      }
    }
  }

  TEST_CASE("Part with drilled holes") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(1.0 / 8.0);
    t1.set_cut_length(3.0 / 8.0);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.1);

    t1.set_holder_diameter(1.8);
    t1.set_holder_length(3.0);
    t1.set_tool_number(1);

    tool t2(0.335, 3.0, 4, HSS, FLAT_NOSE);
    t2.set_cut_diameter(3.0 / 8.0);
    t2.set_cut_length(0.75);

    t2.set_shank_diameter(3.0 / 8.0);
    t2.set_shank_length(0.1);

    t2.set_holder_diameter(1.8);
    t2.set_holder_length(3.0);
    t2.set_tool_number(2);

    tool t3(0.5, 3.0, 2, HSS, FLAT_NOSE);
    t3.set_cut_diameter(0.5);
    t3.set_cut_length(2.25);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.5);

    t3.set_holder_diameter(1.8);
    t3.set_holder_length(3.0);
    t3.set_tool_number(3);

    tool t4(0.1, 3.0, 4, HSS, CHAMFER);
    t4.set_chamfer_included_angle(45.0*2.0);
    
    t4.set_cut_diameter(0.5);
    t4.set_cut_length(1.0);

    t4.set_shank_diameter(3.0 / 8.0);
    t4.set_shank_length(0.1);

    t4.set_holder_diameter(2.0);
    t4.set_holder_length(2.5);
    t4.set_tool_number(2);

    tool t5(0.1, 3.0, 4, HSS, CHAMFER);
    t5.set_chamfer_included_angle(36.0*2.0);
    
    t5.set_cut_diameter(0.5);
    t5.set_cut_length(1.0);

    t5.set_shank_diameter(3.0 / 8.0);
    t5.set_shank_length(0.1);

    t5.set_holder_diameter(2.0);
    t5.set_holder_length(2.5);
    t5.set_tool_number(3);

    tool t6(0.1, 3.0, 4, HSS, TWIST_DRILL);
    t6.set_cut_diameter(0.3515);
    t6.set_cut_length(2.0);

    t6.set_shank_diameter(3.0 / 8.0);
    t6.set_shank_length(0.1);

    t6.set_holder_diameter(2.0);
    t6.set_holder_length(2.5);
    t6.set_tool_number(3);
    
    vector<tool> tools{t1, t2, t3, t4, t5, t6};

    workpiece workpiece_dims(4.0, 4.0, 4.0, ALUMINUM);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/100-013 - Part 1.stl", 0.0001);

    fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

    REQUIRE(p.fixtures().size() == 3);

    const fixture_setup& fix_2 = p.fixtures()[2];

    int num_drilled_holes = 0;
    for (auto& p : fix_2.pockets) {
      if (p.pocket_type() == DRILLED_HOLE_POCKET) {
	num_drilled_holes++;
      }
    }

    REQUIRE(num_drilled_holes == 2);

    fabrication_plan fab_plan =
      fabrication_plan_for_fixture_plan(p, mesh, tools, workpiece_dims);

    REQUIRE(fab_plan.steps().size() == 3);

    for (auto& tp : fab_plan.steps()[0].toolpaths()) {
      if (tp.pocket_type() == DRILLED_HOLE_POCKET) {
	REQUIRE((tp.get_tool().type() == TWIST_DRILL));
      }
    }

  }

  TEST_CASE("Part with freeform surface") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_v = custom_jaw_vice(5.0, 1.5, 8.1, point(0.0, 0.0, 0.0));
    vice test_vice = top_jaw_origin_vice(test_v);
    
    std::vector<plate_height> parallel_plates{0.4995};
    fixtures fixes(test_vice, parallel_plates);

    tool t1{1.0 / 8.0, 3.94, 4, HSS, BALL_NOSE};
    t1.set_cut_diameter(1.0 / 4.0);
    t1.set_cut_length(1.25);

    t1.set_shank_diameter(0.5);
    t1.set_shank_length(0.05);

    t1.set_holder_diameter(2.5);
    t1.set_holder_length(3.5);
    t1.set_tool_number(1);
    
    tool t2{1.0, 3.94, 4, HSS, FLAT_NOSE};
    t2.set_cut_diameter(0.1);
    t2.set_cut_length(1.8);

    t2.set_shank_diameter(1.1);
    t2.set_shank_length(0.05);

    t2.set_holder_diameter(2.5);
    t2.set_holder_length(3.5);

    vector<tool> tools{t1, t2};

    workpiece workpiece_dims(2.58, 2.5, 2.5, ALUMINUM);
    //    workpiece workpiece_dims(1.58, 1.5, 1.5, ALUMINUM);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CubeSubtractSphere.stl", 0.0001);

    fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

    REQUIRE(p.fixtures().size() == 2);

    int num_freeform = 0;
    for (auto& fix_2 : p.fixtures()) {
      for (auto& p : fix_2.pockets) {
	if (p.pocket_type() == FREEFORM_POCKET) {
	  num_freeform++;
	}
      }
    }

    REQUIRE(num_freeform == 1);

    fabrication_plan fab_plan =
      fabrication_plan_for_fixture_plan(p, mesh, tools, workpiece_dims);

    REQUIRE(fab_plan.steps().size() == 2);

    for (auto& tp : fab_plan.steps()[0].toolpaths()) {
      if (tp.pocket_type() == FREEFORM_POCKET) {
	REQUIRE((tp.get_tool().type() == BALL_NOSE));
      }
    }

  }
  
}
