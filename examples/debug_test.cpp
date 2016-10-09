#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Round with thru holes") {
    arena_allocator a;
    set_system_allocator(&a);

    // Change back to emco_vice
    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.1);
    t1.set_cut_length(0.4);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.1);

    t1.set_holder_diameter(2.0);
    t1.set_holder_length(2.5);
    
    vector<tool> tools{t1};
    
    workpiece workpiece_dims(2.0, 2.0, 2.0, BRASS);

    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/RoundEdges2Holes.stl", 0.001);

    cout << "Starting round with thru holes" << endl;
    fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

    REQUIRE(p.fixtures().size() == 2);

    cout << "Done with thru holes" << endl;
  }

  
  TEST_CASE("More parallel plates") {
    arena_allocator a;
    set_system_allocator(&a);

    // Change back to emco_vice
    vice test_vice = large_jaw_vice(5, point(-0.8, -4.4, -3.3));
    std::vector<plate_height> parallel_plates{0.5, 0.7};
    fixtures fixes(test_vice, parallel_plates);

    tool t1(0.1, 3.0, 4, HSS, FLAT_NOSE);
    t1.set_cut_diameter(0.1);
    t1.set_cut_length(0.4);

    t1.set_shank_diameter(3.0 / 8.0);
    t1.set_shank_length(0.1);

    t1.set_holder_diameter(2.0);
    t1.set_holder_length(2.5);
    
    vector<tool> tools{t1};

    workpiece workpiece_dims(3.0, 1.9, 3.0, ACETAL);

    SECTION("onshape part 1 (2)") {
      tool t3{0.2334, 3.94, 4, HSS, FLAT_NOSE};
      t3.set_cut_diameter(0.2334);
      t3.set_cut_length(1.2);

      t3.set_shank_diameter(0.5);
      t3.set_shank_length(0.05);

      t3.set_holder_diameter(2.5);
      t3.set_holder_length(3.5);

      vector<tool> tools{t1, t3};
      
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - Part 1(2).stl", 0.0001);

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

      REQUIRE(p.fixtures().size() == 2);

      for (auto p : p.fixtures()[1].pockets) {
	cout << p.pocket_type() << endl;
      }

      REQUIRE(p.fixtures()[1].pockets.size() == 1);
      REQUIRE(p.fixtures()[0].pockets.size() == 3);
    }

    SECTION("onshape PSU Mount") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/PSU Mount - PSU Mount.stl", 0.0001);

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

      REQUIRE(p.fixtures().size() == 2);

      for (auto f : p.fixtures()) {
	cout << "orientation = " << f.fix.orient.top_normal() << endl;
      }

      REQUIRE(p.fixtures()[1].pockets.size() == 3);
      REQUIRE(p.fixtures()[0].pockets.size() == 7);
    }


    SECTION("Block with hole and side pocket") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/BlockWithHoleAndSidePocket.stl", 0.0001);

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

      REQUIRE(p.fixtures().size() == 3);

      for (auto f : p.fixtures()) {
    	cout << "orientation = " << f.fix.orient.top_normal() << endl;
      }

      REQUIRE(p.fixtures()[0].pockets.size() == 3);
      REQUIRE(p.fixtures()[1].pockets.size() == 1);
      REQUIRE(p.fixtures()[2].pockets.size() == 1);
    }

  }

  // TEST_CASE("Single Shot Tray") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   vice test_vice = emco_vice(point(1.3, -4.4, 3.3));
  //   std::vector<plate_height> plates{0.1, 0.3};
  //   fixtures fixes(test_vice, plates);

  //   workpiece workpiece_dims(2.0, 2.0, 3.98, ALUMINUM);

  //   tool t1(0.25, 3.0, 4, HSS, FLAT_NOSE);
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

  //   auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/onshape_parts/Part Studio 1 - SSTray.stl", 0.001);

  //   fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

  //   REQUIRE(p.fixtures().size() == 2);
  // }

  TEST_CASE("Single Shot Tray") {
    arena_allocator a;
    set_system_allocator(&a);

    vice test_vice = large_jaw_vice(6, point(1.3, -4.4, 3.3));
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

    fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

    cout << "Number of steps = " << p.fixtures().size() << endl;

    REQUIRE(p.fixtures().size() == 3);

  }
  
}
