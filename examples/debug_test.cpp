#define CATCH_CONFIG_MAIN

#define CATCH_CONFIG_MAIN

#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/vtk_debug.h"
#include "synthesis/fixture_analysis.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

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
    t3.set_cut_diameter(0.12);
    t3.set_cut_length(1.2);

    t3.set_shank_diameter(0.5);
    t3.set_shank_length(0.05);

    t3.set_holder_diameter(2.5);
    t3.set_holder_length(3.5);

    // Ridiculous tool used to test feasability
    tool t4{0.2334, 3.94, 4, HSS, FLAT_NOSE};
    t4.set_cut_diameter(0.05);
    t4.set_cut_length(1.2);

    t4.set_shank_diameter(0.5);
    t4.set_shank_length(0.05);

    t4.set_holder_diameter(2.5);
    t4.set_holder_length(3.5);
    
    vector<tool> tools{t1, t2, t3, t4};

    vector<string> part_paths{
      "test/stl-files/onshape_parts//Part Studio 1 - Part 1(29).stl",
	};

    for (auto part_path : part_paths) { //passes) { //part_paths) {
      cout << "Part path: " << part_path << endl;

      auto mesh = parse_stl(part_path, 0.001);

      box bounding = mesh.bounding_box();

      cout << "Bounding box = " << endl;
      cout << bounding << endl;

      fixture_plan p = make_fixture_plan(mesh, fixes, tools, {workpiece_dims});

      cout << "Number of steps = " << p.fixtures().size() << endl;

      // Tentative number just to check the thru hole cutting
      DBG_ASSERT(p.fixtures().size() == 3);

    }
  }

}
