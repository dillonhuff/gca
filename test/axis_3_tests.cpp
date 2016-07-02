#include "catch.hpp"
#include "synthesis/axis_3.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  TEST_CASE("Make surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Mesh box plinth") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MeshBoxPlinth.stl", 0.001);
      auto surfaces = make_surfaces(mesh);
      REQUIRE(surfaces.size() == 2);
    }
  }

  TEST_CASE("Triangle preprocessing") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("CylinderChimneySlot") {

      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl", 0.001);
      auto polygons = preprocess_triangles(mesh);
      stable_sort(begin(polygons), end(polygons),
		  [](const oriented_polygon& x,
		     const oriented_polygon& y)
		  { return x.height() < y.height(); });

      SECTION("One polygon per face") {
	REQUIRE(polygons.size() == 4);
      }

      SECTION("4 surfaces to mill") {
	auto face_inds = preprocess_faces(mesh);
	auto surfaces = merge_surfaces(face_inds, mesh);
	REQUIRE(surfaces.size() == 4);
      }

    }
  }
  
  TEST_CASE("Pocketing") {
    arena_allocator a;
    set_system_allocator(&a);

    double workpiece_depth = 1.0;

    SECTION("CylinderSquare") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl").triangles;
      auto mesh = make_mesh(triangles, 0.001);
      auto pockets = make_pockets(mesh, workpiece_depth);

      SECTION("Two pockets") {
	REQUIRE(pockets.size() == 2);
      }

      SECTION("One pocket ends at 0.200001") {
	REQUIRE(any_of(begin(pockets), end(pockets),
		       [](const pocket& p) {
			 return within_eps(p.get_end_depth(), 0.200001);
		       }));
      }

      SECTION("First pocket has no holes") {
	REQUIRE(pockets.front().get_holes().size() == 1);
      }

      SECTION("Last pocket has one hole") {
	REQUIRE(pockets.back().get_holes().size() == 0);
      }
    }

    SECTION("CylinderChimneySlot") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl").triangles;
      auto mesh = make_mesh(triangles, 0.001);
      auto pockets = make_pockets(mesh, workpiece_depth);
      
      SECTION("4 pockets") {
    	REQUIRE(pockets.size() == 4);
      }
    }

  }

  TEST_CASE("Milling surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    double workpiece_height = 1.0;

    SECTION("Rectangle cylinder") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl").triangles;
      auto mesh = make_mesh(triangles, 0.001);
      double cut_depth = 0.25;
      tool t1(0.05, 3.0, 2, HSS, FLAT_NOSE);
      auto mill_paths = mill_surface(mesh,
				     t1,
				     cut_depth,
				     workpiece_height);
      REQUIRE(mill_paths.size() > 0);
    }

    SECTION("Multiple cylinders") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MultipleCylinders.stl").triangles;
      auto mesh = make_mesh(triangles, 0.001);
      tool t1(0.2, 3.0, 4, HSS, FLAT_NOSE);
      double cut_depth = 0.5;
      auto mill_lines = mill_surface_lines(mesh, t1, cut_depth, workpiece_height);
      REQUIRE(mill_lines.size() > 0);
    }
  }

  TEST_CASE("Milling curved surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    double workpiece_height = 0.5;
    tool t(0.075*2, 3.0, 2, HSS, FLAT_NOSE);
    double cut_depth = 0.1;

    SECTION("SlicedCone") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/SlicedCone.stl").triangles;
      auto mesh = make_mesh(triangles, 0.001);
      auto pockets = make_pockets(mesh, workpiece_height);
      
      SECTION("2 pocket") {
    	REQUIRE(pockets.size() == 1);
      }

      SECTION("Roughing does not overlap the pocket base") {
    	auto rough_lines = rough_pocket(pockets.front(), t, cut_depth);
    	auto rough_points = points(rough_lines);
    	bool all_above_pocket_surface =
    	  all_of(begin(rough_points), end(rough_points),
    		 [&pockets](const point p)
    		 { return pockets.front().above_base(p); });
    	REQUIRE(all_above_pocket_surface);
      }
    }
  }

  bool intersects(const line l, const triangular_mesh& mesh) {
    for (auto i : mesh.face_indexes()) {
      auto t = mesh.face_triangle(i);
      if (intersects(t, l)) {
	return true;
      }
    }
    return false;
  }

  bool all_above(const std::vector<polyline>& pls,
		 const tool t,
		 const triangular_mesh& mesh,
		 const double shift) {
    // The shift is to undo tool radius compensation
    for (auto pl : shift_lines(pls, point(0, 0, -t.length()))) {
      if (any_between(begin(pl), end(pl),
		       [mesh](const point a, const point b) {
			 return intersects(line(a, b), mesh);
		       })) {
	return false;
      }
    }
    return true;
  }

  // TODO: Reintroduce this test
  // TEST_CASE("Toolpaths do not overlap the mesh") {
  //   arena_allocator a;
  //   set_system_allocator(&a);

  //   SECTION("MeshBoxPlinth") {
  //     auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MeshBoxPlinth.stl", 0.001);
  //     tool t(0.32, 3.0, FLAT_NOSE);
  //     double cut_depth = 0.2;
  //     double workpiece_height = 1.7;

  //     SECTION("2 pockets") {
  // 	auto pockets = make_pockets(mesh, workpiece_height);
  // 	REQUIRE(pockets.size() == 2);
  //     }

  //     SECTION("Toolpaths dont overlap the mesh") {
  // 	auto toolpaths = mill_surface_lines(mesh, t, cut_depth, workpiece_height);
  // 	REQUIRE(all_above(toolpaths, t, mesh, -0.1));
  //     }
  //   }
  // }

}
