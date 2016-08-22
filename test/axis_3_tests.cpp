#include "catch.hpp"
#include "feature_recognition/feature_decomposition.h"
#include "process_planning/feature_to_pocket.h"
#include "synthesis/axis_3.h"
#include "synthesis/millability.h"
#include "utils/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  std::vector<std::vector<index_t>> make_top_surfaces(const triangular_mesh& mesh) {
    double normal_degrees_delta = 30.0;
    auto inds = millable_faces(point(0, 0, 1), mesh);
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, mesh, normal_degrees_delta);
    filter_vertical_surfaces(delta_regions, mesh);
    return delta_regions;
  }
  
  std::vector<pocket> make_top_pockets(const triangular_mesh& mesh,
				       const double workpiece_height) {
    vector<vector<index_t>> surfaces = make_top_surfaces(mesh);
    auto merged_surfaces = merge_connected_surfaces(surfaces, mesh);
    auto pockets = pockets_for_surfaces(merged_surfaces, workpiece_height, mesh);
    return pockets;
  }

  
  TEST_CASE("Make surfaces") {
    arena_allocator a;
    set_system_allocator(&a);

    SECTION("Mesh box plinth") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/MeshBoxPlinth.stl", 0.001);
      auto surfaces = make_top_surfaces(mesh);
      REQUIRE(surfaces.size() == 2);
    }
  }

  TEST_CASE("Pocketing") {
    arena_allocator a;
    set_system_allocator(&a);

    double workpiece_depth = 1.0;

    SECTION("CylinderSquare") {
      auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderSquare.stl", 0.001);
      feature_decomposition* f = build_feature_decomposition(mesh, point(0, 0, 1));
      vector<pocket> pockets = feature_pockets(*f);

      SECTION("Two pockets") {
	REQUIRE(pockets.size() == 2);
      }

      SECTION("One pocket ends at 0.200001") {
	REQUIRE(any_of(begin(pockets), end(pockets),
		       [](const pocket& p) {
			 return within_eps(p.get_end_depth(), 0.200001);
		       }));
      }

      SECTION("First pocket has one hole") {
      	REQUIRE(pockets.front().get_holes().size() == 1);
      }

      SECTION("Last pocket has no holes") {
      	REQUIRE(pockets.back().get_holes().size() == 0);
      }
    }

    SECTION("CylinderChimneySlot") {
      vector<triangle> triangles = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/CylinderChimneySlot.stl").triangles;
      auto mesh = make_mesh(triangles, 0.001);
      auto pockets = make_top_pockets(mesh, workpiece_depth);
      
      SECTION("4 pockets") {
    	REQUIRE(pockets.size() == 4);
      }
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
      auto pockets = make_top_pockets(mesh, workpiece_height);
      
      SECTION("2 pocket") {
    	REQUIRE(pockets.size() == 1);
      }

      SECTION("Pocketing does not overlap the pocket base") {
    	auto rough_lines = pockets.front().toolpath_lines(t, cut_depth);
    	auto rough_points = points(rough_lines);
    	bool all_above_pocket_surface =
    	  all_of(begin(rough_points), end(rough_points),
    		 [&pockets](const point p)
    		 { return pockets.front().above_base(p); });
    	REQUIRE(all_above_pocket_surface);
      }
    }
  }

}
