#include "catch.hpp"
#include "synthesis/fixture_analysis.h"
#include "system/arena_allocator.h"
#include "system/parse_stl.h"

namespace gca {

  bool surfaces_share_edge(const unsigned i,
			   const unsigned j,
			   const std::vector<surface>& surfaces) {
    auto ind1 = surfaces[i].index_list();
    auto ind2 = surfaces[j].index_list();
    return share_edge(ind1, ind2, surfaces[i].get_parent_mesh());
  }

  TEST_CASE("Complex rectangular part") {
    arena_allocator a;
    set_system_allocator(&a);
    vice test_vice = emco_vice(point(-0.8, -4.4, -3.3));
    workpiece workpiece_dims(3.5, 2.5, 2.3);
    auto mesh = parse_stl("/Users/dillon/CppWorkspace/gca/test/stl-files/ComplexRectanglePart1.stl", 0.001);
    
    // SECTION("surface allocation requires 4 arrangements") {
    //   auto outer_surfs = outer_surfaces(mesh);
    //   auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
    //   classify_part_surfaces(outer_surfs, aligned_workpiece);
    //   vector<pair<stock_orientation, surface_list>> meshes =
    // 	orientations_to_cut(mesh, outer_surfs);
      
    // }

    SECTION("4 setups, no duplicates, each orientation has 1 connected component") {
      auto outer_surfs = outer_surfaces(mesh);
      auto aligned_workpiece = align_workpiece(outer_surfs, workpiece_dims);
      classify_part_surfaces(outer_surfs, aligned_workpiece);

      auto surfs_to_cut = surfaces_to_cut(mesh, outer_surfs);
      cout << "$$$ " << surfs_to_cut.size() << " surfaces to cut" << endl;
      vector<stock_orientation> all_orients =
	all_stable_orientations(outer_surfs);
      
      vector<pair<stock_orientation, vector<unsigned>>> orients =
	pick_orientations(mesh, surfs_to_cut, all_orients);

      // TEST: 4 setups
      REQUIRE(orients.size() == 4);


      // NO DUPLICATION
      for (unsigned i = 0; i < orients.size(); i++) {
	for (unsigned j = i; j < orients.size(); j++) {
	  vector<unsigned> ig = orients[i].second;
	  vector<unsigned> jg = orients[j].second;
	  REQUIRE(intersection(ig, jg).size() == 0);
	}
      }
      
      // TEST: Each has 1 connected component
      for (auto p : orients) {
	cout << "$$$ checking orientation " << p.first.top_normal() << endl;
	auto surface_inds = p.second;
	cout << "with " << surface_inds.size() << " surfaces" << endl;
	auto connected_comps =
	  connected_components_by(surface_inds,
				  [surfs_to_cut](const unsigned i, const unsigned j)
				  {
				    return surfaces_share_edge(i, j, surfs_to_cut);
				  });
	REQUIRE(connected_comps.size() == 1);
      }
    }
  }
}
