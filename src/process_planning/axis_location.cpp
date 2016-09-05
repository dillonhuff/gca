#include <unordered_map>

#include "geometry/surface.h"
#include "process_planning/axis_location.h"
#include "utils/check.h"

namespace gca {

  point part_axis(const triangular_mesh& m) {
    cout << "Part axis " << endl;

    vector<surface> regions = inds_to_surfaces(const_orientation_regions(m), m);

    // Note: Should really filter duplicates here
    // vector<double> ortho_areas;
    // for (auto r : regions) { ortho_areas.push_back(0.0); }

    unordered_map<surface*, double> ortho_areas;
    for (surface& r : regions) { ortho_areas[&r] = 0.0; }

    // for (unsigned i = 0; i < regions.size(); i++) {
    //   auto& ri = regions[i];
    //   point ni = normal(ri);
    //   for (unsigned j = 0; j < regions.size(); j++) {
    // 	auto& rj = regions[j];



    for (surface& ri : regions) {
      point ni = normal(ri);
      for (surface& rj : regions) {
	if (rj.orthogonal_to(ni, 1.0) ||
	    rj.parallel_to(ni, 1.0) ||
	    rj.antiparallel_to(ni, 1.0)) {
	  DBG_ASSERT(ortho_areas.find(&ri) != end(ortho_areas));

	  ortho_areas[&ri] = ortho_areas[&ri] + rj.surface_area();
	}
      }
    }

    auto max_region = //max_e(begin(ortho_areas), end(ortho_areas));
      max_element(begin(ortho_areas),
		  end(ortho_areas),
		  [](const std::pair<surface*, double>& l,
		     const std::pair<surface*, double>& r) {
		    return l.second < r.second;
		  });

    cout << "Max area = " << max_region->second << endl;

    return normal(*(max_region->first));

    // unsigned d = std::distance(begin(ortho_areas), max_region);

    // cout << "Max area index = " << d << endl;
    
    // return normal(regions[d]);
  }

}
