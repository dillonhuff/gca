#include "geometry/surface.h"
#include "process_planning/axis_location.h"
#include "utils/check.h"

namespace gca {

  point part_axis(const triangular_mesh& m) {
    cout << "Part axis " << endl;

    vector<surface> regions = inds_to_surfaces(const_orientation_regions(m), m);

    // Note: Should really filter duplicates here
    vector<double> ortho_areas;
    for (auto r : regions) { ortho_areas.push_back(0.0); }

    for (unsigned i = 0; i < regions.size(); i++) {
      const auto& ri = regions[i];
      point ni = normal(ri);
      for (unsigned j = 0; j < regions.size(); j++) {
	const auto& rj = regions[j];
	//	if (within_eps(angle_between(normal(regions[i]), normal(regions[j])), 0.0, 1.0)) {
	if (rj.orthogonal_to(ni, 1.0) ||
	    rj.parallel_to(ni, 1.0) ||
	    rj.antiparallel_to(ni, 1.0)) {
	  ortho_areas[i] += regions[j].surface_area();
	}
      }
    }

    auto max_region = max_element(begin(ortho_areas), end(ortho_areas));
    cout << "Max area = " << *max_region << endl;

    unsigned d = std::distance(begin(ortho_areas), max_region);

    cout << "Max area index = " << d << endl;

    
    return normal(regions[d]);
  }

}
