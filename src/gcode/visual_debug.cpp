#include "gcode/visual_debug.h"

#include "synthesis/visual_debug.h"

namespace gca {

  std::vector<polyline> cuts_to_polylines(const std::vector<cut*>& cuts) {
    vector<point> points;
    for (auto& c : cuts) {
      if (c->is_safe_move() || c->is_linear_cut()) {
	points.push_back(c->get_start());
	points.push_back(c->get_end());
      } else {
	double ind = 0;
	while (ind < 1) {
	  points.push_back(c->value_at(ind));
	  ind += 0.1;
	}
	points.push_back(c->get_end());
      }
    }
    return {{points}};
  }

  void vtk_debug_cuts(const std::vector<cut*>& cuts) {
    vector<polyline> lines = cuts_to_polylines(cuts);
    auto pd = polydata_for_polylines(lines);
    visualize_polydatas({pd});
  }

}
