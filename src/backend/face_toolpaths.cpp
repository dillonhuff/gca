#include "backend/face_toolpaths.h"
#include "backend/toolpath_generation.h"
#include "geometry/offset.h"

namespace gca {

  polyline link_with_rapids(const std::vector<polyline>& lines,
			    const double safe_z) {
    if (lines.size() == 0) { return polyline({}); }

    vector<point> pts;
    for (auto pt : lines.front()) {
      pts.push_back(pt);
    }

    for (unsigned i = 1; i < lines.size(); i++) {
      point last = pts.back();
      point safe_up_start(last.x, last.y, safe_z);

      auto& next_line = lines[i];
      point next_pt = next_line.front();
      point safe_up_end (next_pt.x, next_pt.y, safe_z);

      pts.push_back(safe_up_start);
      pts.push_back(safe_up_end);

      for (auto pt : next_line) {
	pts.push_back(pt);
      }
    }

    return polyline(pts);
  }

  toolpath rough_face(const face_parameters& f,
		      const double safe_z,
		      const double start_depth,
		      const double end_depth,
		      const polygon_3& face,
		      const tool& t) {

    // Really ought to test that the polygon is convex
    DBG_ASSERT(face.holes().size() == 0);

    polygon_3 outer_bound = exterior_offset(face, t.radius() + 0.05);

    DBG_ASSERT(f.width_of_cut <= t.radius());
    
    vector<polyline> level =
      zig_lines(outer_bound, {}, f.width_of_cut, t);
    auto lines =
      tile_vertical(level, start_depth, end_depth, f.depth_of_cut);

    cout << "Depth of cut = " << f.depth_of_cut << endl;
    cout << "# of levels = " << cut_depths(start_depth, end_depth, f.depth_of_cut).size() << endl;

    //    vector<polyline> connected_level{link_with_rapids(lines, safe_z)};

    return toolpath(FACE_POCKET,
		    safe_z,
		    f.spindle_speed,
		    f.feedrate,
		    f.feedrate,
		    t,
		    lines);

  }

}
