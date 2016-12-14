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

    point up_dir(0, 0, 1);

    for (unsigned i = 1; i < lines.size(); i++) {
      point last = pts.back();
      point safe_up_start = last + safe_z*up_dir;

      auto& next_line = lines[i];
      point safe_up_end = next_line.front() + safe_z*up_dir;

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

    polygon_3 outer_bound = exterior_offset(face, t.cut_diameter());

    vector<polyline> level =
      zig_lines(outer_bound, {}, t);
    auto lines =
      tile_vertical(level, start_depth, end_depth, f.depth_of_cut);

    cout << "Depth of cut = " << f.depth_of_cut << endl;
    cout << "# of levels = " << cut_depths(start_depth, end_depth, f.depth_of_cut).size() << endl;

    vector<polyline> connected_level{link_with_rapids(lines, safe_z)};

    return {toolpath(FACE_POCKET,
		     safe_z,
		     f.spindle_speed,
		     f.feedrate,
		     f.feedrate,
		     t,
		     connected_level)};

  }

}
