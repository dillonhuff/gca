#include "synthesis/cut.h"
#include "synthesis/linear_cut.h"
#include "synthesis/toolpath_generation.h"

namespace gca {

  vector<polyline> deepen_polyline(vector<double> depths, const polyline& p) {
    vector<polyline> ps;
    for (auto depth : depths) {
      vector<point> pts;
      for (auto pt : p)
	{ pts.push_back(point(pt.x, pt.y, depth)); }
      ps.push_back(polyline(pts));
    }
    return ps;
  }

  vector<double> cut_depths(double start_depth,
			    double end_depth,
			    double cut_depth) {
    vector<double> depths;
    double depth = max(end_depth, start_depth - cut_depth);
    while (true) {
      depths.push_back(depth);
      if (depth == end_depth) {
	break;
      }
      depth = max(end_depth, depth - cut_depth);
    }
    return depths;
  }

  vector<polyline> tile_vertical(const vector<polyline>& ps,
				 double start_depth,
				 double end_depth,
				 double cut_depth) {
    vector<polyline> vertical_slices;
    auto depths = cut_depths(start_depth, end_depth, cut_depth);
    for (auto p : ps) {
      auto cuts = deepen_polyline(depths, p);
      vertical_slices.insert(vertical_slices.end(), cuts.begin(), cuts.end());
    }
    stable_sort(vertical_slices.begin(), vertical_slices.end(),
    		[](const polyline& l, const polyline& r)
    		{ return l.front().z > r.front().z; });
    return vertical_slices;
  }

  vector<polyline> repeated_offsets(const polyline& p,
				    int num_repeats,
				    offset_dir d,
				    double inc) {
    assert(num_repeats > 0);
    vector<polyline> paths;
    paths.push_back(offset(p, d, inc));
    for (int i = 1; i < num_repeats; i++) {
      paths.push_back(offset(paths.back(), d, inc));
    }
    reverse(paths.begin(), paths.end());
    return paths;
  }

  // TODO: Add in tool and workpiece info to compute cut_depth,
  // the degree of the polyline and the number of offsets,
  // and the offset increment
  vector<polyline> pocket_2P5D_lines(const pocket_info_2P5D& pocket) {
    auto paths = repeated_offsets(pocket.outline,
				  1,
				  OFFSET_LEFT,
				  0.1);
    return tile_vertical(paths,
			 pocket.start_depth,
			 pocket.end_depth,
			 0.35);
  }

  cut* mk_cut(const point l, const point r) {
    auto c = linear_cut::make(l, r);
    c->set_spindle_speed(lit::make(3000));
    c->set_feedrate(lit::make(10));
    return c;
  }

  vector<cut*> polyline_cuts(const polyline& p) {
    auto ls = p.lines();
    assert(ls.size() > 0);
    vector<cut*> c;
    for (auto l : ls) {
      c.push_back(mk_cut(l.start, l.end));
    }
    return c;
  }

}
