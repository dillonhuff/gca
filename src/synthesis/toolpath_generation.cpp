#include <cmath>

#include "synthesis/cut.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shape_layout.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

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

  // TODO: Add in tool and workpiece info to compute cut_depth
  // and the offset increment
  // vector<polyline> pocket_2P5D_exterior(const pocket& pocket) {
  //   offset_dir d = exterior_direction(pocket.get_outline());
  //   auto paths = repeated_offsets(pocket.get_outline(),
  // 				  1,
  // 				  d,
  // 				  0.1);
  //   return tile_vertical(paths,
  // 			 pocket.start_depth,
  // 			 pocket.end_depth,
  // 			 0.35);
  // }

  template<typename InputIt>
  bool overlaps_or_intersects_any(line l, InputIt s, InputIt e) {
    if (any_of(s, e,
	       [l](const oriented_polygon& p)
	       { return overlaps(l, p); })) {
      return true;
    }
    return any_of(s, e,
		  [l](const oriented_polygon& p)
		  { return contains(p, l.end) || contains(p, l.start); });
  }

  
  vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					double tool_radius) {
    auto boundaries = pocket.get_boundaries();
    auto holes = pocket.get_holes();
    double sample_increment = tool_radius;
    box b = bounding_box(begin(boundaries), end(boundaries));
    // TODO: Select sample rate from tool_diameter
    auto toolpath_points = sample_points_2d(b,
					    sample_increment,
					    sample_increment,
					    pocket.start_depth);
    delete_if(toolpath_points,
    	      [&holes](const point p)
    	      { return any_of(begin(holes), end(holes),
			      [p](const oriented_polygon& pl)
			      { return contains(pl, p); }); });
    delete_if(toolpath_points,
	      [&boundaries](const point p)
	      { return !any_of(begin(boundaries), end(boundaries),
			       [p](const oriented_polygon& pl)
			       { return contains(pl, p); }); });
    assert(toolpath_points.size() > 0);
    vector<vector<point>> lpts;
    split_by(toolpath_points, lpts,
	     [&holes](const point l, const point r)
	     { return !overlaps_or_intersects_any(line(l, r), begin(holes), end(holes)); });
    vector<polyline> lines;
    for (auto ls : lpts) {
      lines.push_back(ls);
    }
    // Insert finishing lines
    for (auto bound : boundaries) {
      polyline p(bound.vertices);
      lines.push_back(p);
    }
    return lines;
    
    // assert(tool_diameter > 0.0);
    // offset_dir dir = interior_direction(pocket.get_outline());
    // double tool_radius = tool_diameter / 2.0;
    // double tool_surface_area = M_PI * tool_radius * tool_radius;
    // vector<polyline> paths;
    // polyline off = offset(pocket.get_outline(),
    // 			  dir,
    // 			  tool_radius);
    // while (area(off) > tool_surface_area) {
    //   paths.push_back(off);
    //   off = offset(off,
    // 		   dir,
    // 		   tool_radius);
    // }
    // assert(paths.size() > 0);
    // return tile_vertical(paths,
    // 			 pocket.start_depth,
    // 			 pocket.end_depth,
    // 			 0.35);
  }


  cut* mk_cut(const point l, const point r) {
    auto c = linear_cut::make(l, r);
    c->set_spindle_speed(lit::make(3000));
    c->set_feedrate(lit::make(6.4));
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
