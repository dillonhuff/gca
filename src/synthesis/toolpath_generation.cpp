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


  // Change name to reflect roughing and finishing
  vector<polyline> roughing_lines(const vector<oriented_polygon>& holes,
  				  const vector<oriented_polygon>& boundaries,
  				  double last_level,
  				  double tool_radius) {
    double sample_increment = tool_radius;
    box b = bounding_box(begin(boundaries), end(boundaries));
    // TODO: Select sample rate from tool_diameter
    auto toolpath_points = sample_points_2d(b,
  					    sample_increment,
  					    sample_increment,
  					    last_level);
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
    //Insert finishing lines
    for (auto bound : boundaries) {
      polyline p(bound.vertices);
      lines.push_back(p);
    }
    for (auto hole : holes) {
      polyline p(hole.vertices);
      lines.push_back(p);
    }

    return lines;
  }

  vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					double tool_radius) {
    auto bounds = pocket.get_boundaries();
    auto holes = pocket.get_holes();
    vector<oriented_polygon> offset_h(holes.size());
    transform(begin(holes), end(holes), begin(offset_h),
  	      [tool_radius](const oriented_polygon& p)
  	      { return exterior_offset(p, tool_radius); });
    vector<oriented_polygon> bound_polys(bounds.size());
    transform(begin(bounds), end(bounds), begin(bound_polys),
  	      [tool_radius](const oriented_polygon& p)
  	      { return interior_offset(p, tool_radius); });
    // vector<oriented_polygon> offset_holes;
    // for (auto hole : offset_h) {
    //   bool contained_by_bound = false;
    //   for (auto b : bound_polys) {
    // 	if (contains(b, hole)) {
    // 	  contained_by_bound = true;
    // 	  break;
    // 	}
    //   }
    //   if (contained_by_bound) {
    // 	offset_holes.push_back(hole);
    //   }
    // }
    // TODO: Make this an input parameter
    double cut_depth = 0.1;
    vector<polyline> rough_pass = roughing_lines(offset_h, bound_polys, pocket.start_depth, tool_radius);
    return tile_vertical(rough_pass, pocket.start_depth, pocket.end_depth, cut_depth);
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
