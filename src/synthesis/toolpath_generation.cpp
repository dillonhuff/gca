#include <cmath>

#include "synthesis/cut.h"
#include "synthesis/linear_cut.h"
#include "synthesis/shape_layout.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

namespace gca {

  template<typename T>
  int num_elems(vector<vector<T>> v) {
    int i = 0;
    for (auto t : v) {
      i += t.size();
    }
    return i;
  }

  vector<polyline> deepen_polyline(const vector<double>& depths, const polyline& p) {
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
  bool overlaps_or_intersects_any(line l,
				  const vector<triangle>& base,
				  InputIt s,
				  InputIt e) {
    for (auto t : base) {
      if (intersects(t, l)) { return true; } //below(t, l.start) != below(t, l.end)) { return true; }
    }
    if (any_of(s, e,
	       [l](const oriented_polygon& p)
	       { return overlaps(l, p); })) {
      return true;
    }
    return any_of(s, e,
		  [l](const oriented_polygon& p)
		  { return contains(p, l.end) || contains(p, l.start); });
  }

  vector<polyline> finish_passes(const vector<oriented_polygon>& holes,
				 const vector<oriented_polygon>& boundaries,
				 vector<double> depths,
				 double tool_radius) {
    vector<polyline> lines;
    //Insert finishing lines
    for (auto bound : boundaries) {
      polyline p(bound.vertices);
      lines.push_back(p);
    }
    for (auto hole : holes) {
      polyline p(hole.vertices);
      lines.push_back(p);
    }
    // Change to tile vertical
    return lines;
  }

  vector<polyline> finish_pocket(const pocket& pocket,
				 double tool_radius,
				 double cut_depth) {
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
    vector<double> depths = cut_depths(pocket.get_start_depth(),
				       pocket.get_end_depth(),
				       cut_depth);
    return finish_passes(offset_h,
			 bound_polys,
			 depths,
			 tool_radius);
  }

  vector<line> make_lines(const vector<point>& pts) {
    assert(pts.size() > 1);
    vector<line> l;
    for (auto pt = begin(pts); pt != end(pts) - 1; ++pt) {
      l.push_back(line(*pt, *(pt + 1)));
    }
    return l;
  }

  // Change name to reflect roughing and finishing
  vector<polyline> roughing_lines(const vector<triangle>& base,
				  const vector<oriented_polygon>& holes,
  				  const vector<oriented_polygon>& boundaries,
  				  double last_level,
  				  double tool_radius) {
    double sample_increment = tool_radius;
    box b = bounding_box(begin(boundaries), end(boundaries));
    auto not_in_safe_region = [&base, &holes, &boundaries](const point p) {
      bool in_hole = any_of(begin(holes), end(holes),
		       [p](const oriented_polygon& pl)
      { return contains(pl, p); });
      if (in_hole) { return true; }
      bool outside_bounds = !any_of(begin(boundaries), end(boundaries),
				    [p](const oriented_polygon& pl)
      { return contains(pl, p); });
      if (outside_bounds) { return true; }
      for (auto t : base) {
	if (in_projection(t, p) && below(t, p)) { return true; }
      }
      return false;
    };
    auto toolpath_points = sample_filtered_points_2d(b,
						     sample_increment,
						     sample_increment,
						     last_level,
						     not_in_safe_region);
    auto overlaps =
      [&base, &holes](const line l)
      { return overlaps_or_intersects_any(l, 
					  base,
					  begin(holes),
					  end(holes)); };
    // vector<vector<point>> lpts;
    // split_by(toolpath_points, lpts,
    // 	     [&base, &holes](const point l, const point r)
    // 	     { return !overlaps_or_intersects_any(line(l, r), base, begin(holes), end(holes)); });
    vector<polyline> lines;
    if (toolpath_points.size() > 1) {
      auto path_lines = make_lines(toolpath_points);
      cout << "# of lines before = " << path_lines.size() << endl;
      delete_if(path_lines, overlaps);
      cout << "# of lines left = " << path_lines.size() << endl;
      for (auto ls : path_lines) {
	vector<point> pts{ls.start, ls.end};
	lines.push_back(polyline(pts));
      }
    }
    return lines;
  }

  vector<polyline> roughing_passes(const vector<triangle>& base,
				   const vector<oriented_polygon>& holes,
				   const vector<oriented_polygon>& boundaries,
				   vector<double> depths,
				   double tool_radius) {
    vector<polyline> lines;
    for (auto depth : depths) {
      auto rough_level = roughing_lines(base, holes, boundaries, depth, tool_radius);
      lines.insert(end(lines), begin(rough_level), end(rough_level));
    }
    return lines;
  }

  vector<polyline> rough_pocket(const pocket& pocket,
				double tool_radius,
				double cut_depth) {
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
    vector<double> depths = cut_depths(pocket.get_start_depth(),
				       pocket.get_end_depth(),
				       cut_depth);
    vector<polyline> pocket_path = roughing_passes(pocket.base,
						   offset_h,
						   bound_polys,
						   depths,
						   tool_radius);
    return pocket_path;
  }

  vector<polyline> pocket_2P5D_interior(const pocket& pocket,
					double tool_radius,
					double cut_depth) {
    vector<polyline> pocket_path = rough_pocket(pocket, tool_radius, cut_depth);
    auto finish_paths = finish_pocket(pocket, tool_radius, cut_depth);
    pocket_path.insert(end(pocket_path), begin(finish_paths), end(finish_paths));
    return pocket_path;
  }

  // TODO: Move these to somewhere else, they really dont belong here

  
  // TODO: Make the spindle_speed and feedrate parameters explicit
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
