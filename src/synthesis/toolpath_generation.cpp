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
    if (intersects_triangles(l, base)) { return true; }
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

  bool not_in_safe_region(const point p,
			  const vector<triangle>& base,
			  const vector<oriented_polygon>& holes,
			  const vector<oriented_polygon>& boundaries) {
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
  }

  vector<polyline> roughing_lines(const vector<triangle>& base,
				  const vector<oriented_polygon>& holes,
  				  const vector<oriented_polygon>& boundaries,
  				  double last_level,
  				  double tool_radius) {
    double sample_increment = tool_radius;
    assert(boundaries.size() == 1);
    box b = bounding_box(begin(boundaries), end(boundaries));
    auto not_safe = [&base, &holes, &boundaries](const point p)
      { return not_in_safe_region(p, base, holes, boundaries); };
    auto toolpath_points = sample_filtered_points_2d(b,
						     sample_increment,
						     sample_increment,
						     last_level,
						     not_safe);
    auto overlaps =
      [&base, &holes](const line l)
      { return overlaps_or_intersects_any(l, 
    					  base,
    					  begin(holes),
    					  end(holes)); };
    vector<polyline> lines;
    if (toolpath_points.size() > 1) {
      auto path_lines = make_lines(toolpath_points);
      delete_if(path_lines, overlaps);
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
				   // Make this ref
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
				       cut_depth / 2.0);
    vector<polyline> pocket_path = roughing_passes(pocket.base,
						   offset_h,
						   bound_polys,
						   depths,
						   tool_radius);
    return pocket_path;
  }

  vector<polyline> rough_pockets(const vector<pocket>& pockets,
				 double tool_radius,
				 double cut_depth) {
    vector<polyline> ps;
    for (auto pocket : pockets) {
      auto ls = rough_pocket(pocket, tool_radius, cut_depth);
      ps.insert(end(ps), begin(ls), end(ls));
    }
    return ps;
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

  bool same_slope(const line l, const line r, double t) {
    return within_eps(angle_between(l.end - l.start, r.end - r.start), 0, t);
  }

  polyline compress_lines(const polyline& p, double tolerance) {
    assert(p.num_points() > 1);
    if (p.num_points() == 2) { return p; }
    vector<vector<line>> slope_groups;
    split_by(p.lines(), slope_groups,
	     [tolerance](const line l, const line r)
	     { return same_slope(l, r, tolerance); });
    vector<point> pts;
    for (auto slope_group : slope_groups) {
      pts.push_back(slope_group.front().start);
      pts.push_back(slope_group.back().end);
    }
    return polyline(pts);
  }

  std::vector<polyline> rough_box(const box b,
				  double tool_radius,
				  double cut_depth) {
    vector<point> pts = sample_points_2d(b,
					 tool_radius / 2.0,
					 tool_radius / 2.0,
					 0.0);

    vector<vector<point>> pt_lines;
    split_by(pts, pt_lines,
	     [](const point l, const point r)
	     { return within_eps(l.x, r.x); });
    vector<polyline> lines;
    for (auto pt_group : pt_lines) {
      lines.push_back(pt_group);
    }

    auto final_lines = tile_vertical(lines,
				     b.z_max,
				     b.z_min,
				     cut_depth);
    return final_lines;
  }

  std::vector<polyline> drop_sample(const std::vector<triangle>& triangles,
				    double tool_radius) {
    auto mesh = make_mesh(triangles, 0.01);

    box b = mesh.bounding_box();
    b.x_min += 0.01;
    b.y_min += 0.01;
    b.z_min += 0.01;

    vector<point> pts_z = sample_points_2d(b, tool_radius, tool_radius, 1.0);

    vector<point> pts;
    for (auto pt : pts_z) {
      maybe<double> za = mesh.z_at(pt.x, pt.y);
      if (za.just) {
	pts.push_back(point(pt.x, pt.y, za.t)); //mesh.z_at(pt.x, pt.y)));
      }
    }

    vector<polyline> lines;
    lines.push_back(pts);
    return lines;
  }

  std::vector<polyline> drop_sample(const triangular_mesh& mesh,
				    double tool_radius) {
    return drop_sample(mesh.triangle_list(), tool_radius);
  }

  
}
