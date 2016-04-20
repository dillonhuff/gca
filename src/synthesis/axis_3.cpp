#include "geometry/polygon.h"
#include "synthesis/axis_3.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

namespace gca {

  vector<block> polylines_cuts(const vector<polyline>& pocket_lines,
			       const cut_params params) {
    vector<cut*> cuts;
    for (auto p : pocket_lines) {
      auto cs = polyline_cuts(p);
      cuts.insert(cuts.end(), cs.begin(), cs.end());
    }
    return cuts_to_gcode(cuts, params);
  }

  polyline extract_part_base_outline(const vector<triangle>& tris) {
    auto triangles = tris;
    delete_if(triangles, [](const triangle& t)
	      { return !within_eps(t.normal, point(0, 0, -1), 1e-2); });
    auto outlines = merge_triangles(triangles);
    assert(outlines.size() == 1);
    auto base_outline = outlines.front();
    vector<point> vertices = base_outline.vertices;
    vertices.push_back(vertices.front());
    return polyline(vertices);
  }

  vector<block> emco_f1_code(const vector<polyline>& pocket_lines) {
    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = (*pocket_lines.front().begin()).z + 0.05;
    return polylines_cuts(pocket_lines, params);
  }

  box bounding_box(const oriented_polygon& p) {
    return bound_positions(p.vertices);
  }

  template<typename InputIt>
  bool contained_by_any(point p, InputIt l, InputIt r) {
    while (l != r) {
      if (contains(*l, p)) {
	return true;
      }
      ++l;
    }
    return false;
  }

  bool overlaps(line l, const oriented_polygon& p) {
    polyline pl(p.vertices);
    for (auto pll : pl.lines()) {
      if (segment_intersection_2d(l, pll).just)
	{ return true; }
    }
    return false;
  }

  template<typename InputIt>
  bool overlaps_any(line l, InputIt s, InputIt e) {
    while (s != e) {
      if (overlaps(l, *s)) {
	return true;
      }
      ++s;
    }
    return false;
  }

  oriented_polygon exterior_offset(const oriented_polygon& p,
				   double inc) {
    auto vs = p.vertices;
    vs.push_back(p.vertices.front());
    polyline pl(vs);
    auto off_l = offset(pl, exterior_direction(pl), inc);
    vector<point> pts(begin(off_l), end(off_l));
    return oriented_polygon(p.normal, pts);
  }

  oriented_polygon interior_offset(const oriented_polygon& p,
				   double inc) {
    auto vs = p.vertices;
    vs.push_back(p.vertices.front());
    polyline pl(vs);
    auto off_l = offset(pl, interior_direction(pl), inc);
    vector<point> pts(begin(off_l), end(off_l));
    return oriented_polygon(p.normal, pts);
  }

  vector<polyline> roughing_lines(const vector<oriented_polygon>& holes,
				  const vector<oriented_polygon>& boundaries,
				  double last_level) {
    double sample_increment = 0.05;
    box b = bounding_box(begin(boundaries), end(boundaries));
    // TODO: Select sample rate from tool_diameter
    auto toolpath_points = sample_points_2d(b,
					    sample_increment,
					    sample_increment,
					    last_level);
    delete_if(toolpath_points,
    	      [&holes](const point p)
    	      { return contained_by_any(p, begin(holes), end(holes)); });
    delete_if(toolpath_points,
	      [&boundaries](const point p)
	      { return !contained_by_any(p, begin(boundaries), end(boundaries)); });
    assert(toolpath_points.size() > 0);
    vector<vector<point>> lpts;
    split_by(toolpath_points, lpts,
	     [&holes](const point l, const point r)
	     { return !overlaps_any(line(l, r), begin(holes), end(holes)); });
    vector<polyline> lines;
    for (auto ls : lpts) {
      lines.push_back(ls);
    }
    return lines;
  }

  // TODO: Add tool diameter parameter
  template<typename InputIt>
  vector<polyline> level_roughing(InputIt s,
				  InputIt m,
				  InputIt e,
				  double last_level) {
    double tool_radius = 0.01;
    vector<oriented_polygon> offset_h(distance(s, m));
    transform(s, m, begin(offset_h),
	      [tool_radius](const oriented_polygon& p)
	      { return exterior_offset(p, tool_radius); });
    vector<oriented_polygon> bound_polys(distance(m, e));
    transform(m, e, begin(bound_polys),
	      [tool_radius](const oriented_polygon& p)
	      { return interior_offset(p, tool_radius); });
    vector<oriented_polygon> offset_holes;
    for (auto hole : offset_h) {
      bool contained_by_bound = false;
      for (auto b : bound_polys) {
	if (contains(b, hole)) {
	  contained_by_bound = true;
	  break;
	}
      }
      if (contained_by_bound) {
	offset_holes.push_back(hole);
      }
    }
    double bottom = bound_polys.front().pt(0).z;
    double cut_depth = 0.05;
    vector<polyline> rough_pass = roughing_lines(offset_holes, bound_polys, last_level);
    return tile_vertical(rough_pass, last_level, bottom, cut_depth);
  }

  vector<oriented_polygon> preprocess_triangles(vector<triangle>& triangles) {
    delete_if(triangles,
	      [](const triangle t)
	      { return !is_upward_facing(t, 0.01); });
    auto polygons = merge_triangles(triangles);
    stable_sort(begin(polygons), end(polygons),
		[](const oriented_polygon& x,
		   const oriented_polygon& y)
		{ return x.vertices.front().z > y.vertices.front().z; });
    return polygons;
  }

  vector<polyline> mill_surface_lines(vector<oriented_polygon> polygons,
				      double tool_diameter) {
    double start_depth = polygons.front().vertices.front().z;
    double workpiece_height = start_depth + 0.1;
    double last_level = start_depth;
    auto below_level = begin(polygons);
    vector<polyline> pocket_lines;
    while (below_level != end(polygons)) {
      auto level_rough = level_roughing(begin(polygons),
					below_level,
					end(polygons),
					workpiece_height);
      pocket_lines.insert(end(pocket_lines), begin(level_rough), end(level_rough));
      below_level = find_if(below_level, end(polygons),
			    [last_level](const oriented_polygon& p)
			    { return !within_eps(p.vertices.front().z, last_level, 0.01); });
      if (below_level != end(polygons)) {
	workpiece_height = last_level;
	last_level = (*below_level).vertices.front().z;
      }
    }
    return pocket_lines;
  }

  vector<polyline> mill_surface_lines(vector<triangle>& triangles,
				      double tool_diameter) {
    auto polygons = preprocess_triangles(triangles);
    return mill_surface_lines(polygons, tool_diameter);
  }

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter) {
    auto pocket_lines = mill_surface_lines(triangles, tool_diameter);
    return emco_f1_code(pocket_lines);
  }

}
