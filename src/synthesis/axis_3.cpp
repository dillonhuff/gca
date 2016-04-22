#include "geometry/polygon.h"
#include "synthesis/axis_3.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

namespace gca {

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
  
  vector<block> polylines_cuts(const vector<polyline>& pocket_lines,
			       const cut_params params) {
    vector<cut*> cuts;
    for (auto p : pocket_lines) {
      auto cs = polyline_cuts(p);
      cuts.insert(cuts.end(), cs.begin(), cs.end());
    }
    return cuts_to_gcode(cuts, params);
  }

  vector<block> emco_f1_code(const vector<polyline>& pocket_lines) {
    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = (*pocket_lines.front().begin()).z + 0.05;
    return polylines_cuts(pocket_lines, params);
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

  // TODO: Add tool diameter parameter
  // template<typename InputIt>
  // vector<polyline> level_roughing(InputIt s,
  // 				  InputIt m,
  // 				  InputIt e,
  // 				  double last_level,
  // 				  double tool_radius) {
  //   vector<oriented_polygon> offset_h(distance(s, m));
  //   transform(s, m, begin(offset_h),
  // 	      [tool_radius](const oriented_polygon& p)
  // 	      { return exterior_offset(p, tool_radius); });
  //   vector<oriented_polygon> bound_polys(distance(m, e));
  //   transform(m, e, begin(bound_polys),
  // 	      [tool_radius](const oriented_polygon& p)
  // 	      { return interior_offset(p, tool_radius); });
  //   vector<oriented_polygon> offset_holes;
  //   for (auto hole : offset_h) {
  //     bool contained_by_bound = false;
  //     for (auto b : bound_polys) {
  // 	if (contains(b, hole)) {
  // 	  contained_by_bound = true;
  // 	  break;
  // 	}
  //     }
  //     if (contained_by_bound) {
  // 	offset_holes.push_back(hole);
  //     }
  //   }
  //   double bottom = bound_polys.front().pt(0).z;
  //   double cut_depth = 0.05;
  //   vector<polyline> rough_pass = roughing_lines(offset_holes, bound_polys, last_level, tool_radius);
  //   return tile_vertical(rough_pass, last_level, bottom, cut_depth);
  // }

  template<typename InputIt>
  pocket level_pocket(InputIt s,
		      InputIt m,
		      InputIt e,
		      double last_level) {
    vector<oriented_polygon> holes(s, m);
    vector<oriented_polygon> bounds(m, e);
    double bottom = bounds.front().pt(0).z;
    assert(bottom < last_level);
    return pocket(bounds, holes, last_level, bottom);
  }

  vector<pocket> make_pockets(vector<oriented_polygon> polygons) {
    vector<pocket> pockets;
    double start_depth = polygons.front().vertices.front().z;
    double workpiece_height = start_depth + 0.1;
    double last_level = start_depth;
    auto below_level = begin(polygons);
    while (below_level != end(polygons)) {
      auto pocket = level_pocket(begin(polygons),
				 below_level,
				 end(polygons),
				 workpiece_height);
      pockets.push_back(pocket);
      below_level = find_if(below_level, end(polygons),
  			    [last_level](const oriented_polygon& p)
  			    { return !within_eps(p.vertices.front().z, last_level, 0.01); });
      if (below_level != end(polygons)) {
  	workpiece_height = last_level;
  	last_level = (*below_level).vertices.front().z;
      }
    }
    return pockets;
  }

  vector<polyline> mill_pockets(vector<pocket>& pockets,
				double tool_diameter) {
    vector<polyline> lines;
    double tool_radius = tool_diameter / 2.0;
    for (auto pocket : pockets) {
      auto pocket_paths = pocket_2P5D_interior(pocket, tool_radius);
      lines.insert(end(lines), begin(pocket_paths), end(pocket_paths));
    }
    return lines;
  }

  vector<polyline> mill_surface_lines(vector<triangle>& triangles,
				      double tool_diameter) {
    auto polygons = preprocess_triangles(triangles);
    auto pockets = make_pockets(polygons);
    return mill_pockets(pockets, tool_diameter); //mill_surface_lines(polygons, tool_diameter);
  }

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter) {
    auto pocket_lines = mill_surface_lines(triangles, tool_diameter);
    return emco_f1_code(pocket_lines);
  }

  vector<pocket> surface_finishes(vector<triangle>& triangles) {
    vector<pocket> pockets;
    return pockets;
  }

}
