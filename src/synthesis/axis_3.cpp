#include "geometry/polygon.h"
#include "synthesis/axis_3.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/toolpath_generation.h"
#include "system/algorithm.h"

namespace gca {

  void select_visible_triangles(vector<triangle>& triangles) {
    delete_if(triangles,
	      [](const triangle t)
	      { return !is_upward_facing(t, 0.05); });
  }

  vector<oriented_polygon> preprocess_triangles(vector<triangle>& triangles) {
    select_visible_triangles(triangles);
    auto polygons = mesh_bounds(triangles);
    // TODO: Use polygon height member function
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
    auto outlines = mesh_bounds(triangles);
    assert(outlines.size() == 1);
    auto base_outline = outlines.front();
    vector<point> vertices = base_outline.vertices;
    vertices.push_back(vertices.front());
    return polyline(vertices);
  }

  template<typename InputIt>
  pocket level_pocket(InputIt s,
		      InputIt m,
		      InputIt e,
		      double last_level) {
    vector<oriented_polygon> possible_holes(s, m);
    vector<oriented_polygon> possible_bounds(m, e);
    double bottom = (*max_element(begin(possible_bounds),
				 end(possible_bounds),
				  [](const oriented_polygon& x,
				     const oriented_polygon& y)
      { return x.pt(0).z < y.pt(0).z; })).pt(0).z;
    assert(possible_bounds.size() > 0);

    vector<oriented_polygon> bounds;
    for (unsigned i = 0; i < possible_bounds.size(); i++) {
      oriented_polygon& bound = possible_bounds[i];
      bool contained = false;
      for (unsigned j = 0; j < possible_bounds.size(); j++) {
	if (i != j) {
	  oriented_polygon& other = possible_bounds[j];
	  if (contains(other, bound)) {
	    contained = true;
	    break;
	  }
	}
      }
      if (!contained) {
	bounds.push_back(bound);
      }
    }

    vector<oriented_polygon> holes;
    for (auto hole : possible_holes) {
      bool contained_by_bound = false;
      for (auto b : bounds) {
    	if (contains(b, hole)) {
    	  contained_by_bound = true;
    	  break;
    	}
      }
      if (contained_by_bound) {
    	holes.push_back(hole);
      }
    }
    
    assert(bottom < last_level);
    return pocket(bounds, holes, last_level, bottom);
  }

  // vector<pocket> make_pockets(vector<oriented_polygon> polygons) {
  //   vector<pocket> pockets;
  //   double start_depth = polygons.front().vertices.front().z;
  //   double workpiece_height = start_depth + 0.1;
  //   double last_level = start_depth;
  //   auto below_level = begin(polygons);
  //   while (below_level != end(polygons)) {
  //     auto pocket = level_pocket(begin(polygons),
  // 				 below_level,
  // 				 end(polygons),
  // 				 workpiece_height);
  //     pockets.push_back(pocket);
  //     below_level = find_if(below_level, end(polygons),
  // 			    [last_level](const oriented_polygon& p)
  // 			    { return !within_eps(p.vertices.front().z, last_level, 0.01); });
  //     if (below_level != end(polygons)) {
  // 	workpiece_height = last_level;
  // 	last_level = (*below_level).vertices.front().z;
  //     }
  //   }
  //   return pockets;
  // }

  bool adjacent(const triangle l, const triangle r) {
    for (auto l_edge : l.edges()) {
      for (auto r_edge : r.edges()) {
	if (same_line(l_edge, r_edge)) {
	  return true;
	}
      }
    }
    return false;
  }

  vector<triangle> collect_surface(vector<triangle>& triangles) {
    assert(triangles.size() > 0);
    vector<triangle> surface;
    surface.push_back(triangles.back());
    triangles.pop_back();
    int i = 0;
    while (triangles.size() > 0 && i < triangles.size()) {
      auto t = triangles[i];
      bool part_of_surface = false;
      for (auto surface_triangle : surface) {
	if (adjacent(surface_triangle, t)) {
	  part_of_surface = true;
	  break;
	}
      }
      if (part_of_surface) {
	surface.push_back(t);
	triangles.erase(triangles.begin() + i);
      } else {
	i++;
      }
    }
    return surface;
  }

  vector<vector<triangle>> merge_surfaces(vector<triangle>& triangles) {
    vector<vector<triangle>> surfaces;
    while (triangles.size() > 0) {
      surfaces.push_back(collect_surface(triangles));
    }
    return surfaces;
  }

  oriented_polygon extract_boundary(vector<oriented_polygon>& polygons) {
    assert(polygons.size() > 0);
    for (unsigned i = 0; i < polygons.size(); i++) {
      auto possible_bound = polygons[i];
      bool contains_all = true;
      for (unsigned j = 0; j < polygons.size(); j++) {
	if (i != j) {
	  auto possible_hole = polygons[j];
	  if (!contains(possible_bound, possible_hole)) {
	    contains_all = false;
	    break;
	  }
	}
      }
      if (contains_all) {
	polygons.erase(polygons.begin() + i);
	return possible_bound;
      }
    }
    assert(false);
  }

  pocket pocket_for_surface(const vector<triangle>& surface, double top_height) {
    auto bounds = mesh_bounds(surface);
    auto boundary = extract_boundary(bounds);
    vector<oriented_polygon> bound{boundary};
    vector<oriented_polygon> holes = bounds;
    return pocket(bound, holes, top_height, boundary.height());
  }
  
  vector<pocket> make_pockets(vector<triangle>& triangles) {
    vector<vector<triangle>> surfaces = merge_surfaces(triangles);
    // TODO: Add real calculation of highest triangle z
    double start_depth = 4.0;
    double workpiece_height = start_depth + 0.1;
    vector<pocket> pockets;
    for (auto surface : surfaces) {
      pockets.push_back(pocket_for_surface(surface, workpiece_height));
    }
    return pockets;
  }

  vector<polyline> mill_pockets(vector<pocket>& pockets,
				double tool_diameter,
				double cut_depth) {
    vector<polyline> lines;
    double tool_radius = tool_diameter / 2.0;
    for (auto pocket : pockets) {
      auto pocket_paths = pocket_2P5D_interior(pocket, tool_radius, cut_depth);
      lines.insert(end(lines), begin(pocket_paths), end(pocket_paths));
    }
    return lines;
  }

  vector<polyline> mill_surface_lines(vector<triangle>& triangles,
				      double tool_diameter,
				      double cut_depth) {
    select_visible_triangles(triangles);
    auto pockets = make_pockets(triangles);
    return mill_pockets(pockets, tool_diameter, cut_depth);
  }

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter,
			     double cut_depth) {
    auto pocket_lines = mill_surface_lines(triangles, tool_diameter, cut_depth);
    return emco_f1_code(pocket_lines);
  }

}
