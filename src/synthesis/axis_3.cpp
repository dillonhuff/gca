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

  vector<block> emco_f1_code(const vector<polyline>& pocket_lines) {
    assert(pocket_lines.size() > 0);
    for (auto pl : pocket_lines) {
      assert(pl.num_points() > 0);
    }
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

  bool adjacent(const triangle l, const triangle r) {
    for (auto l_edge : l.edges()) {
      for (auto r_edge : r.edges()) {
	// NOTE: Used to be 0.001
	if (same_line(l_edge, r_edge, 0.001)) {
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
	i = 0;
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
    return pocket(bound, holes, top_height, surface);
  }
  
  vector<pocket> make_pockets(vector<triangle>& triangles, double workpiece_height) {
    vector<vector<triangle>> surfaces = merge_surfaces(triangles);
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
				      double cut_depth,
				      double workpiece_height) {
    select_visible_triangles(triangles);
    auto pockets = make_pockets(triangles, workpiece_height);
    return mill_pockets(pockets, tool_diameter, cut_depth);
  }

  vector<block> mill_surface(vector<triangle>& triangles,
			     double tool_diameter,
			     double cut_depth,
			     double workpiece_height) {
    auto pocket_lines = mill_surface_lines(triangles, tool_diameter, cut_depth, workpiece_height);
    return emco_f1_code(pocket_lines);
  }

}
