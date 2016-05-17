#include "geometry/polygon.h"
#include "synthesis/axis_3.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/tool.h"
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

  std::vector<polyline> reflect_y(const std::vector<polyline>& pocket_lines) {
    vector<polyline> reflected;
    for (auto p : pocket_lines) {
      reflected.push_back(p.apply([](const point p)
				  { return point(p.x, -p.y, p.z); }));
    }
    return reflected;
  }

  std::vector<block> emco_f1_code(const std::vector<polyline>& pocket_lines,
				  const double safe_height) {
    assert(pocket_lines.size() > 0);
    for (auto pl : pocket_lines) {
      assert(pl.num_points() > 0);
    }
    auto reflected_lines = reflect_y(pocket_lines);
    cut_params params;
    params.target_machine = EMCO_F1;
    params.safe_height = safe_height;
    return polylines_cuts(reflected_lines, params);
  }

  bool adjacent(const triangle l, const triangle r) {
    for (auto l_edge : l.edges()) {
      for (auto r_edge : r.edges()) {
	if (same_line(l_edge, r_edge, 0.001)) {
	  return true;
	}
      }
    }
    return false;
  }

  std::vector<triangle> collect_surface(std::vector<triangle>& triangles) {
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

  std::vector<std::vector<triangle>>
  merge_surfaces(std::vector<triangle>& triangles) {
    vector<vector<triangle>> surfaces;
    while (triangles.size() > 0) {
      surfaces.push_back(collect_surface(triangles));
    }
    return surfaces;
  }

  std::vector<std::vector<triangle>>
  merge_surfaces(std::vector<index_t> face_inds,
		 const triangular_mesh& mesh) {
    vector<triangle> t;
    for (auto i : face_inds) {
      t.push_back(mesh.face_triangle(i));
    }
    return merge_surfaces(t);
  }

  pocket pocket_for_surface(std::vector<triangle>& surface,
			    double top_height) {
    auto bounds = mesh_bounds(surface);
    auto boundary = extract_boundary(bounds);
    vector<oriented_polygon> bound{boundary};
    vector<oriented_polygon> holes = bounds;
    return pocket(bound, holes, top_height, surface);
  }
  
  std::vector<pocket> make_pockets(std::vector<index_t>& face_inds,
				   const triangular_mesh& mesh,
				   double workpiece_height) {
    vector<vector<triangle>> surfaces = merge_surfaces(face_inds, mesh);
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

  std::vector<polyline> mill_surface_lines(const triangular_mesh& mesh,
					   const tool& t,
					   double cut_depth,
					   double workpiece_height) {
    cout << "START mill_surface_lines" << endl;
    std::vector<index_t> face_inds = select_visible_triangles(mesh);
    auto pockets = make_pockets(face_inds, mesh, workpiece_height);
    auto lines = mill_pockets(pockets, t.diameter(), cut_depth);
    cout << "END mill_surface_lines" << endl;
    return lines;
  }

  vector<block> mill_surface(const triangular_mesh& mesh,
			     const tool& t,
			     double cut_depth,
			     double workpiece_height) {
    auto pocket_lines = shift_lines(mill_surface_lines(mesh, t, cut_depth, workpiece_height), point(0, 0, t.length()));
    
    return emco_f1_code(pocket_lines, workpiece_height + 0.1);
  }

}
