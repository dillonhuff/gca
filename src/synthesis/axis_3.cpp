#include "geometry/polygon.h"
#include "synthesis/axis_3.h"
#include "synthesis/millability.h"
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

  std::vector<index_t> preprocess_faces(const triangular_mesh& mesh) {
    std::vector<index_t> face_inds = millable_faces(point(0, 0, 1), mesh);
    // TODO: Find a way to remove this ad-hoc tolerance
    delete_if(face_inds,
	      [&mesh](const index_t i)
	      { return !is_upward_facing(mesh.face_triangle(i), 0.8); });
    return face_inds;
  }

  std::vector<oriented_polygon> preprocess_triangles(const triangular_mesh& mesh) {
    auto face_inds = preprocess_faces(mesh);
    vector<triangle> triangles;
    for (auto i : face_inds) {
      triangles.push_back(mesh.face_triangle(i));
    }
    auto polygons = mesh_bounds(triangles);
    return polygons;
  }

  std::vector<std::vector<triangle>>
  merge_surfaces(std::vector<index_t>& face_inds,
		 const triangular_mesh& mesh) {
    vector<vector<index_t>> surfaces = connect_regions(face_inds, mesh);
    vector<vector<triangle>> tris;
    for (auto s : surfaces) {
      vector<triangle> ts;
      for (auto i : s) {
    	ts.push_back(mesh.face_triangle(i));
      }
      tris.push_back(ts);
    }
    return tris;
  }

  pocket pocket_for_surface(std::vector<triangle>& surface,
			    double top_height) {
    auto bounds = mesh_bounds(surface);
    auto boundary = extract_boundary(bounds);
    vector<oriented_polygon> bound{boundary};
    vector<oriented_polygon> holes = bounds;
    return pocket(bound, holes, top_height, surface);
  }
  
  std::vector<pocket> make_pockets(std::vector<std::vector<triangle>>& surfaces,
				   double workpiece_height) {
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

  std::vector<std::vector<triangle>> make_surfaces(const triangular_mesh& mesh) {
    std::vector<index_t> face_inds = preprocess_faces(mesh);
    vector<vector<triangle>> surfaces = merge_surfaces(face_inds, mesh);
    return surfaces;
  }

  std::vector<pocket> make_pockets(const triangular_mesh& mesh,
				   const double workpiece_height) {
    vector<vector<triangle>> surfaces = make_surfaces(mesh);
    auto pockets = make_pockets(surfaces, workpiece_height);
    return pockets;
  }

  std::vector<polyline> mill_surface_lines(const triangular_mesh& mesh,
					   const tool& t,
					   double cut_depth,
					   double workpiece_height) {
    auto pockets = make_pockets(mesh, workpiece_height);    
    auto lines = mill_pockets(pockets, t.diameter(), cut_depth);
    return shift_lines(lines, point(0, 0, t.length()));
  }

  vector<block> mill_surface(const triangular_mesh& mesh,
			     const tool& t,
			     double cut_depth,
			     double workpiece_height) {
    auto pocket_lines = mill_surface_lines(mesh, t, cut_depth, workpiece_height);
    return emco_f1_code(pocket_lines, workpiece_height + 0.1);
  }

}
