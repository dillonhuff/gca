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

  // TODO: Really should not be oriented polygons
  vector<oriented_polygon> mesh_bounds(const vector<index_t>& faces,
				       const triangular_mesh& mesh) {
    vector<oriented_polygon> ps;
    if (faces.size() == 0) {
      return ps;
    }
    point normal = mesh.face_orientation(faces.front());
    typedef pair<index_t, index_t> iline;
    vector<iline> tri_lines;
    for (auto i : faces) {
      auto t = mesh.triangle_vertices(i);
      tri_lines.push_back(iline(t.v[0], t.v[1]));
      tri_lines.push_back(iline(t.v[1], t.v[2]));
      tri_lines.push_back(iline(t.v[2], t.v[0]));
    }

    // TODO: Change to sort and count, maybe add to system/algorithm?
    vector<iline> no_ds;
    for (auto l : tri_lines) {
      int count = 0;
      for (auto r : tri_lines) {
	if ((l.first == r.first && l.second == r.second) ||
	    (l.first == r.second && l.second == r.first)) {
	  count++;
	}
      }
      if (count == 1) {
	no_ds.push_back(l);
      }
    }
    vector<line> no_dups;
    for (auto l : no_ds) {
      no_dups.push_back(line(mesh.vertex(l.first), mesh.vertex(l.second)));
    }
    return unordered_segments_to_polygons(normal, no_dups);
  }
  
  std::vector<oriented_polygon> preprocess_triangles(const triangular_mesh& mesh) {
    auto face_inds = preprocess_faces(mesh);
    auto polygons = mesh_bounds(face_inds, mesh);
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

  pocket pocket_for_surface(std::vector<index_t>& surface,
			    double top_height,
			    const triangular_mesh& mesh) {
    auto bounds = mesh_bounds(surface, mesh);
    auto boundary = extract_boundary(bounds);
    vector<oriented_polygon> holes = bounds;
    return pocket(boundary, holes, top_height, surface, mesh);
  }
  
  std::vector<pocket> make_pockets(std::vector<std::vector<index_t>>& surfaces,
				   double workpiece_height,
				   const triangular_mesh& mesh) {
    vector<pocket> pockets;
    for (auto surface : surfaces) {
      pockets.push_back(pocket_for_surface(surface, workpiece_height, mesh));
    }
    return pockets;
  }

  vector<polyline> mill_pockets(vector<pocket>& pockets,
				const tool& t,
				double cut_depth) {
    vector<polyline> lines;
    for (auto pocket : pockets) {
      auto pocket_paths = pocket_2P5D_interior(pocket, t, cut_depth);
      lines.insert(end(lines), begin(pocket_paths), end(pocket_paths));
    }
    return lines;
  }

  bool all_orthogonal_to(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const point n,
			 const double tolerance) {
    for (auto i : triangles) {
      auto t = mesh.face_triangle(i);
      if (!within_eps(angle_between(t.normal, n), 90, tolerance)) {
	return false;
      }
    }
    return true;
  }

  bool all_normals_below(const vector<index_t>& triangles,
			 const triangular_mesh& mesh,
			 const double v) {
    for (auto i : triangles) {
      auto t = mesh.face_triangle(i);
      if (t.normal.normalize().z > v) {
	return false;
      }
    }
    return true;
  }
  
  std::vector<std::vector<index_t>> make_surfaces(const triangular_mesh& mesh) {
    double normal_degrees_delta = 30.0;
    auto inds = millable_faces(point(0, 0, 1), mesh);
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, mesh, normal_degrees_delta);
    delete_if(delta_regions,
    	      [&mesh](const vector<index_t>& surface)
    	      { return all_orthogonal_to(surface, mesh, point(0, 0, 1), 5.0); });
    delete_if(delta_regions,
    	      [&mesh](const vector<index_t>& surface)
    	      { return all_normals_below(surface, mesh, -0.1); });
    return delta_regions;
  }

  std::vector<pocket> make_pockets(const triangular_mesh& mesh,
				   const double workpiece_height) {
    vector<vector<index_t>> surfaces = make_surfaces(mesh);
    auto pockets = make_pockets(surfaces, workpiece_height, mesh);
    return pockets;
  }

  std::vector<polyline> mill_surface_lines(const triangular_mesh& mesh,
					   const tool& t,
					   double cut_depth,
					   double workpiece_height) {
    auto pockets = make_pockets(mesh, workpiece_height);    
    auto lines = mill_pockets(pockets, t, cut_depth);
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
