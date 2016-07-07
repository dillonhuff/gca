#include "geometry/polygon.h"
#include "synthesis/axis_3.h"
#include "synthesis/millability.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "utils/algorithm.h"

namespace gca {

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
      assert(count > 0);
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

  pocket pocket_for_surface(const std::vector<index_t>& surface,
			    double top_height,
			    const triangular_mesh& mesh) {
    return pocket(top_height, surface, &mesh);
  }
  
  std::vector<pocket> make_pockets(const std::vector<std::vector<index_t>>& surfaces,
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
    return shift_lines(lines, point(0, 0, t.length()));
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

  void filter_vertical_surfaces(std::vector<std::vector<index_t>>& delta_regions,
				const triangular_mesh& mesh) {
    delete_if(delta_regions,
    	      [&mesh](const vector<index_t>& surface)
    	      { return all_orthogonal_to(surface, mesh, point(0, 0, 1), 5.0); });
    delete_if(delta_regions,
    	      [&mesh](const vector<index_t>& surface)
    	      { return all_normals_below(surface, mesh, -0.1); });
  }
  
  std::vector<std::vector<index_t>> make_surfaces(const triangular_mesh& mesh) {
    double normal_degrees_delta = 30.0;
    auto inds = millable_faces(point(0, 0, 1), mesh);
    vector<vector<index_t>> delta_regions =
      normal_delta_regions(inds, mesh, normal_degrees_delta);
    filter_vertical_surfaces(delta_regions, mesh);
    return delta_regions;
  }

  std::vector<pocket> make_pockets(const triangular_mesh& mesh,
  				   const double workpiece_height) {
    vector<vector<index_t>> surfaces = make_surfaces(mesh);
    auto merged_surfaces = merge_connected_surfaces(surfaces, mesh);
    auto pockets = make_pockets(merged_surfaces, workpiece_height, mesh);
    return pockets;
  }

  bool has_no_base(const std::vector<index_t>& surface,
		   const triangular_mesh& part) {
    auto side_faces = side_millable_faces(point(0, 0, -1),
					  part.face_indexes(),
					  part);
    // TODO: Sort first? This is disgustingly inefficient
    if (intersection(side_faces, surface).size() == surface.size()) {
      return true;
    }
    return false;
  }

  double min_z(const oriented_polygon& p) {
    auto min_z =
      max_element(begin(p.vertices()), end(p.vertices()),
		  [](const point l, const point r)
		  { return l.z < r.z; });
    return (*min_z).z;
  }

  oriented_polygon base_outline(const std::vector<index_t>& surface,
				const triangular_mesh& part) {
    auto bounds = mesh_bounds(surface, part);
    auto min_poly =
      min_element(begin(bounds), end(bounds),
		  [](const oriented_polygon& l, const oriented_polygon& r)
		  { return min_z(l) < min_z(r); });
    return *min_poly;
  }

  std::vector<pocket>
  closed_vertical_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
				  const triangular_mesh& mesh,
				  double workpiece_height) {
    std::vector<std::vector<index_t>> surfaces = sfs;
    delete_if(surfaces,
    	      [&mesh](const vector<index_t>& surface)
    	      { return !all_orthogonal_to(surface, mesh, point(0, 0, 1), 5.0); });
    vector<pocket> pockets;
    double base_z = mesh.bounding_box().z_min;
    for (auto surface : surfaces) {
      if (has_no_base(surface, mesh)) {
	oriented_polygon outline = project(base_outline(surface, mesh), base_z);
	triangular_mesh new_base = triangulate(outline);
	triangular_mesh* new_base_cpy = allocate<triangular_mesh>();
	triangular_mesh* base_mesh = new (new_base_cpy) triangular_mesh();
	assert(base_mesh == new_base_cpy);
	*base_mesh = new_base;
	vector<oriented_polygon> holes;
	pockets.push_back(pocket(workpiece_height, base_mesh->face_indexes(), new_base_cpy));
      }
    }
    cout << "# vertical pockets = " << pockets.size() << endl;
    return pockets;
  }

  std::vector<pocket>
  make_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
		       const triangular_mesh& mesh,
		       double workpiece_height) {
    // TODO: Optimize this away
    std::vector<std::vector<index_t>> surfaces = sfs;
    vector<pocket> pockets =
      closed_vertical_surface_pockets(sfs, mesh, workpiece_height);
    filter_vertical_surfaces(surfaces, mesh);
    if (surfaces.size() > 0) {
      surfaces = merge_connected_surfaces(surfaces, mesh);
      auto nv_pockets = make_pockets(surfaces, workpiece_height, mesh);
      concat(pockets, nv_pockets);
    }
    sort(begin(pockets), end(pockets),
	 [](const pocket& l, const pocket& r)
	 { return l.get_end_depth() > r.get_end_depth(); });
    return pockets;
  }

}
