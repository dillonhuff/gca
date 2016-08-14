#include "geometry/extrusion.h"
#include "geometry/polygon.h"
#include "geometry/vtk_debug.h"
#include "geometry/vtk_utils.h"
#include "synthesis/axis_3.h"
#include "synthesis/millability.h"
#include "synthesis/shapes_to_gcode.h"
#include "synthesis/tool.h"
#include "synthesis/toolpath_generation.h"
#include "utils/algorithm.h"

namespace gca {

  double max_z(const triangle t) {
    vector<double> zs = {t.v1.z, t.v2.z, t.v3.z};
    return max_e(zs);
  }

  pocket pocket_for_surface(const std::vector<index_t>& sfs,
			    double top_height,
			    const triangular_mesh& mesh) {
    auto regions = constant_orientation_subsurfaces(surface(&mesh, sfs));
    if (regions.size() == 1) {
      point norm = mesh.face_orientation(sfs.front());
      if (within_eps(angle_between(norm, point(0, 0, 1)), 0, 1.0)) {
	cout << "Found flat pocket" << endl;
	vtk_debug_highlight_inds(sfs, mesh);
	auto p = flat_pocket(top_height, sfs, &mesh);
	vtk_debug_polygon(p.get_boundary());
	return p;
      }
    }
    return pocket(freeform_pocket(top_height, sfs, &mesh));
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

  vector<toolpath> mill_pockets(vector<pocket>& pockets,
				const std::vector<tool>& tools,
				const material& stock_material) {
    DBG_ASSERT(pockets.size() > 0);
    double h = (*(max_element(begin(pockets), end(pockets),
			      [](const pocket& l, const pocket& r)
      { return l.get_start_depth() < r.get_start_depth(); }))).get_start_depth();

    double safe_z = h + 0.1;
    
    double cut_depth, speed, feed;
    if (stock_material == ACETAL) {
      cut_depth = 0.2;
      speed = 3000;
      feed = 8.0;
    } else if (stock_material == ALUMINUM) {
      cut_depth = 0.1;
      speed = 3000;
      feed = 5.0;
    } else {
      DBG_ASSERT(false);
    }
    vector<toolpath> lines;
    for (auto pocket : pockets) {
      tool t = pocket.select_tool(tools);
      auto pocket_paths = pocket.toolpath_lines(t, cut_depth);
      lines.push_back(toolpath(pocket.pocket_type(), safe_z, speed, feed, t, pocket_paths));
    }
    return lines;
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

  // TODO: Make this less hacky
  index_poly
  min_index_poly(const std::vector<point>& pts,
		 const std::vector<index_poly>& polys) {
    DBG_ASSERT(polys.size() > 0);
    return *(min_element(begin(polys), end(polys),
			 [pts](const index_poly& l, const index_poly& r)
			 { return pts[l.front()].z < pts[r.front()].z; }));
  }

  std::vector<gca::edge>
  index_poly_to_edges(const index_poly& p) {
    DBG_ASSERT(p.size() > 0);
    vector<gca::edge> edges;
    for (unsigned i = 0; i < p.size(); i++) {
      gca::edge e(p[i], p[(i + 1) % p.size()]);
      edges.push_back(e);
    }

    DBG_ASSERT(p.size() == edges.size());
    
    return edges;
  }

  bool all_concave(const triangular_mesh& m, const std::vector<gca::edge>& e) {
    for (auto ed : e) {
      cout << "Dihedral angle = " << dihedral_angle(ed, m) << endl;
    }
    return all_of(begin(e), end(e), [m](const edge ed)
		  { return dihedral_angle(ed, m) > 180; });
  }

  bool has_no_base(const std::vector<index_t>& surf,
		   const triangular_mesh& part,
		   const std::vector<index_t>& side_faces) {
    // TODO: Sort first? This is disgustingly inefficient
    if (intersection(side_faces, surf).size() == surf.size()) {
      return true;
    }
    return false;
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
    box b = mesh.bounding_box();
    double base_z = b.z_min;
    double top_z = b.z_max;

    auto side_faces = side_millable_faces(point(0, 0, -1),
    					  mesh.face_indexes(),
    					  mesh);

    for (auto surface : surfaces) {
      if (has_no_base(surface, mesh, side_faces)) {
	oriented_polygon outline = project(base_outline(surface, mesh), base_z);
	cout << "Found flat pocket" << endl;
	vtk_debug_highlight_inds(surface, mesh);
	flat_pocket p(top_z, base_z, outline);
	vtk_debug_polygon(p.get_boundary());
	
	pockets.push_back(p);
      }
    }

    return pockets;
  }

  std::vector<pocket>
  make_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
		       const triangular_mesh& mesh,
		       double workpiece_height) {
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

  std::vector<pocket> make_surface_pockets(const triangular_mesh& mesh,
					   std::vector<std::vector<index_t>>& surfaces) {
					   
    double h = max_in_dir(mesh, point(0, 0, 1));
    vector<pocket> pockets =
      make_surface_pockets(surfaces, mesh, h);
    return pockets;
  }

}
