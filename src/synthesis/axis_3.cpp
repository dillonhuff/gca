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

  pocket pocket_for_surface(const std::vector<index_t>& surface,
			    double top_height,
			    const triangular_mesh& mesh) {
    return pocket(freeform_pocket(top_height, surface, &mesh));
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
    assert(pockets.size() > 0);
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
      assert(false);
    }
    vector<toolpath> lines;
    for (auto pocket : pockets) {
      tool t = pocket.select_tool(tools);
      auto pocket_paths = pocket.toolpath_lines(t, cut_depth);
      lines.push_back(toolpath(safe_z, speed, feed, t, pocket_paths));
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

  bool has_no_base(const std::vector<index_t>& surface,
		   const triangular_mesh& part) {
    auto side_faces = side_millable_faces(point(0, 0, -1),
					  part.face_indexes(),
					  part);
    // TODO: Sort first? This is disgustingly inefficient
    if (intersection(side_faces, surface).size() == surface.size()) {
      vtk_debug_highlight_inds(surface, part);
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
    double base_z = mesh.bounding_box().z_min;
    for (auto surface : surfaces) {
      if (has_no_base(surface, mesh)) {
	oriented_polygon outline = project(base_outline(surface, mesh), base_z);
	triangular_mesh new_base =
	  make_mesh(vtk_triangulate_poly(outline), 0.01); //triangulate(outline);
	triangular_mesh* new_base_cpy = allocate<triangular_mesh>();
	triangular_mesh* base_mesh = new (new_base_cpy) triangular_mesh();
	assert(base_mesh == new_base_cpy);
	*base_mesh = new_base;
	vector<oriented_polygon> holes;
	pockets.push_back(freeform_pocket(workpiece_height, base_mesh->face_indexes(), new_base_cpy));
      }
    }
    return pockets;
  }

  std::vector<pocket>
  make_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
		       const triangular_mesh& mesh,
		       double workpiece_height) {
    // TODO: Optimize this away
    std::vector<std::vector<index_t>> surfaces = sfs;
    // TODO: Reintroduce vertical surface pocketing
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
