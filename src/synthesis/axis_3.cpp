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

  // pocket pocket_for_surface(const std::vector<index_t>& sfs,
  // 			    double top_height,
  // 			    const triangular_mesh& mesh) {
  //   auto regions = constant_orientation_subsurfaces(surface(&mesh, sfs));
  //   if (regions.size() == 1) {
  //     point norm = mesh.face_orientation(sfs.front());
  //     if (within_eps(angle_between(norm, point(0, 0, 1)), 0, 1.0)) {
  // 	//vtk_debug_highlight_inds(sfs, mesh);
  // 	auto p = flat_pocket(top_height, sfs, &mesh);
  // 	//vtk_debug_polygon(p.get_boundary());
  // 	return p;
  //     }
  //   }
  //   return pocket(freeform_pocket(top_height, sfs, &mesh));
  // }
  
  // std::vector<pocket> pockets_for_surfaces(const std::vector<std::vector<index_t>>& surfaces,
  // 				   double workpiece_height,
  // 				   const triangular_mesh& mesh) {
  //   vector<pocket> pockets;
  //   for (auto surface : surfaces) {
  //     pockets.push_back(pocket_for_surface(surface, workpiece_height, mesh));
  //   }
  //   return pockets;
  // }

  // oriented_polygon base_outline(const std::vector<index_t>& sfs,
  // 				const triangular_mesh& part) {
  //   std::vector<gca::edge> edges =
  //     orthogonal_boundary_edges(surface(&part, sfs), point(0, 0, 1));

  //   vector<index_poly> ps =
  //     unordered_segments_to_index_polylines(edges);

  //   DBG_ASSERT(ps.size() == 2);

  //   vector<oriented_polygon> bounds;
  //   for (auto ip : ps) {
  //     bounds.push_back(oriented_polygon_for_index_polyline(part.vertex_list(), ip, point(0, 0, 1)));
  //   }

  //   auto min_poly = min_e(bounds, [](const oriented_polygon& l) { return min_z(l); });
  //   return min_poly;
  // }

  // std::vector<pocket>
  // closed_vertical_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
  // 				  const triangular_mesh& mesh,
  // 				  double workpiece_height) {
  //   if (sfs.size() == 0) { return {}; }

  //   auto surf_list = sfs;
  //   delete_if(surf_list,
  //   	      [&mesh](const vector<index_t>& surface)
  //   	      { return !all_orthogonal_to(surface, mesh, point(0, 0, 1), 5.0); });

  //   if (surf_list.size() == 0) { return {}; }

  //   std::vector<std::vector<index_t>> surfaces =
  //     merge_connected_surfaces(surf_list, mesh);
    
  //   vector<pocket> pockets;
  //   box b = mesh.bounding_box();
  //   double base_z = b.z_min;
  //   double top_z = b.z_max;

  //   auto side_faces = side_millable_faces(point(0, 0, -1),
  //   					  mesh.face_indexes(),
  //   					  mesh);

  //   for (auto surface : surfaces) {
  //     if (has_no_base(surface, mesh, side_faces)) {
  // 	oriented_polygon outline = project(base_outline(surface, mesh), base_z);

  // 	flat_pocket p(top_z, base_z, outline);
	
  // 	pockets.push_back(p);
  //     }
  //   }

  //   return pockets;
  // }

  // std::vector<pocket>
  // make_surface_pockets(const std::vector<std::vector<index_t>>& sfs,
  // 		       const triangular_mesh& mesh,
  // 		       double workpiece_height) {
  //   std::vector<std::vector<index_t>> surfaces = sfs;

  //   vector<pocket> pockets =
  //     closed_vertical_surface_pockets(sfs, mesh, workpiece_height);
  //   filter_vertical_surfaces(surfaces, mesh);

  //   if (surfaces.size() > 0) {
  //     surfaces = merge_connected_surfaces(surfaces, mesh);
  //     auto nv_pockets = pockets_for_surfaces(surfaces, workpiece_height, mesh);
  //     concat(pockets, nv_pockets);
  //   }

  //   sort(begin(pockets), end(pockets),
  // 	 [](const pocket& l, const pocket& r)
  // 	 { return l.get_end_depth() > r.get_end_depth(); });

  //   return pockets;
  // }

  // std::vector<pocket> make_surface_pockets(const triangular_mesh& mesh,
  // 					   std::vector<std::vector<index_t>>& surfaces) {
					   
  //   double h = max_in_dir(mesh, point(0, 0, 1));
  //   vector<pocket> pockets =
  //     make_surface_pockets(surfaces, mesh, h);
  //   return pockets;
  // }

  // std::vector<pocket>
  // make_surface_pockets(const triangular_mesh& part,
  // 		       const std::vector<surface>& surfaces) {
  //   std::vector<std::vector<index_t>> inds;
  //   for (auto s : surfaces) {
  //     inds.push_back(s.index_list());
  //   }
  //   auto mesh_cpy = new (allocate<triangular_mesh>()) triangular_mesh(part);
  //   return make_surface_pockets(*mesh_cpy, inds);
  // }

  
}
