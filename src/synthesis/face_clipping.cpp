#include "geometry/vtk_debug.h"
#include "geometry/mesh_operations.h"
#include "synthesis/face_clipping.h"

namespace gca {

  workpiece clipped_workpiece(const workpiece aligned_workpiece,
  			      const triangular_mesh& part_mesh) {
    point x_n = aligned_workpiece.sides[0].normalize();
    point y_n = aligned_workpiece.sides[1].normalize();
    point z_n = aligned_workpiece.sides[2].normalize();
    
    point x_d = diameter(aligned_workpiece.sides[0], part_mesh) * x_n;
    point y_d = diameter(aligned_workpiece.sides[1], part_mesh) * y_n;
    point z_d = diameter(aligned_workpiece.sides[2], part_mesh) * z_n;

    return workpiece(x_d, y_d, z_d, aligned_workpiece.stock_material);
  }

  // TODO: Remove stock param
  plane face_plane(const triangular_mesh& part,
		   const point n) {
    point p = max_point_in_dir(part, n);
    return plane(-1*n, p);
  }

  std::vector<fixture_setup>
  axis_clipping(const triangular_mesh& aligned_workpiece,
		const triangular_mesh& part_mesh,
		const fixtures& f) {
    vector<surface> surfs = outer_surfaces(aligned_workpiece);

    DBG_ASSERT(surfs.size() == 6);

    vector<surface> ax_surfaces =
      take_basis(surfs,
		 [](const surface& l, const surface& r)
		 { return within_eps(angle_between(normal(l), normal(r)), 90, 2.0); },
		 3);
    
    vector<point> norms;
    for (auto ax : ax_surfaces) {
      point n = ax.face_orientation(ax.front());
      norms.push_back(n);
      norms.push_back(-1*n);
    }

    vice v = f.get_vice();
    triangular_mesh current_stock = aligned_workpiece;
    vector<fixture_setup> cut_setups;    
    for (auto n : norms) {
      auto sfs = outer_surfaces(current_stock);

      DBG_ASSERT(sfs.size() == 6);

      auto orients = all_stable_orientations(sfs, v);
      clamp_orientation orient = find_orientation_by_normal(orients, n);

      auto t = mating_transform(current_stock, orient, v);
      cut_setups.push_back(clip_base(apply(t, current_stock), apply(t, part_mesh), fixture(orient, v)));
      
      plane clip_plane = face_plane(part_mesh, n);
      current_stock = clip_mesh(current_stock, clip_plane);
    }

    DBG_ASSERT(cut_setups.size() == 6);

    return cut_setups;
  }

  clipping_plan
  axis_by_axis_clipping(const std::vector<workpiece>& wps, 
			const triangular_mesh& part_mesh,
			const std::vector<tool>& tools,
			const fixtures& f) {
    DBG_ASSERT(wps.size() > 0);

    const auto& w = wps.front();

    vector<surface> surfs_to_cut = surfaces_to_cut(part_mesh);
    triangular_mesh wp_mesh = align_workpiece(surfs_to_cut, w);

    auto clip_setups = axis_clipping(wp_mesh, part_mesh, f);

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, wp_mesh);
    remove_contained_surfaces(clipped_surfs, surfs_to_cut);

    return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups, w);
  }

}
