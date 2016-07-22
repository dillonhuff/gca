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

  std::pair<fixture_setup, fixture_setup>
  axis_clip(const clamp_orientation& orient,
	    const std::vector<clamp_orientation>& clamp_orients,
	    const triangular_mesh& aligned_workpiece,
	    const triangular_mesh& part_mesh,
	    const vice& v) {
    auto pos_t = mating_transform(aligned_workpiece, orient, v);
    auto f1 = clip_base(apply(pos_t, aligned_workpiece), apply(pos_t, part_mesh), v);
    
    point neg_orient_dir = -1*orient.top_normal();
    clamp_orientation neg_orient =
      find_orientation_by_normal(clamp_orients, neg_orient_dir);
    auto neg_t = mating_transform(aligned_workpiece, neg_orient, v);
    auto f2 = clip_base(apply(neg_t, aligned_workpiece), apply(neg_t, part_mesh), v);

    return make_pair(f1, f2);
  }

  // TODO: Remove stock param
  plane face_plane(const triangular_mesh& stock,
		   const triangular_mesh& part,
		   const point n) {
    point p = max_point_in_dir(part, n);
    return plane(-1*n, p);
  }

  std::vector<fixture_setup>
  axis_clipping(const triangular_mesh& aligned_workpiece,
		const triangular_mesh& part_mesh,
		const fixtures& f) {
    vector<surface> surfs = outer_surfaces(aligned_workpiece);

    assert(surfs.size() == 6);

    vector<surface> ax_surfaces =
      take_basis(surfs,
		 [](const surface& l, const surface& r)
		 { return within_eps(l.face_orientation(l.front()).dot(r.face_orientation(r.front())), 0, 0.001); },
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
      cout << "--- Normal = " << n << endl;
      cout << "# of surfaces = " << sfs.size() << endl;

      assert(sfs.size() == 6);

      auto orients = all_stable_orientations(sfs, v);
      clamp_orientation orient = find_orientation_by_normal(orients, n);

      auto t = mating_transform(current_stock, orient, v);
      cut_setups.push_back(clip_base(apply(t, current_stock), apply(t, part_mesh), v));
      
      plane clip_plane = face_plane(current_stock, part_mesh, n);
      current_stock = clip_mesh(current_stock, clip_plane);
    }

    assert(cut_setups.size() == 6);

    return cut_setups;
    // vector<clamp_orientation> clamp_orients =
    //   all_stable_orientations(surfs, f.get_vice());

    // assert(clamp_orients.size() >= 6);

    // vector<clamp_orientation> ax_orients =
    //   three_orthogonal_orients(clamp_orients);


    // for (auto orient : axis_orients) {
    //   auto setup_pair = axis_clip(orient, clamp_orients, aligned_workpiece, part_mesh, f.get_vice());
    //   cut_setups.push_back(setup_pair.first);
    //   cut_setups.push_back(setup_pair.second);
    // }
    // return cut_setups;
  }

  clipping_plan
  axis_by_axis_clipping(const workpiece w, 
			const triangular_mesh& part_mesh,
			std::vector<surface>& surfaces_to_cut,
			const std::vector<tool>& tools,
			const fixtures& f) {
    triangular_mesh wp_mesh = align_workpiece(surfaces_to_cut, w);

    auto clip_setups = axis_clipping(wp_mesh, part_mesh, f);

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, wp_mesh);
    remove_contained_surfaces(clipped_surfs, surfaces_to_cut);

    return clipping_plan(clipped_surfs, clip_setups);
  }

}
