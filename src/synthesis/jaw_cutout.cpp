#include "geometry/vtk_debug.h"
#include "synthesis/workpiece_clipping.h"
#include "synthesis/jaw_cutout.h"

namespace gca {

  // TODO: Better names for left / right jaws
  struct custom_jaw_cutout {
    fixture base_fix;
    fixture clean_fix;
    fabrication_plan* left_jaw;
    fabrication_plan* right_jaw;
  };

  typedef point block_dims;

  struct jaw_dims {
    block_dims blk_dims;
    double h;
    double clip_x;
  };

  jaw_dims
  make_jaw_dims(const contour_surface_decomposition& surfs,
		const vice& v,
		const point axis,
		const point n) {
    block_dims blk(v.x_len(), v.y_len(), v.jaw_height());

    double part_height = diameter(n, surfs.top.get_parent_mesh());
    double h = (blk.z - part_height)*0.25;

    double part_width = diameter(axis, surfs.top.get_parent_mesh());
    double clip_x = part_width / 4.0;

    return jaw_dims{blk, h, clip_x};
  }

  
  point pick_jaw_cutout_axis(const surface& outline) {
    return outline.face_orientation(outline.front());
  }

  triangular_mesh
  block_mesh(const point center,
	     const point a1,
	     const point a2,
	     const point a3) {
    point p0 = center + 1*a1 + 1*a2 + 1*a3;
    point p1 = center + 1*a1 + 1*a2 + -1*a3;
    point p2 = center + 1*a1 + -1*a2 + 1*a3;
    point p3 = center + 1*a1 + -1*a2 + -1*a3;
    point p4 = center + -1*a1 + 1*a2 + 1*a3;
    point p5 = center + -1*a1 + 1*a2 + -1*a3;
    point p6 = center + -1*a1 + -1*a2 + 1*a3;
    point p7 = center + -1*a1 + -1*a2 + -1*a3;
    std::vector<point> pts{p0, p1, p2, p3, p4, p5, p6, p7};
    return make_mesh(triangulate_box_pts(pts), 0.001);
  }

  triangular_mesh
  cutout_mesh(const contour_surface_decomposition& surfs,
	      const vice& v,
	      const point axis,
	      const point n) {
    auto part_mesh = surfs.top.get_parent_mesh();
    point center = max_point_in_dir(part_mesh, axis);
    point left = cross(axis, n);
    // TODO: Insert actual dimesions
    point x_h = (v.x_len() / 2.0)*left.normalize();
    point y_h = (v.y_len() / 2.0)*n.normalize();
    point z_h = (v.jaw_height() / 2.0)*axis.normalize();
    return block_mesh(center, x_h, y_h, z_h);
  }

  // TODO: Produce longer clamps
  boost::optional<custom_jaw_cutout>
  custom_jaw_cutout_fixture(const contour_surface_decomposition& surfs,
			    const vice& v,
			    const point n) {
    //vtk_debug_highlight_inds(outline_of_contour);
    auto outline_of_contour = surfs.outline;
    auto top_of_contour = surfs.top;
    point axis = pick_jaw_cutout_axis(outline_of_contour);
    cout << "axis = " << axis << endl;
    cout << "n = " << n << endl;
    cout << "axis.dot(n) = " << axis.dot(n) << endl;
    assert(within_eps(axis.dot(n), 0, 0.01));

    triangular_mesh a_cutout = cutout_mesh(surfs, v, axis, n);
    vtk_debug_mesh(a_cutout);
    assert(false);

    // pair<triangular_mesh, triangular_mesh> jaws =
    //   custom_jaw_meshes(, axis, v, n);

    point neg_axis = -1*axis;
    double part_diam = diameter(axis, outline_of_contour.get_parent_mesh());

    cout << "neg axis = " << neg_axis << endl;
    cout << "part diameter along axis = " << part_diam << endl;

    // TODO: Sub select max point from only the given surface indexes
    point base_normal = top_of_contour.face_orientation(top_of_contour.front());
    point base_pt = max_point_in_dir(top_of_contour.get_parent_mesh(), base_normal);
    plane base_plane(base_normal, base_pt);

    point left_pt = max_point_in_dir(outline_of_contour.get_parent_mesh(), axis);
    plane left_plane(axis, left_pt);

    point right_pt = max_point_in_dir(outline_of_contour.get_parent_mesh(), neg_axis);
    plane right_plane(neg_axis, right_pt);
    
    clamp_orientation cutout_orient(left_plane, right_plane, base_plane);

    // TODO: Actually construct vice with more sane dimensions
    vice custom_jaw_vice = v;

    // TODO: Actually produce the other fixture
    fixture f(cutout_orient, custom_jaw_vice);
    return custom_jaw_cutout{f, f, nullptr, nullptr}; //make_pair(f, f);
  }

  clipping_plan
  custom_jaw_clip_plan(const triangular_mesh& aligned,
		       const triangular_mesh& part_mesh,
		       const contour_surface_decomposition& surfs,
		       const fixture& top_fix,
		       const fixture& base_fix,
		       const fixture& clean_fix) {
    // TODO: Figure out what function the second operation ought to be
    std::vector<fixture_setup> clip_setups;
    clip_setups.push_back(clip_top_and_sides_transform(aligned, part_mesh, surfs.visible_from_n, top_fix));
    clip_setups.push_back(clip_base_transform(aligned, part_mesh, surfs.visible_from_minus_n, base_fix));
    clip_setups.push_back(clip_base_transform(aligned, part_mesh, {}, clean_fix));

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, aligned);
    auto surfs_to_cut = surfs.rest;

    return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups, {nullptr, nullptr});
  }

  boost::optional<clipping_plan>
  custom_jaw_plan(const triangular_mesh& aligned,
		  const triangular_mesh& part_mesh,
		  const contour_surface_decomposition& surfs,
		  const fixture& top_fix,
		  const point n) {
    boost::optional<custom_jaw_cutout> custom =
      custom_jaw_cutout_fixture(surfs, top_fix.v, -1*n);
    if (custom) {
      std::vector<fixture_setup> clip_setups;
      clip_setups.push_back(clip_top_and_sides_transform(aligned, part_mesh, surfs.visible_from_n, top_fix));
      clip_setups.push_back(clip_base_transform(aligned, part_mesh, surfs.visible_from_minus_n, custom->base_fix));
      clip_setups.push_back(clip_base_transform(aligned, part_mesh, {}, custom->clean_fix));

      auto clipped_surfs =
	stable_surfaces_after_clipping(part_mesh, aligned);
      auto surfs_to_cut = surfs.rest;

      return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups, {nullptr, nullptr});
      // return custom_jaw_clip_plan(aligned,
      // 				  part_mesh,
      // 				  surfs,
      // 				  top_fix,
      // 				  (*custom).first,
      // 				  (*custom).second);
    }
    return boost::none;
  }
  
  
}