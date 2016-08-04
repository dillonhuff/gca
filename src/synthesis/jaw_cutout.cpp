#include "geometry/extrusion.h"
#include "geometry/vtk_debug.h"
#include "geometry/mesh_operations.h"
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

  point pick_jaw_cutout_axis(const contour_surface_decomposition& surfs) {
    vector<surface> viable_regions =
      regions_connected_to_both(surfs.outline, surfs.top, surfs.bottom);

    // TODO: Later sort multiple regions
    assert(viable_regions.size() == 1);

    surface r = viable_regions.front();

    std::vector<gca::edge> edges = shared_edges(r, surfs.bottom);

    assert(edges.size() > 0);

    // TODO: Check edge lengths
    auto middle_ind = static_cast<unsigned>(ceil((edges.size() + 1) / 2.0) - 1);

    assert(0 <= middle_ind && middle_ind < edges.size());

    gca::edge e = edges[middle_ind];
    auto tris = r.edge_face_neighbors(e);
    if (!(tris.size() == 1)) {
      cout << "# of tris = " << tris.size() << endl;
      assert(false);
    }
    return r.face_orientation(tris[0]);
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

  // TODO: Use unordered_segments_to_index_polygons instead
  // std::vector<line>
  // to_lines(const std::vector<gca::edge>& edges,
  // 	   const triangular_mesh& t) {
  //   vector<line> lines;
  //   for (auto e : edges) {
  //     lines.push_back();
  //   }
  //   return lines;
  // }

  std::vector<std::vector<line>>
  clip_with_halfspace(const std::vector<line>& outline,
		      const plane clip_plane) {
    vector<line> ls = outline;
    delete_if(ls, [clip_plane](const line l)
	      { return signed_distance(clip_plane, l.start) < 0 && signed_distance(clip_plane, l.end); });
    
    return {};
  }

  // TODO: Actually clip instead of just filtering
  index_poly
  clip_with_halfspace(const triangular_mesh& part_mesh,
		      const index_poly& p,
		      const plane clip_plane) {
    index_poly q = p;
    delete_if(q, [part_mesh, clip_plane](const index_t ind)
	      { return signed_distance(clip_plane, part_mesh.vertex(ind)) < 0; });
    return q;
  }

  triangular_mesh
  cutout_mesh(const contour_surface_decomposition& surfs,
	      const vice& v,
	      const point axis,
	      const point n) {
    vector<gca::edge> base_edges = shared_edges(surfs.outline, surfs.bottom);
    vector<index_poly> base_polys =
      unordered_segments_to_index_polygons(base_edges);

    assert(base_polys.size() == 1);

    index_poly p = base_polys.front();
    const triangular_mesh& part_mesh = surfs.outline.get_parent_mesh();
    //auto outline = to_lines(base_edges, part_mesh);

    double part_diameter = diameter(axis, part_mesh);
    point axis_pt = max_point_in_dir(part_mesh, axis);
    double d = part_diameter / 5.0;
    point axis_dir = axis.normalize();
    point clip_pt = axis_pt - d*axis_dir;
    plane clip_plane(axis_dir, clip_pt);
    index_poly jaw_outline = clip_with_halfspace(part_mesh, p, clip_plane);

    //double z_h = (v.jaw_height() / 2.0);
    
    triangular_mesh m =
      extrude_layers(part_mesh.vertex_list(),
		     {jaw_outline},
		     {20},
		     n);
    vtk_debug_mesh(m);

    return m;
    //assert(jaw_outline.size() == 1);
    //    assert(false);
    // point left = cross(axis, n);
    // TODO: Insert actual dimesions
    // point x_h = (v.x_len() / 2.0)*left.normalize();
    // point z_h = (v.jaw_height() / 2.0)*n.normalize();
    // triangular_mesh m = block_mesh(center, x_h, y_h, z_h);
    // return m;
    //return boolean_difference(m, part_mesh);
  }

  // TODO: Produce longer clamps
  boost::optional<custom_jaw_cutout>
  custom_jaw_cutout_fixture(const contour_surface_decomposition& surfs,
			    const vice& v,
			    const point n) {
    //vtk_debug_highlight_inds(outline_of_contour);
    auto outline_of_contour = surfs.outline;
    auto top_of_contour = surfs.top;
    point axis = pick_jaw_cutout_axis(surfs);
    cout << "axis = " << axis << endl;
    cout << "n = " << n << endl;
    cout << "axis.dot(n) = " << axis.dot(n) << endl;
    assert(within_eps(axis.dot(n), 0, 0.01));

    //    const triangular_mesh& part_mesh = surfs.top.get_parent_mesh();
    triangular_mesh* a_cutout =
      new (allocate<triangular_mesh>()) triangular_mesh(cutout_mesh(surfs, v, axis, n));
    triangular_mesh* an_cutout =
      new (allocate<triangular_mesh>()) triangular_mesh(cutout_mesh(surfs, v, -1*axis, n));
    vtk_debug_meshes({a_cutout, an_cutout}); //, &part_mesh});

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
      custom_jaw_cutout_fixture(surfs, top_fix.v.without_extras(), -1*n);
    if (custom) {
      std::vector<fixture_setup> clip_setups;
      clip_setups.push_back(clip_top_and_sides_transform(aligned, part_mesh, surfs.visible_from_n, top_fix));
      clip_setups.push_back(clip_base_transform(aligned, part_mesh, surfs.visible_from_minus_n, custom->base_fix));
      clip_setups.push_back(clip_base_transform(aligned, part_mesh, {}, custom->clean_fix));

      auto clipped_surfs =
	stable_surfaces_after_clipping(part_mesh, aligned);
      auto surfs_to_cut = surfs.rest;

      return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups, {nullptr, nullptr});
    }
    return boost::none;
  }
  
  
}
