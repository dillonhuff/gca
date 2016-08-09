#include "geometry/extrusion.h"
#include "geometry/vtk_debug.h"
#include "geometry/mesh_operations.h"
#include "geometry/rigid_arrangement.h"
#include "synthesis/mesh_to_gcode.h"
#include "synthesis/workpiece_clipping.h"
#include "synthesis/jaw_cutout.h"

namespace gca {

  // TODO: Better names for left / right jaws
  struct custom_jaw_cutout {
    fixture base_fix;
    fixture clean_fix;
    triangular_mesh* notch;
    fabrication_plan* left_jaw;
    fabrication_plan* right_jaw;
  };

  std::vector<index_poly>
  clip_with_halfspace(const triangular_mesh& part_mesh,
		      const std::vector<gca::edge>& p,
		      const plane clip_plane) {
    vector<gca::edge> q = p;
    delete_if(q, [part_mesh, clip_plane](const edge e)
	      { return signed_distance(clip_plane, part_mesh.vertex(e.l)) < 0 || signed_distance(clip_plane, part_mesh.vertex(e.r)) < 0; });
    return unordered_segments_to_index_polylines(q);
  }
  
  // TODO: Move to utils/algorithm
  template<typename T, typename F>
  index_t
  find_index(const std::vector<T>& elems, F f) {
    for (index_t i = 0; i < elems.size(); i++) {
      auto& e = elems[i];
      if (f(e)) {
	return i;
      }
    }
    DBG_ASSERT(false);
  }

  std::pair<extrusion, extrusion>
  complete_jaw_outline(const index_poly& pts,
		       const contour_surface_decomposition& surfs,
		       const vice& v,
		       const point axis) {
    point n = surfs.n;
    point a = axis.normalize();
    point prof = cross(n, axis).normalize();
    double x_inc = v.x_len() / 10.0;
    double z_h = v.jaw_height() / 3.0;
    polyline p = to_polyline(pts, surfs.top.get_parent_mesh().vertex_list());
    vector<point> curve_pts(begin(p), end(p));
    vector<point> endpts{curve_pts.front(), curve_pts.back()};

    double y_inc = diameter(a, curve_pts)*2.0;

    point cutout_max = max_along(endpts, prof);
    index_t max_ind = find_index(curve_pts, [cutout_max](const point x)
				 { return within_eps(x, cutout_max); });

    assert(max_ind == curve_pts.size() - 1 ||
	   max_ind == 0);
    
    point cutout_min = min_along(endpts, prof);

    double notch_x_inc = (cutout_max - cutout_min).len() / 3.0;
    double notch_y_inc = diameter(a, curve_pts) / 2.0;

    // Rectangle outline points
    point r1 = cutout_max + x_inc*prof;
    point r2 = cutout_max + x_inc*prof + y_inc*a;
    point r3 = cutout_min - x_inc*prof + y_inc*a;
    point r4 = cutout_min - x_inc*prof;
    vector<point> rect_pts{r1, r2, r3, r4};
    if (max_ind == 0) {
      reverse(begin(curve_pts), end(curve_pts));
    }
    concat(curve_pts, rect_pts);

    // Sanity checks to ensure reordering worked
    index_t min_ind = find_index(curve_pts, [cutout_min](const point x)
    				 { return within_eps(x, cutout_min); });
    assert(min_ind == 0);

    max_ind = find_index(curve_pts, [cutout_max](const point x)
				 { return within_eps(x, cutout_max); });

    assert(max_ind == curve_pts.size() - 5);

    assert(!within_eps(cutout_max, cutout_min, 0.01));
    // Done with checks

    index_poly ip;
    for (index_t i = 0; i < curve_pts.size(); i++) {
      ip.push_back(i);
    }

    index_t rect_start = curve_pts.size() - 4;
    index_t ri0 = rect_start + 0;
    index_t ri1 = rect_start + 1;
    index_t ri2 = rect_start + 2;
    index_t ri3 = rect_start + 3;

    point n1 = cutout_min + notch_x_inc*prof;
    point n2 = cutout_min + notch_x_inc*prof + notch_y_inc*a;
    point n3 = cutout_max - notch_x_inc*prof + notch_y_inc*a;
    point n4 = cutout_max - notch_x_inc*prof;

    concat(curve_pts, {n1, n2, n3, n4});

    index_t notch_start = curve_pts.size() - 4;
    index_t ni0 = notch_start + 0;
    index_t ni1 = notch_start + 1;
    index_t ni2 = notch_start + 2;
    index_t ni3 = notch_start + 3;

    index_poly notch{ni0, ni1, ni2, ni3};
    index_poly notch_negative{min_ind, ni0, ni1, ni2, ni3, max_ind, ri0, ri1, ri2, ri3};
    reverse(begin(notch_negative), end(notch_negative));
    reverse(begin(ip), end(ip));
    extrusion jaw{curve_pts, {ip, notch_negative}, {z_h, (3.0/2.0)*z_h}, n};

    auto shifted_curve_pts = shift( ((z_h)*(-1))*n , curve_pts );
    extrusion notch_e{shifted_curve_pts, {notch}, {z_h}, -1*n};

    return std::make_pair(jaw, notch_e);
  }

  std::pair<triangular_mesh, triangular_mesh>
  cutout_mesh(const contour_surface_decomposition& surfs,
	      const vice& v,
	      const point axis) {
	      //	      const point n) {
    vector<gca::edge> base_edges = shared_edges(surfs.outline, surfs.bottom);

    const triangular_mesh& part_mesh = surfs.outline.get_parent_mesh();

    double part_diameter = diameter(axis, part_mesh);
    point axis_pt = max_point_in_dir(part_mesh, axis);
    double d = part_diameter / 5.0;
    point axis_dir = axis.normalize();
    point clip_pt = axis_pt - d*axis_dir;
    plane clip_plane(axis_dir, clip_pt);
    vector<index_poly> jaw_outline =
      clip_with_halfspace(part_mesh, base_edges, clip_plane);

    assert(jaw_outline.size() == 1);

    pair<extrusion, extrusion> custom_jaw =
      complete_jaw_outline(jaw_outline.front(), surfs, v, axis);

    cout << "About to extrude m " << endl;
    triangular_mesh m = extrude(custom_jaw.first);
    //    auto pd = polydata_for_trimesh(m);
    //debug_print_edge_summary(pd);
    //vtk_debug_mesh_boundary_edges(m);
    assert(m.is_connected());
    cout << "Extruded m " << endl;
    triangular_mesh notch = extrude(custom_jaw.second);
    assert(notch.is_connected());
    cout << "Extruded notch" << endl;

    return std::make_pair(m, notch);
  }

  soft_jaws make_soft_jaws(const contour_surface_decomposition& surfs,
			   const vice& v) {
    auto outline_of_contour = surfs.outline;
    auto top_of_contour = surfs.top;
    point axis = pick_jaw_cutout_axis(surfs);
    cout << "axis = " << axis << endl;
    cout << "n = " << surfs.n << endl;
    cout << "axis.dot(n) = " << axis.dot(surfs.n) << endl;
    DBG_ASSERT(within_eps(axis.dot(surfs.n), 0, 0.01));

    const triangular_mesh& part_mesh = surfs.top.get_parent_mesh();
    auto a_meshes = cutout_mesh(surfs, v, axis);
    auto an_meshes = cutout_mesh(surfs, v, -1*axis);

    rigid_arrangement notch_top;
    notch_top.insert("notch", a_meshes.second);
    notch_top.insert("part", part_mesh);
    notch_top.insert("a_jaw", a_meshes.first);
    notch_top.insert("an_jaw", an_meshes.first);

    //debug_arrangement(notch_top);

    triangular_mesh* a_cutout =
      new (allocate<triangular_mesh>()) triangular_mesh(a_meshes.first);
    triangular_mesh* notch =
      new (allocate<triangular_mesh>()) triangular_mesh(a_meshes.second);
    triangular_mesh* an_cutout =
      new (allocate<triangular_mesh>()) triangular_mesh(an_meshes.first);

    //vtk_debug_meshes({&part_mesh, notch});//{a_cutout, an_cutout}); //, &part_mesh});

    return soft_jaws{axis, notch, a_cutout, an_cutout};
  }

  // TODO: Produce longer clamps
  boost::optional<custom_jaw_cutout>
  custom_jaw_cutout_fixture(const contour_surface_decomposition& surfs,
			    const vice& v,
			    //			    const point n,
			    const fabrication_inputs& fab_inputs) {
    soft_jaws jaw_plan =
      make_soft_jaws(surfs, v); //, n);

    surface outline_of_contour = surfs.outline;
    surface top_of_contour = surfs.top;
    surface bottom_of_contour = surfs.bottom;
    point axis = jaw_plan.axis;
    point neg_axis = -1*axis;
    double part_diam = diameter(axis, outline_of_contour.get_parent_mesh());

    cout << "neg axis = " << neg_axis << endl;
    cout << "part diameter along axis = " << part_diam << endl;

    fabrication_plan ap = make_fabrication_plan(*(jaw_plan.a_jaw), fab_inputs);
    fabrication_plan anp = make_fabrication_plan(*(jaw_plan.an_jaw), fab_inputs);
    fabrication_plan* a_plan =
      new (allocate<fabrication_plan>()) fabrication_plan(ap);
    fabrication_plan* an_plan =
      new (allocate<fabrication_plan>()) fabrication_plan(anp);

    // TODO: Sub select max point from only the given surface indexes
    point base_normal = top_of_contour.face_orientation(top_of_contour.front());
    point base_pt = max_point_in_dir(top_of_contour.get_parent_mesh(), base_normal);
    plane base_plane(base_normal, base_pt);

    point left_pt = max_point_in_dir(*(jaw_plan.a_jaw), axis);
    plane left_plane(axis, left_pt);

    point right_pt = max_point_in_dir(*(jaw_plan.an_jaw), neg_axis);
    plane right_plane(neg_axis, right_pt);
    
    clamp_orientation cutout_orient(left_plane, right_plane, base_plane);

    // TODO: Actually produce the other fixture
    fixture f_clean(cutout_orient, v);

    point notch_base_normal =
      bottom_of_contour.face_orientation(top_of_contour.front());
    point notch_base_pt =
      max_point_in_dir(*(jaw_plan.notch), notch_base_normal);
    plane notch_plane(notch_base_normal, notch_base_pt);
    
    clamp_orientation clean_orient(left_plane, right_plane, notch_plane);
    fixture f_base(clean_orient, v);

    return custom_jaw_cutout{f_base, f_clean, jaw_plan.notch, a_plan, an_plan};
  }

  std::vector<fixture_setup>
  soft_jaw_clip_setups(const triangular_mesh& aligned,
		       const triangular_mesh& part_mesh,
		       const contour_surface_decomposition& surfs,
		       const fixture& top_fix,
		       const custom_jaw_cutout& custom) {
    const triangular_mesh& notch = *(custom.notch);
    const triangular_mesh& a_jaw = *(custom.left_jaw->final_part_mesh());
    const triangular_mesh& an_jaw = *(custom.right_jaw->final_part_mesh());

    const vector<surface>& top_surfs = surfs.visible_from_n;
    const vector<surface>& base_surfs = surfs.visible_from_minus_n;

    homogeneous_transform nt = mating_transform(aligned, top_fix.orient, top_fix.v);

    rigid_arrangement notch_top;
    notch_top.insert("notch", nt, notch);
    notch_top.insert("part", nt, part_mesh);
    notch_top.insert("stock", nt, aligned);
    notch_top.metadata("stock").display_during_debugging = false;

    //debug_arrangement(notch_top);

    homogeneous_transform bt =
      mating_transform(aligned, custom.base_fix.orient, custom.base_fix.v);

    rigid_arrangement base_clip;
    base_clip.insert("notch", bt, notch);
    base_clip.insert("part", bt, part_mesh);
    base_clip.insert("stock", bt, aligned);
    base_clip.metadata("stock").display_during_debugging = false;
    base_clip.insert("a_jaw", bt, a_jaw);
    base_clip.insert("an_jaw", bt, an_jaw);

    //    debug_arrangement(base_clip);
    
    vector<fixture_setup> clip_setups;
    clip_setups.push_back(clip_notch_transform(aligned, part_mesh, notch, top_surfs, top_fix));
    clip_setups.push_back(clip_base_transform(aligned, part_mesh, base_surfs, custom.base_fix));
    clip_setups.push_back(clip_base_transform(notch, part_mesh, {}, custom.clean_fix));
    return clip_setups;
  }
  
  clipping_plan
  soft_jaw_clipping_plan(const triangular_mesh& aligned,
			 const triangular_mesh& part_mesh,
			 const contour_surface_decomposition& surfs,
			 const fixture& top_fix,
			 const custom_jaw_cutout& custom) {
    std::vector<fixture_setup> clip_setups =
      soft_jaw_clip_setups(aligned, part_mesh, surfs, top_fix, custom);

    auto clipped_surfs =
      stable_surfaces_after_clipping(part_mesh, aligned);
    auto surfs_to_cut = surfs.rest;

    return clipping_plan(clipped_surfs, surfs_to_cut, clip_setups, {custom.left_jaw, custom.right_jaw});
  }
  
  boost::optional<clipping_plan>
  custom_jaw_plan(const triangular_mesh& aligned,
		  const triangular_mesh& part_mesh,
		  const contour_surface_decomposition& surfs,
		  const fixture& top_fix,
		  const fabrication_inputs& fab_inputs) {
    // TODO: Actually use the custom jaw cutout fixtures!
    boost::optional<custom_jaw_cutout> custom =
      custom_jaw_cutout_fixture(surfs, top_fix.v.without_extras(), fab_inputs);

    if (custom) {
      return soft_jaw_clipping_plan(aligned, part_mesh, surfs, top_fix, *custom);
    }

    return boost::none;
  }
  
  
}
