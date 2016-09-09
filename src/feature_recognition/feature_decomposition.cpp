#include <boost/numeric/ublas/io.hpp>

#include "feature_recognition/feature_decomposition.h"
#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/rotation.h"
#include "geometry/surface.h"
#include "geometry/vtk_debug.h"
#include "synthesis/contour_planning.h"
#include "utils/arena_allocator.h"
#include "utils/check.h"

namespace gca {

  void check_simplicity(const labeled_polygon_3& p) {
    check_simplicity(p.vertices());

    for (auto h : p.holes()) {
      check_simplicity(h);
    }
  }
  
  gca::feature* node_value(feature_decomposition* f) { return f->feature(); }

  std::vector<point> project_points(const plane pl,
				    const std::vector<point>& pts) {
    vector<point> res_pts;
    for (auto p : pts) {
      res_pts.push_back(project(pl, p));
    }

    return res_pts;
  }
  
  labeled_polygon_3 project_onto(const plane p,
				 const labeled_polygon_3& poly) {

    vector<point> proj_outer = project(p, poly.vertices());

    vector<vector<point>> proj_holes;
    for (auto h : poly.holes()) {
      proj_holes.push_back(project(p, h));
    }

    labeled_polygon_3 l(proj_outer, proj_holes);

    if (!(within_eps(angle_between(l.normal(), p.normal()), 0.0, 0.1))) {
      l.correct_winding_order(p.normal());
    }
    
    DBG_ASSERT(within_eps(angle_between(l.normal(), p.normal()), 0.0, 0.1));
    
    return l;
  }

  boost_poly_2
  to_boost_poly_2(const labeled_polygon_3& p) {
    boost_poly_2 pr;
    for (auto p : p.vertices()) {
      boost::geometry::append(pr, boost::geometry::model::d2::point_xy<double>(p.x, p.y));
    }

    boost::geometry::interior_rings(pr).resize(p.holes().size());
    for (int i = 0; i < p.holes().size(); i++) {
      auto& h = p.holes()[i];
      for (auto p : h) {
	boost::geometry::append(pr, boost::geometry::model::d2::point_xy<double>(p.x, p.y), i);
      }
    }

    // TODO: Add holes
    boost::geometry::correct(pr);
    
    return pr;
  }

  labeled_polygon_3
  to_labeled_polygon_3(const rotation& r, const double z, const boost_poly_2& p) {
    vector<point> vertices;
    for (auto p2d : boost::geometry::exterior_ring(p)) {
      point pt(p2d.get<0>(), p2d.get<1>(), z);
      vertices.push_back(times_3(r, pt));
    }

    vector<vector<point>> holes;
    for (auto ir : boost::geometry::interior_rings(p)) {
      vector<point> hole_verts;
      for (auto p2d : ir) {
	point pt(p2d.get<0>(), p2d.get<1>(), z);
	hole_verts.push_back(times_3(r, pt));
      }
      holes.push_back(clean_vertices(hole_verts));
    }
    return labeled_polygon_3(clean_vertices(vertices), holes);
  }

  // TODO: Version of this code that can handle holes?
  oriented_polygon to_oriented_polygon(const labeled_polygon_3& p) {
    return oriented_polygon(p.normal(), p.vertices());
  }

  labeled_polygon_3
  convex_hull_2D(const std::vector<point>& pts,
		 const point n,
		 const double z_level) {
    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);
    
    auto rotated_pts = apply(r, pts);

    boost_multipoint_2 mp;
    for (auto p : rotated_pts) {
      boost::geometry::append(mp, boost::geometry::model::d2::point_xy<double>(p.x, p.y));
    }

    boost_multipoint_2 res;
    boost::geometry::convex_hull(mp, res);

    vector<point> res_pts;
    for (auto p : res) {
      point pz(p.x(), p.y(), z_level);
      res_pts.push_back(times_3(r_inv, pz));
    }

    return labeled_polygon_3(clean_vertices(res_pts));
  }

  labeled_polygon_3
  convex_hull_2D(const triangular_mesh& m,
		 const point n) {
    double z_level = max_distance_along(m.vertex_list(), n);

    return convex_hull_2D(m.vertex_list(), n, z_level);
  }

  std::vector<point> projected_hull(const plane pl,
				    const std::vector<point>& raw_pts) {
    auto pts = project_points(pl, raw_pts);

    labeled_polygon_3 p =
      convex_hull_2D(pts, pl.normal(), max_distance_along(pts, pl.normal()));

    check_simplicity(p);
      
    p.correct_winding_order(pl.normal());

    return p.vertices();
  }

  std::vector<std::vector<index_t>>
  not_vertical_or_horizontal_regions(const triangular_mesh& m,
				     const std::vector<std::vector<index_t>>& surfs,
				     const point n) {
    auto not_vert_or_horiz =
      select(surfs, [n, m](const std::vector<index_t>& s) {
	  return !all_parallel_to(s, m, n, 3.0) &&
	  !all_orthogonal_to(s, m, n, 3.0) &&
	  !all_antiparallel_to(s, m, n, 3.0);
	});

    // Cull backfaces
    delete_if(not_vert_or_horiz,
	      [n, m](const std::vector<index_t>& inds) {
		return angle_between(m.face_orientation(inds.front()), n) > 90.0;
	      });

    

    vector<index_t> vz;
    for (auto s : not_vert_or_horiz) {
      concat(vz, s);
    }

    if (vz.size() == 0) { return {}; }

    return connect_regions(vz, m);
  }  

  std::vector<labeled_polygon_3>
  planar_polygon_union(const std::vector<labeled_polygon_3>& polys) {
    if (polys.size() == 0) { return {}; }

    // vtk_debug_polygon(p);
    //vtk_debug_polygons(polys);

    double level_z =
      max_distance_along(polys.front().vertices(), polys.front().normal());
    point n = polys.front().normal();

    cout << "n = " << n << endl;
    
    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    cout << "# polys to union = " << polys.size() << endl;

    boost_multipoly_2 result;
    result.push_back(to_boost_poly_2(apply(r, polys.front())));

    for (unsigned i = 1; i < polys.size(); i++) {
      auto& s = polys[i];

      auto bp = to_boost_poly_2(apply(r, s));
      boost_multipoly_2 r_tmp = result;
      boost::geometry::clear(result);
      boost::geometry::union_(r_tmp, bp, result);
    }

    cout << "# polys in result = " << result.size() << endl;

    std::vector<labeled_polygon_3> res;
    for (auto r : result) {
      labeled_polygon_3 lp = to_labeled_polygon_3(r_inv, level_z, r);

      check_simplicity(lp);

      lp.correct_winding_order(polys.front().normal());
      res.push_back(lp);
    }

    return res;
    
  }

  labeled_polygon_3
  virtual_surface_for_surface(const std::vector<index_t>& s,
			      const triangular_mesh& m,
			      const point n) {
    DBG_ASSERT(s.size() > 0);
    
    auto raw_pts = vertexes_on_surface(s, m);
    point max_pt = max_along(raw_pts, n);
    plane top(n, max_pt);

    std::vector<labeled_polygon_3> ts;
    for (auto i : s) {
      triangle t = m.face_triangle(i);
      vector<point> pts = project_points(top, {t.v1, t.v2, t.v3});

      if (no_duplicate_points(pts, 0.001)) {
	labeled_polygon_3 l(pts);

	check_simplicity(l);

	l.correct_winding_order(n);
      
	ts.push_back(l);
      }
    }

    //vtk_debug_highlight_inds(s, m);
    
    std::vector<labeled_polygon_3> result_polys =
      planar_polygon_union(ts);

    if (!(result_polys.size() == 1)) {

      vtk_debug_highlight_inds(s, m);
      
      vtk_debug_polygons(result_polys);
      
      DBG_ASSERT(result_polys.size() == 1);
    }

    return result_polys.front();
  }

  std::vector<labeled_polygon_3>
  build_virtual_surfaces(const triangular_mesh& m,
			 const std::vector<std::vector<index_t>>& surfs,
			 const point n) {
    auto not_vertical_or_horizontal =
      not_vertical_or_horizontal_regions(m, surfs, n);

    cout << "# of virtual surfaces = " << not_vertical_or_horizontal.size() << endl;

    vector<labeled_polygon_3> polys;
    for (auto s : not_vertical_or_horizontal) {
      auto l = virtual_surface_for_surface(s, m, n);
      polys.push_back(l);
    }

    return polys;
  }

  std::vector<labeled_polygon_3>
  horizontal_surfaces(const triangular_mesh& m, const point n) {
    auto inds = m.face_indexes();

    // TODO: More robust way to find constant orientation regions?
    vector<std::vector<index_t>> surfs =
      normal_delta_regions(inds, m, 3.0);

    auto virtual_surfaces =
      build_virtual_surfaces(m, surfs, n);

    // TODO: Add virtual polygons for surfaces that are non horizontal and
    // non vertical
    filter_non_horizontal_surfaces_wrt_dir(surfs, m, n);

    cout << "# of horizontal surfaces in " << n << " = " << surfs.size() << endl;

    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    triangular_mesh mr = apply(r, m);

    vector<labeled_polygon_3> surf_polys = virtual_surfaces;
    for (auto s : surfs) {
      DBG_ASSERT(s.size() > 0);

      auto bounds = mesh_bounds(s, mr);

      DBG_ASSERT(bounds.size() > 0);

      auto boundary = extract_boundary(bounds);

      check_simplicity(boundary.vertices());

      DBG_ASSERT(area(boundary) > 0.001);

      auto holes = bounds;

      vector<vector<point>> hole_verts;
      for (auto h : holes) {
	check_simplicity(h.vertices());
	hole_verts.push_back(apply(r_inv, h.vertices()));
      }

      labeled_polygon_3 lp(apply(r_inv, boundary.vertices()), hole_verts);

      lp.correct_winding_order(n);

      surf_polys.push_back(lp);
    }

    sort(begin(surf_polys), end(surf_polys),
	 [n](const labeled_polygon_3& x, const labeled_polygon_3& y) {
	   return max_distance_along(x.vertices(), n) <
	     max_distance_along(y.vertices(), n);
	 });

    cout << "Got horizontal surfaces" << endl;
    
    return surf_polys;
  }

  surface_levels
  initial_surface_levels(const triangular_mesh& m,
			 const point n) {
    auto h_surfs = horizontal_surfaces(m, n);
    auto surface_it = begin(h_surfs);
    surface_levels levels;
    while (surface_it != end(h_surfs)) {
      const labeled_polygon_3& current_surf = *surface_it;
      double current_surf_dist = max_distance_along(current_surf.vertices(), n);
      auto next_it =
	find_if_not(surface_it, end(h_surfs),
		    [current_surf_dist, n](const labeled_polygon_3& p) {
		      double pd = max_distance_along(p.vertices(), n);
		      return within_eps(current_surf_dist, pd, 0.01);
		    });
      vector<labeled_polygon_3> level(surface_it, next_it);
      levels.push_back(level);

      surface_it = next_it;
    }
    return levels;
  }

  labeled_polygon_3 initial_outline(const triangular_mesh& m,
				    const point n) {

    labeled_polygon_3 top_and_bottom_outline =
      convex_hull_2D(m, n);

    point top_point = max_point_in_dir(m, n);
    plane max_plane(n, top_point);

    labeled_polygon_3 top_poly = project_onto(max_plane, top_and_bottom_outline);

    check_simplicity(top_poly.vertices());
    
    top_poly.correct_winding_order(n);

    plane eps_pl(top_poly.normal(), top_poly.vertex(0) + 0.01*top_poly.normal());
    return project_onto(eps_pl, top_poly);
  }

  labeled_polygon_3 apply(const rotation& r, const labeled_polygon_3& p) {
    vector<point> pts = apply(r, p.vertices());

    vector<vector<point>> holes;
    for (auto h : p.holes()) {
      holes.push_back(apply(r, h));
    }

    labeled_polygon_3 rotated(pts, holes);

    rotated.correct_winding_order(times_3(r, p.normal()));

    point rnorm = rotated.normal();
    point pnorm = p.normal();
    point rtnorm = times_3(r, p.normal());

    // cout << "Original normal             = " << pnorm << endl;
    // cout << "Rotated normal              = " << rnorm << endl;
    // cout << "Rotation of original normal = " << rtnorm << endl;
    
    double theta = angle_between(rotated.normal(), rtnorm);
  
    DBG_ASSERT(within_eps(theta, 0.0, 0.1));

    return rotated;
  }

  std::vector<point> apply(const homogeneous_transform& t,
			   const std::vector<point>& pts) {
    vector<point> rpts;
    for (auto p : pts) {
      rpts.push_back(apply(t, p));
    }
    return rpts;
  }

  labeled_polygon_3 apply(const homogeneous_transform& t,
			  const labeled_polygon_3& p) {
    vector<point> rotated_verts = apply(t, p.vertices());

    vector<vector<point>> holes;
    for (auto h : p.holes()) {
      holes.push_back(apply(t, h));
    }

    labeled_polygon_3 transformed(rotated_verts, holes);
    
    transformed.correct_winding_order(times_3(t.first, p.normal()));

    point rnorm = transformed.normal();
    point pnorm = p.normal();
    point rtnorm = times_3(t.first, p.normal());

    // cout << "Original normal             = " << pnorm << endl;
    // cout << "Transformed normal              = " << rnorm << endl;
    // cout << "Rotation of original normal = " << rtnorm << endl;
    
    double theta = angle_between(transformed.normal(), rtnorm);
  
    DBG_ASSERT(within_eps(theta, 0.0, 0.1));

    return transformed;
  }

  labeled_polygon_3 dilate(const labeled_polygon_3& p, const double tol) {
    auto dr = exterior_offset(p.vertices(), tol);

    vector<vector<point>> dh;
    for (auto h : p.holes()) {
      dh.push_back(interior_offset(h, tol));
    }

    labeled_polygon_3 poly(dr, dh);
    return poly;
  }

  labeled_polygon_3 shift(const point p, const labeled_polygon_3& poly) {
    auto dr = shift(p, poly.vertices());

    vector<vector<point>> dh;
    for (auto h : poly.holes()) {
      dh.push_back(shift(p, h));
    }

    labeled_polygon_3 shifted(dr, dh);
    return shifted;
  }

  labeled_polygon_3 shrink(const labeled_polygon_3& p, const double tol) {
    auto dr = interior_offset(p.vertices(), tol);

    vector<vector<point>> dh;
    for (auto h : p.holes()) {
      dh.push_back(exterior_offset(h, tol));
    }

    labeled_polygon_3 poly(dr, dh);
    return poly;
  }
  
  labeled_polygon_3 smooth_buffer(const labeled_polygon_3& p,
				  const double tol) {
    return shrink(dilate(p, tol), tol);
  }
  
  std::vector<labeled_polygon_3>
  dilate_polygons(const std::vector<labeled_polygon_3>& polys, const double tol) {
    std::vector<labeled_polygon_3> dilated_polys;
    for (auto p : polys) {
      dilated_polys.push_back(dilate(p, tol));
    }
    return dilated_polys;
  }


  boost::optional<std::vector<labeled_polygon_3>>
  subtract_level(const labeled_polygon_3& p,
		 const std::vector<labeled_polygon_3>& to_subtract) {
    DBG_ASSERT(to_subtract.size() > 0);

    cout << "Subtracting" << endl;

    //vtk_debug_polygon(p);
    //vtk_debug_polygons(to_subtract);

    double level_z =
      max_distance_along(to_subtract.front().vertices(), p.normal());
    point n = to_subtract.front().normal();

    cout << "n = " << n << endl;
    
    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    // cout << "rotation = " << r << endl;
    // cout << "r*n = " << times_3(r, n) << endl;

    boost_poly_2 pb = to_boost_poly_2(apply(r, p));

    auto to_subtract_dilated = dilate_polygons(to_subtract, 0.00001);

    cout << "# polys to subtract = " << to_subtract_dilated.size() << endl;

    boost_multipoly_2 result;
    result.push_back(pb);

    for (auto s : to_subtract_dilated) {
      auto bp = to_boost_poly_2(apply(r, s));
      boost_multipoly_2 r_tmp = result;
      boost::geometry::clear(result);
      boost::geometry::difference(r_tmp, bp, result);
    }

    cout << "# polys in result = " << result.size() << endl;

    bool result_same = false;

    if (result.size() > 0) {
      auto lr =
    	max_element(begin(result), end(result),
    		    [](const boost_poly_2& l, const boost_poly_2& r)
    		    { return boost::geometry::area(l) < boost::geometry::area(r); });

      auto& largest_result = *lr;

      // The result is the same as before
      if (within_eps(boost::geometry::area(largest_result),
    		     boost::geometry::area(pb),
    		     0.0005)) {
	cout << "Result is the same" << endl;
	result_same = true;
    	//return boost::none;
      }
    }

    // TODO: How to preserve edge labels etc?
    std::vector<labeled_polygon_3> res;
    for (auto r : result) {
      if (boost::geometry::area(r) > 0.001) {
	labeled_polygon_3 lp = to_labeled_polygon_3(r_inv, level_z, r);

	check_simplicity(lp);

	lp.correct_winding_order(p.normal());
	res.push_back(lp);
      }
    }

    //cout << "RESULT OF SUBTRACTION" << endl;
    //vtk_debug_polygons(res);

    if (result_same) { return boost::none; }

    return res;
  }

  void
  decompose_volume(const labeled_polygon_3& current_level,
		   surface_levels levels,
		   feature_decomposition* parent,
		   const double base_depth) {
    cout << "decompose" << endl;
    cout << "Levels left = " << levels.size() << endl;

    double current_depth =
      max_distance_along(current_level.vertices(), current_level.normal());

    // cout << "current normal = " << current_level.normal() << endl;
    // cout << "current depth  = " << current_depth << endl;
    // cout << "base depth     = " << base_depth << endl;

    DBG_ASSERT(current_depth >= base_depth);
    
    //    vtk_debug_polygon(current_level);
    
    // If there are no levels left then the current_level defines
    // a through feature
    if (levels.size() == 0) {
      double feature_depth = current_depth - base_depth;
      point v = feature_depth*current_level.normal();
      plane base_pl(current_level.normal(), current_level.vertex(0) - v);

      labeled_polygon_3 shifted = project_onto(base_pl, current_level);

      feature* f = new (allocate<feature>()) feature(feature_depth, shifted);
      feature_decomposition* child =
    	new (allocate<feature_decomposition>()) feature_decomposition(f);
      parent->add_child(child);

      return;
    }

    const std::vector<labeled_polygon_3>& level_polys = levels.back();

    boost::optional<vector<labeled_polygon_3>> r_polys =
      subtract_level(current_level, level_polys);

    double next_depth =
      max_distance_along(levels.back().front().vertices(),
			 levels.back().front().normal());

    levels.pop_back();

    // The result is exactly the same as before, just recurse with no
    // updates
    if (!r_polys) {
      decompose_volume(current_level, levels, parent, base_depth);
      return;
    }

    auto result_polys = *r_polys;

    // Add a new feature for the current level
    // and recursively build decompositions for each new level
    // produced by the subtraction
    cout << "Current depth = " << current_depth << endl;
    cout << "Next depth    = " << next_depth << endl;

    if (!(current_depth >= next_depth)) {
      cout << "Direction = " << current_level.normal() << endl;
      
      vtk_debug_polygon(current_level);
      for (auto p : *r_polys) {
	vtk_debug_polygon(p);
	
      }
      DBG_ASSERT(false);
    }

    double feature_depth = current_depth - next_depth;

    point v = feature_depth*current_level.normal();
    plane base_pl(current_level.normal(), current_level.vertex(0) - v);

    labeled_polygon_3 shifted = project_onto(base_pl, current_level);
    
    feature* f = new (allocate<feature>()) feature(feature_depth, shifted);
    feature_decomposition* child =
      new (allocate<feature_decomposition>()) feature_decomposition(f);
    parent->add_child(child);

    vector<feature_decomposition*> children;
    for (auto r : result_polys) {
      decompose_volume(r, levels, child, base_depth);
    }

  }
  
  feature_decomposition*
  build_feature_decomposition(const triangular_mesh& m, const point n) {
    surface_levels levels = initial_surface_levels(m, n);

    labeled_polygon_3 init_outline = initial_outline(m, n);

    DBG_ASSERT(within_eps(angle_between(init_outline.normal(), n), 0.0, 0.01));
    
    double base_depth = min_distance_along(m.vertex_list(), n);

    // cout << "initial # of levels = " << levels.size() << endl;
    for (auto level : levels) {
      DBG_ASSERT(level.size() > 0);

      // cout << "??? z = " << level.front().vertex(0).z << endl;
      // cout << "Distance along = " <<
      // 	max_distance_along(level.front().vertices(), n) << endl;
    }
    // cout << "done levels" << endl;

    feature_decomposition* decomp =
      new (allocate<feature_decomposition>()) feature_decomposition();

    decompose_volume(init_outline, levels, decomp, base_depth);

    auto check_normal = [n](const feature* f) {
      if (f != nullptr) {
	DBG_ASSERT(f->depth() >= 0);
	DBG_ASSERT(within_eps(angle_between(n, f->normal()), 0.0, 1.0));
      }
    };

    traverse_bf(decomp, check_normal);

    return decomp;
  }

  vector<feature*> collect_features(feature_decomposition* f) {
    DBG_ASSERT(f != nullptr);
    
    vector<feature*> features;

    auto func = [&features](feature* f) {
      if (f != nullptr)
	{ features.push_back(f); }
    };

    traverse_bf(f, func);

    return features;
  }

  vector<feature*> collect_leaf_features(feature_decomposition* f) {
    DBG_ASSERT(f != nullptr);
    
    vector<feature*> features;

    auto func = [&features](feature* f) {
      if (f != nullptr)
	{ features.push_back(f); }
    };

    traverse_leaves_bf(f, func);

    return features;
  }
  
  point normal(feature_decomposition* f) {
    point n;
    auto check_normal = [&n](const feature* f) {
      if (f != nullptr) {
	n = f->normal();
      }
    };

    traverse_bf(f, check_normal);

    return n;
  }

  bool tangent(const std::pair<double, double> r1,
	       const std::pair<double, double> r2,
	       const double tol) {
    DBG_ASSERT(r1.first < r1.second);
    DBG_ASSERT(r2.first < r2.second);

    if (within_eps(r1.second, r2.first, tol)) { return true; }
    if (within_eps(r2.second, r1.first, tol)) { return true; }

    return false;
  }

  // TODO: Actually implement this when needed
  std::vector<feature*>
  range_containing(const point n,
		   const feature& maybe_contained,
		   const std::vector<feature*>& maybe_container) {

    auto contained_range = maybe_contained.range_along(n);

    vector<feature*> container;
    for (auto f : maybe_container) {
      auto f_range = f->range_along(n);
      if (intervals_overlap(f_range, contained_range)) {

	// Do not include features that just border the feature
	// in the containing range
	if (!tangent(f_range, contained_range, 0.001)) {
	  container.push_back(f);
	}
      }
    }

    return container;
  }

  // TODO: Should really be containing outlines
  std::vector<feature*>
  overlapping_outlines(const feature& maybe_contained,
		       const std::vector<feature*>& container) {
    vector<feature*> overlapping;

    point n = maybe_contained.normal();

    const rotation r = rotate_from_to(n, point(0, 0, 1));

    labeled_polygon_3 rmc = apply(r, maybe_contained.base());
    auto rmcbp = to_boost_poly_2(rmc);
    //    double rmcbp_area = boost::g::area(rmcbp);
    
    for (auto c : container) {
      DBG_ASSERT(c != nullptr);

      labeled_polygon_3 rc = apply(r, c->base());
      rc.correct_winding_order(point(0, 0, 1));

      auto rcbp = to_boost_poly_2(rc);

      if (boost::geometry::within(rmcbp, rcbp)) {
	overlapping.push_back(c);
      } else {
	//      double rcbp_area = bg::area(rcbp);
	boost_multipoly_2 res;
	boost::geometry::sym_difference(rmcbp, rcbp, res);
      
	double not_shared_area = boost::geometry::area(res);

	if (not_shared_area < 0.001) {
	  overlapping.push_back(c);
	}
      }
    }

    cout << "# of overlapping = " << overlapping.size() << endl;

    return overlapping;
  }
  
  std::vector<feature*>
  containing_subset(const feature& maybe_contained,
		    const std::vector<feature*>& container) {
    for (auto c : container) {
      DBG_ASSERT(angle_eps(maybe_contained.normal(), c->normal(), 180.0, 1.0) ||
		 angle_eps(maybe_contained.normal(), c->normal(), 0.0, 1.0));
    }

    vector<feature*> outlines = overlapping_outlines(maybe_contained, container);

    point n = maybe_contained.normal();
    return range_containing(n, maybe_contained, outlines);
  }

  bool contains(const feature& maybe_contained,
		const std::vector<feature*>& container) {
    for (auto c : container) {
      DBG_ASSERT(angle_eps(maybe_contained.normal(), c->normal(), 180.0, 1.0));
    }

    vector<feature*> sub =
      containing_subset(maybe_contained, container);

    return sub.size() > 0;
  }
  

  bool same_base(const feature& l, const feature& r, const double tol) {
    auto l_p = l.base();
    auto r_p = r.base();

    const rotation rot = rotate_from_to(l_p.normal(), point(0, 0, 1));

    auto l_pr = to_boost_poly_2(apply(rot, l_p));
    auto r_pr = to_boost_poly_2(apply(rot, r_p));

    boost_multipoly_2 res;
    boost::geometry::sym_difference(l_pr, r_pr, res);

    return boost::geometry::area(res) < tol;
  }

  feature* parent_feature(feature* f, feature_decomposition* decomp) {
    for (unsigned i = 0; i < decomp->num_children(); i++) {
      feature* child_feature = decomp->child(i)->feature();

      if (f == child_feature) { return decomp->feature(); }
    }

    for (unsigned i = 0; i < decomp->num_children(); i++) {
      feature* possible_parent = parent_feature(f, decomp->child(i));

      if (possible_parent != nullptr) { return possible_parent; }
    }

    return nullptr;
  }

  boost_poly_2 rotate_to_2D_polygon(const labeled_polygon_3& p) {
    const rotation r = rotate_from_to(p.normal(), point(0, 0, 1));
    return to_boost_poly_2(apply(r, p));
  }

  double base_area(const feature& f) {
    boost_poly_2 bp = rotate_to_2D_polygon(f.base());
    return boost::geometry::area(bp);
  }

}
