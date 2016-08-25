#include <boost/numeric/ublas/io.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

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

  typedef boost::geometry::model::d2::point_xy<double> boost_point_2;
  typedef boost::geometry::model::polygon<boost_point_2> boost_poly_2;
  typedef boost::geometry::model::multi_polygon<boost_poly_2> boost_multipoly_2;
  typedef boost::geometry::model::multi_point<boost_point_2> boost_multipoint_2;

  void check_simplicity(const labeled_polygon_3& p) {
    check_simplicity(p.vertices());

    for (auto h : p.holes()) {
      check_simplicity(h);
    }
  }
  
  gca::feature* node_value(feature_decomposition* f) { return f->feature(); }

  point project(const plane pl, const point p) {
    // double d = -1*(pl.normal().dot(pl.pt()));

    // double scalar_distance_to_plane = pl.normal().dot(p) + d;

    // point projected = p + (-1*scalar_distance_to_plane)*pl.normal();

    // cout << "Original distance to plane  = " << scalar_distance_to_plane << endl;
    // cout << "New point distance to plane = " << pl.normal().dot(projected) + d << endl;
    // cout << "d                           = " << d << endl;
    
    //    DBG_ASSERT(within_eps(pl.normal().dot(projected) + d, 0, 0.001));

    //    return projected;
    
    point v = project_onto(pl.pt() - p, pl.normal());
    return p + v;
  }

  std::vector<point> project(const plane pl, const std::vector<point>& pts) {
    std::vector<point> ppts;

    // TODO: Should really check that pts is planar
    for (unsigned i = 0; i < pts.size(); i++) {
      point p = pts[i];
      //      ppts.push_back(project(pl, p));
      
      point pp1 = pts[(i + 1) % pts.size()];
      point diff = pp1 - p;
      if (!within_eps(angle_between(diff, pl.normal()), 0.0, 1.0) &&
      	  !within_eps(angle_between(diff, pl.normal()), 180.0, 1.0)) {
	ppts.push_back(project(pl, p));
      }
    }

    // TODO: Add tolerance as parameter?
    // TODO: Actual ring unique function?

    auto r_eq = [](const point x, const point y)
      { return components_within_eps(x, y, 0.001); };

    auto last = std::unique(begin(ppts), end(ppts), r_eq);
    ppts.erase(last, end(ppts));

    if (r_eq(ppts.front(), ppts.back())) {
      ppts.pop_back();
    }

    check_simplicity(ppts);

    return ppts;
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

  std::vector<labeled_polygon_3>
  horizontal_surfaces(const triangular_mesh& m, const point n) {
    vector<std::vector<index_t>> surfs = const_orientation_regions(m);

    // TODO: Add virtual polygons for surfaces that are non horizontal and
    // non vertical
    filter_non_horizontal_surfaces_wrt_dir(surfs, m, n);

    cout << "# of horizontal surfaces in " << n << " = " << surfs.size() << endl;

    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    triangular_mesh mr = apply(r, m);

    vector<labeled_polygon_3> surf_polys;
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

  labeled_polygon_3
  convex_hull_2D(const triangular_mesh& m,
		 const point n) {
    double z_level = max_distance_along(m.vertex_list(), n);
    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);
    
    auto rotated_pts = apply(r, m.vertex_list());

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

    return labeled_polygon_3(res_pts);
  }

  labeled_polygon_3 initial_outline(const triangular_mesh& m,
				    const point n) {
    //    oriented_polygon out = convex_hull_2D(m, n);

    labeled_polygon_3 top_and_bottom_outline =
      convex_hull_2D(m, n); //(out->vertices());

    cout << "Top and bottom outline has " << top_and_bottom_outline.vertices().size() << "vertices" << endl;
    vtk_debug_polygon(top_and_bottom_outline);

    point top_point = max_point_in_dir(m, n);
    plane max_plane(n, top_point);

    labeled_polygon_3 top_poly = project_onto(max_plane, top_and_bottom_outline);

    cout << "Projected outline has " << top_poly.vertices().size() << "vertices" << endl;
    
    check_simplicity(top_poly.vertices());
    
    vtk_debug_polygon(top_poly);
    
    top_poly.correct_winding_order(n);

    return top_poly;
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

    cout << "Original normal             = " << pnorm << endl;
    cout << "Rotated normal              = " << rnorm << endl;
    cout << "Rotation of original normal = " << rtnorm << endl;
    
    double theta = angle_between(rotated.normal(), rtnorm);
  
    DBG_ASSERT(within_eps(theta, 0.0, 0.1));

    return rotated;
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

  std::vector<point> clean_vertices(const std::vector<point>& pts) {
    if (pts.size() < 3) { return pts; }

    vector<point> rpts = pts;
    if (components_within_eps(pts.front(), pts.back(), 0.001)) {
      rpts.pop_back();
    }

    return rpts;
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
    //    DBG_ASSERT(p.holes().size() == 0);

    return oriented_polygon(p.normal(), p.vertices());
  }

  boost::optional<std::vector<labeled_polygon_3>>
  subtract_level(const labeled_polygon_3& p,
		 const std::vector<labeled_polygon_3>& to_subtract) {
    DBG_ASSERT(to_subtract.size() > 0);

    double level_z =
      max_distance_along(to_subtract.front().vertices(), p.normal()); //vertex(0).z;
    point n = to_subtract.front().normal();

    cout << "n = " << n << endl;
    
    const rotation r = rotate_from_to(n, point(0, 0, 1));
    const rotation r_inv = inverse(r);

    cout << "rotation = " << r << endl;
    cout << "r*n = " << times_3(r, n) << endl;

    boost_poly_2 pb = to_boost_poly_2(apply(r, p));

    boost_multipoly_2 to_sub;
    for (auto s : to_subtract) {
      to_sub.push_back(to_boost_poly_2(apply(r, s)));
    }
    
    cout << "# polys to subtract = " << to_sub.size() << endl;

    //vtk_debug_polygon(to_oriented_polygon(apply(r, p)));

    boost_multipoly_2 result;
    boost::geometry::difference(pb, to_sub, result);

    cout << "# polys in result = " << result.size() << endl;

    if (result.size() > 0) {
      auto lr =
	max_element(begin(result), end(result),
		    [](const boost_poly_2& l, const boost_poly_2& r)
		    { return boost::geometry::area(l) < boost::geometry::area(r); });

      auto& largest_result = *lr;
      
      // The result is the same as before
      if (within_eps(boost::geometry::area(largest_result),
		     boost::geometry::area(pb),
		     0.005)) {
	return boost::none;
      }
    }

    // TODO: How to preserve edge labels etc?
    std::vector<labeled_polygon_3> res;
    for (auto r : result) {
      if (boost::geometry::area(r) > 0.001) {
	labeled_polygon_3 lp = to_labeled_polygon_3(r_inv, level_z, r);

	vtk_debug_polygon(lp);

	check_simplicity(lp);

	lp.correct_winding_order(p.normal());
	res.push_back(lp);
      }
    }

    for (auto rp : res) {
      //vtk_debug_polygon(to_oriented_polygon(rp));
    }

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

    cout << "current normal = " << current_level.normal() << endl;
    cout << "current depth  = " << current_depth << endl;
    cout << "base depth     = " << base_depth << endl;

    DBG_ASSERT(current_depth >= base_depth);
    
    //    vtk_debug_polygon(current_level);
    
    // If there are no levels left then the current_level defines
    // a through feature
    if (levels.size() == 0) {
      double feature_depth = current_depth - base_depth;
      feature* f = new (allocate<feature>()) feature(feature_depth, current_level);
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
    DBG_ASSERT(current_depth >= next_depth);

    double feature_depth = current_depth - next_depth;

    feature* f = new (allocate<feature>()) feature(feature_depth, current_level);
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

    cout << "initial # of levels = " << levels.size() << endl;
    for (auto level : levels) {
      DBG_ASSERT(level.size() > 0);

      cout << "??? z = " << level.front().vertex(0).z << endl;
    }
    cout << "done levels" << endl;

    feature_decomposition* decomp =
      new (allocate<feature_decomposition>()) feature_decomposition();

    decompose_volume(init_outline, levels, decomp, base_depth);

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
}
