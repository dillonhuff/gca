#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/polygon_3.h"
#include "geometry/vtk_debug.h"

namespace gca {

  
  polygon_3::polygon_3(const std::vector<point> vertices,
		       const std::vector<std::vector<point>> hole_verts) :
    outer_ring(vertices),
    inner_rings{} {

    outer_ring = clean_vertices(outer_ring);
    delete_antennas(outer_ring);

    // There is an occasional test failure here in simple box
    if (!(outer_ring.size() >= 3)) {
      cout << "ERROR: Outer ring size = " << outer_ring.size() << endl;
      vtk_debug_ring(outer_ring);

      DBG_ASSERT(outer_ring.size() >= 3);
    }

    for (auto h : hole_verts) {

      auto new_h = clean_vertices(h);
      cout << "NEW_H delete start" << endl;
      delete_antennas(new_h);
      cout << "END NEW_H delete start" << endl;

      if (!(new_h.size() >= 3)) {
	cout << "ERROR: Inner ring size = " << h.size() << endl;
      } else {
	inner_rings.push_back(new_h);
      }
    }
  }

  std::vector<point>
  clean_for_conversion_to_polygon_3(const std::vector<point>& vertices) {
    auto outer_ring = vertices;

    outer_ring = clean_vertices(outer_ring);

    if (outer_ring.size() < 3) { return outer_ring; }

    delete_antennas(outer_ring);

    return outer_ring;
  }

  void check_simplicity(const labeled_polygon_3& p) {
    check_simplicity(static_cast<const std::vector<point>&>(p.vertices()));

    for (auto h : p.holes()) {
      check_simplicity(static_cast<const std::vector<point>&>(h));
    }
  }

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

  polygon_3
  to_polygon_3(const double z, const boost_poly_2& p) {
    vector<point> vertices;
    for (auto p2d : boost::geometry::exterior_ring(p)) {
      point pt(p2d.get<0>(), p2d.get<1>(), z);
      vertices.push_back(pt);
    }

    vector<vector<point>> holes;
    for (auto ir : boost::geometry::interior_rings(p)) {
      vector<point> hole_verts;
      for (auto p2d : ir) {
	point pt(p2d.get<0>(), p2d.get<1>(), z);
	hole_verts.push_back(pt);
      }
      holes.push_back(clean_vertices(hole_verts));
    }
    return labeled_polygon_3(clean_vertices(vertices), holes);

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

  boost::optional<polygon_3>
  to_labeled_polygon_3_maybe(const rotation& r,
			     const double z,
			     const boost_poly_2& p) {
    vector<point> vertices;
    for (auto p2d : boost::geometry::exterior_ring(p)) {
      point pt(p2d.get<0>(), p2d.get<1>(), z);
      vertices.push_back(times_3(r, pt));
    }

    vertices = clean_for_conversion_to_polygon_3(vertices);

    if (vertices.size() < 3) { return boost::none; }

    vector<vector<point>> holes;
    for (auto ir : boost::geometry::interior_rings(p)) {
      vector<point> hole_verts;
      for (auto p2d : ir) {
	point pt(p2d.get<0>(), p2d.get<1>(), z);
	hole_verts.push_back(times_3(r, pt));
      }

      hole_verts = clean_for_conversion_to_polygon_3(hole_verts);
      if (hole_verts.size() >= 3) {
	holes.push_back(hole_verts);
      }
    }

    return labeled_polygon_3(vertices, holes);
  }

  // TODO: Version of this code that can handle holes?
  oriented_polygon to_oriented_polygon(const labeled_polygon_3& p) {
    return oriented_polygon(p.normal(), p.vertices());
  }

  boost_poly_2 rotate_to_2D(const labeled_polygon_3& p) {
    const rotation r = rotate_from_to(p.normal(), point(0, 0, 1));
    return to_boost_poly_2(apply(r, p));
  }

  double area(const polygon_3& p) {
    auto td = rotate_to_2D(p);
    return bg::area(td);
  }

  boost_multipoly_2
  planar_union_boost(const std::vector<polygon_3>& polys) {
    if (polys.size() == 0) { return {}; }

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

    return result;
  }

  // TODO: Use planar_union_boost as starting point here?
  std::vector<labeled_polygon_3>
  planar_polygon_union(const std::vector<labeled_polygon_3>& polys) {
    if (polys.size() == 0) { return {}; }

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
    for (auto& r : result) {
      boost::optional<polygon_3> lp =
	to_labeled_polygon_3_maybe(r_inv, level_z, r);

      if (lp) {
	check_simplicity(*lp);

	(*lp).correct_winding_order(polys.front().normal());
	res.push_back(*lp);
      }
    }

    return res;
    
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

}
