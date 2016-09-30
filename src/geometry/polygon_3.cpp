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
      delete_antennas(new_h);

      if (!(new_h.size() >= 3)) {
	cout << "ERROR: Inner ring size = " << h.size() << endl;
      } else {
	inner_rings.push_back(new_h);
      }
    }
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

}
