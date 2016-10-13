#include <cassert>

#include "feature_recognition/visual_debug.h"
#include "geometry/offset.h"
#include "geometry/ring.h"
#include "geometry/rotation.h"

#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>

#include "geometry/vtk_debug.h"

namespace gca {

  typedef boost::shared_ptr<Polygon_2> PolygonPtr ;
  typedef std::vector<PolygonPtr> PolygonPtrVector ;
  typedef CGAL::Straight_skeleton_2<K> Ss ;

  typedef CGAL::Polygon_with_holes_2<K> PolygonWithHoles ;
  typedef boost::shared_ptr<PolygonWithHoles> PolygonWithHolesPtr ;
  typedef std::vector<PolygonWithHolesPtr> PolygonWithHolesPtrVector;
  
  typedef boost::shared_ptr<Ss> SsPtr ;  

  Polygon_2
  CGAL_polygon_for_oriented_polygon(const oriented_polygon& p) {
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    return out;
  }

  Polygon_2
  CGAL_polygon_for_points(const std::vector<point>& pts) {
    Polygon_2 out;
    for (auto p : pts) {
      out.push_back(Point(p.x, p.y));
    }

    return out;
  }

  vector<point>
  ring_for_CGAL_polygon(const Polygon_2& off_p,
			const double z) {
    vector<point> res_pts;
    for (auto it = CGAL::CGAL_SS_i::vertices_begin(off_p);
	 it != CGAL::CGAL_SS_i::vertices_end(off_p); ++it) {
      Point vert = *it;
      res_pts.push_back(point(vert.x(), vert.y(), z));
    }

    return res_pts;
  }  

  oriented_polygon
  oriented_polygon_for_CGAL_polygon(const Polygon_2& off_p,
				    const double z,
				    const point n) {
    vector<point> res_pts;
    for (auto it = CGAL::CGAL_SS_i::vertices_begin(off_p);
	 it != CGAL::CGAL_SS_i::vertices_end(off_p); ++it) {
      Point vert = *it;
      res_pts.push_back(point(vert.x(), vert.y(), z));
    }
    return oriented_polygon(n, res_pts);
  }

  std::vector<oriented_polygon> exterior_offset(const oriented_polygon& q,
						const double inc) {
    DBG_ASSERT(q.vertices().size() > 0);

    oriented_polygon p;
    if (signed_area(q) < 0) {
      p = q;
    } else {
      vector<point> pts = q.vertices();
      reverse(begin(pts), end(pts));
      p = oriented_polygon(q.normal, pts);
    }

    DBG_ASSERT(p.vertices().size() > 0);
    
    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    if (!(out.is_simple())) {
      vtk_debug_polygon(p);
      DBG_ASSERT(false);
    }
    
    if (out.orientation() == CGAL::CLOCKWISE) {
      out.reverse_orientation();
    }

    DBG_ASSERT(out.orientation() == CGAL::COUNTERCLOCKWISE);

    DBG_ASSERT(out.is_simple());

    PolygonPtrVector inner_offset_polygons =
      CGAL::create_exterior_skeleton_and_offset_polygons_2(inc, out);
    
    vector<oriented_polygon> results;
    for (auto off_ptr : inner_offset_polygons) {
      Polygon_2 off_p = *off_ptr;
      auto op = oriented_polygon_for_CGAL_polygon(off_p, z_va, p.normal);
      results.push_back(op);
    }
    return results;
  }

  std::vector<oriented_polygon> interior_offset(const oriented_polygon& p,
						const double inc) {

    DBG_ASSERT(p.vertices().size() > 0);

    check_simplicity(p);

    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    if (!(out.is_simple())) {
      vtk_debug_polygon(p);
      DBG_ASSERT(false);
    }
    
    if (out.orientation() == CGAL::CLOCKWISE) {
      out.reverse_orientation();
    }

    DBG_ASSERT(out.orientation() == CGAL::COUNTERCLOCKWISE);

    //    cout << "Starting to compute offset skeleton" << endl;
    SsPtr ss = CGAL::create_interior_straight_skeleton_2(out);
    //    cout << "Done computing to offset skeleton" << endl;

    if (!ss) {
      cout << "straight skeleton is null" << endl;
      for (unsigned i = 0; i < p.vertices().size(); i++) {
	point pt = p.vertices()[i];
	unsigned i1 = (i + 1) % p.vertices().size();
	point pt1 = p.vertices()[i1];

	point dir = (pt1 - pt).normalize();
	
	cout << pt << "         with direction = " << dir << endl;
      }

      vtk_debug_ring(p.vertices());
      DBG_ASSERT(false);
    }

    //    cout << "Starting to compute the interior offset" << endl;
    PolygonPtrVector inner_offset_polygons =
      CGAL::create_offset_polygons_2<Polygon_2>(inc, *ss);
      //  CGAL::create_interior_skeleton_and_offset_polygons_2(inc, out);
    //    cout << "Done computing interior offset" << endl;

    vector<oriented_polygon> results;
    for (auto off_ptr : inner_offset_polygons) {
      Polygon_2 off_p = *off_ptr;
      auto op = oriented_polygon_for_CGAL_polygon(off_p, z_va, p.normal);
      results.push_back(op);
    }
    return results;
  }

  void check_simplicity(const oriented_polygon& p) {
    if (!CGAL_polygon_for_oriented_polygon(p).is_simple()) {
      vtk_debug_polygon(p);
      DBG_ASSERT(false);
    }
  }

  void check_simplicity(const std::vector<point>& rpts) {
    const gca::rotation r = rotate_from_to(ring_normal(rpts), point(0, 0, 1));
    auto pts = apply(r, rpts);

    if (!(no_duplicate_points(pts, 0.0001))) {
      vtk_debug_ring(pts);
      DBG_ASSERT(false);
    }
    
    if (!CGAL_polygon_for_points(pts).is_simple()) {
      cout << "Non simple ring!" << endl;
      for (auto p : pts) {
	cout << p.x << " , " << p.y << " , " << p.z << endl;
      }
      vtk_debug_ring(pts);
      DBG_ASSERT(false);
    }
  }

  std::vector<point> exterior_offset(const std::vector<point>& pts,
				     const double tol) {

    DBG_ASSERT(pts.size() > 2);
    
    point n(0, 0, 1);
    const rotation r = rotate_from_to(ring_normal(pts), n);
    const rotation r_inv = inverse(r);
    auto r_pts = apply(r, pts);

    auto original_rpts = r_pts;

    r_pts = clean_vertices_within_eps(r_pts, 0.005, 0.0000001);

    auto old_rpts = r_pts;

    delete_antennas(r_pts);

    if (r_pts.size() < 3) {
      cout << "Before cleaning" << endl;
      cout << "# of points = " << pts.size() << endl;

      vtk_debug_ring(pts);

      cout << "After cleaning" << endl;
      cout << "# of points = " << old_rpts.size() << endl;

      vtk_debug_ring(old_rpts);

      cout << "# of points after deleting antennas = " << r_pts.size() << endl;

      DBG_ASSERT(false);
    }

    if (has_antenna(r_pts)) {
      DBG_ASSERT(false);
    }

    check_simplicity(r_pts);

    
    auto res = exterior_offset(oriented_polygon(n, r_pts), tol);

    if (res.size() != 2) {
      cout << "Wrong number of exterior offsets!" << endl;
      cout << "# of exterior offsets = " << res.size() << endl;
      cout << "tol = " << tol << endl;

      cout << "# of points in ring = " << r_pts.size() << endl;
      cout << "vector<point> pts{" << endl;
      for (auto p : r_pts) {
	cout << "point(" << p.x << ", " << p.y << ", " << p.z << ")" << ", " << endl;
      }
      cout << "};" << endl;



      vtk_debug_ring(r_pts);
      for (auto r : res) {
	vtk_debug_ring(r.vertices());
      }

      cout << "res" << endl;
      for (auto res_p : res) {
	vtk_debug_polygon(res_p);
      }

      DBG_ASSERT(res.size() == 2);
    }

    auto rpoly = res[1];
    auto rpts = apply(r_inv, rpoly.vertices());

    correct_winding_order(rpts, ring_normal(pts));

    return rpts;
  }

  std::vector<point> interior_offset(const std::vector<point>& pts, const double tol) {
    point n(0, 0, 1);
    const rotation r = rotate_from_to(ring_normal(pts), n);
    const rotation r_inv = inverse(r);
    auto res = interior_offset(oriented_polygon(n, apply(r, pts)), tol);

    DBG_ASSERT(res.size() == 1);

    auto rpoly = res.front();
    auto rpts = apply(r_inv, rpoly.vertices());

    correct_winding_order(rpts, ring_normal(pts));

    return rpts;
  }

  std::vector<std::vector<point>>
  interior_offsets(const std::vector<point>& pts,
		   const double tol) {
    point n(0, 0, 1);
    const rotation r = rotate_from_to(ring_normal(pts), n);
    const rotation r_inv = inverse(r);

    cout << "# of points = " << pts.size() << endl;
    cout << "ring normal = " << ring_normal(pts) << endl;
    cout << "tol         = " << tol << endl;
    //vtk_debug_ring(pts);

    auto r_pts = apply(r, pts);

    check_simplicity(r_pts);

    //vtk_debug_ring(r_pts);
    
    auto res = interior_offset(oriented_polygon(n, r_pts), tol);


    vector<vector<point>> result_pts;
    for (auto rpoly : res) {
      auto rpts = apply(r_inv, rpoly.vertices());
      correct_winding_order(rpts, ring_normal(pts));
      result_pts.push_back(rpts);
    }

    return result_pts;
  }
  

  std::vector<polygon_3> exterior_offset(const std::vector<polygon_3>& polys,
					 const double d) {
    DBG_ASSERT(false);
  }

  void set_orientation(Polygon_2& to_offset) {
    if (!(to_offset.is_simple())) {
      DBG_ASSERT(false);
    }
      
    
    if (to_offset.orientation() == CGAL::CLOCKWISE) {
      to_offset.reverse_orientation();
    }

    DBG_ASSERT(to_offset.orientation() == CGAL::COUNTERCLOCKWISE);
    
  }

  std::vector<polygon_3> interior_offsets_flat(const polygon_3& poly,
					       const double d) {

    DBG_ASSERT(angle_eps(poly.normal(), point(0, 0, 1), 0.0, 1.0));
    DBG_ASSERT(poly.vertices().size() >= 3);

    double z_level = poly.vertices().front().z;

    Polygon_2 outer;
    for (auto p : poly.vertices()) {
      outer.push_back(Point(p.x, p.y));
    }

    set_orientation(outer);

    PolygonWithHoles to_offset( outer );

    for (auto h : poly.holes()) {
      Polygon_2 hole;

      for (auto pt : h) {
	hole.push_back( Point(pt.x, pt.y) );
      }

      set_orientation(hole);
      to_offset.add_hole( hole );
    }

    PolygonWithHolesPtrVector offset_poly_with_holes =
      CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(d, to_offset);

    vector<polygon_3> result_polys;

    for (PolygonWithHolesPtr& p : offset_poly_with_holes) {
      PolygonWithHoles& pg = *p;
      Polygon_2 outer = pg.outer_boundary();
      vector<point> outer_pts = ring_for_CGAL_polygon(outer, z_level);

      vector<vector<point>> holes;
      for (auto it = pg.holes_begin(); it != pg.holes_end(); ++it) {
	Polygon_2& cgal_hole = *it;
	holes.push_back(ring_for_CGAL_polygon(cgal_hole, z_level));
      }

      result_polys.push_back(polygon_3(outer_pts, holes));
    }

    return result_polys;
  }
  
  std::vector<polygon_3> interior_offsets(const polygon_3& poly,
					  const double d) {
    point n(0, 0, 1);
    const rotation r = rotate_from_to(poly.normal(), n);
    const rotation r_inv = inverse(r);

    polygon_3 r_poly = apply(r, poly);

    check_simplicity(r_poly);

    auto res = interior_offsets_flat(r_poly, d);

    // NOTE: Need to add z value adjustment?
    vector<polygon_3> result_pts;
    for (auto res_poly : res) {
      auto rpts = apply(r_inv, res_poly);

      result_pts.push_back(rpts);
    }

    return result_pts;
  }

  std::vector<polygon_3> interior_offset(const std::vector<polygon_3>& polys,
					 const double d) {
    vector<polygon_3> inter_offsets;
    for (auto poly : polys) {
      concat(inter_offsets, interior_offsets(poly, d));
    }
    return inter_offsets;
  }
  
}
