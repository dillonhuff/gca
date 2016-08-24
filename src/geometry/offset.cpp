#include <cassert>

#include "geometry/offset.h"

#include<CGAL/create_offset_polygons_2.h>

#include "geometry/vtk_debug.h"

namespace gca {

  typedef boost::shared_ptr<Polygon_2> PolygonPtr ;
  typedef std::vector<PolygonPtr> PolygonPtrVector ;

  Polygon_2
  CGAL_polygon_for_oriented_polygon(const oriented_polygon& p) {
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    return out;
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
    DBG_ASSERT(signed_area(p) < 0);
    
    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    DBG_ASSERT(out.is_simple());
    DBG_ASSERT(out.orientation() == CGAL::COUNTERCLOCKWISE);

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
    
    PolygonPtrVector inner_offset_polygons =
      CGAL::create_interior_skeleton_and_offset_polygons_2(inc, out);

    vector<oriented_polygon> results;
    for (auto off_ptr : inner_offset_polygons) {
      Polygon_2 off_p = *off_ptr;
      auto op = oriented_polygon_for_CGAL_polygon(off_p, z_va, p.normal);
      results.push_back(op);
    }
    return results;
  }
  
}
