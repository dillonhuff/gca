#include <cassert>

#include<boost/shared_ptr.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_2.h>
#include<CGAL/create_offset_polygons_2.h>

#include "geometry/offset.h"

namespace gca {

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2                    Point;
  typedef CGAL::Polygon_2<K>            Polygon_2;
  typedef boost::shared_ptr<Polygon_2> PolygonPtr ;
  typedef std::vector<PolygonPtr> PolygonPtrVector ;

  std::vector<oriented_polygon> exterior_offset(const oriented_polygon& p,
						const double inc) {
    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    PolygonPtrVector inner_offset_polygons =
      CGAL::create_exterior_skeleton_and_offset_polygons_2(inc, out);

    vector<oriented_polygon> results;
    for (auto off_ptr : inner_offset_polygons) {
      Polygon_2 off_p = *off_ptr;
      vector<point> res_pts;
      for (auto it = CGAL::CGAL_SS_i::vertices_begin(off_p);
	   it != CGAL::CGAL_SS_i::vertices_end(off_p); ++it) {
	Point vert = *it;
	res_pts.push_back(point(vert.x(), vert.y(), z_va));
      }
      results.push_back(oriented_polygon(p.normal, res_pts));
    }
    return results;
  }

  std::vector<oriented_polygon> interior_offset(const oriented_polygon& p,
						const double inc) {
    CHECK(p.vertices().size() > 0);
    
    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    PolygonPtrVector inner_offset_polygons =
      CGAL::create_interior_skeleton_and_offset_polygons_2(inc, out);

    vector<oriented_polygon> results;
    for (auto off_ptr : inner_offset_polygons) {
      Polygon_2 off_p = *off_ptr;
      vector<point> res_pts;
      for (auto it = CGAL::CGAL_SS_i::vertices_begin(off_p);
	   it != CGAL::CGAL_SS_i::vertices_end(off_p); ++it) {
	Point vert = *it;
	res_pts.push_back(point(vert.x(), vert.y(), z_va));
      }
      results.push_back(oriented_polygon(p.normal, res_pts));
    }
    return results;
  }
  
}
