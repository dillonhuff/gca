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

  oriented_polygon exterior_offset(const oriented_polygon& p,
				   const double inc) {
    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    PolygonPtrVector inner_offset_polygons =
      CGAL::create_exterior_skeleton_and_offset_polygons_2(inc, out);

    assert(inner_offset_polygons.size() == 1);

    Polygon_2 off_p = *(inner_offset_polygons.front());

    vector<point> res_pts;
    for (auto it = CGAL::CGAL_SS_i::vertices_begin(off_p);
	 it != CGAL::CGAL_SS_i::vertices_end(off_p); ++it) {
      Point vert = *it;
      res_pts.push_back(point(vert.x(), vert.y(), z_va));
    }
    return oriented_polygon(p.normal, res_pts);
  }

  oriented_polygon interior_offset(const oriented_polygon& p,
				   const double inc) {
    cout << "Start interior offset" << endl;
    assert(p.vertices().size() > 0);
    cout << "Done with vertices check" << endl;
    
    double z_va = p.vertices().front().z;
    Polygon_2 out;
    for (auto p : p.vertices()) {
      out.push_back(Point(p.x, p.y));
    }

    cout << "Interior offset" << endl;

    PolygonPtrVector inner_offset_polygons =
      CGAL::create_interior_skeleton_and_offset_polygons_2(inc, out);

    cout << "Got interior offset" << endl;

    assert(inner_offset_polygons.size() == 1);

    cout << "# of offset polygons = " << inner_offset_polygons.size() << endl;

    Polygon_2 off_p = *(inner_offset_polygons.front());

    cout << "Dereferenced off_p" << endl;

    vector<point> res_pts;
    for (auto it = CGAL::CGAL_SS_i::vertices_begin(off_p);
	 it != CGAL::CGAL_SS_i::vertices_end(off_p); ++it) {
      Point vert = *it;
      cout << "got vert" << endl;
      res_pts.push_back(point(vert.x(), vert.y(), z_va));
    }

    cout << "Done with offset polygon" << endl;
    return oriented_polygon(p.normal, res_pts);
  }
  
}
