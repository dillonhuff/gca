#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include "geometry/homogeneous_transformation.h"
#include "geometry/ring.h"
#include "geometry/rotation.h"

namespace gca {

  namespace bg = boost::geometry;

  typedef bg::model::d2::point_xy<double> boost_point_2;
  typedef bg::model::polygon<boost_point_2> boost_poly_2;
  typedef bg::model::multi_polygon<boost_poly_2> boost_multipoly_2;
  typedef bg::model::multi_point<boost_point_2> boost_multipoint_2;

  typedef bg::model::linestring<boost_point_2> boost_linestring_2;
  typedef bg::model::multi_linestring<boost_linestring_2> boost_multilinestring_2;

  class polygon_3 {
  protected:
    std::vector<point> outer_ring;
    std::vector<std::vector<point>> inner_rings;
    
  public:
    polygon_3(const std::vector<point> vertices) :
      outer_ring(vertices) {
      delete_antennas(outer_ring);
      DBG_ASSERT(outer_ring.size() >= 3);
    }

    polygon_3(const std::vector<point> vertices,
		      const std::vector<std::vector<point>> hole_verts);

    point normal() const {
      return ring_normal(outer_ring);
    }

    point vertex(const unsigned i) const { return outer_ring[i]; }

    const std::vector<point>& vertices() const { return outer_ring; }

    const std::vector<std::vector<point>>& holes() const { return inner_rings; }

    std::vector<point> hole(unsigned i) const { return inner_rings[i]; }
    
    unsigned num_vertices() const { return outer_ring.size(); }

    void correct_winding_order(const point n) {
      gca::correct_winding_order(outer_ring, n);
      for (std::vector<point>& ir : inner_rings) {
	gca::correct_winding_order(ir, n);
      }
    }
  };

  typedef polygon_3 labeled_polygon_3;

  labeled_polygon_3 shift(const point p, const labeled_polygon_3& poly);

  void check_simplicity(const labeled_polygon_3& p);

  labeled_polygon_3 apply(const rotation& r, const labeled_polygon_3& p);
  labeled_polygon_3 apply(const homogeneous_transform& r, const labeled_polygon_3& p);

  typedef std::vector<std::vector<labeled_polygon_3>> surface_levels;

  labeled_polygon_3 smooth_buffer(const labeled_polygon_3& p,
				  const double tol);

  labeled_polygon_3 dilate(const labeled_polygon_3& p, const double tol);

  boost_poly_2
  to_boost_poly_2(const labeled_polygon_3& p);

  labeled_polygon_3 project_onto(const plane p,
				 const labeled_polygon_3& poly);

  std::vector<point> project_points(const plane pl,
				    const std::vector<point>& pts);
  
}
