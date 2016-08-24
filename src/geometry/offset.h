#ifndef GCA_OFFSET_H
#define GCA_OFFSET_H

#include <boost/shared_ptr.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

#include "geometry/polygon.h"
#include "utils/check.h"

namespace gca {

  typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
  typedef K::Point_2                    Point;
  typedef CGAL::Polygon_2<K>            Polygon_2;

  Polygon_2
  CGAL_polygon_for_oriented_polygon(const oriented_polygon& p);
  
  std::vector<oriented_polygon> exterior_offset(const oriented_polygon& p,
						const double inc);

  std::vector<oriented_polygon> interior_offset(const oriented_polygon& p,
						const double inc);
  
}

#endif
