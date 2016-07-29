#ifndef GCA_CONTOUR_PLANNING_H
#define GCA_CONTOUR_PLANNING_H

#include <boost/optional.hpp>

#include "geometry/surface.h"

namespace gca {

  struct contour_surface_decomposition {
    point n;
    surface outline;
    surface top;
    surface bottom;
    std::vector<surface> visible_from_n;
    std::vector<surface> visible_from_minus_n;
    std::vector<surface> rest;
  };

  std::vector<point>
  possible_contour_normals(const triangular_mesh& part_mesh);

  boost::optional<contour_surface_decomposition>
  compute_contour_surfaces(const triangular_mesh& part_mesh);

  boost::optional<contour_surface_decomposition>
  contour_surface_decomposition_in_dir(const triangular_mesh& part_mesh,
				       const point n);

  
}

#endif
