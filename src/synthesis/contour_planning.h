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
    //    std::vector<surface> visible_from_n;
    //    std::vector<surface> visible_from_minus_n;
    std::vector<surface> rest;
  };

  boost::optional<surface>
  contour_outline(const std::vector<index_t>& inds,
		  const triangular_mesh& part_mesh,
		  const point n);

  std::vector<point>
  possible_contour_normals(const triangular_mesh& part_mesh);

  boost::optional<contour_surface_decomposition>
  compute_contour_surfaces(const triangular_mesh& part_mesh);

  boost::optional<contour_surface_decomposition>
  contour_surface_decomposition_in_dir(const triangular_mesh& part_mesh,
				       const point n);


  std::vector<surface>
  regions_connected_to_both(const surface& to_check,
			    const surface& top,
			    const surface& bottom);

  point pick_jaw_cutout_axis(const contour_surface_decomposition& surfs);

  boost::optional<oriented_polygon>
  simple_outline(const triangular_mesh& m, const point n);
}

#endif
