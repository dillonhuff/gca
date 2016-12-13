#ifndef GCA_AXIS_LOCATION_H
#define GCA_AXIS_LOCATION_H

#include "geometry/triangular_mesh.h"
#include "synthesis/clipping_plan.h"

namespace gca {

  struct direction_info {
    point dir;
    bool search_for_freeform_features;
  };

  point part_axis(const triangular_mesh& m);

  point part_axis(const std::vector<index_t>& viable_inds,
		  const triangular_mesh& part);

  std::vector<direction_info>
  select_cut_directions(const triangular_mesh& stock,
			const triangular_mesh& part,
			const fixtures& f,
			const std::vector<tool>& tools);

}

#endif
