#include "geometry/surface.h"
#include "synthesis/tool.h"

namespace gca {

  struct freeform_surface {
    surface s;
    std::vector<tool> tools;
  };

  std::vector<freeform_surface>
  freeform_surface_regions(const triangular_mesh& part,
			   const point n,
			   const std::vector<tool>& tools);

}
