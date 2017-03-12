#include "geometry/point.h"

namespace gca {

  class voxel_volume {
  public:
    voxel_volume(const point origin,
		 const double x_len,
		 const double y_len,
		 const double z_len,
		 const double resolution);
  };


}
