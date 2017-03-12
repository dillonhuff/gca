#include "geometry/point.h"

namespace gca {

  class voxel_volume {
  public:
    voxel_volume(const point origin,
		 const double x_len,
		 const double y_len,
		 const double z_len,
		 const double resolution);

    inline bool is_empty(int x_i, int y_i, int z_i) const {
      return !is_occupied(x_i, y_i, z_i);
    }

    inline bool is_occupied(int x_i, int y_i, int z_i) const {
      return false;
    }


    inline void set_occupied(int x_i, int y_i, int z_i) {
    }
    
  };


}
