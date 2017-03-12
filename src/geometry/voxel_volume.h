#include <boost/container/flat_set.hpp>

#include "geometry/point.h"

namespace gca {

  struct voxel {
    int xi, yi, zi;
  };

  static inline bool operator<(const voxel l, const voxel r) {
    if (l.xi < r.xi) { return true; }
    if (l.xi > r.xi) { return false; }

    if (l.yi < r.yi) { return true; }
    if (l.yi > r.yi) { return false; }

    if (l.zi < r.zi) { return true; }

    return false;
      
  }

  class voxel_volume {

    boost::container::flat_set<voxel> voxels;
  
    
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
      return voxels.find( voxel{x_i, y_i, z_i} ) != voxels.end();
    }


    inline void set_occupied(int x_i, int y_i, int z_i) {
      voxels.insert( voxel{x_i, y_i, z_i} );
    }
    
  };


}
