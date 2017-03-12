#include "catch.hpp"

#include "geometry/voxel_volume.h"

namespace gca {

  TEST_CASE("Initial voxel volume is empty") {
    voxel_volume vol(point(0, 0, 0), 1.0, 1.0, 1.0, 0.1);
    
  }

}
