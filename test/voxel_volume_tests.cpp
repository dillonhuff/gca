#include "catch.hpp"

#include "geometry/voxel_volume.h"

namespace gca {

  TEST_CASE("Initial voxel volume is empty") {
    voxel_volume vol(point(0, 0, 0), 1.0, 1.0, 1.0, 0.1);

    REQUIRE(vol.is_empty(0, 0, 0));
  }


  TEST_CASE("After setting the voxel it is occupied") {
    voxel_volume vol(point(0, 0, 0), 1.0, 1.0, 1.0, 0.1);

    vol.set_occupied(0, 0, 0);

    REQUIRE(vol.is_occupied(0, 0, 0));
  }
  
}
