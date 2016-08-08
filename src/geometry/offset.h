#ifndef GCA_OFFSET_H
#define GCA_OFFSET_H

#include "geometry/polygon.h"
#include "utils/check.h"

namespace gca {
  
  std::vector<oriented_polygon> exterior_offset(const oriented_polygon& p,
						const double inc);

  std::vector<oriented_polygon> interior_offset(const oriented_polygon& p,
						const double inc);
  
}

#endif
