#ifndef GCA_OFFSET_H
#define GCA_OFFSET_H

#include "geometry/polygon.h"

#define CHECK(x) if (!(x)) { std::cout << "CHECK FAILED, EXITING..." << std::endl; exit(-1); }

namespace gca {
  
  std::vector<oriented_polygon> exterior_offset(const oriented_polygon& p,
						const double inc);

  std::vector<oriented_polygon> interior_offset(const oriented_polygon& p,
						const double inc);
  
}

#endif
