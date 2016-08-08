#ifndef GCA_OFFSET_H
#define GCA_OFFSET_H

#include "geometry/polygon.h"

namespace gca {

  oriented_polygon exterior_offset(const oriented_polygon& p,
				   const double inc);

  oriented_polygon interior_offset(const oriented_polygon& p,
				   const double inc);
  
}

#endif
