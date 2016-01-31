#ifndef GCA_LINEAR_CUT_H
#define GCA_LINEAR_CUT_H

#include "geometry/point.h"

namespace gca {

  class linear_cut {
  public:
    point start, end;

  linear_cut(point sp, point ep) :
    start(sp), end(ep) {}

    bool operator==(const linear_cut& other) const {
      bool res = within_eps(start, other.start) && within_eps(end, other.end);
      return res;
    }
    
  };

}

#endif
