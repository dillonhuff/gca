#ifndef GCA_CUT_H
#define GCA_CUT_H

#include "point.h"

namespace gca {

  class cut {
  public:
    point start, end;

  cut(point sp, point ep) :
    start(sp), end(ep) {}

    bool operator==(const cut& other) const {
      bool res = within_eps(start, other.start) && within_eps(end, other.end);
      return res;
    }
    
  };

}

#endif
