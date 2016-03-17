#ifndef GCA_LINE_H
#define GCA_LINE_H

#include "geometry/point.h"

namespace gca {
  
  struct line {
    point start, end;
    line(point s, point e) {
      start = s;
      end = e;
    }
  };

}

#endif
