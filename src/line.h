#ifndef GCA_LINE_H
#define GCA_LINE_H

#include "point.h"

namespace gca {
  
  class line {
  public:
    point start, end;
    line(point s, point e) {
      start = s;
      end = e;
    }
  };

}

#endif
