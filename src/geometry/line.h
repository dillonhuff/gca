#ifndef GCA_LINE_H
#define GCA_LINE_H

#include "geometry/point.h"

namespace gca {
  
  struct line {
    point start, end;
    line(point s, point e) : start(s), end(e) {}

    point value(double t) const
    { return (1.0 - t)*start + t*end; }

    line shift(point t) const
    { return line(start + t, end + t); }

    line scale(double t) const
    { return line(t*start, t*end); }

    line scale_xy(double t) const {
      point se = start;
      se.x = t*se.x;
      se.y = t*se.y;
      point ee = end;
      ee.x = t*ee.x;
      ee.y = t*ee.y;
      return line(se, ee);
    }

    
  };

  ostream& operator<<(ostream& out, line l);

}

#endif
