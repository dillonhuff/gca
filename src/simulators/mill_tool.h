#ifndef GCA_MILL_TOOL_H
#define GCA_MILL_TOOL_H

#include <cassert>
#include <utility>
#include <vector>

#include "geometry/point.h"

namespace gca {

  typedef pair<int, int> column;

  class mill_tool {
  public:
    virtual void columns_to_update(point p,
				   double resolution,
				   vector<column> to_update) const { assert(false); }

    virtual bool contains(point p, double resolution, int i, int j) const { assert(false); }
  };

  class cylindrical_bit : public mill_tool {
  public:
    double diameter;
  cylindrical_bit(double d) : diameter(d) {}

    virtual void columns_to_update(point p,
				   double resolution,
				   vector<column> to_update) const;

    virtual bool contains(point p, double resolution, int i, int j) const;
    inline bool in_circle(point p, double x, double y) const;
  };
}

#endif
