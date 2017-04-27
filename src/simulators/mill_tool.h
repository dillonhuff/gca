#ifndef GCA_MILL_TOOL_H
#define GCA_MILL_TOOL_H

#include <cassert>
#include <utility>
#include <vector>

#include "geometry/point.h"
#include "utils/check.h"

namespace gca {

  typedef pair<int, int> column;

  class mill_tool {
  public:
    virtual bool contains(const point p, const point other) const = 0; //, double resolution, int i, int j) const { DBG_ASSERT(false); }

    virtual double x_min(point p) const { DBG_ASSERT(false); }
    virtual double x_max(point p) const { DBG_ASSERT(false); }
    virtual double y_min(point p) const { DBG_ASSERT(false); }
    virtual double y_max(point p) const { DBG_ASSERT(false); }

    virtual ~mill_tool() {
    }
  };

  class cylindrical_bit : public mill_tool {
  public:
    double diameter;
    double radius;
  cylindrical_bit(double d) : diameter(d), radius(diameter/2.0) {}

    virtual inline double x_min(point p) const { return p.x - radius; }
    virtual inline double x_max(point p) const { return p.x + radius; }
    virtual inline double y_min(point p) const { return p.y - radius; }
    virtual inline double y_max(point p) const { return p.y + radius; }
    

    virtual bool contains(const point p, const point origin) const; //, double resolution, int i, int j) const;
    inline bool in_circle(point p, double x, double y) const;
  };

  class ball_nosed : public mill_tool {
  public:
    double diameter;
    double radius;
  ball_nosed(double d) : diameter(d), radius(diameter/2.0) {}

    virtual inline double x_min(point p) const { return p.x - radius; }
    virtual inline double x_max(point p) const { return p.x + radius; }
    virtual inline double y_min(point p) const { return p.y - radius; }
    virtual inline double y_max(point p) const { return p.y + radius; }
    

    virtual bool contains(const point p, const point origin) const; //, double resolution, int i, int j) const;
    inline bool in_circle(point p, double x, double y) const;
  };

}

#endif
