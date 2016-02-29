#ifndef GCA_CUT_H
#define GCA_CUT_H

#include <cassert>

#include "core/value.h"
#include "geometry/point.h"

namespace gca {

  class cut {
  public:
    point start, end;
    int tool_no;
    value* feedrate;
    
  cut(point s, point e) : start(s), end(e), feedrate(omitted::make()) {}

    virtual point final_orient() const { assert(false); }
    virtual point initial_orient() const { assert(false); }
    virtual inline bool is_safe_move() const { return false; }
    virtual inline bool is_linear_cut() const { return false; }
    virtual inline bool is_circular_arc() const { return false; }
    virtual inline bool is_hole_punch() const { return false; }

    virtual bool operator==(const cut& other) const = 0;
    virtual cut* shift(point shift) const = 0;
    virtual cut* scale(double s) const = 0;
    virtual cut* copy() const = 0;
    virtual void print(ostream& other) const = 0;
    
  };

  ostream& operator<<(ostream& stream, const cut& c);
  ostream& operator<<(ostream& stream, const vector<cut*>& c);

}


#endif
