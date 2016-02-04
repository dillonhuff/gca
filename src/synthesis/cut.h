#ifndef GCA_CUT_H
#define GCA_CUT_H

#include "geometry/point.h"

namespace gca {

  class cut {
  public:
    point start, end;
  cut(point s, point e) : start(s), end(e) {}

    virtual point final_orient() const { assert(false); }
    virtual point initial_orient() const { assert(false); }
    virtual bool operator==(const cut& other) const = 0;
    virtual cut* shift(point shift) const = 0;
    virtual inline bool is_linear_cut() const { return false; }
    virtual inline bool is_circular_arc() const { return false; }
    virtual inline bool is_hole_punch() const { return true; }
    
  };
    

}


#endif
