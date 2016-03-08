#ifndef GCA_CUT_H
#define GCA_CUT_H

#include <cassert>

#include "core/value.h"
#include "geometry/point.h"
#include "synthesis/machine.h"

namespace gca {

  class cut {
  public:
    point start, end;
    tool_name tool_no;
    value* feedrate;
    value* spindle_speed;
    
  cut(point s, point e) : start(s), end(e), tool_no(NO_TOOL), feedrate(omitted::make()), spindle_speed(omitted::make()) {}
  cut(point s, point e, tool_name t) : start(s), end(e), tool_no(t), feedrate(omitted::make()), spindle_speed(omitted::make()) {}

    virtual inline bool is_safe_move() const { return false; }
    virtual inline bool is_linear_cut() const { return false; }
    virtual inline bool is_circular_arc() const { return false; }
    virtual inline bool is_hole_punch() const { return false; }

    virtual point final_orient() const { assert(false); }
    virtual point initial_orient() const { assert(false); }
    
    virtual bool operator==(const cut& other) const = 0;
    virtual cut* shift(point shift) const = 0;
    virtual cut* scale(double s) const = 0;
    virtual cut* scale_xy(double s) const = 0;
    virtual cut* copy() const = 0;
    virtual void print(ostream& other) const = 0;
  };

  ostream& operator<<(ostream& stream, const cut& c);
  ostream& operator<<(ostream& stream, const vector<cut*>& c);

  bool cmp_cuts(const cut* l, const cut* r);
  bool same_cut_properties(const cut& l, const cut& r);
}


#endif
