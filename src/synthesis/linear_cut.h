#ifndef GCA_LINEAR_CUT_H
#define GCA_LINEAR_CUT_H

#include "core/arena_allocator.h"
#include "geometry/point.h"
#include "synthesis/cut.h"

namespace gca {

  class linear_cut : public cut {
  public:
  linear_cut(point sp, point ep) :
    cut(sp, ep) {}

  linear_cut(point sp, point ep, tool_name t) :
    cut(sp, ep, t) {}

    static linear_cut* make(point sp, point ep) {
      linear_cut* mem = allocate<linear_cut>();
      return new (mem) linear_cut(sp, ep);
    }

    static linear_cut* make(point sp, point ep, tool_name tn) {
      linear_cut* mem = allocate<linear_cut>();
      return new (mem) linear_cut(sp, ep, tn);
    }
    
    point final_orient() const {
      return end - start;
    }
    
    point initial_orient() const {
      return end - start;
    }

    bool operator==(const cut& other) const {
      if (other.is_linear_cut()) {
	bool res = within_eps(start, other.start) && within_eps(end, other.end);
	return res;
      }
      return false;
    }

    cut* shift(point sh) const {
      linear_cut* mem = allocate<linear_cut>();
      linear_cut* c = new (mem) linear_cut(start + sh, end + sh);
      c->tool_no = tool_no;
      return c;
    }

    cut* scale(double s) const {
      linear_cut* mem = allocate<linear_cut>();
      linear_cut* c = new (mem) linear_cut(s*start, s*end);
      c->tool_no = tool_no;
      return c;
    }
    
    inline bool is_linear_cut() const { return true; }

    virtual cut* copy() const {
      linear_cut* l = linear_cut::make(start, end);
      l->tool_no = tool_no;
      l->feedrate = feedrate;
      return l;
    }

    void print(ostream& other) const {
      other << "LINEAR CUT: F" << *feedrate << " " << start << " -> " << end;
    }
    
  };

}

#endif
