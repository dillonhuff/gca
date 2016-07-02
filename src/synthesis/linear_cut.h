#ifndef GCA_LINEAR_CUT_H
#define GCA_LINEAR_CUT_H

#include "utils/arena_allocator.h"
#include "geometry/point.h"
#include "gcode/cut.h"

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
      return get_end() - get_start();
    }
    
    point initial_orient() const {
      return get_end() - get_start();
    }

    bool operator==(const cut& other) const {
      if (!same_cut_properties(*this, other)) {
	return false;
      }
      if (other.is_linear_cut()) {
	bool res = within_eps(get_start(), other.get_start()) && within_eps(get_end(), other.get_end());
	return res;
      }
      return false;
    }

    inline bool is_linear_cut() const { return true; }

    virtual cut* copy() const {
      linear_cut* l = linear_cut::make(get_start(), get_end());
      l->tool_no = tool_no;
      l->set_feedrate(settings.feedrate);
      l->set_spindle_speed(settings.spindle_speed);
      l->settings = settings;
      return l;
    }

    void print(ostream& other) const;    
  };

}

#endif
