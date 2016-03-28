#ifndef GCA_SAFE_MOVE_H
#define GCA_SAFE_MOVE_H

#include "synthesis/cut.h"

namespace gca {

  class safe_move : public cut {
  public:
  safe_move(point sp, point ep) :
    cut(sp, ep) {}

  safe_move(point sp, point ep, tool_name t) :
    cut(sp, ep, t) {}
    
    static safe_move* make(point sp, point ep) {
      safe_move* mem = allocate<safe_move>();
      return new (mem) safe_move(sp, ep);
    }

    static safe_move* make(point sp, point ep, tool_name t) {
      safe_move* mem = allocate<safe_move>();
      return new (mem) safe_move(sp, ep, t);
    }
    
    point final_orient() const {
      return end - start;
    }
    
    point initial_orient() const {
      return end - start;
    }

    bool operator==(const cut& other) const {
      if (!same_cut_properties(*this, other)) {
	return false;
      }
      if (other.is_safe_move()) {
	bool res = within_eps(start, other.get_start()) && within_eps(end, other.get_end());
	return res;
      }
      return false;
    }

    cut* shift(point sh) const {
      safe_move* mem = allocate<safe_move>();
      safe_move* m = new (mem) safe_move(start + sh, end + sh);
      m->tool_no = tool_no;
      return m;
    }

    cut* scale(double s) const {
      safe_move* mem = allocate<safe_move>();
      safe_move* m = new (mem) safe_move(s*start, s*end);
      m->tool_no = tool_no;
      return m;
    }

    cut* scale_xy(double s) const {
      safe_move* m = static_cast<safe_move*>(copy());
      m->start = point(s*start.x, s*start.y, start.z);
      m->end = point(s*end.x, s*end.y, end.z);
      return m;
    }

    inline bool is_safe_move() const { return true; }

    virtual cut* copy() const {
      safe_move* c = safe_move::make(start, end);
      c->tool_no = tool_no;
      c->set_feedrate(settings.feedrate);
      c->set_spindle_speed(settings.spindle_speed);
      return c;
    }

    void print(ostream& other) const {
      other << "SAFE MOVE: " << tool_no << " ";
      if (!get_feedrate()->is_omitted()) {
	other << "F" << *get_feedrate() << " ";
      } else {
	other << "<F omitted> ";
      }
      if (!get_spindle_speed()->is_omitted()) {
	other << "S" << *get_spindle_speed() << " ";
      } else {
	other << "<S omitted> ";
      }
      other << start << " -> " << end;
    }
    
  };
  
}

#endif
