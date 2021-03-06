#ifndef GCA_SAFE_MOVE_H
#define GCA_SAFE_MOVE_H

#include "gcode/cut.h"

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
      return get_end() - get_start();
    }
    
    point initial_orient() const {
      return get_end() - get_start();
    }

    bool operator==(const cut& other) const {
      if (!same_cut_properties(*this, other)) {
	return false;
      }
      if (other.is_safe_move()) {
	bool res = within_eps(get_start(), other.get_start()) && within_eps(get_end(), other.get_end());
	return res;
      }
      return false;
    }

    inline bool is_safe_move() const { return true; }

    virtual cut* copy() const {
      safe_move* c = safe_move::make(get_start(), get_end());
      c->tool_no = tool_no;
      c->set_feedrate(settings.feedrate);
      c->set_spindle_speed(settings.spindle_speed);
      c->settings = settings;
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
      other << get_start() << " -> " << get_end();
    }
    
  };
  
}

#endif
