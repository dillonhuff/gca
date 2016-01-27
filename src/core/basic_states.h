#ifndef GCA_BASIC_STATES_H
#define GCA_BASIC_STATES_H

#include "core/instrs/all.h"
#include "core/pass.h"
#include "core/state_names.h"

namespace gca {

  class per_instr_state : public state {
  public:
  per_instr_state(pass& tp) : state(tp) {}
    
    virtual void update_G0(move_instr& ist) { update_default(ist); }
    virtual void update_G1(move_instr& ist) { update_default(ist); }
    virtual void update_G91(instr& ist) { update_default(ist); }
    virtual void update_M2(instr& ist) { update_default(ist); }
    virtual void update_M5(instr& ist) { update_default(ist); }
    virtual void update_M30(instr& ist) { update_default(ist); }
    virtual void update_default(instr& ist) {}

    virtual void update(instr& ist) {
      if (ist.is_G0()) {
	move_instr& mist = static_cast<move_instr&>(ist);
	update_G0(mist);
      } else if (ist.is_G1()) {
	move_instr& mist = static_cast<move_instr&>(ist);
	update_G1(mist);
      } else if (ist.is_G91()) {
	update_G91(ist);
      } else if (ist.is_M2()) {
	update_M2(ist);
      } else if (ist.is_M5()) {
	update_M5(ist);
      } else if (ist.is_M30()) {
	update_M30(ist);
      } else {
	cout << "per_instr_state error: Unsupported instruction " << ist << endl;
	assert(false);
      }
    }
  };

  class orientation_state : public per_instr_state {
  public:
    orientation current;
    
  orientation_state(pass& tp, orientation default_orient) :
    per_instr_state(tp), current(default_orient) {}

    virtual void update_G91(instr& ist) {
      current = GCA_RELATIVE;
    }

  };

  class orientation_checker : public per_instr_state {
  public:
  orientation_checker(pass& tp) : per_instr_state(tp) {}

    virtual void update_G91(instr& ist) {
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_RELATIVE) {
	add_warning("is not needed, relative coordinates are already turned on");
      }
    }
    
  };

  class position_state : public per_instr_state {
  public:
    point before, after, diff;

    position_state(pass& tp, point start)
      : per_instr_state(tp), before(start), after(start) {}

    void update_pos(move_instr& ist) {
      before = after;
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_RELATIVE) {
	double xd = ist.x_with_default(0);
	double yd = ist.y_with_default(0);
	double zd = ist.z_with_default(0);
	point p = point(xd, yd, zd);
      	after = after + p;
      } else {
	double x = ist.x_with_default(after.x);
	double y = ist.y_with_default(after.y);
	double z = ist.z_with_default(after.z);
      	after = point(x, y, z);
      }
      diff = after - before;
    }

    virtual void update_G0(move_instr& ist) {
      update_pos(ist);
    }

    virtual void update_G1(move_instr& ist) {
      update_pos(ist);
    }

  };
  
}

#endif
