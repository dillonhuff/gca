#ifndef GCA_BASIC_STATES_H
#define GCA_BASIC_STATES_H

#include "core/move_instr.h"
#include "core/pass.h"
#include "core/state_names.h"

namespace gca {

  class warning_state : public state {
  protected:
    int num_warns;
    instr* current;
  public:
    warning_state(pass* tp) {
      t = tp;
      num_warns = 0;
    }
    
    int num_warnings() { return num_warns; }
    
    virtual void update(instr& ist) { current = &ist; }
    
    void add_warning(string s) {
      cout << "Warning at position: " << *current << " " << s << endl;
      num_warns++;
    }
  };

  class per_instr_state : public state {
  public:
    virtual void update_G0(move_instr& ist) { update_default(ist); }
    virtual void update_G1(move_instr& ist) { update_default(ist); }
    virtual void update_G91(instr& ist) { update_default(ist); }
    virtual void update_M2(instr& ist) { update_default(ist); }
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
      } else if (ist.is_end_instr()) {
	update_M2(ist);
      } else {
	assert(false);
      }
    }
  };

  class orientation_state : public per_instr_state {
  public:
    orientation current;
    
    orientation_state(pass* tp, orientation default_orient) {
      t = tp;
      current = default_orient;
    }

    virtual void update_G91(instr& ist) {
      current = GCA_RELATIVE;
    }

  };

  class orientation_checker : public per_instr_state {
  public:
    orientation_checker(pass* tp) {
      t = tp;
    }

    virtual void update_G91(instr& ist) {
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_RELATIVE) {
	warning_state* ws = get_state<warning_state>(GCA_WARNING_STATE);
	ws->add_warning("is not needed, relative coordinates are already turned on");
      }
    }
    
  };

  class position_state : public per_instr_state {
  public:
    point before, after, diff;

    position_state(pass* tp, point start) {
      t = tp;
      before = start;
      after = start;
    }

    void update_pos(move_instr& ist) {
      before = after;
      orientation_state* os = get_state<orientation_state>(GCA_ORIENTATION_STATE);
      if (os->current == GCA_RELATIVE) {
      	after = after + ist.pos();
      } else {
      	after = ist.pos();
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
