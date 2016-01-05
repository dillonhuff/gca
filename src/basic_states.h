#ifndef GCA_BASIC_STATES_H
#define GCA_BASIC_STATES_H

#include "state.h"
#include "pass.h"

#define GCA_INSTR_STATE 0
#define GCA_WARNING_STATE 1
#define GCA_POSITION_STATE 2
#define GCA_ORIENTATION_STATE 200
#define GCA_ORIENTATION_CHECKER_STATE 2

namespace gca {

  class current_instr_state : public state {
  public:
    int i;
    
    current_instr_state(pass* tp) {
      t = tp;
      i = -1;
    }

    virtual void update() {
      i++;
    }

    instr* get_instr() {
      assert(i >= 0);
      instr* ist = (*(t->p))[i];
      assert(ist != NULL);
      return ist;
    }
  };
  
  class warning_state : public state {
  protected:
    int num_warns;
  public:
    warning_state() {
      num_warns = 0;
    }
    
    int num_warnings() { return num_warns; }
    
    virtual void update() {}
    
    void add_warning(string s) {
      num_warns++;
    }
  };

  class per_instr_state : public state {
  public:
    instr* get_instr() {
      state* s = get_state(GCA_INSTR_STATE);
      current_instr_state* c = static_cast<current_instr_state*>(s);
      return c->get_instr();
    }

    virtual void update_G0(instr* ist) { update_default(ist); }
    virtual void update_G1(instr* ist) { update_default(ist); }
    virtual void update_G91(instr* ist) { update_default(ist); }
    virtual void update_M2(instr* ist) { update_default(ist); }
    virtual void update_default(instr* ist) {}

    virtual void update() {
      instr* ist = get_instr();
      if (ist->is_G0()) {
	update_G0(ist);
      } else if (ist->is_G1()) {
	update_G1(ist);
      } else if (ist->is_G91()) {
	update_G91(ist);
      } else if (ist->is_end_instr()) {
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

    virtual void update_G91(instr* ist) {
      current = GCA_RELATIVE;
    }

  };

  class orientation_checker : public per_instr_state {
  public:
    orientation_checker(pass* tp) {
      t = tp;
    }

    virtual void update_G91(instr* ist) {
      state* s = get_state(GCA_ORIENTATION_STATE);
      orientation_state* os = static_cast<orientation_state*>(s);
      if (os->current == GCA_RELATIVE) {
	state* s = get_state(GCA_WARNING_STATE);
	warning_state* ws = static_cast<warning_state*>(s);
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

    void update_pos(instr* ist) {
      before = after;
      orientation_state* os = static_cast<orientation_state*>(t->get_state(GCA_ORIENTATION_STATE));
      if (os->current == GCA_RELATIVE) {
      	after = after + ist->pos();
      } else {
      	after = ist->pos();
      }
      diff = after - before;      
    }
    
    virtual void update_G0(instr* ist) {
      update_pos(ist);
    }

    virtual void update_G1(instr* ist) {
      update_pos(ist);
    }
    
  };
  
}

#endif
