#ifndef GCA_BASIC_STATES_H
#define GCA_BASIC_STATES_H

#include "state.h"
#include "pass.h"

#define GCA_INSTR_STATE 0
#define GCA_WARNING_STATE 1
#define GCA_ORIENTATION_STATE 20
#define GCA_ORIENTATION_CHECKER_STATE 2

namespace gca {

  class current_instr_state : public state {
  protected:
    int i;
  public:
    current_instr_state(pass* tp) {
      t = tp;
      i = -1;
    }

    virtual void update() {
      i++;
    }

    instr* get_instr() {
      assert(i >= 0);
      cout << "Getting instruction # " << i << endl;
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

  class gca_position_state : public state {
  protected:
    point before, after, diff;
  public:
    gca_position_state(pass* tp) {
      t = tp;
    }
    
    virtual void update() {
      state* s = get_state(GCA_INSTR_STATE);
      current_instr_state* c = static_cast<current_instr_state*>(s);
      //      s = get_state(GCA_ORIENTATION_STATE);
    }
  };

  class orientation_state : public state {
  public:
    orientation current;
    
    orientation_state(pass* tp) {
      t = tp;
      current = GCA_ABSOLUTE;
    }

    virtual void update() {
      state* s = get_state(GCA_INSTR_STATE);
      current_instr_state* c = static_cast<current_instr_state*>(s);
      instr* ist = c->get_instr();
      if (ist->is_G91()) {
	if (current != GCA_RELATIVE) {
	  current = GCA_RELATIVE;
	}
      }
    }

  };

}

#endif
