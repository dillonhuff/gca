#ifndef GCA_G0_MOVE_CHECKER_H
#define GCA_G0_MOVE_CHECKER_H

#include "basic_states.h"
#include "pass.h"

#define GCA_MOVE_CHECKER_STATE 201

namespace gca {

  class move_checker_state : public per_instr_state {
  public:
    move_checker_state(pass* tp) {
      t = tp;
    }
    
    virtual void update_G0(instr* ist) {
      cout << "Update G0 in move checker" << endl;
      position_state* ps = static_cast<position_state*>(t->get_state(GCA_POSITION_STATE));
      point diff = ps->diff;
      cout << "Diff = " << diff << endl;
      if (diff.z != 0 && (diff.x != 0 || diff.y != 0)) {
	state* s = get_state(GCA_WARNING_STATE);
	warning_state* ws = static_cast<warning_state*>(s);
	ws->add_warning("moves diagonally");
      }
    }
  };

  class g0_move_checker : public pass {
  protected:
    current_instr_state cis;
    position_state ps;
    warning_state s;
    move_checker_state ms;
    orientation_state orient_s;
    
  public:
  g0_move_checker() :
    cis(this), ps(this, point(0, 0, 0)), ms(this), orient_s(this) {
      states[GCA_INSTR_STATE] = &cis;
      states[GCA_WARNING_STATE] = &s;
      states[GCA_POSITION_STATE] = &ps;
      states[GCA_MOVE_CHECKER_STATE] = &ms;
      states[GCA_ORIENTATION_STATE] = &orient_s;
    }
    
    /* virtual int check(ostream& s, gprog* p) const { */
    /*   int num_warnings = 0; */
    /*   point start(0, 0, 0); */
    /*   vector<point> positions; */
    /*   p->all_positions_starting_at(start, positions); */
    /*   for (int i = 1; i < positions.size(); i++) { */
    /* 	point diff = positions[i] - positions[i-1]; */
    /* 	instr* ist = (*p)[i-1]; */
    /* 	if (ist->is_G0() && (diff.z != 0 && (diff.x != 0 || diff.y != 0))) { */
    /* 	  num_warnings++; */
    /* 	  s << "Warning: " << *ist << " moves diagonally" << endl; */
    /* 	} */
    /*   } */
    /*   return num_warnings; */
    /* } */
    
  };

}

#endif
