#ifndef GCA_G0_MOVE_CHECKER_H
#define GCA_G0_MOVE_CHECKER_H

#include "basic_states.h"
#include "pass.h"

namespace gca {

  class g0_move_checker : public pass {
  protected:
    current_instr_state cis;
    warning_state s;
  public:
  g0_move_checker() :
    cis(this) {
      current_instr_state cisp(this);
      cis = cisp;
      states[GCA_CURRENT_INSTR_STATE] = &cis;
      warning_state sp;
      s = sp;
      states[GCA_WARNING_STATE] = &s;
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
