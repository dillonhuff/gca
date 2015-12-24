#ifndef GCA_G0_MOVE_CHECKER_H
#define GCA_G0_MOVE_CHECKER_H

#include "checker.h"

namespace gca {

  class g0_move_checker : checker {
  public:
    virtual bool check(ostream& s, gprog* p) const {
      bool all_correct = true;
      point start(0, 0, 0);
      vector<point> positions = p->all_positions_starting_at(start);
      for (int i = 1; i < positions.size(); i++) {
	point diff = positions[i] - positions[i-1];
	instr* ist = (*p)[i-1];
	if ((diff.z != 0 && (diff.x != 0 || diff.y != 0)) &&
	    (ist->is_G() && ist->v == 0)) {
	  all_correct = false;
	  s << "Warning: " << *ist << " moves diagonally" << endl;
	}
      }
      return all_correct;
    }
    
  };

}

#endif
