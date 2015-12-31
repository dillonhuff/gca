#ifndef GCA_G0_MOVE_CHECKER_H
#define GCA_G0_MOVE_CHECKER_H

#include "checker.h"

namespace gca {

  class g0_move_checker : public checker {
  public:
    virtual int check(ostream& s, gprog* p) const {
      int num_warnings = 0;
      point start(0, 0, 0);
      vector<point> positions;
      p->all_positions_starting_at(start, positions);
      for (int i = 1; i < positions.size(); i++) {
	point diff = positions[i] - positions[i-1];
	instr* ist = (*p)[i-1];
	if ((diff.z != 0 && (diff.x != 0 || diff.y != 0)) &&
	    (ist->is_G() && ist->v == 0)) {
	  num_warnings++;
	  s << "Warning: " << *ist << " moves diagonally" << endl;
	}
      }
      return num_warnings;
    }
    
  };

}

#endif
