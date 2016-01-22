#include <iostream>

#include "core/gprog.h"
#include "core/move_instr.h"

namespace gca {

  void gprog::all_positions_starting_at(point start, vector<point>& positions) {
    positions.push_back(start);
    orientation ori = GCA_ABSOLUTE;
    for (int i = 1; i < size() + 1; i++) {
      instr* next = instrs[i-1];
      if (next->is_G()) {
	move_instr mnext = *(static_cast<move_instr*>(next));
	if (next->is_G90()) {
	  ori = GCA_ABSOLUTE;
	} else if (next->is_G91()) {
	  ori = GCA_RELATIVE;
	} else if (ori == GCA_ABSOLUTE) {
	  positions.push_back(mnext.pos());
	} else {
	  positions.push_back(positions[i-1] + mnext.pos());
	}
      } else {
	positions.push_back(positions[i-1]);
      }
    }
  }
  
  point gprog::last_position() {
    if (size() == 0) {
      return point(0, 0, 0);
    }
    vector<point> positions;
    all_positions_starting_at(point(0, 0, 0), positions);
    return positions.back();
  }

  void gprog::print(ostream& s) {
    for (ilist::iterator it = begin();
	 it != end(); ++it) {
      s << **it << endl;
    }
  }

  bool gprog::operator==(const gprog& other) {
    if (other.size() != this->size()) {
      cout << "WRONG SIZE!" << endl;
      return false;
    }
    for (int i = 0; i < size(); i++) {
      if (*(other.instrs[i]) != *(instrs[i])) {
	cout << "Instrs not equal!" << endl;
	cout << *(other.instrs[i]) << endl;
	cout << *(instrs[i]) << endl;
	return false;
      }
    }
    return true;
  }
  

  ostream& operator<<(ostream& stream, gprog& p) {
    p.print(stream);
    return stream;
  }
  
}
