#include <iostream>

#include "core/gprog.h"

namespace gca {

  void gprog::all_positions_starting_at(point start, vector<point>& positions) {
    positions.push_back(start);
    orientation ori = GCA_ABSOLUTE;
    for (unsigned i = 1; i < size() + 1; i++) {
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

  void gprog::print_nc_output(ostream& s) {
    for (ilist::iterator it = begin();
	 it != end(); ++it) {
      (*it)->print_nc_output(s);
      s << endl;
    }
  }

  bool instrs_eq(instr* l, instr* r) {
    bool res = *l == *r;
    if (!res) {
      cout << "instrs not equal: ";
      cout << *l << "\t" << *r << endl;
    }
    return res;
  }

  bool gprog::operator==(const gprog& other) const {
    if (other.size() != this->size()) {
      cout << "WRONG SIZE!" << endl;
      return false;
    }
    return equal(begin(), end(), other.begin(), instrs_eq);
  }
  

  ostream& operator<<(ostream& stream, gprog& p) {
    p.print(stream);
    return stream;
  }
  
}
