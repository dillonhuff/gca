#include <iostream>

#include "gprog.h"

namespace gca {

  vector<point> gprog::all_positions() {
    point default_start(0, 0, 0);
    vector<point> positions;
    for (int i = 0; i < size(); i++) {
      instr next = *(instrs[i]);
      assert(next.is_abs());
      if (instrs[i]->is_G()) {
	positions.push_back(next.pos());
      } else {
	if (i == 0) {
	  positions.push_back(default_start);
	} else {
	  positions.push_back(positions[i-1]);
	}
      }
    }
    return positions;
  }

  vector<point> gprog::all_positions_starting_at(point start) {
    vector<point> positions;
    positions.push_back(start);
    for (int i = 1; i < size() + 1; i++) {
      instr next = *(instrs[i-1]);
      if (next.is_G()) {
	if (next.is_abs()) {
	  positions.push_back(next.pos());
	} else {
	  positions.push_back(positions[i-1] + next.pos());
	}
      } else {
	positions.push_back(positions[i-1]);
      }
    }
    return positions;
  }
  
  point gprog::last_position() {
    for (ilist::reverse_iterator rit = instrs.rbegin();
	 rit != instrs.rend(); ++rit) {
      if ((*rit)->is_G()) {
	return (*rit)->pos();
      }
    }
    assert(false);
  }

  void gprog::print(ostream& s) {
    for (ilist::iterator it = begin();
	 it != end(); ++it) {
      s << **it << endl;
    }
  }

  ostream& operator<<(ostream& stream, gprog& p) {
    p.print(stream);
    return stream;
  }
  
}
