#include <iostream>

#include "gprog.h"

namespace gca {

  bool gprog::all_abs() const {
    for (int i = 0; i < size(); i++) {
      instr* ist = instrs[i];
      if (ist->is_G() && (ist->v == 0 || ist->v == 1)) {
	if (ist->is_rel()) {
	  return false;
	}
      }
    }
    return true;
  }

  bool gprog::all_rel() const {
    for (int i = 0; i < size(); i++) {
      instr* ist = instrs[i];
      if (ist->is_G() && (ist->v == 0 || ist->v == 1)) {
	if (ist->is_abs()) {
	  return false;
	}
      }
    }
    return true;
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
    vector<point> positions = all_positions_starting_at(point(0, 0, 0));
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
      return false;
    }
    for (int i = 0; i < size(); i++) {
      if (*(other.instrs[i]) != *(instrs[i])) {
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
