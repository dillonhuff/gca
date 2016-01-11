#include <iostream>

#include "core/gprog.h"
#include "core/move_instr.h"

namespace gca {

  bool gprog::all_abs() const {
    for (int i = 0; i < size(); i++) {
      instr* ist = instrs[i];
      if (ist->is_G0() || ist->is_G1()) {
	move_instr* mist = static_cast<move_instr*>(ist);
	if (mist->is_rel()) {
	  return false;
	}
      }
    }
    return true;
  }

  bool gprog::all_rel() const {
    for (int i = 0; i < size(); i++) {
      instr* ist = instrs[i];
      if (ist->is_G0() || ist->is_G1()) {
	move_instr* mist = static_cast<move_instr*>(ist);
	if (mist->is_abs()) {
	  return false;
	}
      }
    }
    return true;
  }
  
  void gprog::all_positions_starting_at(point start, vector<point>& positions) {
    positions.push_back(start);
    for (int i = 1; i < size() + 1; i++) {
      instr* next = instrs[i-1];
      if (next->is_G()) {
	move_instr mnext = *(static_cast<move_instr*>(next));
	if (mnext.is_abs()) {
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
