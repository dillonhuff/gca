#ifndef GCA_GPROG_H
#define GCA_GPROG_H

#include <vector>

#include "src/instr.h"

using namespace std;

namespace gca {

  class gprog {
  protected:
    vector<instr*> instrs;

  public:
    void push_back(instr* i) { instrs.push_back(i); }

    unsigned int size() const { return instrs.size(); }

    bool operator==(const gprog& other) {
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
  };

}

#endif
