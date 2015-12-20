#ifndef GCA_GPROG_H
#define GCA_GPROG_H

#include <vector>

#include "instr.h"

using namespace std;

namespace gca {

  typedef vector<instr*> ilist;

  class gprog {
  protected:
    ilist instrs;

  public:
    ilist::iterator begin() { return instrs.begin(); }
    ilist::iterator end() { return instrs.end(); }
    
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

    void print(ostream& s);

  };

  ostream& operator<<(ostream& stream, gprog& p);

}

#endif
