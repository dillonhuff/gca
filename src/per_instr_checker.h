#ifndef GCA_PER_INSTR_CHECKER_H
#define GCA_PER_INSTR_CHECKER_H

#include "src/gprog.h"
#include "src/instr_checker.h"

namespace gca {

  class per_instr_checker {
  protected:
    instr_checker* c;
    
  public:
    per_instr_checker() {
    }
    
    per_instr_checker(instr_checker* cp) {
      c = cp;
    }
    
    virtual bool check(ostream& out, gprog* p) {
      bool all_true = true;
      for (ilist::iterator it = p->begin();
	   it != p->end(); ++it) {
	if (!c->check(out, *it)) {
	  all_true = false;
	}
      }
      return all_true;
    }
  };

}

#endif
