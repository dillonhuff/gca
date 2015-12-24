#ifndef GCA_PER_INSTR_CHECKER_H
#define GCA_PER_INSTR_CHECKER_H

#include "gprog.h"
#include "instr_checker.h"

namespace gca {

  class per_instr_checker {
  protected:
  public:
    virtual bool check_instr(ostream& out, instr* p) const { assert(false); }

  public:
    virtual bool check(ostream& out, gprog* p) {
      bool all_true = true;
      for (ilist::iterator it = p->begin();
	   it != p->end(); ++it) {
	if (!check_instr(out, *it)) {
	  all_true = false;
	}
      }
      return all_true;
    }
  };

}

#endif
