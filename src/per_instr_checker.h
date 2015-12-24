#ifndef GCA_PER_INSTR_CHECKER_H
#define GCA_PER_INSTR_CHECKER_H

#include "checker.h"
#include "gprog.h"
#include "instr_checker.h"

namespace gca {

  class per_instr_checker : public checker {
  protected:
  public:
    virtual bool check_instr(ostream& out, instr* p) const { assert(false); }

  public:
    virtual int check(ostream& out, gprog* p) const {
      int num_warnings = 0;
      for (ilist::iterator it = p->begin();
	   it != p->end(); ++it) {
	if (!check_instr(out, *it)) {
	  num_warnings++;
	}
      }
      return num_warnings;
    }
  };

}

#endif
