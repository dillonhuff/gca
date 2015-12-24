#ifndef GCA_MULTI_CHECKER_H
#define GCA_MULTI_CHECKER_H

#include "checker.h"

namespace gca {

  class multi_checker : public checker {
  protected:
    vector<checker*> checkers;
    
  public:
    multi_checker(vector<checker*>& cs) {
      checkers = cs;
    }
    
    virtual int check(ostream& s, gprog* p) const {
      int num_warnings = 0;
      for (int i = 0; i < checkers.size(); i++) {
	num_warnings += checkers[i]->check(s, p);
      }
      return num_warnings;
    }
  };

}

#endif
