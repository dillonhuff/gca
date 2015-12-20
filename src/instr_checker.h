#ifndef GCA_INSTR_CHECKER_H
#define GCA_INSTR_CHECKER_H

#include "src/instr.h"

namespace gca {

  class instr_checker {
  public:
    virtual bool check(ostream& out, instr* p) {
      return true;
    }
    
  };
  
}

#endif
