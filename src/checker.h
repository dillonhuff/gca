#ifndef GCA_CHECKER_H
#define GCA_CHECKER_H

#include "gprog.h"

using namespace std;

namespace gca {

  class checker {
  public:
    virtual int check(ostream& s, gprog* i) const {
      assert(false);
    }
  };
  
}
#endif
