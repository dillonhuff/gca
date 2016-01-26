#ifndef GCA_CALLBACK_H
#define GCA_CALLBACK_H

#include "core/gprog.h"

namespace gca {
  
  template<typename T>
    class callback {
    virtual T call(gprog* p, int i, instr* is) = 0;
  };
 
}


#endif
