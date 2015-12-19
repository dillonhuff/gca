#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "src/arena_allocator.h"
#include "src/gprog.h"
#include "src/instr.h"

namespace gca {

  class context {

  protected:
    arena_allocator a;

  public:
    
    gprog* mk_gprog() {
      gprog* mem = a.allocate<gprog>();
      return new (mem) gprog();
    }

    instr* mk_minstr(int val) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_M, val);
    }

    instr* mk_G0(double x, double y, double z) {
      instr* mem = a.allocate<instr>();
      instr* i = new (mem) instr(GCA_G, 0);
      return i;
    }
    
  };

}

#endif
