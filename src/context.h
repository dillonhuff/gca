#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "gprog.h"
#include "instr.h"

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
      return new (mem) instr(GCA_G, 0, x, y, z);
    }

    instr* mk_G1(double x, double y, double z) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 1, x, y, z);
    }
    
  };

}

#endif
