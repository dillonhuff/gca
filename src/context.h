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
      gprog* mem = static_cast<gprog*>(a.allocate(sizeof(gprog)));
      return new (mem) gprog();
    }

    instr* mk_minstr(int val) {
      instr* mem = static_cast<instr*>(a.allocate(sizeof(instr)));
      return new (mem) instr(0, val);
    }
    
  };

}

#endif
