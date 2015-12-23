#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "cut.h"
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

    instr* mk_G0(double x, double y, double z, orientation orient=GCA_ABSOLUTE) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 0, x, y, z, orient);
    }

    instr* mk_G1(double x, double y, double z, double feed_rate=1.0, orientation orient=GCA_ABSOLUTE) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 1, x, y, z, feed_rate, orient);
    }

    cut* mk_cut(point start, point end) {
      cut* mem = a.allocate<cut>();
      return new (mem) cut(start, end);
    }

  };

}

#endif
