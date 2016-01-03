#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "cut.h"
#include "gprog.h"
#include "instr.h"
#include "toolpath.h"

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

    instr* mk_G91() {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 91);
    }
    
    instr* mk_G0(double x, double y, double z, orientation orient=GCA_ABSOLUTE) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 0, point(x, y, z), orient);
    }

    instr* mk_G0(point p, orientation orient=GCA_ABSOLUTE) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 0, p, orient);
    }
    
    instr* mk_G1(double x, double y, double z, double feed_rate=1.0, orientation orient=GCA_ABSOLUTE) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 1, point(x, y, z), feed_rate, orient);
    }

    cut* mk_cut(point start, point end) {
      cut* mem = a.allocate<cut>();
      return new (mem) cut(start, end);
    }

    instr* mk_inverted_orientation(instr* i) {
      instr* mem = a.allocate<instr>();
      instr* new_i = new (mem) instr(i);
      new_i->swap_orientation();
      return new_i;
    }

    instr* mk_instr_cpy(instr* i) {
      instr* mem = a.allocate<instr>();
      instr* new_i = new (mem) instr(i);
      return new_i;
    }

    toolpath* mk_toolpath() {
      toolpath* t = a.allocate<toolpath>();
      return t;
    }

  };

}

#endif
