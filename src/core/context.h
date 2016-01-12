#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "synthesis/cut.h"
#include "core/assign_instr.h"
#include "core/gprog.h"
#include "core/instr.h"
#include "core/move_instr.h"

namespace gca {

  class context {

  protected:
    arena_allocator a;

  public:

    var* mk_var(int v) {
      var* vr = a.allocate<var>();
      return new (vr) var();
    }

    lit* mk_lit(double v) {
      lit* l = a.allocate<lit>();
      return new (l) lit(v);
    }

    assign_instr* mk_assign(var* v, value* e) {
      assign_instr* mem = a.allocate<assign_instr>();
      return new (mem) assign_instr(v, e);
    }
    
    gprog* mk_gprog() {
      gprog* mem = a.allocate<gprog>();
      return new (mem) gprog();
    }

    instr* mk_minstr(int val) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_M, val);
    }

    instr* mk_tinstr(int val) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_T, val);
    }

    instr* mk_sinstr(int val) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_S, val);
    }

    // TODO: Actually make use of the control string
    instr* mk_finstr(int val, string s) {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_F, val);
    }
    
    instr* mk_G91() {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 91);
    }

    instr* mk_G90() {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 90);
    }
    
    move_instr* mk_G0(double x, double y, double z, orientation orient=GCA_ABSOLUTE) {
      move_instr* mem = a.allocate<move_instr>();
      lit* v = a.allocate<lit>();
      lit* frp = new (v) lit(-1);            
      return new (mem) move_instr(GCA_G, 0, point(x, y, z), frp, orient);
    }

    move_instr* mk_G53(point p, orientation orient=GCA_ABSOLUTE) {
      move_instr* mem = a.allocate<move_instr>();
      lit* v = a.allocate<lit>();
      lit* frp = new (v) lit(-1);                  
      return new (mem) move_instr(GCA_G, 53, p, frp, orient);
    }
    
    move_instr* mk_G0(point p, orientation orient=GCA_ABSOLUTE) {
      move_instr* mem = a.allocate<move_instr>();
      lit* v = a.allocate<lit>();
      lit* frp = new (v) lit(-1);      
      return new (mem) move_instr(GCA_G, 0, p, frp, orient);
    }
    
    move_instr* mk_G1(double x, double y, double z, double feed_rate=1.0, orientation orient=GCA_ABSOLUTE) {
      move_instr* mem = a.allocate<move_instr>();
      lit* v = a.allocate<lit>();
      lit* frp = new (v) lit(feed_rate);
      return new (mem) move_instr(GCA_G, 1, point(x, y, z), frp, orient);
    }

    move_instr* mk_G1(double x, double y, double z, value* feed_rate, orientation orient=GCA_ABSOLUTE) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 1, point(x, y, z), feed_rate, orient);
    }
    
    cut* mk_cut(point start, point end) {
      cut* mem = a.allocate<cut>();
      return new (mem) cut(start, end);
    }

    instr* mk_instr_cpy(instr* i) {
      instr* new_i;
      if (i->is_move_instr()) {
	move_instr* mi = static_cast<move_instr*>(i);
	move_instr* mem = a.allocate<move_instr>();
	move_instr* new_i_m = new (mem) move_instr(mi);
	new_i = static_cast<instr*>(new_i_m);
      } else {
	instr* mem = a.allocate<instr>();
	new_i = new (mem) instr(i);
      }
      return new_i;
    }

  };

}

#endif
