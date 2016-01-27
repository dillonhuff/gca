#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "synthesis/cut.h"
#include "core/gprog.h"
#include "core/instrs/all.h"

namespace gca {

  class context {

  protected:
    arena_allocator a;

  public:

    var* mk_var(int v) {
      var* vr = allocate<var>(); //a.allocate<var>();//static_cast<var*>(alloc(sizeof(var)));
      return new (vr) var(v);
    }

    lit* mk_lit(double v) {
      lit* l = allocate<lit>();
      return new (l) lit(v);
    }

    omitted* mk_omitted() {
      omitted* mem = a.allocate<omitted>();
      return new (mem) omitted();
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

    f_instr* mk_finstr(int val, string s) {
      f_instr* mem = a.allocate<f_instr>();
      return new (mem) f_instr(val, s);
    }

    comment* mk_comment(char ld, char rd, string t) {
      comment* mem = a.allocate<comment>();
      return new (mem) comment(ld, rd, t);
    }
    
    instr* mk_G91() {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 91);
    }

    instr* mk_G90() {
      instr* mem = a.allocate<instr>();
      return new (mem) instr(GCA_G, 90);
    }
    
    move_instr* mk_G0(double x, double y, double z) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 0, mk_lit(x), mk_lit(y), mk_lit(z), mk_omitted());
    }

    move_instr* mk_G0(value* x, value* y, value* z) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 0, x, y, z, mk_omitted());
    }
    
    move_instr* mk_G53(value* x, value* y, value* z) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 53, x, y, z, mk_omitted());
    }
    
    move_instr* mk_G0(point p) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 0, mk_lit(p.x), mk_lit(p.y), mk_lit(p.z), mk_omitted());
    }
    
    move_instr* mk_G1(double x, double y, double z, double feed_rate=1.0) {
      move_instr* mem = a.allocate<move_instr>();
      lit* v = a.allocate<lit>();
      lit* frp = new (v) lit(feed_rate);
      return new (mem) move_instr(GCA_G, 1, mk_lit(x), mk_lit(y), mk_lit(z), frp);
    }

    move_instr* mk_G1(value* x, value* y, value* z, value* feed_rate) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 1, x, y, z, feed_rate);
    }
    
    move_instr* mk_G1(double x, double y, double z, value* feed_rate) {
      move_instr* mem = a.allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 1, mk_lit(x), mk_lit(y), mk_lit(z), feed_rate);
    }

    g2_instr* mk_G2(value* x, value* y, value* z,
		      value* i, value* j, value* k,
		      value* feed_rate) {
      g2_instr* mem = a.allocate<g2_instr>();
      return new (mem) g2_instr(x, y, z, i, j, k, feed_rate);
    }

    g3_instr* mk_G3(value* x, value* y, value* z,
		      value* i, value* j, value* k,
		      value* feed_rate) {
      g3_instr* mem = a.allocate<g3_instr>();
      return new (mem) g3_instr(x, y, z, i, j, k, feed_rate);
    }
    
    cut* mk_cut(point start, point end) {
      cut* mem = a.allocate<cut>();
      return new (mem) cut(start, end);
    }

    instr* mk_instr_cpy(instr* i) {
      instr* new_i;
      if (i->is_move_instr()) {
	if (i->is_G0() || i->is_G1() || i->is_G53()) {
	  move_instr* mi = static_cast<move_instr*>(i);
	  move_instr* mem = a.allocate<move_instr>();
	  move_instr* new_i_m = new (mem) move_instr(mi);
	  new_i = static_cast<instr*>(new_i_m);
	} else if (i->is_g2_instr()) {
	  g2_instr* gi = static_cast<g2_instr*>(i);
	  new_i = mk_G2(gi->get_x(), gi->get_y(), gi->get_z(),
		       gi->i, gi->j, gi->k,
		       gi->feed_rate);
	} else if (i->is_g3_instr()) {
	  g3_instr* gi = static_cast<g3_instr*>(i);
	  new_i = mk_G3(gi->get_x(), gi->get_y(), gi->get_z(),
		       gi->i, gi->j, gi->k,
		       gi->feed_rate);
	} else {
	  cout << "ERROR: DO NOT SUPPORT " << *i << endl;
	  assert(false);
	}
      } else {
	instr* mem = a.allocate<instr>();
	new_i = new (mem) instr(i);
      }
      return new_i;
    }

  };

}

#endif
