#include "core/context.h"

namespace gca {
  
  omitted* mk_omitted() {
    omitted* mem = allocate<omitted>();
    return new (mem) omitted();
  }

  var* mk_var(int v) {
    var* vr = allocate<var>();
    return new (vr) var(v);
  }

  lit* mk_lit(double v) {
    lit* l = allocate<lit>();
    return new (l) lit(v);
  }
  
  move_instr* mk_G0(point p) {
    move_instr* mem = allocate<move_instr>();
    return new (mem) move_instr(GCA_G, 0, mk_lit(p.x), mk_lit(p.y), mk_lit(p.z), mk_omitted());
  }
    
  move_instr* mk_G1(double x, double y, double z, double feed_rate) {
    move_instr* mem = allocate<move_instr>();
    lit* v = allocate<lit>();
    lit* frp = new (v) lit(feed_rate);
    return new (mem) move_instr(GCA_G, 1, mk_lit(x), mk_lit(y), mk_lit(z), frp);
  }

  
  move_instr* mk_G1(value* x, value* y, value* z, value* feed_rate) {
    move_instr* mem = allocate<move_instr>();
    return new (mem) move_instr(GCA_G, 1, x, y, z, feed_rate);
  }
    
  move_instr* mk_G1(double x, double y, double z, value* feed_rate) {
    move_instr* mem = allocate<move_instr>();
    return new (mem) move_instr(GCA_G, 1, mk_lit(x), mk_lit(y), mk_lit(z), feed_rate);
  }

  g2_instr* mk_G2(value* x, value* y, value* z,
		  value* i, value* j, value* k,
		  value* feed_rate) {
    g2_instr* mem = allocate<g2_instr>();
    return new (mem) g2_instr(x, y, z, i, j, k, feed_rate);
  }

  g3_instr* mk_G3(value* x, value* y, value* z,
		  value* i, value* j, value* k,
		  value* feed_rate) {
    g3_instr* mem = allocate<g3_instr>();
    return new (mem) g3_instr(x, y, z, i, j, k, feed_rate);
  }
  
  move_instr* mk_G0(double x, double y, double z) {
    move_instr* mem = allocate<move_instr>();
    return new (mem) move_instr(GCA_G, 0, mk_lit(x), mk_lit(y), mk_lit(z), mk_omitted());
  }

  move_instr* mk_G0(value* x, value* y, value* z) {
    move_instr* mem = allocate<move_instr>();
    return new (mem) move_instr(GCA_G, 0, x, y, z, mk_omitted());
  }
      
  instr* mk_instr_cpy(instr* i) {
    instr* new_i;
    if (i->is_move_instr()) {
      if (i->is_G0() || i->is_G1() || i->is_G53()) {
	move_instr* mi = static_cast<move_instr*>(i);
	move_instr* mem = allocate<move_instr>();
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
      instr* mem = allocate<instr>();
      new_i = new (mem) instr(i);
    }
    return new_i;
  }


}
