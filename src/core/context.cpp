#include "core/context.h"

namespace gca {

  instr* mk_instr_cpy(instr* i) {
    instr* new_i;
    if (i->is_move_instr()) {
      if (i->is_G0()) {
	g0_instr* mi = static_cast<g0_instr*>(i);
	g0_instr* mem = allocate<g0_instr>();
	g0_instr* new_i_m = new (mem) g0_instr(mi);
	new_i = static_cast<instr*>(new_i_m);	
      } else if (i->is_G1()) {
	g1_instr* mi = static_cast<g1_instr*>(i);
	g1_instr* mem = allocate<g1_instr>();
	g1_instr* new_i_m = new (mem) g1_instr(mi);
	new_i = static_cast<instr*>(new_i_m);		
      } else if (i->is_G53()) {
	g53_instr* mi = static_cast<g53_instr*>(i);
	g53_instr* mem = allocate<g53_instr>();
	g53_instr* new_i_m = new (mem) g53_instr(mi);
	new_i = static_cast<instr*>(new_i_m);
      } else if (i->is_g2_instr()) {
	g2_instr* gi = static_cast<g2_instr*>(i);
	new_i = g2_instr::make(gi->get_x(), gi->get_y(), gi->get_z(),
		      gi->i, gi->j, gi->k,
		      gi->feed_rate);
      } else if (i->is_g3_instr()) {
	g3_instr* gi = static_cast<g3_instr*>(i);
	new_i = g3_instr::make(gi->get_x(), gi->get_y(), gi->get_z(),
		      gi->i, gi->j, gi->k,
		      gi->feed_rate);
      } else {
	cout << "ERROR: DO NOT SUPPORT " << *i << endl;
	assert(false);
      }
    } else {
      new_i = i->copy();
    }
    return new_i;
  }


  assign_instr* mk_assign(var* v, value* e) {
    assign_instr* mem = allocate<assign_instr>();
    return new (mem) assign_instr(v, e);
  }
    
  gprog* mk_gprog() {
    gprog* mem = allocate<gprog>();
    return new (mem) gprog();
  }

  m2_instr* mk_m2_instr() {
    return new (allocate<m2_instr>()) m2_instr();
  }

  m30_instr* mk_m30_instr() {
    return new (allocate<m30_instr>()) m30_instr();
  }

  m5_instr* mk_m5_instr() {
    return new (allocate<m5_instr>()) m5_instr();
  }

  m3_instr* mk_m3_instr() {
    return new (allocate<m3_instr>()) m3_instr();
  }
  
  t_instr* mk_t_instr(int val) {
    instr* mem = allocate<t_instr>();
    return new (mem) t_instr(val);
  }

  s_instr* mk_s_instr(int val) {
    return new (allocate<s_instr>()) s_instr(val);
  }

  f_instr* mk_f_instr(int val, string s) {
    f_instr* mem = allocate<f_instr>();
    return new (mem) f_instr(val, s);
  }

  comment* mk_comment(char ld, char rd, string t) {
    comment* mem = allocate<comment>();
    return new (mem) comment(ld, rd, t);
  }
    
  g91_instr* mk_G91() {
    return new (allocate<g91_instr>()) g91_instr();
  }

  g90_instr* mk_G90() {
    return new (allocate<g90_instr>()) g90_instr();
  }

  g20_instr* mk_G20() {
    return new (allocate<g20_instr>()) g20_instr();
  }
  
  g21_instr* mk_G21() {
    return new (allocate<g21_instr>()) g21_instr();
  }
  
  g64_instr* mk_G64() {
    return new (allocate<g64_instr>()) g64_instr();
  }
  
  g53_instr* mk_G53(value* x, value* y, value* z) {
    g53_instr* mem = allocate<g53_instr>();
    return new (mem) g53_instr(x, y, z, omitted::make());
  }

  hole_punch* mk_hole_punch(double x, double y, double z, double r) {
    hole_punch* mem = allocate<hole_punch>();
    return new (mem) hole_punch(point(x, y, z), r);
  }

  b_spline* mk_b_spline(int degree) {
    b_spline* mem = allocate<b_spline>();
    return new (mem) b_spline(degree);
  }
}
