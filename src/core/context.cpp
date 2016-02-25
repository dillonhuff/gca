#include "core/context.h"

namespace gca {

  assign_instr* mk_assign(var* v, value* e) {
    assign_instr* mem = allocate<assign_instr>();
    return new (mem) assign_instr(v, e);
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

  m30_instr* mk_m30_instr() {
    return new (allocate<m30_instr>()) m30_instr();
  }

  m5_instr* mk_m5_instr() {
    return new (allocate<m5_instr>()) m5_instr();
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
