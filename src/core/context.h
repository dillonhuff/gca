#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "core/gprog.h"
#include "core/instrs/all.h"
#include "geometry/b_spline.h"
#include "synthesis/hole_punch.h"
#include "synthesis/linear_cut.h"

namespace gca {

  instr* mk_instr_cpy(instr* i);

  assign_instr* mk_assign(var* v, value* e);
    
  gprog* mk_gprog();

  t_instr* mk_t_instr(int val);

  s_instr* mk_s_instr(int val);

  f_instr* mk_f_instr(int val, string s);

  comment* mk_comment(char ld, char rd, string t);

  m2_instr* mk_m2_instr();
  m30_instr* mk_m30_instr();
  m5_instr* mk_m5_instr();
  m3_instr* mk_m3_instr();
  g91_instr* mk_G91();
  g90_instr* mk_G90();
  g20_instr* mk_G20();
  g21_instr* mk_G21();
  g64_instr* mk_G64();
  
    
  g53_instr* mk_G53(value* x, value* y, value* z);

  hole_punch* mk_hole_punch(double x, double y, double z, double r);

  b_spline* mk_b_spline(int degree);

}

#endif
