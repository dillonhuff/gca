#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "core/gprog.h"
#include "core/instrs/all.h"
#include "synthesis/hole_punch.h"
#include "synthesis/linear_cut.h"

namespace gca {

  omitted* mk_omitted();

  var* mk_var(int v);
  
  lit* mk_lit(double v);

  g0_instr* mk_G0(point p);  
  g0_instr* mk_G0(double x, double y, double z);
  g0_instr* mk_G0(value* x, value* y, value* z);
      
  g1_instr* mk_G1(double x, double y, double z, double feed_rate=1.0);
  g1_instr* mk_G1(value* x, value* y, value* z, value* feed_rate);
  g1_instr* mk_G1(double x, double y, double z, value* feed_rate);
  
  g2_instr* mk_G2(value* x, value* y, value* z,
		  value* i, value* j, value* k,
		  value* feed_rate);  

  g3_instr* mk_G3(value* x, value* y, value* z,
		  value* i, value* j, value* k,
		  value* feed_rate);

  instr* mk_instr_cpy(instr* i);

  linear_cut* mk_linear_cut(point start, point end);

  assign_instr* mk_assign(var* v, value* e);
    
  gprog* mk_gprog();

  m2_instr* mk_m2_instr();
  m30_instr* mk_m30_instr();
  m5_instr* mk_m5_instr();
  m3_instr* mk_m3_instr();

  t_instr* mk_tinstr(int val);

  s_instr* mk_sinstr(int val);

  f_instr* mk_finstr(int val, string s);

  comment* mk_comment(char ld, char rd, string t);
    
  g91_instr* mk_G91();
  g90_instr* mk_G90();
    
  g53_instr* mk_G53(value* x, value* y, value* z);

  hole_punch* mk_hole_punch(double x, double y, double z, double r);

}

#endif
