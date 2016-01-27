#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "synthesis/cut.h"
#include "core/gprog.h"
#include "core/instrs/all.h"

namespace gca {

  omitted* mk_omitted();

  var* mk_var(int v);
  
  lit* mk_lit(double v);
  
  move_instr* mk_G0(point p);
    
  move_instr* mk_G1(double x, double y, double z, double feed_rate=1.0);
  
  move_instr* mk_G1(value* x, value* y, value* z, value* feed_rate);
    
  move_instr* mk_G1(double x, double y, double z, value* feed_rate);
  
  g2_instr* mk_G2(value* x, value* y, value* z,
		  value* i, value* j, value* k,
		  value* feed_rate);
  
  g3_instr* mk_G3(value* x, value* y, value* z,
		  value* i, value* j, value* k,
		  value* feed_rate);
  
  move_instr* mk_G0(double x, double y, double z);

  move_instr* mk_G0(value* x, value* y, value* z);
      
  instr* mk_instr_cpy(instr* i);

  cut* mk_cut(point start, point end);

  assign_instr* mk_assign(var* v, value* e);
    
  gprog* mk_gprog();

  instr* mk_minstr(int val);

  instr* mk_tinstr(int val);

  instr* mk_sinstr(int val);

  f_instr* mk_finstr(int val, string s);

  comment* mk_comment(char ld, char rd, string t);
    
  instr* mk_G91();

  instr* mk_G90();
    
  move_instr* mk_G53(value* x, value* y, value* z);
  
  class context {
  protected:
  public:
  };
  
}

#endif
