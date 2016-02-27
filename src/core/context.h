#ifndef GCA_CONTEXT_H
#define GCA_CONTEXT_H

#include "arena_allocator.h"
#include "core/gprog.h"
#include "core/instrs/all.h"
#include "geometry/b_spline.h"
#include "synthesis/hole_punch.h"
#include "synthesis/linear_cut.h"

namespace gca {

  assign_instr* mk_assign(var* v, value* e);
    
  f_instr* mk_f_instr(int val, string s);

  comment* mk_comment(char ld, char rd, string t);

  g91_instr* mk_G91();
  g20_instr* mk_G20();
  g21_instr* mk_G21();
  g64_instr* mk_G64();
  
    
  g53_instr* mk_G53(value* x, value* y, value* z);

  b_spline* mk_b_spline(int degree);

}

#endif
