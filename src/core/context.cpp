#include "core/context.h"

namespace gca {

  assign_instr* mk_assign(var* v, value* e) {
    assign_instr* mem = allocate<assign_instr>();
    return new (mem) assign_instr(v, e);
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

}
