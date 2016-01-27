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
  
  class context {

  protected:

  public:

    assign_instr* mk_assign(var* v, value* e) {
      assign_instr* mem = allocate<assign_instr>();
      return new (mem) assign_instr(v, e);
    }
    
    gprog* mk_gprog() {
      gprog* mem = allocate<gprog>();
      return new (mem) gprog();
    }

    instr* mk_minstr(int val) {
      instr* mem = allocate<instr>();
      return new (mem) instr(GCA_M, val);
    }

    instr* mk_tinstr(int val) {
      instr* mem = allocate<instr>();
      return new (mem) instr(GCA_T, val);
    }

    instr* mk_sinstr(int val) {
      instr* mem = allocate<instr>();
      return new (mem) instr(GCA_S, val);
    }

    f_instr* mk_finstr(int val, string s) {
      f_instr* mem = allocate<f_instr>();
      return new (mem) f_instr(val, s);
    }

    comment* mk_comment(char ld, char rd, string t) {
      comment* mem = allocate<comment>();
      return new (mem) comment(ld, rd, t);
    }
    
    instr* mk_G91() {
      instr* mem = allocate<instr>();
      return new (mem) instr(GCA_G, 91);
    }

    instr* mk_G90() {
      instr* mem = allocate<instr>();
      return new (mem) instr(GCA_G, 90);
    }
    
    move_instr* mk_G53(value* x, value* y, value* z) {
      move_instr* mem = allocate<move_instr>();
      return new (mem) move_instr(GCA_G, 53, x, y, z, mk_omitted());
    }
        
    cut* mk_cut(point start, point end) {
      cut* mem = allocate<cut>();
      return new (mem) cut(start, end);
    }


  };
  
}

#endif
