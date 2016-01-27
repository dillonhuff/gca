#ifndef GCA_INSTR_H
#define GCA_INSTR_H

#include <cassert>
#include <iostream>

#include "geometry/point.h"

#define GCA_M 0
#define GCA_G 1
#define GCA_T 2
#define GCA_S 3
#define GCA_F 4

#define GCA_ABSOLUTE 0
#define GCA_RELATIVE 1
#define GCA_NONE 2

using namespace std;

namespace gca {

  typedef int orientation;
  typedef int instr_class;
  typedef int instr_val;

  class instr {
  protected:
    instr_class c;
    instr_val v;
    
  public:
    double feed_rate;

    /* instr() {} */

    /* instr(instr* i) { */
    /*   c = i->c; */
    /*   v = i->v; */
    /* } */
    
    /* instr(instr_class cp, instr_val vp) { */
    /*   assert(cp != GCA_G || (vp != 1 && vp != 0)); */
    /*   c = cp; */
    /*   v = vp; */
    /* } */

    virtual instr* copy() const { assert(false); }

    virtual inline bool is_g3_instr() const { return false; }
    virtual inline bool is_g2_instr() const { return false; }
    virtual inline bool is_f_instr() const { return false; }
    virtual inline bool is_comment() const { return false; }
    virtual inline bool is_assign_instr() const { return false; }
    virtual inline bool is_move_instr() const { return false; }
    virtual inline bool is_end_instr() const { return false; }
    
    virtual bool operator==(const instr& other) const;

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    virtual void print(ostream& s) const;

    virtual inline bool is_M3() const { return false; }
    virtual inline bool is_M5() const { return false; }
    virtual inline bool is_M2() const { return false; }
    virtual inline bool is_M30() const { return false; }
    virtual inline bool is_G() const { return is_G0() || is_G1() || is_G2() || is_G53() || is_G90() || is_G91(); }
    virtual inline bool is_M() const { return is_M3() || is_M5() || is_M2() || is_M30(); }
    virtual inline bool is_F() const { return false; }
    virtual inline bool is_S() const { return false; }
    virtual inline bool is_T() const { return false; }
    virtual inline bool is_G0() const { return false; }
    virtual inline bool is_G1() const { return false; }
    virtual inline bool is_G2() const { return false; }
    virtual inline bool is_G3() const { return false; }    
    virtual inline bool is_G91() const { return false; }
    virtual inline bool is_G90() const { return false; }
    virtual inline bool is_G53() const { return false; }

  };

  ostream& operator<<(ostream& stream, const instr& i);

}

#endif
