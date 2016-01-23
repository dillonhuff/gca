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

    instr() {}

    instr(instr* i) {
      c = i->c;
      v = i->v;
    }
    
    instr(instr_class cp, instr_val vp) {
      assert(cp != GCA_G || (vp != 1 && vp != 0));
      c = cp;
      v = vp;
    }

    virtual inline bool is_g3_instr() const { return false; }
    virtual inline bool is_g2_instr() const { return false; }
    virtual inline bool is_f_instr() const { return false; }
    virtual inline bool is_comment() const { return false; }
    virtual inline bool is_assign_instr() const { return false; }
    virtual inline bool is_move_instr() const { return false; }
    inline bool is_end_instr() const {
      return c == GCA_M && (v == 2 || v == 30);
    }
    
    virtual bool operator==(const instr& other) const;

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    virtual void print(ostream& s) const;

    inline bool is_G() const { return c == GCA_G; }
    inline bool is_F() const { return c == GCA_F; }
    inline bool is_S() const { return c == GCA_S; }
    inline bool is_T() const { return c == GCA_T; }
    inline bool is_G0() const { return is_G() && v == 0; }
    inline bool is_G1() const { return is_G() && v == 1; }
    inline bool is_G91() const { return is_G() && v == 91; }
    inline bool is_G90() const { return is_G() && v == 90; }

  };

  ostream& operator<<(ostream& stream, const instr& i);

}

#endif
