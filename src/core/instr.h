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
      //position = i->pos();
      //feed_rate = i->feed_rate;
      //orient = i->orient;
    }
    
    instr(instr_class cp, instr_val vp) {
      assert(cp != GCA_G || (vp != 1 && vp != 0));
      c = cp;
      v = vp;
    }

    inline bool is_end_instr() const {
      return c == GCA_M && (v == 2 || v == 30);
    }
    
    bool operator==(const instr& other);

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    void print(ostream& s) const;

    inline bool is_G() const { return c == GCA_G; }
    inline bool is_F() const { return c == GCA_F; }
    inline bool is_S() const { return c == GCA_S; }
    inline bool is_T() const { return c == GCA_T; }
    inline bool is_G0() const { return is_G() && v == 0; }
    inline bool is_G1() const { return is_G() && v == 1; }
    inline bool is_G91() const { return is_G() && v == 91; }

  };

  ostream& operator<<(ostream& stream, const instr& i);

}

#endif
