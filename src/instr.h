#ifndef GCA_INSTR_H
#define GCA_INSTR_H

#include <cassert>
#include <iostream>

#define GCA_M 0
#define GCA_G 1

using namespace std;

namespace gca {

  typedef int instr_class;
  typedef int instr_val;

  class instr {
  public:
    instr_class c;
    instr_val v;
    double x, y, z;
    double feed_rate;
    
    instr(instr_class cp, instr_val vp) {
      assert(cp != GCA_G);
      c = cp;
      v = vp;
      feed_rate = -1.0;
    }

    instr(instr_class cp, instr_val vp, double xp, double yp, double zp) {
      assert(cp == GCA_G);
      c = cp;
      v = vp;
      x = xp;
      y = yp;
      z = zp;
      feed_rate = -1.0;
    }
    

    instr(instr_class cp, instr_val vp, double xp, double yp, double zp, double frp) {
      assert(cp == GCA_G);
      assert(frp > 0);
      c = cp;
      v = vp;
      x = xp;
      y = yp;
      z = zp;
      feed_rate = frp;
    }
    
    bool operator==(const instr& other);

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    void print(ostream& s);

  };

  ostream& operator<<(ostream& stream, instr& i);

}

#endif
