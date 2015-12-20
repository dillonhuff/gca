#ifndef GCA_INSTR_H
#define GCA_INSTR_H

#include <cassert>

#define GCA_M 0
#define GCA_G 1

namespace gca {

  typedef int instr_class;
  typedef int instr_val;

  class instr {

  protected:
    instr_class c;
    instr_val v;
    double x, y, z;

  public:
    instr(instr_class cp, instr_val vp) {
      assert(cp != GCA_G);
      c = cp;
      v = vp;
    }

    instr(instr_class cp, instr_val vp, double xp, double yp, double zp) {
      assert(cp == GCA_G);
      c = cp;
      v = vp;
      x = xp;
      y = yp;
      z = zp;
    }
    
    bool operator==(const instr& other) {
      if (c != other.c || v != other.v) {
	return false;
      }
      if (c == GCA_G) {
	return x == other.x && y == other.y && z == other.z;
      }
      return true;
    }

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

  };

}

#endif
