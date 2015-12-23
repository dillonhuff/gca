#ifndef GCA_INSTR_H
#define GCA_INSTR_H

#include <cassert>
#include <iostream>

#include "point.h"

#define GCA_M 0
#define GCA_G 1

#define GCA_ABSOLUTE 0
#define GCA_RELATIVE 1

using namespace std;

namespace gca {

  typedef int orientation;
  typedef int instr_class;
  typedef int instr_val;

  class instr {
  protected:
    orientation orient;
    
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
      orient = GCA_ABSOLUTE;
    }

    instr(instr_class cp, instr_val vp, double xp, double yp, double zp,
	  orientation orientp=GCA_ABSOLUTE) {
      assert(cp == GCA_G);
      c = cp;
      v = vp;
      x = xp;
      y = yp;
      z = zp;
      feed_rate = -1.0;
      orient = orientp;
    }
    

    instr(instr_class cp, instr_val vp, double xp, double yp, double zp, double frp, orientation orientp=GCA_ABSOLUTE) {
      assert(cp == GCA_G);
      assert(frp > 0);
      c = cp;
      v = vp;
      x = xp;
      y = yp;
      z = zp;
      feed_rate = frp;
      orient = orientp;
    }
    
    bool operator==(const instr& other);

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    void print(ostream& s);

    bool is_G() const { return c == GCA_G; }

    point pos() const { return point(x, y, z); }

    bool is_abs() const { return orient == GCA_ABSOLUTE; }
    bool is_rel() const { return !is_abs(); }

  };

  ostream& operator<<(ostream& stream, instr& i);

}

#endif
