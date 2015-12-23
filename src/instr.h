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
    point position;
    
  public:
    instr_class c;
    instr_val v;
    double feed_rate;

    instr(instr* i) {
      c = i->c;
      v = i->v;
      position = i->pos();
      feed_rate = i->feed_rate;
      orient = i->orient;
    }
    
    instr(instr_class cp, instr_val vp) {
      assert(cp != GCA_G);
      c = cp;
      v = vp;
      feed_rate = -1.0;
      position = point(0, 0, 0);
      orient = GCA_ABSOLUTE;
    }

    instr(instr_class cp, instr_val vp, point p, orientation orientp=GCA_ABSOLUTE) {
      assert(cp == GCA_G);
      c = cp;
      v = vp;
      position = p;
      feed_rate = -1.0;
      orient = orientp;
    }
    

    instr(instr_class cp, instr_val vp, point p, double frp, orientation orientp=GCA_ABSOLUTE) {
      assert(cp == GCA_G);
      assert(frp > 0);
      c = cp;
      v = vp;
      position = p;
      feed_rate = frp;
      orient = orientp;
    }
    
    bool operator==(const instr& other);

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

    void print(ostream& s) const;

    bool is_G() const { return c == GCA_G; }

    point pos() const { return position; }

    bool is_abs() const { return orient == GCA_ABSOLUTE; }
    bool is_rel() const { return !is_abs(); }

    void swap_orientation() {
      orient = orient == GCA_ABSOLUTE ? GCA_RELATIVE : GCA_ABSOLUTE;
    }

  };

  ostream& operator<<(ostream& stream, const instr& i);

}

#endif
