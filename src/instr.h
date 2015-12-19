#ifndef GCA_INSTR_H
#define GCA_INSTR_H

//#define M 0

namespace gca {

  typedef int instr_class;
  typedef int instr_val;

  class instr {

  protected:
    instr_class c;
    instr_val v;

  public:
    instr(instr_class cp, instr_val vp) {
      c = cp;
      v = vp;
    }

    bool operator==(const instr& other) {
      return c == other.c && v == other.v;
    }

    bool operator!=(const instr& other) {
      return !(*this == other);
    }

  };

}

#endif
