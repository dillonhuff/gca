#ifndef GCA_DIFFS_ALL_H
#define GCA_DIFFS_ALL_H

#include <iostream>

#include "core/instrs/all.h"

namespace gca {

  class diff {
  public:
    virtual ~diff() {}
    virtual void print(ostream& s) const = 0;
    virtual bool same_effect(const diff& other) const = 0;
    virtual inline bool is_shift_xyz() const { return false; }
  };

  class swap : public diff {
  public:
    instr* l;
    instr* r;

  swap(instr* lp, instr* rp) : l(lp), r(rp) {}

    virtual bool same_effect(const diff& other) const { return false; }

    virtual void print(ostream& s) const {
      s << *l << " --> " << *r;
    }

  };

  class append : public diff {
  public:
    vector<instr*> instrs;

    virtual bool same_effect(const diff& other) const { return false; }
    virtual void print(ostream& s) const {
      s << "---------- Append -----------" << endl;
      for (int i = 0; i < instrs.size(); i++) {
	s << *instrs[i] << endl;
      }
      s << "-----------------------------" << endl;
    }
  };

  class shift_xyz : public diff {
  public:
    point p;
  shift_xyz(point sf) : p(sf) {}

    inline bool is_shift_xyz() const { return true; }
    virtual bool same_effect(const diff& other) const {
      if (other.is_shift_xyz()) {
	const shift_xyz& os = static_cast<const shift_xyz&>(other);
	return within_eps(p, os.p);
      }
      return false;
    }
    
    virtual void print(ostream& s) const {
      s << "shift " << p;
    }
  };

  ostream& operator<<(ostream& s, const diff& d) {
    d.print(s);
    return s;
  }
}

#endif
