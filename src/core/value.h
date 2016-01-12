#ifndef GCA_VALUE_H
#define GCA_VALUE_H

#include <cassert>
#include <iostream>

using namespace std;

namespace gca {

  class value {
  public:

    virtual inline bool is_lit() const { return false; }
    
    virtual bool operator==(const value& other) const {
      assert(false);
    }

    virtual void print(ostream& other) const {
      assert(false);
    }
  };

  class lit : public value {
  public:
    double v;

  lit(double vp) : v(vp) {}
    
    virtual inline bool is_lit() const { return true; }

    virtual bool operator==(const value& other) const {
      if (other.is_lit()) {
	const lit& other_lit = static_cast<const lit&>(other);
	return other_lit.v == v;
      }
      return false;
    }
  };  

  class var : public value {
  public:
    string n;
  };

  ostream& operator<<(ostream& s, const value& v);
}

#endif
