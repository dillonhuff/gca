#ifndef GCA_VALUE_H
#define GCA_VALUE_H

#include <cassert>
#include <iostream>

#include "geometry/point.h"

using namespace std;

namespace gca {

  class value {
  public:

    virtual inline bool is_omitted() const { return false; }
    virtual inline bool is_lit() const { return false; }
    virtual inline bool is_var() const { return false; }
    
    virtual bool operator==(const value& other) const {
      assert(false);
    }
    
    virtual void print(ostream& other) const {
      assert(false);
    }

    virtual void print_eps(ostream& s, double eps) const {
      print(s);
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
	return within_eps(other_lit.v, v, 0.001);
      }
      return false;
    }

    virtual void print(ostream& other) const { other << v; }

    virtual void print_eps(ostream& s, double eps) const {
      double abs_i = v >= 0 ? v : -1*v;
      if (abs_i >= eps) {
	s << v;
      } else {
	cout << 0.0;
      }
    }

  };  

  class var : public value {
  public:
    int n;

  var(int np) : n(np) {}

    virtual inline bool is_var() const { return true; }

    virtual bool operator==(const value& other) const {
      if (other.is_var()) {
	const var& other_v = static_cast<const var&>(other);
	return other_v.n == n;
      }
      return false;
    }    

    virtual void print(ostream& other) const { other << '#' << n; }
  };

  class omitted : public value {
    virtual inline bool is_omitted() const { return true; }
    
    virtual bool operator==(const value& other) const {
      if (other.is_omitted()) {
	return true;
      }
      return false;
    }

    virtual void print(ostream& other) const {}
    
  };

  ostream& operator<<(ostream& s, const value& v);
}

#endif
