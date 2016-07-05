#ifndef GCA_VALUE_H
#define GCA_VALUE_H

#include <cassert>
#include <iostream>

#include "utils/arena_allocator.h"
#include "geometry/point.h"

using namespace std;

namespace gca {

  class value {
  public:

    virtual inline bool is_omitted() const { return false; }
    virtual inline bool is_lit() const { return false; }
    virtual inline bool is_var() const { return false; }
    virtual inline bool is_ilit() const { return false; }    
    virtual bool operator==(const value& other) const = 0;    
    virtual void print(ostream& other) const = 0;

    virtual void print_eps(ostream& s, double ) const {
      print(s);
    }
  };

  class lit : public value {
  public:
    double v;

  lit(double vp) : v(vp) {}

    static lit* make(double vp) {
      lit* mem = allocate<lit>();
      return new (mem) lit(vp);
    }
    
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
	s << 0.0;
      }
    }
  };  

  class ilit : public value {
  public:
    int v;

  ilit(int vp) : v(vp) {}

    static ilit* make(double vp) {
      ilit* mem = allocate<ilit>();
      return new (mem) ilit(vp);
    }
    
    virtual inline bool is_ilit() const { return true; }

    virtual bool operator==(const value& other) const {
      if (other.is_ilit()) {
	const ilit& other_lit = static_cast<const ilit&>(other);
	return other_lit.v == v;
      }
      return false;
    }

    virtual void print(ostream& other) const { other << v; }
  };  
  
  class var : public value {
  public:
    int n;

  var(int np) : n(np) {}

    static var* make(int np) {
      var* mem = allocate<var>();
      return new (mem) var(np);
    }
    
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
  public:
    static omitted* make() {
      omitted* mem = allocate<omitted>();
      return new (mem) omitted();
    }

    virtual inline bool is_omitted() const { return true; }
    
    virtual bool operator==(const value& other) const {
      if (other.is_omitted()) {
	return true;
      }
      return false;
    }

    virtual void print(ostream& ) const {}
    
  };

  ostream& operator<<(ostream& s, const value& v);
}

#endif
