#ifndef GCA_RELATION_H
#define GCA_RELATION_H

#include <map>
#include <vector>

namespace gca {

  template<typename L, typename R>
  class relation {
  protected:
    typedef unsigned l_index;
    typedef unsigned r_index;
    
    std::vector<L> left;
    std::vector<R> right;

    std::map<l_index, std::vector<r_index> > l_to_r;
    std::map<r_index, std::vector<l_index> > r_to_l;

  public:
    relation(const std::vector<L>& ls,
	     const std::vector<R>& rs) :
      left(ls), right(rs) {
      for (unsigned i = 0; i < left.size(); i++) {
	l_to_r[i] = {};
      }

      for (unsigned i = 0; i < right.size(); i++) {
	r_to_l[i] = {};
      }
    }

    void insert(const l_index l, const r_index r) {
      l_to_r[l].push_back(r);
      r_to_l[r].push_back(l);
    }

    inline unsigned right_size() const { return right.size(); }

    const std::map<l_index, std::vector<r_index> >
    left_to_right() const { return l_to_r; }
    //    std::map<r_index, std::vector<l_index> > r_to_l;
    
  };
}

#endif
