#ifndef GCA_SYSTEM_ALGORITHM_H
#define GCA_SYSTEM_ALGORITHM_H

#include <utility>

using namespace std;

namespace gca {

  template<typename E, typename T>
  void remove(E e, T& t) {
    t.erase(remove(begin(t), end(t), e), end(t));
  }

  template<typename E, typename T>
  bool elem(E e, T t) {
    return find(begin(t), end(t), e) != end(t);
  }

  template<typename InputIt, typename OutputIt, typename F>
  OutputIt apply_between(InputIt s, InputIt e, OutputIt r, F f) {
    while (s != (e - 1)) {
      *r = f(*s, *(s + 1));
      ++r;
      ++s;
    }
    return r;
  }

  template<typename InputIt, typename F>
  void apply_between(InputIt s, InputIt e, F f) {
    if (s == e) { return; }
    while (s != (e - 1)) {
      f(*s, *(s + 1));
      ++s;
    }
  }
  
  template<typename InputIt, typename F>
  bool all_between(InputIt s, InputIt e, F f) {
    while (s != (e - 1)) {
      if (!f(*s, *(s + 1))) { return false; };
      ++s;
    }
    return true;
  }

  template<typename InputIt, typename F>
  void greedy_adjacent_chains(InputIt s, InputIt e, F f) {
    if (s == e) { return; }
    while (s < e - 1) {
      auto n = s + 1;
      bool found_adjacent = false;
      while (n < e) {
	if (f(*s, *n)) {
	  found_adjacent = true;
	  break;
	}
	n++;
      }
      if (found_adjacent) { swap(*(s + 1), *n); }
      s++;
    }
  }

  // Could use adjacent_find in place of this function, but I
  // found that the return policy of giving the first element of
  // the matching pair if a match occurs, and end otherwise made
  // iteration awkward when you want to construct new data structures
  // out of the subranges produced by calls to adjacent_find
  template<typename InputIt, typename F>
  pair<InputIt, InputIt> find_between(InputIt s, InputIt e, F f) {
    while (s != (e - 1)) {
      if (f(*s, *(s + 1))) { return pair<InputIt, InputIt>(s, s + 1); }
      ++s;
    }
    return pair<InputIt, InputIt>(s, e);
  }

  template<typename I, typename F>
  void split_by(const vector<I>& elems, vector<vector<I>>& res, F f) {
    auto it = elems.begin();
    auto not_f = [&f](const I& i, const I& j) { return !f(i, j); };
    while (it != elems.end()) {
      auto r = find_between(it, elems.end(), not_f);
      res.push_back(vector<I>(it, r.first + 1));
      it = r.second;
    }
  }

  template<typename I, typename F>
  vector<vector<I>> group_unary(const vector<I>& elems, F f) {
    vector<vector<I>> grouped;
    auto match = [&f](const I& i, const I& j) { return f(i) == f(j); };
    split_by(elems, grouped, match);
    return grouped;
  }

  template<typename I, typename F>
  void delete_if(I& c, F f) {
    c.erase(remove_if(c.begin(), c.end(), f), c.end());
  }

  template<typename T, typename Q>
  pair<T, Q>
  mk_pair(T t, Q q) { return pair<T, Q>(t, q); }

  // TODO: Figure out why this implementation always calls
  // the default constructor
  template<typename InputIt1, typename InputIt2, typename OutputIt>
  OutputIt
  zip(InputIt1 ps, InputIt1 pe, InputIt2 qs, OutputIt r) {
    while (ps != pe) {
      *r = mk_pair(*ps, *qs);
      ++ps;
      ++qs;
      ++r;
    }
    return r;
  }

  template<typename T, typename F>
  void
  drop_while(T& t, F f) {
    auto r = find_if_not(t.begin(), t.end(), f);
    auto s = distance(t.begin(), r);
    rotate(t.begin(), r, t.end());
    t.erase(t.begin() + t.size() - s, t.end());
  }

  // template<typename T, typename Q>
  // struct pair_iter {
  //   typename T::iterator t;
  //   typename Q::iterator q;
  //   pair_iter(typename T::iterator tp, typename Q::iterator qp) : t(tp), q(qp) {}

  //   friend pair_iter<T, Q> operator-(const pair_iter<T, Q> i, size_t t)
  //   { return pair_iter<T, Q>(i.t - t, i.q - t); }

  //   bool operator!=(const pair_iter<T, Q> other)
  //   { return (t != other.t) || (q != other.q); }

  //   std::pair<T, Q> operator
  // };

  // template<typename T, typename Q>
  // pair_iter<T, Q> pair_begin(const T& t, const Q& q) {
  //   return pair_iter<T, Q>(t.begin(), q.begin());
  // }

  // template<typename T, typename Q>
  // pair_iter<T, Q> pair_end(const T& t, const Q& q) {
  //   return pair_iter<T, Q>(t.end(), q.end());
  // }
}

#endif
