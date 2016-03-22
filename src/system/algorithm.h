#ifndef GCA_SYSTEM_ALGORITHM_H
#define GCA_SYSTEM_ALGORITHM_H

#include <utility>

using namespace std;

namespace gca {

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
  void delete_if(I& c, F f) {
    c.erase(remove_if(c.begin(), c.end(), f), c.end());
  }
}

#endif
