#ifndef GCA_SYSTEM_ALGORITHM_H
#define GCA_SYSTEM_ALGORITHM_H

#include <utility>

using namespace std;

namespace gca {

  template<typename input_it, typename output_it, typename F>
  output_it apply_between(input_it s, input_it e, output_it r, F f) {
    while (s != (e - 1)) {
      *r = f(*s, *(s + 1));
      ++r;
      ++s;
    }
    return r;
  }

  template<typename input_it, typename F>
  bool all_between(input_it s, input_it e, F f) {
    while (s != (e - 1)) {
      if (!f(*s, *(s + 1))) { return false; };
      ++s;
    }
    return true;
  }

  template<typename input_it, typename F>
  void greedy_adjacent_chains(input_it s, input_it e, F f) {
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

  template<typename input_it, typename F>
  pair<input_it, input_it> find_between(input_it s, input_it e, F f) {
    while (s != (e - 1)) {
      if (f(*s, *(s + 1))) { return pair<input_it, input_it>(s, s + 1); }
      ++s;
    }
    return pair<input_it, input_it>(s, e);
  }
}

#endif
