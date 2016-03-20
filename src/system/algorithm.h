#ifndef GCA_SYSTEM_ALGORITHM_H
#define GCA_SYSTEM_ALGORITHM_H

namespace gca {

  template<typename input_it, typename output_it, typename F>
  void apply_between(input_it s, input_it e, output_it r, F f) {
    while (s != (e - 1)) {
      *r = f(*s, *(s + 1));
      ++r;
      ++s;
    }
  }

}

#endif
