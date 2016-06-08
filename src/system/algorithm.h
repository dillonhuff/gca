#ifndef GCA_SYSTEM_ALGORITHM_H
#define GCA_SYSTEM_ALGORITHM_H

#include <utility>

namespace gca {

  template<typename T>
  void concat(T& extend, const T& add) {
    extend.insert(end(extend), begin(add), end(add));
  }

  template<typename E, typename T>
  void remove(E e, T& t) {
    t.erase(std::remove(begin(t), end(t), e), end(t));
  }

  template<typename E, typename T>
  bool elem(E e, T t) {
    return std::find(begin(t), end(t), e) != end(t);
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
  bool any_between(InputIt s, InputIt e, F f) {
    while (s != (e - 1)) {
      if (f(*s, *(s + 1))) { return true; };
      ++s;
    }
    return false;
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
      if (found_adjacent) { std::swap(*(s + 1), *n); }
      s++;
    }
  }

  // Could use adjacent_find in place of this function, but I
  // found that the return policy of giving the first element of
  // the matching pair if a match occurs, and end otherwise made
  // iteration awkward when you want to construct new data structures
  // out of the subranges produced by calls to adjacent_find
  template<typename InputIt, typename F>
  std::pair<InputIt, InputIt> find_between(InputIt s, InputIt e, F f) {
    while (s != (e - 1)) {
      if (f(*s, *(s + 1))) { return std::pair<InputIt, InputIt>(s, s + 1); }
      ++s;
    }
    return std::pair<InputIt, InputIt>(s, e);
  }

  template<typename I, typename F>
  std::vector<std::vector<I>> split_by(const std::vector<I>& elems, F f) {
    std::vector<std::vector<I>> split;
    split_by(elems, split, f);
    return split;
  }
  
  template<typename I, typename F>
  void split_by(const std::vector<I>& elems, std::vector<std::vector<I>>& res, F f) {
    auto it = elems.begin();
    auto not_f = [&f](const I& i, const I& j) { return !f(i, j); };
    while (it != elems.end()) {
      auto r = find_between(it, end(elems), not_f);
      res.push_back(std::vector<I>(it, r.first + 1));
      it = r.second;
    }
  }

  template<typename I, typename F>
  std::vector<std::vector<I>> group_unary(const std::vector<I>& elems, F f) {
    std::vector<std::vector<I>> grouped;
    auto match = [&f](const I& i, const I& j) { return f(i) == f(j); };
    split_by(elems, grouped, match);
    return grouped;
  }

  template<typename I, typename F>
  void delete_if(I& c, F f) {
    c.erase(remove_if(begin(c), end(c), f), end(c));
  }

  template<typename T, typename Q>
  std::pair<T, Q>
  mk_pair(T t, Q q) { return std::pair<T, Q>(t, q); }

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
  void drop_while(T& t, F f) {
    auto r = std::find_if_not(begin(t), end(t), f);
    auto s = std::distance(begin(t), r);
    std::rotate(begin(t), r, end(t));
    t.erase(begin(t) + t.size() - s, end(t));
  }

  template<typename T, typename F>
  void take_while(T& t, F f) {
    auto r = find_if_not(begin(t), end(t), f);
    auto s = distance(begin(t), r);
    t.erase(begin(t) + s, end(t));
  }

  template<typename T>
  void subtract(T& a, const T& b) {
    delete_if(a,
	      [&b](const typename T::value_type& i)
	      { return std::find(begin(b), end(b), i) != end(b); });
  }

  template<typename T, typename F>
  std::vector<T> select(const std::vector<T>& v, F f) {
    std::vector<T> selected;
    for (auto e : v) {
      if (f(e)) {
	selected.push_back(f);
      }
    }
    return selected;
  }

  template<typename I, typename P>
  std::vector<unsigned>
  dfs_by(std::vector<unsigned>& inds, const std::vector<I>& elems, P p) {
    std::vector<unsigned> comp;
    if (inds.size() == 0) {
      return comp;
    }
    std::vector<unsigned> buf;
    buf.push_back(inds.back());
    inds.pop_back();
    while (buf.size() > 0) {
      auto next = buf.back();
      buf.pop_back();
      comp.push_back(next);
      std::vector<unsigned> inds_to_remove;
      for (unsigned i = 0; i < inds.size(); i++) {
	unsigned u = inds[i];
	if (u != next && p(elems[u], elems[next])) {
	  buf.push_back(u);
	  inds_to_remove.push_back(u);
	}
      }
      subtract(inds, inds_to_remove);
    }
    return comp;
  }

  template<typename I, typename P>
  std::vector<std::vector<unsigned>>
  connected_components_by(const std::vector<I>& elems, P p) {
    std::vector<std::vector<unsigned>> components;
    std::vector<unsigned> inds(elems.size());
    std::iota(begin(inds), end(inds), 0);
    while (inds.size() > 0) {
      components.push_back(dfs_by(inds, elems, p));
    }
    return components;
  }

  // TODO: system/algorithm ?
  template<typename I, typename F>
  vector<I>
  greedy_chain(const I& init, const std::vector<I>& elems, F f) {
    std::vector<I> chain;
    chain.push_back(init);
    if (elems.size() == 0) { return chain; }
    vector<unsigned> used;
    bool found_next = true;
    while (found_next) {
      found_next = false;
      for (unsigned i = 0; i < elems.size(); i++) {
	if (!elem(i, used) && f(elems[i], chain.back())) {
	  chain.push_back(elems[i]);
	  used.push_back(i);
	  found_next = true;
	}
      }
    }
    return chain;
  }


  template<typename I>
  std::vector<I>
  copy_not_indexes(const std::vector<I>& elems,
		   std::vector<unsigned>& inds) {
    std::sort(begin(inds), end(inds));
    std::vector<I> cp_elems;
    for (unsigned i = 0; i < elems.size(); i++) {
      if (!(std::binary_search(begin(inds), end(inds), i))) {
    	cp_elems.push_back(elems[i]);
      }
    }
    return cp_elems;
  }
  
}

#endif
