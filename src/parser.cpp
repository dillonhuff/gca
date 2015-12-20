#include <cctype>
#include <iostream>

#include "src/parser.h"

namespace gca {

  void ignore_whitespace(size_t* i, string s) {
    while (*i < s.size() && isspace(s[*i])) {
      (*i)++;
    }
  }

  double parse_double(size_t* i, string s) {
    size_t j = *i;
    double v = stod(s.substr(*i), &j);
    *i += j;
    return v;
  }

  int parse_int(size_t* i, string s) {
    size_t j = *i;
    int v = stoi(s.substr(*i), &j);
    *i += j;
    return v;
  }
  
  double parse_option_coordinate(char c, size_t* i, string s) {
    ignore_whitespace(i, s);
    if (s[*i] == c) {
      (*i)++;
      return parse_double(i, s);
    }
    return 0.0;
  }

  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    string::size_type i = 0;
    while ( i < s.size()) {
      cout << "i = " << i << endl;
      cout << "s.substr(i) = " << s.substr(i) << endl;
      ignore_whitespace(&i, s);
      if (s[i] == 'M') {
	i++;
	int val = parse_int(&i, s);
      	p->push_back(c.mk_minstr(val));
      } else if (s[i] == 'G') {
	i++;
	int val = parse_int(&i, s);
	if (val == 0) {
	  double x = parse_option_coordinate('X', &i, s);
	  double y = parse_option_coordinate('Y', &i, s);
	  double z = parse_option_coordinate('Z', &i, s);
	  p->push_back(c.mk_G0(x, y, z));
	} else if (val == 1) {
	  double x = parse_option_coordinate('X', &i, s);
	  double y = parse_option_coordinate('Y', &i, s);
	  double z = parse_option_coordinate('Z', &i, s);
	  p->push_back(c.mk_G1(x, y, z));
	} else {
	  cout << "Unrecognized instr code for instr letter: " << val << endl;
	  assert(false);
	}
	ignore_whitespace(&i, s);
      } else {
	assert(false);
      }
    }
    return p;
  }
  
}
