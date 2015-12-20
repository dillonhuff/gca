#include <cctype>
#include <iostream>

#include "src/parser.h"

namespace gca {

  void ignore_whitespace(size_t* i, string s) {
    while (*i < s.size() && isspace(s[*i])) {
      (*i)++;
    }
  }

  double parse_option_coordinate(char c, size_t* i, string s) {
    cout << "c = " << c << endl;
    cout << "Parse option: " << s.substr(*i) << endl;
    ignore_whitespace(i, s);
    if (s[*i] == c) {
      (*i)++;
      size_t j = *i;
      double v = stod(s.substr(*i), &j);
      *i += j;
      cout << "v = " << v << endl;
      cout << "s = " << s.substr(*i) << endl;
      return v;
    }
    assert(false);
  }

  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    for (string::size_type i = 0; i < s.size(); i++) {
      if (s[i] == 'M') {
	i++;
      	int val = stoi(s.substr(i), &i);
      	p->push_back(c.mk_minstr(val));
      } if (s[i] == 'G') {
	i++;
	int val = stoi(s.substr(i), &i);
	if (val == 0) {
	  i++;
	  double x = parse_option_coordinate('X', &i, s);
	  double y = parse_option_coordinate('Y', &i, s);
	  double z = parse_option_coordinate('Z', &i, s);
	  p->push_back(c.mk_G0(x, y, z));
	} else if (val == 1) {
	  i++;
	  double x = parse_option_coordinate('X', &i, s);
	  double y = parse_option_coordinate('Y', &i, s);
	  double z = parse_option_coordinate('Z', &i, s);
	  p->push_back(c.mk_G1(x, y, z));	  
	} else {
	  cout << "Unrecognized instr code for instr letter: " << val << endl;
	  assert(false);
	}
      }
    }
    return p;
  }
  
}
