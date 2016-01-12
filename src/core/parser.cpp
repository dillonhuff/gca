#include <cctype>
#include <fstream>
#include <iostream>
#include <streambuf>

#include "parser.h"

namespace gca {

  void ignore_comment_with_delimiters(char sc, char ec, size_t* i, string s) {
    if (s[*i] == sc) {
      while (*i < s.size() && s[*i] != ec) { (*i)++; }
      (*i)++;
    }    
  }

  void ignore_comment(size_t* i, string s) {
    ignore_comment_with_delimiters('(', ')', i, s);
    ignore_comment_with_delimiters('[', ']', i, s);
  }

  void ignore_whitespace(size_t* i, string s) {
    while (*i < s.size() && (s[*i] == '(' || isspace(s[*i]))) {
      while (*i < s.size() && isspace(s[*i])) { (*i)++; }
      if (*i >= s.size()) { break; }
      ignore_comment(i, s);
    }
  }

  void parse_char(char c, size_t* i, string s) {
    if (s[*i] == c) {
      (*i)++;
      return;
    }
    cout << "Cannot parse char " << c << " from string " << s.substr(*i) << endl;
    assert(false);
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

  double parse_coordinate(char c, size_t* i, string s) {
    ignore_whitespace(i, s);
    assert(s[*i] == c);
    (*i)++;
    return parse_double(i, s);
  }

  double parse_option_coordinate(char c, size_t* i, string s, double def=0.0) {
    ignore_whitespace(i, s);
    if (s[*i] == c) {
      (*i)++;
      return parse_double(i, s);
    }
    return def;
  }

  point parse_point(context& c, gprog* p, size_t* i, string s, orientation ori) {
    double x, y, z;
    if (p->size() > 0 && ori == GCA_ABSOLUTE) {
      point lp = p->last_position();
      x = parse_option_coordinate('X', i, s, lp.x);
      y = parse_option_coordinate('Y', i, s, lp.y);
      z = parse_option_coordinate('Z', i, s, lp.z);
    } else if (ori == GCA_ABSOLUTE) {
      x = parse_coordinate('X', i, s);
      y = parse_coordinate('Y', i, s);
      z = parse_coordinate('Z', i, s);
    } else {
      x = parse_option_coordinate('X', i, s, 0);
      y = parse_option_coordinate('Y', i, s, 0);
      z = parse_option_coordinate('Z', i, s, 0);
    }
    return point(x, y, z);
  }

  instr* parse_G1(context& c, gprog* p, size_t* i, string s, orientation ori) {
    double default_feedrate = 1.0;
    double fr = parse_option_coordinate('F', i, s, default_feedrate);
    point pt = parse_point(c, p, i, s, ori);
    if (within_eps(fr, default_feedrate)) {
      fr = parse_option_coordinate('F', i, s, default_feedrate);
    }
    return c.mk_G1(pt.x, pt.y, pt.z, fr, ori);
  }

  instr* parse_G0(context& c, gprog* p, size_t* i, string s, orientation ori) {
    point pt = parse_point(c, p, i, s, ori);
    return c.mk_G0(pt, ori);
  }

  instr* parse_G53(context& c, gprog* p, size_t* i, string s, orientation ori) {
    point pt = parse_point(c, p, i, s, ori);
    return c.mk_G53(pt, ori);
  }

  // TODO: Actually parse the coordinate string
  string parse_coord_letters(size_t* i, string s) {
    char c = s[*i];
    while (c == 'X' || c == 'Y' || c == 'Z') {
      (*i)++;
      c = s[*i];
    }
    return "";
  }
  
  instr* parse_next_instr(context& c,
			  gprog* p,
			  size_t* i,
			  string s,
			  orientation* ori) {
    instr* is;
    char next_char = s[*i];
    (*i)++;
    int val = parse_int(i, s);
    if (next_char == 'M') {
      is = c.mk_minstr(val);
    } else if (next_char == 'T') {
      is = c.mk_tinstr(val);
    } else if (next_char == 'S') {
      is = c.mk_sinstr(val);
    } else if (next_char == 'F') {
      ignore_whitespace(i, s);
      string str = parse_coord_letters(i, s);
      is = c.mk_finstr(val, str);
    } else if (next_char == 'G') {
      if (val == 0) {
	is = parse_G0(c, p, i, s, *ori);
      } else if (val == 1) {
	is = parse_G1(c, p, i, s, *ori);
      } else if (val == 91) {
	is = c.mk_G91();
	*ori = GCA_RELATIVE;
      } else if (val == 90) {
	is = c.mk_G90();
	*ori = GCA_ABSOLUTE;
      } else if (val == 53) {
	is = parse_G53(c, p, i, s, *ori);
      } else {
	cout << "Unrecognized instr code for instr letter: " << val << endl;
	assert(false);
      }
      ignore_whitespace(i, s);
    } else if (next_char == '#') {
      cout << s.substr(*i) << endl;
      parse_char('=', i, s);
      double e = parse_double(i, s);
      is = c.mk_assign(c.mk_var(val), c.mk_lit(e));
    } else {
      cout << "Cannot parse string: " << s.substr(*i) << endl;
      assert(false);
    }
    return is;
  }
  
  gprog* parse_gprog(context& c, string s) {
    gprog* p = c.mk_gprog();
    string::size_type i = 0;
    orientation ori = GCA_ABSOLUTE;
    while (i < s.size()) {
      ignore_whitespace(&i, s);
      if (i >= s.size()) { break; }
      instr* is = parse_next_instr(c, p, &i, s, &ori);
      p->push_back(is);
    }
    return p;
  }

  gprog* read_file(context& c, string file_name) {
    ifstream t(file_name);
    string str((istreambuf_iterator<char>(t)),
	       istreambuf_iterator<char>());
    return parse_gprog(c, str);
  }

}
