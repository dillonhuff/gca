#include "core/parser.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <streambuf>

namespace gca {

  void ignore_comment_with_delimiters(char sc, char ec, size_t* i, const string& s) {
    if (s[*i] == sc) {
      while (*i < s.size() && s[*i] != ec) { (*i)++; }
      (*i)++;
    }    
  }

  void parse_char(char c, size_t* i, const string& s) {
    if (s[*i] == c) {
      (*i)++;
      return;
    }
    cout << "Cannot parse char " << c << " from string " << s.substr(*i) << endl;
    assert(false);
  }
  
  comment* parse_comment_with_delimiters(char sc, char ec,
					 size_t* i, const string& s) {
    string text = "";
    parse_char(sc, i, s);
    while (s[*i] != ec) {
      text += s[*i];
      (*i)++;
    }
    parse_char(ec, i, s);
    return mk_comment(sc, ec, text);
  }
  
  void ignore_comment(size_t* i, const string& s) {
    ignore_comment_with_delimiters('(', ')', i, s);
    ignore_comment_with_delimiters('[', ']', i, s);
  }

  void ignore_whitespace(size_t* i, const string& s) {
    while (*i < s.size() && isspace(s[*i])) {
      (*i)++;
    }
  }

  double parse_double(size_t* i, const string& s) {
    size_t j = *i;
    double v = stod(s.substr(*i), &j);
    *i += j;
    return v;
  }

  int parse_int(size_t* i, const string& s) {
    size_t j = *i;
    int v = stoi(s.substr(*i), &j);
    *i += j;
    return v;
  }

  double parse_coordinate(char c, size_t* i, const string& s) {
    ignore_whitespace(i, s);
    assert(s[*i] == c);
    (*i)++;
    return parse_double(i, s);
  }

  double parse_option_coordinate(char c, size_t* i, const string& s, double def=0.0) {
    ignore_whitespace(i, s);
    if (s[*i] == c) {
      (*i)++;
      return parse_double(i, s);
    }
    return def;
  }

  value* parse_option_value(char v,
			    size_t* i, const string& s) {
    ignore_whitespace(i, s);
    if (s[*i] == v) {
      parse_char(v, i, s);
      if (s[*i] == '#') {
	parse_char('#', i, s);
	int val = parse_int(i, s);
	return mk_var(val);
      } else {
	double d = parse_double(i, s);
	return mk_lit(d);
      }
    }
    return omitted::make();
  }
  
  void parse_position_values(gprog* p, size_t* i, const string& s,
			     value** x, value** y, value** z) {
    *x = parse_option_value('X', i, s);
    *y = parse_option_value('Y', i, s);
    *z = parse_option_value('Z', i, s);
  }

  instr* parse_G1(gprog* p, size_t* i, const string& s) {
    value* default_feedrate = omitted::make();
    value* fr = parse_option_value('F', i, s);
    value* xv = NULL;
    value* yv = NULL;
    value* zv = NULL;
    parse_position_values(p, i, s, &xv, &yv, &zv);
    if (*default_feedrate == *fr) {
      fr = parse_option_value('F', i, s);
    }
    return mk_G1(xv, yv, zv, fr);
  }

  instr* parse_G0(gprog* p, size_t* i, const string& s) {
    ignore_whitespace(i, s);
    value* xv = NULL;
    value* yv = NULL;
    value* zv = NULL;
    parse_position_values(p, i, s, &xv, &yv, &zv);
    return mk_G0(xv, yv, zv);
  }

  instr* parse_G2(gprog* p, size_t* i, const string& s) {
    ignore_whitespace(i, s);
    value* default_feedrate = omitted::make();
    value* fr = parse_option_value('F', i, s);    
    value* xv = NULL;
    value* yv = NULL;
    value* zv = NULL;
    parse_position_values(p, i, s, &xv, &yv, &zv);
    value* iv = parse_option_value('I', i, s);
    value* jv = parse_option_value('J', i, s);
    value* kv = parse_option_value('K', i, s);
    if (*default_feedrate == *fr) {
      fr = parse_option_value('F', i, s);
    }    
    return mk_G2(xv, yv, zv, iv, jv, kv, fr);
  }

  instr* parse_G3(gprog* p, size_t* i, const string& s) {
    ignore_whitespace(i, s);
    value* default_feedrate = omitted::make();
    value* fr = parse_option_value('F', i, s);    
    value* xv = NULL;
    value* yv = NULL;
    value* zv = NULL;
    parse_position_values(p, i, s, &xv, &yv, &zv);
    value* iv = parse_option_value('I', i, s);
    value* jv = parse_option_value('J', i, s);
    value* kv = parse_option_value('K', i, s);
    if (*default_feedrate == *fr) {
      fr = parse_option_value('F', i, s);
    }    
    return mk_G3(xv, yv, zv, iv, jv, kv, fr);
  }
  
  instr* parse_G53(gprog* p, size_t* i, const string& s) {
    value* xv = NULL;
    value* yv = NULL;
    value* zv = NULL;
    parse_position_values(p, i, s, &xv, &yv, &zv);
    return mk_G53(xv, yv, zv);
  }

  string parse_option_char(size_t* i, const string& s, char t) {
    if (s[*i] == t) {
      (*i)++;
      char str[2];
      str[0] = t;
      str[1] = '\0';
      string c(str);
      return c;
    }
    return "";
  }

  string parse_coord_letters(size_t* i, const string& s) {
    string x = parse_option_char(i, s, 'X');
    string y = parse_option_char(i, s, 'Y');
    string z = parse_option_char(i, s, 'Z');
    return x + y + z;
  }

  instr* parse_ginstr(gprog* p,
		      int val, size_t* i, const string& s) {
    instr* is;
    if (val == 0) {
      is = parse_G0(p, i, s);
    } else if (val == 1) {
      is = parse_G1(p, i, s);
    } else if (val == 91) {
      is = mk_G91();
    } else if (val == 90) {
      is = mk_G90();
    } else if (val == 53) {
      is = parse_G53(p, i, s);
    } else if (val == 2) {
      is = parse_G2(p, i, s);
    } else if (val == 3) {
      is = parse_G3(p, i, s);
    } else {
      cout << "Unrecognized instr code for instr letter: " << val << endl;
      assert(false);
    }
    ignore_whitespace(i, s);
    return is;
  }

  instr* parse_next_instr(gprog* p,
			  size_t* i,
			  const string& s) {
    instr* is;
    char next_char = s[*i];
    if (next_char == '(') {
      is = parse_comment_with_delimiters('(', ')', i, s);
      return is;
    } else if (next_char == '[') {
      is = parse_comment_with_delimiters('[', ']', i, s);
      return is;
    }
    (*i)++;
    int val = parse_int(i, s);
    if (next_char == 'M') {
      if (val == 2) {
	is = mk_m2_instr();
      } else if (val == 30) {
	is = mk_m30_instr();
      } else if (val == 5) {
	is = mk_m5_instr();
      } else if (val == 3) {
	is = mk_m3_instr();
      } else {
	cout << "M value not supported " << val << endl;
	assert(false);
      }
    } else if (next_char == 'T') {
      is = mk_t_instr(val);
    } else if (next_char == 'S') {
      is = mk_s_instr(val);
    } else if (next_char == 'F') {
      ignore_whitespace(i, s);
      string str = parse_coord_letters(i, s);
      is = mk_f_instr(val, str);
    } else if (next_char == 'G') {
      is = parse_ginstr(p, val, i, s);
    } else if (next_char == '#') {
      parse_char('=', i, s);
      double e = parse_double(i, s);
      is = mk_assign(mk_var(val), mk_lit(e));
    } else {
      cout << "Cannot parse string: " << s.substr(*i) << endl;
      assert(false);
    }
    return is;
  }
  
  gprog* parse_gprog(const string& s) {
    gprog* p = mk_gprog();
    string::size_type i = 0;
    while (i < s.size()) {
      ignore_whitespace(&i, s);
      if (i >= s.size()) { break; }
      instr* is = parse_next_instr(p, &i, s);
      p->push_back(is);
    }
    return p;
  }

  gprog* read_file(string file_name) {
    ifstream t(file_name);
    string str((istreambuf_iterator<char>(t)),
	       istreambuf_iterator<char>());
    return parse_gprog(str);
  }

}
