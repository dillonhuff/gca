#include <algorithm>
#include <fstream>
#include <sstream>
#include <streambuf>

#include "core/lexer.h"
#include "core/parse_stream.h"

namespace gca {

  ostream& operator<<(ostream& stream, const token& ic) {
    ic.print(stream);
    return stream;
  }

  ostream& operator<<(ostream& stream, const block& block) {
    for (block::const_iterator it = block.begin(); it != block.end(); ++it)
      { stream << **it << " "; }
    return stream;
  }

  ostream& operator<<(ostream& stream, const vector<block>& blocks) {
    for (vector<block>::const_iterator it = blocks.begin(); it != blocks.end(); ++it)
      { stream << *it << endl; }
    return stream;
  }
  
  int parse_i(parse_state& s) {
    string digits = "";
    while (s.chars_left() && isdigit(s.next())) {
      digits += s.next();
      s++;
    }
    return stoi(digits);
  }

  double parse_dbl(parse_state& s) {
    string text = "";
    if (s.next() == '-') { text += s.next(); s++; }
    while (s.chars_left() && isdigit(s.next())) { text += s.next(); s++; }
    if (s.next() == '.') { text += s.next(); s++; }
    while (s.chars_left() && isdigit(s.next())) { text += s.next(); s++; }
    return atof(text.c_str());
  }
  
  value* parse_c_val(char c, parse_state& s) {
    switch(c) {
    case 'X':
    case 'Y':
    case 'Z':
    case 'I':
    case 'J':
    case 'K':
    case 'F':
    case 'H':
    case 'R':
    case 'Q':
    case 'x':
    case 'y':
    case 'z':
    case 'i':
    case 'j':
    case 'k':
    case 'f':
    case 'h':
    case 'r':
    case 'q':
      return lit::make(parse_dbl(s));
    case 'G':
    case 'M':
    case 'N':
    case 'O':
    case 'T':
    case 'S':
    case 'P':
    case 'D':
    case 'L':
    case 'g':
    case 'm':
    case 'n':
    case 'o':
    case 't':
    case 's':
    case 'p':
    case 'd':
    case 'l':
      return ilit::make(parse_i(s));
    default:
      cout << "Invalid c = " << c << endl;
      cout << "Inavlid c as int = " << ((int) c) << endl;
      cout << "Is EOF? " << (((int) c) == EOF) << endl;
      assert(false);
    }
  }

  token* parse_token(parse_state& s) {
    if (s.next() == '[') {
      string cs = parse_comment_with_delimiters('[', ']', s);
      return cmt::make(cs);
    } else if (s.next() == '(') {
      string cs = parse_comment_with_delimiters('(', ')', s);
      return cmt::make(cs);
    } else {
      char c = s.next();
      s++;
      value* v = parse_c_val(c, s);
      return new (allocate<icode>()) icode(c, *v);
    }
  }

  block lex_gprog_line(const string& str) {
    block ts;
    parse_state s(str);
    int i = 0;
    while (s.chars_left()) {
      ignore_whitespace(s);
      if (!s.chars_left()) { break; }
      ts.push_back(parse_token(s));
      i++;
    }
    return ts;
  }

  vector<block> lex_gprog(const string& str) {
    vector<block> blocks;
    string::const_iterator line_start = str.begin();
    string::const_iterator line_end;
    while (line_start < str.end()) {
      line_end = find(line_start, str.end(), '\n');
      string line(line_start, line_end);
      if (line.size() > 0) { blocks.push_back(lex_gprog_line(line)); }
      line_start += line.size() + 1;
    }
    return blocks;
  }

  bool cmp_tokens(const token* l, const token* r)
  { return (*l) == (*r); }
  
  bool operator==(const block& l, const block& r) {
    if (l.size() != r.size())
      { return false; }
    return equal(l.begin(), l.end(), r.begin(), cmp_tokens);
  }

  bool operator==(const vector<block>& l, const vector<block>& r) {
    if (l.size() != r.size())
      { return false; }
    return equal(l.begin(), l.end(), r.begin());
  }
}
