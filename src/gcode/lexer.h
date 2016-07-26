#ifndef GCA_LEXER_H
#define GCA_LEXER_H

#include <vector>

#include "gcode/value.h"

using namespace std;

namespace gca {

  enum orientation {
    GCA_ABSOLUTE = 0,
    GCA_RELATIVE = 1,
    GCA_NONE
  };
  
  enum token_type {
    COMMENT = 0,
    ICODE
  };

  struct token {
    token_type ttp;
    string text;
    char c;
    value* v;
    int line_no;

    token(const token& x) :
      ttp(x.ttp), text(x.text), c(x.c), v(x.v), line_no(x.line_no) {}
    token(string textp) : ttp(COMMENT), text(textp), line_no(-1) {}
    token(char cp, value* vp) : ttp(ICODE), c(cp), v(vp), line_no(-1) {}
    token(char cp, int vp) : ttp(ICODE), c(cp), v(ilit::make(vp)), line_no(-1) {}
    token(char cp, double vp) : ttp(ICODE), c(cp), v(lit::make(vp)), line_no(-1) {}

    token& operator=(const token& x) {
      ttp = x.ttp;
      text = x.text;
      c = x.c;
      v = x.v;
      line_no = x.line_no;
      return *this;
    }

    bool operator==(const token& other) const {
      if (ttp != other.ttp) { return false; }
      return ((ttp == COMMENT) && (text == other.text)) ||
	((ttp == ICODE) && ((c == other.c) && (*v == *(other.v))));
    }
    
    token_type tp() const { return ttp; }
    
    void print(ostream& stream) const {
      if (ttp == COMMENT) {
	stream << "(*** " << text << " ***)";
      } else {
	stream << c << *v;
      }
    }
  };
  
  typedef vector<token> block;

  bool cmp_tokens(const token* l, const token* r);
  
  ostream& operator<<(ostream& stream, const token& ic);
  ostream& operator<<(ostream& stream, const block& block);
  ostream& operator<<(ostream& stream, const vector<block>& blocks);

  vector<block> lex_gprog(const string& s);
  vector<block> lex_file(const string& file_path);

  bool operator==(const vector<block>& l, const vector<block>& r);

  struct cmp_token_to {
    const token* t;
    cmp_token_to(const token* tp) : t(tp) {}
    bool operator()(const token& l) { return l == (*t); }
  };
  
}

#endif
