#ifndef GCA_LEXER_H
#define GCA_LEXER_H

#include <vector>

#include "core/value.h"

using namespace std;

namespace gca {

  enum token_type {
    COMMENT = 0,
    ICODE
  };

  struct token {
    token_type ttp;
    string text;
    char c;
    const value& v;

    token(string textp) : ttp(COMMENT), text(textp), v(*omitted::make()) {}
    token(char cp, const value& vp) : ttp(ICODE), c(cp), v(vp) {}
    
    bool operator==(const token& other) const {
      if (ttp != other.ttp) { return false; }
      return ((ttp == COMMENT) && (text == other.text)) ||
	((ttp == ICODE) && ((c == other.c) && (v == other.v)));
    }
    
    token_type tp() const { return ttp; }
    
    void print(ostream& stream) const {
      if (ttp == COMMENT) {
	stream << text;
      } else {
	stream << c << v;
      }
    }
  };
  
  struct cmt : public token {
    cmt(string textp) : token(textp) {}

    static cmt* make(string textp)
    { return new (allocate<cmt>()) cmt(textp); }
  };
  
  struct icode : public token {

    icode(char cp, const value& vp) : token(cp, vp) {}

    static icode* make(char c, const value& v)
    { return new (allocate<icode>()) icode(c, v); }
    static icode* make(char c, double v)
    { return new (allocate<icode>()) icode(c, *lit::make(v)); }
    static icode* make(char c, int v)
    { return new (allocate<icode>()) icode(c, *ilit::make(v)); }
  };

  typedef vector<token*> block;

  bool cmp_tokens(const token* l, const token* r);
  
  ostream& operator<<(ostream& stream, const token& ic);
  ostream& operator<<(ostream& stream, const block& block);
  ostream& operator<<(ostream& stream, const vector<block>& blocks);

  vector<block> lex_gprog(const string& s);

  bool operator==(const vector<block>& l, const vector<block>& r);

  struct cmp_token_to {
    const token* t;
    cmp_token_to(const token* tp) : t(tp) {}
    bool operator()(const token* l) { return (*l) == (*t); }
  };
  
}

#endif
