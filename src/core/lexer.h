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
    virtual bool operator==(const token& other) const = 0;
    virtual token_type tp() const = 0;
    virtual void print(ostream& stream) const = 0;
  };
  
  struct cmt : public token {
    string text;

    cmt(string textp) : text(textp) {}

    static cmt* make(string textp)
    { return new (allocate<cmt>()) cmt(textp); }

    bool operator==(const token& other) const {
      if (other.tp() != COMMENT)
	{return false; }
      const cmt& ci = static_cast<const cmt&>(other);
      return text == ci.text;
    }
    token_type tp() const { return COMMENT; }
    void print(ostream& stream) const
    { stream << text; }
  };
  
  struct icode : public token {
    char c;
    const value& v;

    icode(char cp, const value& vp) : c(cp), v(vp) {}

    static icode* make(char c, const value& v)
    { return new (allocate<icode>()) icode(c, v); }
    static icode* make(char c, double v)
    { return new (allocate<icode>()) icode(c, *lit::make(v)); }
    static icode* make(char c, int v)
    { return new (allocate<icode>()) icode(c, *ilit::make(v)); }
    
    bool operator==(const token& other) const {
      if (other.tp() != ICODE)
	{return false; }
      const icode& ci = static_cast<const icode&>(other);
      return c == ci.c && v == ci.v;
    }
    token_type tp() const { return ICODE; }
    void print(ostream& stream) const
    { stream << c << v; }
  };

  typedef token* tok;
  typedef vector<tok> block;

  bool cmp_tokens(const tok l, const tok r);
  
  ostream& operator<<(ostream& stream, const token& ic);
  ostream& operator<<(ostream& stream, const block& block);
  ostream& operator<<(ostream& stream, const vector<block>& blocks);

  vector<block> lex_gprog(const string& s);

  bool operator==(const vector<block>& l, const vector<block>& r);

  struct cmp_token_to {
    tok t;
    cmp_token_to(tok tp) : t(tp) {}
    bool operator()(tok l) { return (*l) == (*t); }
  };
  
}

#endif
