#ifndef GCA_COMMENT_H
#define GCA_COMMENT_H

#include "core/instrs/instr.h"

namespace gca {

  class comment : public instr {
  public:
    char ld, rd;
    string text;
  comment(char ldp, char rdp, string c) : ld(ldp), rd(rdp), text(c) {}

    static comment* make(char ld, char rd, string t) {
      comment* mem = allocate<comment>();
      return new (mem) comment(ld, rd, t);
    }

    inline bool is_comment() const { return true; }

    virtual bool operator==(const instr& i) const {
      if (i.is_comment()) {
	const comment& ci = static_cast<const comment&>(i);
	return ld == ci.ld && rd == ci.rd && text == ci.text;
      }
      return false;
    }

    virtual void print(ostream& s) const {
      s << ld << text << rd;
    }

  };
}

#endif
