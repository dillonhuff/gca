#ifndef GCA_PARSER_H
#define GCA_PARSER_H

#include <string>

#include "core/context.h"
#include "core/gprog.h"

using namespace std;

namespace gca {

  gprog* parse_gprog(const string& s);
  gprog* read_file(string file_name);
  
}

#endif
