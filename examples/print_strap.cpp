#include <iostream>

#include "core/parser.h"

using namespace gca;

int main() {
  
  string fn = "/Users/dillon/CppWorkspace/tenths/external-examples/straps/straight-strap-4x-width=.5.tap";
  gprog* p = read_file(fn);
  cout << *p;
  return 0;
}
