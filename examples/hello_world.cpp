#include "core/context.h"
#include "core/parser.h"

using namespace gca;

int main(int argc, char** argv) {
  if (argc != 2) {
    cout << "Usage: gdiff <gcode_file_path>" << endl;
    return 0;
  }
  string file = argv[1];
  
  gprog* p = read_file(file);
  cout << *p;
  return 0;
}
