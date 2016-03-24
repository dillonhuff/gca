#include "system/file.h"

namespace gca {

  bool ends_with(string const& value, string const& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
  }

}
