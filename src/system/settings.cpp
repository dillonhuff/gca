#include "system/settings.h"

namespace gca {

#ifdef __APPLE__
  string project_path = "/Users/dillon/CppWorkspace/";
#elif __linux__
  string project_path = "/home/probotix/CppWorkspace/";
#else
# error unknown compiler
#endif

}
