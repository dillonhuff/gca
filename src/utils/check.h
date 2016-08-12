#ifndef GCA_CHECK_H
#define GCA_CHECK_H

#include <signal.h>

#define DBG_ASSERT(x) if (!(x)) { std::cout << "CHECK AT " << __FILE__ << ": " << __LINE__ << ", EXITING..." << std::endl; abort(); }


#endif
