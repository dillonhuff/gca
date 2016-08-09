#ifndef GCA_CHECK_H
#define GCA_CHECK_H

#include <signal.h>

#define DBG_ASSERT(x) if (!(x)) { std::cout << "CHECK FAILED, EXITING..." << std::endl; abort(); }


#endif
