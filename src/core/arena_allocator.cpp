#include <cstdlib>

#include "core/arena_allocator.h"

namespace gca {
  arena_allocator* system_allocator = NULL;

  void set_system_allocator(arena_allocator* a) {
    system_allocator = a;
  }

  void* alloc(size_t s) {
    assert(system_allocator != NULL);
    void* to_alloc = system_allocator->alloc(s);
    return to_alloc;
  }
  
};
