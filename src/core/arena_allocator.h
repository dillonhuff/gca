#ifndef GCA_ARENA_ALLOCATOR_H
#define GCA_ARENA_ALLOCATOR_H

#include <cassert>
#include <vector>

using namespace std;

#define DEFAULT_ARENA_SIZE 10000000

namespace gca {
  
  class arena_allocator {
  protected:
    char* start;
    char* current;
    size_t size;
    size_t space_left;
    
  public:
    arena_allocator() {
      size = DEFAULT_ARENA_SIZE;
      space_left = size;
      start = static_cast<char*>(malloc(size));
      current = start;
    }

    ~arena_allocator() {
      delete start;
    }

    template<typename T>
    T* allocate() {
      size_t s = sizeof(T);
      space_left = space_left - s;
      assert(space_left > 0);
      void* to_alloc = current;
      current += s;
      return static_cast<T*>(to_alloc);
    }

  };
 
}

#endif
