CC := g++
CXX_FLAGS := -std=c++98 -pedantic -I. -Wall -Werror -g

SRC_FILES := $(shell find . -name "*.cpp")
HEADER_FILES := $(shell find . -name "*.h")

ALL_FILES := $(SRC_FILES)
ALL_FILES += $(HEADER_FILES)

OBJS := $(SRC_FILES:.cpp=.o)

 %.o : %.cpp
	$(CC) $(CXX_FLAGS) -c $< -o $@

all-tests: $(HEADER_FILES) $(OBJS)
	$(CC) $(CXX_FLAGS) $(OBJS) -o $@

clean:
	find . -name "*.o" -type f -delete
	rm -f all-tests
