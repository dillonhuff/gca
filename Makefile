CC := g++
CXX_FLAGS := -std=c++98 -pedantic -I./src -I./test -Wall -Werror -g

SRC_FILES := $(shell find ./src -name "*.cpp")
TEST_FILES := $(shell find ./test -name "*.cpp")
SRC_HEADER_FILES := $(shell find ./src -name "*.h")

ALL_SRC := $(SRC_FILES)
ALL_SRC += $(TEST_FILES)

ALL_FILES := $(ALL_SRC)
ALL_FILES += $(SRC_HEADER_FILES)

SRC_OBJS := $(SRC_FILES:.cpp=.o)
OBJS := $(ALL_SRC:.cpp=.o)

######## Example files ######################
CHECK_BOUNDS := examples/check_bounds.cpp
CHECK_BOUNDS_OBJ := $(CHECK_BOUNDS:.cpp=.o)

SIMPLE_PROG := examples/simple_prog.cpp
SIMPLE_PROG_OBJ := $(SIMPLE_PROG:.cpp=.o)

#############################################

 %.o : %.cpp $(SRC_HEADER_FILES)
	$(CC) $(CXX_FLAGS) -c $< -o $@

all-tests: $(SRC_HEADER_FILES) $(OBJS)
	$(CC) $(CXX_FLAGS) $(OBJS) -o $@

check-bounds-example: $(SRC_HEADER_FILES) $(SRC_OBJS) $(CHECK_BOUNDS_OBJ)
	$(CC) $(CXX_FLAGS) $(SRC_OBJS) $(CHECK_BOUNDS_OBJ) -o $@

simple-prog: $(SRC_HEADER_FILES) $(SRC_OBJS) $(SIMPLE_PROG_OBJ)
	$(CC) $(CXX_FLAGS) $(SRC_OBJS) $(SIMPLE_PROG_OBJ) -o $@

clean:
	find . -name "*.o" -type f -delete
	rm -f all-tests
	rm -f check-bounds-example
