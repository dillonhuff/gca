#!/bin/sh
filename=$1
filename=${filename%.*}
BUILD_RULE=$(g++ -I./src -I./test -MM -MG -MT "${filename}.o" $1)
echo "${BUILD_RULE}"
echo -e "\t\$(CC) \$(CXX_FLAGS) -c \$< -o \$@"
