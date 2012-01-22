#
# Requires that either Z3 include/lib is int the PATH or Z3_DIR must be set
#
# -DVERBOSE to get some more output
# -DDEBUG for debugging info
#

CC=g++
CXXFLAGS=-std=c++0x -Wall -DVERBOSE
LIBS=-lz3 -fopenmp -lgmpxx -lgmp
Z3_INCLUDE=$(Z3_DIR)/include/
Z3_LIB=$(Z3_DIR)/lib/

all: test

test: *.h test.cc
	$(CC) $(CXXFLAGS) -I$(Z3_INCLUDE) -L$(Z3_LIB) $(LIBS) -o $@ $@.cc

clean:
	-rm -f test
