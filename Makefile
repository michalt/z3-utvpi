CC=g++
CXXFLAGS=-std=c++0x
#-Wall -Weffc++
# LIBS=-lgmpxx -lgmp
LIBS=-lz3 -fopenmp -lgmpxx -lgmp
Z3_INCLUDE=/home/m/software/z3/include/
Z3_LIB=/home/m/software/z3/lib/

all: test

test: *.h test.cc
	$(CC) $(CXXFLAGS) -I$(Z3_INCLUDE) -L$(Z3_LIB) $(LIBS) -o $@ $@.cc

test2: *.h test2.cc
	$(CC) $(CXXFLAGS) -I$(Z3_INCLUDE) -L$(Z3_LIB) $(LIBS) -o $@ $@.cc

test2-clang: *.h test2.cc
	clang++ $(CXXFLAGS) -I$(Z3_INCLUDE) -c -o $@ test2.cc
	g++ $@.o -L$(Z3_LIB) $(LIBS) -o $@

test-clang: common.h utvpi_graph.h utvpi_graph-inl.h test.cc
	clang++ $(CXXFLAGS) -I$(Z3_INCLUDE) -c -o $@.o test.cc
	g++ $@.o -L$(Z3_LIB) $(LIBS) -o $@

# utvpi: inequality_graph.o utvpi.o
# $(CC) $(CXXFLAGS) -I$(Z3_INCLUDE) -L$(Z3_LIB) $(LIBS) -o $@ $+

# utvpi.o: utvpi.cc
# $(CC) -c $(CXXFLAGS) -I$(Z3_INCLUDE) -L$(Z3_LIB) -o $@ utvpi.cc

# inequality_graph.o: inequality_graph.h inequality_graph.cc
# $(CC) -c $(CXXFLAGS) -I$(Z3_INCLUDE) -L$(Z3_LIB) -o $@ inequality_graph.cc

clean:
	-rm -f inequality_graph.o utvpi
