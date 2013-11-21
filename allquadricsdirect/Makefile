CXX := g++

CXXFLAGS := -Iinclude -Ilib/jama125 -Ilib/tnt_126 -Ilib/CLAPACK/include
CXXFLAGS_DEBUG := -Wall -g 
CXXFLAGS_RELEASE := -O3 

LDFLAGS := -L/opt/local/lib -llapack -lblas



# List of the common source files
objs=quadricfitting.o roots3and4.o
src=src/quadricfitting.cpp src/roots3and4.cpp

# Makefile rules
all: libaqd.a

libaqd.a: $(objs)
	rm -f libaqd.a
	ar rs libaqd.a $^

quadricfitting.o: src/quadricfitting.cpp src/roots3and4.cpp
	$(CXX) $(CXXFLAGS) -c $<

roots3and4.o: src/roots3and4.cpp
	$(CXX) $(CFLAGS) -c src/roots3and4.cpp

clean:
	rm -f $(objs) libaqd.a
