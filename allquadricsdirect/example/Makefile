CXX := g++

CXXFLAGS := -I../include -I../src
CXXFLAGS_DEBUG := -Wall -g 
CXXFLAGS_RELEASE := -O3 

LDFLAGS := -llapack -lblas -L../ -laqd

# List of the common source files
objs=aqd_example.o 
src=aqd_example.cpp 

# Makefile rules
all: aqd_example

aqd_example: $(objs)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

aqd_example.o: aqd_example.cpp 
	$(CXX) $(CXXFLAGS) -c aqd_example.cpp

clean:
	rm -f $(objs) aqd_example
