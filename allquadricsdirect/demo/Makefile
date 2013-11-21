CXX := g++

CXXFLAGS := -I../include -I../src -Iglfw-2.6.bin
CXXFLAGS_DEBUG := -Wall -g 
CXXFLAGS_RELEASE := -O3 

LDFLAGS := -llapack -lblas -L../ -Lglfw-2.6.bin -laqd -lglfw -framework Cocoa -framework IOKit -framework OpenGL

# List of the common source files
objs=main.o glutils.o
src=main.cpp glutils.cpp

# Makefile rules
all: aqd_demo

aqd_demo: $(objs)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LDFLAGS)

glutils.o: src/glutils.cpp
	$(CXX) $(CXXFLAGS) -c $^

main.o: src/main.cpp src/glutils.cpp
	$(CXX) $(CXXFLAGS) -c src/main.cpp

clean:
	rm -f $(objs) aqd_demo
