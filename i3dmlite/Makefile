TARGET := i3dm

SRCDIR := src
SOURCES := $(wildcard $(SRCDIR)/*.cpp)
BUILDDIR := .
OBJDIR := $(BUILDDIR)/src
OBJECTS := $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o, $(SOURCES))


#Get the current OS
UNAME := $(shell uname -s)

CXX := g++

CXXFLAGS := -Iinclude -Ilib/jama125 -Ilib/tnt_126 -Ilib/CLAPACK/include -Ilib/AntTweakBar/include
CXXFLAGS_DEBUG := -Wall -g 
CXXFLAGS_RELEASE := -O3 

LDFLAGS := -llapack -lblas -Lglfw-2.6.bin -lglfw -framework Cocoa -framework IOKit -framework OpenGL -Llib/AntTweakBar/lib -lAntTweakBar

#-----------------------------------------


$(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@mkdir -p $(BUILDDIR)
	@mkdir -p $(OBJDIR)
	@echo "Compiling .cpp to .o for $(PLATFORM) Platform:  $<" 
	@$(CXX) -c -Wall $(CXXFLAGS) $< -o $@

$(TARGET): $(OBJECTS)
	@echo "Linking .o files into:  $(TARGET)"
	@$(CXX) $(LDFLAGS) $(OBJECTS) -o $(BUILDDIR)/$(TARGET)
	@echo "Build success!"
	
default: $(TARGET)
	
all: default

clean:
	@echo "Removing objects" 
	@rm src/*.o

depend:
	makedepend -- $(CXXFLAGS) $(CFLAGS) -- $(SOURCES)


# Don't place anything below this line, since
# the make depend program will overwrite it
# DO NOT DELETE THIS LINE -- make depend depends on it.

src/glutils.o: src/glutils.h include/algebra3.h /usr/include/stdlib.h
src/glutils.o: /usr/include/Availability.h
src/glutils.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
src/glutils.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
src/glutils.o: /usr/include/sys/_symbol_aliasing.h
src/glutils.o: /usr/include/sys/_posix_availability.h
src/glutils.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
src/glutils.o: /usr/include/sys/wait.h /usr/include/sys/signal.h
src/glutils.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
src/glutils.o: /usr/include/i386/signal.h /usr/include/i386/_structs.h
src/glutils.o: /usr/include/sys/_structs.h /usr/include/machine/_structs.h
src/glutils.o: /usr/include/sys/resource.h /usr/include/machine/endian.h
src/glutils.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
src/glutils.o: /usr/include/libkern/_OSByteOrder.h
src/glutils.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
src/glutils.o: /usr/include/machine/types.h /usr/include/i386/types.h
src/glutils.o: /usr/include/assert.h /usr/include/math.h include/GL/glfw.h
src/kinematicfieldfit.o: src/kinematicfieldfit.h include/algebra3.h
src/kinematicfieldfit.o: /usr/include/stdlib.h /usr/include/Availability.h
src/kinematicfieldfit.o: /usr/include/AvailabilityInternal.h
src/kinematicfieldfit.o: /usr/include/_types.h /usr/include/sys/_types.h
src/kinematicfieldfit.o: /usr/include/sys/cdefs.h
src/kinematicfieldfit.o: /usr/include/sys/_symbol_aliasing.h
src/kinematicfieldfit.o: /usr/include/sys/_posix_availability.h
src/kinematicfieldfit.o: /usr/include/machine/_types.h
src/kinematicfieldfit.o: /usr/include/i386/_types.h /usr/include/sys/wait.h
src/kinematicfieldfit.o: /usr/include/sys/signal.h
src/kinematicfieldfit.o: /usr/include/sys/appleapiopts.h
src/kinematicfieldfit.o: /usr/include/machine/signal.h
src/kinematicfieldfit.o: /usr/include/i386/signal.h
src/kinematicfieldfit.o: /usr/include/i386/_structs.h
src/kinematicfieldfit.o: /usr/include/sys/_structs.h
src/kinematicfieldfit.o: /usr/include/machine/_structs.h
src/kinematicfieldfit.o: /usr/include/sys/resource.h
src/kinematicfieldfit.o: /usr/include/machine/endian.h
src/kinematicfieldfit.o: /usr/include/i386/endian.h
src/kinematicfieldfit.o: /usr/include/sys/_endian.h
src/kinematicfieldfit.o: /usr/include/libkern/_OSByteOrder.h
src/kinematicfieldfit.o: /usr/include/libkern/i386/_OSByteOrder.h
src/kinematicfieldfit.o: /usr/include/alloca.h /usr/include/machine/types.h
src/kinematicfieldfit.o: /usr/include/i386/types.h /usr/include/assert.h
src/kinematicfieldfit.o: /usr/include/math.h src/mesh.h src/glutils.h
src/kinematicfieldfit.o: include/GL/glfw.h lib/CLAPACK/include/f2c.h
src/kinematicfieldfit.o: lib/CLAPACK/include/blaswrap.h
src/kinematicfieldfit.o: lib/CLAPACK/include/clapack.h
src/main.o: src/editorstate.h src/view.h src/glutils.h include/algebra3.h
src/main.o: /usr/include/stdlib.h /usr/include/Availability.h
src/main.o: /usr/include/AvailabilityInternal.h /usr/include/_types.h
src/main.o: /usr/include/sys/_types.h /usr/include/sys/cdefs.h
src/main.o: /usr/include/sys/_symbol_aliasing.h
src/main.o: /usr/include/sys/_posix_availability.h
src/main.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
src/main.o: /usr/include/sys/wait.h /usr/include/sys/signal.h
src/main.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
src/main.o: /usr/include/i386/signal.h /usr/include/i386/_structs.h
src/main.o: /usr/include/sys/_structs.h /usr/include/machine/_structs.h
src/main.o: /usr/include/sys/resource.h /usr/include/machine/endian.h
src/main.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
src/main.o: /usr/include/libkern/_OSByteOrder.h
src/main.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
src/main.o: /usr/include/machine/types.h /usr/include/i386/types.h
src/main.o: /usr/include/assert.h /usr/include/math.h include/GL/glfw.h
src/main.o: src/quadricfitting.h src/mesh.h src/kffProfileEditor.h
src/main.o: src/kinematicfieldfit.h lib/AntTweakBar/include/AntTweakBar.h
src/main.o: /usr/include/stddef.h
src/mesh.o: src/mesh.h include/algebra3.h /usr/include/stdlib.h
src/mesh.o: /usr/include/Availability.h /usr/include/AvailabilityInternal.h
src/mesh.o: /usr/include/_types.h /usr/include/sys/_types.h
src/mesh.o: /usr/include/sys/cdefs.h /usr/include/sys/_symbol_aliasing.h
src/mesh.o: /usr/include/sys/_posix_availability.h
src/mesh.o: /usr/include/machine/_types.h /usr/include/i386/_types.h
src/mesh.o: /usr/include/sys/wait.h /usr/include/sys/signal.h
src/mesh.o: /usr/include/sys/appleapiopts.h /usr/include/machine/signal.h
src/mesh.o: /usr/include/i386/signal.h /usr/include/i386/_structs.h
src/mesh.o: /usr/include/sys/_structs.h /usr/include/machine/_structs.h
src/mesh.o: /usr/include/sys/resource.h /usr/include/machine/endian.h
src/mesh.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
src/mesh.o: /usr/include/libkern/_OSByteOrder.h
src/mesh.o: /usr/include/libkern/i386/_OSByteOrder.h /usr/include/alloca.h
src/mesh.o: /usr/include/machine/types.h /usr/include/i386/types.h
src/mesh.o: /usr/include/assert.h /usr/include/math.h src/glutils.h
src/mesh.o: include/GL/glfw.h
src/quadricfitting.o: src/quadricfitting.h include/algebra3.h
src/quadricfitting.o: /usr/include/stdlib.h /usr/include/Availability.h
src/quadricfitting.o: /usr/include/AvailabilityInternal.h
src/quadricfitting.o: /usr/include/_types.h /usr/include/sys/_types.h
src/quadricfitting.o: /usr/include/sys/cdefs.h
src/quadricfitting.o: /usr/include/sys/_symbol_aliasing.h
src/quadricfitting.o: /usr/include/sys/_posix_availability.h
src/quadricfitting.o: /usr/include/machine/_types.h
src/quadricfitting.o: /usr/include/i386/_types.h /usr/include/sys/wait.h
src/quadricfitting.o: /usr/include/sys/signal.h
src/quadricfitting.o: /usr/include/sys/appleapiopts.h
src/quadricfitting.o: /usr/include/machine/signal.h
src/quadricfitting.o: /usr/include/i386/signal.h /usr/include/i386/_structs.h
src/quadricfitting.o: /usr/include/sys/_structs.h
src/quadricfitting.o: /usr/include/machine/_structs.h
src/quadricfitting.o: /usr/include/sys/resource.h
src/quadricfitting.o: /usr/include/machine/endian.h
src/quadricfitting.o: /usr/include/i386/endian.h /usr/include/sys/_endian.h
src/quadricfitting.o: /usr/include/libkern/_OSByteOrder.h
src/quadricfitting.o: /usr/include/libkern/i386/_OSByteOrder.h
src/quadricfitting.o: /usr/include/alloca.h /usr/include/machine/types.h
src/quadricfitting.o: /usr/include/i386/types.h /usr/include/assert.h
src/quadricfitting.o: /usr/include/math.h src/mesh.h src/glutils.h
src/quadricfitting.o: include/GL/glfw.h lib/jama125/jama_eig.h
src/quadricfitting.o: lib/tnt_126/tnt_array1d.h lib/tnt_126/tnt_i_refvec.h
src/quadricfitting.o: lib/tnt_126/tnt_array2d.h lib/tnt_126/tnt_math_utils.h
src/quadricfitting.o: lib/jama125/jama_svd.h lib/tnt_126/tnt_array1d_utils.h
src/quadricfitting.o: lib/tnt_126/tnt_array2d_utils.h
src/quadricfitting.o: lib/CLAPACK/include/f2c.h
src/quadricfitting.o: lib/CLAPACK/include/blaswrap.h
src/quadricfitting.o: lib/CLAPACK/include/clapack.h src/roots3and4.h
src/roots3and4.o: /usr/include/math.h /usr/include/sys/cdefs.h
src/roots3and4.o: /usr/include/sys/_symbol_aliasing.h
src/roots3and4.o: /usr/include/sys/_posix_availability.h
src/roots3and4.o: /usr/include/Availability.h
src/roots3and4.o: /usr/include/AvailabilityInternal.h
