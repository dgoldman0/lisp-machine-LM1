# LM-1 chiplet module: the C++ model of LM-1 as one shared library.
#
#   make          build build/lm1_chiplet.so
#   make clean    remove build/
#
# The library is built for the release's baseline, x86-64-v2, so that it
# runs on every machine the release supports (decision 23 of the release
# plan), and with fixed flags, so that the same sources give the same
# library on every machine.
#
# The library carries the digest of the files it was built from: the
# SHA-256 of `sha256sum` over this Makefile, the export list, and the
# headers and sources sorted together, in that order.  LM-1's Python
# binding (emu/lm1/chiplet.py) computes the same digest from the same files
# and builds the library again when they differ, so no test runs against a
# library older than its sources.  The two lists of files must stay alike.

CXX ?= g++
LM1C_CXXFLAGS := -std=c++17 -O2 -march=x86-64-v2 -fPIC -fvisibility=hidden \
                 -Wall -Wextra -Iinclude
LM1C_LDFLAGS := -shared -Wl,--version-script=exports.map

SOURCES := $(sort $(wildcard src/*.cpp))
HEADERS := $(sort $(wildcard include/*.h src/*.h))
DIGESTED := Makefile exports.map $(sort $(HEADERS) $(SOURCES))
DIGEST := $(shell sha256sum $(DIGESTED) | sha256sum | cut -c1-64)

LIBRARY := build/lm1_chiplet.so

.PHONY: all clean
all: $(LIBRARY)

# One compiler run over every source, written beside the library and then
# moved into place, so a reader never finds half a library.
$(LIBRARY): $(DIGESTED)
	@mkdir -p build
	$(CXX) $(LM1C_CXXFLAGS) -DLM1C_SOURCE_DIGEST='"$(DIGEST)"' $(SOURCES) \
	    $(LM1C_LDFLAGS) -o $@.tmp
	mv -f $@.tmp $@

clean:
	rm -rf build
