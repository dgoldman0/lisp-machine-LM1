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
# The library implements MegaPad's chiplet module interface, and builds
# against MegaPad's own header for it, shared/accel/chiplet_module.h in the
# workspace's MegaPad checkout (decision C of design/chiplet-module.md).
# MEGAPAD_ROOT names that checkout, as the link test bench names it, and
# defaults to the checkout beside this repository's.
#
# The library carries the digest of the files it was built from: the
# SHA-256 of what `sha256sum` prints for this Makefile, the export list,
# and the headers and sources sorted together, in that order, followed by
# what it prints for MegaPad's header, named by its path inside MegaPad.
# LM-1's Python binding (emu/lm1/chiplet.py) computes the same digest from
# the same files and builds the library again when they differ, so no test
# runs against a library older than its sources or the header.  The two
# lists of files must stay alike.

CXX ?= g++
MEGAPAD_ROOT ?= $(abspath $(CURDIR)/../../megapad)
MP64_HEADER_NAME := shared/accel/chiplet_module.h
MP64_HEADER := $(MEGAPAD_ROOT)/$(MP64_HEADER_NAME)

LM1C_CXXFLAGS := -std=c++17 -O2 -march=x86-64-v2 -fPIC -fvisibility=hidden \
                 -Wall -Wextra -Iinclude -I$(MEGAPAD_ROOT)/shared/accel
LM1C_LDFLAGS := -shared -Wl,--version-script=exports.map

SOURCES := $(sort $(wildcard src/*.cpp))
HEADERS := $(sort $(wildcard include/*.h src/*.h))
DIGESTED := Makefile exports.map $(sort $(HEADERS) $(SOURCES))
# Expanded when the library is built, once MegaPad's header is known to be
# there.
DIGEST = $(shell (sha256sum $(DIGESTED) && cd $(MEGAPAD_ROOT) && \
                  sha256sum $(MP64_HEADER_NAME)) | sha256sum | cut -c1-64)

LIBRARY := build/lm1_chiplet.so

.PHONY: all clean
all: $(LIBRARY)

# One compiler run over every source, written beside the library and then
# moved into place, so a reader never finds half a library.
$(LIBRARY): $(DIGESTED) $(MP64_HEADER)
	@mkdir -p build
	$(CXX) $(LM1C_CXXFLAGS) -DLM1C_SOURCE_DIGEST='"$(DIGEST)"' $(SOURCES) \
	    $(LM1C_LDFLAGS) -o $@.tmp
	mv -f $@.tmp $@

$(MP64_HEADER):
	@echo "The chiplet module builds against MegaPad's $(MP64_HEADER_NAME)," >&2
	@echo "which $(MEGAPAD_ROOT) lacks; set MEGAPAD_ROOT to a MegaPad checkout." >&2
	@exit 1

clean:
	rm -rf build
