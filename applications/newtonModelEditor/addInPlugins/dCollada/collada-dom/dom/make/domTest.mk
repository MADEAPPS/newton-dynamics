include make/common.mk

src := $(wildcard test/$(colladaVersion)/*.cpp)
targets := $(outPath)domTest$(exeSuffix)

# DOM defs. This is extra complicated because of the installTest make target. The extra
# complexity is justified since installTest is very useful.
ifneq ($(os),mac)
libSuffix := $(if $(findstring windows,$(os)),.lib,$(if $(findstring ps3,$(os)),.a,.so))
domPath := $(if $(installTest),$(installPrefix)/lib/,$(outPath))
domVersionTag := $(if $(findstring windows,$(os)),$(domVersionNoDots),)
domName := $(domPath)libcollada$(colladaVersionNoDots)dom$(domVersionTag)$(debugSuffix)$(libSuffix)
ifeq ($(os),linux)
# Note: Linking in this way on Linux is necessary for the shared lib loader to
# find the DOM .so when you're not running from the top-level DOM path
# (accomplished via the -Wl,-rpath linker option.
libOpts += -L$(domPath) -lcollada$(colladaVersionNoDots)dom$(debugSuffix)
sharedLibSearchPaths += $(abspath $(outPath))
else
libOpts += $(domName)
endif
else ifeq ($(os),mac)
domPath := $(if $(installTest),$(installPrefix)/,$(outPath))
domFramework := Collada$(colladaVersionNoDots)Dom$(debugSuffix).framework
domName := $(domPath)$(domFramework)
# On Mac we use the framework for linking. If we're doing an install test then
# use the installed framework, which we'll assume is a system framework path.
ifeq ($(installTest),)
libOpts += -F$(outPath)
endif
libOpts += -framework $(notdir $(basename $(domName)))
endif

ifeq ($(installTest),)
includeOpts += -Iinclude -Iinclude/$(colladaVersion)
else ifeq ($(os),linux)
includeOpts += -I$(installPrefix)/include/colladadom -I$(installPrefix)/include/colladadom/$(colladaVersion)
else ifeq ($(os),mac)
includeOpts += -I$(installPrefix)/$(domFramework)/Headers
endif
dependentLibs += $(domName)

# PCRE defs
ifeq ($(os),ps3)
libOpts += $(addprefix external-libs/pcre/lib/$(os)/,libpcrecpp.a libpcre.a)
endif

# TinyXml defs
ifneq ($(findstring tinyxml,$(xmlparsers)),)
# Notify domTest.cpp if we're supposed to do TinyXml tests
ccFlags += -DTINYXML
ifeq ($(os),ps3)
libOpts += external-libs/tinyxml/lib/$(os)/libtinyxml.a
endif
endif

# Boost defs
ifeq ($(os),linux)
libOpts += -lboost_filesystem
else
includeOpts += -Iexternal-libs/boost
libOpts += external-libs/boost/lib/$(buildID)/libboost_system.a
libOpts += external-libs/boost/lib/$(buildID)/libboost_filesystem.a
endif
ifeq ($(os),ps3)
# PS3 doesn't support C++ locales, so tell boost not to use them
ccFlags += -DBOOST_NO_STD_LOCALE
endif

ifeq ($(os),ps3)
# On PS3 we need to make a .self from the .elf
postCreateExeCommand := make_fself $(targets) $(targets:.elf=.self)
endif

include make/rules.mk
