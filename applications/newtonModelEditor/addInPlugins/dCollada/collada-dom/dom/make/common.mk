ifeq ($(os),ps3)
cc := ppu-lv2-g++
ar := ppu-lv2-ar rcs
exeSuffix := .elf
else
cc := g++
ar := ar rcs
exeSuffix :=
endif

ccFlags := -Wall
ifeq ($(conf),debug)
ccFlags += -g -D_DEBUG
debugSuffix := -d
else
ccFlags += -O2 -DNDEBUG
debugSuffix :=
endif

ifeq ($(os),mac)
# Add the -arch flags to specify what architectures we're building for.
ccFlags += $(addprefix -arch ,$(subst x86,i386,$(archs)))
endif

libOpts :=
ifeq ($(os),windows)
# In case we're using the Cygwin compiler/linker, instruct cygwin to use the
# MinGW compiler to get a native Windows build. If you actually want a
# Cygwin-ized build you should comment this out.
ccFlags += -mno-cygwin
libOpts += -mno-cygwin
endif

ifeq ($(colladaVersion),1.4)
ccFlags += -DCOLLADA14
endif
ifeq ($(colladaVersion),1.5)
ccFlags += -DCOLLADA15
endif

# Clear out a bunch of variables that may have previously been set
src :=
targets :=
includeOpts :=
sharedLibSearchPaths :=
dependentLibs :=
postCreateExeCommand :=

buildID := $(os)
ifeq ($(os),windows)
buildID := mingw
endif

outPath := build/$(buildID)-$(colladaVersion)$(if $(findstring debug,$(conf)),$(debugSuffix))/
objPath := $(outPath)obj/
colladaVersionNoDots := $(subst .,,$(colladaVersion))
xmlparsers := $(if $(findstring ps3,$(os)),tinyxml,$(parsers))