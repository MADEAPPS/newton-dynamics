include make/common.mk

src := external-libs/minizip/src/ioapi.c \
       external-libs/minizip/src/mztools.c \
       external-libs/minizip/src/unzip.c \
       external-libs/minizip/src/zip.c

ifeq ($(findstring $(os),linux mac),)
src += external-libs/minizip/src/iowin32.c
endif


includeOpts := -Iexternal-libs/minizip/include

ifneq ($(findstring $(os),linux mac),)
ccFlags += -fPIC
endif

# mingw: link agaist zlib
ifeq ($(findstring $(os),linux mac),)
libOpts += -Lexternal-libs/libxml2/mingw/lib
libOpts += -lz
else ifeq ($(os),mac)
# libOpts += -Lexternal-libs/libxml2/mingw/lib
libOpts += -lz
endif

libName := libminizip$(debugSuffix)
ifeq ($(os),mac)
libVersion := 1.2
else
libVersion := 1.2.3
endif
libVersionNoDots := $(subst .,,$(libVersion))

targets :=
ifeq ($(os),linux)
# On Linux we build a static lib and a shared lib
targets += $(addprefix $(outPath),$(libName).a)
targets += $(addprefix $(outPath),$(libName).so)

else ifeq ($(os),windows)
# On Windows we build a static lib and a DLL
#windowsLibName := libcollada$(colladaVersionNoDots)dom
windowsLibName := $(libName)
targets += $(addprefix $(outPath),$(windowsLibName)$(debugSuffix).a)
targets += $(addprefix $(outPath),$(windowsLibName)$(libVersionNoDots)$(debugSuffix).dll)

else ifeq ($(os),mac)
# On Mac we build a framework
#targets += $(addprefix $(outPath),$(libName)$(debugSuffix).framework)
#frameworkHeadersPath = $(framework)/Versions/$(libVersion)/Headers
#copyFrameworkHeadersCommand = cp -R include/* $(frameworkHeadersPath) && \
#  mv $(frameworkHeadersPath)/$(colladaVersion)/dom $(frameworkHeadersPath)/dom && \
#  find -E $(frameworkHeadersPath) -maxdepth 1 -type d -regex '.*[0-9]+\.[0-9]+' | xargs rm -r
#frameworkResourcesPath = $(framework)/Versions/$(libVersion)/Resources
#sedReplaceExpression := -e 's/(colladaVersionNoDots)/$(colladaVersionNoDots)/g' \
#                        -e 's/(domVersion)/$(domVersion)/g' \
#                        -e 's/(debugSuffix)/$(debugSuffix)/g'
#copyFrameworkResourcesCommand = cp -R make/macFrameworkResources/* $(frameworkResourcesPath) && \
#  sed $(sedReplaceExpression) make/macFrameworkResources/Info.plist > $(frameworkResourcesPath)/Info.plist && \
#  sed $(sedReplaceExpression) make/macFrameworkResources/English.lproj/InfoPlist.strings > $(frameworkResourcesPath)/English.lproj/InfoPlist.strings
targets += $(addprefix $(outPath),$(libName).a)

else ifeq ($(os),ps3)
# On PS3 we build a static lib, since PS3 doesn't support shared libs
targets += $(addprefix $(outPath),$(libName).a)
endif

include make/rulesC.mk
