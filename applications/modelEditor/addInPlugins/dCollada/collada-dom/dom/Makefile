#!/usr/bin/make
#
# Copyright 2006 Sony Computer Entertainment Inc.
#
# Licensed under the MIT Open Source License, for details please see license.txt or the website
# http://www.opensource.org/licenses/mit-license.php
# 

# Set oldMakeExport=yes to generate a makefile suitable for use with a pre-3.80
# version of make.
ifeq ($(oldMakeExport),yes)
printCommands := yes
printMessages := no
endif

# By default, build messages are printed. These messages let the user know
# what's happening during the build.
ifneq ($(printMessages),no)
define printMessage
$(if $(not $(findstring plainEcho,$(2))),@)echo $(1)
endef
else
define printMessage
endef
endif

# By default, the actual build commands aren't printed.
ifneq ($(printCommands),yes)
.SILENT:
endif

# os: 'linux', 'mac', 'windows', or 'ps3'. Use the 'uname' command to decide the
# default value. To detect when we're on Windows we'll check to see if we're
# running on Cygwin or MinGW.
os := linux
ifneq ($(shell uname | grep -i darwin),)
os := mac
else ifneq ($(or $(shell uname | grep -i cygwin),$(shell uname | grep -i mingw)),)
os := windows
endif

# nativeArch: For internal use. Don't override this, instead override 'arch'.
nativeArch := x86
ifneq ($(shell uname -p | grep -i powerpc),)
nativeArch := ppc
endif

# arch: x86 (or i386), x64 (or x86_64), ppc, ppc64
arch := $(nativeArch)

# project: 'dom', 'domTest', or 'all'
project := all

# Release/debug configuration: 'release', 'debug', or 'all'
conf := release

# Collada version: '1.4', '1.5', or 'all'
colladaVersion := 1.4

# parser: 'libxml', 'tinyxml', or 'all'.
parser := libxml

# file: Set this to the name of a source file (eg 'dae.cpp') to build just that file
file :=

# Include any custom build settings
-include ~/.collada-dom/customSettings.mk
-include make/customSettings.mk

-include make/installPrefix.mk

ifneq ($(findstring test,$(MAKECMDGOALS)),)
# If we're running the automated tests, make sure that everything gets a chance to build
project := all
endif

# Initialize the build variables
define setBuildVar
$(1)s := $(subst all,$(2),$($(1)))
ifneq ($$(filter-out $(2),$$($(1)s)),)
$$(error Invalid setting: $(1)=$($(1)))
endif
endef

oss := $(sort $(os))
ifneq ($(filter-out linux mac ps3 windows,$(oss)),)
$(error Invalid setting os=$(os))
endif

archs := $(sort $(subst i386,x86,$(arch)))
ifneq ($(filter-out x86 ppc,$(archs)),)
$(error Invalid setting arch=$(arch))
endif

$(eval $(call setBuildVar,project,minizip dom domTest))
$(eval $(call setBuildVar,colladaVersion,1.4 1.5))
$(eval $(call setBuildVar,conf,debug release))
$(eval $(call setBuildVar,parser,libxml tinyxml))

comma := ,
domMajorVersion := 2
domMinorVersion := 1
domVersion := $(domMajorVersion).$(domMinorVersion)
domVersionNoDots := $(subst .,,$(domVersion))


# Have make automatically delete a target if there's an error in one of the rule commands
.DELETE_ON_ERROR:

allOutputFiles :=
allTargets :=

define includeConfig
override project := $(1)
override os := $(2)
override colladaVersion := $(3)
override conf := $(4)
include make/$(1).mk
allOutputFiles += $$(outputFiles)
allTargets += $$(targets)
endef

$(foreach _project,$(projects),\
$(foreach _os,$(oss),\
$(foreach _colladaVersion,$(colladaVersions),\
$(foreach _conf,$(confs),\
$(eval $(call includeConfig,$(_project),$(_os),$(_colladaVersion),$(_conf)))))))

buildObj := $(filter %$(file:.cpp=.o),$(filter %.o,$(allOutputFiles)))

.DEFAULT_GOAL := all
.PHONY: all
ifeq ($(file),)
# Do a normal build of all targets
all: $(allTargets)
else ifneq ($(buildObj),)
# Just build the source files the user requested
all: $(buildObj)
else
$(error Invalid source file - $(file))
endif

allOutputPaths := $(sort $(dir $(allOutputFiles)))
$(allOutputPaths):
	if [ ! -d $@ ] ; then mkdir -p $@; fi;


##############################################
# clean
#
.PHONY: clean
clean:
	$(call printMessage,Removing build files.)
# Delete all the output files
	rm -rf $(allOutputFiles)
# Delete each empty output folder
	for path in $(allOutputPaths); do \
		if [ -d $$path ] && [ `find $$path -type f | wc -l` -eq 0 ]; then rm -r $$path ; fi \
	done;
# Delete the build folder if it's empty
	if [ -d build ] && [ `find build -type f | wc -l` -eq 0 ] ; then rm -r build ; fi


##############################################
# test target for running the automated tests.
#
# Note: I originally had a separate target for each test, one reason being so that each
# test suite could be run in parallel. However when running in parallel I'd get weird errors where
# some of the tests would fail and some would succeed, with error messages like this:
#   I/O error : No such file or directory
#   I/O error : No such file or directory
#   error : xmlNewTextWriterFilename : out of memory!
# To work around this, I changed to using a single target with a shell loop to run each test program.
# This way the tests are forced to run serially.
domTestOpts := -all
domTestExes := $(filter %domTest,$(allTargets))
.PHONY: test
test: $(domTestExes)
	@for testExe in $(domTestExes); do \
		$(call printMessage,$$testExe $(domTestOpts);,plainEcho) \
		$$testExe $(domTestOpts); \
	done


##############################################
# install/uninstall
#
ifneq ($(filter install uninstall installTest,$(MAKECMDGOALS)),)
# You can only install on Mac or Linux. Check for that.
ifeq ($(oss),linux)
prefix := /usr/local
else ifeq ($(oss),mac)
prefix := /Library/Frameworks
else
$(error You can only install with 'os' set to 'mac' or 'linux')
endif
endif

.PHONY: uninstall
ifneq ($(installPrefix),)
ifeq ($(oss),linux)
uninstall:
	$(call printMessage,Uninstalling from $(installPrefix))
	rm -rf $(installPrefix)/include/colladadom
	rm -f  $(installPrefix)/lib/libcollada*dom*
else ifeq ($(oss),mac)
uninstall:
	$(call printMessage,Uninstalling from $(prefix))
	 rm -rf $(installPrefix)/Collada*Dom*.framework
endif
else ifneq ($(findstring uninstall,$(MAKECMDGOALS)),)
$(error Can't uninstall because we don't know what path we installed to (missing make/installPrefix.mk file))
else
uninstall:
endif

# 1st param is build path, 2nd param is framework name, 3rd param is framework version number
# e.g. $(call installMacFrameworkCmd,build/mac-1.4,Collada14Dom.framework,2.0)
define installMacFrameworkCmd
if [ -d $(1) ]; then \
cp -R $(1)/$(2) $(prefix)/$(2); \
install_name_tool -id $(prefix)/$(2)/Versions/$(3)/$(basename $(2)) $(prefix)/$(2)/$(basename $(2)); \
fi;
endef

.PHONY: install
ifeq ($(oss),linux)
install: uninstall
	$(call printMessage,Installing to $(prefix))
# Write the install prefix to the file make/installPrefix.mk so we can retrieve it for uninstalling.
	echo 'installPrefix := $(prefix)' > make/installPrefix.mk
# Install headers
	cp -R include $(prefix)/include/colladadom
# We write this as a loop to avoid an error when there are no files to remove
	for svndir in $(find $(prefix)/include/colladadom -name '.svn'); do rm -rf $svndir; done
# Install linux-1.4 libs
	if [ -d build/linux-1.4 ]; then cp -P build/linux-1.4/libcollada*dom* $(prefix)/lib; fi;
# Install linux-1.4-d libs
	if [ -d build/linux-1.4-d ]; then cp -P build/linux-1.4-d/libcollada*dom* $(prefix)/lib; fi;
else ifeq ($(oss),mac)
install: uninstall
	$(call printMessage,Installing to $(prefix))
	echo 'installPrefix := $(prefix)' > make/installPrefix.mk
	$(call installMacFrameworkCmd,build/mac-1.4,Collada14Dom.framework,$(domVersion))
	$(call installMacFrameworkCmd,build/mac-1.4-d,Collada14Dom-d.framework,$(domVersion))
endif

.PHONY: installTest
installTest:
	$(MAKE) install
	$(MAKE) clean project=domTest
	$(MAKE) test installTest=yes
	$(MAKE) uninstall
	$(MAKE) clean project=domTest
	$(MAKE) project=domTest
	$(call printMessage,installTest done)
