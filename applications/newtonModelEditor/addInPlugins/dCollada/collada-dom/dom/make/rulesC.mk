#
# Copyright 2006 Sony Computer Entertainment Inc.
#
# Licensed under the MIT Open Source License, for details please see license.txt or the website
# http://www.opensource.org/licenses/mit-license.php
# 
#


##############################################################################
#
#   As rules.mk works only for .cpp files and minizip uses .c files,
#   I copied and changed it.
#
##############################################################################



# If we're building a lib of any sort libVersion will be set to a version number
# of the form major.minor, e.g. 2.0. Parse out the major and minor version numbers.
libMajorVersion := $(word 1,$(subst ., ,$(libVersion)))
libMinorVersion := $(word 2,$(subst ., ,$(libVersion)))

obj := $(addprefix $(objPath),$(notdir $(src:.c=.o)))
dependencyFiles := $(obj:.o=.d)
outputFiles := $(obj) $(dependencyFiles) $(targets)
outputFiles += $(foreach so,$(filter %.so,$(targets)),$(so).$(libMajorVersion) $(so).$(libVersion))

# Parse the targets, which can contain any of the following:
#  static lib (.a)
#  shared lib (.so)
#  Windows dynamic lib (.dll)
#  Mac dynamic lib (.dylib)
#  framework (.framework)
#  exe (no extension, or .elf on PS3)
staticLib := $(filter %.a,$(targets))
sharedLib := $(filter %.so,$(targets))
dll := $(filter %.dll,$(targets))
dylib := $(filter %.dylib,$(targets))
framework := $(filter %.framework,$(targets))
# For each framework we need a corresponding .dylib
dylib += $(framework:.framework=.dylib)
# Any target other than a .a, .so, .dll, .dylib, or .framework is considered an exe
exe := $(filter-out $(staticLib) $(sharedLib) $(dll) $(dylib) $(framework),$(targets))

ifneq ($(obj),)
# Pull in dependency info for *existing* .o files
-include $(dependencyFiles)

# Make has weird evaluation semantics, so we have to be careful to capture the state of
# any values we use in rule commands. This is the reason for all the target-specific variables.
#$(obj): cc := $(cc)
$(obj): cc := gcc
$(obj): ccFlags := $(ccFlags)
$(obj): ccFlagsNoArch := $(filter-out -arch ppc ppc64 i386 x86_64,$(ccFlags))
$(obj): includeOpts := $(includeOpts)

# Call createObjRule with a source file path
define createObjRule
srcPath := $(1)

# Pick off the matching obj files
objFiles := $$(addprefix $$(objPath),$$(notdir $$(filter $$(srcPath)%,$$(src:.c=.o))))

# We're going to automatically generate make rules for our header dependencies.
# See this for more info: http://www.cs.berkeley.edu/~smcpeak/autodepend/autodepend.html
# When using the -M option to generate dependency info, we can't have any -arch flags or
# gcc complains.
$$(objFiles): $$(objPath)%.o: $$(srcPath)%.c | $$(sort $$(dir $$(objFiles)))
	@echo Compiling $$< to $$@
	$$(cc) -c $$< $$(ccFlags) $$(includeOpts) -o $$@
	@$$(cc) -MM $$< $$(ccFlagsNoArch) $$(includeOpts) > $$(@:.o=.d)
	@mv -f $$(@:.o=.d) $$(@:.o=.d.tmp)
	@sed -e 's|.*:|$$@:|' < $$(@:.o=.d.tmp) > $$(@:.o=.d)
	@sed -e 's/.*://' -e 's/\\$$$$//' < $$(@:.o=.d.tmp) | fmt -1 | \
		sed -e 's/^ *//' -e 's/$$$$/:/' >> $$(@:.o=.d)
	@rm -f $$(@:.o=.d.tmp)
endef

srcPaths := $(sort $(dir $(src)))
$(foreach path,$(srcPaths),$(eval $(call createObjRule,$(path))))
endif

# Rule for static libs
ifneq ($(staticLib),)
$(staticLib): ar := $(ar)
$(staticLib): $(obj) | $(dir $(staticLib))
	@echo Creating $@
	$(ar) $@ $^
endif

# Rules for shared libs
ifneq ($(sharedLib),)
sharedLibMajor := $(sharedLib).$(libMajorVersion)
sharedLibMajorMinor := $(sharedLib).$(libVersion)
$(sharedLibMajorMinor): cc := $(cc)
$(sharedLibMajorMinor): ccFlags := $(ccFlags)
$(sharedLibMajorMinor): libOpts := $(libOpts)
$(sharedLibMajorMinor): $(dependentLibs) $(obj) | $(dir $(sharedLibMajorMinor))
	@echo Linking $@
	$(cc) $(ccFlags) -shared -o $@ $^ $(libOpts)

$(sharedLibMajor): $(sharedLibMajorMinor) | $(dir $(sharedLibMajor))
	cd $(dir $@)  &&  ln -sf $(notdir $^) $(notdir $@)

$(sharedLib): $(sharedLibMajor) | $(dir $(sharedLib))
	cd $(dir $@)  &&  ln -sf $(notdir $^) $(notdir $@)
endif

# Rules for dlls
ifneq ($(dll),)
$(dll): cc := $(cc)
$(dll): ccFlags := $(ccFlags)
$(dll): libOpts := $(libOpts)
$(dll): $(dependentLibs) $(obj) | $(dir $(dll))
	@echo Linking $@
	$(cc) $(ccFlags) -Wl,--out-implib,$(@:.dll=.lib) -shared -o $@ $^ $(libOpts)
endif

# Rule for Mac-style dynamic libs
ifneq ($(dylib),)
$(dylib): cc := $(cc)
$(dylib): ccFlags := $(ccFlags)
$(dylib): libOpts := $(libOpts)
$(dylib): libVersion := $(libVersion)
$(dylib): $(dependentLibs) $(obj) | $(dir $(dylib))
	@echo Linking $@
	$(cc) $(ccFlags) -dynamiclib -install_name $(notdir $@) -current_version $(libVersion).0 \
		-compatibility_version $(libVersion).0 -o $@ $^ $(libOpts)
endif

# Rule for Mac frameworks
ifneq ($(framework),)
$(framework): framework := $(framework)
$(framework): dylib := $(dylib)
$(framework): dylibNoExt := $(notdir $(basename $(dylib)))
$(framework): libVersion := $(libVersion)
$(framework): frameworkCurVersionPath := $(framework)/Versions/$(libVersion)
$(framework): copyFrameworkHeadersCommand := $(copyFrameworkHeadersCommand)
$(framework): copyFrameworkResourcesCommand := $(copyFrameworkResourcesCommand)
$(framework): $(dylib)
	@echo Creating framework $@
# First remove the framework folder if it's already there. Otherwise we get errors about
# files already existing and such.
	rm -rf $(framework)
# Set up the headers
	mkdir -p $(frameworkCurVersionPath)/Headers
	$(copyFrameworkHeadersCommand)
# Set up the resources
	mkdir -p $(frameworkCurVersionPath)/Resources
	$(copyFrameworkResourcesCommand)
# Set up the dylib. It's conventional in Mac frameworks to drop the .dylib extension.
	cp $(dylib) $(frameworkCurVersionPath)/$(dylibNoExt)
# Use install_name_tool to fix the lib name of the dylib
	install_name_tool -id $(abspath $(frameworkCurVersionPath)/$(dylibNoExt)) $(frameworkCurVersionPath)/$(dylibNoExt)
# Set up the "current version" links
	ln -s $(libVersion) $(framework)/Versions/Current
	ln -s Versions/Current/Headers $(framework)/Headers
	ln -s Versions/Current/Resources $(framework)/Resources
	ln -s Versions/Current/$(dylibNoExt) $(framework)/$(dylibNoExt)
# Remove any .svn folders we may have inadvertently copied to the framework
	find $@ -name '.svn' | xargs rm -r
endif

# Rules for exes
ifneq ($(exe),)
$(exe): cc := $(cc)
$(exe): ccFlags := $(ccFlags)
$(exe): obj := $(obj)
$(exe): libOpts := $(libOpts)
$(exe): sharedLibSearchPathCommand := $(addprefix -Wl$(comma)-rpath$(comma),$(sharedLibSearchPaths))
$(exe): postCreateExeCommand := $(postCreateExeCommand)
$(exe): $(dependentLibs) $(obj) | $(dir $(exe))
	@echo Linking $@
	$(cc) $(ccFlags) -o $@ $(obj) $(libOpts) $(sharedLibSearchPathCommand)
	$(postCreateExeCommand)
endif
