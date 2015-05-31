#*******************************************************
# Newton game dynamics 
# copy right by Julio Jerez 2012
#
#*******************************************************
#
# Generic makefile 
# this make file generate the libraries:
# dg, physics, and newton
#
#*******************************************************  


# ******************************************************
#
# low level library
#
# ******************************************************
LIB_NAME = libdAnimation.a

SDK_MATH = ../../../dMath
SDK_CONTAINERS = ../../../dContainers
SDK_JOINTS = ../../../dCustomJoints

NEWTON_SDK = ../../../posix64


LIB_SRCS = \
	$(SRC_PATH)/dAnimationBlendNode.cpp \
	$(SRC_PATH)/dAnimationClip.cpp \
	$(SRC_PATH)/dAnimationNode.cpp \
	$(SRC_PATH)/dAnimationPlayer.cpp \
	$(SRC_PATH)/dBezierSpline.cpp \
	$(SRC_PATH)/dAnimationStdAfx.cpp

# ******************************************************
#
# Allsource files
#
# ******************************************************
ALL_SRC_FILES = $(LIB_SRCS) 
DG_OBJ_FILES = $(ALL_SRC_FILES:.cpp=.o)

COMPILER = gcc

# POSIX options	 gcc 4.2.2
CPU_FLAGS = -O2 -g -msse  -msse2 -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant 
FLAGS = -c -Wall -Wno-strict-aliasing -D_POSIX_VER -DD_JOINTLIBRARY_STATIC_LIB $(CPU_FLAGS) -I$(NEWTON_SDK) -I$(INCLUDED_PATH) -I$(SDK_MATH) -I$(SDK_CONTAINERS) -I$(SDK_JOINTS)
 


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<


# libraries targets
libTemp.a : $(DG_OBJ_FILES)
	ar r $@ $?
	#strip -g -S -d -v $@  -o$@ 
	cp $@ $(NEWTON_SDK)/$(LIB_NAME)


# clean all objects target
clean :
	rm $(DG_OBJ_FILES)
	rm *.a
	touch $(ALL_SRC_FILES)
