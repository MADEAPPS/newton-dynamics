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
LIB_NAME = libdVisualDebuggerServer.a

INCLUDED_PATH = ../../../dVisualDebuggerServer
SRC_PATH = ../../../dVisualDebuggerServer

SDK_MATH = ../../../dMath
SDK_CONTAINERS = ../../../dContainers
SDK_ENET = ../../../thirdParty/enet-1.2/include

NEWTON_SDK = ../../../posix64


LIB_SRCS = \
	$(SRC_PATH)/DebuggerServer.cpp \
	$(SRC_PATH)/ServerBodyProxyMap.cpp \
	$(SRC_PATH)/NewtonDebuggerServer.cpp

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
FLAGS = -c -Wall -Wno-strict-aliasing -D_POSIX_VER  $(CPU_FLAGS) -I$(NEWTON_SDK) -I$(INCLUDED_PATH) -I$(SDK_ENET) -I$(SDK_MATH) -I$(SDK_CONTAINERS)
 


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
