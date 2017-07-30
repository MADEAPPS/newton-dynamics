#*******************************************************
# Newton game dynamics 
# copy right by Julio Jerez 2002 - 2012
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
LIB_NAME = libdMath.a

INCLUDED_PATH = ../../../dMath
SRC_PATH =  ../../../dMath

NEWTON_SDK = ../../../posix32

LIB_SRCS = \
	$(SRC_PATH)/dMathDefines.cpp \
	$(SRC_PATH)/dMatrix.cpp \
	$(SRC_PATH)/dQuaternion.cpp \
	$(SRC_PATH)/dVector.cpp

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
FLAGS = -c -Wall -Wno-strict-aliasing -D_MINGW_32_VER  $(CPU_FLAGS) -I$(INCLUDED_PATH) 


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<	


# libraries targets
libTemp.a : $(DG_OBJ_FILES)
	ar r $@ $?

clean :
	del /S ..\..\*.o
