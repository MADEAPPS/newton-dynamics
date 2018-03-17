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
LIB_NAME = libdContainers.a

INCLUDED_PATH = ../../../dContainers
INCLUDED_MATH_PATH = ../../../dMath

SRC_PATH = ../../../dContainers

NEWTON_SDK = ../../../posix32

LIB_SRCS = \
	$(SRC_PATH)/dBaseHierarchy.cpp \
	$(SRC_PATH)/dClassInfo.cpp \
	$(SRC_PATH)/dContainersStdAfx.cpp \
	$(SRC_PATH)/dCRC.cpp \
	$(SRC_PATH)/dRefCounter.cpp \
	$(SRC_PATH)/dString.cpp \
	$(SRC_PATH)/dTree.cpp


# ******************************************************
#
# Allsource files
#
# ******************************************************
ALL_SRC_FILES = $(LIB_SRCS) 
DG_OBJ_FILES = $(ALL_SRC_FILES:.cpp=.o)

COMPILER = gcc

# POSIX options	 gcc 4.2.2
CPU_FLAGS = -O2 -msse -g -msse2 -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant
FLAGS = -c -Wall -Wno-strict-aliasing -D_POSIX_VER  $(CPU_FLAGS) -I$(INCLUDED_PATH) -I$(INCLUDED_MATH_PATH)


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<	


# libraries targets
libTemp.a : $(DG_OBJ_FILES)
	ar r $@ $?
	#strip -g -S -d -v $@  -o$@ 
	cp $@ $(NEWTON_SDK)/$(LIB_NAME) 	
	
	#gcc -shared -Wl,-soname,libNewton.so $? -o libNewton.so 
	#cp libNewton.so ../newtonSDK/sdk/libNewton.so
	#sudo cp libNewton.so /usr/lib


# clean all objects target
clean :
	rm $(DG_OBJ_FILES)
	rm *.a
	touch $(ALL_SRC_FILES)
