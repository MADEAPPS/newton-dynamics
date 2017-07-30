#*******************************************************
# Newton game dynamics 
# copy right by Julio Jerez 2002 - 2005 
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
LIB_NAME = libTinyXML.a

INCLUDED_PATH = ../../../../thirdParty/tinyxml
SRC_PATH = ../../../../thirdParty/tinyxml

NEWTON_SDK = ../../../../posix64

LIB_SRCS = \
	$(SRC_PATH)/tinystr.cpp \
	$(SRC_PATH)/tinyxml.cpp \
	$(SRC_PATH)/tinyxmlerror.cpp \
	$(SRC_PATH)/tinyxmlparser.cpp 

# ******************************************************
#
# Allsource files
#
# ******************************************************
ALL_SRC_FILES = $(LIB_SRCS) 
DG_OBJ_FILES = $(ALL_SRC_FILES:.cpp=.o)

COMPILER = gcc

# POSIX options	 gcc 4.2.2
CPU_FLAGS = -fPIC -O2 -g -msse  -msse2 -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant 
FLAGS = -c -Wall -Wno-strict-aliasing -D_POSIX_VER  $(CPU_FLAGS) -I$(INCLUDED_PATH) 


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<	


# libraries targets
libTemp.a : $(DG_OBJ_FILES)
	ar r $@ $?
	strip -g -S -d -v $@  -o$@ 
	cp $@ $(NEWTON_SDK)/$(LIB_NAME) 	
	


# clean all objects target
clean :
	rm $(DG_OBJ_FILES)
	rm *.a
	touch $(ALL_SRC_FILES)

 

	
