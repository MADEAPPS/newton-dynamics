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
LIB_NAME = libdJointLibrary.a

INCLUDED_PATH = ../../../dCustomJoints
SRC_PATH = ../../../dCustomJoints

SDK_MATH = ../../../dMath
SDK_CONTAINERS = ../../../dContainers

NEWTON_SDK = ../../../posix32

LIB_SRCS = \
	$(SRC_PATH)/CustomAlloc.cpp \
	$(SRC_PATH)/CustomJointLibraryStdAfx.cpp \
	$(SRC_PATH)/Custom6DOF.cpp \
	$(SRC_PATH)/CustomBallAndSocket.cpp \
	$(SRC_PATH)/CustomCorkScrew.cpp \
	$(SRC_PATH)/CustomDryRollingFriction.cpp \
	$(SRC_PATH)/CustomGear.cpp \
	$(SRC_PATH)/CustomHinge.cpp \
	$(SRC_PATH)/CustomJoint.cpp \
	$(SRC_PATH)/CustomKinematicController.cpp \
	$(SRC_PATH)/CustomPathFollow.cpp \
	$(SRC_PATH)/CustomPulley.cpp \
	$(SRC_PATH)/CustomRackAndPinion.cpp \
	$(SRC_PATH)/CustomSlider.cpp \
	$(SRC_PATH)/CustomSlidingContact.cpp \
	$(SRC_PATH)/CustomUniversal.cpp \
	$(SRC_PATH)/CustomUpVector.cpp \
	$(SRC_PATH)/CustomUserBlank.cpp \
	$(SRC_PATH)/CustomControllerManager.cpp \
	$(SRC_PATH)/CustomSkeletonTransformManager.cpp \
	$(SRC_PATH)/CustomTriggerManager.cpp \
	$(SRC_PATH)/CustomPlayerControllerManager.cpp \
	$(SRC_PATH)/CustomVehicleControllerManager.cpp
	

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
FLAGS = -c -Wall -Wno-strict-aliasing -D_POSIX_VER -DD_JOINTLIBRARY_STATIC_LIB $(CPU_FLAGS) -I$(NEWTON_SDK) -I$(INCLUDED_PATH) -I$(SDK_MATH) -I$(SDK_CONTAINERS)


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<


# libraries targets
libTemp.a : $(DG_OBJ_FILES)
	ar r $@ $?
#	strip -g -S -d -v $@  -o$@ 
	cp $@ $(NEWTON_SDK)/$(LIB_NAME)
	
	#gcc -shared -Wl,-soname,libNewton.so $? -o libNewton.so 
	#cp libNewton.so ../newtonSDK/sdk/libNewton.so
	#sudo cp libNewton.so /usr/lib


# clean all objects target
clean :
	rm $(DG_OBJ_FILES)
	rm *.a
	touch $(ALL_SRC_FILES)
