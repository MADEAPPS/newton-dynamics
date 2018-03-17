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
LIB_NAME = libdScene.a

INCLUDED_PATH = ../../../dScene
SRC_PATH = ../../../dScene

SDK_MATH = ../../../dMath
SDK_CONTAINERS = ../../../dContainers
SDK_XML = ../../../thirdParty/tinyxml

NEWTON_SDK = ../../../posix32


LIB_SRCS = \
	$(SRC_PATH)/dBoneNodeInfo.cpp \
	$(SRC_PATH)/dCollisionBoxNodeInfo.cpp \
	$(SRC_PATH)/dCollisionCapsuleNodeInfo.cpp \
	$(SRC_PATH)/dCollisionChamferCylinderNodeInfo.cpp \
	$(SRC_PATH)/dCollisionCompoundNodeInfo.cpp \
	$(SRC_PATH)/dCollisionConeNodeInfo.cpp \
	$(SRC_PATH)/dCollisionConvexHullNodeInfo.cpp \
	$(SRC_PATH)/dCollisionCylinderNodeInfo.cpp \
	$(SRC_PATH)/dCollisionNodeInfo.cpp \
	$(SRC_PATH)/dCollisionSphereNodeInfo.cpp \
	$(SRC_PATH)/dCollisionTreeNodeInfo.cpp \
	$(SRC_PATH)/dDrawUtils.cpp \
	$(SRC_PATH)/dGeometryNodeInfo.cpp \
	$(SRC_PATH)/dGeometryNodeModifierInfo.cpp \
	$(SRC_PATH)/dGeometryNodeSkinModifierInfo.cpp \
	$(SRC_PATH)/dMaterialNodeInfo.cpp \
	$(SRC_PATH)/dMeshNodeInfo.cpp \
	$(SRC_PATH)/dNodeInfo.cpp \
	$(SRC_PATH)/dRigidbodyNodeInfo.cpp \
	$(SRC_PATH)/dRootNodeInfo.cpp \
	$(SRC_PATH)/dSceneCacheInfo.cpp \
	$(SRC_PATH)/dScene.cpp \
	$(SRC_PATH)/dSceneCacheInfo.cpp \
	$(SRC_PATH)/dSceneGraph.cpp \
	$(SRC_PATH)/dSceneModelInfo.cpp \
	$(SRC_PATH)/dSceneNodeInfo.cpp \
	$(SRC_PATH)/dSceneRender.cpp \
	$(SRC_PATH)/dSceneStdafx.cpp \
	$(SRC_PATH)/dTextureNodeInfo.cpp \
	$(SRC_PATH)/dUndoRedo.cpp \
	$(SRC_PATH)/dVariable.cpp

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
FLAGS = -c -Wall -Wno-strict-aliasing -D_POSIX_VER  $(CPU_FLAGS) -I$(NEWTON_SDK) -I$(INCLUDED_PATH) -I$(SDK_MATH) -I$(SDK_CONTAINERS) -I$(SDK_XML) 
 


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
