#*******************************************************
#
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
# dg low level library
#
# ******************************************************
DG_INCLUDED_PATH = ../../source/core
DG_PATH = $(DG_INCLUDED_PATH)/
DG_SRCS = \
	$(DG_PATH)dgAABBPolygonSoup.cpp \
	$(DG_PATH)dgAsyncThread.cpp \
	$(DG_PATH)dgConvexHull3d.cpp \
	$(DG_PATH)dgConvexHull4d.cpp \
	$(DG_PATH)dg.cpp \
	$(DG_PATH)dgCRC.cpp \
	$(DG_PATH)dgDebug.cpp \
	$(DG_PATH)dgDelaunayTetrahedralization.cpp \
	$(DG_PATH)dgGeneralMatrix.cpp \
	$(DG_PATH)dgGeneralVector.cpp \
	$(DG_PATH)dgGoogol.cpp \
	$(DG_PATH)dgIntersections.cpp \
	$(DG_PATH)dgMatrix.cpp \
	$(DG_PATH)dgMemory.cpp \
	$(DG_PATH)dgMutexThread.cpp \
	$(DG_PATH)dgNode.cpp \
	$(DG_PATH)dgPolygonSoupBuilder.cpp \
	$(DG_PATH)dgPolyhedra.cpp \
	$(DG_PATH)dgPolyhedraMassProperties.cpp \
	$(DG_PATH)dgQuaternion.cpp \
	$(DG_PATH)dgRandom.cpp \
	$(DG_PATH)dgRefCounter.cpp \
	$(DG_PATH)dgRef.cpp \
	$(DG_PATH)dgSmallDeterminant.cpp \
	$(DG_PATH)dgSPDMatrix.cpp \
	$(DG_PATH)dgObb.cpp \
	$(DG_PATH)dgThread.cpp \
	$(DG_PATH)dgThreadHive.cpp \
	$(DG_PATH)dgTree.cpp \
	$(DG_PATH)dgTypes.cpp


# ******************************************************
#
# Physics engine files
#
# ******************************************************
DG_INCLUDED_PHYSICS_PATH = ../../source/physics
DG_PHYSICS_PATH = $(DG_INCLUDED_PHYSICS_PATH)/
DG_PHYSICS_SRCS = \
	$(DG_PHYSICS_PATH)dgBody.cpp \
	$(DG_PHYSICS_PATH)dgDynamicBody.cpp \
	$(DG_PHYSICS_PATH)dgKinematicBody.cpp \
	$(DG_PHYSICS_PATH)dgBallConstraint.cpp \
	$(DG_PHYSICS_PATH)dgBilateralConstraint.cpp \
	$(DG_PHYSICS_PATH)dgBody.cpp \
	$(DG_PHYSICS_PATH)dgDynamicBody.cpp \
	$(DG_PHYSICS_PATH)dgKinematicBody.cpp \
	$(DG_PHYSICS_PATH)dgBodyMasterList.cpp \
	$(DG_PHYSICS_PATH)dgBroadPhase.cpp \
	$(DG_PHYSICS_PATH)dgCollisionBox.cpp \
	$(DG_PHYSICS_PATH)dgCollisionBVH.cpp \
	$(DG_PHYSICS_PATH)dgCollisionCapsule.cpp \
	$(DG_PHYSICS_PATH)dgCollisionChamferCylinder.cpp \
	$(DG_PHYSICS_PATH)dgCollisionCompoundFractured.cpp \
	$(DG_PHYSICS_PATH)dgCollisionCompound.cpp \
	$(DG_PHYSICS_PATH)dgCollisionCone.cpp \
	$(DG_PHYSICS_PATH)dgCollisionConvex.cpp \
	$(DG_PHYSICS_PATH)dgCollisionConvexHull.cpp \
	$(DG_PHYSICS_PATH)dgCollisionConvexPolygon.cpp \
	$(DG_PHYSICS_PATH)dgCollision.cpp \
	$(DG_PHYSICS_PATH)dgCollisionCylinder.cpp \
	$(DG_PHYSICS_PATH)dgCollisionDeformableClothPatch.cpp \
	$(DG_PHYSICS_PATH)dgCollisionDeformableSolidMesh.cpp \
	$(DG_PHYSICS_PATH)dgCollisionDeformableMesh.cpp \
	$(DG_PHYSICS_PATH)dgCollisionHeightField.cpp \
	$(DG_PHYSICS_PATH)dgCollisionInstance.cpp \
	$(DG_PHYSICS_PATH)dgCollisionMesh.cpp \
	$(DG_PHYSICS_PATH)dgCollisionNull.cpp \
	$(DG_PHYSICS_PATH)dgCollisionScene.cpp \
	$(DG_PHYSICS_PATH)dgCollisionSphere.cpp \
	$(DG_PHYSICS_PATH)dgCollisionTaperedCapsule.cpp \
	$(DG_PHYSICS_PATH)dgCollisionTaperedCylinder.cpp \
	$(DG_PHYSICS_PATH)dgCollisionUserMesh.cpp \
	$(DG_PHYSICS_PATH)dgConstraint.cpp \
	$(DG_PHYSICS_PATH)dgContact.cpp \
	$(DG_PHYSICS_PATH)dgCorkscrewConstraint.cpp \
	$(DG_PHYSICS_PATH)dgDeformableBody.cpp \
	$(DG_PHYSICS_PATH)dgDeformableContact.cpp \
	$(DG_PHYSICS_PATH)dgHingeConstraint.cpp \
	$(DG_PHYSICS_PATH)dgNarrowPhaseCollision.cpp \
	$(DG_PHYSICS_PATH)dgSlidingConstraint.cpp \
	$(DG_PHYSICS_PATH)dgUniversalConstraint.cpp \
	$(DG_PHYSICS_PATH)dgUpVectorConstraint.cpp \
	$(DG_PHYSICS_PATH)dgUserConstraint.cpp \
	$(DG_PHYSICS_PATH)dgWorld.cpp \
	$(DG_PHYSICS_PATH)dgDeformableBodiesUpdate.cpp \
	$(DG_PHYSICS_PATH)dgWorldDynamicsParallelSolver.cpp \
	$(DG_PHYSICS_PATH)dgWorldDynamicsSimpleSolver.cpp \
	$(DG_PHYSICS_PATH)dgWorldDynamicUpdate.cpp

	
# ******************************************************
#
# mesh gemotry 
#
# ******************************************************
DG_INCLUDED_MESH_PATH = ../../source/meshUtil
DG_MESH_PATH = $(DG_INCLUDED_MESH_PATH)/
DG_MESH_SRCS = \
	$(DG_MESH_PATH)dgMeshEffect1.cpp \
	$(DG_MESH_PATH)dgMeshEffect2.cpp \
	$(DG_MESH_PATH)dgMeshEffect3.cpp \
	$(DG_MESH_PATH)dgMeshEffect4.cpp \
	$(DG_MESH_PATH)dgMeshEffect5.cpp \
	$(DG_MESH_PATH)dgMeshEffect6.cpp 

# ******************************************************
#
# open cl 
#
# ******************************************************
DG_INCLUDED_OPENCL_PATH = ../../source/openCL
DG_OPENCL_PATH = $(DG_INCLUDED_OPENCL_PATH)/
#DG_OPENCL_SRCS = \
#	$(DG_OPENCL_PATH)dgOpencl.cpp \
#	$(DG_OPENCL_PATH)dgOpenclInstance.cpp \
#	$(DG_OPENCL_PATH)dgOpenclBroadPhase.cpp

	

# ******************************************************
#
# Newton engine files
#g++ -shared -o libNewton.so libNewton.a
# ******************************************************
DG_INCLUDED_NEWTON_PATH = ../../source/newton
DG_NEWTON_PATH = $(DG_INCLUDED_NEWTON_PATH)/
DG_NEWTON_SRCS = \
	$(DG_NEWTON_PATH)Newton.cpp \
	$(DG_NEWTON_PATH)NewtonClass.cpp

# ******************************************************
#
# Allsource files
#
# ******************************************************
ALL_SRC_FILES = $(DG_SRCS) $(DG_PHYSICS_SRCS) $(DG_OPENCL_SRCS) $(DG_MESH_SRCS) $(DG_NEWTON_SRCS)
DG_OBJ_FILES = $(ALL_SRC_FILES:.cpp=.o)

COMPILER = gcc

# POSIX options	 gcc 4.4.2
#CPU_FLAGS = -m32 -O0 -fPIC -g -msse -msse2 -mfpmath=sse 
#CPU_FLAGS = -m32 -O0 -fPIC -g -msse -msse4.1 -mfpmath=sse 
#CPU_FLAGS = -m32 -O2 -fpic -g -msse -msse4.1 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant
CPU_FLAGS = -m32 -O2 -fpic -g -msse -msse3 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant


FLAGS  = -c -Wall -Wno-strict-aliasing -D_POSIX_VER $(CPU_FLAGS) -I$(DG_INCLUDED_PATH) -I$(DG_INCLUDED_PHYSICS_PATH) -I$(DG_INCLUDED_MESH_PATH) -I$(DG_INCLUDED_OPENCL_PATH)


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<

# main target
engine : libNewton.a


# clean all objects target
clean :
	rm $(DG_OBJ_FILES)
	touch $(ALL_SRC_FILES)

# libraries targets
libNewton.a : $(DG_OBJ_FILES)
	ar r $@ $?
#	strip -g -S -d -v libNewton.a -o libNewton.a
	cp libNewton.a ../../../packages/posix32/libNewton.a
	cp ../../source/newton/Newton.h ../../../packages/posix32/Newton.h
	$(COMPILER) -shared -Wl,-soname,libNewton.so $? -o libNewton.so 
	cp libNewton.so ../../../packages/posix32/libNewton.so
	#sudo cp libNewton.so /usr/lib
