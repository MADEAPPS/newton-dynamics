#*******************************************************
#
# Newton game dynamics 
# copy right by Julio Jerez 2003 - 2010 
# Copyright (c) <2010> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely
#
#*******************************************************

#*******************************************************
#
# SDK Sample tutorial: 
#
#*******************************************************  


# ******************************************************
#
# project source files
#
# ******************************************************


# generic sdk flags
SDK_FLAGS = -O0 -c -Wall -D_LINUX_VER -g -pg
#SDK_FLAGS = -m32 -O3 -c -Wall -Werror -D_LINUX_VER -g 
#SDK_FLAGS = -m32 -O3 -c -Wall -D_LINUX_VER -g 

PLATFORM = linux32


# include path
SDK_MAIN_PATH = ../../sdkDemos
SDK_DEMOS_PATH = ../../sdkDemos/demos
SDK_TOOLBOX_PATH = ../../sdkDemos/toolBox
SDK_NEWTON_PATH = ../../../../packages/$(PLATFORM)
SDK_MATH_PATH = ../../../../packages/dMath
SDK_SCENE_PATH = ../../../../packages/dScene
SDK_JOINTS_PATH = ../../../../packages/dCustomJoints
SDK_CONTAINERS_PATH = ../../../../packages/dContainers
SDK_XML_PATH = ../../../../packages/thirdParty/tinyxml
QT_HEADERS_PATH = /usr/include/qt4


SDK_INCLUDE_PATH = \
		-I ../../../../coreLibrary_200/source/newton \
		-I $(SDK_MAIN_PATH) \
		-I $(SDK_DEMOS_PATH) \
		-I $(SDK_XML_PATH) \
		-I $(SDK_MATH_PATH) \
		-I $(SDK_SCENE_PATH) \
		-I $(SDK_JOINTS_PATH) \
		-I $(SDK_NEWTON_PATH) \
		-I $(SDK_TOOLBOX_PATH) \
		-I $(SDK_CONTAINERS_PATH) \
		-I $(QT_HEADERS_PATH) 


# sub project path
NEWTON_CORE_MAKE = ../../../../coreLibrary_200/projets/$(PLATFORM)
SDK_MATH_MAKE = ../../../../packages/projects/$(PLATFORM)/dMath
SDK_SCENE_MAKE = ../../../../packages/projects/$(PLATFORM)/dScene
SDK_JOINTS_MAKE = ../../../../packages/projects/$(PLATFORM)/dCustomJoints
SDK_CONTAINERS_MAKE = ../../../../packages/projects/$(PLATFORM)/dContainers
SDK_XML_MAKE = ../../../../packages/projects/$(PLATFORM)/thirdParty/tinyxml




LIBS  = -L/usr/lib -lGL -lGLU -lGLEW -lQtCore -lQtGui -lQtOpenGL 
			

# this use newton as shared or static library
NEWTON_LIB = \
	$(SDK_NEWTON_PATH)/libNewton.a \
	$(SDK_NEWTON_PATH)/libdMath.a \
	$(SDK_NEWTON_PATH)/libdScene.a \
	$(SDK_NEWTON_PATH)/libTinyXML.a \
	$(SDK_NEWTON_PATH)/libdContainers.a \
	$(SDK_NEWTON_PATH)/libdJointLibrary.a 


SDK_MAIN_SRC = \
		$(SDK_MAIN_PATH)/DemoMesh.cpp \
		$(SDK_MAIN_PATH)/DemoCamera.cpp \
		$(SDK_MAIN_PATH)/DemoEntity.cpp \
		$(SDK_MAIN_PATH)/NewtonDemos.cpp \
		$(SDK_MAIN_PATH)/PhysicsUtils.cpp \
		$(SDK_MAIN_PATH)/DemoEntityManager.cpp \
		$(SDK_MAIN_PATH)/DemoDialogHelpers.cpp \
		$(SDK_MAIN_PATH)/moc_NewtonDemos.cpp 
		

SDK_DEMOS_SRC = \
		$(SDK_DEMOS_PATH)/BasicFriction.cpp \
		$(SDK_DEMOS_PATH)/BasicSphereSpin.cpp \
		$(SDK_DEMOS_PATH)/BasicStacking.cpp \
		$(SDK_DEMOS_PATH)/ClosestDistance.cpp \
		$(SDK_DEMOS_PATH)/ConvexApproximation.cpp \
		$(SDK_DEMOS_PATH)/MeshCollision.cpp \
		$(SDK_DEMOS_PATH)/SceneCollision.cpp \
		$(SDK_DEMOS_PATH)/SoftBodies.cpp \
		$(SDK_DEMOS_PATH)/SimpleConvexShatter.cpp
#	$(SDK_DEMOS_PATH)/BasicRestitution.cpp \
#	$(SDK_DEMOS_PATH)/ArchimedesBuoyancy.cpp \
#	$(SDK_DEMOS_PATH)/BasicCustomJoints.cpp \
#	$(SDK_DEMOS_PATH)/BasicRobots.cpp \
#	$(SDK_DEMOS_PATH)/BasicRagDoll.cpp \
#	$(SDK_DEMOS_PATH)/BasicStacking.cpp \
#	$(SDK_DEMOS_PATH)/CompoundCollision.cpp \
#	$(SDK_DEMOS_PATH)/ConvexCast.cpp \
#	$(SDK_DEMOS_PATH)/Jengas.cpp \
#	$(SDK_DEMOS_PATH)/LegacyJoints.cpp \
#	$(SDK_DEMOS_PATH)/MeshCollision.cpp \
#	$(SDK_DEMOS_PATH)/PrecessingTops.cpp \
#	$(SDK_DEMOS_PATH)/Pyramids.cpp \
#	$(SDK_DEMOS_PATH)/RayCastCar.cpp \
#	$(SDK_DEMOS_PATH)/ScaledCollision.cpp \
#	$(SDK_DEMOS_PATH)/SpecialTrackJoints.cpp \
#	$(SDK_DEMOS_PATH)/Towers.cpp \
#	$(SDK_DEMOS_PATH)/TriggersAndForceFields.cpp \
#	$(SDK_DEMOS_PATH)/UnstableStacks.cpp \
#	$(SDK_DEMOS_PATH)/CharacterController.cpp \
#	$(SDK_DEMOS_PATH)/Walls.cpp 
#	$(SDK_DEMOS_PATH)/BuildingDestruction.cpp \
#	$(SDK_DEMOS_PATH)/RealTimeSimpleDestruction.cpp \
#	$(SDK_DEMOS_PATH)/PrefabSimpleDestruction.cpp \


SDK_TOOLBOX_SRC = \
		$(SDK_TOOLBOX_PATH)/DebugDisplay.cpp \
		$(SDK_TOOLBOX_PATH)/dHeightResolutionTimer.cpp \
		$(SDK_TOOLBOX_PATH)/dRuntimeProfiler.cpp \
		$(SDK_TOOLBOX_PATH)/dSimulationThread.cpp \
		$(SDK_TOOLBOX_PATH)/OpenGlUtil.cpp \
		$(SDK_TOOLBOX_PATH)/SkyBox.cpp \
		$(SDK_TOOLBOX_PATH)/TargaToOpenGl.cpp \
		$(SDK_TOOLBOX_PATH)/toolbox_stdafx.cpp 



		
SDK_SRCS = $(SDK_MAIN_SRC) $(SDK_DEMOS_SRC) $(SDK_TOOLBOX_SRC) 

SDK_OBJS = $(SDK_SRCS:.cpp=.o)

COMPILER = g++
FLAGS = $(SDK_FLAGS) $(SDK_INCLUDE_PATH)


.SUFFIXES : .o .cpp
.cpp.o :
	$(COMPILER) $(FLAGS) -o $@ $<	

# main target
NewtonDemos : $(SDK_OBJS)
	make -C $(NEWTON_CORE_MAKE)
	make -C $(SDK_XML_MAKE)
	make -C $(SDK_MATH_MAKE)
	make -C $(SDK_SCENE_MAKE)
	make -C $(SDK_JOINTS_MAKE)
	make -C $(SDK_CONTAINERS_MAKE)
	gcc $? $(NEWTON_LIB) $(LIBS) -o $@.elf	



# clean all objects target
clean :
	make clean -C $(NEWTON_CORE_MAKE)
	make clean -C $(SDK_XML_MAKE)
	make clean -C $(SDK_MATH_MAKE)
	make clean -C $(SDK_SCENE_MAKE)
	make clean -C $(SDK_JOINTS_MAKE)
	make clean -C $(SDK_CONTAINERS_MAKE)
	rm $(SDK_OBJS)
	touch $(SDK_SRCS)
	

	
	
	
	
