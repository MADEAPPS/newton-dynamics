#*******************************************************
#
# Newton game dynamics 
# copy right by Julio Jerez 2003 - 2012
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
SDK_FLAGS = -m32 -O0 -fpic -msse -msse2 -c -Wall -D_POSIX_VER -DD_JOINTLIBRARY_STATIC_LIB -g `wx-config --libs` `wx-config --cxxflags` `freetype-config --cflags` `freetype-config --libs`

# -pg
#SDK_FLAGS = -m32 -O3 -fpic -msse -msse2 -c -Wall -Werror -D_POSIX_VER -g 
#SDK_FLAGS = -m32 -O3 -fpic -msse -msse2 -c -Wall -D_POSIX_VER -g 

PLATFORM = posix32

# include path
SDK_MAIN_PATH = ../../sdkDemos
SDK_DEMOS_PATH = ../../sdkDemos/demos
SDK_TOOLBOX_PATH = ../../sdkDemos/toolBox
SDK_NEWTON_PATH = ../../../../packages/$(PLATFORM)
SDK_MATH_PATH = ../../../../packages/dMath
SDK_SCENE_PATH = ../../../../packages/dScene
SDK_JOINTS_PATH = ../../../../packages/dCustomJoints
SDK_CONTAINERS_PATH = ../../../../packages/dContainers
SDK_VISUALDEBUGGER_PATH = ../../../../packages/dVisualDebuggerServer
SDK_AILIBRARY_PATH = ../../../../packages/dAILibrary
SDK_XML_PATH = ../../../../packages/thirdParty/tinyxml




SDK_INCLUDE_PATH = \
		-I ../../../../coreLibrary_300/source/newton \
		-I ../../../../coreLibrary_300/source/core \
		-I ../../../../coreLibrary_300/source/ai \
		-I $(SDK_MAIN_PATH) \
		-I $(SDK_TOOLBOX_PATH) \
		-I $(SDK_DEMOS_PATH) \
		-I $(SDK_NEWTON_PATH) \
		-I $(SDK_MATH_PATH) \
		-I $(SDK_CONTAINERS_PATH) \
		-I $(SDK_SCENE_PATH) \
		-I $(SDK_JOINTS_PATH) \
		-I $(SDK_VISUALDEBUGGER_PATH) \
		-I $(SDK_AILIBRARY_PATH) \
		-I $(SDK_XML_PATH)



# sub project path
NEWTON_CORE_MAKE = ../../../../coreLibrary_300/projects/$(PLATFORM)
SDK_MATH_MAKE = ../../../../packages/projects/$(PLATFORM)/dMath
SDK_SCENE_MAKE = ../../../../packages/projects/$(PLATFORM)/dScene
SDK_JOINTS_MAKE = ../../../../packages/projects/$(PLATFORM)/dCustomJoints
SDK_CONTAINERS_MAKE = ../../../../packages/projects/$(PLATFORM)/dContainers
SDK_VISUALDEBUGGER_MAKE = ../../../../packages/projects/$(PLATFORM)/dVisualDebuggerServer
SDK_AILIBRARY_MAKE = ../../../../packages/projects/$(PLATFORM)/dAILibrary
SDK_XML_MAKE = ../../../../packages/projects/$(PLATFORM)/thirdParty/tinyxml




LIBS  = -L/usr/lib -lstdc++ -lrt -lm -lGL -lGLU -lGLEW -lpthread -lFOX-1.6 -lenet -lopenal


# this use newton as shared or static library
NEWTON_LIB = \
	$(SDK_NEWTON_PATH)/libNewton.a \
	$(SDK_NEWTON_PATH)/libdMath.a \
	$(SDK_NEWTON_PATH)/libdScene.a \
	$(SDK_NEWTON_PATH)/libdContainers.a \
	$(SDK_NEWTON_PATH)/libdJointLibrary.a \
	$(SDK_NEWTON_PATH)/libdVisualDebuggerServer.a \
	$(SDK_NEWTON_PATH)/libdAILibrary.a \
	$(SDK_NEWTON_PATH)/libTinyXML.a


		
SDK_MAIN_SRC = \
		$(SDK_MAIN_PATH)/DemoAIListener.cpp \
		$(SDK_MAIN_PATH)/DemoCameraListener.cpp \
		$(SDK_MAIN_PATH)/DemoEntityListener.cpp \
		$(SDK_MAIN_PATH)/DemoListenerBase.cpp \
		$(SDK_MAIN_PATH)/DemoSoundListener.cpp \
		$(SDK_MAIN_PATH)/DemoVisualDebugerListener.cpp \
		$(SDK_MAIN_PATH)/DemoAIAgent.cpp \
		$(SDK_MAIN_PATH)/DemoCamera.cpp \
		$(SDK_MAIN_PATH)/DemoEntity.cpp \
		$(SDK_MAIN_PATH)/DemoEntityManager.cpp \
		$(SDK_MAIN_PATH)/DemoMesh.cpp \
		$(SDK_MAIN_PATH)/NewtonDemos.cpp
		


SDK_DEMOS_SRC = \
		$(SDK_DEMOS_PATH)/AdvancedPlayerController.cpp \
		$(SDK_DEMOS_PATH)/BasicCar.cpp \
		$(SDK_DEMOS_PATH)/BasicFriction.cpp \
		$(SDK_DEMOS_PATH)/BasicPlayerController.cpp \
		$(SDK_DEMOS_PATH)/BasicRagDoll.cpp \
		$(SDK_DEMOS_PATH)/BasicRestitution.cpp \
		$(SDK_DEMOS_PATH)/BasicStacking.cpp \
		$(SDK_DEMOS_PATH)/ClosestDistance.cpp \
		$(SDK_DEMOS_PATH)/CompoundCollision.cpp \
		$(SDK_DEMOS_PATH)/ContinueCollision.cpp \
		$(SDK_DEMOS_PATH)/ConvexApproximation.cpp \
		$(SDK_DEMOS_PATH)/ConvexCast.cpp \
		$(SDK_DEMOS_PATH)/HeavyVehicles.cpp \
		$(SDK_DEMOS_PATH)/HeighFieldCollision.cpp \
		$(SDK_DEMOS_PATH)/KinematicBodies.cpp \
		$(SDK_DEMOS_PATH)/MeshCollision.cpp \
		$(SDK_DEMOS_PATH)/MultiRayCasting.cpp \
		$(SDK_DEMOS_PATH)/PostCompoundCreateBuildTest.cpp \
		$(SDK_DEMOS_PATH)/PrecessingTops.cpp \
		$(SDK_DEMOS_PATH)/PrimitiveCollision.cpp \
		$(SDK_DEMOS_PATH)/PuckSlide.cpp \
		$(SDK_DEMOS_PATH)/ScaledCollision.cpp \
		$(SDK_DEMOS_PATH)/SceneCollision.cpp \
		$(SDK_DEMOS_PATH)/SimpleBooleans.cpp \
		$(SDK_DEMOS_PATH)/SimpleConvexShatter.cpp \
		$(SDK_DEMOS_PATH)/SoftBodies.cpp \
		$(SDK_DEMOS_PATH)/SuperCar.cpp \
		$(SDK_DEMOS_PATH)/UserStaticCollision.cpp \
		$(SDK_DEMOS_PATH)/UsingNewtonMeshTool.cpp

#	$(SDK_DEMOS_PATH)/ArchimedesBuoyancy.cpp \
#	$(SDK_DEMOS_PATH)/BasicCustomJoints.cpp \
#	$(SDK_DEMOS_PATH)/BasicRobots.cpp \
#	$(SDK_DEMOS_PATH)/BasicRagDoll.cpp \
#	$(SDK_DEMOS_PATH)/BasicStacking.cpp \
#	$(SDK_DEMOS_PATH)/Jengas.cpp \
#	$(SDK_DEMOS_PATH)/LegacyJoints.cpp \
#	$(SDK_DEMOS_PATH)/MeshCollision.cpp \
#	$(SDK_DEMOS_PATH)/Pyramids.cpp \
#	$(SDK_DEMOS_PATH)/RayCastCar.cpp \
#	$(SDK_DEMOS_PATH)/ScaledCollision.cpp \
#	$(SDK_DEMOS_PATH)/SpecialTrackJoints.cpp \
#	$(SDK_DEMOS_PATH)/Towers.cpp \
#	$(SDK_DEMOS_PATH)/TriggersAndForceFields.cpp \
#	$(SDK_DEMOS_PATH)/UnstableStacks.cpp \
#	$(SDK_DEMOS_PATH)/Walls.cpp 
#	$(SDK_DEMOS_PATH)/BuildingDestruction.cpp \
#	$(SDK_DEMOS_PATH)/RealTimeSimpleDestruction.cpp \
#	$(SDK_DEMOS_PATH)/PrefabSimpleDestruction.cpp \


SDK_TOOLBOX_SRC = \
		$(SDK_TOOLBOX_PATH)/DebugDisplay.cpp \
		$(SDK_TOOLBOX_PATH)/dHighResolutionTimer.cpp \
		$(SDK_TOOLBOX_PATH)/dRuntimeProfiler.cpp \
		$(SDK_TOOLBOX_PATH)/dSoundManager.cpp \
		$(SDK_TOOLBOX_PATH)/HeightFieldPrimitive.cpp \
		$(SDK_TOOLBOX_PATH)/MousePick.cpp \
		$(SDK_TOOLBOX_PATH)/OpenGlUtil.cpp \
		$(SDK_TOOLBOX_PATH)/PhysicsUtils.cpp \
		$(SDK_TOOLBOX_PATH)/SkyBox.cpp \
		$(SDK_TOOLBOX_PATH)/TargaToOpenGl.cpp \
		$(SDK_TOOLBOX_PATH)/toolbox_stdafx.cpp \
		$(SDK_TOOLBOX_PATH)/UserHeightFieldCollision.cpp \
		$(SDK_TOOLBOX_PATH)/UserPlaneCollision.cpp
#		$(SDK_TOOLBOX_PATH)/LevelPrimitive.cpp \
#		$(SDK_TOOLBOX_PATH)/PlaneCollision.cpp \
#		$(SDK_TOOLBOX_PATH)/ShaderPrograms.cpp \


#NOTE as long as no NewtonAI headers in Newton.h
DG_AI_PATH = ../../../../coreLibrary_300/source/ai/
SDK_AI_SRCS = \
	$(DG_AI_PATH)dgAIAgent.cpp \
	$(DG_AI_PATH)dgAIStdafx.cpp \
	$(DG_AI_PATH)dgAIWorld.cpp \
	$(DG_AI_PATH)NewtonAI.cpp

SDK_SRCS = $(SDK_MAIN_SRC) $(SDK_DEMOS_SRC) $(SDK_TOOLBOX_SRC) $(SDK_AI_SRCS)


SDK_OBJS = $(SDK_SRCS:.cpp=.o)

COMPILER = gcc
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
	make -C $(SDK_AILIBRARY_MAKE)
	make -C $(SDK_VISUALDEBUGGER_MAKE)
#	gcc $? $(NEWTON_LIB) $(FOX_LIB) $(FMOD_LIB) $(ENET_LIB) $(LIBS) -o $@.elf
	gcc $? $(NEWTON_LIB) $(LIBS) -o $@.elf

# clean all objects target
clean :
	rm $(SDK_OBJS)
	touch $(SDK_SRCS)
	make clean -C $(NEWTON_CORE_MAKE)
	make clean -C $(SDK_XML_MAKE)
	make clean -C $(SDK_MATH_MAKE)
	make clean -C $(SDK_SCENE_MAKE)
	make clean -C $(SDK_JOINTS_MAKE)
	make clean -C $(SDK_CONTAINERS_MAKE)
	make clean -C $(SDK_AILIBRARY_MAKE)
	make clean -C $(SDK_VISUALDEBUGGER_MAKE)
