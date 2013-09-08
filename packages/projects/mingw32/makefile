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


PLATFORM = mingw32


# sub project path
NEWTON_CORE_MAKE = ../../../coreLibrary_300/projects/$(PLATFORM)
SDK_MATH_MAKE = ../../../../packages/projects/$(PLATFORM)/dMath
SDK_SCENE_MAKE = ../../../../packages/projects/$(PLATFORM)/dScene
SDK_JOINTS_MAKE = ../../../../packages/projects/$(PLATFORM)/dCustomJoints
SDK_CONTAINERS_MAKE = ../../../../packages/projects/$(PLATFORM)/dContainers
SDK_VISUALDEBUGGER_MAKE = ../../../../packages/projects/$(PLATFORM)/dVisualDebuggerServer
SDK_AILIBRARY_MAKE = ../../../../packages/projects/$(PLATFORM)/dAILibrary
SDK_XML_MAKE = ../../../../packages/projects/$(PLATFORM)/thirdParty/tinyxml


# main target
NetwonSDK : 
	make -C $(NEWTON_CORE_MAKE)
#	make -C $(SDK_MATH_MAKE)
#	make -C $(SDK_SCENE_MAKE)
#	make -C $(SDK_CONTAINERS_MAKE)
#	make -C $(SDK_JOINTS_MAKE)	
#	make -C $(SDK_XML_MAKE)

# clean all objects target
clean :
	make -C $(NEWTON_CORE_MAKE) clean
#	make clean -C $(SDK_XML_MAKE)
#	make clean -C $(SDK_MATH_MAKE)
#	make clean -C $(SDK_SCENE_MAKE)
#	make clean -C $(SDK_JOINTS_MAKE)
#	make clean -C $(SDK_CONTAINERS_MAKE)
#	make clean -C $(SDK_VISUALDEBUGGER_MAKE)
