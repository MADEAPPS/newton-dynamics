# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

#import os
#import sys
#import bpy
#
#blenderPath = os.getenv('Blender') + '/scripts/addons/newtonPy'
#print (blenderPath)
#
#if not blenderPath in sys.path:
#	sys.path.append(blenderPath)

import newton

# create the panel inteface
class NewtonWorld(newton.ndWorld):
    """create and interface to the newton workd"""
