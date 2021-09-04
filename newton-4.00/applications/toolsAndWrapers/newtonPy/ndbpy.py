# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

import bpy
import sys
import os

blenderPath = os.getenv('Blender') + '/scripts/addons/newtonPy'
print (blenderPath)

if not blenderPath in sys.path:
   sys.path.append(blenderPath)

import newton
import testClass

pyVec = testClass.PythonVector ()
print ('python vector: ', pyVec.x, pyVec.y, pyVec.z) 

cppVec = newton.dVector(1.0, 2.0, 3.0, 4.0)
print ('cpp    vector ',  cppVec.GetX(), cppVec.GetY(), cppVec.GetZ())