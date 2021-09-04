bl_info = {
    "name": "NewtonPhysics",
    "author": "Newton Dynamics 4.00",
    "version": (1, 0),
    "blender": (2, 90, 0),
    "location": "View3D > Add > Mesh > New Object",
    "description": "Adds physics properties to a mesh using netwon physics",
    "warning": "",
    "doc_url": "",
    "category": "Physics",
}

# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

import os
import sys
import bpy
from bpy.types import (Panel, Operator)

blenderPath = os.getenv('Blender') + '/scripts/addons/newtonPy'
print (blenderPath)

if not blenderPath in sys.path:
   sys.path.append(blenderPath)

import newton


class ButtomOperator(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "NewtonWorld.1"
    bl_label = "NewtonWorld"

    def execute(self, context):
		cppVec = newton.dVector(1.0, 2.0, 3.0, 4.0)
		print ('cpp    vector ',  cppVec.GetX(), cppVec.GetY(), cppVec.GetZ())
		return {'FINISHED'}

class NewtonWorldPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "NewtonWorld"
    bl_idname = "OBJECT_PT_newtonWorld"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'UI'
    bl_context = "object"

    def draw(self, context):
        layout = self.layout

        obj = context.object

        row = layout.row()
        row.label(text="Hello world!", icon='WORLD_DATA')

        row = layout.row()
        row.label(text="Active object is: " + obj.name)
        row = layout.row()
        row.prop(obj, "name")

        row = layout.row()
        row.operator("mesh.primitive_cube_add")


from bpy.utils import (register_class, unregister_class)
_classes = [ButtomOperator, NewtonWorldPanel]

def register():
	for cls in _classes:
		register_class(cls)

def unregister():
	for cls in _classes:
		unregister_class(cls)

if __name__ == "__main__":
    register()
