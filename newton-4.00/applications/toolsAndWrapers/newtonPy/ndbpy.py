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

blenderPath = os.getenv('Blender') + '/scripts/addons/newtonPy'
print (blenderPath)

if not blenderPath in sys.path:
	sys.path.append(blenderPath)

import newton
import newtonWorld

from bpy.types import (Panel, Operator)

# create the panel inteface
class NewtonWorldPanel(bpy.types.Panel):
    """Creates a Panel in the scene context for editing newton physics"""
    bl_label = "Newton World"
    bl_idname = "SCENE_PT_layout"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "scene"

    world = None

    def draw(self, context):
        layout = self.layout

        scene = context.scene

        # Create a simple row.
        #layout.label(text=" Simple Row:")
		#
        #row = layout.row()
        #row.prop(scene, "frame_start")
        #row.prop(scene, "frame_end")
		#
        ## Create an row where the buttons are aligned to each other.
        #layout.label(text=" Aligned Row:")
		#
        #row = layout.row(align=True)
        #row.prop(scene, "frame_start")
        #row.prop(scene, "frame_end")
		#
        ## Create two columns, by using a split layout.
        #split = layout.split()
		#
        ## First column
        #col = split.column()
        #col.label(text="Column One:")
        #col.prop(scene, "frame_end")
        #col.prop(scene, "frame_start")
		#
        ## Second column, aligned
        #col = split.column(align=True)
        #col.label(text="Column Two:")
        #col.prop(scene, "frame_start")
        #col.prop(scene, "frame_end")
		#
        ## Big render button
        layout.label(text="Big Button:")
        row = layout.row()
        row.scale_y = 3.0
        row.operator("render.render")
		
        ## Different sizes in a row
        #layout.label(text="Different button sizes:")
        #row = layout.row(align=True)
        #row.operator("render.render")
		#
        #sub = row.row()
        #sub.scale_x = 2.0
        #sub.operator("render.render")
		#
        #row.operator("render.render")

        
        if self.world is None:
            print ("esto es una mierda")
            world = newtonWorld.NewtonWorld()
            layout.operator("rigidbody.world_add")
        else:
            print ("esto es different mierda")
            world = None
            layout.operator("rigidbody.world_remove")




#*****************************************************
#
#
#*****************************************************
def main(context):
	cppVec = newton.dVector(1.0, 2.0, 3.0, 4.0)
	print ('cpp    vector ',  cppVec.GetX(), cppVec.GetY(), cppVec.GetZ())
    #for ob in context.scene.objects:
    #    print(ob)

class SimpleOperator(bpy.types.Operator):
    """Tooltip"""
    bl_idname = "object.simple_operator"
    bl_label = "Simple Object Operator"

    @classmethod
    def poll(cls, context):
        return context.active_object is not None

    def execute(self, context):
        main(context)
        return {'FINISHED'}

# register classes
def register():
	bpy.utils.register_class(NewtonWorldPanel)
	bpy.utils.register_class(SimpleOperator)

def unregister():
	bpy.utils.unregister_class(SimpleOperator)
	bpy.utils.unregister_class(NewtonWorldPanel)

if __name__ == "__main__":
    register()

    # test call
    bpy.ops.object.simple_operator()