# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

import bpy
import newton

# create the panel inteface
#class NewtonWorld(newton.ndWorld):
#class NewtonWorld(bpy.types.bpy_struct):
#    """create and interface to the newton workd"""

class TestManager(bpy.types.Object):
    """create and interface to the newton workd"""

    def __init__(self, object):
        #print ("crate managet one once")
        self.name = 'newton_world'


class NewtonWorldCreateHomeObject(bpy.types.Operator):
    """Creates a newton world home"""
    bl_label = 'create newton world home'
    bl_idname = 'view3d.newton_world_create_home'
    bl_description = "create newton world home"

    def execute(self, context):
        scene = context.scene
        selectedObjec = context.active_object
        bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False, align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))
        if selectedObjec == context.active_object:
            print ('change to [object mode] operator canceled')
            return {'CANCELLED'}

        context.active_object.name = 'newtonHome'
        return {'FINISHED'}

class NewtonWorldCreate(bpy.types.Operator):
    """Creates a newton world"""
    bl_label = 'create newton world'
    bl_idname = 'view3d.newton_world_create'
    bl_description = "create a newton world"

    def execute(self, context):
        scene = context.scene

        # this does not works.
        #scene.newton_world = bpy.data.objects.new('newton_world', None) 
        scene.newton_world = TestManager(context.active_object)
        return {'FINISHED'}

class NewtonWorldDestroy(bpy.types.Operator):
    """Destroy a newton world"""
    bl_label = 'delete newton world'
    bl_idname = 'view3d.newton_world_destroy'
    bl_description = "destroy a newton world"

    def execute(self, context):
        scene = context.scene

        scene.newton_world.name = 'newtonHome'
        scene.newton_world = None
        return {'FINISHED'}
