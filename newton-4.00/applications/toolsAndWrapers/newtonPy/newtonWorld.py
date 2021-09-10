# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

import bpy
import newton

#newtonWorld = newton.ndWorld()
newtonWorld = newton.NewtonWorld()

def NewtonStart(scene):
    fps = scene.render.fps / scene.render.fps_base
    #timestep = 1.0/fps
    #print("nominal time step ", timestep)
    newtonWorld.SetSubSteps(1.0/fps)

def NewtonUpdate(scene):
    fps = scene.render.fps / scene.render.fps_base
    #timestep = 1.0/fps
    #print("Frame Change ", scene.frame_current, " timestep ", timestep)
    newtonWorld.Update (1.0/fps)


bpy.app.handlers.depsgraph_update_pre.append(NewtonStart)
bpy.app.handlers.frame_change_pre.append(NewtonUpdate)


class NewtonWorldProperties(bpy.types.PropertyGroup):
    solverNominalFps: bpy.props.FloatProperty(name= "solver fix fps", description="solve fix frames per seconds", default = 120, min=60, max=600)
    solverIterations: bpy.props.IntProperty(name= "solver iterations", description="Set the number of solver iterations per step", default = 4, min=4, max=16)
    
    #my_float_vector : bpy.props.FloatVectorProperty(name= "Scale", soft_min= 0, soft_max= 1000, default= (1,1,1))
    #
    #my_enum: bpy.props.EnumProperty(
    #    name= "Enumerator / Dropdown",
    #    description= "sample text",
    #    items= [('OP1', "Add Cube", ""),
    #            ('OP2', "Add Sphere", ""),
    #            ('OP3', "Add Suzanne", "")
    #    ]
    #)

#class NewtonWorldCreateHomeObject(bpy.types.Operator):
#    """Creates a newton world home"""
#    bl_label = 'create newton world'
#    bl_idname = 'view3d.newton_world_create_home'
#    bl_description = "create newton world"
#
#    def execute(self, context):
#        scene = context.scene
#        selectedObjec = context.active_object
#        bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False, align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))
#        if selectedObjec == context.active_object:
#            print ('change to [object mode] operator canceled')
#            return {'CANCELLED'}
#
#        context.active_object.name = 'newtonHome'
#        world = NewtonWorld(context.active_object)
#        scene.newton_world = world
#        return {'FINISHED'}

#class NewtonWorldCreate(bpy.types.Operator):
#    """Creates a newton world"""
#    bl_label = 'create newton world'
#    bl_idname = 'view3d.newton_world_create'
#    bl_description = "create a newton world"
#
#    def execute(self, context):
#        scene = context.scene
#        world = NewtonWorld(context.active_object)
#        scene.newton_world = world
#        return {'FINISHED'}

#class NewtonWorldDestroy(bpy.types.Operator):
#    """Destroy a newton world"""
#    bl_label = 'delete newton world'
#    bl_idname = 'view3d.newton_world_destroy'
#    bl_description = "destroy a newton world"
#
#    def execute(self, context):
#        scene = context.scene
#
#        scene.newton_world.name = 'newtonHome'
#        scene.newton_world = None
#        return {'FINISHED'}



class NewtonWorldSetProperty(bpy.types.Operator):
    """newton world set engine properties"""
    bl_label = 'newton world set property'
    bl_idname = 'view3d.newton_world_set_property'
    bl_description = "newton world set property"

    def execute(self, context):
        scene = context.scene
        propertyGroup = scene.newton_world_properties

        # set all solve properties
        #newtonWorld.SetSubSteps(propertyGroup.solverSubSteps)
        newtonWorld.SetTimestep(1.0 / propertyGroup.solverNominalFps)
        newtonWorld.SetIterations(propertyGroup.solverIterations)
        return {'FINISHED'}
