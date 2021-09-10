# Copyright (c) <2003-2021> <Newton Game Dynamics>
 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

import bpy
import newton

class NewtonWorldProperties(bpy.types.PropertyGroup):
    solverSubSteps: bpy.props.IntProperty(name= "solver substeps", description="number of solver sub step per ticks", default = 2, min=0, max=8)
    solverIterations: bpy.props.IntProperty(name= "solver iterations", description="Set the number of solver iterations per sub step", default = 4, min=4, max=16)
    
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

class NewtonWorld(bpy.types.Object):
    """create an interface to the newton workd"""

    def __init__(self, object):
        #print ("create manager one once")
        self.name = 'newton_world'
        self.world = newton.ndWorld()

        #self.iterations = self.world.GetSolverIterations()
        #self.SetIterations(10)

    def SetIterations (self, iterations):
        self.world.SetSolverIterations(iterations)

class NewtonWorldCreateHomeObject(bpy.types.Operator):
    """Creates a newton world home"""
    bl_label = 'create newton world'
    bl_idname = 'view3d.newton_world_create_home'
    bl_description = "create newton world"

    def execute(self, context):
        scene = context.scene
        selectedObjec = context.active_object
        bpy.ops.mesh.primitive_cube_add(size=1, enter_editmode=False, align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))
        if selectedObjec == context.active_object:
            print ('change to [object mode] operator canceled')
            return {'CANCELLED'}

        context.active_object.name = 'newtonHome'
        world = NewtonWorld(context.active_object)

        print ("test class NewtonWorld")
        # this line works
        world.SetIterations(10)

        # this line fail in blender but not in python
        scene.newton_world = world
        scene.newton_world.SetIterations(10)

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
        world = NewtonWorld(context.active_object)
        scene.newton_world = world
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

class NewtonWorldSetProperty(bpy.types.Operator):
    """newton world set engine properties"""
    bl_label = 'newton world set property'
    bl_idname = 'view3d.newton_world_set_property'
    bl_description = "newton world set property"

    def findWorld(self):
        col = bpy.data.collections.get("Collection")
        if col:
            for obj in col.objects:
                if obj.name == 'newton_world':
                    print ("esta es una pura mierda 0")
                    return obj
        return None


    def execute(self, context):
        scene = context.scene
        world = self.findWorld()
        propertyGroup = scene.newton_world_properties
        world.SetIterations(propertyGroup.solverIterations)
        return {'FINISHED'}
