import bpy
import math
import bmesh
import mathutils
import xml.etree.ElementTree as ET

def SaveMesh(context, filepath):
    scale = mathutils.Vector((1.0, 1.0, 1.0))
    location = mathutils.Vector((0.0, 0.0, 0.0))
    eulers = mathutils.Euler((math.radians(90.0), math.radians(0.0), math.radians(-90.0)), 'XYZ')
    matrix = mathutils.Matrix.LocRotScale(location, eulers, scale)
    matrix.invert()
    
    # Exit edit mode before exporting, so current object states are exported properly.
    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')

    #parse blender selected node and all it children
    object = context.object
    xmlRoot = ET.Element("ndMesh")
    ParseDataRecurse(context, xmlRoot, object, matrix)
    
    # 4. Create an ElementTree object from the root element
    tree = ET.ElementTree(xmlRoot)
    
    # 5. Write the tree to an XML file
    tree.write(filepath, encoding="utf-8", xml_declaration=True)
    
    # 6. (debug) Print the XML to the console
    print(ET.tostring(xmlRoot, encoding="utf-8").decode())
    
    return {'FINISHED'}


def ParseDataRecurse(context, xmlNode, blenderNode, matrix):
    objectData = blenderNode.data
    if (objectData != None):
        print(objectData.name)
    
    for child in blenderNode.children:    
        ParseDataRecurse(context, xmlNode, child, matrix)