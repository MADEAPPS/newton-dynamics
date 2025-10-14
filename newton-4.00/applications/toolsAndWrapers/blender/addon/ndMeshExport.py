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
    SaveNodeDataRecurse(context, xmlRoot, object, matrix)
    
    # 4. Create an ElementTree object from the root element
    tree = ET.ElementTree(xmlRoot)
    
    # 5. Write the tree to an XML file
    ET.indent(xmlRoot)
    tree.write(filepath, encoding="utf-8", xml_declaration=True)
    
    # 6. (debug) Print the XML to the console
    #print(ET.tostring(xmlRoot, encoding="utf-8").decode())
    
    return {'FINISHED'}

def SaveNodeDataGeopmentry(context, xmlNode, blenderNode, matrix):
    xmlGeomtry = ET.SubElement(xmlNode, "geometry")

def SaveNodeData(context, xmlNode, blenderNode, matrix):
    objectData = blenderNode.data
    if (objectData != None):
        xmlName = ET.SubElement(xmlNode, "name")
        xmlName.set('string', objectData.name)
        
        xmlMatrix = ET.SubElement(xmlNode, "matrix")
        
        vertices = objectData.vertices
        if (vertices != None):
            SaveNodeDataGeopmentry(context, xmlNode, blenderNode, matrix)

def SaveNodeDataRecurse(context, xmlNode, blenderNode, matrix):
    SaveNodeData(context, xmlNode, blenderNode, matrix)
    
    for blenderChild in blenderNode.children:
        xmlChild = ET.SubElement(xmlNode, "ndMesh")
        SaveNodeDataRecurse(context, xmlChild, blenderChild, matrix)