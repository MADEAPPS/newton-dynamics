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
    
    # 1. Create the root element
    root = ET.Element("ndMesh")
    
    
    # 4. Create an ElementTree object from the root element
    tree = ET.ElementTree(root)
    
    # 5. Write the tree to an XML file
    tree.write(filepath, encoding="utf-8", xml_declaration=True)
    
    # 6. (debug) Print the XML to the console
    print(ET.tostring(root, encoding="utf-8").decode())
    
    return {'FINISHED'}
