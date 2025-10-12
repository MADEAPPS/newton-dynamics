import bpy
import xml.etree.ElementTree as ET

def LoadMesh(context, filepath, use_some_setting):
    tree = ET.parse(filepath)
    root = tree.getroot()
    ParseNode(context, root)
    return {'FINISHED'}


def ParseNode(context, meshNode):
    name = meshNode.find('name')
    print(name.get('string'))
    
    matrix = meshNode.find('matrix')
    posit = matrix.find('posit')
    angles = matrix.find('angles')
    print(posit.get('float3'))
    print(angles.get('float3'))
    
    for book in meshNode.findall('ndMesh'):
        ParseNode(context, book)