import bpy
import xml.etree.ElementTree as ET

def LoadMesh(context, filepath, use_some_setting):
    tree = ET.parse(filepath)
    root = tree.getroot()
    mesh = ParseNode(context, root, None)
    return {'FINISHED'}


def TestShit(xmlNode):
    name = xmlNode.find('name')
    print(name.get('string'))
    
    matrix = xmlNode.find('matrix')
    posit = [float(x) for x in matrix.find('posit').get('float3').split()]
    angles = [float(x) for x in matrix.find('angles').get('float3').split()]
    print(posit, angles)
    
def ParseGeomentry(node, xmlNode):
    xmlVertices = xmlNode.find('vertices')
    print(xmlVertices)
    
    
def ParseNode(context, xmlNode, blenderParentNode):
    #TestShit(meshNode)
    
    #create a geometry and a mesh node and link it to the scene and parent
    meshData = bpy.data.meshes.new(xmlNode.find('name').get('string'))
    meshNode = bpy.data.objects.new(xmlNode.find('name').get('string'), meshData);

    meshNode.parent_type = 'OBJECT'
    meshNode.parent = blenderParentNode
    bpy.context.scene.collection.objects.link(meshNode)
    
    xmlGeometry = xmlNode.find('ndGeometry')
    if (xmlGeometry != None):
        ParseGeomentry(meshNode, xmlGeometry)
    
    # add all the children nodes
    for book in xmlNode.findall('ndMesh'):
        ParseNode(context, book, meshNode)
        
    return meshNode