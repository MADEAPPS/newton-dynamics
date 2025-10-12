import bpy
import math
import bmesh
import mathutils
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
    
def CalculateTransform(xmlMatrix):
    posit = [float(x) for x in xmlMatrix.find('posit').get('float3').split()]
    angles = [float(x) for x in xmlMatrix.find('angles').get('float3').split()]

    scale = mathutils.Vector((1.0, 1.0, 1.0))
    location = mathutils.Vector((posit[0], posit[1], posit[2]))
    rotation = mathutils.Euler((math.radians(angles[0]), math.radians(angles[1]), math.radians(angles[2])), 'XYZ')
    return mathutils.Matrix.LocRotScale(location, rotation, scale)

def ParseVertices(meshObj, xmlVertices):
    #positions = xmlVertices.find('vertices')
    #indices = xmlVertices.find('indices')
    indices = [int(x) for x in xmlVertices.find('indices').get('intArray').split()]
    posit = [float(x) for x in xmlVertices.find('positions').get('float3Array').split()]
    print(posit)
    print(indices)


def ParseGeomentry(nodeData, xmlNode):
    mesh = bmesh.new()
    ParseVertices(mesh, xmlNode.find('vertices'))
    
    
    mesh.to_mesh(nodeData)
    nodeData.update()
    mesh.free()
   
    
        
def ParseNode(context, xmlNode, blenderParentNode):
    #TestShit(meshNode)
    
    #create a geometry and a mesh node and link it to the scene and parent
    data = bpy.data.meshes.new(xmlNode.find('name').get('string'))
    node = bpy.data.objects.new(xmlNode.find('name').get('string'), data);
    
    node.parent_type = 'OBJECT'
    node.parent = blenderParentNode
    bpy.context.scene.collection.objects.link(node)
    
    #set the transform
    node.matrix_basis = CalculateTransform(xmlNode.find('matrix'))
    
    xmlGeometry = xmlNode.find('ndGeometry')
    if (xmlGeometry != None):
        ParseGeomentry(data, xmlGeometry)
    
    # add all the children nodes
    for book in xmlNode.findall('ndMesh'):
        ParseNode(context, book, node)
        
    return node