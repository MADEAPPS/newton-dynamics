import bpy
import math
import bmesh
import mathutils
import xml.etree.ElementTree as ET

def LoadMesh(context, filepath, use_some_setting):
    tree = ET.parse(filepath)
    root = tree.getroot()
    mesh = ParseNode(context, root, None)

    scale = mathutils.Vector((1.0, 1.0, 1.0))
    location = mathutils.Vector((0.0, 0.0, 0.0))
    rotation = mathutils.Euler((math.radians(90.0), math.radians(0.0), math.radians(-90.0)), 'XYZ')
    TransformModel(mesh, mathutils.Matrix.LocRotScale(location, rotation, scale))
    return {'FINISHED'}

def TransformModel(node, rotation):
    transposeRotation = rotation.copy()
    transposeRotation.invert()
    node.matrix_basis = rotation @ node.matrix_basis @ transposeRotation
    
    for vert in node.data.vertices:
        vert.co = rotation @ vert.co
    
    for child in node.children:
        TransformModel(child, rotation)
        
def CalculateTransform(xmlMatrix):
    posit = [float(x) for x in xmlMatrix.find('posit').get('float3').split()]
    angles = [float(x) for x in xmlMatrix.find('angles').get('float3').split()]
    scale = mathutils.Vector((1.0, 1.0, 1.0))
    location = mathutils.Vector((posit[0], posit[1], posit[2]))
    rotation = mathutils.Euler((math.radians(angles[0]), math.radians(angles[1]), math.radians(angles[2])), 'XYZ')
    return mathutils.Matrix.LocRotScale(location, rotation, scale)

def ParseVertices(meshObj, xmlVertices, xmlFaces):
    indices = [int(x) for x in xmlVertices.find('indices').get('intArray').split()]
    faces = [int(x) for x in xmlFaces.find('faceIndexCount').get('intArray').split()]
    posit = [float(x) for x in xmlVertices.find('positions').get('float3Array').split()]
    
    for i in range(0, len(posit), 3):
        x = posit[i + 0]
        y = posit[i + 1]
        z = posit[i + 2]
        meshObj.verts.new ((x, y, z))
        
    # Ensure the lookup table is updated for vertex indexing
    meshObj.verts.ensure_lookup_table()        

    indexCount = 0
    for i in range(0, len(faces), 1):
        meshFace = []        
        count = faces[i]
        for j in range(0, count, 1):
            index = indices[indexCount + j]
            meshFace.append(meshObj.verts[index])
        meshObj.faces.new(meshFace)                    
        indexCount = indexCount + count;        

def ParseGeomentry(nodeData, xmlNode):
    mesh = bmesh.new()
    ParseVertices(mesh, xmlNode.find('vertices'), xmlNode.find('faces'))
    
    mesh.to_mesh(nodeData)
    nodeData.update()
    mesh.free()
        
def ParseNode(context, xmlNode, blenderParentNode):
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