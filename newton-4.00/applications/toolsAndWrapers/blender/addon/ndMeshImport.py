# Copyright (c) <2003-2022> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely

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
    quaternion = mathutils.Euler((math.radians(90.0), math.radians(0.0), math.radians(-90.0)), 'XYZ')
    matrix = mathutils.Matrix.LocRotScale(location, quaternion, scale)
    TransformModel(mesh, matrix)
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
    quaternion = mathutils.Euler((math.radians(angles[0]), math.radians(angles[1]), math.radians(angles[2])), 'XYZ')
    return mathutils.Matrix.LocRotScale(location, quaternion, scale)

def ParseFaces(xmlFaces):
    xmlLayers = xmlFaces.find('faceLayers')
    faces = [int(x) for x in xmlFaces.find('faceIndexCount').get('intArray').split()]

    layersCount = 1
    if (xmlLayers != None):
        layers = [int(x) for x in xmlLayers.get('intArray').split()]
        for x in range(0, len(layers), 1):
            layersCount = max(layers[x] + 1, layersCount) 
    else:
        layers = []
        for i in range(0, len(faces), 1):
            layers.append(int(0))
    return layersCount, layers, faces

def ParseVertices(meshObj, xmlVertices, layersCount, layers, faces):
    indices = [int(x) for x in xmlVertices.find('indices').get('intArray').split()]
    posit = [float(x) for x in xmlVertices.find('positions').get('float3Array').split()]

    #for j  in layersCount, 1: 
    for i in range(0, len(posit), 3):
        x = posit[i + 0]
        y = posit[i + 1]
        z = posit[i + 2]
        meshObj.verts.new ((x, y, z))
        
    # Ensure the lookup table is updated for vertex indexing
    meshObj.verts.ensure_lookup_table()   

    indexCount = 0
    #baseSize = len(posit) / 3
    baseSize = 0
    for i in range(0, len(faces), 1):
        count = faces[i]
        #only load face of layer zero for now.
        if (layers[i] == 0):
            meshFace = []
            baseIndex = int (layers[i] * baseSize)            
            for j in range(0, count, 1):
                index0 = indices[indexCount + j] 
                index = int(index0 + baseIndex)
                meshFace.append(meshObj.verts[index])
            meshObj.faces.new(meshFace)
        indexCount = indexCount + count;        

def ParseNormals(meshObj, xmlNormals, layers, faces):
    indices = [int(x) for x in xmlNormals.find('indices').get('intArray').split()]
    posit = [float(x) for x in xmlNormals.find('normals').get('float3Array').split()]
    print (len(indices), len(posit), len(meshObj.loops))

    #meshObj.use_auto_smooth = True
    indexCount = 0
    custom_normals = []    
    for i in range(0, len(faces), 1):
        count = faces[i]
        if (layers[i] == 0):
            for j in range(0, count, 1):
                index0 = indices[indexCount + j] 
                index = int(index0)
                k = int(indices[index] * 3)
                x = posit[k + 0]
                y = posit[k + 1]
                z = posit[k + 2]
                normal = mathutils.Vector((x, y, z))
                custom_normals.append(normal)
        indexCount = indexCount + count;        

    # asign the face vertex normals        
    meshObj.normals_split_custom_set(custom_normals)

def ParseGeomentry(nodeData, xmlNode):
    
    layersCount, layers, faces = ParseFaces(xmlNode.find('faces'))
            
    #create a mesh and add the facse and vertices
    mesh = bmesh.new()
    #ParseVertices(mesh, xmlNode.find('vertices'), xmlNode.find('faces'))
    ParseVertices(mesh, xmlNode.find('vertices'), layersCount, layers, faces)
    mesh.to_mesh(nodeData)
    nodeData.update()
    mesh.free()
    
    # add the atibutes, normal, uv, weight, ets.
    ParseNormals(nodeData, xmlNode.find('normal'), layers, faces)
        
def ParseNode(context, xmlNode, blenderParentNode):
    #create a geometry and a mesh node and link it to the scene and parent
    data = bpy.data.meshes.new(xmlNode.find('name').get('string'))
    node = bpy.data.objects.new(xmlNode.find('name').get('string'), data);
    
    node.parent_type = 'OBJECT'
    node.parent = blenderParentNode
    bpy.context.scene.collection.objects.link(node)
    
    #set the transform
    node.matrix_basis = CalculateTransform(xmlNode.find('matrix'))
    
    xmlGeometry = xmlNode.find('geometry')
    if (xmlGeometry != None):
        ParseGeomentry(data, xmlGeometry)
    
    # add all the children nodes
    for book in xmlNode.findall('ndMesh'):
        ParseNode(context, book, node)
        
    return node