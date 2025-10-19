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

def SaveNodeDataGeometry(context, xmlNode, blenderNode, matrix):
    xmlGeomtry = ET.SubElement(xmlNode, "geometry")
    
    objectData = blenderNode.data

    mesh = bmesh.new()
    mesh.from_mesh(objectData)
    
    #rotate and save vertex list
    indexList = []
    vertexList = []
    faceIndexList = []
    for face in mesh.faces:
        faceIndexList.append(len(face.verts))
        for vert in face.verts:
            indexList.append(vert.index)
    
    #for vert in objectData.vertices:
    for vert in mesh.verts:
        #p = vert.co @ matrix
        p = matrix @ vert.co
        vertexList.append(p.x)
        vertexList.append(p.y)
        vertexList.append(p.z)

    xmlPosition = ET.SubElement(xmlGeomtry, "vertices")
    xmlPositions = ET.SubElement(xmlPosition, "positions")
    xmlPositIndices = ET.SubElement(xmlPosition, "indices")

    xmlFaces = ET.SubElement(xmlGeomtry, "faces")
    mlFaceMaterial = ET.SubElement(xmlFaces, "faceMaterial")
    xmlFaceIndexCount = ET.SubElement(xmlFaces, "faceIndexCount")
        
    #save the vertex array
    pointString = ' '.join(map(str, vertexList))
    xmlPositions.set('count', str(len(mesh.verts)))
    xmlPositions.set('float3Array', pointString)
    
    # save the vertex Index array 
    indexString = ' '.join(map(str, indexList))
    xmlPositIndices.set('count', str(len(indexList)))
    xmlPositIndices.set('intArray', indexString)

    # save face index
    faceIndexString = ' '.join(map(str, faceIndexList))
    xmlFaceIndexCount.set('count', str(len(faceIndexList)))
    xmlFaceIndexCount.set('intArray', faceIndexString)
    
    # save material ....
    
    # Free the BMesh data when done
    mesh.free()
    
def SaveNodeDataTransfrom(context, xmlNode, blenderNode, rotation):
    xmlMatrix = ET.SubElement(xmlNode, "matrix")
    
    transposeRotation = rotation.copy()
    transposeRotation.invert()
    matrix = rotation @ blenderNode.matrix_basis @ transposeRotation
    
    location, quaternion, scale = matrix.decompose()
    eulers = quaternion.to_euler('XYZ')
    degrees = mathutils.Vector((math.degrees(eulers.x), math.degrees(eulers.y), math.degrees(eulers.z)))
    
    xmlPosit = ET.SubElement(xmlMatrix, "posit")
    xmlPosit.set('float3', ' '.join(map(str, location)))

    xmlAngles = ET.SubElement(xmlMatrix, "angles")
    xmlAngles.set('float3', ' '.join(map(str, degrees)))
        
def SaveNodeData(context, xmlNode, blenderNode, matrix):
    objectData = blenderNode.data
    if (objectData != None):
        xmlName = ET.SubElement(xmlNode, "name")
        xmlName.set('string', objectData.name)
        
        SaveNodeDataTransfrom(context, xmlNode, blenderNode, matrix)
        
        vertices = objectData.vertices
        if (vertices != None):
            SaveNodeDataGeometry(context, xmlNode, blenderNode, matrix)

def SaveNodeDataRecurse(context, xmlNode, blenderNode, matrix):
    SaveNodeData(context, xmlNode, blenderNode, matrix)
    
    for blenderChild in blenderNode.children:
        xmlChild = ET.SubElement(xmlNode, "ndMesh")
        SaveNodeDataRecurse(context, xmlChild, blenderChild, matrix)