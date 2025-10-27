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

def LoadMesh(context, filepath):
    tree = ET.parse(filepath)
    xmlRoot = tree.getroot()
    
    index = filepath.rfind("\\")
    path = filepath[:index]
    
    #create the hierchical mesh and static geometries
    bonesTarget = {}
    rootMesh = ParseNode(context, path, xmlRoot, None, bonesTarget)
    
    # add gemometry modieres
    armatureList = []
    ParseGeometryModifiers (xmlRoot, rootMesh, bonesTarget, armatureList)

    scale = mathutils.Vector((1.0, 1.0, 1.0))
    location = mathutils.Vector((0.0, 0.0, 0.0))
    quaternion = mathutils.Euler((math.radians(90.0), math.radians(0.0), math.radians(-90.0)), 'XYZ')
    matrix = mathutils.Matrix.LocRotScale(location, quaternion, scale)
    
    TransformModel(rootMesh, matrix)
    for skin in armatureList:
        TransformArmature(matrix, skin)
    
    return {'FINISHED'}

def TransformModel(node, rotation):
    transposeRotation = rotation.copy()
    transposeRotation.invert()
    node.matrix_basis = rotation @ node.matrix_basis @ transposeRotation
    
    for vert in node.data.vertices:
        vert.co = rotation @ vert.co

    # I do not understand it but it seems blender rotaion the normal automatically
    #custom_normals = []
    #for poly in node.data.polygons:
    #    for loop_index in poly.loop_indices:
    #        loop = node.data.loops[loop_index]
    #        #loop.normal = rotation @ loop.normal
    #        custom_normals.append(rotation @ loop.normal)
    #node.data.normals_split_custom_set(custom_normals)
    
    for child in node.children:
        TransformModel(child, rotation)
        
def TransformArmature(rotation, armature):
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = armature
    armature.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    
    for bone in armature.data.edit_bones:
        head = rotation @ bone.head
        tail = rotation @ bone.tail
        bone.head = head
        bone.tail = tail
    
    bpy.ops.object.mode_set(mode='OBJECT')                           

        
def CalculateTransform(xmlMatrix):
    posit = [float(x) for x in xmlMatrix.find('posit').get('float3').split()]
    angles = [float(x) for x in xmlMatrix.find('angles').get('float3').split()]
    scale = mathutils.Vector((1.0, 1.0, 1.0))
    location = mathutils.Vector((posit[0], posit[1], posit[2]))
    quaternion = mathutils.Euler((math.radians(angles[0]), math.radians(angles[1]), math.radians(angles[2])), 'XYZ')
    return mathutils.Matrix.LocRotScale(location, quaternion, scale)

def ParseFaces(xmlFaces):
    faces = [int(x) for x in xmlFaces.find('faceIndexCount').get('intArray').split()]
    faceMaterials = [int(x) for x in xmlFaces.find('faceMaterial').get('intArray').split()]
    
    layersCount = 1
    xmlLayers = xmlFaces.find('faceLayers')
    if (xmlLayers != None):
        layers = [int(x) for x in xmlLayers.get('intArray').split()]
        for x in range(0, len(layers), 1):
            layersCount = max(layers[x] + 1, layersCount) 
    else:
        layers = []
        for i in range(0, len(faces), 1):
            layers.append(int(0))
    return layersCount, layers, faces, faceMaterials

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

    indexCount = 0
    custom_normals = []
    for i in range(0, len(faces), 1):
        count = faces[i]
        if (layers[i] == 0):
            for j in range(0, count, 1):
                index0 = indices[indexCount + j] 
                index = int(index0)
                k = int(index * 3)
                x = posit[k + 0]
                y = posit[k + 1]
                z = posit[k + 2]
                normal = mathutils.Vector((x, y, z))
                custom_normals.append(normal)
        indexCount = indexCount + count;        

    # asign the face vertex normals        
    meshObj.normals_split_custom_set(custom_normals)

def ParseUvs(meshObj, xmlUVs, layers, faces):
    indices = [int(x) for x in xmlUVs.find('indices').get('intArray').split()]
    posit = [float(x) for x in xmlUVs.find('uv').get('float3Array').split()]

    indexCount = 0
    custom_uv = []
    for i in range(0, len(faces), 1):
        count = faces[i]
        if (layers[i] == 0):
            for j in range(0, count, 1):
                index0 = indices[indexCount + j] 
                index = int(index0)
                k = int(index * 3)
                x = posit[k + 0]
                y = posit[k + 1]
                z = posit[k + 2]
                uv = mathutils.Vector((x, y, z))
                custom_uv.append(uv)
        indexCount = indexCount + count;        
        
    meshObj.uv_layers.new(name="UVMap")
    uv_layer = meshObj.uv_layers.active
    
    for face, loop in enumerate(meshObj.loops):
        uv_layer.data[face].uv = (custom_uv[face].x, custom_uv[face].y)
        
def ParseMaterials(meshObj, path, xmlMaterials, layers, faceMaterials):
    for xmlMaterial in xmlMaterials.findall('material'): 
        materialName = xmlMaterial.find('name').get('string')
        material = bpy.data.materials.new(name=materialName)
        
        material.use_nodes = True
        nodes = material.node_tree.nodes
        
        # I do not really understand how to map the Blinn Paramaters to a blender Principled BSDF material
        #principled_bsdf = nodes.get("Principled BSDF")
        #principled_bsdf.inputs["Metallic"].default_value = 0.1
        #principled_bsdf.inputs["Roughness"].default_value = 0.7
        materialOutput = nodes.get("Material Output")
       
        #create the diffuse texture node
        textureName = xmlMaterial.find('texture').get('string')
        texturePath = path + "\\" + textureName
        image = bpy.data.images.load(texturePath)
        textureNode = nodes.new(type='ShaderNodeTexImage')
        textureNode.image = image
        textureNode.location = (-600, 50)
        
        # Create a new Vector Math node and set the operation
        # Set the operation of the Vector Math node (e.g., ADD, SUBTRACT, MULTIPLY, DOT_PRODUCT, CROSS_PRODUCT, NORMALIZE, etc.)
        diffuseColor = [float(x) for x in xmlMaterial.find('diffuse').get('float3').split()]
        vectorMathNode = nodes.new(type='ShaderNodeVectorMath')
        vectorMathNode.operation = 'MULTIPLY'   
        vectorMathNode.name = "diffuseColor"
        vectorMathNode.inputs[0].default_value = (1.0, 1.0, 1.0) # Vector A
        vectorMathNode.inputs[1].default_value = (diffuseColor[0], diffuseColor[1], diffuseColor[2]) # Vector B
        vectorMathNode.location = (-300, 50) 
        #print (vectorMathNode.name)
        
        # create the specular shader
        opacity = float(xmlMaterial.find('opacity').get('float'))
        shaderSpecularNode = nodes.new(type='ShaderNodeEeveeSpecular')
        #shaderSpecularNode.transparency  = 1.0 - opacity
        shaderSpecularNode.inputs["Transparency"].default_value = 1.0 - opacity
        shaderSpecularNode.location = (-50, 70)

        material.node_tree.links.new(textureNode.outputs['Color'], vectorMathNode.inputs[0])
        material.node_tree.links.new(vectorMathNode.outputs['Vector'], shaderSpecularNode.inputs['Base Color'])
        material.node_tree.links.new(shaderSpecularNode.outputs['BSDF'], materialOutput.inputs['Surface'])
       
        for node in nodes:
            if node.type == 'BSDF_PRINCIPLED':
                nodes.remove(node)        
                break

        meshObj.materials.append(material)

    # assign this material to the faces.
    for faceIndex in range(0, len(meshObj.polygons), 1):
        materialIndex = int(faceMaterials[faceIndex])
        meshObj.polygons[faceIndex].material_index = materialIndex

def ParseGeomentry(node, nodeData, path, xmlNode):
    
    layersCount, layers, faces, faceMaterials = ParseFaces(xmlNode.find('faces'))
            
    #create a mesh and add the facse and vertices
    mesh = bmesh.new()
    xmlVerticesNode = xmlNode.find('vertices')
    ParseVertices(mesh, xmlVerticesNode, layersCount, layers, faces)
    mesh.to_mesh(nodeData)
    nodeData.update()
    mesh.free()
    
    # add the attributes: material, normal, uv, ets.
    ParseMaterials(nodeData, path, xmlNode, layers, faceMaterials)
    ParseNormals(nodeData, xmlNode.find('normal'), layers, faces)
    ParseUvs(nodeData, xmlNode.find('uvs'), layers, faces)
        
def ParseNode(context, path, xmlNode, blenderParentNode, bonesTarget):
    #create a geometry and a mesh node and link it to the scene and parent
    nodeName = xmlNode.find('name').get('string')
    data = bpy.data.meshes.new(nodeName)
    node = bpy.data.objects.new(nodeName, data)
    
    node.parent_type = 'OBJECT'
    node.parent = blenderParentNode
    bpy.context.scene.collection.objects.link(node)
    
    #set the transform
    node.matrix_basis = CalculateTransform(xmlNode.find('matrix'))
    
    #get the bone target position
    target = [float(x) for x in xmlNode.find('type').get('target').split()]
    bonesTarget[nodeName] = target
    
    #print (nodeName)
    xmlGeometry = xmlNode.find('geometry')
    if (xmlGeometry != None):
        ParseGeomentry(node, data, path, xmlGeometry)
    
    # add all the children nodes
    for child in xmlNode.findall('ndMesh'):
        ParseNode(context, path, child, node, bonesTarget)
    return node

def ParseVerticesWeights(meshNode, xmlVertices, layersCount):
    xmlVertexGroupSet = xmlVertices.find('vertexWeights')
    if (xmlVertexGroupSet != None):
        
        # create an amature for all the bones
        armature_data = bpy.data.armatures.new(meshNode.data.name)
        armature_object = bpy.data.objects.new(meshNode.data.name, armature_data)
        bpy.context.collection.objects.link(armature_object)
        
        # set the per vertex skinning weights
        boneNames = ["bone0", "bone1", "bone2", "bone3"]
        boneWeights = ["weight0", "weight1", "weight2", "weight3"]
        
        #add all the vertex groups
        for xmlVertexGroup in xmlVertexGroupSet.findall('vert'):
            vertexIndex = int(xmlVertexGroup.get('vertID'))
            for i in range(0, len(boneNames), 1):
                groupName = xmlVertexGroup.get(boneNames[i])
                if (groupName != None):
                    if groupName not in meshNode.vertex_groups:
                        meshNode.vertex_groups.new(name=groupName)
                    vertexGroup = meshNode.vertex_groups[groupName]
                    weightValue = float(xmlVertexGroup.get(boneWeights[i]))
                    vertexGroup.add([vertexIndex], weightValue, 'REPLACE')

def BuildAmatureSkeleton(armatureObject, meshNode, boneNameList, parentBone, boneTargetDictionary, parentMatrix):
    armatureMatrix = parentMatrix @ meshNode.matrix_basis
    try:
        index = boneNameList.index(meshNode.data.name)
        amatureBoneList = armatureObject.edit_bones
        newBone = amatureBoneList.new(boneNameList[index])
        if parentBone != None:
            newBone.parent = parentBone

        localTarget = boneTargetDictionary[meshNode.data.name]
        origin = armatureMatrix.translation
        target = armatureMatrix @ mathutils.Vector((localTarget[0], localTarget[1], localTarget[2], 1.0))
        
        newBone.head = (origin.x, origin.y, origin.z)
        newBone.tail = (target.x, target.y, target.z)
            
    except ValueError:
        newBone = parentBone
    
    for child in meshNode.children:
        BuildAmatureSkeleton(armatureObject, child, boneNameList, newBone, boneTargetDictionary, armatureMatrix)

def ParseAmatures(xmlNode, meshNode, rootNode, boneTargetDictionary, armatureList):
    xmlVertices = xmlNode.find('vertices')
    xmlVertexGroupSet = xmlVertices.find('vertexWeights')
    if (xmlVertexGroupSet != None):
        boneNames = ["bone0", "bone1", "bone2", "bone3"]
        boneWeights = ["weight0", "weight1", "weight2", "weight3"]

        # create an amature for all the bones
        armatureData = bpy.data.armatures.new(meshNode.data.name)
        armatureObject = bpy.data.objects.new(meshNode.data.name, armatureData)
        bpy.context.collection.objects.link(armatureObject)
        
        # Add an Armature modifier to the mesh, this is making too hard to add bone
        armature_modifier = meshNode.modifiers.new(name='Armature', type='ARMATURE')
        armature_modifier.object = armatureObject
        
        #save amature for post processing 
        armatureList.append(armatureObject)
        
        # generate the bone list
        boneNameList = []
        for xmlVertexGroup in xmlVertexGroupSet.findall('vert'):
            for i in range(0, len(boneNames), 1):
                groupName = xmlVertexGroup.get(boneNames[i])
                if (groupName != None):
                   if groupName not in meshNode.vertex_groups:
                       meshNode.vertex_groups.new(name=groupName)
                       boneNameList.append(groupName)    

        bpy.ops.object.select_all(action='DESELECT')
        bpy.context.view_layer.objects.active = armatureObject
        armatureObject.select_set(True)
        bpy.ops.object.mode_set(mode='EDIT')
        
        BuildAmatureSkeleton(armatureData, rootNode, boneNameList, None, boneTargetDictionary, mathutils.Matrix.Identity(4))
        bpy.ops.object.mode_set(mode='OBJECT')                           

        # set the per vertex skinning weights
        for xmlVertexGroup in xmlVertexGroupSet.findall('vert'):
            vertexIndex = int(xmlVertexGroup.get('vertID'))
            for i in range(0, len(boneNames), 1):
                groupName = xmlVertexGroup.get(boneNames[i])
                if (groupName != None):
                    vertexGroup = meshNode.vertex_groups[groupName]
                    weightValue = float(xmlVertexGroup.get(boneWeights[i]))
                    vertexGroup.add([vertexIndex], weightValue, 'REPLACE')


def ParseGeometryModifiers (xmlNode, rootMeshNode, boneTargetDictionary, armatureList):
    xmlGeometry = xmlNode.find('geometry')
    if (xmlGeometry != None):
        nodeName = xmlNode.find('name').get('string')
        meshNode = bpy.data.objects.get(nodeName)
        ParseAmatures(xmlGeometry, meshNode, rootMeshNode, boneTargetDictionary, armatureList)
    
    # add all the children nodes
    for xmlChild in xmlNode.findall('ndMesh'):
        ParseGeometryModifiers(xmlChild, rootMeshNode, boneTargetDictionary, armatureList)
        
