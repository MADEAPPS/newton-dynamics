#!BPY
"""
Name: 'Newton Alchemedia Format (.xml)'
Blender: 243
Group: 'Export'
Tooltip: 'Export to a Newton Alchemedia file format (.xml).'
"""

# --------------------------------------------------------------------------
# Licence  
# Created:     20/08/2010 
# Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
# License:     
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely
# --------------------------------------------------------------------------

import Blender, bpy, math, pyScene

from Blender import Mesh, Object, sys
from Blender.BGL import *
from Blender.Draw import *
from Blender.Window import *
from Blender.Mathutils import Vector
import struct
from types import *

#global variables
g_filename = Create ("C:/Newton_200/NewtonSDK/samples/bin/test.xml")

# Events
EVENT_NOEVENT			= 1
EVENT_SCENE				= 2
EVENT_CHOOSE_FILENAME	= 3
EVENT_EXIT				= 100


#
# Get scene Node from scene node
#
def GetMeshNode (scene, sceneNode):
	childLink = scene.GetFirstChildLink (sceneNode)
	while childLink != None: 
		childNode = scene.GetNodeFromLink(childLink)
		if scene.IsMeshNode(childNode) == 1:
		   return childNode 
		childLink = scene.GetNextChildLink (sceneNode, childLink)
	return None	   


#
# Create blender empty object from node
#
def CreateBlenderEmptyOject (scene, node, blenderScene): 
	object = blenderScene.objects.new('Empty', scene.GetNodeName(node))
	return object 
	
	
	

#
# export and Blender mesh 
#
def ExportMesh(scene, sceneNode, blenderScene, blenderObjectMesh):
	'''convert a blender mesh to an alchemedia node'''
	
	# get the blender mesh from teh object
	blenderMesh = blenderObjectMesh.data

	# create the mesh node	
	meshNode = scene.CreateMeshNode(sceneNode)
	meshInfo = pyScene.pyMesh(scene, meshNode)
	meshInfo.SetName (blenderObjectMesh.getName())
	
	# add the mesh materials
	hasMaterial = False
	materialMap = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1];
	materials = blenderMesh.getMaterials (1)
	for i, material in enumerate (materials):
		if material != None:
			hasMaterial = True
			materialMap[i] = i;
			materialNode = scene.CreateMaterialNode(meshNode, i)
			materialInfo = pyScene.pyMaterial(scene, materialNode)
			
			# export any texture
			for texture in material.getTextures():
				if texture and texture.tex.type == Blender.Texture.Types.IMAGE:
					image = texture.tex.image
					if image != None:
						# create a texture and attache to the material
						name = image.getName()
						textureNode = scene.CreateTextureNode(name)
						scene.AddReference (materialNode, textureNode)
						
						# link the texture ID to and the materail
						textureInfo = pyScene.pyTexture(scene, textureNode)
						materialInfo.SetAmbientTextId (textureInfo.GetId())
						materialInfo.SetDiffuseTextId (textureInfo.GetId())
					
					
					
			
	
	# if the mesh do not have material add a default material			
	if hasMaterial == False:
		materialMap[0] = 0;
		materialNode = scene.CreateMaterialNode(meshNode, 0)
	
	# create the vertex List and face arrays
	faceCount = len(blenderMesh.faces)	
	vertexCount = len(blenderMesh.verts)
	
	faceIndexCountArray = pyScene.intArray(faceCount)
	vertexArray = pyScene.doubleArray(vertexCount * 3)
	faceMaterialIndexArray = pyScene.intArray(faceCount)	
	vertexIndexArray = pyScene.intArray(faceCount * 4)
	normalArray = pyScene.doubleArray(faceCount * 4 * 3)
	uvArray = pyScene.doubleArray(faceCount * 4 * 2)
	
	# copy vertex points from the vertex list
	for i, v in enumerate(blenderMesh.verts):
		vertexArray[i * 3 + 0] = v.co.x 
		vertexArray[i * 3 + 1] = v.co.y 
		vertexArray[i * 3 + 2] = v.co.z 
	

	# copy face data from the face list
	index = 0	
	if blenderMesh.hasFaceUV() == True:
		# mesh with uv mapping 
		for i, face in enumerate(blenderMesh.faces):
			pointCount = len(face.v)
			faceMaterialIndexArray[i] = face.mat
			faceIndexCountArray[i] = pointCount
			
			for j, point in enumerate(face):
				vertexIndexArray[index + j] = point.index
				
				normalIndex = (index + j) * 3
				normalArray[normalIndex + 0] = point.no.x
				normalArray[normalIndex + 1] = point.no.y
				normalArray[normalIndex + 2] = point.no.z
				
				uvIndex = (index + j) * 2
				
				uvArray[uvIndex + 0] = face.uv[j][0]
				uvArray[uvIndex + 1] = face.uv[j][1]
			index = index + pointCount
	else:
		# mesh have no uv mapping
		for i, face in enumerate(blenderMesh.faces):
			pointCount = len(face.v)
			faceMaterialIndexArray[i] = face.mat
			faceIndexCountArray[i] = pointCount
			
			for j, point in enumerate(face):
				vertexIndexArray[index + j] = point.index
				
				normalIndex = (index + j) * 3
				normalArray[normalIndex + 0] = point.no.x
				normalArray[normalIndex + 1] = point.no.y
				normalArray[normalIndex + 2] = point.no.z
				
				uvIndex = (index + j) * 2
				uvArray[uvIndex + 0] = 0
				uvArray[uvIndex + 1] = 0
			index = index + pointCount
				
	# create the alchemedia mesh
	meshInfo.BuildFromVertexListIndexList(faceCount, faceIndexCountArray, faceMaterialIndexArray, vertexArray, vertexIndexArray, normalArray, uvArray);
	
	# normal are completly bogus in blender, just recalculate proper normals 
	#meshInfo.SmoothNormals (blenderMesh.degr)
	meshInfo.SmoothNormals (30)
	
	
#
# export and Blender mesh 
#
def ExportRigidBody(scene, sceneNode, blenderScene, blenderObject):
	'''convert a blender mesh to an alchemedia node'''
	
	# create the mesh node	
	rigidBody = scene.CreateRigidbodyNode(sceneNode)
	rigidBodyInfo = pyScene.pyRigidBody(scene, rigidBody)
	
	rigidBodyInfo.SetName (blenderObject.getName())
	rigidBodyInfo.SetShape (blenderObject.rbShapeBoundType)
	
	if blenderObject.rbFlags & Blender.Object.RBFlags.RIGIDBODY:
		rigidBodyInfo.SetMass(blenderObject.rbMass)
	
		

	
#
# Export all hierarchy of objects
#
def	ExportObject(scene, parentNode, blenderScene, blenderObject):
	''' recusivally load convert a blender scene to an alchemedia node'''

	# create an alchemedia scene node
	node = scene.CreateSceneNode(parentNode)
	sceneObject = pyScene.pyObject(scene, node);
	
	sceneObject.SetName (blenderObject.getName())
	
	# save teh transformation matrix
	(x, y, z) = blenderObject.getLocation('worldspace')
	(SizeX, SizeY, SizeZ) = blenderObject.getSize('worldspace')
	euler = blenderObject.getEuler('worldspace')
	sceneObject.SetMatrix(x, y, z, euler.x, euler.y, euler.z, SizeX, SizeY, SizeZ);

	# if this object has a mesh, export the mesh
	if blenderObject.getType() == 'Mesh':
		ExportMesh (scene, node, blenderScene, blenderObject)
		
	# if thsi object is a rigif body export body
	isRigBody = blenderObject.rbFlags & (Blender.Object.RBFlags.RIGIDBODY | Blender.Object.RBFlags.BOUNDS)
	#isNotRigBody = blenderObject.rbFlags & Blender.Object.RBFlags.GHOST
	if isRigBody and blenderObject.isSoftBody == False:
		ExportRigidBody(scene, node, blenderScene, blenderObject)
			
			
	# it appears blender does not has interface for getting the children of an object
	# I need to do a brute for scan here
	for object in blenderScene.objects: 
		if object.getParent() == blenderObject:
			type = object.getType()
			if type == 'Mesh' or type == 'Empty':
				ExportObject(scene, node, blenderScene, object)
	

#
# implement main scene loader function
#
def SaveAlchemediaScene(filename):
	''' Save a newton Alchemedia file '''
	
	#create an empty scene
	scene = pyScene.pyScene()
	
	# get the active blender scene
	blenderScene = bpy.data.scenes.active
	
	#iterate over all object saarching for root nodes
	for object in blenderScene.objects: 	
		if object.getParent() == None:
			type = object.getType()
			if type == 'Mesh' or type == 'Empty':
				ExportObject(scene, None, blenderScene, object)
				
	scene.Save (filename)




######################################################
# Callbacks for Window functions
######################################################
def filename_callback(input_filename):
	global g_filename
	g_filename.val = input_filename


######################################################
# GUI Loader
######################################################
def draw_gui():
	global g_filename
	global EVENT_NOEVENT, EVENT_SCENE, EVENT_CHOOSE_FILENAME, EVENT_EXIT

	########## Titles
	glClear(GL_COLOR_BUFFER_BIT)
	glRasterPos2d(8, 100)
	Text("Alchemedia Exporter")

	# File name chooser Button
	BeginAlign()
	filename = String("file to save: ", EVENT_NOEVENT, 10, 55, 410, 18, g_filename.val, 255, "Alchemdia xml file to save")
	Button("Browse", EVENT_CHOOSE_FILENAME, 420, 55, 80, 18)
	EndAlign()
	
	# Load and Exit Buttons
	Button("Save", EVENT_SCENE, 10, 10, 80, 18)
	Button("Cancel", EVENT_EXIT, 170, 10, 80, 18)

def event(evt, val):	
	if (evt == QKEY and not val):
		Blender.Draw.Exit()

def bevent(evt):
	global g_filename
	global EVENT_NOEVENT, EVENT_SCENE, EVENT_CHOOSE_FILENAME, EVENT_EXIT

	# Manages GUI events
	if (evt == EVENT_EXIT):
		Blender.Draw.Exit()
	elif (evt == EVENT_CHOOSE_FILENAME):
		FileSelector(filename_callback, "Alchemedia XML File Selection")
	elif (evt == EVENT_SCENE):
		SaveAlchemediaScene(g_filename.val)
		Blender.Redraw()
		Blender.Draw.Exit()
		return
	

if __name__ == '__main__':
	Register(draw_gui, event, bevent)
