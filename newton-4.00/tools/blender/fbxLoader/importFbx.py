# --------------------------------------------------------------------------
# Copyright (c) <2003-2021> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely
# --------------------------------------------------------------------------


##!BPY
#"""
#Name: 'Newton 4.00 (.fbx)'
#Blender: 293
#Group: 'Import'
#Tooltip: 'Import fbx file using Newton tool (.fbx).'
#"""
#
#
## --------------------------------------------------------------------------
## Copyright (c) <2003-2021> <Newton Game Dynamics>
## 
## This software is provided 'as-is', without any express or implied
## warranty. In no event will the authors be held liable for any damages
## arising from the use of this software.
## 
## Permission is granted to anyone to use this software for any purpose,
## including commercial applications, and to alter it and redistribute it
## freely
## --------------------------------------------------------------------------
#
##import Blender, bpy, math, pyScene
#import bpy
#
#from Blender import Mesh, Object, Material, Texture, Image, sys
#from Blender.BGL import *
#from Blender.Draw import *
#from Blender.Window import *
#from Blender.Mathutils import Vector
#import os
#import struct
#from types import *
#
#
##global variables
#g_filename = Create ("C:/tmp/test.fbx")
#g_textureMap = {}
#
## Events
#EVENT_NOEVENT			= 1
#EVENT_SCENE				= 2
#EVENT_CHOOSE_FILENAME	= 3
#EVENT_EXIT				= 100
#
#
###
### Get scene Node from scene node
###
##def GetMeshNode (scene, sceneNode):
##	childLink = scene.GetFirstChildLink (sceneNode)
##	while childLink != None: 
##		childNode = scene.GetNodeFromLink(childLink)
##		if scene.IsMeshNode(childNode) == True:
##		   return childNode 
##		childLink = scene.GetNextChildLink (sceneNode, childLink)
##	return None	   
##
##
###
### Create blender Texture object from node
###
##def CreateBlenderTextureFromNode (scene, textureNode, blenderScene, path): 
##	tex = Texture.New(scene.GetNodeName(textureNode))
##	
##	texture = pyScene.pyTexture (scene, textureNode)
##	imageName = path + '/' + texture.GetImageName()
##	image = Blender.Image.Load(imageName)
##	tex.setImage(image) 
##	return tex
##
##
###
### Create blender Material object from node
###
##def CreateBlenderMaterialFromNode (scene, materialNode, blenderScene): 
##	mat = Material.New(scene.GetNodeName(materialNode))
##
##	# load the textures for this material	
##	childLink = scene.GetFirstChildLink (materialNode)
##	while childLink != None: 
##		textureNode = scene.GetNodeFromLink(childLink)
##		if scene.IsTextureNode(textureNode) == True:
##			sourceTexture = pyScene.pyTexture(scene, textureNode)
##			texture = g_textureMap[sourceTexture.GetId()]
##			#mat.setTexture(0, texture) 
##			mat.setTexture(0, texture, Texture.TexCo.UV, Texture.MapTo.COL)
##		childLink = scene.GetNextChildLink (materialNode, childLink)
##	
##	return mat
##
###
### Create blender empty object from node
###
##def CreateBlenderEmptyOject (scene, node, blenderScene): 
##	object = blenderScene.objects.new('Empty', scene.GetNodeName(node))
##	return object 
##
###
### Create blender mesh object from node
###
##def CreateBlenderMeshObjectFromNode (scene, meshNode, blenderScene): 
##	# Get a alchemedia mesh for the node and buidl a blender mesh
##	mesh = pyScene.pyMesh (scene, meshNode)	
##	
##	# create a blender mesh
##	newMesh = bpy.data.meshes.new(scene.GetNodeName(meshNode)) 		
##	
##	# Create all verte and face data
##	vertexList = []		
##	vertexCount = mesh.GetVertexCount()
##	for i in range(0, vertexCount):
##		vertex = mesh.GetVertex (i)
##		vertexList.append([vertex.x, vertex.y, vertex.z]) 
##		
##	faceList = []
##	faceNode = mesh.GetFirstTriangle()
##	while faceNode != None:
##		face = mesh.GetTriangle (faceNode);
##		faceList.append([face.p0.vertex, face.p1.vertex, face.p2.vertex]) 
##		faceNode = mesh.GetNextTriangle(faceNode)
##
##	newMesh.verts.extend(vertexList)         # add vertices to mesh
##	newMesh.faces.extend(faceList)           # add faces to the mesh (also adds edges)
##
##	# create all materials from this mesh
##	materialIndex = 0
##	materialMap = {}
##	meshMaterials = newMesh.materials
##	childLink = scene.GetFirstChildLink (meshNode)
##	while childLink != None: 
##		childNode = scene.GetNodeFromLink(childLink)
##		if scene.IsMaterialNode(childNode) == True:
##			# make a blender material and a alchemdia material
##			material = CreateBlenderMaterialFromNode (scene, childNode, blenderScene) 	
##			meshMaterials.append(material)
##			
##			# add a map key for asigning faces
##			sourceMaterial = pyScene.pyMaterial(scene, childNode)
##			materialMap[sourceMaterial.GetId()] = materialIndex
##			
##			materialIndex = materialIndex + 1
##		childLink = scene.GetNextChildLink (meshNode, childLink)
##	newMesh.materials = meshMaterials
##
##	
##	# In a secund pass asign material and uv mapping to faces
##	faceIndex = 0
##	newMesh.faceUV= True
##	faceNode = mesh.GetFirstTriangle()
##	while faceNode != None:
##		face = mesh.GetTriangle (faceNode);
##		newMesh.faces[faceIndex].mat = materialMap[face.materialIndex]
##
##		uv0 = mesh.GetUV0(face.p0.uv0)
##		uv1 = mesh.GetUV0(face.p1.uv0)
##		uv2 = mesh.GetUV0(face.p2.uv0)
##		newMesh.faces[faceIndex].uv = [Vector(uv0.x, uv0.y), Vector(uv1.x, uv1.y), Vector(uv2.x, uv2.y)]
##		
##		faceIndex = faceIndex + 1
##		faceNode = mesh.GetNextTriangle(faceNode)
##	
##	# link mesh to blend objects	
##	object = blenderScene.objects.new(newMesh, scene.GetNodeName(meshNode))	
##	
##
##	# calculate normal after mesh is parented	
##	#newMesh.mode |= Blender.Mesh.Modes.AUTOSMOOTH
##	#newMesh.degr = 30
##	#for face in newMesh.faces:
##	#	face.smooth = 1
##	#newMesh.calcNormals()
##
##	return object 
##
##	
###
### Load all scene objects
###
##def LoadNodesScene(scene, rootNode, blenderScene, chidrenList):
##	''' recusivally load convert a scene to a blender scene'''
##
##	blenderObject = None
##	meshNode = GetMeshNode(scene, rootNode)
##	if meshNode != None:
##		blenderObject = CreateBlenderMeshObjectFromNode (scene, meshNode, blenderScene)
##	else:
##		blenderObject = CreateBlenderEmptyOject (scene, rootNode, blenderScene)
##		
##	# add the object to the childer list
##	chidrenList.append(blenderObject)	
##	
##	# see if this node has more children and add then to the scene
##	myChidren = []
##	childLink = scene.GetFirstChildLink (rootNode)
##	while childLink != None: 
##		childNode = scene.GetNodeFromLink(childLink)
##		if scene.IsSceneNode(childNode) == True:
##		   LoadNodesScene(scene, childNode, blenderScene, myChidren)
##		childLink = scene.GetNextChildLink (rootNode, childLink)
##		
##	blenderObject.makeParent(myChidren) 
##
##	# set the object name
##	blenderObject.setName (scene.GetNodeName(rootNode))
##	
##	# set position and oriention
##	object = pyScene.pyObject (scene, rootNode)
##	
##	#posit = object.GetLocalPosition()
##	#eulers = object.GetLocalEulers()
##	#scale = object.GetLocalScale()
##	#euler = [eulers.x, eulers.y, eulers.z]
##	#blenderObject.setSize(scale.x, scale.y, scale.z) 
##	#blenderObject.setLocation(posit.x, posit.y, posit.z) 
##	#blenderObject.setEuler(euler) 
##	
##	objectMatrix = object.GetMatrix4x4()
##	blenderMatrix = Blender.Mathutils.Matrix([1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1])
##	
##	blenderMatrix[0][0] = objectMatrix.e00
##	blenderMatrix[0][1] = objectMatrix.e01
##	blenderMatrix[0][2] = objectMatrix.e02
##	
##	blenderMatrix[1][0] = objectMatrix.e10
##	blenderMatrix[1][1] = objectMatrix.e11
##	blenderMatrix[1][2] = objectMatrix.e12
##
##	blenderMatrix[2][0] = objectMatrix.e20
##	blenderMatrix[2][1] = objectMatrix.e21
##	blenderMatrix[2][2] = objectMatrix.e22
##	
##	blenderMatrix[3][0] = objectMatrix.e30
##	blenderMatrix[3][1] = objectMatrix.e31
##	blenderMatrix[3][2] = objectMatrix.e32
##	
##	blenderObject.setMatrix(blenderMatrix)
#
#	
#
#
## implement main scene loader function
##
#def LoadAlchemediaScene(filename):
#	''' load fbx file '''
#	#scene = pyScene.pyScene()
#	#scene.Load (filename)
#	#root = scene.GetRoot() 
#	#
#	## get the active blender scene
#	#blenderScene = bpy.data.scenes.active
#	#
#	## load all unique textures
#	#(path, name) = os.path.split (filename)
#	#
#	#childLink = scene.GetFirstChildLink (root)
#	#while childLink != None: 
#	#	textureNode = scene.GetNodeFromLink(childLink)
#	#	if scene.IsTextureNode(textureNode) == True:
#	#		texture = CreateBlenderTextureFromNode (scene, textureNode, blenderScene, path)
#	#		
#	#		# add a map key for asigning faces
#	#		sourceTexture = pyScene.pyTexture(scene, textureNode)
#	#		g_textureMap[sourceTexture.GetId()] = texture
#	#	childLink = scene.GetNextChildLink (root, childLink)
#	#
#	#	
#	## import all objects into a blender scene	
#	#myChidren = []
#	#childLink = scene.GetFirstChildLink (root)
#	#while childLink != None: 
#	#	node = scene.GetNodeFromLink(childLink)
#	#	if scene.IsSceneNode(node) == True:
#	#	   LoadNodesScene(scene, node, blenderScene, myChidren)
#	#	childLink = scene.GetNextChildLink (root, childLink)
#	#
#	## make sure everthing is updated before exiting
#	## I see this in some demos, but nthing seeme to makes the scene render completly
#	#blenderScene.update(1)
#	##Blender.Redraw()
#
#
#######################################################
## Callbacks for Window functions
#######################################################
#def filename_callback(input_filename):
#	global g_filename
#	g_filename.val = input_filename
#
#
#######################################################
## GUI Loader
#######################################################
#def draw_gui():
#	global g_filename
#	global EVENT_NOEVENT, EVENT_SCENE, EVENT_CHOOSE_FILENAME, EVENT_EXIT
#
#	########## Titles
#	glClear(GL_COLOR_BUFFER_BIT)
#	glRasterPos2d(8, 100)
#	Text("Alchemedia Loader")
#
#	# File name chooser Button
#	BeginAlign()
#	filename = String("file to load: ", EVENT_NOEVENT, 10, 55, 410, 18, g_filename.val, 255, "Alchemdia xml file to load")
#	Button("Browse", EVENT_CHOOSE_FILENAME, 420, 55, 80, 18)
#	EndAlign()
#	
#	# Load and Exit Buttons
#	Button("Load", EVENT_SCENE, 10, 10, 80, 18)
#	Button("Cancel", EVENT_EXIT, 170, 10, 80, 18)
#
#def event(evt, val):	
#	if (evt == QKEY and not val):
#		Blender.Draw.Exit()
#
#def bevent(evt):
#	global g_filename
#	global EVENT_NOEVENT, EVENT_SCENE, EVENT_CHOOSE_FILENAME, EVENT_EXIT
#
#	# Manages GUI events
#	if (evt == EVENT_EXIT):
#		Blender.Draw.Exit()
#	elif (evt == EVENT_CHOOSE_FILENAME):
#		FileSelector(filename_callback, "fbx file selection")
#	elif (evt == EVENT_SCENE):
#		if not Blender.sys.exists(g_filename.val):
#			PupMenu('file does not exist')
#			return
#		else:
#			LoadAlchemediaScene(g_filename.val)
#			Blender.Redraw()
#			Blender.Draw.Exit()
#			return
#	
#if __name__ == '__main__':
#	Register(draw_gui, event, bevent)


bl_info = {
    "name": "My Test Add-on",
    "blender": (2, 80, 0),
    "category": "Import",
}

def register():
    print("Hello World")

def unregister():
    print("Goodbye World")
