/////////////////////////////////////////////////////////////////////////////
// Name:        ColladaImport.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#ifndef __D_COLLADA_IMPORT_PLUGIN__
#define __D_COLLADA_IMPORT_PLUGIN__



#include <dae.h>
#include <dae/domAny.h>
#include <dom/domCOLLADA.h>
#include <dom/domConstants.h>
#include <dom/domProfile_COMMON.h>


//class domNode;
//class domMesh;
//class domGeometry;
//class daeDocument;
//class domVisual_scene;


class ColladaImport: public dImportPlugin
{
	public:
	ColladaImport();
	~ColladaImport();

	virtual const char* GetMenuName () { return "Collada 1.4 import";}
	virtual const char* GetFileExtension () { return ".dae";}
	virtual const char* GetFileDescription () {return "Import collada 1.4 scene";}
	virtual const char* GetSignature () {return "Collada 1.4 import";}


	static ColladaImport* GetPlugin();

//	virtual bool Import (const char* fileName, dScene* world);
	virtual bool Import (const char* const fileName, dPluginInterface* const interface);

	private:
	class SourceBuffer
	{
		public: 
		SourceBuffer (domSource* source);
		~SourceBuffer ();

		int m_count;
		dVector* m_data;

	};

	class ImageCache: public dTree<dScene::dTreeNode*, int>
	{
	};

	class GeometryChache: public dTree<dScene::dTreeNode*, domGeometry*>
	{
	};

	class NodeChache: public dTree<dScene::dTreeNode*, domNode*>
	{
	};


	dMatrix GetMatrix (domNode* node) const;
	SourceBuffer* CreateSource (domSource* source);
	void GetOffsetsAndSources (const domInputLocalOffset_Array& inputArray, 
							   int& vertexOffset, int& normalOffset, int& uv0Offset, int& uv1Offset, 
							   domSource** posSource, domSource** normalSrc, domSource** uv0Source, domSource** uv1Source);

	void LoadTriangles(DAE* collada, dScene* world, dScene::dTreeNode* gemetryNode, ImageCache& imageCache,
					   domTriangles_Array& trianglesArray, int* faceIndexCount, int* materialIndex, 
					   int* vertex, int* normal, int* uv0, int* uv1,
					   domSource** posSource, domSource** normalSrc, domSource** uv0Source, domSource** uv1Source);

	void LoadPolygons(DAE* collada, dScene* world, dScene::dTreeNode* gemetryNode, ImageCache& imageCache,
					  domPolygons_Array &polygonArray, int* faceIndexCount, int* materialIndex, 
					  int* vertex, int* normal, int* uv0, int* uv1,
				      domSource** posSource, domSource** normalSrc, domSource** uv0Source, domSource** uv1Source);

	void LoadPolygonsList(DAE* collada, dScene* world, dScene::dTreeNode* gemetryNode, ImageCache& imageCache,
						  domPolylist_Array &polygonListArray, int* faceIndexCount, int* materialIndex, 
						  int* vertex, int* normal, int* uv0, int* uv1,
					      domSource** posSource, domSource** normalSrc, domSource** uv0Source, domSource** uv1Source);

	domImage* FindImage (domLibrary_images* library, const char*name) const;
	domTechnique *FindProfileExtra (domExtra_Array& extraArray, const char* keyword) const;
	domMaterial* FindMaterial (domLibrary_materials* library, const char* name) const;
	int CreateMaterial (DAE* collada, const char* matName, ImageCache& imageCache, dScene* world, dScene::dTreeNode* gemetryNode);
	dScene::dTreeNode* LoadMesh (DAE* collada, domMesh* colladaMesh, dScene::dTreeNode* parent, dScene* world, ImageCache& imageCache);
	dScene::dTreeNode* LoadConvexMesh (DAE* collada, domConvex_mesh* colladaMesh, dScene::dTreeNode* parent, dScene* world);

	void LoadSkinController (DAE* collada, dScene* world, domInstance_controller* colladaSkinController, GeometryChache& meshCache);
	void LoadNodeData (DAE* collada, dScene::dTreeNode* node, domNode* colladaNode, dScene* world, GeometryChache& meshCache, ImageCache& imageCache);
	void LoadVisualScene (DAE* collada, domVisual_scene* visualScene, dScene* world, GeometryChache& meshCache, ImageCache& imageCache, NodeChache& nodeCache);

	dScene::dTreeNode* LoadCollision (DAE* collada, const domRigid_body::domTechnique_common::domShape* shape, dScene* world, dScene::dTreeNode* parent, GeometryChache& meshCache);
	dScene::dTreeNode* LoadCollision (DAE* collada, const domRigid_body::domTechnique_common::domShape_Array& shapeArray, dScene* world, dScene::dTreeNode* rigidBody, GeometryChache& meshCache, int isDynamicsBody);
	dScene::dTreeNode* LoadRigidBody (DAE* collada, domRigid_body* rigidBody, dScene* world, dScene::dTreeNode* model, GeometryChache& meshCache);
	void LoadPhysicScene (DAE* collada, dScene* world, const NodeChache& nodeCache, GeometryChache& meshCache);


	

	dFloat m_scale;
	dMatrix m_globalRotation;
};

#endif