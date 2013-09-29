/////////////////////////////////////////////////////////////////////////////
// Name:        dScene.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
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

#ifndef _D_SCENE_H_
#define _D_SCENE_H_

#include "dSceneGraph.h"

struct NewtonWorld;
class dCameraNodeInfo;
class dRootNodeInfo;
class dMaterialNodeInfo;
class dTextureNodeInfo;

#define D_SCENE_ROOT_NODE_NAME		"NewtonGameDynamnics"
#define D_SCENE_FILE_DESCRITION		"Newton Dynamnics 3.00, universal file format"

// adding first revision number to the file
//#define D_SCENE_REVISION_NUMBER 100

// change SceneNodeMatrix Matrix from global to local
//#define D_SCENE_REVISION_NUMBER 101


// added a texture cache, material cache, and mesh cache as children of root node.
//#define D_SCENE_REVISION_NUMBER 102

// adding the geometry transform node info, 
#define D_SCENE_REVISION_NUMBER 103


#define D_TEXTURE_CACHE_NODE_MAME	"dTextureCache"
#define D_MATERIAL_CACHE_NODE_MAME	"dMaterialCache"
#define D_GEOMETRY_CACHE_NODE_MAME	"dGeometryCache"

class dScene: public dSceneGraph, public dRefCounter
{
	public:

	class dSceneExportCallback
	{
		public:
		// this call back should here the use should take the user data from the body create newtonMesh from it and return that back
		virtual NewtonMesh* CreateVisualMesh (NewtonBody* const body, char* const name, int maxNameSize) const = 0;

		// Get the material information for this visual Mesh material ID
		//... 
		void SetMaterialInfo (dMaterialNodeInfo& materialInfo, const NewtonMesh* const mesh, const NewtonBody* const body, int MaterialID) {}
		void SetTextureInfo (dTextureNodeInfo& textureInfo, const NewtonMesh* const mesh, const NewtonBody* const body, int MaterialID) {}
	};



	DSCENE_API dScene(NewtonWorld* const newton);
	DSCENE_API dScene(const dScene& me);
	virtual DSCENE_API ~dScene(void);
	virtual DSCENE_API void CleanUp();
	virtual DSCENE_API dTreeNode* GetRootNode() const;

	virtual DSCENE_API dTreeNode* CreateModelNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateSceneNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateGeometryTransformNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateBoneNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateRigidbodyNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionBoxNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionConeNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionSphereNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionCapsuleNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionCylinderNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionCompoundNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionConvexHullNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionChamferCylinderNode(dTreeNode* const parent);
	//virtual DSCENE_API dScene::dTreeNode* CreateCollisioTreeNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisioTreeNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateMeshNode(dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateSkinModifierNode(dTreeNode* const parent);

	virtual DSCENE_API dTreeNode* CreateMaterialNode (int id);
	virtual DSCENE_API dTreeNode* CreateTextureNode (const char* const pathName);

	virtual DSCENE_API dTreeNode* GetTextureCacheNode ();
	virtual DSCENE_API dTreeNode* GetMaterialCacheNode ();
	virtual DSCENE_API dTreeNode* GetGeometryCacheNode ();

	virtual DSCENE_API dTreeNode* AddNode(dNodeInfo* const sceneInfo, dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateNode (const char* const className, dTreeNode* const parent);
	virtual DSCENE_API dTreeNode* CreateCollisionFromNewtonCollision(dTreeNode* const parent, NewtonCollision* const collision);
	
	virtual DSCENE_API void AddReference(dTreeNode* const parent, dTreeNode* const child);
	virtual DSCENE_API void RemoveReference(dTreeNode* const node1, dTreeNode* const node2);

	virtual DSCENE_API dTreeNode* GetFirstNode () const;
	virtual DSCENE_API dTreeNode* GetNextNode (dTreeNode* const node) const;
	virtual DSCENE_API dTreeNode* FindNode (dNodeInfo* const info) const;
	 
	
	virtual DSCENE_API void* GetFirstChildLink(dTreeNode* const parentNode) const;
	virtual DSCENE_API void* GetNextChildLink(dTreeNode* const parentNode, void* const link) const;

	virtual DSCENE_API void* GetFirstParentLink(dTreeNode* const childNode) const;
	virtual DSCENE_API void* GetNextParentLink(dTreeNode* const childNode, void* const link) const;

	//virtual DSCENE_API bool IsParentLink (void* const link) const;
	virtual DSCENE_API dTreeNode* GetNodeFromLink (void* const child) const;

	//virtual DSCENE_API void CloneInfoFromNode(dTreeNode* const node);
	virtual DSCENE_API dNodeInfo* CloneNodeInfo(dTreeNode* const node) const;
	virtual DSCENE_API dNodeInfo* GetInfoFromNode(dTreeNode* const node) const;
	
	virtual DSCENE_API dTreeNode* FindMaterialById(int materialId);
	virtual DSCENE_API dTreeNode* FindMaterialBySignature(dCRCTYPE signature);

	virtual DSCENE_API dTreeNode* FindTextureByTextId(dCRCTYPE textId);
	virtual DSCENE_API dTreeNode* FindMaterialById(dTreeNode* const parentNode, int materialId) const;
	virtual DSCENE_API dTreeNode* FindTextureByTextId(dTreeNode* const parentNode, dCRCTYPE textId) const;
	

	virtual DSCENE_API dTreeNode* FindChildByName(dTreeNode* const parentNode, const char* const name) const;
	virtual DSCENE_API dTreeNode* FindChildByType(dTreeNode* const parentNode, dCRCTYPE type) const;
	virtual DSCENE_API dTreeNode* FindParentByType(dTreeNode* const child, dCRCTYPE type) const;

	virtual DSCENE_API NewtonWorld* GetNewtonWorld() const {return m_newton;}

	virtual DSCENE_API void DeleteNode (dTreeNode* const node);
	virtual DSCENE_API void MergeScene (dScene* const scene);
	virtual DSCENE_API void UnmergeScene (dScene* const scene);

	virtual DSCENE_API void Serialize (const char* const fileName);
	virtual DSCENE_API bool Deserialize (const char* const fileName);

	virtual DSCENE_API dFloat RayCast (const dVector& p0, const dVector& p1, dList<dTreeNode*>& traceRoot) const;

	virtual DSCENE_API void FreezeScale () const;
	virtual DSCENE_API void FreezePivot () const;
	virtual DSCENE_API void BakeTransform (dMatrix& matrix) const;
	
	virtual DSCENE_API int GetRevision() const;

	virtual DSCENE_API void RemoveUnusedVertex();
	

	virtual DSCENE_API void SetNodeLRU (dTreeNode* const node, int lru);
	virtual DSCENE_API int GetNodeLRU (dTreeNode* const node) const;

	virtual DSCENE_API void NewtonWorldToScene (const NewtonWorld* const world, dSceneExportCallback* const visualContext);
	virtual DSCENE_API void SceneToNewtonWorld (NewtonWorld* world, dList<NewtonBody*>& loadedBodies);


	virtual DSCENE_API void DeleteDuplicateTextures();
	virtual DSCENE_API void DeleteDuplicateMaterials();
	virtual DSCENE_API void DeleteDuplicateGeometries();

	
	protected:
	virtual DSCENE_API dTreeNode* GetCacheNode (const char* const cacheName);

	int m_revision;

	NewtonWorld* m_newton;
};


#endif