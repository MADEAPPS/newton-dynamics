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



	dScene(NewtonWorld* const newton);
	dScene(const dScene& me);
	virtual ~dScene(void);

	virtual void CleanUp();
	
	virtual dTreeNode* GetRootNode() const;

	virtual dTreeNode* CreateModelNode(dTreeNode* const parent);
	virtual dTreeNode* CreateSceneNode(dTreeNode* const parent);
	virtual dTreeNode* CreateGeometryTransformNode(dTreeNode* const parent);
	virtual dTreeNode* CreateBoneNode(dTreeNode* const parent);
	virtual dTreeNode* CreateRigidbodyNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionBoxNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionConeNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionSphereNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionCapsuleNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionCylinderNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionCompoundNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionConvexHullNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionChamferCylinderNode(dTreeNode* const parent);
	dScene::dTreeNode* CreateCollisioTreeNode(dTreeNode* const parent);
	virtual dTreeNode* CreateMeshNode(dTreeNode* const parent);
	virtual dTreeNode* CreateSkinModifierNode(dTreeNode* const parent);

	virtual dTreeNode* CreateMaterialNode (int id);
	virtual dTreeNode* CreateTextureNode (const char* const pathName);

	virtual dTreeNode* GetTextureCacheNode ();
	virtual dTreeNode* GetMaterialCacheNode ();
	virtual dTreeNode* GetGeometryCacheNode ();

	virtual dTreeNode* AddNode(dNodeInfo* const sceneInfo, dTreeNode* const parent);
	virtual dTreeNode* CreateNode (const char* const className, dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionFromNewtonCollision(dTreeNode* const parent, NewtonCollision* const collision);
	
	virtual void AddReference(dTreeNode* const parent, dTreeNode* const child);
	virtual void RemoveReference(dTreeNode* const node1, dTreeNode* const node2);

	virtual dTreeNode* GetFirstNode () const;
	virtual dTreeNode* GetNextNode (dTreeNode* const node) const;
	virtual dTreeNode* FindNode (dNodeInfo* const info) const;
	 
	
	virtual void* GetFirstChild(dTreeNode* const parentNode) const;
	virtual void* GetNextChild(dTreeNode* const parentNode, void* const link) const;

	virtual void* GetFirstParent(dTreeNode* const childNode) const;
	virtual void* GetNextParent(dTreeNode* const childNode, void* const link) const;

	//virtual bool IsParentLink (void* const link) const;
	virtual dTreeNode* GetNodeFromLink (void* const child) const;

	//virtual void CloneInfoFromNode(dTreeNode* const node);
	virtual dNodeInfo* CloneNodeInfo(dTreeNode* const node) const;
	virtual dNodeInfo* GetInfoFromNode(dTreeNode* const node) const;
	
	virtual dTreeNode* FindMaterialById(int materialId);
	virtual dTreeNode* FindMaterialBySignature(dCRCTYPE signature);

	virtual dTreeNode* FindTextureByTextId(dCRCTYPE textId);
	virtual dTreeNode* FindMaterialById(dTreeNode* const parentNode, int materialId) const;
	virtual dTreeNode* FindTextureByTextId(dTreeNode* const parentNode, dCRCTYPE textId) const;
	

	virtual dTreeNode* FindChildByType(dTreeNode* const parentNode, dCRCTYPE type) const;
	virtual dTreeNode* FindParentByType(dTreeNode* const child, dCRCTYPE type) const;

	virtual NewtonWorld* GetNewtonWorld() const {return m_newton;}

	virtual void DeleteNode (dTreeNode* const node);
	virtual void MergeScene (dScene* const scene);

	virtual void Serialize (const char* const fileName);
	virtual bool Deserialize (const char* const fileName);

	virtual dFloat RayCast (const dVector& p0, const dVector& p1, dList<dTreeNode*>& traceRoot) const;

	virtual void FreezeScale () const;
	virtual void FreezePivot () const;
	virtual void BakeTransform (dMatrix& matrix) const;
	
	virtual int GetRevision() const;

	virtual void RemoveUnusedVertex();
	

	virtual void SetNodeLRU (dTreeNode* const node, int lru);
	virtual int GetNodeLRU (dTreeNode* const node) const;

	virtual void NewtonWorldToScene (const NewtonWorld* const world, dSceneExportCallback* const visualContext);
	virtual void SceneToNewtonWorld (NewtonWorld* world, dList<NewtonBody*>& loadedBodies);


	virtual void DeleteDuplicateTextures();
	virtual void DeleteDuplicateMaterials();
	virtual void DeleteDuplicateGeometries();

	
	protected:
	virtual dTreeNode* GetCacheNode (const char* const cacheName);

	int m_revision;

	NewtonWorld* m_newton;
};


#endif