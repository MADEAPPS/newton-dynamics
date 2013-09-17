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

#ifndef __NE_COLLADA_EXPORT_PLUGIN__
#define __NE_COLLADA_EXPORT_PLUGIN__


#include <dae.h>
#include <dae/domAny.h>
#include <dom/domCOLLADA.h>
#include <dom/domConstants.h>
#include <dom/domProfile_COMMON.h>


class ColladaExport: public dExportPlugin
{
	public:
	ColladaExport();
	~ColladaExport();

	static ColladaExport* GetPlugin();
	virtual const char* GetMenuName () { return "Collada 1.4 export";}
	virtual const char* GetSignature () {return "Collada 1.4 export";}
	virtual const char* GetFileExtension () { return ".dae";}
	virtual const char* GetFileDescription () {return "Export collada 1.4 scene";}

//	virtual void Export (const char* fileName, const dScene* world);
	virtual void Export (const char* const fileName, dPluginInterface* const interface);


	private:
/*
	// make a mesh cache
	struct Name
	{	
		char m_name[128];
	};

	class UniqueName: public dTree<Name, dNodeInfo*> 
	{	
		dTree<Name, unsigned> cache;
		public:

		void AddName (dNodeInfo* info)
		{
			Name nameCopy;
			int uniqueName = 0;
			sprintf (nameCopy.m_name, "%s", info->GetName());
			while (!cache.Insert (nameCopy, dCRC (nameCopy.m_name))) {
				uniqueName ++;
				sprintf (nameCopy.m_name, "%s_%d", info->GetName(), uniqueName);
			}
			Insert (nameCopy, info);
		}

		const char* FindName (dNodeInfo* info) const
		{
			dTreeNode* node = Find(info);
			if (node) {
				return node->GetInfo().m_name;
			}
			return NULL;
		}
	};

	UniqueName m_nameMap;
*/
//	void AddMeshToModel (dModel* model, const dScene* world, dScene::dTreeNode* sceneNode, 
//						 const dTree<dMesh*, dScene::dTreeNode*>& meshCache, dTree<int, dScene::dTreeNode*>& boneIdCache);

	struct UniqueName
	{	
		dScene::dTreeNode* m_node;
		char m_name[128];
	};

	class UniqueNameFilter: public dTree<UniqueName, int> 
	{
		public:
		void FixName (char* const name) const
		{
			for (int i = 0; name[i]; i ++) {
				if (isspace(name[i])) {
					name[i] = '_';
				}
			}
		}

		const char* CreateUniqueName (const dScene* world, dScene::dTreeNode* node)
		{
			_ASSERTE (0);
			return "";
/*
			UniqueName name; 
			dNodeInfo* info = world->GetInfoFromNode(node);
			name.m_node = node;
			sprintf (name.m_name, "%s", info->GetName()); 
			FixName (name.m_name);
			int crc = dCRC (name.m_name);
			dTreeNode* cacheNode = Insert(name, crc);
			int id = 1;
			while (!cacheNode) {
				name.m_node = node;
				sprintf (name.m_name, "%s%02d", info->GetName(), id); 
				FixName (name.m_name);
				id ++;
				//crc = dCRC (node, sizeof (dScene::dTreeNode*), dCRC (name.m_name));
				crc = dCRC (name.m_name);
				cacheNode = Insert(name, crc);
			}
				
			return cacheNode->GetInfo().m_name;
*/
		}

		const char* FindUniqueName (const dScene* world, dScene::dTreeNode* node)
		{
			_ASSERTE (0);
			return "";
/*
			dNodeInfo* info = world->GetInfoFromNode(node);
			char name[256];
			strcpy (name, info->GetName());
			FixName (name);
			int crc = dCRC (name);
			dTreeNode* cacheNode = Find(crc);
			_ASSERTE (cacheNode);
			int id = 1;
			while (cacheNode->GetInfo().m_node != node) {
				sprintf (name, "%s%02d", info->GetName(), id); 
				FixName (name);
				id ++;
				cacheNode = Find(crc);
				_ASSERTE (cacheNode);
			}
			return cacheNode->GetInfo().m_name;
*/
		}
	};

	class ModifierNodes: public dTree<domNode *, dScene::dTreeNode*>
	{

	};
	
	void FixName (char* const name) const;
	domImage* FindImage (domLibrary_images* library, const char*name) const;
	domMaterial* FindMaterial (domLibrary_materials* library, const char* name) const;

	void AddDefaultMaterial (domLibrary_effects* effectLibrary, domLibrary_materials* materialLibrary);

	void AddMesh (daeDocument *document, const dScene* scene, dScene::dTreeNode* meshNode, UniqueNameFilter& uniqueNames);
	void AddMeshInstance (daeDocument *document, const dScene* scene, daeElement *node, dScene::dTreeNode* meshNode, UniqueNameFilter& uniqueNames);

	void AddScene (daeDocument* document);
	void AddAsset (daeDocument *document);
	void AddCameraLibrary (daeDocument *document, const dScene* scene);
	void AddImageLibrary (daeDocument *document, const dScene* scene);
	void AddMaterialLibrary (daeDocument *document, const dScene* scene);
	void AddGeometryLibrary (daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames);
	void AddControllerLibrary (daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames);
	
	void AddControllerInstances(daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames, ModifierNodes& modifierList);
	void AddVisualSceneLibrary (daeDocument *document, const dScene* scene, UniqueNameFilter& uniqueNames, ModifierNodes& modifierList);

	void SceneToCollada (DAE* collada, const dScene* scene, UniqueNameFilter& uniqueNames);
	dMatrix m_globalRotation;

};

#endif