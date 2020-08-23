/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonMeshEffectExport.h
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

#ifndef _D_FBX_IMPORT_H_
#define _D_FBX_IMPORT_H_



class dfbxImport: public dImportPlugin
{
	public:
	static dImportPlugin* GetPluginFBX();
	static dImportPlugin* GetPluginOBJ();
	static dImportPlugin* GetPluginDAE();

	class GlobalNoceMap: public dTree<dScene::dTreeNode*, FbxNode*>
	{
	};

//	class GlobalMeshMap: public dTree<dPluginScene::dTreeNode*, FbxGeometry*>
	class GlobalMeshMap: public dTree<dPluginScene::dTreeNode*, FbxNodeAttribute*>
	{
	};

	class GlobalMaterialMap: public dTree<dPluginScene::dTreeNode*, FbxSurfaceMaterial*>
	{
	};

	class GlobalTextureMap: public dTree<dPluginScene::dTreeNode*, FbxTexture*>
	{
	};

	class LocalMaterialMap: public dTree<dPluginScene::dTreeNode*, int>
	{
	};


	class UsedMaterials: public dTree<int, dPluginScene::dTreeNode*>
	{
	};


	class ImportStackData
	{
		public:
		ImportStackData (const dMatrix& parentMatrix, FbxNode* const fbxNode, dPluginScene::dTreeNode* parentNode)
			:m_parentMatrix(parentMatrix)
			,m_fbxNode(fbxNode)
			,m_parentNode(parentNode)
		{
		}
		dMatrix m_parentMatrix;
		FbxNode* m_fbxNode;
		dPluginScene::dTreeNode* m_parentNode;
	};

	public:
	dfbxImport (const char* const ext, const char* const signature, const char* const description);
	virtual ~dfbxImport() {}
	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return m_ext;}
	virtual const char* GetSignature () {return m_signature;}
	virtual const char* GetFileDescription () {return m_description;}

	virtual bool Import (const char* const fileName, dPluginInterface* const interface);

	private:
	void PopulateScene (FbxScene* const fbxScene, dPluginScene* const ngdScene);
	void LoadHierarchy  (FbxScene* const fbxScene, dPluginScene* const ngdScene, GlobalNoceMap& nodeMap);
	void ImportTexture (dPluginScene* const ngdScene, FbxProperty pProperty, dPluginScene::dTreeNode* const materialNode, GlobalTextureMap& textureCache);
	void ImportSkeleton (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
	void ImportLineShape (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
	void ImportNurbCurveShape (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
	void ImportMaterials (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const meshNode, GlobalMaterialMap& materialCache, LocalMaterialMap& localMaterilIndex, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials);
	void ImportMeshNode (FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, GlobalNoceMap& nodeMap);
	
	static bool ProgressCallback (float pPercentage, FbxString pStatus);

	static dPluginInterface* m_interface;
	
	int m_materialId;
	char m_ext[32];
	char m_signature[64];
	char m_description[64];
};

#endif
