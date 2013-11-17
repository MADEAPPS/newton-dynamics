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

	class NodeMap: public dTree<dScene::dTreeNode*, FbxNode*>
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
	dfbxImport(const char* const ext, const char* const signature, const char* const description)
		:dImportPlugin() 
	{
		strcpy (m_ext, ext);
		strcpy (m_signature, signature);
		strcpy (m_description, description);
	}
	~dfbxImport() {}

	virtual const char* GetMenuName () { return GetSignature();}

	virtual const char* GetFileExtension () { return m_ext;}
	virtual const char* GetSignature () {return m_signature;}
	virtual const char* GetFileDescription () {return m_description;}

	virtual bool Import (const char* const fileName, dPluginInterface* const interface);

	private:
	void PopulateScene (FbxScene* const fbxScene, dPluginScene* const ngdScene);
	void LoadHierarchy  (FbxScene* const fbxScene, dPluginScene* const ngdScene, NodeMap& nodeMap);
	void ImportMeshNode (FbxNode* const fbxMeshNode, dPluginScene* const ngdScene, dPluginScene::dTreeNode* const node, dTree<dPluginScene::dTreeNode*, FbxMesh*>& meshCache);

	char m_ext[32];
	char m_signature[32];
	char m_description[32];
};

#endif
