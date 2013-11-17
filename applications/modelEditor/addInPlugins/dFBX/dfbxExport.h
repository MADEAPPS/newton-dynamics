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

#ifndef _D_FBX_EXPORT_H_
#define _D_FBX_EXPORT_H_


class dfbxExport: public dExportPlugin
{
	public:
	static dExportPlugin* GetPluginFBX();
	static dExportPlugin* GetPluginOBJ();
	static dExportPlugin* GetPluginDAE();

	public:
	dfbxExport(const char* const ext, const char* const signature, const char* const description)
		:dExportPlugin()
	{
		strcpy (m_ext, ext);
		strcpy (m_signature, signature);
		strcpy (m_description, description);
	}
	~dfbxExport(){}
	
	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetFileExtension () { return m_ext;}
	virtual const char* GetSignature () {return m_signature;}
	virtual const char* GetFileDescription () {return m_description;}
	virtual void Export (const char* const fileName, dPluginInterface* const interface);

	private:
	void BuildMeshes (dPluginScene* const ngdScene, FbxScene* const fbxScene, dTree<FbxMesh*, dPluginScene::dTreeNode*>& meshMap);
	void LoadNodes (dPluginScene* const scene, FbxScene* const fbxScene, dTree<FbxMesh*, dPluginScene::dTreeNode*>& meshMap);
	void LoadNode (dPluginScene* const scene, FbxScene* const fbxScene, FbxNode* const fpxRoot, dPluginScene::dTreeNode* const node, dTree<FbxMesh*, dPluginScene::dTreeNode*>& meshMap);
	

	char m_ext[32];
	char m_signature[32];
	char m_description[32];

};

#endif