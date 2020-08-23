/////////////////////////////////////////////////////////////////////////////
// Name:        pyScene.h
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

#pragma once

class dScene;

class pyScene
{
public:
	pyScene(void);
	virtual ~pyScene(void);

	void Load (const char* name);
	void Save (const char* name);

	void* GetRoot();
	void* GetFirstChildLink(void* parent);
	void* GetNextChildLink(void* parent, void* current);
	void AddReference (void* parentNode, void* childNode);

	void* GetNodeFromLink(void* link);
	bool IsSceneNode (void* node);
	bool IsMeshNode (void* node);
	bool IsMaterialNode (void* node);
	bool IsTextureNode (void* node);
	const char* GetNodeName (void* node);

	void* CreateSceneNode(void* parent);
	void* CreateMeshNode(void* parent);
	void* CreateRigidbodyNode(void* parent);
	void* CreateTextureNode(const char* pathName);
	void* CreateMaterialNode(void* parentMesh, int materilID);

	dScene* GetScene() const {return m_scene;}

	private:
	dScene* m_scene;

//	friend class pyMesh;
//	friend class pyObject;
//	friend class pyTexture;
//	friend class pyMaterial;
};
