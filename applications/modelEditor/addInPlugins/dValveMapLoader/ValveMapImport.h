/////////////////////////////////////////////////////////////////////////////
// Name:        ValveMapImport.h
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

#ifndef __NE_VALVE_IMPORT_PLUGIN__
#define __NE_VALVE_IMPORT_PLUGIN__

#include "dPlugInStdafx.h"
#include "dImportPlugin.h"

class ValveMapImport: public dImportPlugin
{
public:
	ValveMapImport();
	~ValveMapImport();

	virtual const char* GetMenuName () { return "Valve Map File import";}
	virtual const char* GetFileExtension () { return "vmf";}
	virtual const char* GetFileDescription () {return "Valve Map File import";}

	virtual bool Import (const char* fileName, dScene* world);

	static ValveMapImport* GetPlugin();


	private:
	int NextChar (FILE* file);
	void SkipChar (FILE* file, char chr);
	void SkipClass (FILE* file);
	void LoadVersionInfo(FILE* file, dScene* world, void* userData);
	void LoadVisGroups(FILE* file, dScene* world, void* userData);
	void LoadViewSettings(FILE* file, dScene* world, void* userData);
	void LoadWorld (FILE* file, dScene* world, void* userData);
	void LoadSolid (FILE* file, dScene* world, void* userData);
	void LoadSide (FILE* file, dScene* world, void* userData);
	void LoadEntity (FILE* file, dScene* world, void* userData);
	void LoadCamera (FILE* file, dScene* world, void* userData);
	void LoadCordon (FILE* file, dScene* world, void* userData);

	struct Property
	{
		unsigned m_name;
		char m_value[256];
	};
	
	class PropertyList: public dList<Property>
	{
		public:

		Property* GetProperty (const char* name);

	};
	
	void ReadProperties (FILE* file, dList<Property>& list);
	

	void ParceMapFile (FILE* file, dScene* world, void* userData);
//	void AddModel (dModel* model, dScene* world);
//	void CreateMeshGeomtry (dScene::dTreeNode* node, dMesh* mesh, dScene* world);
//	void AddSkeleton (dBone* bone, dModel* model, dScene* world, dScene::dTreeNode* parentNode, dTree<dScene::dTreeNode*, int>& sceneNodes);

};

#endif