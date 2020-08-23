/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshNGD.cpp
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

#include "StdAfx.h"
#include "dMeshNGD.h"


dMeshNGD::dMeshNGD()
	:dPluginMesh()
{
}

dMeshNGD::~dMeshNGD()
{
}


dMeshNGD* dMeshNGD::GetPlugin()
{
	static dMeshNGD plugin;
	return &plugin;
}

void dMeshNGD::Destroy (dPluginInterface* const interface, dPluginScene* const asset)
{
	dAssert(0);
//	asset->Release();
}


dPluginScene* dMeshNGD::Create (dPluginInterface* const interface)
{
	dAssert(0);
	return NULL;
/*
	dPluginScene* const scene = interface->GetScene();
	NewtonWorld* const world = scene->GetNewtonWorld();
	FXMainWindow* const mainWindow = (FXMainWindow*) NewtonWorldGetUserData(world);
	FXFileDialog open(mainWindow, "load NGD mesh asset");
	open.setPatternList("*.ngd");
	open.setDirectory (interface->GetFilePath());
	dPluginScene* asset = NULL;
	if(open.execute()){
		asset = new dPluginScene(world);
		asset->Deserialize(open.getFilename().text());
	}

	return asset;
*/
}

