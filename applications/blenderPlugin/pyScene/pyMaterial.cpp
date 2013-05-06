/////////////////////////////////////////////////////////////////////////////
// Name:        pyMaterial.h
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

#include "StdAfx.h"
#include "pyScene.h"
#include "pyMaterial.h"


pyMaterial::pyMaterial(pyScene* scene, void* materialNode)
	:pyBaseNodeInfo<dMaterialNodeInfo>(scene, materialNode)
{
}

pyMaterial::~pyMaterial(void)
{
}


long long pyMaterial::GetId()
{
	dMaterialNodeInfo* info = GetInfo();
	return info->GetId();
}

void pyMaterial::SetAmbientTextId(int id)
{
	dMaterialNodeInfo* info = GetInfo();
	info->SetAmbientTextId(id);
}

void pyMaterial::SetDiffuseTextId(int id)
{
	dMaterialNodeInfo* info = GetInfo();
	info->SetDiffuseTextId(id);
}
