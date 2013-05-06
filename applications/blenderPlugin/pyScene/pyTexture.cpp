/////////////////////////////////////////////////////////////////////////////
// Name:        pyTextures.h
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
#include "pyTexture.h"

pyTexture::pyTexture(pyScene* scene, void* textureNode)
	:pyBaseNodeInfo<dTextureNodeInfo>(scene, textureNode)
{
}

pyTexture::~pyTexture(void)
{
}


int pyTexture::GetId()
{
	dTextureNodeInfo* info = GetInfo();
	return info->GetId();
}

const char* pyTexture::GetImageName()
{
	dTextureNodeInfo* info = GetInfo();
	return info->GetPathName();
}