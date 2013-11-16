/////////////////////////////////////////////////////////////////////////////
// Name:        precompile header.h
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


// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#ifndef _D_FBX_STDAFX_H_
#define _D_FBX_STDAFX_H_


#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers

#include <windows.h>
#include <crtdbg.h>
#include <fbxsdk.h>

#include <dTree.h>
#include <dList.h>
#include <dContainersAlloc.h>

#include <dPluginStdafx.h>
#include <dUndoRedo.h>
#include <dSceneStdafx.h>

#include <dScene.h>
#include <dNodeInfo.h>
#include <dSceneGraph.h>
#include <dUndoCurrentScene.h>

//#include <dRootNodeInfo.h>
//#include <dMeshNodeInfo.h>
//#include <dSceneNodeInfo.h>
//#include <dTextureNodeInfo.h>
//#include <dMaterialNodeInfo.h>

#include <dPluginUtils.h>
#include <dPluginScene.h>
#include <dExportPlugin.h>
#include <dImportPlugin.h>
#include <dPluginInterface.h>

#endif
