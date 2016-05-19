/////////////////////////////////////////////////////////////////////////////
// Name:        pyScens.i
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


/* File : example.i */
%module pyScene

%{
	#include "pyTypes.h"
	#include "pyBaseNodeInfo.h"
	#include "pyMesh.h"
	#include "pyScene.h"
	#include "pyObject.h"
	#include "pyTexture.h"
	#include "pyMaterial.h"
	#include "pyRigidBody.h"
%}

/* Let's just grab the original header file here */


%include "pyBaseNodeInfo.h"
%template(objInfo) pyBaseNodeInfo<dSceneNodeInfo>;
%template(meshInfo) pyBaseNodeInfo<dMeshNodeInfo>;
%template(texInfo) pyBaseNodeInfo<dTextureNodeInfo>;
%template(matInfo) pyBaseNodeInfo<dMaterialNodeInfo>;
%template(rigidBidyInfo) pyBaseNodeInfo<dRigidbodyNodeInfo>;


%include "carrays.i"
%include "pyTypes.h"
%include "pyMesh.h"
%include "pyScene.h"
%include "pyObject.h"
%include "pyTexture.h"
%include "pyMaterial.h"
%include "pyRigidBody.h"


%array_class(int, intArray);
%array_class(double, doubleArray);

