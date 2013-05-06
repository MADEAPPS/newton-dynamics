/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#pragma once


#include "msPlugIn.h"
#include <dPluginStdafx.h>
#include <dScene.h>
#include <dBoneNodeInfo.h>
#include <dMeshNodeInfo.h>
#include <dSceneNodeInfo.h>
#include <dTextureNodeInfo.h>
#include <dMaterialNodeInfo.h>
#include <dGeometryNodeSkinModifierInfo.h>


struct msModel;

class BoneMap;

class Importer : public cMsPlugIn
{
	public:
	Importer ();
    virtual ~Importer ();


    int             GetType ();
    const char *    GetTitle ();
    int             Execute (msModel* pModel);

//	void EnumerateBones (dScene& scene, BoneMap& boneMap);
	void ConvertMeshToSkins(dScene& scene);
//	void MergeEqualMaterials(dScene& scene);
//  void AddSkeleton (msModel* pModel, dModel& model);

};

