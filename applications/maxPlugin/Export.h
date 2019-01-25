/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __NEWTON_MAX_EXPORTER_H__
#define __NEWTON_MAX_EXPORTER_H__


class Mtl;
class Mesh;
class INode;
class Options;
class Matrix3;
class TriObject;
class Interface;
class TextureList;
class MaterialMap;
class ObjectState;
class MaterialList;
class ExpInterface;
class MaterialFilter;



#include <dSceneStdAfx.h>
#include <dScene.h>
#include <dBoneNodeInfo.h>
#include <dMeshNodeInfo.h>
#include <dSceneNodeInfo.h>
#include <dTextureNodeInfo.h>
#include <dMaterialNodeInfo.h>
#include <dGeometryNodeSkinClusterInfo.h>

#include "Max.h"
#include "tab.h"


class Export
{
	public:
	class NodeMap: public dTree<dScene::dTreeNode*, INode*>
	{
	};

	class ImageCache: public dTree<dScene::dTreeNode*, dCRCTYPE>
	{};

	Export(const char* const name, ExpInterface* const expip, Interface* const ip, const Options& options);
	~Export(void);

	void LoadSkeletonAndGeomtry (INode* const node, dScene& scene);
	void LoadGeometries (INode* const node, dScene& scene, NodeMap& nodeMap);
	dScene::dTreeNode* LoadObject(INode* const node, ObjectState* const os, dScene& scene, NodeMap& nodeMap, ImageCache& imageCache, int& materialID, int& DefaultMatID);

	void GetNodeList (Tab<INode *> &nodeTab);

//	TriObject* GetTriObject (ObjectState* const os, int& deleteIt);
	dVector CalcVertexNormal (Mesh& max5Mesh, int faceNo, int faceIndex) const;

	PolyObject* GetPolyObject (ObjectState* const os, int& deleteIt);
	dVector CalcVertexNormal (MNMesh& max5Mesh, int faceNo, int faceIndex) const;

	Interface* m_ip;
	ExpInterface* m_expip;
	char m_path[256];
};

#endif