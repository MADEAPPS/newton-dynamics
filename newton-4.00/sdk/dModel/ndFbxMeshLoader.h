/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _ND_FBX_MESH_LOADER_H_
#define _ND_FBX_MESH_LOADER_H_

#include "ndMesh.h"
class ndAnimationSequence;

using namespace ofbx;

class ndFbxMeshLoader : public ndClassAlloc
{
	class ndFbxAnimationTrack;
	class ndFbx2ndMeshNodeMap;
	class ndFbx2MeshNodeStackData;

	public:
	ndFbxMeshLoader();
	virtual ~ndFbxMeshLoader();

	virtual ndAnimationSequence* LoadAnimation(const char* const fullPathName);
	virtual ndMesh* LoadMesh(const char* const fullPathName, bool loadAnimation);

	private:
	void FreezeScale(ndMesh* const entity);
	void AlignToWorld(ndMesh* const entity);
	void OptimizeCurve(ndMesh::ndCurve& curve);
	void OptimizeAnimation(ndMesh* const model);
	ndMesh* Fbx2ndMesh(ofbx::IScene* const fbxScene);
	void OptimizeRotationCurve(ndMesh::ndCurve& curve);
	ndMatrix ofbxMatrix2dMatrix(const ofbx::Matrix& fbxMatrix);
	ndMatrix GetCoordinateSystemMatrix(ofbx::IScene* const fbxScene);
	void ApplyTransform(ndMesh* const entity, const ndMatrix& transform);
	void ApplyAllTransforms(ndMesh* const mesh, const ndMatrix& unitMatrix);
	void LoadAnimation(const ofbx::IScene* const fbxScene, ndMesh* const model);
	void ImportMeshNode(ofbx::Object* const fbxNode, ndFbx2ndMeshNodeMap& nodeMap);
	void ImportMaterials(const ofbx::Mesh* const fbxMesh, ndMeshEffect* const mesh);
	ndInt32 GetChildrenNodes(const ofbx::Object* const node, ofbx::Object** buffer);
	ndAnimationSequence* CreateSequence(ndMesh* const model, const char* const name);
	ndMesh* CreateMeshHierarchy(ofbx::IScene* const fbxScene, ndFbx2ndMeshNodeMap& nodeMap);
	ndMatrix GetKeyframe(ndMesh::ndCurveValue& scale, ndMesh::ndCurveValue& position, ndMesh::ndCurveValue& rotation);
	void LoadAnimationLayer(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ofbx::IScene* const fbxScene, const ofbx::AnimationLayer* const animLayer);
	void LoadAnimationCurve(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ofbx::IScene* const, const ofbx::Object* const bone, const ofbx::AnimationLayer* const animLayer, ndFloat32 duration, ndInt32 framesCount);
};

#endif