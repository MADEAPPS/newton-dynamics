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

class ndAnimationSequence;

using namespace ofbx;

class ndFbxMeshLoader : public ndClassAlloc
{
	class ndFbxAnimationTrack;
	class ndFbxMeshEffectNodeGlobalNodeMap;
	class ndFbxImportMeshEffectNodeStackData;

	public:
	ndFbxMeshLoader();
	virtual ~ndFbxMeshLoader();

	virtual ndAnimationSequence* LoadAnimation(const char* const fullPathName);
	virtual ndMeshEffectNode* LoadMesh(const char* const fullPathName, bool loadAnimation = false);

	private:
	void FreezeScale(ndMeshEffectNode* const entity);
	void AlignToWorld(ndMeshEffectNode* const entity);
	void OptimizeCurve(ndMeshEffectNode::ndCurve& curve);
	ndMatrix ofbxMatrix2dMatrix(const ofbx::Matrix& fbxMatrix);
	void OptimizeRotationCurve(ndMeshEffectNode::ndCurve& curve);
	ndMatrix GetCoordinateSystemMatrix(ofbx::IScene* const fbxScene);
	ndMeshEffectNode* FbxToMeshEffectNode(ofbx::IScene* const fbxScene);
	void ApplyTransform(ndMeshEffectNode* const entity, const ndMatrix& transform);
	void ImportMaterials(const ofbx::Mesh* const fbxMesh, ndMeshEffect* const mesh);
	ndInt32 GetChildrenNodes(const ofbx::Object* const node, ofbx::Object** buffer);
	void LoadAnimation(const ofbx::IScene* const fbxScene, ndMeshEffectNode* const model);
	ndAnimationSequence* CreateSequence(ndMeshEffectNode* const model, const char* const name);
	void ImportMeshNode(ofbx::Object* const fbxNode, ndFbxMeshEffectNodeGlobalNodeMap& nodeMap);
	void ApplyAllTransforms(ndMeshEffectNode* const meshEffectNode, const ndMatrix& unitMatrix, const ndMatrix& upAxis);
	ndMeshEffectNode* LoadMeshEffectNodeHierarchy(ofbx::IScene* const fbxScene, ndFbxMeshEffectNodeGlobalNodeMap& nodeMap);
	ndMatrix GetKeyframe(ndMeshEffectNode::ndCurveValue& scale, ndMeshEffectNode::ndCurveValue& position, ndMeshEffectNode::ndCurveValue& rotation);
	void LoadAnimationLayer(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ofbx::IScene* const fbxScene, const ofbx::AnimationLayer* const animLayer);
	void LoadAnimationCurve(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ofbx::IScene* const, const ofbx::Object* const bone, const ofbx::AnimationLayer* const animLayer, ndFloat32 duration, ndFloat32 timestep, ndInt32 framesCount);
};

#endif