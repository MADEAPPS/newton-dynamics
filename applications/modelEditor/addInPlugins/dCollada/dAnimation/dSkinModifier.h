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


#ifndef _D_MESH_SKIN_MODIFIER_H_
#define _D_MESH_SKIN_MODIFIER_H_

#include <dAnimationStdAfx.h>
#include <dMesh.h>

class dBone;
class dModel;
class dMeshInstance;

class dSkinModifier: public dMeshInstance::dModifier
{
	public: 
	class dBoneVertexWeightData
	{
		public:
		int m_boneId;
		int m_vertexIndex;
		dFloat m_weight;
	};

	struct dBoneWeightIndex
	{
		int m_index[4];
	};

	dSkinModifier(const dMesh* mesh);
	virtual ~dSkinModifier();
	virtual dModifier* CreateCopy (const dMeshInstance& instance, const dModel& model) const;

//	void SetRootNode (const dBone* bone);
	void SetBindingPose(const dMeshInstance& mesh, const dModel& model, dBoneVertexWeightData* skinData, int skinDataCount); 

	void ApplyGlobalScale(dFloat scale);
	void ApplyGlobalTransform (const dMatrix& transform);

	int m_bonesCount;
//	const dBone* m_rootBone;
	dMatrix m_shapeBindMatrix;
	const dBone** m_skinnedBones;
	dVector* m_vertexWeight;
	dMatrix* m_bindingMatrices;
	dBoneWeightIndex* m_boneWeightIndex;
};


#endif 

