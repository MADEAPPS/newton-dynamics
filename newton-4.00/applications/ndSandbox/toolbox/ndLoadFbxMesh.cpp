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

#include "ndSandboxStdafx.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndLoadFbxMesh.h"
#include "ndDemoSkinMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"
#include "ndAnimationSequence.h"

using namespace ofbx;

#define D_ANIM_BASE_FREQ ndFloat32 (30.0f)

class fbxMeshEffectNodeGlobalNodeMap : public ndTree<ndMeshEffectNode*, const ofbx::Object*>
{
};

class fbxImportndMeshEffectNodeStackData
{
	public:
	fbxImportndMeshEffectNodeStackData()
	{
	}

	fbxImportndMeshEffectNodeStackData(const ofbx::Object* const fbxNode, ndMeshEffectNode* const parentNode)
		:m_fbxNode(fbxNode)
		, m_parentNode(parentNode)
	{
	}

	const ofbx::Object* m_fbxNode;
	ndMeshEffectNode* m_parentNode;
};

static ndMatrix GetCoordinateSystemMatrix(ofbx::IScene* const fbxScene)
{
	const ofbx::GlobalSettings* const globalSettings = fbxScene->getGlobalSettings();

	ndMatrix convertMatrix(ndGetIdentityMatrix());

	ndFloat32 scaleFactor = globalSettings->UnitScaleFactor;
	convertMatrix[0][0] = ndFloat32(scaleFactor / 100.0f);
	convertMatrix[1][1] = ndFloat32(scaleFactor / 100.0f);
	convertMatrix[2][2] = ndFloat32(scaleFactor / 100.0f);

	ndMatrix axisMatrix(ndGetZeroMatrix());
	axisMatrix.m_up[globalSettings->UpAxis] = ndFloat32(globalSettings->UpAxisSign);
	axisMatrix.m_front[globalSettings->FrontAxis] = ndFloat32(globalSettings->FrontAxisSign);
	axisMatrix.m_right = axisMatrix.m_front.CrossProduct(axisMatrix.m_up);
	axisMatrix = axisMatrix.Transpose();
	
	convertMatrix = axisMatrix * convertMatrix;

	return convertMatrix;
}

static ndInt32 GetChildrenNodes(const ofbx::Object* const node, ofbx::Object** buffer)
{
	ndInt32 count = 0;
	ndInt32 index = 0;
	while (ofbx::Object* child = node->resolveObjectLink(index))
	{
		if (child->isNode())
		{
			buffer[count] = child;
			count++;
			ndAssert(count < 1024);
		}
		index++;
	}
	return count;
}

static ndMatrix ofbxMatrix2dMatrix(const ofbx::Matrix& fbxMatrix)
{
	ndMatrix matrix;
	for (ndInt32 i = 0; i < 4; ++i)
	{
		for (ndInt32 j = 0; j < 4; ++j)
		{
			matrix[i][j] = ndFloat32 (fbxMatrix.m[i * 4 + j]);
		}
	}
	return matrix;
}

static void ImportMaterials(const ofbx::Mesh* const fbxMesh, ndMeshEffect* const mesh)
{
	ndArray<ndMeshEffect::ndMaterial>& materialArray = mesh->GetMaterials();
	
	ndInt32 materialCount = fbxMesh->getMaterialCount();
	if (materialCount == 0)
	{
		ndMeshEffect::ndMaterial defaultMaterial;
		materialArray.PushBack(defaultMaterial);
	}
	else
	{
		for (ndInt32 i = 0; i < materialCount; ++i)
		{
			ndMeshEffect::ndMaterial material;
			const ofbx::Material* const fbxMaterial = fbxMesh->getMaterial(i);
			ndAssert(fbxMaterial);

			ofbx::Color color = fbxMaterial->getDiffuseColor();
			material.m_diffuse = ndVector(color.r, color.g, color.b, 1.0f);
			
			color = fbxMaterial->getAmbientColor();
			material.m_ambient = ndVector(color.r, color.g, color.b, 1.0f);
			
			color = fbxMaterial->getSpecularColor();
			material.m_specular = ndVector(color.r, color.g, color.b, 1.0f);
			
			material.m_opacity = ndFloat32(fbxMaterial->getOpacityFactor());
			material.m_shiness = ndFloat32(fbxMaterial->getShininess());
			
			const ofbx::Texture* const texture = fbxMaterial->getTexture(ofbx::Texture::DIFFUSE);
			if (texture)
			{
				char textName[1024];
				ofbx::DataView dataView = texture->getRelativeFileName();
				dataView.toString(textName);
				char* namePtr = strrchr(textName, '\\');
				if (!namePtr)
				{
					namePtr = strrchr(textName, '/');
				}
				if (namePtr)
				{
					namePtr++;
				}
				else
				{
					namePtr = textName;
				}
				strncpy(material.m_textureName, namePtr, sizeof(material.m_textureName));
			}
 			else
			{
				strcpy(material.m_textureName, "default.tga");
			}
			materialArray.PushBack(material);
		}
	}
}


class dFbxAnimationTrack
{
	public:
	class dCurveValue
	{
		public:
		ndFloat32 m_x;
		ndFloat32 m_y;
		ndFloat32 m_z;
		ndFloat32 m_time;
	};

	class dCurve : public ndList <dCurveValue>
	{
		public:
		dCurve()
			:ndList <dCurveValue>()
		{
		}

		dCurveValue Evaluate(ndFloat32 t) const
		{
			for (ndNode* ptr = GetFirst(); ptr->GetNext(); ptr = ptr->GetNext()) 
			{
				dCurveValue& info1 = ptr->GetNext()->GetInfo();
				if (info1.m_time >= t) 
				{
					dCurveValue& info0 = ptr->GetInfo();
					dCurveValue val;
					ndFloat32 param = (t - info0.m_time) / (info1.m_time - info0.m_time);
					val.m_x = info0.m_x + (info1.m_x - info0.m_x) * param;
					val.m_y = info0.m_y + (info1.m_y - info0.m_y) * param;
					val.m_z = info0.m_z + (info1.m_z - info0.m_z) * param;
					val.m_time = info0.m_time + (info1.m_time - info0.m_time) * param;
					return val;
				}
			}
			ndAssert(0);
			return dCurveValue();
		}
	};

	dFbxAnimationTrack()
	{
	}

	~dFbxAnimationTrack()
	{
	}

	void AddKeyframe(ndFloat32 time, const ndMatrix& matrix)
	{
		ndVector scale;
		ndVector euler0;
		ndVector euler1;
		ndMatrix transform;
		ndMatrix eigenScaleAxis;
		matrix.PolarDecomposition(transform, scale, eigenScaleAxis);
		transform.CalcPitchYawRoll(euler0, euler1);

		AddScale(time, scale.m_x, scale.m_y, scale.m_z);
		AddPosition(time, matrix.m_posit.m_x, matrix.m_posit.m_y, matrix.m_posit.m_z);
		AddRotation(time, euler0.m_x, euler0.m_y, euler0.m_z);
	}

	ndMatrix GetKeyframe(ndFloat32 time) const
	{
		dCurveValue scale(m_scale.Evaluate(time));
		dCurveValue position(m_position.Evaluate(time));
		dCurveValue rotation(m_rotation.Evaluate(time));

		ndMatrix scaleMatrix(ndGetIdentityMatrix());
		scaleMatrix[0][0] = scale.m_x;
		scaleMatrix[1][1] = scale.m_y;
		scaleMatrix[2][2] = scale.m_z;
		ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
		matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
		return matrix;
	}

	void OptimizeCurves()
	{
		if (m_scale.GetCount()) 
		{
			OptimizeCurve(m_scale);
		}
		if (m_position.GetCount()) 
		{
			OptimizeCurve(m_position);
		}
		if (m_rotation.GetCount()) 
		{
			for (dCurve::ndNode* node = m_rotation.GetFirst(); node->GetNext(); node = node->GetNext()) 
			{
				const dCurveValue& value0 = node->GetInfo();
				dCurveValue& value1 = node->GetNext()->GetInfo();
				value1.m_x = FixAngleAlias(value0.m_x, value1.m_x);
				value1.m_y = FixAngleAlias(value0.m_y, value1.m_y);
				value1.m_z = FixAngleAlias(value0.m_z, value1.m_z);
			}

			OptimizeCurve(m_rotation);
		}
	}

	void ApplyTransform(const ndMatrix& transform)
	{
		ndMatrix invert(transform.Inverse4x4());
		dCurve::ndNode* scaleNode = m_scale.GetFirst();
		dCurve::ndNode* positNode = m_position.GetFirst();
		for (dCurve::ndNode* rotationNode = m_rotation.GetFirst(); rotationNode; rotationNode = rotationNode->GetNext()) 
		{
			ndVector euler0;
			ndVector euler1;

			dCurveValue& scaleValue = scaleNode->GetInfo();
			dCurveValue& positValue = positNode->GetInfo();
			dCurveValue& rotationValue = rotationNode->GetInfo();

			ndMatrix scaleMatrix(ndGetIdentityMatrix());
			scaleMatrix[0][0] = scaleValue.m_x;
			scaleMatrix[1][1] = scaleValue.m_y;
			scaleMatrix[2][2] = scaleValue.m_z;
			ndMatrix m(scaleMatrix * ndPitchMatrix(rotationValue.m_x) * ndYawMatrix(rotationValue.m_y) * ndRollMatrix(rotationValue.m_z));
			m.m_posit = ndVector(positValue.m_x, positValue.m_y, positValue.m_z, 1.0f);
			ndMatrix matrix(invert * m * transform);

			ndVector scale;
			ndMatrix output;
			ndMatrix eigenScaleAxis;
			matrix.PolarDecomposition(output, scale, eigenScaleAxis);
			output.CalcPitchYawRoll(euler0, euler1);
			//dTrace(("%d %f %f %f\n", m_rotation.GetCount(), euler0.m_x * dRadToDegree, euler0.m_y * dRadToDegree, euler0.m_z * dRadToDegree));

			scaleValue.m_x = scale.m_x;
			scaleValue.m_y = scale.m_y;
			scaleValue.m_z = scale.m_z;

			rotationValue.m_x = euler0.m_x;
			rotationValue.m_y = euler0.m_y;
			rotationValue.m_z = euler0.m_z;

			positValue.m_x = output.m_posit.m_x;
			positValue.m_y = output.m_posit.m_y;
			positValue.m_z = output.m_posit.m_z;

			positNode = positNode->GetNext();
			scaleNode = scaleNode->GetNext();
		}
	}

	private:
	void AddScale(ndFloat32 time, ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		dCurveValue& value = m_scale.Append()->GetInfo();
		value.m_x = x;
		value.m_y = y;
		value.m_z = z;
		value.m_time = time;
	}

	void AddPosition(ndFloat32 time, ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		dCurveValue& value = m_position.Append()->GetInfo();
		value.m_x = x;
		value.m_y = y;
		value.m_z = z;
		value.m_time = time;
	}

	void AddRotation(ndFloat32 time, ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		dCurveValue& value = m_rotation.Append()->GetInfo();
		value.m_x = x;
		value.m_y = y;
		value.m_z = z;
		value.m_time = time;
	}

	ndFloat32 FixAngleAlias(ndFloat32 angleA, ndFloat32 angleB) const
	{
		ndFloat32 sinA = ndSin(angleA);
		ndFloat32 cosA = ndCos(angleA);
		ndFloat32 sinB = ndSin(angleB);
		ndFloat32 cosB = ndCos(angleB);

		ndFloat32 num = sinB * cosA - cosB * sinA;
		ndFloat32 den = cosA * cosB + sinA * sinB;
		angleB = angleA + ndAtan2(num, den);
		return angleB;
	}

	ndFloat32 Interpolate(ndFloat32 x0, ndFloat32 t0, ndFloat32 x1, ndFloat32 t1, ndFloat32 t) const
	{
		return x0 + (x1 - x0) * (t - t0) / (t1 - t0);
	}

	void OptimizeCurve(ndList<dCurveValue>& curve)
	{
		const ndFloat32 tol = 5.0e-5f;
		const ndFloat32 tol2 = tol * tol;
		for (dCurve::ndNode* node0 = curve.GetFirst(); node0->GetNext(); node0 = node0->GetNext()) 
		{
			const dCurveValue& value0 = node0->GetInfo();
			for (dCurve::ndNode* node1 = node0->GetNext()->GetNext(); node1; node1 = node1->GetNext()) 
			{
				const dCurveValue& value1 = node1->GetPrev()->GetInfo();
				const dCurveValue& value2 = node1->GetInfo();
				ndVector p1(value1.m_x, value1.m_y, value1.m_z, ndFloat32(0.0f));
				ndVector p2(value2.m_x, value2.m_y, value2.m_z, ndFloat32(0.0f));
		
				ndFloat32 dist_x = value1.m_x - Interpolate(value0.m_x, value0.m_time, value2.m_x, value2.m_time, value1.m_time);
				ndFloat32 dist_y = value1.m_y - Interpolate(value0.m_y, value0.m_time, value2.m_y, value2.m_time, value1.m_time);
				ndFloat32 dist_z = value1.m_z - Interpolate(value0.m_z, value0.m_time, value2.m_z, value2.m_time, value1.m_time);
		
				ndVector err(dist_x, dist_y, dist_z, 0.0f);
				ndFloat32 mag2 = err.DotProduct(err).GetScalar();
				if (mag2 > tol2) 
				{
					break;
				}
				curve.Remove(node1->GetPrev());
			}
		}
	}

	dCurve m_scale;
	dCurve m_position;
	dCurve m_rotation;

	friend class dFbxAnimation;
};

class dFbxAnimation : public ndTree <dFbxAnimationTrack, ndString>
{
	public:
	dFbxAnimation()
		:ndTree <dFbxAnimationTrack, ndString>()
		,m_length(0.0f)
		,m_timestep(0.0f)
		,m_framesCount(0)
	{
	}

	dFbxAnimation(const dFbxAnimation& source, ndMeshEffectNode* const entity, const ndMatrix& matrix)
		:ndTree <dFbxAnimationTrack, ndString>()
		,m_length(source.m_length)
		,m_timestep(source.m_timestep)
		,m_framesCount(source.m_framesCount)
	{
		Iterator iter(source);
		for (iter.Begin(); iter; iter++)
		{
			Insert(iter.GetKey());
		}

		FreezeScale(entity, source);
		ApplyTransform(matrix);
		OptimizeCurves();
	}

	void OptimizeCurves()
	{
		Iterator iter(*this);
		for (iter.Begin(); iter; iter++)
		{
			dFbxAnimationTrack& track = iter.GetNode()->GetInfo();
			track.OptimizeCurves();
		}
	}

	void FreezeScale(ndMeshEffectNode* const entity, const dFbxAnimation& source)
	{
		ndMatrix parentMatrixStack[1024];
		ndMeshEffectNode* stackPool[1024];

		ndFloat32 deltaTimeAcc = ndFloat32 (0.0f);
		for (ndInt32 i = 0; i < m_framesCount; ++i)
		{
			for (ndMeshEffectNode* node = (ndMeshEffectNode*)entity->GetFirst(); node; node = (ndMeshEffectNode*)node->GetNext())
			{
				const dFbxAnimation::ndNode* const aniNode = source.Find(node->GetName());
				if (aniNode)
				{
					const dFbxAnimationTrack& track = aniNode->GetInfo();
					const ndMatrix matrix(track.GetKeyframe(deltaTimeAcc));
					ndAssert(0);
					//node->SetMeshMatrix(matrix);
				}
			}

			ndInt32 stack = 1;
			stackPool[0] = entity;
			parentMatrixStack[0] = ndGetIdentityMatrix();
			while (stack)
			{
				stack--;
				ndMatrix parentMatrix(parentMatrixStack[stack]);
				ndMeshEffectNode* const rootNode = stackPool[stack];

				ndAssert(0);
				//ndMatrix transform(rootNode->GetMeshMatrix() * parentMatrix);
				ndMatrix transform(parentMatrix);
				ndMatrix matrix;
				ndMatrix stretchAxis;
				ndVector scale;
				transform.PolarDecomposition(matrix, scale, stretchAxis);

				ndMatrix scaledAxis(ndGetIdentityMatrix());
				scaledAxis[0][0] = scale[0];
				scaledAxis[1][1] = scale[1];
				scaledAxis[2][2] = scale[2];
				ndMatrix newParentMatrix(stretchAxis * scaledAxis);

				dFbxAnimation::ndNode* const aniNode = Find(rootNode->GetName());
				if (aniNode)
				{
					dFbxAnimationTrack& track = aniNode->GetInfo();
					track.AddKeyframe(deltaTimeAcc, matrix);
				}

				for (ndMeshEffectNode* node = (ndMeshEffectNode*)rootNode->GetChild(); node; node = (ndMeshEffectNode*)node->GetSibling())
				{
					stackPool[stack] = node;
					parentMatrixStack[stack] = newParentMatrix;
					stack++;
				}
			}
			deltaTimeAcc += m_timestep;
		}
	}

	void ApplyTransform(const ndMatrix& transform)
	{
		Iterator iter(*this);
		for (iter.Begin(); iter; iter++)
		{
			dFbxAnimationTrack& track = iter.GetNode()->GetInfo();
			track.ApplyTransform(transform);
		}
	}

	ndAnimationSequence* CreateSequence(const char* const name) const
	{
		ndAnimationSequence* const sequence = new ndAnimationSequence;
		sequence->SetName(name);
		ndAssert(0);
		//sequence->m_period = m_length;

		Iterator iter(*this);
		for (iter.Begin(); iter; iter++)
		{
			const dFbxAnimationTrack& fbxTrack = iter.GetNode()->GetInfo();
			ndAnimationKeyFramesTrack* const track = sequence->AddTrack();

			track->m_name = iter.GetKey();
			//dTrace(("name: %s\n", track->m_name.GetStr()));

			const dFbxAnimationTrack::dCurve& position = fbxTrack.m_position;
			for (dFbxAnimationTrack::dCurve::ndNode* node = position.GetFirst(); node; node = node->GetNext())
			{
				dFbxAnimationTrack::dCurveValue& keyFrame = node->GetInfo();
				track->m_position.m_param.PushBack(keyFrame.m_time);
				track->m_position.PushBack(ndVector(keyFrame.m_x, keyFrame.m_y, keyFrame.m_z, ndFloat32(1.0f)));
			}

			const dFbxAnimationTrack::dCurve& rotation = fbxTrack.m_rotation;
			for (dFbxAnimationTrack::dCurve::ndNode* node = rotation.GetFirst(); node; node = node->GetNext())
			{
				dFbxAnimationTrack::dCurveValue& keyFrame = node->GetInfo();
				track->m_rotation.m_param.PushBack(keyFrame.m_time);
				const ndMatrix transform(ndPitchMatrix(keyFrame.m_x) * ndYawMatrix(keyFrame.m_y) * ndRollMatrix(keyFrame.m_z));
				const ndQuaternion quat(transform);
				ndAssert(quat.DotProduct(quat).GetScalar() > 0.999f);
				ndAssert(quat.DotProduct(quat).GetScalar() < 1.001f);
				track->m_rotation.PushBack(quat);
				//dTrace(("%f %f %f %f %f\n", keyFrame.m_param, quat.m_x, quat.m_y, quat.m_z, quat.m_w));
			}
		}

		return sequence;
	}

	ndFloat32 m_length;
	ndFloat32 m_timestep;
	ndInt32 m_framesCount;
};

static void LoadAnimationCurve(ofbx::IScene* const, const ofbx::Object* const bone, const ofbx::AnimationLayer* const animLayer, dFbxAnimation& animation)
{
	const ofbx::AnimationCurveNode* const scaleNode = animLayer->getCurveNode(*bone, "Lcl Scaling");
	const ofbx::AnimationCurveNode* const rotationNode = animLayer->getCurveNode(*bone, "Lcl Rotation");
	const ofbx::AnimationCurveNode* const translationNode = animLayer->getCurveNode(*bone, "Lcl Translation");

	dFbxAnimationTrack& track = animation.Insert(bone->name)->GetInfo();

	Vec3 scale;
	Vec3 rotation;
	Vec3 translation;

	ndFloat32 timeAcc = 0.0f;
	ndFloat32 timestep = animation.m_timestep;

	ndVector scale1;
	ndVector euler0;
	ndVector euler1;
	ndMatrix transform;
	ndMatrix eigenScaleAxis;
	ndMatrix boneMatrix(ofbxMatrix2dMatrix(bone->getLocalTransform()));
	boneMatrix.PolarDecomposition(transform, scale1, eigenScaleAxis);
	transform.CalcPitchYawRoll(euler0, euler1);
	for (ndInt32 i = 0; i < animation.m_framesCount; ++i)
	{
		scale.x = scale1.m_x;
		scale.y = scale1.m_y;
		scale.z = scale1.m_z;
		rotation.x = euler0.m_x;
		rotation.y = euler0.m_y;
		rotation.z = euler0.m_z;
		translation.x = transform.m_posit.m_x;
		translation.y = transform.m_posit.m_y;
		translation.z = transform.m_posit.m_z;

		if (scaleNode)
		{
			scale = scaleNode->getNodeLocalTransform(timeAcc);
		}
		if (rotationNode)
		{
			rotation = rotationNode->getNodeLocalTransform(timeAcc);
		}
		if (translationNode)
		{
			translation = translationNode->getNodeLocalTransform(timeAcc);
		}
		ndMatrix matrix(ofbxMatrix2dMatrix(bone->evalLocal(translation, rotation, scale)));
		track.AddKeyframe(timeAcc, matrix);

		timeAcc += timestep;
	}
}

static void LoadAnimationLayer(ofbx::IScene* const fbxScene, const ofbx::AnimationLayer* const animLayer, dFbxAnimation& animation)
{
	ndInt32 stack = 0;
	ofbx::Object* stackPool[1024];
	const ofbx::Object* const rootNode = fbxScene->getRoot();
	ndAssert(rootNode);
	stack = GetChildrenNodes(rootNode, stackPool);

	const ofbx::TakeInfo* const animationInfo = fbxScene->getTakeInfo(0);
	//animation.m_length = ndFloat32(animationInfo->local_time_to - animationInfo->local_time_from);

	ndFloat32 period = ndFloat32(animationInfo->local_time_to - animationInfo->local_time_from);
	ndFloat32 framesFloat = period * D_ANIM_BASE_FREQ;

	ndInt32 frames = ndInt32(ndFloor(framesFloat));
	ndAssert(frames > 0);
	ndFloat32 timestep = period / (ndFloat32)frames;

	animation.m_length = period;
	animation.m_timestep = timestep;
	animation.m_framesCount = frames;

	while (stack)
	{
		stack--;
		ofbx::Object* const bone = stackPool[stack];
		LoadAnimationCurve(fbxScene, bone, animLayer, animation);

		stack += GetChildrenNodes(bone, &stackPool[stack]);
		ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0]) - 64));
	}
}

static void LoadAnimation(ofbx::IScene* const fbxScene, dFbxAnimation& animation)
{
	ndInt32 animationCount = fbxScene->getAnimationStackCount();
	for (int i = 0; i < animationCount; ++i)
	{
		const ofbx::AnimationStack* const animStack = fbxScene->getAnimationStack(i);
		
		ndInt32 layerCount = 0;
		while (const ofbx::AnimationLayer* const animLayer = animStack->getLayer(layerCount))
		{
			LoadAnimationLayer(fbxScene, animLayer, animation);
			layerCount++;
		}
	}

	animation.OptimizeCurves();
}

static void ImportMeshNode(ofbx::Object* const fbxNode, fbxMeshEffectNodeGlobalNodeMap& nodeMap)
{
	const ofbx::Mesh* const fbxMesh = (ofbx::Mesh*)fbxNode;

	ndAssert(nodeMap.Find(fbxNode));
	ndMeshEffectNode* const entity = nodeMap.Find(fbxNode)->GetInfo();
	ndSharedPtr<ndMeshEffect> mesh(new ndMeshEffect());
	
	ndMatrix pivotMatrix(ofbxMatrix2dMatrix(fbxMesh->getGeometricMatrix()));
	entity->m_meshMatrix = pivotMatrix;
	
	const ofbx::Geometry* const geom = fbxMesh->getGeometry();
	const ofbx::Vec3* const vertices = geom->getVertices();
	ndInt32 indexCount = geom->getIndexCount();
	
	ndArray<ndInt32> indexArray;
	indexArray.SetCount(indexCount);
	memcpy(&indexArray[0], geom->getFaceIndices(), indexCount * sizeof(ndInt32));
	
	ndInt32 faceCount = 0;
	for (ndInt32 i = 0; i < indexCount; ++i)
	{
		if (indexArray[i] < 0)
		{
			faceCount++;
		}
	}
	
	ndArray<ndInt32> faceIndexArray;
	ndArray<ndInt32> faceMaterialArray;

	faceIndexArray.SetCount(faceCount);
	faceMaterialArray.SetCount(faceCount);
	
	ImportMaterials(fbxMesh, *mesh);
	
	ndInt32 count = 0;
	ndInt32 faceIndex = 0;
	const ndArray<ndMeshEffect::ndMaterial>& materialArray = mesh->GetMaterials();
	ndInt32 materialId = (materialArray.GetCount() <= 1) ? 0 : -1;
	for (ndInt32 i = 0; i < indexCount; ++i)
	{
		count++;
		if (indexArray[i] < 0)
		{
			indexArray[i] = -indexArray[i] - 1;
			faceIndexArray[faceIndex] = count;
			if (materialId == 0)
			{
				faceMaterialArray[faceIndex] = materialId;
			}
			else
			{
				ndInt32 fbxMatIndex = geom->getMaterials()[faceIndex];
				faceMaterialArray[faceIndex] = fbxMatIndex;
			}
			count = 0;
			faceIndex++;
		}
	}
	
	ndMeshEffect::dMeshVertexFormat format;
	format.m_vertex.m_data = &vertices[0].x;
	format.m_vertex.m_indexList = &indexArray[0];
	format.m_vertex.m_strideInBytes = sizeof(ofbx::Vec3);
	
	format.m_faceCount = faceCount;
	format.m_faceIndexCount = &faceIndexArray[0];
	format.m_faceMaterial = &faceMaterialArray[0];
	
	ndArray<ndVector> normalArray;
	if (geom->getNormals())
	{
		//normalArray.Resize(indexCount);
		normalArray.SetCount(indexCount);
		const ofbx::Vec3* const normals = geom->getNormals();
		for (ndInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec3 n = normals[i];
			normalArray[i] = ndVector(ndFloat32(n.x), ndFloat32(n.y), ndFloat32(n.z), ndFloat32(0.0f));
		}
	
		format.m_normal.m_data = &normalArray[0].m_x;
		format.m_normal.m_indexList = &indexArray[0];
		format.m_normal.m_strideInBytes = sizeof(ndVector);
	}
	
	ndArray<ndVector> uvArray;
	if (geom->getUVs())
	{
		//uvArray.Resize(indexCount);
		uvArray.SetCount(indexCount);
		const ofbx::Vec2* const uv = geom->getUVs();
		for (ndInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec2 n = uv[i];
			uvArray[i] = ndVector(ndFloat32(n.x), ndFloat32(n.y), ndFloat32(0.0f), ndFloat32(0.0f));
		}
		format.m_uv0.m_data = &uvArray[0].m_x;
		format.m_uv0.m_indexList = &indexArray[0];
		format.m_uv0.m_strideInBytes = sizeof(ndVector);
	}
	
	// import skin if there is any
	if (geom->getSkin())
	{
		const ofbx::Skin* const skin = geom->getSkin();
		ndInt32 clusterCount = skin->getClusterCount();
	
		ndTree <const ofbx::Cluster*, const Object*> clusterBoneMap;
		for (ndInt32 i = 0; i < clusterCount; ++i)
		{
			const ofbx::Cluster* const cluster = skin->getCluster(i);
			const ofbx::Object* const link = cluster->getLink();
			clusterBoneMap.Insert(cluster, link);
		}
	
		for (int i = 0; i < clusterCount; ++i)
		{
			const ofbx::Cluster* const fbxCluster = skin->getCluster(i);
			const ofbx::Object* const fbxBone = fbxCluster->getLink();
			if (nodeMap.Find(fbxBone))
			{
				ndMeshEffect::dVertexCluster* const cluster = mesh->CreateCluster(fbxBone->name);
	
				ndAssert(fbxCluster->getIndicesCount() == fbxCluster->getWeightsCount());
				ndInt32 clusterIndexCount = fbxCluster->getIndicesCount();
				const ndInt32* const indices = fbxCluster->getIndices();
				const double* const weights = fbxCluster->getWeights();
				for (ndInt32 j = 0; j < clusterIndexCount; ++j)
				{
					cluster->m_vertexIndex.PushBack(indices[j]);
					cluster->m_vertexWeigh.PushBack(ndFloat32(weights[j]));
				}
			}
		}
	}
	
	mesh->BuildFromIndexList(&format);
	entity->SetMesh(mesh);
	//mesh->RepairTJoints();
}

static ndMeshEffectNode* LoadMeshEffectNodeHierarchy(ofbx::IScene* const fbxScene, fbxMeshEffectNodeGlobalNodeMap& nodeMap)
{
	ndInt32 stack = 0;
	ndFixSizeArray<ofbx::Object*, 1024> buffer;
	ndFixSizeArray <fbxImportndMeshEffectNodeStackData, 1024> nodeStack;

	buffer.SetCount(1024);
	nodeStack.SetCount(1024);

	const ofbx::Object* const rootNode = fbxScene->getRoot();
	ndAssert(rootNode);
	stack = GetChildrenNodes(rootNode, &buffer[0]);
	
	ndMeshEffectNode* rootEntity = nullptr;
	if (stack > 1)
	{
		rootEntity = new ndMeshEffectNode(nullptr);
		rootEntity->SetName("dommyRoot");
	}
	
	for (ndInt32 i = 0; i < stack; ++i)
	{
		ofbx::Object* const child = buffer[stack - i - 1];
		nodeStack[i] = fbxImportndMeshEffectNodeStackData(child, rootEntity);
	}
	
	while (stack)
	{
		stack--;
		fbxImportndMeshEffectNodeStackData data(nodeStack[stack]);
	
		ndMeshEffectNode* const node = new ndMeshEffectNode(data.m_parentNode);
		if (!rootEntity)
		{
			rootEntity = node;
		}
	
		const ndMatrix localMatrix(ofbxMatrix2dMatrix(data.m_fbxNode->getLocalTransform()));
	
		node->SetName(data.m_fbxNode->name);
		node->m_matrix = localMatrix;
	
		nodeMap.Insert(node, data.m_fbxNode);
		const ndInt32 count = GetChildrenNodes(data.m_fbxNode, &buffer[0]);
		for (ndInt32 i = 0; i < count; ++i)
		{
			ofbx::Object* const child = buffer[count - i - 1];
			nodeStack[stack] = fbxImportndMeshEffectNodeStackData(child, node);
			stack++;
			ndAssert(stack < ndInt32(sizeof(nodeStack) / sizeof(nodeStack[0])));
		}
	}

	return rootEntity;
}

static ndMeshEffectNode* FbxToMeshEffectNode(ofbx::IScene* const fbxScene)
{
	fbxMeshEffectNodeGlobalNodeMap nodeMap;
	ndMeshEffectNode* const entity = LoadMeshEffectNodeHierarchy(fbxScene, nodeMap);

	fbxMeshEffectNodeGlobalNodeMap::Iterator iter(nodeMap);
	for (iter.Begin(); iter; iter++)
	{
		ofbx::Object* const fbxNode = (ofbx::Object*)iter.GetKey();
		ofbx::Object::Type type = fbxNode->getType();
		switch (type)
		{
			case ofbx::Object::Type::MESH:
			{
				ImportMeshNode(fbxNode, nodeMap);
				break;
			}
	
			case ofbx::Object::Type::NULL_NODE:
			{
				break;
			}
	
			case ofbx::Object::Type::LIMB_NODE:
			{
				break;
			}
	
			//case FbxNodeAttribute::eLine:
			//{
			//	ImportLineShape(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
			//	break;
			//}
			//case FbxNodeAttribute::eNurbsCurve:
			//{
			//	ImportNurbCurveShape(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
			//	break;
			//}
			//case FbxNodeAttribute::eMarker:
			//case FbxNodeAttribute::eNurbs:
			//case FbxNodeAttribute::ePatch:
			//case FbxNodeAttribute::eCamera:
			//case FbxNodeAttribute::eCameraStereo:
			//case FbxNodeAttribute::eCameraSwitcher:
			//case FbxNodeAttribute::eLight:
			//case FbxNodeAttribute::eOpticalReference:
			//case FbxNodeAttribute::eOpticalMarker:
			//case FbxNodeAttribute::eTrimNurbsSurface:
			//case FbxNodeAttribute::eBoundary:
			//case FbxNodeAttribute::eNurbsSurface:
			//case FbxNodeAttribute::eShape:
			//case FbxNodeAttribute::eLODGroup:
			//case FbxNodeAttribute::eSubDiv:
			//case FbxNodeAttribute::eCachedEffect:
			//case FbxNodeAttribute::eUnknown:
			default:
				ndAssert(0);
				break;
		}
	}

	return entity;
}

static void FreezeScale(ndMeshEffectNode* const entity)
{
	ndInt32 stack = 1;
	ndMeshEffectNode* entBuffer[1024];
	ndMatrix parentMatrix[1024];
	entBuffer[0] = entity;
	parentMatrix[0] = ndGetIdentityMatrix();
	while (stack)
	{
		stack--;
		ndMatrix scaleMatrix(parentMatrix[stack]);
		ndMeshEffectNode* const ent = entBuffer[stack];

		ndMatrix transformMatrix;
		ndMatrix stretchAxis;
		ndVector scale;
		ndMatrix matrix(ent->m_matrix * scaleMatrix);
		matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
		ent->m_matrix = transformMatrix;
		scaleMatrix = ndMatrix(ndGetIdentityMatrix(), scale, stretchAxis);

		ndSharedPtr<ndMeshEffect> mesh = ent->GetMesh();
		if (*mesh)
		{
			matrix = ent->m_meshMatrix * scaleMatrix;
			matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
			ent->m_meshMatrix = transformMatrix;
			ndMatrix meshMatrix(ndGetIdentityMatrix(), scale, stretchAxis);
			mesh->ApplyTransform(meshMatrix);
		}

		for (ndMeshEffectNode* child = (ndMeshEffectNode*)ent->GetChild(); child; child = (ndMeshEffectNode*)child->GetSibling())
		{
			entBuffer[stack] = child;
			parentMatrix[stack] = scaleMatrix;
			stack++;
		}
	}
}

static void ApplyTransform(ndMeshEffectNode* const entity, const ndMatrix& transform)
{
	ndInt32 stack = 1;
	ndMeshEffectNode* entBuffer[1024];
	entBuffer[0] = entity;
	ndMatrix invTransform(transform.Inverse4x4());
	while (stack)
	{
		stack--;
		ndMeshEffectNode* const ent = entBuffer[stack];

		ndMatrix entMatrix(invTransform * ent->m_matrix * transform);
		ent->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> mesh = ent->GetMesh();
		if (*mesh)
		{
			ndMatrix meshMatrix(invTransform * ent->m_meshMatrix * transform);
			ent->m_meshMatrix = meshMatrix;
			mesh->ApplyTransform(transform);
		}

		for (ndMeshEffectNode* child = (ndMeshEffectNode*)ent->GetChild(); child; child = (ndMeshEffectNode*)child->GetSibling())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

ndMeshEffectNode* LoadFbxMeshEffectNode(const char* const meshName)
{
	char outPathName[1024];
	dGetWorkingFileName(meshName, outPathName);

	FILE* fp = fopen(outPathName, "rb");
	if (!fp)
	{
		ndAssert(0);
		return nullptr;
	}

	size_t readBytes = 0;
	readBytes++;
	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	ndArray<ofbx::u8> content;
	content.SetCount(file_size);
	readBytes = fread(&content[0], 1, size_t(file_size), fp);
	ofbx::IScene* const fbxScene = ofbx::load(&content[0], file_size, (ofbx::u64)ofbx::LoadFlags::TRIANGULATE);

	const ndMatrix convertMatrix(GetCoordinateSystemMatrix(fbxScene));

	ndMeshEffectNode* const meshEffectNode = FbxToMeshEffectNode(fbxScene);
	FreezeScale(meshEffectNode);
	ApplyTransform(meshEffectNode, convertMatrix);

	fbxScene->destroy();
	return meshEffectNode;
}

ndAnimationSequence* LoadFbxAnimation(const char* const fileName)
{
	char outPathName[1024];
	dGetWorkingFileName(fileName, outPathName);

	FILE* fp = fopen(outPathName, "rb");
	if (!fp)
	{
		ndAssert(0);
		return nullptr;
	}

	size_t readBytes = 0;
	readBytes++;
	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	ndArray<ofbx::u8> content;
	content.SetCount(file_size);
	readBytes = fread(&content[0], 1, size_t(file_size), fp);
	ofbx::IScene* const fbxScene = ofbx::load(&content[0], file_size, (ofbx::u64)ofbx::LoadFlags::TRIANGULATE);

	const ndMatrix convertMatrix(GetCoordinateSystemMatrix(fbxScene));

	dFbxAnimation animation;
	LoadAnimation(fbxScene, animation);
	//ndMeshEffectNode* const entity = FbxToEntity(fbxScene);
	ndAssert(0);
	ndMeshEffectNode* const entity = FbxToMeshEffectNode(fbxScene);
	dFbxAnimation newAnimation(animation, entity, convertMatrix);

	delete entity;
	fbxScene->destroy();

	return newAnimation.CreateSequence(fileName);
}