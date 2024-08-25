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

#include "ndModelStdafx.h"
#include "ndFbxMeshLoader.h"
#include "ndAnimationSequence.h"

using namespace ofbx;

#define ND_ANIM_BASE_FREQ ndFloat32 (30.0f)
#define ND_ANIM_ERROR_TOL ndFloat32 (5.0e-5f)
#define ND_ANIM_ERROR_TOL2 (ND_ANIM_ERROR_TOL * ND_ANIM_ERROR_TOL)

class ndFbxMeshLoader::ndFbx2MeshNodeStackData
{
	public:
	ndFbx2MeshNodeStackData()
	{
	}

	ndFbx2MeshNodeStackData(const ofbx::Object* const fbxNode, ndMesh* const parentNode)
		:m_fbxNode(fbxNode)
		,m_parentNode(parentNode)
	{
	}

	const ofbx::Object* m_fbxNode;
	ndMesh* m_parentNode;
};

class ndFbxMeshLoader::ndFbx2ndMeshNodeMap : public ndTree<ndMesh*, const ofbx::Object*>
{
};

class ndFbxMeshLoader::ndFbxAnimationTrack
{
	public:
	class dCurve : public ndMesh::ndCurve
	{
		public:
		dCurve()
			:ndMesh::ndCurve()
		{
		}
	};

	ndFbxAnimationTrack()
	{
	}

	void SetDuration(ndFloat32 duration)
	{
		m_scale.m_lenght = ndReal(duration);
		m_position.m_lenght = ndReal(duration);
		m_rotation.m_lenght = ndReal(duration);
	}

	void AddKeyframe(ndFloat32 time, const ndMatrix& matrix)
	{
		ndVector scale;
		ndVector euler1;
		ndMatrix transform;
		ndMatrix eigenScaleAxis;
		matrix.PolarDecomposition(transform, scale, eigenScaleAxis);
		ndVector euler0(transform.CalcPitchYawRoll(euler1));

		AddScale(time, scale.m_x, scale.m_y, scale.m_z);
		AddPosition(time, matrix.m_posit.m_x, matrix.m_posit.m_y, matrix.m_posit.m_z);
		AddRotation(time, euler0.m_x, euler0.m_y, euler0.m_z);
	}

	void AddScale(ndFloat32 time, ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		ndMesh::ndCurveValue& value = m_scale.Append()->GetInfo();
		value.m_x = ndReal(x);
		value.m_y = ndReal(y);
		value.m_z = ndReal(z);
		value.m_time = ndReal(time);
	}

	void AddPosition(ndFloat32 time, ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		ndMesh::ndCurveValue& value = m_position.Append()->GetInfo();
		value.m_x = ndReal(x);
		value.m_y = ndReal(y);
		value.m_z = ndReal(z);
		value.m_time = ndReal(time);
	}

	void AddRotation(ndFloat32 time, ndFloat32 x, ndFloat32 y, ndFloat32 z)
	{
		ndMesh::ndCurveValue& value = m_rotation.Append()->GetInfo();
		value.m_x = ndReal(x);
		value.m_y = ndReal(y);
		value.m_z = ndReal(z);
		value.m_time = ndReal(time);
	}

	dCurve m_scale;
	dCurve m_position;
	dCurve m_rotation;
};

ndFbxMeshLoader::ndFbxMeshLoader()
	:ndClassAlloc()
{
}

ndFbxMeshLoader::~ndFbxMeshLoader()
{
}

ndMatrix ndFbxMeshLoader::GetCoordinateSystemMatrix(ofbx::IScene* const fbxScene)
{
	const ofbx::GlobalSettings& globalSettings = *fbxScene->getGlobalSettings();

	ndMatrix convertMatrix(ndGetIdentityMatrix());

	ndFloat32 scaleFactor = globalSettings.UnitScaleFactor;
	convertMatrix[0][0] = ndFloat32(scaleFactor / 100.0f);
	convertMatrix[1][1] = ndFloat32(scaleFactor / 100.0f);
	convertMatrix[2][2] = ndFloat32(scaleFactor / 100.0f);

	ndMatrix axisMatrix(ndGetZeroMatrix());
	//axisMatrix.m_up[globalSettings->UpAxis] = ndFloat32(globalSettings->UpAxisSign);
	//axisMatrix.m_front[globalSettings->FrontAxis] = ndFloat32(globalSettings->FrontAxisSign);
	//axisMatrix.m_right = axisMatrix.m_front.CrossProduct(axisMatrix.m_up);
	//axisMatrix = axisMatrix.Transpose();

	switch (globalSettings.UpAxis)
	{
		case UpVector_AxisX:
			ndAssert(0);
			break;

		case UpVector_AxisY:
			//axisMatrix = ndPitchMatrix(-90.0f * ndDegreeToRad) * ndYawMatrix(90.0f * ndDegreeToRad);
			axisMatrix[0][2] = ndFloat32(-1.0f);
			axisMatrix[1][0] = ndFloat32(-1.0f);
			axisMatrix[2][1] = ndFloat32(1.0f);
			axisMatrix[3][3] = ndFloat32(1.0f);
			break;

		case UpVector_AxisZ:
			//axisMatrix.m_up[globalSettings->UpAxis] = ndFloat32(globalSettings->UpAxisSign);
			//axisMatrix.m_front[globalSettings->FrontAxis] = ndFloat32(globalSettings->FrontAxisSign);
			//axisMatrix.m_right = axisMatrix.m_front.CrossProduct(axisMatrix.m_up);
			//axisMatrix = axisMatrix.Transpose();

			axisMatrix[0][2] = ndFloat32(-1.0f);
			axisMatrix[1][0] = ndFloat32(-1.0f);
			axisMatrix[2][1] = ndFloat32(1.0f);
			axisMatrix[3][3] = ndFloat32(1.0f);
			break;
	}
	
	convertMatrix = axisMatrix * convertMatrix;

	return convertMatrix;
}

ndInt32 ndFbxMeshLoader::GetChildrenNodes(const ofbx::Object* const node, ofbx::Object** buffer)
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

ndMatrix ndFbxMeshLoader::ofbxMatrix2dMatrix(const ofbx::Matrix& fbxMatrix)
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

void ndFbxMeshLoader::ImportMaterials(const ofbx::Mesh* const fbxMesh, ndMeshEffect* const meshEffect)
{
	ndArray<ndMeshEffect::ndMaterial>& materialArray = meshEffect->GetMaterials();
	
	ndInt32 materialCount = fbxMesh->getMaterialCount();
	if (materialCount == 0)
	{
		ndMeshEffect::ndMaterial material;
		const ofbx::Geometry* const geom = fbxMesh->getGeometry();
		Color color(geom->getRgbDisplayColor());
		material.m_diffuse = ndVector(color.r, color.g, color.b, 1.0f);
		material.m_ambient = ndVector(color.r, color.g, color.b, 1.0f);
		material.m_specular = ndVector(color.r, color.g, color.b, 1.0f);

		materialArray.PushBack(material);
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
				strcpy(material.m_textureName, "default.png");
			}
			materialArray.PushBack(material);
		}
	}
}

ndMatrix ndFbxMeshLoader::GetKeyframe(ndMesh::ndCurveValue& scale, ndMesh::ndCurveValue& position, ndMesh::ndCurveValue& rotation)
{
	ndMatrix scaleMatrix(ndGetIdentityMatrix());
	scaleMatrix[0][0] = scale.m_x;
	scaleMatrix[1][1] = scale.m_y;
	scaleMatrix[2][2] = scale.m_z;
	ndMatrix matrix(scaleMatrix * ndPitchMatrix(rotation.m_x) * ndYawMatrix(rotation.m_y) * ndRollMatrix(rotation.m_z));
	matrix.m_posit = ndVector(position.m_x, position.m_y, position.m_z, 1.0f);
	return matrix;
}

void ndFbxMeshLoader::FreezeScale(ndMesh* const mesh)
{
	ndInt32 stack = 1;
	ndMatrix parentMatrix[1024];
	ndMesh* entBuffer[1024];
	
	entBuffer[0] = mesh;
	parentMatrix[0] = ndGetIdentityMatrix();

	while (stack)
	{
		stack--;
		ndMatrix scaleMatrix(parentMatrix[stack]);
		ndMesh* const meshNode = entBuffer[stack];

		ndVector scale;
		ndMatrix stretchAxis;
		ndMatrix transformMatrix;
		
		ndMatrix matrix(meshNode->m_matrix * scaleMatrix);
		matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
		meshNode->m_matrix = transformMatrix;
		scaleMatrix = ndMatrix(ndGetIdentityMatrix(), scale, stretchAxis);

		ndSharedPtr<ndMeshEffect> effectMesh (meshNode->GetMesh());
		if (*effectMesh)
		{
			matrix = meshNode->m_meshMatrix * scaleMatrix;
			matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
			meshNode->m_meshMatrix = transformMatrix;
			ndMatrix meshMatrix(ndGetIdentityMatrix(), scale, stretchAxis);
			effectMesh->ApplyTransform(meshMatrix);
		}

		ndMesh::ndCurve& scaleCurve = meshNode->GetScaleCurve();
		ndMesh::ndCurve& positCurve = meshNode->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = meshNode->GetRotationCurve();
		if (scaleCurve.GetCount() || positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndMesh::ndCurve::ndNode* scaleNode = scaleCurve.GetFirst();
			ndMesh::ndCurve::ndNode* positNode = positCurve.GetFirst();
			ndMesh::ndCurve::ndNode* rotationNode = rotationCurve.GetFirst();

			ndAssert(scaleCurve.GetCount() == positCurve.GetCount());
			ndAssert(scaleCurve.GetCount() == rotationCurve.GetCount());

			ndMatrix parentAnimScale (parentMatrix[stack]);
			for (ndInt32 i = 0; i < scaleCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& scaleValue = scaleNode->GetInfo();
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

				ndVector animScale;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(GetKeyframe(scaleValue, positValue, rotationValue) * parentAnimScale);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);

				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));

				rotationValue.m_x = ndReal(euler.m_x);
				rotationValue.m_y = ndReal(euler.m_y);
				rotationValue.m_z = ndReal(euler.m_z);

				positValue.m_x = ndReal(animTransformMatrix.m_posit.m_x);
				positValue.m_y = ndReal(animTransformMatrix.m_posit.m_y);
				positValue.m_z = ndReal(animTransformMatrix.m_posit.m_z);

				//scaleValue.m_x = animScale.m_x;
				//scaleValue.m_y = animScale.m_y;
				//scaleValue.m_z = animScale.m_z;

				scaleValue.m_x = 1.0f;
				scaleValue.m_y = 1.0f;
				scaleValue.m_z = 1.0f;

				scaleNode = scaleNode->GetNext();
				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
		}

		for (ndMesh* child = meshNode->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			parentMatrix[stack] = scaleMatrix;
			stack++;
		}
	}
}

void ndFbxMeshLoader::ApplyTransform(ndMesh* const mesh, const ndMatrix& transform)
{
	mesh->ApplyTransform(transform);
}

void ndFbxMeshLoader::AlignToWorld(ndMesh* const mesh)
{
	ndMesh* entBuffer[1024];

	ndInt32 stack = 0;
	ndMatrix rotation(mesh->m_matrix);
	ndVector posit(rotation.m_posit);
	rotation.m_posit = ndVector::m_wOne;

	ndMatrix invRotation(rotation.OrthoInverse());
	for (ndMesh* child = mesh->GetFirstChild(); child; child = child->GetNext())
	{
		entBuffer[stack] = child;
		stack++;
	}

	mesh->m_matrix = ndGetIdentityMatrix();
	mesh->m_matrix.m_posit = posit;
	if (mesh->GetScaleCurve().GetCount())
	{
		ndAssert(0);
	}

	while (stack)
	{
		stack--;
		ndMesh* const meshNode = entBuffer[stack];

		ndMatrix entMatrix(invRotation * meshNode->m_matrix * rotation);
		meshNode->m_matrix = entMatrix;

		ndSharedPtr<ndMeshEffect> effectMesh (meshNode->GetMesh());
		if (*effectMesh)
		{
			ndMatrix meshMatrix(invRotation * meshNode->m_meshMatrix * rotation);
			meshNode->m_meshMatrix = meshMatrix;
			effectMesh->ApplyTransform(rotation);
		}

		ndMesh::ndCurve& scaleCurve = meshNode->GetScaleCurve();
		ndMesh::ndCurve& positCurve = meshNode->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = meshNode->GetRotationCurve();
		if (scaleCurve.GetCount() || positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndMesh::ndCurve::ndNode* positNode = positCurve.GetFirst();
			ndMesh::ndCurve::ndNode* rotationNode = rotationCurve.GetFirst();

			ndAssert(scaleCurve.GetCount() == positCurve.GetCount());
			ndAssert(scaleCurve.GetCount() == rotationCurve.GetCount());

			ndMesh::ndCurveValue scaleValue;
			scaleValue.m_x = 1.0f;
			scaleValue.m_y = 1.0f;
			scaleValue.m_z = 1.0f;
			for (ndInt32 i = 0; i < scaleCurve.GetCount(); ++i)
			{
				ndMesh::ndCurveValue& positValue = positNode->GetInfo();
				ndMesh::ndCurveValue& rotationValue = rotationNode->GetInfo();

				ndVector animScale;
				ndMatrix stretchAxis;
				ndMatrix animTransformMatrix;
				ndMatrix keyframe(invRotation * GetKeyframe(scaleValue, positValue, rotationValue) * rotation);
				keyframe.PolarDecomposition(animTransformMatrix, animScale, stretchAxis);

				ndVector euler0;
				ndVector euler(animTransformMatrix.CalcPitchYawRoll(euler0));

				rotationValue.m_x = ndReal(euler.m_x);
				rotationValue.m_y = ndReal(euler.m_y);
				rotationValue.m_z = ndReal(euler.m_z);

				positValue.m_x = ndReal(animTransformMatrix.m_posit.m_x);
				positValue.m_y = ndReal(animTransformMatrix.m_posit.m_y);
				positValue.m_z = ndReal(animTransformMatrix.m_posit.m_z);

				positNode = positNode->GetNext();
				rotationNode = rotationNode->GetNext();
			}
			scaleCurve.RemoveAll();
		}

		for (ndMesh* child = meshNode->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

void ndFbxMeshLoader::ApplyAllTransforms(ndMesh* const mesh, const ndMatrix& coordinateSystem)
{
	FreezeScale(mesh);
	ApplyTransform(mesh, coordinateSystem);
	AlignToWorld(mesh);
}

void ndFbxMeshLoader::ImportMeshNode(ofbx::Object* const fbxNode, ndFbx2ndMeshNodeMap& nodeMap)
{
	const ofbx::Mesh* const fbxMesh = (ofbx::Mesh*)fbxNode;

	ndAssert(nodeMap.Find(fbxNode));
	ndMesh* const entity = nodeMap.Find(fbxNode)->GetInfo();
	ndSharedPtr<ndMeshEffect> meshEffect(new ndMeshEffect());
	
	ndMatrix pivotMatrix(ofbxMatrix2dMatrix(fbxMesh->getGeometricMatrix()));
	entity->m_meshMatrix = pivotMatrix;
	
	const ofbx::Geometry* const geom = fbxMesh->getGeometry();
	ndInt32 indexCount = geom->getIndexCount();
	
	ndArray<ndMeshEffect::ndUV> uvArray;
	ndArray<ndMeshEffect::ndNormal> normalArray;
	ndArray<ndMeshEffect::ndVertexWeight> vertexWeights;

	ndArray<ndInt32> indexArray;
	ndArray<ndInt32> faceIndexArray;
	ndArray<ndInt32> faceMaterialArray;
	ndArray<ndInt32> vertexWeightsIndexArray;
	ndMeshEffect::ndMeshVertexFormat format;

	indexArray.SetCount(indexCount);
	ndMemCpy(&indexArray[0], geom->getFaceIndices(), indexCount);
	
	ndInt32 faceCount = 0;
	for (ndInt32 i = 0; i < indexCount; ++i)
	{
		if (indexArray[i] < 0)
		{
			faceCount++;
		}
	}
	
	ndInt32 count = 0;
	ndInt32 faceIndex = 0;

	faceIndexArray.SetCount(faceCount);
	faceMaterialArray.SetCount(faceCount);
	ImportMaterials(fbxMesh, *meshEffect);
	const ndArray<ndMeshEffect::ndMaterial>& materialArray = meshEffect->GetMaterials();
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

	const ofbx::Vec3* const vertices = geom->getVertices();
	format.m_vertex.m_data = (ndFloat64*)&vertices[0].x;
	format.m_vertex.m_indexList = &indexArray[0];
	format.m_vertex.m_strideInBytes = sizeof(ofbx::Vec3);
	
	format.m_faceCount = faceCount;
	format.m_faceIndexCount = &faceIndexArray[0];
	format.m_faceMaterial = &faceMaterialArray[0];
	
	if (geom->getNormals())
	{
		normalArray.SetCount(indexCount);
		const ofbx::Vec3* const normals = geom->getNormals();
		for (ndInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec3 n = normals[i];
			normalArray[i] = ndMeshEffect::ndNormal(ndFloat32(n.x), ndFloat32(n.y), ndFloat32(n.z));
		}
	
		format.m_normal.m_data = &normalArray[0].m_x;
		format.m_normal.m_indexList = &indexArray[0];
		format.m_normal.m_strideInBytes = sizeof(ndMeshEffect::ndNormal);
	}
	
	if (geom->getUVs())
	{
		uvArray.SetCount(indexCount);
		const ofbx::Vec2* const uv = geom->getUVs();
		for (ndInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec2 n = uv[i];
			//uvArray[i] = ndMeshEffect::ndUV(ndFloat32(n.x), ndFloat32(1.0f - n.y));
			uvArray[i] = ndMeshEffect::ndUV(ndFloat32(n.x), ndFloat32(n.y));
		}
		format.m_uv0.m_data = &uvArray[0].m_u;
		format.m_uv0.m_indexList = &indexArray[0];
		format.m_uv0.m_strideInBytes = sizeof(ndMeshEffect::ndUV);
	}
	
	// import skin if there is any
	//if (0)
	if (geom->getSkin())
	{
		const ofbx::Skin* const skin = geom->getSkin();
		ndInt32 clusterCount = skin->getClusterCount();

		vertexWeights.SetCount(geom->getVertexCount());
		for (ndInt32 i = 0; i < vertexWeights.GetCount(); ++i)
		{
			vertexWeights[i].Clear();
			vertexWeightsIndexArray.PushBack(i);
		}
		for (ndInt32 i = 0; i < clusterCount; ++i)
		{
			const ofbx::Cluster* const fbxCluster = skin->getCluster(i);
			ndInt32 clusterIndexCount = fbxCluster->getIndicesCount();
			if (clusterIndexCount)
			{
				const ofbx::Object* const fbxBone = fbxCluster->getLink();
				ndInt32 hashId = ndInt32(ndCRC64(fbxBone->name) & 0xffffffff);
				const ndInt32* const indices = fbxCluster->getIndices();
				const ndFloat64* const weights = fbxCluster->getWeights();
				for (ndInt32 j = 0; j < clusterIndexCount; ++j)
				{
					ndInt32 index = indices[j];
					vertexWeights[index].SetWeight(hashId, ndReal(weights[j]));
				}
			}
		}
		format.m_vertexWeight.m_data = &vertexWeights[0];
		format.m_vertexWeight.m_indexList = &vertexWeightsIndexArray[0];
		format.m_vertexWeight.m_strideInBytes = sizeof(ndMeshEffect::ndVertexWeight);
	}
	
	meshEffect->BuildFromIndexList(&format);
	entity->SetMesh(meshEffect);
	//mesh->RepairTJoints();
}

ndMesh* ndFbxMeshLoader::CreateMeshHierarchy(ofbx::IScene* const fbxScene, ndFbx2ndMeshNodeMap& nodeMap)
{
	ndInt32 stack = 0;
	ndFixSizeArray<ofbx::Object*, 1024> buffer;
	ndFixSizeArray <ndFbx2MeshNodeStackData, 1024> nodeStack;

	buffer.SetCount(1024);
	nodeStack.SetCount(1024);

	const ofbx::Object* const rootNode = fbxScene->getRoot();
	ndAssert(rootNode);
	stack = GetChildrenNodes(rootNode, &buffer[0]);
	
	// putting all node under a dummy root, makes Max and Blender fbx compatible
	ndMesh* const mesh = new ndMesh(nullptr);
	mesh->SetName("dommyRoot");

	const ofbx::GlobalSettings& globalSettings = *fbxScene->getGlobalSettings();
	if (globalSettings.UpAxis == UpVector_AxisY)
	{
		mesh->m_matrix = ndPitchMatrix(90.0f * ndDegreeToRad);
	}

	for (ndInt32 i = 0; i < stack; ++i)
	{
		ofbx::Object* const child = buffer[i];
		nodeStack[i] = ndFbx2MeshNodeStackData(child, mesh);
	}
	
	while (stack)
	{
		stack--;
		ndFbx2MeshNodeStackData data(nodeStack[stack]);
	
		ndMesh* const node = new ndMesh(data.m_parentNode);
		ndMatrix localMatrix(ofbxMatrix2dMatrix(data.m_fbxNode->getLocalTransform()));
	
		node->SetName(data.m_fbxNode->name);
		node->m_matrix = localMatrix;
	
		nodeMap.Insert(node, data.m_fbxNode);
		const ndInt32 count = GetChildrenNodes(data.m_fbxNode, &buffer[0]);
		for (ndInt32 i = 0; i < count; ++i)
		{
			ofbx::Object* const child = buffer[count - i - 1];
			nodeStack[stack] = ndFbx2MeshNodeStackData(child, node);
			stack++;
			ndAssert(stack < ndInt32(sizeof(nodeStack) / sizeof(nodeStack[0])));
		}
	}

	return mesh;
}

ndMesh* ndFbxMeshLoader::Fbx2ndMesh(ofbx::IScene* const fbxScene)
{
	ndFbx2ndMeshNodeMap nodeMap;
	ndMesh* const mesh = CreateMeshHierarchy(fbxScene, nodeMap);

	ndFbx2ndMeshNodeMap::Iterator iter(nodeMap);
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

	return mesh;
}

void ndFbxMeshLoader::LoadAnimationCurve(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ofbx::IScene* const, const ofbx::Object* const bone, const ofbx::AnimationLayer* const animLayer, ndFloat32 duration, ndInt32 framesCount)
{
	const ofbx::AnimationCurveNode* const scaleNode = animLayer->getCurveNode(*bone, "Lcl Scaling");
	const ofbx::AnimationCurveNode* const rotationNode = animLayer->getCurveNode(*bone, "Lcl Rotation");
	const ofbx::AnimationCurveNode* const translationNode = animLayer->getCurveNode(*bone, "Lcl Translation");

	if (scaleNode || rotationNode || translationNode)
	{
		ndFbxAnimationTrack& track = tracks.Insert(bone->name)->GetInfo();
		track.SetDuration(duration);

		Vec3 scale;
		Vec3 rotation;
		Vec3 translation;

		ndVector scale1;
		ndVector euler1;
		ndMatrix transform;
		ndMatrix eigenScaleAxis;
		ndMatrix boneMatrix(ofbxMatrix2dMatrix(bone->getLocalTransform()));
		boneMatrix.PolarDecomposition(transform, scale1, eigenScaleAxis);
		ndVector euler0(transform.CalcPitchYawRoll(euler1));
		euler0 = euler0.Scale(180.0f / ndPi);

		ndFloat32 timeAcc = 0.0f;
		ndFloat32 timestep = duration / ndFloat32 (framesCount);
		for (ndInt32 i = 0; i <= framesCount; ++i)
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

			if (i == framesCount)
			{
				timeAcc = duration;
			}

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
}

void ndFbxMeshLoader::LoadAnimationLayer(ndTree <ndFbxAnimationTrack, ndString>& tracks, const ofbx::IScene* const fbxScene, const ofbx::AnimationLayer* const animLayer)
{
	ndInt32 stack = 0;
	ofbx::Object* stackPool[1024];

	const ofbx::Object* const rootNode = fbxScene->getRoot();
	ndAssert(rootNode);
	stack = GetChildrenNodes(rootNode, stackPool);

	const ofbx::TakeInfo* const animationInfo = fbxScene->getTakeInfo(0);
	ndFloat32 period = ndFloat32(animationInfo->local_time_to - animationInfo->local_time_from);
	ndFloat32 framesFloat = period * ND_ANIM_BASE_FREQ;
	ndInt32 frames = ndInt32(ndFloor(framesFloat));

	while (stack)
	{
		stack--;
		ofbx::Object* const bone = stackPool[stack];
		LoadAnimationCurve(tracks, fbxScene, bone, animLayer, period, frames);

		stack += GetChildrenNodes(bone, &stackPool[stack]);
		ndAssert(stack < ndInt32(sizeof(stackPool) / sizeof(stackPool[0]) - 64));
	}
}

void ndFbxMeshLoader::LoadAnimation(const ofbx::IScene* const fbxScene, ndMesh* const model)
{
	ndInt32 animationCount = fbxScene->getAnimationStackCount();
	// only load one animation per file
	animationCount = 1;

	ndTree <ndFbxAnimationTrack, ndString> tracks;
	for (ndInt32 i = 0; i < animationCount; ++i)
	{
		const ofbx::AnimationStack* const animStack = fbxScene->getAnimationStack(i);

		ndInt32 layerCount = 0;
		while (const ofbx::AnimationLayer* const animLayer = animStack->getLayer(layerCount))
		{
			LoadAnimationLayer(tracks, fbxScene, animLayer);
			layerCount++;
			// only one layer per file 
			break;
		}
	}

	ndInt32 stack = 1;
	ndMesh* entBuffer[1024];
	entBuffer[0] = model;
	while (stack)
	{
		stack--;
		ndMesh* const ent = entBuffer[stack];
		ndTree <ndFbxAnimationTrack, ndString>::ndNode* node = tracks.Find(ent->GetName());
		if (node)
		{
			ndFbxAnimationTrack* const track = &node->GetInfo();

			ndMesh::ndCurve& scale = ent->GetScaleCurve();
			scale.RemoveAll();
			scale.m_lenght = track->m_scale.m_lenght;
			for (ndMesh::ndCurve::ndNode* srcNode = track->m_scale.GetFirst(); srcNode; srcNode = srcNode->GetNext())
			{
				scale.Append(srcNode->GetInfo());
			}

			ndMesh::ndCurve& posit = ent->GetPositCurve();
			posit.RemoveAll();
			posit.m_lenght = track->m_position.m_lenght;
			for (ndMesh::ndCurve::ndNode* srcNode = track->m_position.GetFirst(); srcNode; srcNode = srcNode->GetNext())
			{
				posit.Append(srcNode->GetInfo());
			}

			ndMesh::ndCurve& rotation = ent->GetRotationCurve();
			rotation.RemoveAll();
			rotation.m_lenght = track->m_rotation.m_lenght;
			for (ndMesh::ndCurve::ndNode* srcNode = track->m_rotation.GetFirst(); srcNode; srcNode = srcNode->GetNext())
			{
				rotation.Append(srcNode->GetInfo());
			}
		}

		for (ndMesh* child = ent->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

void ndFbxMeshLoader::OptimizeCurve(ndMesh::ndCurve& curve)
{
	auto Interpolate = [](ndFloat32 x0, ndFloat32 t0, ndFloat32 x1, ndFloat32 t1, ndFloat32 t)
	{
		return x0 + (x1 - x0) * (t - t0) / (t1 - t0);
	};

	for (ndMesh::ndCurve::ndNode* node0 = curve.GetFirst(); node0->GetNext(); node0 = node0->GetNext())
	{
		const ndMesh::ndCurveValue& value0 = node0->GetInfo();
		for (ndMesh::ndCurve::ndNode* node1 = node0->GetNext()->GetNext(); node1; node1 = node1->GetNext())
		{
			const ndMesh::ndCurveValue& value1 = node1->GetPrev()->GetInfo();
			const ndMesh::ndCurveValue& value2 = node1->GetInfo();

			ndFloat32 dist_x = value1.m_x - Interpolate(value0.m_x, value0.m_time, value2.m_x, value2.m_time, value1.m_time);
			ndFloat32 dist_y = value1.m_y - Interpolate(value0.m_y, value0.m_time, value2.m_y, value2.m_time, value1.m_time);
			ndFloat32 dist_z = value1.m_z - Interpolate(value0.m_z, value0.m_time, value2.m_z, value2.m_time, value1.m_time);

			ndVector err(dist_x, dist_y, dist_z, ndFloat32 (0.0f));
			ndFloat32 mag2 = err.DotProduct(err).GetScalar();
			if (mag2 > ND_ANIM_ERROR_TOL2)
			{
				break;
			}
			curve.Remove(node1->GetPrev());
		}
	}

	if (curve.GetCount() == 2)
	{
		const ndMesh::ndCurveValue& value0 = curve.GetFirst()->GetInfo();
		const ndMesh::ndCurveValue& value1 = curve.GetFirst()->GetNext()->GetInfo();

		ndFloat32 dist_x = value1.m_x - value0.m_x;
		ndFloat32 dist_y = value1.m_y - value0.m_y;
		ndFloat32 dist_z = value1.m_z - value0.m_z;

		ndVector err(dist_x, dist_y, dist_z, 0.0f);
		ndFloat32 mag2 = err.DotProduct(err).GetScalar();
		if (mag2 < ND_ANIM_ERROR_TOL2)
		{
			curve.Remove(curve.GetFirst()->GetNext());
		}
	}
}

void ndFbxMeshLoader::OptimizeRotationCurve(ndMesh::ndCurve& curve)
{
	ndMesh::ndCurveValue eulerRef(curve.GetFirst()->GetInfo());
	for (ndMesh::ndCurve::ndNode* node = curve.GetFirst()->GetNext(); node; node = node->GetNext())
	{
		ndMesh::ndCurveValue value(node->GetInfo());
		ndFloat32 angleError = ndAnglesSub(value.m_z, eulerRef.m_z);
		if (ndAbs(angleError) > ndPi * ndFloat32(0.5f))
		{
			ndMatrix m(ndPitchMatrix(value.m_x) * ndYawMatrix(value.m_y) * ndRollMatrix(value.m_z));
			ndVector euler1;
			ndVector euler(m.CalcPitchYawRoll(euler1));
			angleError = ndAnglesSub(euler.m_z, ndFloat32 (eulerRef.m_z));
			if (ndAbs(angleError) > ndPi * ndFloat32(0.5f))
			{
				euler = euler1;
			}
			value.m_x = ndReal(euler.m_x);
			value.m_y = ndReal(euler.m_y);
			value.m_z = ndReal(euler.m_z);
		}
		eulerRef = value;
	}

	OptimizeCurve(curve);
}

ndAnimationSequence* ndFbxMeshLoader::CreateSequence(ndMesh* const model, const char* const name)
{
	ndAnimationSequence* const sequence = new ndAnimationSequence;
	sequence->SetName(name);

	ndInt32 stack = 1;
	ndMesh* entBuffer[1024];
	entBuffer[0] = model;
	ndFloat32 duration = ndFloat32(0.0f);
	while (stack)
	{
		stack--;
		ndMesh* const meshNode = entBuffer[stack];

		ndMesh::ndCurve& scaleCurve = meshNode->GetScaleCurve();
		ndMesh::ndCurve& positCurve = meshNode->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = meshNode->GetRotationCurve();
		if (scaleCurve.GetCount() || positCurve.GetCount() || rotationCurve.GetCount())
		{
			ndAnimationKeyFramesTrack* const track = sequence->AddTrack();
			track->m_name = meshNode->GetName();

			if (positCurve.GetCount())
			{
				duration = ndMax(duration, ndFloat32(positCurve.GetLast()->GetInfo().m_time));
			}

			for (ndMesh::ndCurve::ndNode* srcNode = positCurve.GetFirst(); srcNode; srcNode = srcNode->GetNext())
			{
				ndMesh::ndCurveValue& keyFrame = srcNode->GetInfo();
				track->m_position.m_time.PushBack(keyFrame.m_time);
				track->m_position.PushBack(ndVector(keyFrame.m_x, keyFrame.m_y, keyFrame.m_z, ndFloat32(1.0f)));
			}

			if (rotationCurve.GetCount())
			{
				duration = ndMax(duration, ndFloat32(rotationCurve.GetLast()->GetInfo().m_time));
			}

			ndQuaternion rotation;
			for (ndMesh::ndCurve::ndNode* srcNode = rotationCurve.GetFirst(); srcNode; srcNode = srcNode->GetNext())
			{
				ndMesh::ndCurveValue& keyFrame = srcNode->GetInfo();

				const ndMatrix transform(ndPitchMatrix(keyFrame.m_x) * ndYawMatrix(keyFrame.m_y) * ndRollMatrix(keyFrame.m_z));
				ndQuaternion quat(transform);
				ndAssert(quat.DotProduct(quat).GetScalar() > 0.999f);
				ndAssert(quat.DotProduct(quat).GetScalar() < 1.001f);

				if (track->m_rotation.GetCount())
				{
					ndFloat32 dot = quat.DotProduct(rotation).GetScalar();
					if (dot < ndFloat32(0.0f))
					{
						quat = quat.Scale(ndFloat32(-1.0f));
					}
				}

				track->m_rotation.PushBack(quat);
				track->m_rotation.m_time.PushBack(keyFrame.m_time);

				rotation = quat;
			}
		}

		for (ndMesh* child = meshNode->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}

	sequence->m_duration = duration;
	return sequence;
}

void ndFbxMeshLoader::OptimizeAnimation(ndMesh* const model)
{
	ndInt32 stack = 1;
	ndMesh* entBuffer[1024];
	entBuffer[0] = model;
	while (stack)
	{
		stack--;
		ndMesh* const meshNode = entBuffer[stack];

		ndMesh::ndCurve& scaleCurve = meshNode->GetScaleCurve();
		ndMesh::ndCurve& positCurve = meshNode->GetPositCurve();
		ndMesh::ndCurve& rotationCurve = meshNode->GetRotationCurve();
		if (scaleCurve.GetCount() || positCurve.GetCount() || rotationCurve.GetCount())
		{
			OptimizeCurve(positCurve);
			OptimizeRotationCurve(rotationCurve);
		}

		for (ndMesh* child = meshNode->GetFirstChild(); child; child = child->GetNext())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

ndMesh* ndFbxMeshLoader::LoadMesh(const char* const fullPathName, bool loadAnimation)
{
	FILE* const file = fopen(fullPathName, "rb");
	if (!file)
	{
		ndAssert(0);
		return nullptr;
	}

	size_t readBytes = 0;
	readBytes++;
	fseek(file, 0, SEEK_END);
	long file_size = ftell(file);
	fseek(file, 0, SEEK_SET);
	ndArray<ofbx::u8> content;
	content.SetCount(file_size);
	readBytes = fread(&content[0], 1, size_t(file_size), file);
	fclose(file);

	ndSharedPtr<ofbx::IScene> fbxScene(ofbx::load(&content[0], ndInt32(file_size), (ofbx::u64)ofbx::LoadFlags::TRIANGULATE));

	const ndMatrix convertMatrix(GetCoordinateSystemMatrix(*fbxScene));
	ndMesh* const mesh = Fbx2ndMesh(*fbxScene);
	if (loadAnimation)
	{
		LoadAnimation(*fbxScene, mesh);
	}
	ApplyAllTransforms(mesh, convertMatrix);

	if (loadAnimation)
	{
		OptimizeAnimation(mesh);
	}
	return mesh;
}

ndAnimationSequence* ndFbxMeshLoader::LoadAnimation(const char* const fullPathName)
{
	ndSharedPtr<ndMesh> mesh(LoadMesh(fullPathName, true));
	ndAnimationSequence* const sequence = CreateSequence(*mesh, fullPathName);
	return sequence;
}