/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"

fbxDemoEntity::fbxDemoEntity(ndDemoEntity* const parent)
	:ndDemoEntity(dGetIdentityMatrix(), parent)
	,m_fbxMeshEffect(nullptr)
{
}

fbxDemoEntity::~fbxDemoEntity()
{
	if (m_fbxMeshEffect)
	{
		delete m_fbxMeshEffect;
	}
}

void fbxDemoEntity::CleanIntermediate()
{
	if (m_fbxMeshEffect)
	{
		delete m_fbxMeshEffect;
		m_fbxMeshEffect = nullptr;
	}

	for (fbxDemoEntity* child = (fbxDemoEntity*)GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
	{
		child->CleanIntermediate();
	}
}

void fbxDemoEntity::BuildRenderMeshes(ndDemoEntityManager* const scene)
{
	dInt32 stack = 1;
	fbxDemoEntity* entBuffer[1024];
	entBuffer[0] = this;
	while (stack)
	{
		stack--;
		fbxDemoEntity* const ent = entBuffer[stack];
	
		if (ent->m_fbxMeshEffect)
		{
			ndDemoMesh* const mesh = new ndDemoMesh(ent->GetName().GetStr(), ent->m_fbxMeshEffect, scene->GetShaderCache());
			ent->SetMesh(mesh, ent->GetMeshMatrix());
			mesh->Release();

			if ((ent->GetName().Find("hidden") >= 0) || (ent->GetName().Find("Hidden") >= 0))
			{
				mesh->m_isVisible = false;
			}
		}
	
		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

void fbxDemoEntity::ApplyTransform(const dMatrix& transform)
{
	dInt32 stack = 1;
	fbxDemoEntity* entBuffer[1024];
	entBuffer[0] = this;
	dMatrix invTransform(transform.Inverse4x4());
	while (stack)
	{
		stack--;
		fbxDemoEntity* const ent = entBuffer[stack];

		dMatrix entMatrix(invTransform * ent->GetRenderMatrix() * transform);
		ent->SetRenderMatrix(entMatrix);

		dQuaternion rot(entMatrix);
		ent->SetMatrixUsafe(rot, entMatrix.m_posit);
		ent->SetMatrixUsafe(rot, entMatrix.m_posit);

		if (ent->m_fbxMeshEffect)
		{
			dMatrix meshMatrix(invTransform * ent->GetMeshMatrix() * transform);
			ent->SetMeshMatrix(meshMatrix);
			ent->m_fbxMeshEffect->ApplyTransform(transform);
		}

		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

class fbxGlobalNoceMap : public dTree<fbxDemoEntity*, const ofbx::Object*>
{
};

class fbxImportStackData
{
	public:
	fbxImportStackData()
	{
	}

	fbxImportStackData(const ofbx::Object* const fbxNode, ndDemoEntity* const parentNode)
		:m_fbxNode(fbxNode)
		,m_parentNode(parentNode)
	{
	}

	const ofbx::Object* m_fbxNode;
	ndDemoEntity* m_parentNode;
};

static dMatrix GetCoordinateSystemMatrix(ofbx::IScene* const fbxScene)
{
	const ofbx::GlobalSettings* const globalSettings = fbxScene->getGlobalSettings();

	dMatrix convertMatrix(dGetIdentityMatrix());

	dFloat32 scaleFactor = globalSettings->UnitScaleFactor;
	convertMatrix[0][0] = dFloat32(scaleFactor / 100.0f);
	convertMatrix[1][1] = dFloat32(scaleFactor / 100.0f);
	convertMatrix[2][2] = dFloat32(scaleFactor / 100.0f);

	dMatrix axisMatrix(dGetZeroMatrix());
	axisMatrix.m_up[globalSettings->UpAxis] = dFloat32(globalSettings->UpAxisSign);
	axisMatrix.m_front[globalSettings->FrontAxis] = dFloat32(globalSettings->FrontAxisSign);
	axisMatrix.m_right = axisMatrix.m_front.CrossProduct(axisMatrix.m_up);
	axisMatrix = axisMatrix.Transpose();
	
	convertMatrix = axisMatrix * convertMatrix;

	return convertMatrix;
}

static dInt32 GetChildrenNodes(const ofbx::Object* const node, ofbx::Object** buffer)
{
	dInt32 count = 0;
	dInt32 index = 0;
	while (ofbx::Object* child = node->resolveObjectLink(index))
	{
		if (child->isNode())
		{
			buffer[count] = child;
			count++;
			dAssert(count < 1024);
		}
		index++;
	}
	return count;
}

static dMatrix ofbxMatrix2dMatrix(const ofbx::Matrix& fbxMatrix)
{
	dMatrix matrix;
	for (dInt32 i = 0; i < 4; i++)
	{
		for (dInt32 j = 0; j < 4; j++)
		{
			matrix[i][j] = dFloat32 (fbxMatrix.m[i * 4 + j]);
		}
	}
	return matrix;
}

static fbxDemoEntity* LoadHierarchy(ofbx::IScene* const fbxScene, fbxGlobalNoceMap& nodeMap)
{
	dInt32 stack = 0;
	ofbx::Object* buffer[1024];
	fbxImportStackData nodeStack[1024];
	const ofbx::Object* const rootNode = fbxScene->getRoot();
	dAssert(rootNode);

	fbxDemoEntity* entity = nullptr;
	if (rootNode) 
	{
		stack = GetChildrenNodes(rootNode, buffer);
		entity = (stack > 1) ? new fbxDemoEntity(nullptr) : nullptr;

		for (dInt32 i = 0; i < stack; i++)
		{
			ofbx::Object* const child = buffer[stack - i - 1];
			nodeStack[i] = fbxImportStackData(child, entity);
		}
	}

	while (stack)
	{
		stack--;
		fbxImportStackData data(nodeStack[stack]);

		fbxDemoEntity* const node = new fbxDemoEntity(data.m_parentNode);
		if (!entity)
		{
			entity = node;
		}

		dMatrix localMatrix(ofbxMatrix2dMatrix(data.m_fbxNode->getLocalTransform()));

		node->SetName(data.m_fbxNode->name);
		node->SetRenderMatrix(localMatrix);

		nodeMap.Insert(node, data.m_fbxNode);
		const dInt32 count = GetChildrenNodes(data.m_fbxNode, buffer);
		for (dInt32 i = 0; i < count; i++) 
		{
			ofbx::Object* const child = buffer[count - i - 1];
			nodeStack[stack] = fbxImportStackData(child, node);
			stack++;
			dAssert(stack < dInt32(sizeof(nodeStack) / sizeof(nodeStack[0])));
		}
	}
	return entity;
}

static void ImportMaterials(const ofbx::Mesh* const fbxMesh, ndMeshEffect* const mesh)
{
	dArray<ndMeshEffect::dMaterial>& materialArray = mesh->GetMaterials();
	
	dInt32 materialCount = fbxMesh->getMaterialCount();
	if (materialCount == 0)
	{
		ndMeshEffect::dMaterial defaultMaterial;
		materialArray.PushBack(defaultMaterial);
	}
	else
	{
		for (dInt32 i = 0; i < materialCount; i++)
		{
			ndMeshEffect::dMaterial material;
			const ofbx::Material* const fbxMaterial = fbxMesh->getMaterial(i);
			dAssert(fbxMaterial);

			ofbx::Color color = fbxMaterial->getDiffuseColor();
			material.m_diffuse = dVector(color.r, color.g, color.b, 1.0f);
			
			color = fbxMaterial->getAmbientColor();
			material.m_ambient = dVector(color.r, color.g, color.b, 1.0f);
			
			color = fbxMaterial->getSpecularColor();
			material.m_specular = dVector(color.r, color.g, color.b, 1.0f);
			
			material.m_opacity = dFloat32(fbxMaterial->getOpacityFactor());
			material.m_shiness = dFloat32(fbxMaterial->getShininess());
			
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

static void ImportMeshNode(ofbx::Object* const fbxNode, fbxGlobalNoceMap& nodeMap)
{
	const ofbx::Mesh* const fbxMesh = (ofbx::Mesh*)fbxNode;

	dAssert(nodeMap.Find(fbxNode));
	fbxDemoEntity* const entity = nodeMap.Find(fbxNode)->GetInfo();
	ndMeshEffect* const mesh = new ndMeshEffect();
	mesh->SetName(fbxMesh->name);

	dMatrix pivotMatrix(ofbxMatrix2dMatrix(fbxMesh->getGeometricMatrix()));
	entity->SetMeshMatrix(pivotMatrix);
	entity->m_fbxMeshEffect = mesh;

	const ofbx::Geometry* const geom = fbxMesh->getGeometry();

	
	const ofbx::Vec3* const vertices = geom->getVertices();
	dInt32 indexCount = geom->getIndexCount();
	dInt32* const indexArray = new dInt32 [indexCount];
	memcpy(indexArray, geom->getFaceIndices(), indexCount * sizeof(dInt32));

	dInt32 faceCount = 0;
	for (dInt32 i = 0; i < indexCount; i++)
	{
		if (indexArray[i] < 0)
		{
			faceCount++;
		}
	}

	dInt32* const faceIndexArray = new dInt32[faceCount];
	dInt32* const faceMaterialArray = new dInt32[faceCount];

	ImportMaterials(fbxMesh, mesh);

	dInt32 count = 0;
	dInt32 faceIndex = 0;
	const dArray<ndMeshEffect::dMaterial>& materialArray = mesh->GetMaterials();
	dInt32 materialId = (materialArray.GetCount() <= 1) ? 0 : -1;
	for (dInt32 i = 0; i < indexCount; i++)
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
				dInt32 fbxMatIndex = geom->getMaterials()[faceIndex];
				//fbxMatIndex = fbxMatIndex == 0 ? 0 : 1;
				faceMaterialArray[faceIndex] = fbxMatIndex;
			}
			count = 0;
			faceIndex++;
		}
	}

	ndMeshEffect::dMeshVertexFormat format;
	format.m_vertex.m_data = &vertices[0].x;
	format.m_vertex.m_indexList = indexArray;
	format.m_vertex.m_strideInBytes = sizeof(ofbx::Vec3);

	format.m_faceCount = faceCount;
	format.m_faceIndexCount = faceIndexArray;
	format.m_faceMaterial = faceMaterialArray;
	
	dArray<dVector> normalArray;
	if (geom->getNormals())
	{
		normalArray.Resize(indexCount);
		normalArray.SetCount(indexCount);
		const ofbx::Vec3* const normals = geom->getNormals();
		for (dInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec3 n = normals[i];
			normalArray[i] = dVector(dFloat32(n.x), dFloat32(n.y), dFloat32(n.z), dFloat32(0.0f));
		}

		format.m_normal.m_data = &normalArray[0].m_x;
		format.m_normal.m_indexList = indexArray;
		format.m_normal.m_strideInBytes = sizeof(dVector);
	}

	dArray<dVector> uvArray;
	if (geom->getUVs())
	{
		uvArray.Resize(indexCount);
		uvArray.SetCount(indexCount);
		const ofbx::Vec2* const uv = geom->getUVs();
		for (dInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec2 n = uv[i];
			uvArray[i] = dVector(dFloat32(n.x), dFloat32(n.y), dFloat32(0.0f), dFloat32(0.0f));
		}
		format.m_uv0.m_data = &uvArray[0].m_x;
		format.m_uv0.m_indexList = indexArray;
		format.m_uv0.m_strideInBytes = sizeof(dVector);
	}

	mesh->BuildFromIndexList(&format);
	//mesh->RepairTJoints();

	// import skin if there is any
	//dInt32 deformerCount = fbxMesh->GetDeformerCount(FbxDeformer::eSkin);
	if (geom->getSkin())
	{
		dAssert(0);
		#if 0
		FbxSkin* const skin = geom->getSkin();
		for (dInt32 i = 0; i < deformerCount; i++)
		{
			// count the number of weights
			dInt32 skinVertexDataCount = 0;
			dInt32 clusterCount = skin->GetClusterCount();
			for (dInt32 i = 0; i < clusterCount; i++) {
				FbxCluster* cluster = skin->GetCluster(i);
				skinVertexDataCount += cluster->GetControlPointIndicesCount();
			}
			dGeometryNodeSkinModifierInfo::dBoneVertexWeightData* const skinVertexData = new dGeometryNodeSkinModifierInfo::dBoneVertexWeightData[skinVertexDataCount];
				
			dInt32 actualSkinVertexCount = 0;
			for (dInt32 i = 0; i < clusterCount; i++)
			{
				FbxCluster* const cluster = skin->GetCluster(i);
				FbxNode* const fbxBone = cluster->GetLink(); // Get a reference to the bone's node
				
																//char xxx[256];
																//sprintf (xxx, "%s", fbxBone->GetName());
				GlobalNoceMap::dTreeNode* const boneNode = nodeMap.Find(fbxBone);
				if (boneNode) {
					dPluginScene::dTreeNode* const bone = boneNode->GetInfo();
				
					// Get the bind pose
					//FbxAMatrix bindPoseMatrix;
					//cluster->GetTransformLinkMatrix(bindPoseMatrix);
				
					dInt32 *boneVertexIndices = cluster->GetControlPointIndices();
					double *boneVertexWeights = cluster->GetControlPointWeights();
					// Iterate through all the vertices, which are affected by the bone
					dInt32 numBoneVertexIndices = cluster->GetControlPointIndicesCount();
					for (dInt32 j = 0; j < numBoneVertexIndices; j++) {
						dInt32 boneVertexIndex = boneVertexIndices[j];
						float boneWeight = (float)boneVertexWeights[j];
						skinVertexData[actualSkinVertexCount].m_vertexIndex = boneVertexIndex;
						skinVertexData[actualSkinVertexCount].m_weight = boneWeight;
						skinVertexData[actualSkinVertexCount].m_boneNode = bone;
						actualSkinVertexCount++;
						dAssert(actualSkinVertexCount <= skinVertexDataCount);
					}
				}
			}
				
			char skinName[256];
			sprintf(skinName, "%s_skin", fbxMeshNode->GetName());
			dPluginScene::dTreeNode* const skinNode = ngdScene->CreateSkinModifierNode(meshNode);
			dGeometryNodeSkinModifierInfo* const info = (dGeometryNodeSkinModifierInfo*)ngdScene->GetInfoFromNode(skinNode);
			info->SetName(skinName);
			info->SkinMesh(skinNode, ngdScene, skinVertexData, actualSkinVertexCount);
				
			delete[] skinVertexData;
		}
		#endif
	}

	delete[] faceMaterialArray;
	delete[] faceIndexArray;
	delete[] indexArray;
}

static fbxDemoEntity* FbxToEntity(ofbx::IScene* const fbxScene)
{
	fbxGlobalNoceMap nodeMap;
	fbxDemoEntity* const entity = LoadHierarchy(fbxScene, nodeMap);

	fbxGlobalNoceMap::Iterator iter(nodeMap);
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
		
			//case FbxNodeAttribute::eSkeleton:
			//{
			//	ImportSkeleton(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
			//	break;
			//}
			//
			//case FbxNodeAttribute::eLine:
			//{
			//	ImportLineShape(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
			//	break;
			//}
			//
			//case FbxNodeAttribute::eNurbsCurve:
			//{
			//	ImportNurbCurveShape(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials);
			//	break;
			//}

			//
			//case FbxNodeAttribute::eMarker:
			//case FbxNodeAttribute::eNurbs:
			//case FbxNodeAttribute::ePatch:
			//case FbxNodeAttribute::eCamera:
			//case FbxNodeAttribute::eCameraStereo:
			//case FbxNodeAttribute::eCameraSwitcher:
			//case FbxNodeAttribute::eLight:
			//case FbxNodeAttribute::eOpticalReference:
			//case FbxNodeAttribute::eOpticalMarker:
			//
			//case FbxNodeAttribute::eTrimNurbsSurface:
			//case FbxNodeAttribute::eBoundary:
			//case FbxNodeAttribute::eNurbsSurface:
			//case FbxNodeAttribute::eShape:
			//case FbxNodeAttribute::eLODGroup:
			//case FbxNodeAttribute::eSubDiv:
			//case FbxNodeAttribute::eCachedEffect:
			//case FbxNodeAttribute::eUnknown:
			default:
				dAssert(0);
				break;
		}
	}

	//UsedMaterials::Iterator iter1(usedMaterials);
	//for (iter1.Begin(); iter1; iter1++) {
	//	dInt32 count = iter1.GetNode()->GetInfo();
	//	if (!count) {
	//		dScene::dTreeNode* const materiaCacheNode = ngdScene->FindGetMaterialCacheNode();
	//		dScene::dTreeNode* const materialNode = iter1.GetKey();
	//		void* nextLink;
	//		for (void* link = ngdScene->GetFirstParentLink(materialNode); link; link = nextLink) {
	//			nextLink = ngdScene->GetNextParentLink(materialNode, link);
	//			dScene::dTreeNode* const parentNode = ngdScene->GetNodeFromLink(link);
	//			if (parentNode != materiaCacheNode) {
	//				ngdScene->RemoveReference(parentNode, materialNode);
	//			}
	//		}
	//	}
	//}
	return entity;
}

static void FreezeScale(fbxDemoEntity* const entity)
{
	dInt32 stack = 1;
	fbxDemoEntity* entBuffer[1024];
	dMatrix parentMatrix[1024];
	entBuffer[0] = entity;
	parentMatrix[0] = dGetIdentityMatrix();
	while (stack)
	{
		stack--;
		dMatrix scaleMatrix(parentMatrix[stack]);
		fbxDemoEntity* const ent = entBuffer[stack];

		dMatrix transformMatrix;
		dMatrix stretchAxis;
		dVector scale;
		dMatrix matrix(ent->GetRenderMatrix() * scaleMatrix);
		matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
		ent->SetRenderMatrix(transformMatrix);
		scaleMatrix = dMatrix(dGetIdentityMatrix(), scale, stretchAxis);

		if (ent->m_fbxMeshEffect)
		{
			matrix = ent->GetMeshMatrix() * scaleMatrix;
			matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
			ent->SetMeshMatrix(transformMatrix);
			dMatrix meshMatrix(dGetIdentityMatrix(), scale, stretchAxis);
			ent->m_fbxMeshEffect->ApplyTransform(meshMatrix);
		}

		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			parentMatrix[stack] = scaleMatrix;
			stack++;
		}
	}
}

fbxDemoEntity* LoadFbxMesh(const char* const meshName)
{
	char outPathName[1024];
	dGetWorkingFileName(meshName, outPathName);

	FILE* fp = fopen(outPathName, "rb");
	if (!fp)
	{
		dAssert(0);
		return nullptr;
	}

	fseek(fp, 0, SEEK_END);
	long file_size = ftell(fp);
	fseek(fp, 0, SEEK_SET);
	ofbx::u8* content = new ofbx::u8[file_size];
	fread(content, 1, file_size, fp);
	ofbx::IScene* const fbxScene = ofbx::load((ofbx::u8*)content, file_size, (ofbx::u64)ofbx::LoadFlags::TRIANGULATE);

	dMatrix convertMatrix(GetCoordinateSystemMatrix(fbxScene));
	fbxDemoEntity* const entity = FbxToEntity(fbxScene);
	FreezeScale(entity);
	entity->ApplyTransform(convertMatrix);

	fbxScene->destroy();
	delete[] content;
	return entity;
}

