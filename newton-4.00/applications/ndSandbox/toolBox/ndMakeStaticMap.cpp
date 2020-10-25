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
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"

void fbxDemoEntity::CleanIntermidiate()
{
	if (m_fbxMeshEffect)
	{
		delete m_fbxMeshEffect;
		m_fbxMeshEffect = nullptr;
	}

	for (fbxDemoEntity* child = (fbxDemoEntity*)GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
	{
		child->CleanIntermidiate();
	}
}

class fbxGlobalNoceMap : public dTree<fbxDemoEntity*, const ofbx::Object*>
{
};

class fbxDemoSubMeshMaterial : public ndDemoSubMeshMaterial
{
	public: 
	~fbxDemoSubMeshMaterial()
	{
		m_textureHandle = 0;
	}

	dInt32 m_index;
};

class fbxGlobalMaterialMap : public dTree<fbxDemoSubMeshMaterial, const ofbx::Material*>
{
	public:
	fbxGlobalMaterialMap()
	{
		m_index = 0;
	}

	dInt32 AddMaterial(const ofbx::Material* const fbxMaterial)
	{
		dTreeNode* node = Find(fbxMaterial);
		if (!node)
		{
			node = Insert(fbxDemoSubMeshMaterial(), fbxMaterial);
			fbxDemoSubMeshMaterial& meshMaterial = node->GetInfo();

			if (fbxMaterial)
			{
				const ofbx::Texture* const texture = fbxMaterial->getTexture(ofbx::Texture::DIFFUSE);
				if (texture)
				{
					char textName[1024];
					ofbx::DataView dataView = texture->getRelativeFileName();
					dataView.toString(textName);
					strncpy (meshMaterial.m_textureName, textName, sizeof (meshMaterial.m_textureName));
					meshMaterial.m_textureHandle = LoadTexture(textName);
				}
			}

			meshMaterial.m_index = m_index;
			m_index++;
		}
		return node->GetInfo().m_index;
	}

	dInt32 m_index;
};

class fbxImportStackData
{
	public:
	fbxImportStackData()
	{
	}

	//fbxImportStackData(const dMatrix& parentMatrix, const ofbx::Object* const fbxNode, ndDemoEntity* const parentNode)
	fbxImportStackData(const ofbx::Object* const fbxNode, ndDemoEntity* const parentNode)
		:m_fbxNode(fbxNode)
		,m_parentNode(parentNode)
	{
	}

	const ofbx::Object* m_fbxNode;
	ndDemoEntity* m_parentNode;
};

static void showObjectGUI(const ofbx::Object& object)
{
	const char* label;
	switch (object.getType())
	{
		case ofbx::Object::Type::GEOMETRY: label = "geometry"; break;
		case ofbx::Object::Type::MESH: label = "mesh"; break;
		case ofbx::Object::Type::MATERIAL: label = "material"; break;
		case ofbx::Object::Type::ROOT: label = "root"; break;
		case ofbx::Object::Type::TEXTURE: label = "texture"; break;
		case ofbx::Object::Type::NULL_NODE: label = "null"; break;
		case ofbx::Object::Type::LIMB_NODE: label = "limb node"; break;
		case ofbx::Object::Type::NODE_ATTRIBUTE: label = "node attribute"; break;
		case ofbx::Object::Type::CLUSTER: label = "cluster"; break;
		case ofbx::Object::Type::SKIN: label = "skin"; break;
		case ofbx::Object::Type::ANIMATION_STACK: label = "animation stack"; break;
		case ofbx::Object::Type::ANIMATION_LAYER: label = "animation layer"; break;
		case ofbx::Object::Type::ANIMATION_CURVE: label = "animation curve"; break;
		case ofbx::Object::Type::ANIMATION_CURVE_NODE: label = "animation curve node"; break;
		default: assert(false); break;
	}

	//ImGuiTreeNodeFlags flags = g_selected_object == &object ? ImGuiTreeNodeFlags_Selected : 0;
	char tmp[128];
	sprintf_s(tmp, "%lld %s (%s)", object.id, object.name, label);
	int i = 0;
	while (ofbx::Object* child = object.resolveObjectLink(i))
	{
		showObjectGUI(*child);
		++i;
	}
	if (object.getType() == ofbx::Object::Type::ANIMATION_CURVE) 
	{
		dAssert(0);
		//showCurveGUI(object);
	}
}

static dMatrix GetCodinateSystemMatrix(ofbx::IScene* const fbxScene)
{
	const ofbx::GlobalSettings* const globalSettings = fbxScene->getGlobalSettings();

	dMatrix convertMatrix(dGetIdentityMatrix());

	dFloat32 scaleFactor = globalSettings->UnitScaleFactor;
	convertMatrix[0][0] = dFloat32(scaleFactor / 100.0f);
	convertMatrix[1][1] = dFloat32(scaleFactor / 100.0f);
	convertMatrix[2][2] = dFloat32(scaleFactor / 100.0f);
	
	//int sign;
	dVector upVector(0.0f, 1.0f, 0.0f, 0.0f);
	dVector frontVector(1.0f, 0.0f, 0.0f, 0.0f);
	if (globalSettings->UpAxis == ofbx::UpVector_AxisX) 
	{
		dAssert(0);
	}
	else if (globalSettings->UpAxis == ofbx::UpVector_AxisY) 
	{
		dAssert(0);
	//	upVector = dVector(0.0f, 1.0f * sign, 0.0f, 0.0f);
	//	if (axisSystem.GetFrontVector(sign) == FbxAxisSystem::eParityEven) {
	//		frontVector = dVector(1.0f * sign, 0.0f, 0.0f, 0.0f);
	//	}
	//	else {
	//		frontVector = dVector(0.0f, 0.0f, 1.0f * sign, 0.0f);
	//	}
	}
	else 
	{
		upVector = dVector(dFloat32 (globalSettings->UpAxisSign), 0.0f, 0.0f, 0.0f);
		if (globalSettings->FrontAxis == ofbx::FrontVector_ParityEven)
		{
			dAssert(0);
			//frontVector = dVector(1.0f * sign, 0.0f, 0.0f, 0.0f);
		}
		else 
		{
			frontVector = dVector(0.0f, 0.0f, -dFloat32 (globalSettings->FrontAxisSign), 0.0f);
		}
	}
	
	dMatrix axisMatrix(dGetIdentityMatrix());
	axisMatrix.m_front = frontVector;
	axisMatrix.m_up = upVector;
	axisMatrix.m_right = frontVector.CrossProduct(upVector);
	axisMatrix = axisMatrix * dYawMatrix(dPi);
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
		for (int i = 0; i < count; i++) 
		{
			ofbx::Object* const child = buffer[count - i - 1];
			nodeStack[stack] = fbxImportStackData(child, node);
			stack++;
			dAssert(stack < sizeof(nodeStack) / sizeof(nodeStack[0]));
		}
	}
	return entity;
}

static dInt32 ImportMaterials(const ofbx::Mesh* const fbxMesh, fbxGlobalMaterialMap& materialCache)
{
	dInt32 materialId = -1;
	dInt32 materialCount = fbxMesh->getMaterialCount();
	if (materialCount == 0)
	{
		materialId = materialCache.AddMaterial(nullptr);
	}
	else if (materialCount == 1)
	{
		materialId = materialCache.AddMaterial(fbxMesh->getMaterial(0));
	}
	else
	{
		for (dInt32 i = 0; i < materialCount; i++)
		{
			materialCache.AddMaterial(fbxMesh->getMaterial(i));
		}
	}
	return materialId;
}

//static void ImportMeshNode(FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, GlobalNoceMap& nodeMap)
static void ImportMeshNode(ofbx::Object* const fbxNode, fbxGlobalNoceMap& nodeMap, fbxGlobalMaterialMap& materialCache)
{
	const ofbx::Mesh* const fbxMesh = (ofbx::Mesh*)fbxNode;

	dAssert(nodeMap.Find(fbxNode));
	fbxDemoEntity* const entity = nodeMap.Find(fbxNode)->GetInfo();
	dMeshEffect* const mesh = new dMeshEffect();
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

	dInt32 materialId = ImportMaterials(fbxMesh, materialCache);

	dInt32 count = 0;
	dInt32 index = 0;
	for (dInt32 i = 0; i < indexCount; i++)
	{
		count++;
		if (indexArray[i] < 0)
		{
			indexArray[i] = -indexArray[i] - 1;
			faceIndexArray[index] = count;
			if (materialId != -1)
			{
				faceMaterialArray[index] = materialId;
			}
			else
			{
				//dAssert (count < geom->getM)
				dInt32 fbxMatIndex = geom->getMaterials()[count];
				const ofbx::Material* const fbxMaterial = fbxMesh->getMaterial(fbxMatIndex);
				dAssert (materialCache.Find(fbxMaterial));
				fbxDemoSubMeshMaterial& material = materialCache.Find(fbxMaterial)->GetInfo();
				faceMaterialArray[index] = material.m_index;
			}
			count = 0;
			index++;
		}
	}

	dMeshEffect::dMeshVertexFormat format;

	format.m_vertex.m_data = &vertices[0].x;
	format.m_vertex.m_indexList = indexArray;
	format.m_vertex.m_strideInBytes = sizeof(ofbx::Vec3);

	format.m_faceCount = faceCount;
	format.m_faceIndexCount = faceIndexArray;
	format.m_faceMaterial = faceMaterialArray;
	
	dArray<dVector> normalArray;
	if (geom->getNormals())
	{
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
	uvArray.SetCount(indexCount);
	memset(&uvArray[0].m_x, 0, indexCount * sizeof(dVector));
	if (geom->getUVs())
	{
		const ofbx::Vec2* const uv = geom->getUVs();
		for (dInt32 i = 0; i < indexCount; ++i)
		{
			ofbx::Vec2 n = uv[i];
			uvArray[i] = dVector(dFloat32(n.x), dFloat32(n.y), dFloat32(0.0f), dFloat32(0.0f));
		}
	}
	format.m_uv0.m_data = &uvArray[0].m_x;
	format.m_uv0.m_indexList = indexArray;
	format.m_uv0.m_strideInBytes = sizeof(dVector);

	mesh->BuildFromIndexList(&format);
	mesh->RepairTJoints();

	// import skin if there is any
	//int deformerCount = fbxMesh->GetDeformerCount(FbxDeformer::eSkin);
	if (geom->getSkin())
	{
		dAssert(0);
		#if 0
		FbxSkin* const skin = geom->getSkin();
		for (int i = 0; i < deformerCount; i++)
		{
			// count the number of weights
			int skinVertexDataCount = 0;
			int clusterCount = skin->GetClusterCount();
			for (int i = 0; i < clusterCount; i++) {
				FbxCluster* cluster = skin->GetCluster(i);
				skinVertexDataCount += cluster->GetControlPointIndicesCount();
			}
			dGeometryNodeSkinModifierInfo::dBoneVertexWeightData* const skinVertexData = new dGeometryNodeSkinModifierInfo::dBoneVertexWeightData[skinVertexDataCount];
				
			int actualSkinVertexCount = 0;
			for (int i = 0; i < clusterCount; i++)
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
				
					int *boneVertexIndices = cluster->GetControlPointIndices();
					double *boneVertexWeights = cluster->GetControlPointWeights();
					// Iterate through all the vertices, which are affected by the bone
					int numBoneVertexIndices = cluster->GetControlPointIndicesCount();
					for (int j = 0; j < numBoneVertexIndices; j++) {
						int boneVertexIndex = boneVertexIndices[j];
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

static fbxDemoEntity* FbxToEntity(ofbx::IScene* const fbxScene, fbxGlobalMaterialMap& materialCache)
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
				//ImportMeshNode(fbxScene, ngdScene, fbxNode, node, meshCache, materialCache, textureCache, usedMaterials, nodeMap);
				ImportMeshNode(fbxNode, nodeMap, materialCache);
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
			//case FbxNodeAttribute::eNull:
			//{
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
	//	int count = iter1.GetNode()->GetInfo();
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

		if (ent->m_fbxMeshEffect)
		{
			dMatrix matrix(ent->GetRenderMatrix() * scaleMatrix);
			dMatrix transformMatrix;
			dMatrix stretchAxis;
			dVector scale;
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
		}

		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			parentMatrix[stack] = scaleMatrix;
			stack++;
		}
	}
}

static void ApplyTransform(fbxDemoEntity* const entity, const dMatrix& cordinateSystem)
{
	dInt32 stack = 1;
	fbxDemoEntity* entBuffer[1024];
	entBuffer[0] = entity;
	dMatrix invCordinateSystem(cordinateSystem.Inverse4x4());
	while (stack)
	{
		stack--;
		fbxDemoEntity* const ent = entBuffer[stack];

		if (ent->m_fbxMeshEffect)
		{
			dMatrix entMatrix(invCordinateSystem * ent->GetRenderMatrix() * cordinateSystem);
			ent->SetRenderMatrix(entMatrix);
			if (ent->m_fbxMeshEffect)
			{
				dMatrix meshMatrix (invCordinateSystem * ent->GetMeshMatrix() * cordinateSystem);
				ent->SetMeshMatrix(meshMatrix);
				ent->m_fbxMeshEffect->ApplyTransform(cordinateSystem);
			}
		}

		for (fbxDemoEntity* child = (fbxDemoEntity*)ent->GetChild(); child; child = (fbxDemoEntity*)child->GetSibling())
		{
			entBuffer[stack] = child;
			stack++;
		}
	}
}

fbxDemoEntity* LoadFbxMesh(ndDemoEntityManager* const scene, const char* const meshName)
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

	fbxGlobalMaterialMap materialCache;

	dMatrix convertMatrix(GetCodinateSystemMatrix(fbxScene));
	fbxDemoEntity* const entity = FbxToEntity(fbxScene, materialCache);
	FreezeScale(entity);
	ApplyTransform(entity, convertMatrix);

	fbxScene->destroy();
	delete[] content;

	ndDemoSubMeshMaterial* const materials = dAlloca(ndDemoSubMeshMaterial, materialCache.GetCount());
	memset(materials, 0, materialCache.GetCount() * sizeof(ndDemoSubMeshMaterial));
	while (materialCache.GetRoot())
	{
		fbxDemoSubMeshMaterial material(materialCache.GetRoot()->GetInfo());
		materials[material.m_index] = material;
		materialCache.Remove(materialCache.GetRoot());
	}

	for (fbxDemoEntity* child = (fbxDemoEntity*)entity->GetFirst(); child; child = (fbxDemoEntity*)child->GetNext())
	{
		child->ResetMatrix(*scene, child->GetRenderMatrix());
		if (child->m_fbxMeshEffect)
		{
			if (child->GetName().Find("hidden"))
			{
				ndDemoMesh* const mesh = new ndDemoMesh("fbxMesh", child->m_fbxMeshEffect, scene->GetShaderCache(), materials);
				child->SetMesh(mesh, child->GetMeshMatrix());
				mesh->Release();
			}
		}
	}

	return entity;
}

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName)
{
	fbxDemoEntity* const entity = LoadFbxMesh(scene, meshName);
	scene->AddEntity(entity);

	//ndPhysicsWorld* const world = scene->GetWorld();
	//dVector floor[] =
	//{
	//	{ 100.0f, 0.0f,  100.0f, 1.0f },
	//	{ 100.0f, 0.0f, -100.0f, 1.0f },
	//	{ -100.0f, 0.0f, -100.0f, 1.0f },
	//	{ -100.0f, 0.0f,  100.0f, 1.0f },
	//};
	//dInt32 index[][3] = { { 0, 1, 2 },{ 0, 2, 3 } };
	//
	//dPolygonSoupBuilder meshBuilder;
	//meshBuilder.Begin();
	//meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[0][0], 3);
	//meshBuilder.AddFaceIndirect(&floor[0].m_x, sizeof(dVector), 31, &index[1][0], 3);
	//meshBuilder.End(true);
	//
	//ndShapeInstance box(new ndShapeStaticBVH(meshBuilder));
	//dMatrix uvMatrix(dGetIdentityMatrix());
	//uvMatrix[0][0] *= 0.025f;
	//uvMatrix[1][1] *= 0.025f;
	//uvMatrix[2][2] *= 0.025f;
	//ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);
	//
	//dMatrix matrix(dGetIdentityMatrix());
	//matrix.m_posit.m_y = -0.5f;
	//ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	//entity->SetMesh(geometry, dGetIdentityMatrix());
	//
	//ndBodyDynamic* const body = new ndBodyDynamic();
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	//body->SetMatrix(matrix);
	//body->SetCollisionShape(box);
	//
	//world->AddBody(body);
	//
	//scene->AddEntity(entity);
	//geometry->Release();

	return nullptr;
}