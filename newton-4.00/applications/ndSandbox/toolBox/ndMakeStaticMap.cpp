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

class fbxImportStackData
{
	public:
	fbxImportStackData()
	{
	}

	//fbxImportStackData(const dMatrix& parentMatrix, const ofbx::Object* const fbxNode, ndDemoEntity* const parentNode)
	fbxImportStackData(const ofbx::Object* const fbxNode, ndDemoEntity* const parentNode)
		//:m_parentMatrix(parentMatrix)
		:m_fbxNode(fbxNode)
		,m_parentNode(parentNode)
	{
	}

	//dMatrix m_parentMatrix;
	const ofbx::Object* m_fbxNode;
	ndDemoEntity* m_parentNode;
};

static bool saveAsOBJ(ofbx::IScene& scene, const char* path)
{
	FILE* fp = fopen(path, "wb");
	if (!fp) return false;
	int obj_idx = 0;
	int indices_offset = 0;
	int normals_offset = 0;
	int mesh_count = scene.getMeshCount();
	for (int j = 0; j < mesh_count; ++j)
	{
		fprintf(fp, "o obj%d\ng grp%d\n", j, obj_idx);

		const ofbx::Mesh& mesh = *scene.getMesh(j);
		const ofbx::Geometry& geom = *mesh.getGeometry();
		int vertex_count = geom.getVertexCount();
		const ofbx::Vec3* vertices = geom.getVertices();
		for (int i = 0; i < vertex_count; ++i)
		{
			ofbx::Vec3 v = vertices[i];
			fprintf(fp, "v %f %f %f\n", v.x, v.y, v.z);
		}

		bool has_normals = geom.getNormals() != nullptr;
		if (has_normals)
		{
			const ofbx::Vec3* normals = geom.getNormals();
			int count = geom.getIndexCount();

			for (int i = 0; i < count; ++i)
			{
				ofbx::Vec3 n = normals[i];
				fprintf(fp, "vn %f %f %f\n", n.x, n.y, n.z);
			}
		}

		bool has_uvs = geom.getUVs() != nullptr;
		if (has_uvs)
		{
			const ofbx::Vec2* uvs = geom.getUVs();
			int count = geom.getIndexCount();

			for (int i = 0; i < count; ++i)
			{
				ofbx::Vec2 uv = uvs[i];
				fprintf(fp, "vt %f %f\n", uv.x, uv.y);
			}
		}

		const int* faceIndices = geom.getFaceIndices();
		int index_count = geom.getIndexCount();
		bool new_face = true;
		for (int i = 0; i < index_count; ++i)
		{
			if (new_face)
			{
				fputs("f ", fp);
				new_face = false;
			}
			int idx = (faceIndices[i] < 0) ? -faceIndices[i] : (faceIndices[i] + 1);
			int vertex_idx = indices_offset + idx;
			fprintf(fp, "%d", vertex_idx);

			if (has_uvs)
			{
				int uv_idx = normals_offset + i + 1;
				fprintf(fp, "/%d", uv_idx);
			}
			else
			{
				fprintf(fp, "/");
			}

			if (has_normals)
			{
				int normal_idx = normals_offset + i + 1;
				fprintf(fp, "/%d", normal_idx);
			}
			else
			{
				fprintf(fp, "/");
			}

			new_face = faceIndices[i] < 0;
			fputc(new_face ? '\n' : ' ', fp);
		}

		indices_offset += vertex_count;
		normals_offset += index_count;
		++obj_idx;
	}
	fclose(fp);
	return true;
}

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

static void showObjectsGUI(const ofbx::IScene& scene)
{
	const ofbx::Object* root = scene.getRoot();
	if (root)
	{
		showObjectGUI(*root);
	}

	int count = scene.getAnimationStackCount();
	for (int i = 0; i < count; ++i)
	{
		const ofbx::Object* stack = scene.getAnimationStack(i);
		showObjectGUI(*stack);
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
	//FbxAxisSystem axisSystem = settings.GetAxisSystem();
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
		upVector = dVector(1.0f * globalSettings->UpAxisSign, 0.0f, 0.0f, 0.0f);
		if (globalSettings->FrontAxis == ofbx::FrontVector_ParityEven)
		{
			dAssert(0);
			//frontVector = dVector(1.0f * sign, 0.0f, 0.0f, 0.0f);
		}
		else 
		{
			frontVector = dVector(0.0f, 0.0f, -1.0f * globalSettings->FrontAxisSign, 0.0f);
		}
	}
	
	dMatrix axisMatrix(dGetIdentityMatrix());
	axisMatrix.m_front = frontVector;
	axisMatrix.m_up = upVector;
	axisMatrix.m_right = frontVector.CrossProduct(upVector);
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
		node->SetMatrix(localMatrix);

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

//static void ImportMeshNode(FbxScene* const fbxScene, dPluginScene* const ngdScene, FbxNode* const fbxMeshNode, dPluginScene::dTreeNode* const node, GlobalMeshMap& meshCache, GlobalMaterialMap& materialCache, GlobalTextureMap& textureCache, UsedMaterials& usedMaterials, GlobalNoceMap& nodeMap)
static void ImportMeshNode(ofbx::Object* const fbxNode, fbxGlobalNoceMap& nodeMap)
{
	const ofbx::Mesh* const fbxMesh = (ofbx::Mesh*)fbxNode;

	dAssert(nodeMap.Find(fbxNode));
	fbxDemoEntity* const entity = nodeMap.Find(fbxNode)->GetInfo();
	dMeshEffect* const mesh = new dMeshEffect();
	mesh->SetName(fbxMesh->name);

	dMatrix pivotMatrix(ofbxMatrix2dMatrix(fbxMesh->getGeometricMatrix()));
	entity->SetMeshMatrix(pivotMatrix);
	entity->m_fbxMeshEffect = mesh;

	//LocalMaterialMap localMaterialIndex;
	//ImportMaterials(fbxScene, ngdScene, fbxMeshNode, meshNode, materialCache, localMaterialIndex, textureCache, usedMaterials);

	const ofbx::Geometry* const geom = fbxMesh->getGeometry();

	//int faceCount = fbxMesh->GetPolygonCount();
	//int indexCount = 0;
	//for (int i = 0; i < faceCount; i++) 
	//{
	//	indexCount += fbxMesh->GetPolygonSize(i);
	//}

	//dInt32* const faceIndexList = new dInt32[faceCount];
	//dInt32* const materialIndex = new dInt32[faceCount];
	//dInt32* const vertexIndex = new dInt32[indexCount];
	//dInt32* const normalIndex = new dInt32[indexCount];
	//dInt32* const uv0Index = new dInt32[indexCount];
	//dInt32* const uv1Index = new dInt32[indexCount];
	//dVector* const normalArray = new dVector[indexCount];
	//dVector* const uv0Array = new dVector[indexCount];
	//dVector* const uv1Array = new dVector[indexCount];
	//
	//const FbxVector4* const controlPoints = fbxMesh->GetControlPoints();
	//for (int i = 0; i < fbxMesh->GetControlPointsCount(); i++) {
	//	const FbxVector4& p = controlPoints[i];
	//	vertexArray[i] = dVector(dFloat(p[0]), dFloat(p[1]), dFloat(p[2]), 0.0f);
	//}

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

	dInt32 count = 0;
	dInt32 index = 0;
	dInt32* const faceIndexArray = new dInt32[faceCount];
	dInt32* const faceMaterialArray = new dInt32[faceCount];
	for (dInt32 i = 0; i < indexCount; i++)
	{
		count++;
		if (indexArray[i] < 0)
		{
			indexArray[i] = -indexArray[i] - 1;
			faceIndexArray[index] = count;
			faceMaterialArray[index] = 0;
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
	if (geom->getUVs())
	{
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

	//FbxGeometryElementUV* const uvArray = fbxMesh->GetElementUV();
	//FbxLayerElement::EMappingMode uvMapingMode = uvArray ? uvArray->GetMappingMode() : FbxGeometryElement::eNone;
	//FbxLayerElement::EReferenceMode uvRefMode = uvArray ? uvArray->GetReferenceMode() : FbxGeometryElement::eIndex;
	//
	//FbxGeometryElementMaterial* const materialArray = fbxMesh->GetElementMaterial();
	//FbxLayerElement::EMappingMode materialMapingMode = materialArray ? materialArray->GetMappingMode() : FbxGeometryElement::eNone;
	//FbxLayerElement::EReferenceMode materialRefMode = materialArray ? materialArray->GetReferenceMode() : FbxGeometryElement::eIndex;
	//
	//int index = 0;
	//for (int i = 0; i < faceCount; i++) {
	//	int polygonIndexCount = fbxMesh->GetPolygonSize(i);
	//
	//	int materialID = DEFUALT_MATERIAL_ID;
	//	if (materialArray) {
	//		if (materialMapingMode == FbxGeometryElement::eByPolygon) {
	//			materialID = (materialRefMode == FbxGeometryElement::eDirect) ? i : materialArray->GetIndexArray().GetAt(i);
	//		}
	//		else {
	//			materialID = (materialRefMode == FbxGeometryElement::eDirect) ? 0 : materialArray->GetIndexArray().GetAt(0);
	//		}
	//	}
	//	LocalMaterialMap::dTreeNode* const matNode = localMaterialIndex.Find(materialID);
	//	dAssert(matNode);
	//	dMaterialNodeInfo* const material = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(matNode->GetInfo());
	//	materialIndex[i] = material->GetId();
	//
	//	dAssert(usedMaterials.Find(matNode->GetInfo()));
	//	usedMaterials.Find(matNode->GetInfo())->GetInfo() += 1;
	//
	//	faceIndexList[i] = polygonIndexCount;
	//	for (int j = 0; j < polygonIndexCount; j++) {
	//		vertexIndex[index] = fbxMesh->GetPolygonVertex(i, j);
	//		FbxVector4 n(0, 1, 0, 0);
	//		fbxMesh->GetPolygonVertexNormal(i, j, n);
	//		normalArray[index] = dVector(dFloat(n[0]), dFloat(n[1]), dFloat(n[2]), 0.0f);
	//		normalIndex[index] = index;
	//
	//		FbxVector2 uv(0, 0);
	//		if (uvMapingMode == FbxGeometryElement::eByPolygonVertex) {
	//			int textIndex = (uvRefMode == FbxGeometryElement::eDirect) ? index : uvArray->GetIndexArray().GetAt(index);
	//			uv = uvArray->GetDirectArray().GetAt(textIndex);
	//		}
	//		uv0Index[index] = index;
	//		uv0Array[index] = dVector(dFloat(uv[0]), dFloat(uv[1]), 0.0f, 0.0f);
	//
	//		//uv1Index[index] = 0;
	//		//uv1Array[index] = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	//
	//		index++;
	//		dAssert(index <= indexCount);
	//	}
	//}

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

static fbxDemoEntity* FbxToEntity(ofbx::IScene* const fbxScene)
{
	fbxGlobalNoceMap nodeMap;
	//GlobalTextureMap textureCache;
	//GlobalMaterialMap materialCache;
	//UsedMaterials usedMaterials;
	//
	//dScene::dTreeNode* const defaulMaterialNode = ngdScene->CreateMaterialNode(DEFUALT_MATERIAL_ID);
	//dMaterialNodeInfo* const defaulMaterial = (dMaterialNodeInfo*)ngdScene->GetInfoFromNode(defaulMaterialNode);
	//defaulMaterial->SetName("default_material");
	//usedMaterials.Insert(0, defaulMaterialNode);
	//
	//m_materialId = 0;
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
				ImportMeshNode(fbxNode, nodeMap);
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

static void BakeScale(fbxDemoEntity* const entity)
{
	dInt32 stack = 1;
	fbxDemoEntity* entBuffer[1024];
	dMatrix parentMatrix[1024];
	entBuffer[0] = entity;
	parentMatrix[0] = dGetIdentityMatrix();
	while (stack)
	{
		stack--;
		dMatrix scaleMatrix(dGetIdentityMatrix());
		fbxDemoEntity* const ent = entBuffer[stack];

		if (ent->m_fbxMeshEffect)
		{
			dMatrix matrix(ent->GetRenderMatrix() * scaleMatrix);
			dMatrix transformMatrix;
			dMatrix stretchAxis;
			dVector scale;
			matrix.PolarDecomposition(transformMatrix, scale, stretchAxis);
			ent->SetMatrix(transformMatrix);
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

static void ApplyCordinade(fbxDemoEntity* const entity, const dMatrix& cordinateSystem)
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
			dMatrix matrix(invCordinateSystem * ent->GetRenderMatrix() * cordinateSystem);
			ent->SetMatrix(matrix);
			if (ent->m_fbxMeshEffect)
			{
				matrix = invCordinateSystem * ent->GetMeshMatrix() * cordinateSystem;
				ent->SetMeshMatrix(matrix);
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

//saveAsOBJ(*fbxScene, "xxx.txt");

	dMatrix convertMatrix(GetCodinateSystemMatrix(fbxScene));
	fbxDemoEntity* const entity = FbxToEntity(fbxScene);
	BakeScale(entity);
	ApplyCordinade(entity, convertMatrix);

	fbxScene->destroy();
	delete[] content;

	for (fbxDemoEntity* child = (fbxDemoEntity*)entity->GetFirst(); child; child = (fbxDemoEntity*)child->GetNext())
	{
		child->ResetMatrix(*scene, child->GetMeshMatrix());
		if (child->m_fbxMeshEffect)
		{
			ndDemoMesh* const mesh = new ndDemoMesh("fbxMesh", child->m_fbxMeshEffect, scene->GetShaderCache());
			child->SetMesh(mesh, child->GetMeshMatrix());
			mesh->Release();
		}
	}

	return entity;
}

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName)
{
	fbxDemoEntity* const entity_ = LoadFbxMesh(scene, meshName);
	scene->AddEntity(entity_);

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