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

#include "stdafx.h"
#include "Importer.h"


#define SCALE 100.0f

class BoneMap: public dTree<int, dScene::dTreeNode*>
{

};

Importer::Importer ()
{
}

Importer::~Importer ()
{
}

int Importer::GetType ()
{	
	return cMsPlugIn::eTypeImport;
}

const char* Importer::GetTitle ()
{
	return "Newton Alchemedia Importer...";
}


struct SortVertex
{
	float m_x;
	float m_y;
	float m_z;
	float m_u;
	float m_v;
	float weidhts[4];
	int	weidhtsIndex[4];
};

/*





void Importer::AddSkeleton (msModel* pModel, dModel& model)
{
	int boneCount = model.GetBoneCount();
	for (int i = 0; i < boneCount; i ++) {
		dBone* srcBone = model.FindBone(i);
		_ASSERTE (srcBone);

		int boneIndex = msModel_AddBone (pModel);
		msBone* pBone = msModel_GetBoneAt (pModel, boneIndex);

		msBone_SetName (pBone, srcBone->GetName());
		if (srcBone->GetParent()) {
			msBone_SetParentName (pBone, srcBone->GetParent()->GetName());
		}

		msVec3 position;
		msVec3 rotation;
		dMatrix matrix (srcBone->GetMatrix());

		position[0] = matrix.m_posit.m_x;
		position[1] = matrix.m_posit.m_y;
		position[2] = matrix.m_posit.m_z;

		dVector angle (matrix.GetXYZ_EulerAngles());
		rotation[0] = angle.m_x; 
		rotation[1] = angle.m_y; 
		rotation[2] = angle.m_z; 

		msBone_SetPosition (pBone, position);
		msBone_SetRotation (pBone, rotation);
	}
}
*/


/*
void Importer::EnumerateBones (dScene& scene, BoneMap& boneMap)
{
	int index = 0;
	int stack = 1;
	dScene::dTreeNode* pool[1024];

	pool[0] = scene.GetRoot();
	while (stack) {
		stack --;
		dScene::dTreeNode* node = pool[stack];
		boneMap.Insert (index, node);
		index ++;

		for (void* ptr = scene.GetFirstChild(node); ptr; ptr = scene.GetNextChild(node, ptr) ) {
			node = scene.GetNodeFromLink(ptr);
			dNodeInfo* info = scene.GetInfoFromNode(node);
			if (info->IsType(dSceneNodeInfo::GetRttiType())) {
				pool[stack] = node;
				stack ++;
			}
		}
	}
}


void Importer::MergeEqualMaterials(dScene& scene)
{
	dTree<dScene::dTreeNode*, int> materialFilter;

	dScene::Iterator iter (scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene.GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			for (void* ptr = scene.GetFirstChild(meshNode); ptr; ptr = scene.GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
				dNodeInfo* info = scene.GetInfoFromNode(node);
				if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {

					dMaterialNodeInfo* material = (dMaterialNodeInfo*) info;

					int crc = material->GetDiffuseTextId();
					crc = dCRC (&material->GetDiffuseColor(), sizeof (dVector), crc);
					crc = dCRC (&material->GetAmbientColor(), sizeof (dVector), crc);
#if 0
					if (indexCount > node->GetInfo().m_indexCount) {

						dMesh::dListNode* newNode;
						node->GetInfo().m_textureHandle = 1;

						newNode = skin->Addtop();
						dSubMesh& srcMesh = node->GetInfo();
						dSubMesh& subMesh = newNode->GetInfo();

						subMesh.m_ambient = srcMesh.m_ambient;
						subMesh.m_diffuse = srcMesh.m_diffuse;
						subMesh.m_specular = srcMesh.m_specular;
						subMesh.m_shiness = srcMesh.m_shiness;
						strcpy (subMesh.m_textureName, srcMesh.m_textureName);
						subMesh.AllocIndexData (indexCount);

						indexCount = 0;
						node = node->GetPrev();
						dMesh::dListNode* nextNode;
						for (dMesh::dListNode* node1 = node; node1; node1 = nextNode) {
							nextNode = node1->GetNext();
							if (node1->GetInfo().m_textureHandle == 1) {
								memcpy (&subMesh.m_indexes[indexCount], node1->GetInfo().m_indexes, sizeof (unsigned short) * node1->GetInfo().m_indexCount);
								indexCount += node1->GetInfo().m_indexCount;
								skin->Remove (node1);
							}
						}
					}
#endif
				}
			}
		}
	}
}


void Importer::MergeEqualMaterials(dScene& scene)
{
	dTree<dScene::dTreeNode*, int> materialFilter;

	dScene::Iterator iter (scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene.GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			for (void* ptr = scene.GetFirstChild(meshNode); ptr; ptr = scene.GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
				dNodeInfo* info = scene.GetInfoFromNode(node);
				if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {

					dMaterialNodeInfo* material = (dMaterialNodeInfo*) info;

					int crc = material->GetDiffuseTextId();
					crc = dCRC (&material->GetDiffuseColor(), sizeof (dVector), crc);
					crc = dCRC (&material->GetAmbientColor(), sizeof (dVector), crc);
#if 0
					if (indexCount > node->GetInfo().m_indexCount) {

						dMesh::dListNode* newNode;
						node->GetInfo().m_textureHandle = 1;

						newNode = skin->Addtop();
						dSubMesh& srcMesh = node->GetInfo();
						dSubMesh& subMesh = newNode->GetInfo();

						subMesh.m_ambient = srcMesh.m_ambient;
						subMesh.m_diffuse = srcMesh.m_diffuse;
						subMesh.m_specular = srcMesh.m_specular;
						subMesh.m_shiness = srcMesh.m_shiness;
						strcpy (subMesh.m_textureName, srcMesh.m_textureName);
						subMesh.AllocIndexData (indexCount);

						indexCount = 0;
						node = node->GetPrev();
						dMesh::dListNode* nextNode;
						for (dMesh::dListNode* node1 = node; node1; node1 = nextNode) {
							nextNode = node1->GetNext();
							if (node1->GetInfo().m_textureHandle == 1) {
								memcpy (&subMesh.m_indexes[indexCount], node1->GetInfo().m_indexes, sizeof (unsigned short) * node1->GetInfo().m_indexCount);
								indexCount += node1->GetInfo().m_indexCount;
								skin->Remove (node1);
							}
						}
					}
#endif
				}
			}
		}
	}
}
*/

void Importer::ConvertMeshToSkins(dScene& scene)
{
	dScene::Iterator iter (scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene.GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			dScene::dTreeNode* skinModifierNode = NULL;	
			for (void* ptr = scene.GetFirstChild(meshNode); ptr; ptr = scene.GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
				dNodeInfo* info = scene.GetInfoFromNode(node);
				if (info->GetTypeId() == dGeometryNodeSkinModifierInfo::GetRttiType()) {
					skinModifierNode = node;
					break;
				}
			}
			if (!skinModifierNode) {
				dMeshNodeInfo* meshInfo = (dMeshNodeInfo*) scene.GetInfoFromNode(meshNode);
				dScene::dTreeNode* skinNode = scene.CreateSkinModifierNode(meshNode);
				
				NewtonMesh* mesh = meshInfo->GetMesh();
				int stride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat);
				int vCount = NewtonMeshGetVertexCount(mesh);
//				const dFloat* const vertex = NewtonMeshGetVertexArray(mesh);
				dGeometryNodeSkinModifierInfo::dBoneVertexWeightData* skindata = new dGeometryNodeSkinModifierInfo::dBoneVertexWeightData[vCount];

				// find parent node, for now only one parent node because ms do no support intance
				dScene::dTreeNode* parentNode = NULL;
				for (void* ptr = scene.GetFirstParent(meshNode); ptr; ptr = scene.GetNextParent(meshNode, ptr)) {
					dScene::dTreeNode* pnode = scene.GetNodeFromLink(ptr);
					dNodeInfo* info = scene.GetInfoFromNode(pnode);
					if (info->GetTypeId() == dSceneNodeInfo::GetRttiType()) {
						parentNode = pnode;
						break;
					}
				}
			
//				dScene::dTreeNode* bone = boneMap.Find()
				for (int i = 0; i < vCount; i ++) {
					skindata[i].m_boneNode = parentNode;
					skindata[i].m_weight = 1.0f;
					skindata[i].m_vertexIndex = i;
				}

				dGeometryNodeSkinModifierInfo* skinInfo = (dGeometryNodeSkinModifierInfo*) scene.GetInfoFromNode(skinNode);
				skinInfo->SkinMesh(skinNode, &scene, skindata, vCount);
				delete[] skindata;
			}
		}
	}
}



int Importer::Execute (msModel *pModel)
{
	if (!pModel) {
		return -1;
	}

	// choose filename
	OPENFILENAME ofn;
	memset (&ofn, 0, sizeof (OPENFILENAME));

	char szFile[MS_MAX_PATH];
	char szFileTitle[MS_MAX_PATH];
	char szDefExt[32] = "xml";
	char szFilter[128] = "Newton Alchemedia  (*.xml)\0*.xml\0\0";
	szFile[0] = '\0';
	szFileTitle[0] = '\0';

	ofn.lStructSize = sizeof (OPENFILENAME);
	ofn.lpstrDefExt = szDefExt;
	ofn.lpstrFilter = szFilter;
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = MS_MAX_PATH;
	ofn.lpstrFileTitle = szFileTitle;
	ofn.nMaxFileTitle = MS_MAX_PATH;
	ofn.Flags = OFN_HIDEREADONLY | OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST;
	ofn.lpstrTitle = "Import Newton Alchemedia models";

	if (!::GetOpenFileName (&ofn))
		return 0;

	dMatrix scaleMatrix (GetIdentityMatrix());
	scaleMatrix[0][0] = SCALE;
	scaleMatrix[1][1] = SCALE;
	scaleMatrix[2][2] = SCALE;
	dMatrix globalRotation (scaleMatrix * dPitchMatrix(-3.14159265f * 0.5f));

	NewtonWorld* newton = NewtonCreate();
	dScene scene (newton);

	scene.Deserialize (szFile);

//	BoneMap boneMap;
//	EnumerateBones (scene, boneMap);
	ConvertMeshToSkins(scene);
//	MergeEqualMaterials(scene);


	// create all unique materials
	int mat_name_ID = 0;
	dTree<int, int> uniqueMaterials;

	dScene::Iterator iter (scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene.GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			dGeometryNodeInfo* meshInfo = (dGeometryNodeInfo*) info;

			for (void* ptr = scene.GetFirstChild(meshNode); ptr; ptr = scene.GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* materialNode = scene.GetNodeFromLink(ptr);
				dNodeInfo* info = scene.GetInfoFromNode(materialNode);
				if (info->GetTypeId() == dMaterialNodeInfo::GetRttiType()) {
					char name[256];
					sprintf (name, "%s_%d", meshInfo->GetName(), mat_name_ID);
					int CRC = dCRC (name);
					mat_name_ID ++;

					int matIndex = msModel_AddMaterial (pModel);
					uniqueMaterials.Insert (matIndex, CRC);
					msMaterial* mat = msModel_GetMaterialAt (pModel, matIndex);

					msMaterial_SetName (mat, name);

					dMaterialNodeInfo* materialInfo = (dMaterialNodeInfo*) info;

					float scale = 1.0f;
					msVec4 diffuse;
					diffuse[0] = materialInfo->GetDiffuseColor().m_x * scale;
					diffuse[1] = materialInfo->GetDiffuseColor().m_y * scale;
					diffuse[2] = materialInfo->GetDiffuseColor().m_z * scale;
					diffuse[3] = 0.0f;
					msMaterial_SetDiffuse (mat, diffuse);

					msVec4 ambient;
					ambient[0] = materialInfo->GetAmbientColor().m_x * scale;
					ambient[1] = materialInfo->GetAmbientColor().m_y * scale;
					ambient[2] = materialInfo->GetAmbientColor().m_z * scale;
					ambient[3] = 0.0f;
					msMaterial_SetAmbient (mat, ambient);

					msVec4 specular;
					specular[0] = materialInfo->GetSpecularColor().m_x * scale;
					specular[1] = materialInfo->GetSpecularColor().m_y * scale;
					specular[2] = materialInfo->GetSpecularColor().m_z * scale;
					specular[3] = 0.0f;
					msMaterial_SetSpecular (mat, specular);
					msMaterial_SetShininess (mat, materialInfo->GetShininess());
		//			msMaterial_SetShininess (mat, 60);

					if (materialInfo->GetDiffuseTextId() != -1) {
						dScene::dTreeNode* textureNode = scene.FindTextureByTextId(materialNode, materialInfo->GetDiffuseTextId());
						dTextureNodeInfo* texture = (dTextureNodeInfo*) scene.GetInfoFromNode(textureNode);
						msMaterial_SetDiffuseTexture (mat, texture->GetPathName());
					}
				}
			}
		}
	}


//	for (dList<dMeshInstance>::dListNode* node = model.m_meshList.GetFirst(); node; node = node->GetNext()) { 

	for (iter.Begin(); iter; iter ++) {
	dScene::dTreeNode* meshNode = iter.GetNode();
		dNodeInfo* info = scene.GetInfoFromNode(meshNode);
		if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
	//		dFloat vMoodule = 0.0f;
	//		dMesh* geo = node->GetInfo().m_mesh;
	//		dSkinModifier* skin = (dSkinModifier*) node->GetInfo().GetModifier();
	//		_ASSERTE (skin);
	//		_ASSERTE (geo);
	//		for (dMesh::dListNode* segNode = geo->GetFirst(); segNode; segNode = segNode->GetNext()) {

			dScene::dTreeNode* skinModifierNode = NULL;	
			for (void* ptr = scene.GetFirstChild(meshNode); ptr; ptr = scene.GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* node = scene.GetNodeFromLink(ptr);
				dNodeInfo* info = scene.GetInfoFromNode(node);
				if (info->GetTypeId() == dGeometryNodeSkinModifierInfo::GetRttiType()) {
					skinModifierNode = node;
					break;
				}
			}
			_ASSERTE (skinModifierNode);
			dGeometryNodeSkinModifierInfo* skinModifier = (dGeometryNodeSkinModifierInfo*) scene.GetInfoFromNode(skinModifierNode);

			_ASSERTE (info->GetTypeId() == dGeometryNodeInfo::GetRttiType());
			dMeshNodeInfo* meshInfo = (dMeshNodeInfo*) info;
			NewtonMesh* mesh = meshInfo->GetMesh();
			int stride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat);
			int vCount = NewtonMeshGetVertexCount(mesh);
			const dFloat* const vertex = NewtonMeshGetVertexArray(mesh);
			for (void* ptr = scene.GetFirstChild(meshNode); ptr; ptr = scene.GetNextChild(meshNode, ptr)) {
				dScene::dTreeNode* materialNode = scene.GetNodeFromLink(ptr);
				dMaterialNodeInfo* materialInfo = (dMaterialNodeInfo*) materialNode;
				int id = materialInfo->GetId();

//				dSubMesh& segment = segNode->GetInfo();
//				int *normalIndex = new int[segment.m_indexCount];
//				int *postIndex = new int[segment.m_indexCount];
//				SortVertex* sortPosit = new SortVertex[segment.m_indexCount];
//				dVector* normal = new dVector[segment.m_indexCount];
/*
				for (int i = 0; i < vCount; i ++) {
					int index = segment.m_indexes[i];

					memset (&sortPosit[i], 0, sizeof (SortVertex));
					sortPosit[i].m_x = geo->m_vertex[index * 3 + 0];
					sortPosit[i].m_y = geo->m_vertex[index * 3 + 1];
					sortPosit[i].m_z = geo->m_vertex[index * 3 + 2];
					sortPosit[i].m_u = geo->m_uv[index * 2 + 0];
					sortPosit[i].m_v = geo->m_uv[index * 2 + 1];

					normal[i].m_x = geo->m_normal[index * 3 + 0];
					normal[i].m_y = geo->m_normal[index * 3 + 1];
					normal[i].m_z = geo->m_normal[index * 3 + 2];

	//				sortPosit[i].weidhts[0] = 1.0f;
	//				sortPosit[i].weidhtsIndex[0] = geo->m_boneID;
	//				if (geo->m_weighList) {
					for (int j = 0; j < 3; j ++) {
						sortPosit[i].weidhts[j] = skin->m_vertexWeight[index][j];
						sortPosit[i].weidhtsIndex[j] = skin->m_skinnedBones[skin->m_boneWeightIndex[index].m_index[j]]->m_boneID;
					}
					vMoodule = (dAbs (sortPosit[i].m_v) > vMoodule) ? dAbs (sortPosit[i].m_v) : vMoodule;  
				}
*/

//				if (vMoodule == dFloor (vMoodule)) {
//					vMoodule = dFloor (vMoodule);
//				} else {
//					vMoodule = dFloor (vMoodule) + 1.0f;
//				}
//
//				int vCount;
//				int nCount;
//				vCount = dModel::dPackVertexArray (&sortPosit[0].m_x, 5, sizeof (SortVertex), segment.m_indexCount, postIndex);
//				nCount = dModel::dPackVertexArray (&normal[0].m_x, 3, sizeof (dVector), segment.m_indexCount, normalIndex);

//				dMatrix binMatrix (skinModifier);
//				binMatrix.TransformTriplex(&sortPosit[0].m_x, sizeof (SortVertex), &sortPosit[0].m_x, sizeof (SortVertex), vCount);
//				binMatrix.m_posit = dVector (0.0f, 0.0f, 0.0f, 1.0f);
//				binMatrix.TransformTriplex(&normal[0].m_x, sizeof (dVector), &normal[0].m_x, sizeof (dVector), nCount);

				msMesh* pMesh = msModel_GetMeshAt (pModel, msModel_AddMesh (pModel));
				msMesh_SetName (pMesh, meshInfo->GetName());

				char name[256];
				sprintf (name, "%s_%d", meshInfo->GetName(), mat_name_ID);
				mat_name_ID ++;

				dTree<int, int>::dTreeNode* matNode;
				matNode = uniqueMaterials.Find (dCRC (name));
				if (matNode) {
					msMesh_SetMaterialIndex (pMesh, matNode->GetInfo());
				}

				for (int i = 0; i < vCount; i ++) {
					//msVec2 uv;
					//msVertex vertex;

					int index = msMesh_AddVertex (pMesh);
					msVertex *pVertex = msMesh_GetVertexAt (pMesh, index);
/*
					vertex.Vertex[0] = sortPosit[i].m_x;
					vertex.Vertex[1] = sortPosit[i].m_y;
					vertex.Vertex[2] = sortPosit[i].m_z;
					msVertex_SetVertex (pVertex, vertex.Vertex);

					uv[0] = sortPosit[i].m_u;
					uv[1] = vMoodule - sortPosit[i].m_v;
					msVertex_SetTexCoords (pVertex, uv);

					int boneIndex0 = sortPosit[i].weidhtsIndex[0];
					int boneIndex1 = sortPosit[i].weidhtsIndex[1];
					int boneIndex2 = sortPosit[i].weidhtsIndex[2];
					int boneIndex3 = sortPosit[i].weidhtsIndex[3];

					int weight0 = int (sortPosit[i].weidhts[0] * 100);
					int weight1 = int (sortPosit[i].weidhts[1] * 100);
					int weight2 = int (sortPosit[i].weidhts[2] * 100);
					int weight3 = int (sortPosit[i].weidhts[3] * 100);

					msVertex_SetBoneIndex (pVertex, boneIndex0);


	#if 0
					// note in milk shape msMesh_GetVertexExAt does really works so only 100 weigh is possible
					// at leas I do no know who to read and write that informations
					if ((weight0 > 0) && (weight0 <= 100)){
						msVertexEx* extraVertex = msMesh_GetVertexExAt (pMesh, index);
						msVertexEx_SetBoneWeights (extraVertex, 0, weight0);

						msVertexEx_SetBoneIndices (extraVertex, 0, boneIndex1);
						msVertexEx* w = msMesh_GetVertexExAt (pMesh, index);
						w = msMesh_GetVertexExAt (pMesh, index);

						if (weight1 > 0) {
							msVertexEx_SetBoneWeights (extraVertex, 1, weight1);
							msVertexEx_SetBoneIndices (extraVertex, 1, boneIndex2);
							if (weight2 > 0) {
								msVertexEx_SetBoneWeights (extraVertex, 2, weight2);
								msVertexEx_SetBoneIndices (extraVertex, 2, boneIndex3);
							}
						}
					}
	#endif
*/
				}

/*
				for (int i = 0; i < nCount; i ++) {
					msVec3 vertex;
					vertex[0] = normal[i].m_x;
					vertex[1] = normal[i].m_y;
					vertex[2] = normal[i].m_z;
					int index = msMesh_AddVertexNormal (pMesh);
					msMesh_SetVertexNormalAt (pMesh, index, vertex);
				}

				for (int i = 0; i < segment.m_indexCount; i += 3) {
					word nIndices[3];
					msTriangle *pTriangle;
					pTriangle = msMesh_GetTriangleAt (pMesh, msMesh_AddTriangle (pMesh));

					nIndices[0] = word (postIndex[i + 0]);
					nIndices[1] = word (postIndex[i + 1]);
					nIndices[2] = word (postIndex[i + 2]);
					msTriangle_SetVertexIndices (pTriangle, nIndices);

					// invert the winding for mikshape
					nIndices[0] = word (normalIndex[i + 0]);
					nIndices[1] = word (normalIndex[i + 1]);
					nIndices[2] = word (normalIndex[i + 2]);
					msTriangle_SetNormalIndices (pTriangle, nIndices);
					msTriangle_SetSmoothingGroup (pTriangle, 1);
				}

				delete[] normal;
				delete[] sortPosit;
				delete[] postIndex;
				delete[] normalIndex;
			}
*/
			}
		}
	}

//	AddSkeleton (pModel, model);
	return 0;
}
